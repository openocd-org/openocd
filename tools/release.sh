#!/bin/sh -e
# release.sh: openocd release process automation
# Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>
# Release under the GNU GPL v2 (or later versions).

## set these to control the build process
#CONFIG_OPTS=""
#MAKE_OPTS=""

## DO NOT PERFORM LIVE RELEASES UNLESS YOU ARE THE RELEASE MANAGER!!!
RELEASE_DRY_RUN=1
## set this to perform individual steps on past releases
RELEASE_VERSION=

die() {
	echo "$@" >&2
	exit 1
}

svn_info_get() {
	svn info | grep "$1" | cut -d':' -f2- | cut -c2-
}

svn_setup_load() {
	SVN_ROOT="$(svn_info_get 'Repository Root')"
	SVN_URL="$(svn_info_get 'URL')"

	SVN_TRUNK="${SVN_ROOT}/trunk"

	SVN_BRANCHES="${SVN_ROOT}/branches"
	PACKAGE_BRANCH="${SVN_BRANCHES}/${PACKAGE_RELEASE}"

	SVN_TAGS="${SVN_ROOT}/tags"
	PACKAGE_TAG="${SVN_TAGS}/${PACKAGE_RELEASE}"

	if [ "${SVN_URL}" = "${SVN_TRUNK}" ]; then
		RELEASE_TYPE=minor
	elif [ "${SVN_URL/${SVN_BRANCHES}/}" != "${SVN_URL}" ]; then
		RELEASE_TYPE=micro
	else
		echo "error: bad URL: ${SVN_URL}" >&2
		die "unable to branch from the current location"
	fi
}
svn_setup_show() {
	cat <<INFO
Release Type: ${RELEASE_TYPE}
  Branch URL: ${PACKAGE_BRANCH}
     Tag URL: ${PACKAGE_TAG}
INFO
}

do_svn_echo_msg() { echo "svn: $1: $3"; }
do_svn_echo() {
	case "$1" in
	commit)
		do_svn_echo_msg "$@"
		shift 3
		[ "$*" ] && echo "Files: $@"
		;;
	copy|move)
		do_svn_echo_msg "$@"
		echo "From: ${4:-$2}"
		echo "  To: ${5:-$3}"
		;;
	*)
		local ACTION="$1"
		shift
		echo "svn: ${ACTION}: $@"
		;;
	esac
}
do_svn() {
	do_svn_echo "$@"
	[ "${RELEASE_DRY_RUN}" ] || svn "$@"
}


package_info_load_name() {
	grep AC_INIT configure.in | perl -ne 's/^.+\(\[([-\w]*)\],.+$/$1/ and print'
}
package_info_load_version() {
	grep AC_INIT configure.in | perl -ne 's/^.+\[([-\w\.]*)\],$/$1/ and print'
}

package_info_load() {
	[ -f "configure.in" ] || \
		die "package_info_load: configure.in is missing"

	PACKAGE_NAME="$(package_info_load_name)"
	# todo: fix this
	PACKAGE_TARNAME="${PACKAGE_NAME}"

	PACKAGE_VERSION="$(package_info_load_version)"
	[ "${RELEASE_VERSION}" ] || \
		RELEASE_VERSION=${PACKAGE_VERSION/-in-development/}

	[ "${PACKAGE_NAME}" -a "${PACKAGE_VERSION}" ] || \
		die "package information is missing from configure script"

	PACKAGE_VERSION_TAGS=
	[ "${PACKAGE_VERSION/-/}" = "${PACKAGE_VERSION}" ] || \
		PACKAGE_VERSION_TAGS="-${PACKAGE_VERSION#*-}"
	PACKAGE_VERSION_BASE="${PACKAGE_VERSION%%-*}"
	PACKAGE_MICRO="${PACKAGE_VERSION_BASE##*.}"
	PACKAGE_MAJOR_AND_MINOR="${PACKAGE_VERSION_BASE%.*}"
	PACKAGE_MAJOR="${PACKAGE_MAJOR_AND_MINOR%.*}"
	PACKAGE_MINOR="${PACKAGE_MAJOR_AND_MINOR#*.}"

	PACKAGE_STRING="${PACKAGE_NAME} ${PACKAGE_VERSION}"
	if [ "${RELEASE_DRY_RUN}" ]; then
		PACKAGE_RELEASE="${PACKAGE_TARNAME}-${PACKAGE_VERSION}"
	else
		PACKAGE_RELEASE="${PACKAGE_TARNAME}-${RELEASE_VERSION}"
	fi
}

package_info_show() {
	cat <<INFO
Name: ${PACKAGE_TARNAME}
Release: ${RELEASE_VERSION}
Version: ${PACKAGE_VERSION}
   Number: ${PACKAGE_VERSION_BASE}
   Series: ${PACKAGE_MAJOR_AND_MINOR}
    Major: ${PACKAGE_MAJOR}
    Minor: ${PACKAGE_MINOR}
    Micro: ${PACKAGE_MICRO}
     Tags: ${PACKAGE_VERSION_TAGS}
 Branch: ${PACKAGE_RELEASE}
Release: ${PACKAGE_TARNAME}-${PACKAGE_VERSION_BASE}${PACKAGE_VERSION_TAGS}
INFO
}

usage() {
	cat << USAGE
usage: $0 <command>

Main Commands:
  info          Show a summary of the next pending release.
  release       Release the current tree as an archive.
  upload        Upload archives to berliOS project site

Build Commands:
  bootstrap     Prepare the working copy for configuration and building.
  configure     Configures the package; runs bootstrap, if needed.
  build         Compiles the project; runs configure, if needed.

Packaging Commands:
  changelog     Generate a new ChangeLog using svn2cl.
  package       Produce new distributable source archives.
  stage         Move archives to staging area for upload.

Repository Commands:
  commit        Perform branch and tag, as appropriate for the version.
  branch        Create a release branch from the project trunk.
  tag           Create a tag for the current release branch.

Other Commands:
  version ...   Perform version number and tag manipulations.
  maryslamb     Mary had a little lamb, but no one noticed.
  clean         Forces regeneration of results.
  clean_all     Removes all traces of the release process.
  help          Provides this list of commands.
  
For more information about this script, see the Release Processes page
in the OpenOCD Developer's Manual (doc/manual/release.txt).

WARNING: This script should be used by the Release Manager ONLY.
USAGE
	exit 0
}
do_usage() { usage; }
do_help()  { usage; }

do_info_show() {
	echo "Current Release Analysis:"
	package_info_show
	svn_setup_show
}

do_info() {
	package_info_load
	svn_setup_load
	do_info_show
}

do_bootstrap() {
	echo -n "Bootstrapping..."
	./bootstrap 2>&1 | perl tools/logger.pl > "release-bootstrap.log"
}
maybe_bootstrap() { [ -f "configure" ] || do_bootstrap; }

do_configure() {
	maybe_bootstrap
	echo -n "Configuring..."
	./configure ${CONFIG_OPTS} 2>&1 | perl tools/logger.pl > "release-config.log"
}
maybe_configure() { [ -f "Makefile" ] || do_configure; }

do_build() {
	maybe_configure
	echo -n "Compiling OpenOCD ${PACKAGE_VERSION}"
	make ${MAKE_OPTS} -C doc stamp-vti 2>&1 \
		| perl tools/logger.pl > "release-version.log"
	make ${MAKE_OPTS} 2>&1 \
		| perl tools/logger.pl > "release-make.log"
}
maybe_build() { [ -f "src/openocd" ] || do_build; }
do_build_clean() { [ -f Makefile ] && make maintainer-clean >/dev/null; }

maybe_rebuild() {
	if [ -f "configure" ]; then
		echo "Re-running autoconf..."
		autoconf
		echo "Re-running automake..."
		automake
	fi
	if [ -f "Makefile" ]; then
		do_configure
		do_build
	fi
}

do_changelog() {
	echo "Updating working copy to HEAD..."
	do_svn update
	echo "Creating ChangeLog..."
	svn2cl -i --authors AUTHORS.ChangeLog
}
maybe_changelog() {
	if [ -z "${RELEASE_DRY_RUN}" ] \
		|| [ ! -f ChangeLog ] \
		|| [ "$(cat ChangeLog | wc -l)" -lt 2 ]
	then
		do_changelog
	fi
}
do_changelog_clean() {
	do_svn revert ChangeLog
}

do_package() {
	package_info_load
	maybe_changelog
	maybe_build
	echo "Building distribution packages..."
	make ${MAKE_OPTS} distcheck 2>&1 | perl tools/logger.pl > "release-pkg.log"
}
maybe_package() { [ -f "${PACKAGE_RELEASE}.zip" ] || do_package; }
do_package_clean() {
	for EXT in tar.gz tar.bz2 zip; do
		rm -v -f *.${EXT}
	done
}

do_stage() {
	maybe_package
	echo "Staging package archives:"
	mkdir -p archives
	for EXT in tar.gz tar.bz2 zip; do
		mv -v "${PACKAGE_RELEASE}.${EXT}" archives/
	done
	cp -a NEWS archives/
	cp -a ChangeLog archives/
}
do_stage_clean() { rm -v -f -r archives; }

do_clean() {
	do_build_clean
	do_package_clean
	rm -v -f configure

	svn revert configure.in
	rm -v -f release-*.log
}
do_clean_all() {
	do_clean
	do_changelog_clean
	do_stage_clean
}

do_version_usage() {
	cat << USAGE
usage: $0 version <command>
Version Commands:
  tag {add|remove} <label>     Add or remove the specified tag.
  bump {major|minor|micro|rc}  Bump the specified version number;
                               resets less-significant numbers to zero.
			       All but 'rc' releases drop that tag.
USAGE
}

do_version_sed() {
	local OLD_VERSION="${PACKAGE_VERSION}"
	local NEW_VERSION="$1"
	local MSG="$2"

	sed -i -e "/AC_INIT/ s|${OLD_VERSION}|${NEW_VERSION}|" configure.in
	package_info_load
	echo "${MSG}: ${OLD_VERSION} -> ${NEW_VERSION}"
}
do_version_bump_sed() {
	local NEW_VERSION="$1"
	[ -z "${PACKAGE_VERSION_TAGS}" ] || \
		NEW_VERSION="${NEW_VERSION}${PACKAGE_VERSION_TAGS}"

	do_version_sed "${NEW_VERSION}" \
		"Bump ${CMD} package version number"
}
do_version_bump_major() {
	has_version_tag 'rc\d' do_version_
	do_version_bump_sed "$((PACKAGE_MAJOR + 1)).0.0"
}
do_version_bump_minor() {
	do_version_bump_sed "${PACKAGE_MAJOR}.$((PACKAGE_MINOR + 1)).0"
}
do_version_bump_micro() {
	do_version_bump_sed "${PACKAGE_MAJOR_AND_MINOR}.$((PACKAGE_MICRO + 1))"
}
do_version_bump_rc() {
	die "patch missing: -rc support is not implemented"
}
do_version_bump() {
	CMD="$1"
	shift
	case "${CMD}" in
	major|minor|micro|rc)
		eval "do_version_bump_${CMD}"
		;;
	*)
		do_version_usage
		;;
	esac
}

has_version_tag() {
	test "${PACKAGE_VERSION/-${TAG}/}" != "${PACKAGE_VERSION}"
}

do_version_tag_add() {
	local TAG="$1"
	has_version_tag && die "error: tag '-${TAG}' exists in '${PACKAGE_VERSION}'"
	do_version_sed "${PACKAGE_VERSION}-${TAG}" \
		"Add '-${TAG}' version tag"
}
do_version_tag_remove() {
	local TAG="$1"
	has_version_tag || die "error: tag '-${TAG}' missing from '${PACKAGE_VERSION}'"
	do_version_sed "${PACKAGE_VERSION/-${TAG}/}" \
		"Remove '-${TAG}' version tag"
}
do_version_tag() {
	CMD="$1"
	shift
	case "${CMD}" in
	add|remove)
		local i=
		for i in "$@"; do 
			eval "do_version_tag_${CMD}" "${i}"
		done
		;;
	*)
		do_version_usage
		;;
	esac
}

do_version_commit() {
	[ "$(svn diff configure.in | wc -l)" -gt 0 ] || \
		die "error: no version changes to commit"
	do_svn commit -m "$1" configure.in
}

do_version() {
	package_info_load
	CMD="$1"
	shift
	case "${CMD}" in
	tag|bump)
		do_version_commit "$(eval "do_version_${CMD}" "$@")"
		maybe_rebuild
		;;
	commit)
		local MSG="$1"
		[ "${MSG}" ] || die "usage: $0 version commit <message>"
		do_version_commit "${MSG}"
		maybe_rebuild
		;;
	*)
		do_version_usage
		;;
	esac
}


do_branch() {
	package_info_load
	svn_setup_load
	do_svn copy -m "Branching version ${PACKAGE_VERSION}" \
		"${SVN_TRUNK}" "${PACKAGE_BRANCH}"
}
do_tag() {
	package_info_load
	svn_setup_load
	do_svn copy -m "Tagging version ${PACKAGE_VERSION}" \
		"${PACKAGE_BRANCH}" "${PACKAGE_TAG}"
}
do_commit() {
	package_info_load
	svn_setup_load

	[ "${PACKAGE_VERSION/in-development/}" = "${PACKAGE_VERSION}" ] || \
		die "'${PACKAGE_NAME}-${PACKAGE_VERSION}' cannot be released"

	[ "${PACKAGE_VERSION%.0}" = "${PACKAGE_VERSION}" ] || \
		do_branch
	do_tag
}


do_release_step_prep() {
	do_version tag remove in-development
	# reset RELEASE_VERSION now to allow release version to be detected
	export RELEASE_VERSION=
}
do_release_step_commit() { do_commit; }

do_release_step_branch_bump() {
	local TYPE="$1"
	echo "Bump ${TYPE} version and add tag:"
	do_version_bump ${TYPE}
	do_version_tag_add in-development
}
do_release_step_branch() {
	do_svn switch "${PACKAGE_BRANCH}"
	package_info_load
	do_version_commit "$(do_release_step_branch_bump micro)"
	do_svn switch "${SVN_URL}"
	package_info_load
}
do_release_step_bump() {
	# major and minor releases require branch version update too
	[ "${RELEASE_TYPE}" = "micro" ] || do_release_step_branch
	# bump the current tree version as required.
	do_version_commit "$(do_release_step_branch_bump "${RELEASE_TYPE}")"

	# archive NEWS and create new one from template
	do_svn move "NEWS" "NEWS-${RELEASE_VERSION}"

	[ "${RELEASE_DRY_RUN}" ] || cat >NEWS <<NEWS
This file should include items worth mentioning in the
OpenOCD ${PACKAGE_RELEASE} source archive release.

The following areas of OpenOCD functionality changed in this release:

JTAG Layer:
Target Layer:
Flash Layer:
Board, Target, and Interface Configuration Scripts:
Documentation:
Build and Release:

For more details about what has changed since the last release,
see the ChangeLog associated with this source archive.  For older NEWS,
see the NEWS files associated with each release (i.e. NEWS-<version>).

For more information about contributing test reports, bug fixes, or new
features and device support, please read the new Developer Manual (or
the BUGS and PATCHES files in the source archive).
NEWS

	MSG=<<MSG
Archive released NEWS file: NEWS -> NEWS-${RELEASE_VERSION}
Create new NEWS file from relesse script template.
MSG
	do_svn commit -m "${MSG}" NEWS NEWS-${RELEASE_VERSION}
}
do_release_step_package() {
	local A=${PACKAGE_TAG}
	local B=${A/https/http}
	local PACKAGE_BUILD=${B/${USER}@/}
	do_svn switch "${PACKAGE_BUILD}"
	do_stage
	do_clean
}

do_release_step_1() { do_release_step_prep; }
do_release_step_2() { do_release_step_commit; }
do_release_step_3() { do_release_step_bump; }
do_release_step_4() { do_release_step_package; }

do_release_check() {
	echo -n "Are you sure you want to release '${PACKAGE_RELEASE}'?"
	read ANSWER
	if [ "${ANSWER}" != 'y' ]; then
		echo "Live release aborted!"
		exit 0
	fi
}
do_countdown() {
	echo -n "$1 in "
	for i in $(seq 5 -1 1); do
		echo -n "$i, "
	done
	echo "go!"
}

do_release() {
	package_info_load
	package_info_show

	if [ -z "${RELEASE_DRY_RUN}" ]; then
		do_release_check
		do_countdown "Starting live release"
	fi

	local i=
	for i in $(seq 1 4); do
		eval "do_release_step_${i}"
	done
}
do_all() { do_release "$@"; }

do_reset() {
	maybe_bootstrap
	maybe_configure
	do_clean_all
	svn revert configure.in
}

OPTIONS=$(getopt -o V --long live -n $0 -- "$@")
if [ $? != 0 ] ; then echo "Terminating..." >&2 ; exit 1 ; fi
eval set -- "${OPTIONS}"
while true; do
	case "$1" in
	--live)
		export RELEASE_DRY_RUN=
		shift
		;;
	-V)
		exec $0 info
		;;
	--)
		shift
		break
		;;
	*)
		echo "Internal error"
		exit 1
		;;
	esac
done

CMD=$1
[ "${CMD}" ] || usage
shift

ACTION_CMDS="bootstrap|configure|build|changelog|package|stage|clean"
MISC_CMDS="all|info|version|tag|branch|commit|release|reset|help|usage"
CLEAN_CMDS="build_clean|changelog_clean|package_clean|stage_clean|clean_all"
CMDS="|${ACTION_CMDS}|${CLEAN_CMDS}|${MISC_CMDS}|"
is_command() { echo "${CMDS}" | grep "|$1|" >/dev/null; }

if is_command "${CMD}"; then
	eval "do_${CMD}" "$@"
else
	echo "error: unknown command: '${CMD}'"
	usage
fi
