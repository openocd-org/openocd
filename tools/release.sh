#!/bin/bash
# release.sh: openocd release process automation
# Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>
# Release under the GNU GPL v2 (or later versions).

# FIXME Remove more bash-isms.  Fix errors making "ash -e" lose.

## set these to control the build process
#CONFIG_OPTS=""
#MAKE_OPTS=""

## specifies the --next release type: major, minor, micro, rc, tag
#RELEASE_TYPE=tag
## For tag release type, specifies the name of the tag (e.g. "foo").
## The default is the current user name, as found by the 'id' command.
#RELEASE_TAG="$(id -un)"

. "tools/release/helpers.sh"

VERSION_SH="tools/release/version.sh"

usage() {
	cat << USAGE
usage: $0 <command> ...
Command Options:
  --next name   The branch's next release type: major, minor, micro, rc, tag.
  --next-tag name   The name for the package version tag.
  --live        Perform the actions in the repository.

Main Commands:
  info          Show a summary of the next pending release.
  release       Release the current tree as an archive.

Build Commands:
  bootstrap     Prepare the working copy for configuration and building.
  configure     Configures the package; runs bootstrap, if needed.
  build         Compiles the project; runs configure, if needed.

Packaging Commands:
  package       Produce new distributable source archives.
  stage         Move archives to staging area for upload.

Other Commands:
  clean         Forces regeneration of results.
  clean_all     Removes all traces of the release process.
  help          Provides this list of commands.

For more information about this script, see the Release Processes page
in the OpenOCD Developer's Manual (doc/manual/release.txt).
USAGE
	exit 0
}
do_usage() { usage; }
do_help()  { usage; }

do_info() {
	echo "Current Release Analysis:"
	package_info_show
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


do_package() {
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
		local FILE="${PACKAGE_RELEASE}.${EXT}"
		# create archive signatures
		for HASH in sha256; do
			echo "sign: ${FILE}.${HASH}"
			${HASH}sum "${FILE}" > "archives/${FILE}.${HASH}"
		done
		# save archive
		mv -v "${FILE}" archives/
	done
	cp -a NEWS archives/
}
do_stage_clean() { rm -v -f -r archives; }

do_clean() {
	do_build_clean
	do_package_clean
	rm -v -f release-*.log
}
do_clean_all() {
	do_clean
	do_stage_clean
}

do_version_commit() {
	[ "$*" ] || die "usage: $0 commit <message>"
	git add configure.ac || die "error: no version changes to commit"
	git commit -q -m "$*" configure.ac
}

do_version_finalize() {
	echo "The ${PACKAGE_NAME} ${RELEASE_VERSION} release."
	echo
	${VERSION_SH} tag remove dev
	[ -z "${RELEASE_FINAL}" ] || ${VERSION_SH} bump final rc
}
has_dev_tag() {
	[ "${PACKAGE_VERSION/dev/}" != "${PACKAGE_VERSION}" ]
}
do_release_step_branch() {
	git checkout -b "v${RELEASE_VERSION}-release"
}

do_release_step_tag() {
	do_version_commit "$(do_version_finalize)"
	package_info_load
	[ "${PACKAGE_VERSION/dev/}" = "${PACKAGE_VERSION}" ] || \
		die "'${PACKAGE_NAME}-${PACKAGE_VERSION}' should not be tagged"
	local MSG="The ${PACKAGE_STRING} release."
	git tag -m "${MSG}" "v${PACKAGE_VERSION}"
}

do_bump_version() {
	echo -n "Bump ${RELEASE_TYPE} "
	[ -z "${RELEASE_TAG}" ] || echo -n "-${RELEASE_TAG} "
	echo -n "version and add "
	[ -z "${RELEASE_START_RC}" ] || echo -n "-rc0"
	echo "-dev tag."
	echo
	${VERSION_SH} bump "${RELEASE_TYPE}" "${RELEASE_TAG}"
	[ -z "${RELEASE_START_RC}" ] || ${VERSION_SH} bump tag rc
	${VERSION_SH} tag add dev
}
do_release_step_bump() {
	# bump the version number
	do_version_commit "$(do_bump_version)"
}

do_release_step_news_msg() {
	cat <<MSG
Archive and recreate NEWS file.

Archive released NEWS file as NEWS-${RELEASE_VERSION}.
Create new NEWS file from release script template.
MSG
}
do_release_step_news() {
	# only archive the NEWS file for major/minor releases
	[ "${RELEASE_TYPE}" = "major" -o "${RELEASE_TYPE}" = "minor" ] || \
		return 0
	# archive NEWS and create new one from template
	git mv "NEWS" "NEWS-${RELEASE_VERSION}"

	cat >NEWS <<NEWS
This file includes highlights of the changes made in the
OpenOCD ${NEXT_RELEASE_VERSION} source archive release.  See the
repository history for details about what changed, including
bugfixes and other issues not mentioned here.

JTAG Layer:
Boundary Scan:
Target Layer:
Flash Layer:
Board, Target, and Interface Configuration Scripts:
Documentation:
Build and Release:

For more details about what has changed since the last release,
see the git repository history.  With gitweb, you can browse that
in various levels of detail.

For older NEWS, see the NEWS files associated with each release
(i.e. NEWS-<version>).

For more information about contributing test reports, bug fixes, or new
features and device support, please read the new Developer Manual (or
the BUGS and PATCHES.txt files in the source archive).
NEWS
	git add NEWS

	local MSG="$(do_release_step_news_msg)"
	git commit -q -m "${MSG}" NEWS "NEWS-${RELEASE_VERSION}"
}

do_release_step_package() {
	[ -z "${RELEASE_FAST}" ] || return 0

	git checkout -q "v${RELEASE_VERSION}"
	do_stage
	do_clean
}

do_release_step_rebranch() {
	# return to the new development head
	local OLD_BRANCH="v${RELEASE_VERSION}-release"
	git checkout "${OLD_BRANCH}"

	# create new branch with new version information
	package_info_load
	git checkout -b "v${PACKAGE_VERSION}"
	git branch -d "${OLD_BRANCH}"
}

do_release_setup() {
	echo "Starting $CMD for ${RELEASE_VERSION}..."
	[ "${RELEASE_TYPE}" ] || \
		die "The --next release type must be provided.  See --help."
}

do_release_check() {
	[ -z "${RELEASE_FAST}" ] || return 0
	echo "Are you sure you want to ${CMD} '${PACKAGE_RELEASE}', "
	echo -n "   to start a new  ${RELEASE_TYPE}  development cycle? (y/N) "
	read ANSWER
	if [ "${ANSWER}" != 'y' ]; then
		echo "Live release aborted!"
		exit 0
	fi
	do_countdown "Starting live release"
}
do_countdown() {
	echo -n "$1 in "
	for i in $(seq 5 -1 1); do
		echo -n "$i, "
		sleep 1
	done
	echo "go!"
}

do_branch() {
	do_release_setup
	local i=
	for i in branch bump rebranch; do
		"do_release_step_${i}"
	done
}

do_release() {
	local CMD='release'
	do_release_setup
	do_release_check
	local i=
	for i in branch tag bump news package rebranch; do
		"do_release_step_${i}"
	done
}
do_all() { do_release "$@"; }

do_reset() {
	maybe_bootstrap
	maybe_configure
	do_clean_all
	git checkout configure.ac
}

LONGOPTS="fast,final,start-rc,next-tag:,next:,help"
OPTIONS=$(getopt -o 'V,n:' --long "${LONGOPTS}" -n $0 -- "$@")
if [ $? != 0 ] ; then echo "Terminating..." >&2 ; exit 1 ; fi
eval set -- "${OPTIONS}"
while true; do
	case "$1" in
	--fast)
		RELEASE_FAST=yes
		shift
		;;
	--final)
		RELEASE_FINAL=yes
		shift
		;;
	--start-rc)
		RELEASE_START_RC=yes
		shift
		;;
	-n|--next)
		export RELEASE_TYPE="$2"
		shift 2
		;;
	--next-tag)
		export RELEASE_TAG="$2"
		shift 2
		;;
	-V)
		exec $0 info
		;;
	--)
		shift
		break
		;;
	--help)
		usage
		shift
		;;
	*)
		echo "Internal error"
		exit 1
		;;
	esac
done

case "${RELEASE_TYPE}" in
major|minor|micro|rc)
	;;
tag)
	[ "${RELEASE_TAG}" ] || RELEASE_TAG="$(id -u -n)"
	;;
'')
	;;
*)
	die "Unknown release type '${RELEASE_TYPE}'"
	;;
esac

CMD=$1
[ "${CMD}" ] || usage
shift

ACTION_CMDS="bootstrap|configure|build|package|stage|clean"
MISC_CMDS="all|info|release|branch|reset|help|usage"
CLEAN_CMDS="build_clean|package_clean|stage_clean|clean_all"
CMDS="|${ACTION_CMDS}|${CLEAN_CMDS}|${MISC_CMDS}|"
is_command() { echo "${CMDS}" | grep "|$1|" >/dev/null; }

package_info_load
if is_command "${CMD}"; then
	"do_${CMD}" "$@"
	echo "Done with '${CMD}'." >&2
else
	echo "error: unknown command: '${CMD}'"
	usage
fi
