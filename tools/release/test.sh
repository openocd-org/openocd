#!/bin/sh -e

SRC="$1"
if [ "${SRC}" ]; then
	shift
else
	SRC="${0%%/*}"
fi
if [ ! -d "${SRC}/.git" ]; then
	echo "'${SRC}' is not a git repository"
	exit 1
fi

DST="$1"
[ "${DST}" ] || DST="release-${SRC}"

export RELEASE_FAST=yes

cat <<EOF
Starting test release clone process:
   from: '${SRC}'
     to: '${DST}'
This will destroy any contents in '${DST}'.
EOF
echo -n "Press Control-C to abort in "
for i in $(seq 5 -1 1); do echo -n "$i "; sleep 1; done
echo "go!"

rm -rf "${DST}"
git clone "${SRC}" "${DST}"

cd "${DST}"
#     TAG+RELEASE     NEW BRANCH (w/ -dev)
# 	0.3.0 		0.4.0-rc0
tools/release.sh release --next='minor' --start-rc

git checkout -q "v0.3.0"
# 	<none>		0.3.1
tools/release.sh branch --next='micro'
# 	0.3.1 		0.3.2
tools/release.sh release --next='micro'

git checkout "v0.4.0-rc0-dev"
# 	0.4.0-rc0	0.4.0-rc1
tools/release.sh release --next='rc'
# 	0.4.0		1.0.0-rc0
tools/release.sh release --next='major' --final --start-rc

git checkout -q "v0.4.0"
# 	<none>		0.4.1
tools/release.sh branch --next='micro'
# 	0.4.1 		0.4.2
tools/release.sh release --next='micro'

git checkout "v1.0.0-rc0-dev"
# 	1.0.0-rc0	1.0.0-rc1
tools/release.sh release --next='rc'
# 	1.0.0		1.1.0-rc0
tools/release.sh release --next='minor' --final --start-rc

git checkout -q "v1.0.0"
# 	<none>		1.0.1
tools/release.sh branch --next='micro'
# 	1.0.1 		1.0.2
tools/release.sh release --next='micro'

git checkout "v1.1.0-rc0-dev"
# 	1.1.0-rc0	1.1.0-rc1
tools/release.sh release --next='rc'
# 	1.1.0		1.2.0
tools/release.sh release --next='minor' --final --start-rc

git checkout -q "v1.0.0"
tools/release.sh branch --next='major' --start-rc

# 	<none>		2.0.0-rc0
git checkout "v2.0.0-rc0-dev"
# 	2.0.0-rc0	2.0.0-rc1
tools/release.sh release --next='rc'
# 	2.0.0-rc1	2.0.0-rc2
tools/release.sh release --next='rc'
# 	2.0.0		2.1.0-rc0
tools/release.sh release --next='minor' --final --start-rc

git checkout -q "v1.1.0"
# 	<none>		1.1.1
tools/release.sh branch --next='micro'
# 	1.1.1 		1.1.2
tools/release.sh release --next='micro'

git checkout -q "v2.0.0"
# 	<none>		2.0.0
tools/release.sh branch --next='micro'
# 	2.0.1		2.0.2
tools/release.sh release --next='micro'

git checkout "v1.2.0-rc0-dev"
# 	1.2.0-rc0	1.2.0-rc1
tools/release.sh release --next='rc'
# 	1.2.0		1.3.0-rc0
tools/release.sh release --next='micro' --final

git checkout "v2.1.0-rc0-dev"
# 	2.1.0-rc0	2.1.0-rc1
tools/release.sh release --next='rc'
# 	2.1.0-rc1	2.1.0-rc2
tools/release.sh release --next='rc'
# 	2.1.0		2.2.0-rc0
tools/release.sh release --next='minor' --final --start-rc

git checkout -q "v2.1.0"
# 	<none>		2.1.1
tools/release.sh branch --next='micro'
# 	2.1.1		2.1.2
tools/release.sh release --next='micro'

git checkout "v2.2.0-rc0-dev"
# 	2.2.0-rc0	2.2.0-rc1
tools/release.sh release --next='rc'

gitk --all
