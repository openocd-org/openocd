#!/bin/sh
TOPDIR=`pwd`
USERNAME=$1

if [ "x$1" = "x" ] ; then
	echo "Usage:	$0	<Username>"
	exit 1
fi

add_remote()
{
	remote_exist=`grep remote .git/config | grep review	| wc -l`
	if [ "x$remote_exist" = "x0" ] ; then
		git remote add review ssh://$USERNAME@review.openocd.org:29418/openocd.git
		git config remote.review.push HEAD:refs/for/master
	else
		echo "Remote review exists"
	fi
}

update_commit_msg()
{
	cd "${TOPDIR}/.git/hooks"
	save_file=commit-msg-`date +%F-%T`
	mv commit-msg $save_file
	printf "%-30s"	"Updating commit-msg"
	status="OK"
	wget -o log	https://review.openocd.org/tools/hooks/commit-msg	|| status="FAIL"
	echo $status
	if [ $status = "FAIL" ] ; then
		mv	$save_file	commit-msg
	fi
	chmod a+x commit-msg
}

add_remote
update_commit_msg
