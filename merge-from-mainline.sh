#! /bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# (c) 2020, Google
progname="${0##*/}"

[ "USAGE: repo_sync_rebase_prune

Perform an aggresive repo sync and cleanup" ]
repo_sync_rebase_prune() {
  if [ ! -d .repo ]; then
    echo "Not at top of repo" >&2
    return 1
  fi
  repo sync -j64 || true
  repo rebase || return 1
  repo sync -j64 2>&1 |
    tee /dev/stderr |
    grep '^error: \|^Failing repos:\|^Please commit your changes or stash them' >/dev/null &&
    exit 1
  repo rebase || return 1
  repo prune || return 1
  return 0
}

repo_sync_rebase_prune || exit

[ "USAGE: do_merge <directory> <branch>

Perform a merge over a repo, using BRANCH as the holder." ]

BRANCH="merge.`date +%s`"
AUTHOR_NAME="`git config user.name`"
AUTHOR_EMAIL="`git config user.email`"

do_merge() {
  dir=${1}
  shift
  from_branch=${1}
  shift
  (
    cd ${dir} || exit 1
    branch=`git branch -r 2>&1 | sed -n 's/ *m\/.* -> //p'`
    [ -n "${branch}" ] || branch=partner/android-gs-pixel-5.10-stabilization
    repo start ${BRANCH} . || exit 1
    commits="`git cherry -v ${branch} ${from_branch} |
                sed -n 's/^[+] //p'`"
    titles="`echo \"${commits}\" |
               sed 's/[0-9a-fA-F]* /  /'`"
    bug="`echo \"${commits}\" |
            while read sha title; do
              git show --no-patch ${sha}
            done |
            sed -n 's/ *Bug: //p' |
            tr ' \t,' '\n\n\n' |
            sed 's@b/\([0-9]\)@\1@g' |
            sort -u |
            grep -v '^$' |
            sed 's/.*/Bug: &/'`"
    git merge --no-ff ${from_branch} --m "Merge ${from_branch} into ${branch}
${@}

* ${from_branch}:
${titles}

Signed-off-by: ${AUTHOR_NAME} "'<'"${AUTHOR_EMAIL}"'>'"
${bug}"
  ) ||
    echo Failed merge of ${dir} >&2
}

[ "Perform Merge" ]

find common private -type d -name .git |
  while read gitdir; do
    dir=${gitdir%/.git}
    case ${dir} in
      common)
        do_merge ${dir} partner-common/android12-5.10-staging
        ;;
      */wlan/*)
        do_merge ${dir} partner/android-gs-pixel-4.19 "
NB: All development for this driver for 4.19, 5.4 and 5.10 is in
    partner/android-gs-pixel-4.19 tree.  If there are any problems
    with one tree, then the maintainers have made it their
    responsibility to not create branches and figure out a solution
    that works in all three kernel versions and beyond."
        ;;
      *)
        do_merge ${dir} partner/android-gs-pixel-mainline
        ;;
    esac ||
      echo ERROR: merge ${dir} failed
  done 2>&1 |
  tee /dev/stderr |
  grep 'Failed merge of ' |
  (
    OUT=
    while read o; do
      OUT="${OUT}
${o}"
    done
    repo prune
    echo "${OUT}" >&2
  )
