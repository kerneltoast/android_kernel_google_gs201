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
  (
    cd ${dir} || exit 1
    branch=`git branch -r 2>&1 | sed -n 's/ *m\/.* -> //p'`
    [ -n "${branch}" ] || branch=partner/android-gs-pixel-5.10-stabilization
    repo start ${BRANCH} || exit 1
    commits="`git cherry -v ${branch} ${1} |
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
    git merge --no-ff ${*} --m "Merge remote-tracing branch '${1}' into ${branch}

* ${1}:
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
        do_merge ${dir} partner/android-gs-pixel-4.19
        ;;
      *)
        do_merge ${dir} partner/android-gs-pixel-mainline
        ;;
    esac
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
