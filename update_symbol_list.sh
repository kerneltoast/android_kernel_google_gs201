#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

function usage {
  echo "USAGE: $0 [-p|--prepare-aosp-abi] [-cbr|--current-branch]"
  echo
  echo "  -p   | --prepare-aosp-abi    Update the AOSP ABI representation and symbol list in aosp/"
  echo "  -cbr | --current-branch      Use the current AOSP branch for updating the AOSP ABI"
}

# Add a trap to remove the temporary vmlinux in case of an error occurs before
# we finish.
cleanup_trap() {
  rm -f ${VMLINUX_TMP} ${TMP_LIST} ${COMMIT_TEXT}
  exit $1
}
trap 'cleanup_trap' EXIT

function exit_if_error {
  if [ $1 -ne 0 ]; then
    echo "ERROR: $2: retval=$1" >&2
    exit $1
  fi
}

# Cat the symbol lists together, sort and update them to all match
# $1 The final symbol list (included in the list to sort)
# $@ The symbol lists to sort and merge together
function merge_and_sort_symbol_lists {
  # Remove blank lines and comments. Then sort
  TMP_LIST=$(mktemp -t symbol_list.XXXX)
  cat "$@" > ${TMP_LIST}
  sed -i '/^$/d' ${TMP_LIST}
  sed -i '/^#/d' ${TMP_LIST}
  sort -u ${TMP_LIST} > $1

  # Need to have the AOSP and pixel symbol lists match to avoid the ksymtab
  # vs symbol list comparison check.
  if [ $# -gt 1 ]; then
    for (( i=2; i<=$#; i++ )); do
      cp -f "$1" "${!i}"
    done
  fi

  rm -f ${TMP_LIST}
}

function update_aosp_abi {
  echo "========================================================"
  echo " Extracting symbols and updating the AOSP ABI"
  local out_dir="out_aosp_abi"
  local pixel_symbol_list="android/abi_gki_aarch64_generic"

  pushd aosp/ >/dev/null
    # Rebase to aosp/android12-5.10 ToT before updating the ABI
    if [ -n "${AOSP_BACKUP_BRANCH}" ]; then
      git checkout -b ${AOSP_BACKUP_BRANCH} && git checkout --detach
      err=$? && exit_if_error ${err} "Failed to create backup branch ${AOSP_BACKUP_BRANCH}!"
    fi
    git fetch aosp android12-5.10 && git rebase FETCH_HEAD
    err=$? && exit_if_error ${err} "Failed to rebase aosp/ to the ToT!"

    if [ -n "${FOR_AOSP_PUSH_BRANCH}" ]; then
      git checkout -b ${FOR_AOSP_PUSH_BRANCH}
      err=$? && exit_if_error ${err} "Failed to create the branch ${AOSP_BACKUP_BRANCH}!"
    fi
  popd >/dev/null

  # First, rollback any symbol list changes in aosp/ and then regenerate the
  # list based on AOSP ToT and the updated pixel symbol list. This ensures that
  # we only add symbols needed based on the current pixel changes.
  #
  # Note: we are purposefully not using `--additions-only` in order to avoid
  # adding symbols in the pixel tree during development that later get removed.
  # To retain symbols in the same way as `--additions-only` does, we are
  # cat'ing the private/gs-google/ and ToT aosp/ symbol lists together when
  # preparing for the AOSP ABI update. This retains all symbols in the aosp
  # version of the pixel symbol list.
  git -C aosp show aosp/android12-5.10:"${pixel_symbol_list}" \
    > aosp/android/abi_gki_aarch64_generic
  extract_pixel_symbols 0 "private/gs-google/${pixel_symbol_list}"
  merge_and_sort_symbol_lists "aosp/${pixel_symbol_list}" \
    "private/gs-google/${pixel_symbol_list}"

  # Update the AOSP ABI xml now
  rm -rf ${out_dir}
  OUT_DIR=${out_dir} \
    BUILD_CONFIG=aosp/build.config.gki.aarch64 \
    SKIP_CP_KERNEL_HDR=1 \
    LTO=full \
    DIST_DIR= \
    KBUILD_MIXED_TREE= \
    SKIP_MRPROPER= \
    build/build_abi.sh --update "$@"

  # TODO: How do I know if the build failed or the ABI xml was updated??

  # Create the git commit for aosp/android12-5.10
  COMMIT_TEXT=$(mktemp -t abi_commit_text.XXXXX)
  if [ -f "${out_dir}/dist/abi.report.short" ]; then
    echo "ANDROID: Update the ABI xml and symbol list" > ${COMMIT_TEXT}
    echo >> ${COMMIT_TEXT}
    cat ${out_dir}/dist/abi.report.short >> ${COMMIT_TEXT}
  else
    echo "ANDROID: Update the ABI symbol list" > ${COMMIT_TEXT}
    echo >> ${COMMIT_TEXT}
    echo "Update the generic symbol list." >> ${COMMIT_TEXT}
  fi
  echo >> ${COMMIT_TEXT}
  echo "Bug: XXX" >> ${COMMIT_TEXT}
  git -C aosp commit -s -F ${COMMIT_TEXT} -- android/
  err=$?
  echo "========================================================"
  if [ "${err}" = "0" ]; then
    if [ -n "${FOR_AOSP_PUSH_BRANCH}" ]; then
      echo " An ABI commit in aosp/ was created for you on the branch ${FOR_AOSP_PUSH_BRANCH}."
    else
      echo " An ABI commit in aosp/ was created for you on the current branch."
    fi
    echo " Please verify the commit before pushing. Leave the rest of the commit text as-is."
    echo " Here are the steps to perform:"
    echo
    echo "   cd aosp"
    echo "   git show ${FOR_AOSP_PUSH_BRANCH}"
    echo "   git push aosp ${FOR_AOSP_PUSH_BRANCH:-HEAD}:refs/for/android12-5.10"
    echo
    if [ -n "${FOR_AOSP_PUSH_BRANCH}" ]; then
      echo " After pushing your changes to aosp/, you can delete the temporary"
      echo " branch: ${FOR_AOSP_PUSH_BRANCH} using the command:"
      echo
      echo "   cd aosp"
      echo "   git branch -D ${FOR_AOSP_PUSH_BRANCH}"
      echo
    fi
  else
    echo " No changes were detected after running build_abi.sh."
  fi

  rm -f ${COMMIT_TEXT}
  # Rollback to the original commit and remove the backup branch we created
  pushd aosp >/dev/null
    if [ -n "${AOSP_BACKUP_BRANCH}" ]; then
      git checkout ${AOSP_BACKUP_BRANCH}
      git checkout --detach
      git branch -D ${AOSP_BACKUP_BRANCH}
    fi
  popd >/dev/null
}

# Extract the kernel module symbols. Additionally, we strip out the core ABI
# symbols and sort the symbol list. We do our own sort in order to retain
# a predictable order when cat'ing the symbol lists in the pixel tree and aosp/
# tree.
# $1 Specifies if --additions-only should be used
# $2 The symbol list to update/create
function extract_pixel_symbols {
  echo "========================================================"
  echo " Extracting symbols and updating the symbol list"
  local clang_prebuilt_bin=$(. private/gs-google/build.config.common && \
    echo $CLANG_PREBUILT_BIN)
  local additions_only=$1
  local pixel_symbol_list=$2

  if [ "${additions_only}" != "0" ]; then
    ADD_ONLY_FLAG="--additions-only"
  fi

  # Need to copy over the vmlinux to be under the same directory as we point
  # extract_symbols to.
  cp ${DIST_DIR}/vmlinux ${VMLINUX_TMP}

  PATH=${PATH}:${clang_prebuilt_bin}
  build/abi/extract_symbols              \
      --symbol-list ${pixel_symbol_list} \
      --skip-module-grouping             \
      ${ADD_ONLY_FLAG}                   \
      ${BASE_OUT}/device-kernel/private
  err=$? && exit_if_error ${err} "Failed to extract symbols!"

  # Strip the core ABI symbols from the pixel symbol list
  grep "^ " aosp/android/abi_gki_aarch64_core | while read l; do
    sed -i "/\<$l\>/d" ${pixel_symbol_list}
  done

  # Sort the pixel symbol list
  merge_and_sort_symbol_lists ${pixel_symbol_list}

  # Clean up
  rm -f ${VMLINUX_TMP}
}

export SKIP_MRPROPER=1
export BASE_OUT=${OUT_DIR:-out}/mixed/
export DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/}
VMLINUX_TMP=${BASE_OUT}/device-kernel/private/vmlinux
# Use mktemp -u to create a random branch name
AOSP_BACKUP_BRANCH=$(basename $(mktemp -u -t aosp_backup.XXXX))
FOR_AOSP_PUSH_BRANCH="update_symbol_list-delete-after-push"
PREPARE_AOSP_ABI=${PREPARE_AOSP_ABI:-0}

ARGS=()
while [[ $# -gt 0 ]]; do
  next="$1"
  case ${next} in
    -p|--prepare-aosp-abi)
      PREPARE_AOSP_ABI=1
      ;;
    -cbr|--current-branch)
      FOR_AOSP_PUSH_BRANCH=
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      ARGS+=("$1")
      ;;
  esac
  shift
done
set -- "${ARGS[@]}"

if [ "${PREPARE_AOSP_ABI}" != "0" ]; then
  if ! git -C aosp diff --quiet HEAD; then
    exit_if_error 1 \
      "Found uncommitted changes in aosp/. Commit your changes before updating the ABI"
  fi

  if [ -n "${FOR_AOSP_PUSH_BRANCH}" ]; then
    if git -C aosp branch | grep "^\*\{0,1\}\s*\<${FOR_AOSP_PUSH_BRANCH}\>$" 2>&1 >/dev/null; then
      echo "The branch '${FOR_AOSP_PUSH_BRANCH}' already exists in aosp/. Please delete"
      echo "this branch (git -D ${FOR_AOSP_PUSH_BRANCH}) before continuing. Alternatively,"
      echo "you can use \`-cbr|--current-branch\` to use the current aosp/ local branch."
      exit 1
    fi
  else
    AOSP_BACKUP_BRANCH=
  fi
fi

BUILD_KERNEL=1 TRIM_NONLISTED_KMI=0 ENABLE_STRICT_KMI=0 ./build_slider.sh "$@"
err=$? && exit_if_error ${err} "Failed to run ./build_slider.sh!"

if [ "${PREPARE_AOSP_ABI}" != "0" ]; then
  update_aosp_abi "$@"
else
  extract_pixel_symbols 1 "private/gs-google/android/abi_gki_aarch64_generic"
  merge_and_sort_symbol_lists "aosp/android/abi_gki_aarch64_generic" \
    "private/gs-google/android/abi_gki_aarch64_generic"

  echo "========================================================"
  echo " The symbol list has been updated locally in aosp/ and private/gs-google."
  echo " Compiling with BUILD_KERNEL=1 is now required until the new symbol(s)"
  echo " are merged. Re-compile using the below command:"
  echo
  echo " SKIP_MRPROPER=1 BUILD_KERNEL=1 ./build_slider.sh"
  echo
fi
