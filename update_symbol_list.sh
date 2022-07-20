# SPDX-License-Identifier: GPL-2.0
#!/bin/bash

function usage {
  echo "USAGE: $0 [-p|--prepare-aosp-abi BUG_NUMBER [-c|--continue] [--change-id CHANGE_ID]]"
  echo
  echo "  -p | --prepare-aosp-abi BUG_NUMBER   Update the AOSP ABI xml and symbol list in ${GKI_KERNEL_DIR}/ "
  echo "                                       and create a commit with the provide BUG_NUMBER."
  echo "  -c | --continue                      Continue after the rebase failure"
  echo "  --change-id CHANGE_ID                Use this Change-Id when creating the AOSP commit"
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

function verify_aosp_tree {
  if [ "${PREPARE_AOSP_ABI}" = "0" ]; then
    return
  fi

  pushd ${GKI_KERNEL_DIR} >/dev/null
    if ! git diff --quiet HEAD; then
      exit_if_error 1 \
        "Found uncommitted changes in ${GKI_KERNEL_DIR}/. Commit your changes before updating the ABI"
    fi

    if [ "${CONTINUE_AFTER_REBASE}" = "0" ]; then
      if git branch | grep "\<${FOR_AOSP_PUSH_BRANCH}\>" 2>&1 >/dev/null; then
        echo "The branch '${FOR_AOSP_PUSH_BRANCH}' already exists in ${GKI_KERNEL_DIR}/. Please delete" >&2
        echo "this branch (git branch -D ${FOR_AOSP_PUSH_BRANCH}) before continuing." >&2
        exit 1
      fi

      AOSP_CUR_BRANCH_OR_SHA1=$(git branch --show-current)
      if [ -z "${AOSP_CUR_BRANCH_OR_SHA1}" ]; then
        AOSP_CUR_BRANCH_OR_SHA1=$(git log -1 --pretty="format:%H")
      fi
    else
      # Make sure they didn't switch branches when addressing the rebase conflict
      if [ "${FOR_AOSP_PUSH_BRANCH}" != "$(git branch --show-current)" ]; then
        exit_if_error 1 "For --continue, you need to be on the branch ${FOR_AOSP_PUSH_BRANCH}"
      fi
    fi
  popd >/dev/null
}

function update_aosp_abi {
  echo "========================================================"
  echo " Extracting symbols and updating the AOSP ABI"
  local out_dir="out_aosp_abi"
  local pixel_symbol_list="android/abi_gki_aarch64_generic"

  # Rebase to ${GKI_KERNEL_REMOTE}/${GKI_KERNEL_BRANCH} ToT before updating the ABI
  pushd ${GKI_KERNEL_DIR}/ >/dev/null
    if [ "${CONTINUE_AFTER_REBASE}" = "0" ]; then
      git checkout -b ${FOR_AOSP_PUSH_BRANCH}
    fi
    git fetch ${GKI_KERNEL_REMOTE} ${GKI_KERNEL_BRANCH} && git rebase FETCH_HEAD
    err=$?
    if [ "${err}" != "0" ]; then
      echo "ERROR: Failed to rebase your ${GKI_KERNEL_DIR}/ change(s) to the ${GKI_KERNEL_REMOTE} ToT." >&2
      echo "To resolve this, please manually resolve the rebase conflicts" >&2
      echo "and run: git rebase --continue. Then resume this script" >&2
      echo "using the command:" >&2
      echo >&2
      echo "  $0 --prepare-aosp-abi ${BUG} --continue" >&2
      echo >&2
      echo "To return to your original tree in ${GKI_KERNEL_DIR}/ after finishing the" >&2
      echo "ABI update, run this git command:" >&2
      echo >&2
      echo "  git checkout ${AOSP_CUR_BRANCH_OR_SHA1}" >&2
      echo >&2
      exit 1
    fi
  popd >/dev/null

  # First, rollback any symbol list changes in ${GKI_KERNEL_DIR}/ and then regenerate the
  # list based on AOSP ToT and the updated pixel symbol list. This ensures that
  # we only add symbols needed based on the current pixel changes.
  #
  # Note: we are purposefully not using `--additions-only` in order to avoid
  # adding symbols in the pixel tree during development that later get removed.
  # To retain symbols in the same way as `--additions-only` does, we are
  # cat'ing the private/gs-google/ and ToT ${GKI_KERNEL_DIR}/ symbol lists together when
  # preparing for the AOSP ABI update. This retains all symbols in the aosp
  # version of the pixel symbol list.
  git -C ${GKI_KERNEL_DIR} show ${GKI_KERNEL_REMOTE}/${GKI_KERNEL_BRANCH}:"${pixel_symbol_list}" \
    > ${GKI_KERNEL_DIR}/android/abi_gki_aarch64_generic
  extract_pixel_symbols 0 "private/gs-google/${pixel_symbol_list}"
  merge_and_sort_symbol_lists "${GKI_KERNEL_DIR}/${pixel_symbol_list}" \
    "private/gs-google/${pixel_symbol_list}"

  # Create the symbol list commit and check if the ABI xml needs to be updated
  COMMIT_TEXT=$(mktemp -t abi_sym_commit_text.XXXXX)
  echo "ANDROID: Update the ABI symbol list" > ${COMMIT_TEXT}
  echo >> ${COMMIT_TEXT}
  echo "Update the generic symbol list." >> ${COMMIT_TEXT}
  echo >> ${COMMIT_TEXT}
  echo "Bug: ${BUG}" >> ${COMMIT_TEXT}
  if [ -n "${CHANGE_ID}" ]; then
    echo "Change-Id: ${CHANGE_ID}" >> ${COMMIT_TEXT}
  fi
  git -C ${GKI_KERNEL_DIR} commit -s -F ${COMMIT_TEXT} -- android/
  commit_ret=$?
  rm -f ${COMMIT_TEXT}

  # Check if the added symbols are included in another symbol list
  verify_new_symbols_require_abi_update "${pixel_symbol_list}"

  if [ "${ABI_XML_UPDATE_REQUIRED}" != "0" ]; then
    # Update the AOSP ABI xml now
    rm -rf ${out_dir}
    OUT_DIR=${out_dir} \
      BUILD_CONFIG=${GKI_KERNEL_DIR}/build.config.gki.aarch64 \
      SKIP_CP_KERNEL_HDR=1 \
      LTO=full \
      DIST_DIR= \
      KBUILD_MIXED_TREE= \
      SKIP_MRPROPER= \
      build/build_abi.sh --update "$@"
    # TODO: How do I know if the build failed or the ABI xml was updated??

    # Create the git ABI xml commit for ${GKI_KERNEL_REMOTE}/${GKI_KERNEL_BRANCH} if needed
    if [ -f "${out_dir}/dist/abi.report.short" ]; then
      if [ "${commit_ret}" = "0" ]; then
        # The ACK team requires the symbol list and xml changes to be committed
        # in a single patch. So reset the git repo to drop the symbol list
        # commit we made above.
        git -C ${GKI_KERNEL_DIR} reset HEAD~1
      fi

      COMMIT_TEXT=$(mktemp -t abi_xml_commit_text.XXXXX)
      echo "ANDROID: Update the ABI representation" > ${COMMIT_TEXT}
      echo >> ${COMMIT_TEXT}
      cat ${out_dir}/dist/abi.report.short >> ${COMMIT_TEXT}
      echo >> ${COMMIT_TEXT}
      echo "Bug: ${BUG}" >> ${COMMIT_TEXT}
      if [ -n "${CHANGE_ID}" ]; then
        echo "Change-Id: ${CHANGE_ID}" >> ${COMMIT_TEXT}
      fi
      git -C ${GKI_KERNEL_DIR} commit -s -F ${COMMIT_TEXT} -- android/
      commit_ret=$?
      rm -f ${COMMIT_TEXT}
    fi
  fi

  echo "========================================================"
  if ! git -C ${GKI_KERNEL_DIR} diff --quiet ${GKI_KERNEL_REMOTE}/${GKI_KERNEL_BRANCH}..HEAD; then
    if [ "${commit_ret}" = "0" ]; then
      if [ -n "${FOR_AOSP_PUSH_BRANCH}" ]; then
        echo " An ABI commit in ${GKI_KERNEL_DIR}/ was created for you on the branch ${FOR_AOSP_PUSH_BRANCH}."
      else
        echo " An ABI commit in ${GKI_KERNEL_DIR}/ was created for you on the current branch."
      fi
    else
      echo " The ABI xml and symbol list is up-to-date."
    fi
    echo " Please verify your commit(s) before pushing. Here are the steps to perform:"
    echo
    echo "   cd ${GKI_KERNEL_DIR}"
    echo "   git log --oneline ${FOR_AOSP_PUSH_BRANCH}"
    echo "   git push ${GKI_KERNEL_REMOTE} ${FOR_AOSP_PUSH_BRANCH:-HEAD}:refs/for/${GKI_KERNEL_BRANCH}"
    echo
    if [ -n "${FOR_AOSP_PUSH_BRANCH}" ]; then
      echo " After pushing your changes to ${GKI_KERNEL_DIR}/, you can delete the temporary"
      echo " branch: ${FOR_AOSP_PUSH_BRANCH} using the command:"
      echo
      echo "   cd ${GKI_KERNEL_DIR}"
      echo "   git branch -D ${FOR_AOSP_PUSH_BRANCH}"
      echo
    fi
  else
    echo " No changes were detected after rebasing to the tip of tree."
  fi

  # Rollback to the original branch/commit
  if [ -n "${AOSP_CUR_BRANCH_OR_SHA1}" ]; then
    git -C ${GKI_KERNEL_DIR} checkout ${AOSP_CUR_BRANCH_OR_SHA1}
  fi
}

# Extract the kernel module symbols. Additionally, we strip out the core ABI
# symbols and sort the symbol list. We do our own sort in order to retain
# a predictable order when cat'ing the symbol lists in the pixel tree and ${GKI_KERNEL_DIR}/
# tree.
# $1 Specifies if --additions-only should be used
# $2 The symbol list to update/create
function extract_pixel_symbols {
  echo "========================================================"
  echo " Extracting symbols and updating the symbol list"
  local clang_version=$(. private/gs-google/build.config.constants && \
    echo $CLANG_VERSION)
  local clang_prebuilt_bin=prebuilts/clang/host/linux-x86/clang-${clang_version}/bin
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
  exit_if_error $? "Failed to extract symbols!"

  # Strip the core ABI symbols from the pixel symbol list
  grep "^ " ${GKI_KERNEL_DIR}/android/abi_gki_aarch64_core | while read l; do
    sed -i "/\<$l\>/d" ${pixel_symbol_list}
  done

  # Sort the pixel symbol list
  merge_and_sort_symbol_lists ${pixel_symbol_list}

  # Clean up
  rm -f ${VMLINUX_TMP}
}

# $1 The symbol list to verify
function verify_new_symbols_require_abi_update {
  local pixel_symbol_list=$1

  pushd ${GKI_KERNEL_DIR}/ >/dev/null
    git diff --name-only ${GKI_KERNEL_REMOTE}/${GKI_KERNEL_BRANCH}..HEAD | grep -v "\<${pixel_symbol_list}\>"
    err=$?
    if [ "${err}" = "0" ]; then
      # Found other files beside the pixel symbol list
      ABI_XML_UPDATE_REQUIRED=1
      popd >/dev/null
      return
    fi

    local added_symbols=$(git diff ${GKI_KERNEL_REMOTE}/${GKI_KERNEL_BRANCH}..HEAD "${pixel_symbol_list}" \
      | sed -n 's/^+  \([a-zA-Z_0-9]\+\)/\1/p')
    for s in ${added_symbols}; do
      grep "^  $s\>" --exclude=abi_gki_aarch64.xml \
        --exclude="${pixel_symbol_list}" android/abi_gki_aarch64*
      err=$?
      if [ "${err}" = "1" ]; then
        # Didn't find this symbol in any other symbol list
        ABI_XML_UPDATE_REQUIRED=1
        popd >/dev/null
        return
      fi
    done
  popd >/dev/null
}

export SKIP_MRPROPER=1
export BASE_OUT=${OUT_DIR:-out}/mixed/
export DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/}
VMLINUX_TMP=${BASE_OUT}/device-kernel/private/vmlinux
# Use mktemp -u to create a random branch name
FOR_AOSP_PUSH_BRANCH="update_symbol_list-delete-after-push"
BUILD_AOSP_KERNEL=${BUILD_AOSP_KERNEL:-1}
if [ -z "${GKI_KERNEL_DIR}" ];then
  GKI_KERNEL_DIR="aosp"
fi
if [ -z "${GKI_KERNEL_REMOTE}" ];then
  GKI_KERNEL_REMOTE="aosp"
fi
if [ -z "${GKI_KERNEL_BRANCH}" ];then
  GKI_KERNEL_BRANCH="android13-5.10"
fi
if [ -z "${BUILD_SCRIPT}" ];then
  BUILD_SCRIPT="./build_slider.sh"
fi
PREPARE_AOSP_ABI=${PREPARE_AOSP_ABI:-0}
CONTINUE_AFTER_REBASE=0
CHANGE_ID=
ABI_XML_UPDATE_REQUIRED=0

ARGS=()
while [[ $# -gt 0 ]]; do
  next="$1"
  case ${next} in
    -p|--prepare-aosp-abi)
      PREPARE_AOSP_ABI=1
      BUG="$2"
      if ! [[ "${BUG}" =~ ^[0-9]+$ ]]; then
        exit_if_error 1 "Bug numbers should be digits."
      fi
      shift
      ;;
    -c|--continue)
      CONTINUE_AFTER_REBASE=1
      ;;
    --change-id)
      CHANGE_ID="$2"
      if ! [[ "${CHANGE_ID}" =~ ^I[0-9a-f]{40}$ ]]; then
        exit_if_error 1 \
          "Invalid Change-Id. Make sure it starts with 'I' followed by 40 hex characters"
      fi
      shift
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

if [ "${CONTINUE_AFTER_REBASE}" = "1" -a "${PREPARE_AOSP_ABI}" = "0" ]; then
  echo "ERROR: --prepare-aosp-abi is required if --continue is set" >&2
  usage
  exit 1
fi

# Verify the aosp tree is in a good state before compiling anything
verify_aosp_tree

if [ "${CONTINUE_AFTER_REBASE}" = "0" ]; then
  if [ "${BUILD_AOSP_KERNEL}" = "1" ]; then
    BUILD_AOSP_KERNEL=1 TRIM_NONLISTED_KMI=0 ENABLE_STRICT_KMI=0 ${BUILD_SCRIPT} "$@"
  elif [ "${BUILD_STAGING_KERNEL}" = "1" ]; then
    BUILD_STAGING_KERNEL=1 TRIM_NONLISTED_KMI=0 ENABLE_STRICT_KMI=0 ${BUILD_SCRIPT} "$@"
  fi
  exit_if_error $? "Failed to run ${BUILD_SCRIPT}!"
fi

if [ "${PREPARE_AOSP_ABI}" != "0" ]; then
  update_aosp_abi "$@"
else
  extract_pixel_symbols 1 "private/gs-google/android/abi_gki_aarch64_generic"
  merge_and_sort_symbol_lists "${GKI_KERNEL_DIR}/android/abi_gki_aarch64_generic" \
    "private/gs-google/android/abi_gki_aarch64_generic"

  echo "========================================================"
  echo " The symbol list has been updated locally in ${GKI_KERNEL_DIR}/ and private/gs-google."
  echo " Compiling with BUILD_AOSP_KERNEL=1 or BUILD_STAGING_KERNEL=1 is now required until"
  echo " the new symbol(s) are merged. Re-compile using the below command:"
  echo
  if [ "${BUILD_AOSP_KERNEL}" = "1" ]; then
    echo " SKIP_MRPROPER=1 BUILD_AOSP_KERNEL=1 ${BUILD_SCRIPT}"
  elif [ "${BUILD_STAGING_KERNEL}" = "1" ]; then
    echo " SKIP_MRPROPER=1 BUILD_STAGING_KERNEL=1 ${BUILD_SCRIPT}"
  fi
  echo
fi
