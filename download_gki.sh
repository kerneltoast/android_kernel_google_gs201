#!/bin/bash

function exit_and_clean_if_error {
  if [ $1 -ne 0 ]; then
    echo "ERROR: $2: retval=$1" >&2
    cd ${CUR_DIR}
    rm -rf ${TEMP_DIR}
    exit $1
  fi
}

CUR_DIR=$(pwd)
FILES_LIST=${CUR_DIR}/private/gs-google/GKI-files
GKI_PREBUILTS_DIR=${2:-${CUR_DIR}/prebuilts/boot-artifacts/kernel/}
GKI_BUILD=$1
ALLOW_PREBUILTS_MISMATCH=${ALLOW_PREBUILTS_MISMATCH:-0}

if [ -z "${GKI_BUILD}" ]; then
  echo "No GKI build number provided."
  exit 1
fi

mkdir -p ${GKI_PREBUILTS_DIR}
TEMP_DIR=$(mktemp -d)

cd ${TEMP_DIR}
echo "Downloading GKI binaries from build ab/${GKI_BUILD}..."
${CUR_DIR}/build/gki/download_from_ci ${GKI_BUILD} kernel_aarch64
exit_and_clean_if_error $? "Could not download prebuilts"

echo "Checking if GKI binaries match the current aosp/ revision..."
PREBUILTS_SHA=$(strings vmlinux | grep "Linux version 5.10" |
		    sed -n "s/^.*-g\([0-9a-fA-F]\{12\}\)-.*/\1/p")
MANIFEST_SHA=$(cat ${CUR_DIR}/.repo/manifests/default.xml |
		   grep "path=\"aosp\"" |
		   sed -n "s/^.*revision=\"\([0-9a-fA-F]\{12\}\).*/\1/p")
if [ "${PREBUILTS_SHA}" != "${MANIFEST_SHA}" -a \
     "${ALLOW_PREBUILTS_MISMATCH}" = "0" ]; then
    echo "The downloaded prebuilts do not match the manifest's SHA! Please"
    echo "  check the GKI_BUILD provided, or use ALLOW_PREBUILTS_MISMATCH=1"
    echo "  if this difference is expected."
    echo "  PREBUILTS_SHA=${PREBUILTS_SHA}"
    echo "   MANIFEST_SHA=${MANIFEST_SHA}"
    exit_and_clean_if_error 1 "Mismatch between manifest and prebuilts"
fi

echo "Copying files listed in ${FILES_LIST} to ${GKI_PREBUILTS_DIR}..."
mv $(cat ${FILES_LIST}) ${GKI_PREBUILTS_DIR}
exit_and_clean_if_error $? "Unable to copy all files"

cd ${GKI_PREBUILTS_DIR}
echo "Update the GKI binaries to ab/${GKI_BUILD}

Update the GKI binaries based on the given build.
" > ${TEMP_DIR}\commit_body
git add *
git commit -s -F ${TEMP_DIR}\commit_body

cd ${CUR_DIR}
rm -rf ${TEMP_DIR}

echo "Done."
