#!/bin/bash

CUR_DIR=$(pwd)
FILES_LIST=${CUR_DIR}/private/gs-google/GKI-files
GKI_PREBUILTS_DIR=${2:-${CUR_DIR}/prebuilts/boot-artifacts/kernel/}
GKI_BUILD=$1

if [ -z "${GKI_BUILD}" ]; then
  echo "No GKI build number provided."
  exit 1
fi

mkdir -p ${GKI_PREBUILTS_DIR}
TEMP_DIR=$(mktemp -d)

cd ${TEMP_DIR}
echo "Downloading GKI binaries from build ab/${GKI_BUILD}..."
${CUR_DIR}/build/gki/download_from_ci ${GKI_BUILD} kernel_aarch64
error_code=$?
if [ "${error_code}" -ne "0" ]; then
  echo "ERROR: Could not download prebuilts! retval=${error_code}" 1>&2
  cd ${CUR_DIR}
  rm -rf ${TEMP_DIR}
  exit 1
fi
echo "Copying files listed in ${FILES_LIST} to ${GKI_PREBUILTS_DIR}..."
mv $(cat ${FILES_LIST}) ${GKI_PREBUILTS_DIR}
error_code=$?
if [ "${error_code}" -ne "0" ]; then
  echo "ERROR: Unable to copy all files. retval=${error_code}" 1>&2
  cd ${CUR_DIR}
  rm -rf ${TEMP_DIR}
  exit 1
fi

cd ${GKI_PREBUILTS_DIR}
echo "Update the GKI binaries to ab/${GKI_BUILD}

Update the GKI binaries based on the given build.
" > ${TEMP_DIR}\commit_body
git add *
git commit -s -F ${TEMP_DIR}\commit_body

cd ${CUR_DIR}
rm -rf ${TEMP_DIR}

echo "Done."
