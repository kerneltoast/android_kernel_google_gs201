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
GKI_FILES=(
  System.map
  vmlinux
  vmlinux.symvers
  modules.builtin
  modules.builtin.modinfo
  boot-lz4.img
  Image.lz4
)
GKI_PREBUILTS_DIR=${2:-${CUR_DIR}/prebuilts/boot-artifacts/kernel/}
GKI_BUILD=$1
ALLOW_PREBUILTS_MISMATCH=${ALLOW_PREBUILTS_MISMATCH:-0}
USE_UNSIGNED_USER_IMG=${USE_UNSIGNED_USER_IMG:-0}
BASE_TARGET="kernel_aarch64"
FETCH_ARTIFACT_CMD="/google/data/ro/projects/android/fetch_artifact"
IMG_TAR_FILE="certified-boot-img-${GKI_BUILD}.tar.gz"

if [ -z "${GKI_BUILD}" ]; then
  echo "No GKI build number provided."
  exit 1
fi

mkdir -p ${GKI_PREBUILTS_DIR}
TEMP_DIR=$(mktemp -d)

cd ${TEMP_DIR}
echo "Downloading GKI binaries from build ab/${GKI_BUILD} via fetch_artifact..."
if [ "${USE_UNSIGNED_USER_IMG}" = "0" ]; then
  file="signed/${IMG_TAR_FILE}"
  echo "Downloading signed boot.img..."
  ${FETCH_ARTIFACT_CMD} \
      --bid ${GKI_BUILD} \
      --target ${BASE_TARGET} ${file}
  exit_and_clean_if_error $? "Unable to download signed boot image"
fi

echo "Downloading prebuilts..."
for file in "${GKI_FILES[@]}"; do
  if grep -q "boot.*\.img" <<< ${file} ; then
    BOOT_IMG_NAME=${file}
  fi
  ${FETCH_ARTIFACT_CMD} \
      --bid ${GKI_BUILD} \
      --target ${BASE_TARGET} ${file}
  exit_and_clean_if_error $? "Error downloading ${file}"
done

if [ "${USE_UNSIGNED_USER_IMG}" = "0" ]; then
  tar -zxvf ${IMG_TAR_FILE}
  exit_and_clean_if_error $? "Failed to extract ${IMG_TAR_FILE}"
fi

if [ -f "vmlinux" ]; then
  SHA_FILE="vmlinux"
else
  exit_and_clean_if_error 1 "No vmlinux downloaded"
fi

echo "Checking if GKI binaries match the current aosp/ revision..."
PREBUILTS_SHA=$(strings ${SHA_FILE} | grep "Linux version [0-9]\+\.[0-9]\+" |
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

echo "Copying GKI files to ${GKI_PREBUILTS_DIR}..."
mv -v ${GKI_FILES[@]} ${GKI_PREBUILTS_DIR}
exit_and_clean_if_error $? "Unable to copy all files"

cd ${GKI_PREBUILTS_DIR}
if [ -n "${BOOT_IMG_NAME}" -a -f "${BOOT_IMG_NAME}" ]; then
  mv ${BOOT_IMG_NAME} boot.img
  echo "Copied ${BOOT_IMG_NAME} to boot.img."
fi
echo "Update the GKI binaries to ab/${GKI_BUILD}

Update the GKI binaries based on the given build. The prebuilts now have
the following SHA, taken from the ${SHA_FILE} banner: ${PREBUILTS_SHA}
" > ${TEMP_DIR}/commit_body
git add *
git commit -s -F ${TEMP_DIR}/commit_body

cd ${CUR_DIR}
rm -rf ${TEMP_DIR}

echo "Done."
