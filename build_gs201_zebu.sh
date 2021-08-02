#!/bin/sh
# SPDX-License-Identifier: GPL-2.0

if [ -n "${BUILD_CONFIG}" ]; then
  echo "ERROR: setting BUILD_CONFIG is not supported for $0" >&2
  echo "usage: $0"
  echo
  echo "See build.sh for supported configs."
  exit 1
fi

export MIXED_BUILD=1
export SKIP_RM_DIST_DIR=1

BASE_OUT=${OUT_DIR:-out}/mixed/
export OUT_DIR

# share a common DIST_DIR
export DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/}

OUT_DIR=${BASE_OUT}/android13-5.10-staging/
# Now build the GKI kernel
SKIP_CP_KERNEL_HDR=1 \
  BUILD_CONFIG=common/build.config.gki.aarch64 \
  build/build.sh KCFLAGS=-Werror "$@"
error_code=$?
if [ $error_code -ne 0 ]; then
  echo "ERROR: Failed to compile android13-5.10-staging. (ret=$error_code)" >&2
  exit "$error_code"
fi

OUT_DIR=${BASE_OUT}/device-kernel/
# build the zebu Kernel
BUILD_CONFIG=private/gs-google/build.config.gs201_hybrid \
  build/build.sh KCFLAGS=-Werror "$@"

export UNPACK_BOOTIMG_PATH="tools/mkbootimg/unpack_bootimg.py"

mkdir -p "${DIST_DIR}/zebu/"
python "$UNPACK_BOOTIMG_PATH" --boot_img "${DIST_DIR}/boot.img" --out "${DIST_DIR}/ext_bootimg"
python "$UNPACK_BOOTIMG_PATH" --boot_img "${DIST_DIR}/vendor_boot.img" --out "${DIST_DIR}/ext_vendor_bootimg"
cat "${DIST_DIR}/ext_vendor_bootimg/vendor_ramdisk" "${DIST_DIR}/ext_bootimg/ramdisk" > \
  "${DIST_DIR}/zebu/buildroot_v201105_r03_ttySAC0_debug.cramfs"
#cp -v "${DIST_DIR}/ext_bootimg/kernel" "${DIST_DIR}/zebu/Image"
cp -v "${OUT_DIR}/private/gs-google/arch/arm64/boot/Image" "${DIST_DIR}/zebu/Image"
#cp -v "${DIST_DIR}/ext_vendor_bootimg/dtb" "${DIST_DIR}/zebu/devicetree.dtb"
cp -v "${DIST_DIR}/gs201-out.dtb" "${DIST_DIR}/zebu/exynos9855-emulator9855.dtb"

