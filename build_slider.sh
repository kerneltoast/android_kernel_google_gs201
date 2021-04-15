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

BASE_OUT=${OUT_DIR:-out}/mixed/
export OUT_DIR

# share a common DIST_DIR
export DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/}

# build with LTO=thin by default
export LTO=${LTO:-thin}

if [ ${LTO} == "none" ]; then
    ENABLE_STRICT_KMI=0
else
    ENABLE_STRICT_KMI=1
fi

if [ -n "${EXPERIMENTAL_BUILD}" ]; then
    KERNEL_OUT_DIR=android12-5.10-staging
    KERNEL_BUILD_CONFIG=common/build.config.gki.aarch64
else
    KERNEL_OUT_DIR=android12-5.10
    KERNEL_BUILD_CONFIG=aosp/build.config.gki.aarch64
fi

OUT_DIR=${BASE_OUT}/${KERNEL_OUT_DIR}/

# Now build the GKI kernel
SKIP_CP_KERNEL_HDR=1 \
  KMI_SYMBOL_LIST_STRICT_MODE=${ENABLE_STRICT_KMI} \
  BUILD_CONFIG=${KERNEL_BUILD_CONFIG} \
  build/build.sh KCFLAGS=-Werror "$@"
error_code=$?
if [ $error_code -ne 0 ]; then
  echo "ERROR: Failed to compile android12-5.10-staging. (ret=$error_code)" >&2
  exit "$error_code"
fi

OUT_DIR=${BASE_OUT}/device-kernel/
# build the whitefin/slider Kernel
BUILD_CONFIG=private/gs-google/build.config.slider \
  build/build.sh KCFLAGS=-Werror "$@"
error_code=$?
if [ $error_code -ne 0 ]; then
  echo "ERROR: Failed to compile device-kernel. (ret=$error_code)" >&2
  exit "$error_code"
fi
