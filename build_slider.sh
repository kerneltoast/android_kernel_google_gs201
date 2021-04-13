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
BUILD_SCRIPT="build/build.sh"
if [ -n "${BUILD_ABI}" ]; then
  BUILD_SCRIPT="build/build_abi.sh --update"
  export MIXED_BUILD=
fi

if [ ${LTO} == "none" ]; then
    ENABLE_STRICT_KMI=0
else
    ENABLE_STRICT_KMI=1
fi

if [ -z "${BUILD_ABI}" ]; then
  if [ -n "${EXPERIMENTAL_BUILD}" ]; then
    KERNEL_OUT_DIR=android12-5.10-staging
    KERNEL_BUILD_CONFIG=common/build.config.gki.aarch64
  else
    KERNEL_OUT_DIR=android12-5.10
    KERNEL_BUILD_CONFIG=aosp/build.config.gki.aarch64
  fi

  # build with LTO=thin by default
  export LTO=${LTO:-thin}

  # Now build the GKI kernel
  SKIP_CP_KERNEL_HDR=1 \
    OUT_DIR=${BASE_OUT}/${KERNEL_OUT_DIR} \
    DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/} \
    KMI_SYMBOL_LIST_STRICT_MODE=${ENABLE_STRICT_KMI} \
    BUILD_CONFIG=${KERNEL_BUILD_CONFIG} \
    build/build.sh KCFLAGS=-Werror "$@"
  error_code=$?
  if [ $error_code -ne 0 ]; then
    echo "ERROR: Failed to compile ${KERNEL_OUT_DIR}. (ret=$error_code)" >&2
    exit "$error_code"
  fi
fi

# build the whitefin/slider Kernel
BUILD_CONFIG=private/gs-google/build.config.slider \
  OUT_DIR=${BASE_OUT}/device-kernel/ \
  DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/} \
  ${BUILD_SCRIPT} KCFLAGS=-Werror "$@"
error_code=$?
if [ $error_code -ne 0 ]; then
  echo "ERROR: Failed to compile device-kernel. (ret=$error_code)" >&2
  exit "$error_code"
fi

if [ -n "${BUILD_ABI}" ]; then
  # Strip the core ABI symbols from the pixel symbol list
  grep "^ " private/gs-google/android/abi_gki_aarch64_core | while read l; do
    sed -i "/\<$l\>/d" private/gs-google/android/abi_gki_aarch64_generic
  done
fi
