#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

function exit_if_error {
  if [ $1 -ne 0 ]; then
    echo "ERROR: $2: retval=$1" >&2
    exit $1
  fi
}

function copy_gki_prebuilts {
  mkdir -p ${DIST_DIR}
  echo "Copying GKI prebuilts from ${GKI_PREBUILTS_DIR} to ${DIST_DIR}."
  cp ${GKI_PREBUILTS_DIR}/* ${DIST_DIR}/
}

function build_gki {
  echo "Building GKI kernel using ${KERNEL_BUILD_CONFIG}..."
  SKIP_CP_KERNEL_HDR=1 \
    OUT_DIR=${BASE_OUT}/${KERNEL_OUT_DIR} \
    DIST_DIR=${DIST_DIR} \
    LTO=${LTO} \
    KMI_SYMBOL_LIST_STRICT_MODE=${ENABLE_STRICT_KMI} \
    TRIM_NONLISTED_KMI=${TRIM_NONLISTED_KMI} \
    BUILD_CONFIG=${KERNEL_BUILD_CONFIG} \
    build/build.sh KCFLAGS=-Werror "$@"
  exit_if_error $? "Failed to compile ${KERNEL_OUT_DIR}"
}

function build_abi {
  echo "Building device kernel for ABI..."
  # LTO=full is required for building ABI, and we must build the full kernel.
  BUILD_CONFIG=${DEVICE_KERNEL_BUILD_CONFIG} \
    OUT_DIR=${BASE_OUT}/device-kernel/ \
    DIST_DIR=${DIST_DIR} \
    LTO=full \
    MIXED_BUILD= \
    KBUILD_MIXED_TREE= \
    build/build_abi.sh --update KCFLAGS=-Werror "$@"
  exit_if_error $? "Failed to compile device kernel"

  # Strip symbols from the _core symbol list
  grep "^ " private/gs-google/android/abi_gki_aarch64_core | while read l; do
    sed -i "/\<$l\>/d" private/gs-google/android/abi_gki_aarch64_generic
  done
}

function build_pixel {
  echo "Building device kernel..."
  BUILD_CONFIG=${DEVICE_KERNEL_BUILD_CONFIG} \
    OUT_DIR=${BASE_OUT}/device-kernel/ \
    DIST_DIR=${DIST_DIR} \
    LTO=${LTO} \
    MIXED_BUILD=1 \
    KBUILD_MIXED_TREE=${GKI_BINARIES_DIR} \
    KMI_SYMBOL_LIST_STRICT_MODE=${ENABLE_STRICT_KMI} \
    TRIM_NONLISTED_KMI=${TRIM_NONLISTED_KMI} \
    build/build.sh KCFLAGS=-Werror "$@"
  exit_if_error $? "Failed to compile device kernel"
}

EXPERIMENTAL_BUILD=${EXPERIMENTAL_BUILD:-0}
BUILD_ABI=${BUILD_ABI:-0}
BASE_OUT=${OUT_DIR:-out}/mixed/
DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/}
LTO=${LTO:-thin}
GKI_BINARIES_DIR=$(readlink -m ${DIST_DIR})
GKI_PREBUILTS_DIR=$(readlink -m "prebuilts/boot-artifacts/kernel/")
DEFAULT_CONFIG="private/gs-google/build.config.slider"
DEVICE_KERNEL_BUILD_CONFIG=${DEVICE_KERNEL_BUILD_CONFIG:-${DEFAULT_CONFIG}}
TRIM_NONLISTED_KMI=${TRIM_NONLISTED_KMI:-1}
if [ -z "${BUILD_KERNEL}" ]; then
  if [ "${EXPERIMENTAL_BUILD}" != "0" -o -n "${GKI_DEFCONFIG_FRAGMENT}" ]; then
    BUILD_KERNEL=1
  else
    BUILD_KERNEL=0
  fi
fi

if [ "${LTO}" = "none" ]; then
  ENABLE_STRICT_KMI=0
else
  ENABLE_STRICT_KMI=${ENABLE_STRICT_KMI:-1}
fi

if [ "${EXPERIMENTAL_BUILD}" != "0" ]; then
  KERNEL_OUT_DIR=android12-5.10-staging
  KERNEL_BUILD_CONFIG=common/build.config.gki.aarch64
else
  KERNEL_OUT_DIR=android12-5.10
  KERNEL_BUILD_CONFIG=aosp/build.config.gki.aarch64
fi

if [ -n "${BUILD_CONFIG}" ]; then
  err_msg="setting BUILD_CONFIG is not supported for $0
  usage: $0
  See build.sh for supported configs"
  exit_if_error 1 "${err_msg}"
fi

if [ "${BUILD_KERNEL}" = "0" -a "${EXPERIMENTAL_BUILD}" != "0" ]; then
  exit_if_error 1 "BUILD_KERNEL=0 is incompatible with EXPERIMENTAL_BUILD"
fi

if [ "${LTO}" = "none" -a "${BUILD_KERNEL}" = "0" ]; then
  exit_if_error 1 "LTO=none requires BUILD_KERNEL=1 or EXPERIMENTAL_BUILD=1"
fi


if [ "${BUILD_ABI}" != "0" ]; then
  build_abi
else
  if [ "${BUILD_KERNEL}" != "0" -o "${EXPERIMENTAL_BUILD}" != "0" ]; then
    build_gki
  else
    copy_gki_prebuilts
  fi
  build_pixel
fi
