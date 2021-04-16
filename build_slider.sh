#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

if [ -n "${BUILD_CONFIG}" ]; then
  echo "ERROR: setting BUILD_CONFIG is not supported for $0" >&2
  echo "usage: $0"
  echo
  echo "See build.sh for supported configs."
  exit 1
fi

function copy_gki_prebuilts {
  DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/}
  GKI_PREBUILTS_DIR=${GKI_PREBUILTS_DIR:-$(pwd)/prebuilts/boot-artifacts/kernel}

  mkdir -p ${DIST_DIR}
  echo "Copying GKI prebuilts from ${GKI_PREBUILTS_DIR} to ${DIST_DIR}."
  cp ${GKI_PREBUILTS_DIR}/* ${DIST_DIR}/
}

function build_gki {
  if [ "${EXPERIMENTAL_BUILD}" = "1" ]; then
    KERNEL_OUT_DIR=android12-5.10-staging
    KERNEL_BUILD_CONFIG=common/build.config.gki.aarch64
    # The -staging branch does not trim, so cannot have strict KMI.
    ENABLE_STRICT_KMI=0
  else
    KERNEL_OUT_DIR=android12-5.10
    KERNEL_BUILD_CONFIG=aosp/build.config.gki.aarch64
  fi

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
}

export MIXED_BUILD=1
# build with LTO=thin by default
export LTO=${LTO:-thin}

BASE_OUT=${OUT_DIR:-out}/mixed/
BUILD_SCRIPT="build/build.sh"
if [ "${BUILD_ABI}" = "1" ]; then
  BUILD_SCRIPT="build/build_abi.sh --update"
  # ABI update always needs to use full LTO
  export LTO=full
  export MIXED_BUILD=
fi

if [ -z "${BUILD_KERNEL}" ]; then
  if [ "${EXPERIMENTAL_BUILD}" = "1" -o -n "${GKI_DEFCONFIG_FRAGMENT}" ]; then
    BUILD_KERNEL=1
  else
    BUILD_KERNEL=0
  fi
fi

if [ "${LTO}" = "none" ]; then
  if [ "${BUILD_KERNEL}" != "1" -a "${EXPERIMENTAL_BUILD}" != "1" ]; then
    echo "LTO=none is only supported with BUILD_KERNEL=1 or EXPERIMENTAL_BUILD=1"
    exit 1
  fi
  ENABLE_STRICT_KMI=0
else
  ENABLE_STRICT_KMI=1
fi

if [ "${BUILD_ABI}" != "1" ]; then
  if [ "${BUILD_KERNEL}" = "0" -a "${EXPERIMENTAL_BUILD}" = "1" ]; then
    echo "BUILD_KERNEL=0 is incompatible with EXPERIMENTAL_BUILD=1."
    exit 1
  elif [ "${BUILD_KERNEL}" = "1" ]; then
    build_gki
  else
    copy_gki_prebuilts
  fi
fi

DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/}

if [ "${BUILD_ABI}" = "1" -o "${EXPERIMENTAL_BUILD}" = "1" ]; then
  GKI_BINARIES_DIR=
else
  GKI_BINARIES_DIR=$(readlink -m ${DIST_DIR})
fi
# build the whitefin/slider Kernel
BUILD_CONFIG=private/gs-google/build.config.slider \
  OUT_DIR=${BASE_OUT}/device-kernel/ \
  DIST_DIR=${DIST_DIR} \
  KBUILD_MIXED_TREE=${GKI_BINARIES_DIR} \
  ${BUILD_SCRIPT} KCFLAGS=-Werror "$@"
error_code=$?
if [ $error_code -ne 0 ]; then
  echo "ERROR: Failed to compile device-kernel. (ret=$error_code)" >&2
  exit "$error_code"
fi

if [ "${BUILD_ABI}" = "1" ]; then
  # Strip the core ABI symbols from the pixel symbol list
  grep "^ " private/gs-google/android/abi_gki_aarch64_core | while read l; do
    sed -i "/\<$l\>/d" private/gs-google/android/abi_gki_aarch64_generic
  done
fi
