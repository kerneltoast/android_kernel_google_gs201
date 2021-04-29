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

if [ -n "${BUILD_ABI}" ]; then
  echo "The ABI update workflow has changed. Please read go/gki-p21-workflow"
  echo "  for instructions on updating ABI/symbol list."
  exit_if_error 1 "BUILD_ABI is deprecated"
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


if [ "${EXPERIMENTAL_BUILD}" = "0" -a "${BUILD_KERNEL}" != "0" ]; then
  pushd aosp/ > /dev/null
    # Booting AOSP ToT does not always work; throw a warning to prevent this.
    REPO_SHA=$(git log -1 --pretty="format:%H" m/s-dev-gs-pixel-5.10)
    LOCAL_MERGE_BASE=$(git merge-base HEAD aosp/android12-5.10)
    if [ "${REPO_SHA}" != "${LOCAL_MERGE_BASE}" ]; then
      echo "Your aosp/ directory appears to be synced to a point beyond the"
      echo "  latest AOSP merge point. This is not supported, currently, as"
      echo "  it is prone to errors. Please base any changes on"
      echo "  m/s-dev-gs-pixel-5.10."
      exit_if_error 1 "aosp/ is not based on m/s-dev-gs-pixel-5.10"
    fi
  popd > /dev/null
fi

if [ "${BUILD_KERNEL}" != "0" -o "${EXPERIMENTAL_BUILD}" != "0" ]; then
  build_gki
else
  copy_gki_prebuilts
fi
build_pixel
