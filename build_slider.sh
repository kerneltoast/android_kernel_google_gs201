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
CHECK_DIRTY_AOSP=0
if [ -z "${BUILD_KERNEL}" ]; then
  if [ "${EXPERIMENTAL_BUILD}" != "0" -o -n "${GKI_DEFCONFIG_FRAGMENT}" ]; then
    BUILD_KERNEL=1
  else
    CHECK_DIRTY_AOSP=1
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
  exit_if_error 1 "BUILD_CONFIG is not supported for $0"
fi

if [ "${BUILD_KERNEL}" = "0" ]; then
  if [ "${EXPERIMENTAL_BUILD}" != "0" -o -n "${GKI_DEFCONFIG_FRAGMENT}" ]; then
    echo "BUILD_KERNEL=0 is incompatible with EXPERIMENTAL_BUILD=1 and"
    echo "  GKI_DEFCONFIG_FRAGMENT."
    exit_if_error 1 "Flags incompatible with BUILD_KERNEL=0 detected"
  elif [ "${LTO}" = "none" ]; then
    echo "LTO=none requires BUILD_KERNEL=1, EXPERIMENTAL_BUILD=1, or"
    echo "  GKI_DEFCONFIG_FRAGMENT to be set."
    exit_if_error 1 "LTO=none requires building the kernel"
  fi
fi

if [ "${EXPERIMENTAL_BUILD}" = "0" -a "${BUILD_KERNEL}" != "0" ]; then
  pushd aosp/ > /dev/null
    # Booting AOSP ToT does not always work; throw a warning to prevent this.
    REPO_SHA=$(git log -1 --pretty="format:%H" m/s-dev-gs-pixel-5.10)
    LOCAL_MERGE_BASE=$(git merge-base HEAD aosp/android12-5.10)
    if [ -n "${LOCAL_MERGE_BASE}" -a "${REPO_SHA}" != "${LOCAL_MERGE_BASE}" ]; then
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

# If BUILD_KERNEL is not explicitly set, be sure that there are no aosp/
# changes not present in the prebuilt.
if [ "${CHECK_DIRTY_AOSP}" != "0" ]; then
  PREBUILTS_SHA=$(strings ${GKI_PREBUILTS_DIR}/vmlinux |
		      grep "Linux version 5.10" |
		      sed -n "s/^.*-g\([0-9a-f]\{12\}\)-.*/\1/p")
  pushd aosp/ > /dev/null
    # The AOSP sha can sometimes be longer than 12 characters; fix its length.
    AOSP_SHA=$(git log -1 --abbrev=12 --pretty="format:%h")
    if [ "${PREBUILTS_SHA}" != "${AOSP_SHA}" -o -n \
         "$(git --no-optional-locks status -uno --porcelain ||
            git diff-index --name-only HEAD)" ]; then
      echo "WARNING: There are aosp/ changes which are not in the prebuilts."
      echo "  Because you did not specify BUILD_KERNEL=0 or 1, $0"
      echo "  defaulted to building with the prebuilts. Please be aware that"
      echo "  your changes to aosp/ will not be present in the final images. If"
      echo "  you have made changes to aosp/, it is recommended to explicitly"
      echo "  set BUILD_KERNEL=0 if you wish to use the prebuilts, or to 1 if"
      echo "  you wish to build any local changes you may have."
    fi
  popd > /dev/null
fi
