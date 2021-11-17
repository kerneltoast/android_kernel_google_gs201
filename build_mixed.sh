#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

# build_mixed.sh takes as input a GKI source tree or GKI prebuilts as
#   well as a device kernel source tree, compiles them, and then combines
#   them into a set of flashable images to boot a GKI build.
#
# Usage:
#   To build the GKI kernel:
#     GKI_KERNEL_BUILD_CONFIG=path/to/gki/config \
#       DEVICE_KERNEL_BUILD_CONFIG=path/to/device/config \
#       build_mixed.sh
#
#   To use GKI prebuilts:
#      GKI_KERNEL_PREBUILTS_DIR=path/to/gki/prebuilts \
#        DEVICE_KERNEL_BUILD_CONFIG=path/to/device/config \
#        build_mixed.sh
#
#   Note that you must always set one of either GKI_KERNEL_PREBUILTS_DIR or
#   GKI_KERNEL_BUILD_CONFIG.
#
# The following environment variables are considered during execution:
#
#   DEVICE_KERNEL_BUILD_CONFIG
#     The build config for the device kernel to be passed into build.sh. This
#     config is always required to be set.
#
#   GKI_KERNEL_BUILD_CONFIG
#     The build config for the GKI kernel to be passed into build.sh. This
#     config is required if you are building the GKI kernel, and is incompatible
#     with GKI_KERNEL_PREBUILTS_DIR.
#
#   GKI_DEFCONFIG_FRAGMENT
#     An optional path to an additional build.config fragment. Only works if
#     your base build config supports GKI_DEFCONFIG_FRAGMENT, such as
#     build.config.gki.aarch64. This is incompatible with GKI_KERNEL_PREBUILTS_DIR.
#
#   GKI_KERNEL_PREBUILTS_DIR
#     A path to a set of GKI prebuilts. These prebuilts can be used to skip the
#     compilation of the GKI kernel. Incompatible with GKI_KERNEL_BUILD_CONFIG.
#
#     The following must be found in this directory to work:
#       vmlinux
#       System.map
#       vmlinux.symvers
#       modules.builtin
#       modules.builtin.modinfo
#       Image.lz4

function print_usage {
cat << EOF
$0 takes as input a GKI source tree or GKI prebuilts as
  well as a device kernel source tree, compiles them, and then combines
  them into a set of flashable images to boot a GKI build.

Example usage of $0:
  To build the GKI kernel:
    GKI_KERNEL_BUILD_CONFIG=path/to/gki/config \\
      DEVICE_KERNEL_BUILD_CONFIG=path/to/device/config \\
      $0

  To use GKI prebuilts:
    GKI_KERNEL_PREBUILTS_DIR=path/to/gki/prebuilts \\
      DEVICE_KERNEL_BUILD_CONFIG=path/to/device/config \\
      $0

EOF
}

function exit_if_error {
  if [ $1 -ne 0 ]; then
    echo "ERROR: $2: retval=$1" >&2
    exit $1
  fi
}

function copy_gki_prebuilts {
  mkdir -p ${DIST_DIR}
  echo "Copying GKI prebuilts from ${GKI_KERNEL_PREBUILTS_DIR} to ${DIST_DIR}."
  cp ${GKI_KERNEL_PREBUILTS_DIR}/* ${DIST_DIR}/
}

function build_gki {
  echo "Building GKI kernel using ${GKI_KERNEL_BUILD_CONFIG}..."
  BUILD_CONFIG=${GKI_KERNEL_BUILD_CONFIG} \
    OUT_DIR=${BASE_OUT}/${GKI_KERNEL_OUT_DIR}/ \
    DIST_DIR=${DIST_DIR} \
    SKIP_CP_KERNEL_HDR=1 \
    build/build.sh KCFLAGS=-Werror "$@"
  exit_if_error $? "Failed to compile GKI kernel"
}

function build_device_kernel {
  echo "Building device kernel using ${DEVICE_KERNEL_BUILD_CONFIG}..."
  BUILD_CONFIG=${DEVICE_KERNEL_BUILD_CONFIG} \
    OUT_DIR=${BASE_OUT}/${DEVICE_KERNEL_OUT_DIR}/ \
    DIST_DIR=${DIST_DIR} \
    MIXED_BUILD=1 \
    KBUILD_MIXED_TREE=${GKI_BINARIES_DIR} \
    build/build.sh KCFLAGS=-Werror "$@"
  exit_if_error $? "Failed to compile device kernel"
}

BASE_OUT=${OUT_DIR:-out}/mixed/
DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/}
GKI_KERNEL_OUT_DIR=${GKI_KERNEL_OUT_DIR:-"gki-kernel"}
DEVICE_KERNEL_OUT_DIR=${DEVICE_KERNEL_OUT_DIR:-"device-kernel"}
GKI_BINARIES_DIR=$(readlink -m ${DIST_DIR})

if [ -n "${GKI_KERNEL_PREBUILTS_DIR}" ]; then
  GKI_KERNEL_PREBUILTS_DIR=$(readlink -m "${GKI_KERNEL_PREBUILTS_DIR}")
fi

if [ -n "${BUILD_CONFIG}" ]; then
  print_usage
  exit_if_error 1 "BUILD_CONFIG is not supported for $0"
fi

if [ -z "${DEVICE_KERNEL_BUILD_CONFIG}" ]; then
  print_usage
  exit_if_error 1 "No DEVICE_KERNEL_BUILD_CONFIG set"
fi

if [ -n "${GKI_KERNEL_PREBUILTS_DIR}" ]; then
  if [ -n "${GKI_KERNEL_BUILD_CONFIG}" ]; then
    print_usage
    exit_if_error 1 "Flags incompatible with GKI_KERNEL_PREBUILTS_DIR detected"
  elif [ ! -d ${GKI_KERNEL_PREBUILTS_DIR} ]; then
    exit_if_error 1 "${GKI_KERNEL_PREBUILTS_DIR} does not exist"
  fi
elif [ -z "${GKI_KERNEL_BUILD_CONFIG}" ]; then
  print_usage
  exit_if_error 1 "Must set GKI_KERNEL_PREBUILTS_DIR or GKI_KERNEL_BUILD_CONFIG"
fi

if [ -n "${GKI_KERNEL_BUILD_CONFIG}" ]; then
  build_gki "$@"
else
  copy_gki_prebuilts
fi

build_device_kernel "$@"
