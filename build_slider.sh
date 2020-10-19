#!/bin/sh
# SPDX-License-Identifier: GPL-2.0

export MIXED_BUILD=1

BASE_OUT=${OUT_DIR:-out}/mixed/
export OUT_DIR

# share a common DIST_DIR
export DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/}

OUT_DIR=${BASE_OUT}/android12-5.10-staging/
# Now build the GKI kernel
SKIP_CP_KERNEL_HDR=1 \
  BUILD_CONFIG=common/build.config.gki.aarch64 \
  build/build.sh "$@"
error_code=$?
if [ $error_code -ne 0 ]; then
  echo "ERROR: Failed to compile android12-5.10-staging. (ret=$error_code)" >&2
  exit "$error_code"
fi

OUT_DIR=${BASE_OUT}/device-kernel/
# build the whitefin/slider Kernel
BUILD_CONFIG=private/gs-google/build.config.slider \
  build/build.sh "$@"

