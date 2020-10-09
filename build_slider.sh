#!/bin/sh
# SPDX-License-Identifier: GPL-2.0

export MIXED_BUILD=1

export OUT_DIR DIST_DIR

BASE_OUT=out/mixed/

# share a common DIST_DIR
DIST_DIR=${BASE_OUT}/dist/

OUT_DIR=${BASE_OUT}/android12-5.10-staging/
# Now build the GKI kernel
SKIP_CP_KERNEL_HDR=1 \
  BUILD_CONFIG=common/build.config.gki.aarch64 \
  build/build.sh "$@"

OUT_DIR=${BASE_OUT}/device-kernel/
# build the whitefin/slider Kernel
BUILD_CONFIG=private/gs-google/build.config.slider \
  build/build.sh "$@"

