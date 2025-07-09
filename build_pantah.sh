#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

parameters=
if [ "${BUILD_AOSP_KERNEL}" = "1" ]; then
  echo "WARNING: BUILD_AOSP_KERNEL is deprecated." \
    "Use --kernel_package=@//aosp instead." >&2
  parameters="--kernel_package=@//aosp"
fi

if [ "${BUILD_STAGING_KERNEL}" = "1" ]; then
  echo "WARNING: BUILD_STAGING_KERNEL is deprecated." \
    "Use --kernel_package=@//aosp-staging instead." >&2
  parameters="--kernel_package=@//aosp-staging"
fi

exec tools/bazel run \
    ${parameters} \
    --config=stamp \
    --config=pantah \
    //private/devices/google/pantah:gs201_pantah_dist "$@"
