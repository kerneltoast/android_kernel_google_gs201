#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
BUILD_AOSP_KERNEL=1 \
BUILD_STAGING_KERNEL=0 \
BUILD_SCRIPT="./build_cloudripper.sh" \
DEVICE_KERNEL_BUILD_CONFIG=private/gs-google/build.config.cloudripper \
private/gs-google/update_symbol_list.sh "$@"
