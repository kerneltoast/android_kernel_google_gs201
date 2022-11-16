#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
BUILD_AOSP_KERNEL=1 \
BUILD_STAGING_KERNEL=0 \
BUILD_SCRIPT="./build_slider.sh" \
DEVICE_KERNEL_BUILD_CONFIG=private/gs-google/build.config.slider \
private/gs-google/update_symbol_list.sh "$@"
