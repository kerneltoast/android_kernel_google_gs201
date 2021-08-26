#!/bin/sh
# SPDX-License-Identifier: GPL-2.0
BUILD_KERNEL=1 \
DEVICE_KERNEL_BUILD_CONFIG=private/gs-google/build.config.cloudripper \
private/gs-google/build_slider.sh "$@"
