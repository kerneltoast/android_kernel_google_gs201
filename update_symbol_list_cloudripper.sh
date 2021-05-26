#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
EXPERIMENTAL_BUILD=1 DEVICE_KERNEL_BUILD_CONFIG=private/gs-google/build.config.cloudripper \
private/gs-google/update_symbol_list.sh "$@"
