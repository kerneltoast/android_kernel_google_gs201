#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

exec tools/bazel run \
    --config=stamp \
    --config=lynx \
    //private/devices/google/lynx:gs201_lynx_dist "$@"
