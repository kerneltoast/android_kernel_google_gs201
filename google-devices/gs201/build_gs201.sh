#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only

exec tools/bazel run \
    --config=stamp \
    --config=gs201 \
    //private/devices/google/gs201:dist "$@"
