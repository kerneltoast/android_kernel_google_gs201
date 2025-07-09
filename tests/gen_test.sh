#!/bin/bash
# Copyright (C) 2016 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# We want to generate two device tree blob (.dtb) files by combining
# the "base" and "add" device tree source (.dts) files in two
# different ways.
#
# 1) Use /include/ and compile via dtc to make the "gold standard"
#
# 2) Compile them separately dtc, and join them with the
#    ufdt_apply_overlay program
#
# Then, compare 1) and 2) with dts_diff (diff merged nodes) and return 0
# iff they are identical.

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
source ${SCRIPT_DIR}/common.sh

on_exit() {
  rm -rf "$TEMP_DIR"
}

# Constants
IN_DATA_DIR="testdata"

TEMP_DIR=`mktemp -d`
# The script will exit directly if any command fails.
set -e
trap on_exit EXIT

# Global variables
TESTCASE_NAME=$1
BASE_DTS="$IN_DATA_DIR/${TESTCASE_NAME}-base.dts"
OVERLAY_DTS="$IN_DATA_DIR/${TESTCASE_NAME}-overlay.dts"
REF_MERGED_DTS="$TEMP_DIR/${TESTCASE_NAME}-ref-merged.dts"
OVL_MERGED_DTS="$TEMP_DIR/${TESTCASE_NAME}-ovl-merged.dts"

#
# Complie and diff
#
$SCRIPT_DIR/apply_overlay.sh --fdt "$BASE_DTS" "$OVERLAY_DTS" "$REF_MERGED_DTS"
$SCRIPT_DIR/apply_overlay.sh --ufdt "$BASE_DTS" "$OVERLAY_DTS" "$OVL_MERGED_DTS"
dts_diff "$REF_MERGED_DTS" "$OVL_MERGED_DTS"
