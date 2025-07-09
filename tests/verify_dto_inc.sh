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

PROG_NAME=`basename $0`

function usage() {
  echo "Usage:"
  echo "  $PROG_NAME <Base DTS> <Overlay DTS> <Output DTS>"
}

function on_exit() {
  rm -rf "$TEMP_DIR"
}

#
# Start
#

if [[ $# -lt 3 ]]; then
  usage
  exit 1
fi

BASE_DTS=$1
OVERLAY_DTS=$2
OUT_DTS=$3

TEMP_DIR=`mktemp -d`
# The script will exit directly if any command fails.
set -e
trap on_exit EXIT

# Finds '/dts-v1/; and /plugin/;' then replace them with '/* REMOVED */'
OVERLAY_DTS_DIR=`dirname "$OVERLAY_DTS"`
OVERLAY_DTS_NAME=`basename "$OVERLAY_DTS"`
OVERLAY_DT_WO_HEADER_DTS="$TEMP_DIR/$OVERLAY_DTS_NAME"
sed "s/\\(\\/dts-v1\\/\\s*;\\|\\/plugin\\/\\s*;\\)/\\/\\* REMOVED \\*\\//g" \
  "$OVERLAY_DTS" > "$OVERLAY_DT_WO_HEADER_DTS"

# Appends /include/ ...;
BASE_DTS_DIR=`dirname "$BASE_DTS"`
BASE_DTS_NAME=`basename "$BASE_DTS"`
BASE_DT_WITH_INC_DTS="$TEMP_DIR/$BASE_DTS_NAME"
cp "$BASE_DTS" "$BASE_DT_WITH_INC_DTS"
echo "/include/ \"$OVERLAY_DT_WO_HEADER_DTS\"" >> "$BASE_DT_WITH_INC_DTS"

# Simulate device tree overlay
MERGED_DTB="$BASE_DT_WITH_INC_DTS.dtb"
dtc -@ -i "$BASE_DTS_DIR" -i "$OVERLAY_DTS_DIR" -O dtb -o "$MERGED_DTB" "$BASE_DT_WITH_INC_DTS"

# Dump
dtc -s -O dts -o "$OUT_DTS" "$MERGED_DTB"
