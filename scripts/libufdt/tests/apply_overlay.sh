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

# Set to true to get more debug information
DEBUG=false
# Remove the comment to enable valgrind
# Require build with $mmma external/valgrind
#VALGRIND="valgrind --leak-check=yes --show-reachable=yes"

function usage() {
  echo "Usage:"
  echo "  $PROG_NAME (--fdt|--ufdt) (--remote) <Base DTS> <Overlay DTS> <Output DTS>"
}

function on_exit() {
  rm -rf "$TEMP_DIR"
}

#
# Start
#

# Setup OVERLAY
if [ "$1" == "--fdt" ]; then
  shift
  OVERLAY="fdt_apply_overlay"
elif [ "$1" == "--ufdt" ]; then
  shift
  OVERLAY="ufdt_apply_overlay"
else
  usage
  exit 1
fi

# --remote: run overlay on the device with adb
if [ "$1" == "--remote" ]; then
  shift
  EXE_PATH="${ANDROID_PRODUCT_OUT}/obj/EXECUTABLES"
  REMOTE_PATH="/data/local/tmp"
  adb push "${EXE_PATH}/${OVERLAY}_intermediates/${OVERLAY}" \
    "$REMOTE_PATH" > /dev/null
fi

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

# Compile the *-base.dts to make *-base.dtb
BASE_DTS_NAME=`basename "$BASE_DTS"`
BASE_DTB_NAME="${BASE_DTS_NAME}-base.dtb"
BASE_DTB="${TEMP_DIR}/${BASE_DTB_NAME}"
dtc -@ -qq -O dtb -o "$BASE_DTB" "$BASE_DTS"
if $DEBUG; then
  echo "[base.dts]"
  dtc -O dts "$BASE_DTB"
fi

# Compile the *-overlay.dts to make *-overlay.dtb
OVERLAY_DTS_NAME=`basename "$OVERLAY_DTS"`
OVERLAY_DTB_NAME="${OVERLAY_DTS_NAME}-overlay.dtb"
OVERLAY_DTB="${TEMP_DIR}/${OVERLAY_DTB_NAME}"
dtc -@ -qq -O dtb -o "$OVERLAY_DTB" "$OVERLAY_DTS"
if $DEBUG; then
  echo "[overlay.dts]"
  dtc -O dts "$OVERLAY_DTB"
fi

# Run ufdt_apply_overlay to combine *-base.dtb and *-overlay.dtb
# into *-merged.dtb
MERGED_DTB_NAME="${BASE_DTS_NAME}-merged.dtb"
MERGED_DTB="${TEMP_DIR}/${MERGED_DTB_NAME}"
if [ -z "$REMOTE_PATH" ]; then
  $VALGRIND "$OVERLAY" "$BASE_DTB" "$OVERLAY_DTB" "$MERGED_DTB"
else
  adb push "$BASE_DTB" "$REMOTE_PATH" > /dev/null
  adb push "$OVERLAY_DTB" "$REMOTE_PATH" > /dev/null
  adb shell "
    cd "$REMOTE_PATH" &&
    "./${OVERLAY}" "$BASE_DTB_NAME" "$OVERLAY_DTB_NAME" "$MERGED_DTB_NAME"
  "
  adb pull "${REMOTE_PATH}/${MERGED_DTB_NAME}" "$MERGED_DTB" > /dev/null
fi

if [ ! -z "$REMOTE_PATH" ]; then
  # clean up
  adb shell "
    cd "$REMOTE_PATH" &&
    rm -f "$OVERLAY" &&
    rm -f "$BASE_DTB_NAME" &&
    rm -f "$OVERLAY_DTB_NAME" &&
    rm -f "$MERGED_DTB_NAME"
  " > /dev/null
fi

# Dump
dtc -s -O dts -o "$OUT_DTS" "$MERGED_DTB"
if $DEBUG; then
  echo "[merged.dts]"
  cat $OUT_DTS
fi
