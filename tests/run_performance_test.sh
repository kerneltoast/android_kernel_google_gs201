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

# Include some functions from common.sh.
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
source ${SCRIPT_DIR}/common.sh

OUT_DATA_DIR="test_out"

# Disable REMOTE for test on local
REMOTE="--remote"

TEST_COUNT=10
DEBUG=false


apply_overlay() {
  local overaly="$1"
  local base_dts="$2"
  local overlay_dts="$3"
  local merged_dts="$4"

  local summary=0
  for i in $(seq 1 $TEST_COUNT); do
    TIME=$(./apply_overlay.sh "--$overaly" $REMOTE \
      "$base_dts" "$overlay_dts" "$merged_dts" \
        | sed -E "s/.*took ([0-9]+\\.[0-9]+) secs/\\1/")
    if $DEBUG; then
      echo "time=$TIME"
    fi
    summary=$(echo "$summary + $TIME" | bc -l)
  done
  AVERAY=$(echo "scale=9; $summary/$TEST_COUNT" | bc -l)
  printf "%10s Average time=%s\n" "($overaly)" "$AVERAY"
}

run_performance_test() {
  # see ufdt_gen_test_dts.c for detail
  local node_depth="$1"
  local node_unused="$2"
  local node_count="$3"
  local append_count="$4"
  local override_count="$5"

  mkdir -p "$OUT_DATA_DIR" >& /dev/null

  #
  # Prepare dtb and dtbo files
  #
  local base_dts="${OUT_DATA_DIR}/base.dts"
  echo "  Base DT: depth=$node_depth unused=$node_unused nodes=$node_count"
  ufdt_gen_test_dts -n $node_count -d $node_depth -u $node_unused \
    -o "$base_dts"
  if $DEBUG; then
    cat "$base_dts"
  fi

  local overlay_dts="${OUT_DATA_DIR}/overlay.dts"
  echo "  Overlay DT: append=$append_count override=$override_count"
  ufdt_gen_test_dts -p -a $append_count -w $override_count \
    -o "$overlay_dts"
  if $DEBUG; then
    cat "$overlay_dts"
  fi

  local merged_dts="${OUT_DATA_DIR}/overlay_merged.dts"
  apply_overlay fdt $base_dts $overlay_dts $merged_dts
  apply_overlay ufdt $base_dts $overlay_dts $merged_dts

  rm -rf "$OUT_DATA_DIR"
}

main() {
  alert "========== Running Performance Tests =========="

  if [ -z "$ANDROID_BUILD_TOP" ]; then
    die "Run envsetup.sh / lunch yet?"
  fi

  if ! command_exists ufdt_gen_test_dts ||
     ! command_exists dtc; then
    die "Run mmma $(dirname $SCRIPT_DIR) yet?"
  fi

  if [ "$REMOTE" == "--remote" ]; then
    adb get-state > /dev/null
    if [ "$?" -ne "0" ]; then
      die "adb can not connect to device."
    fi
  fi

  # cd to ${SCRIPT_DIR} in a subshell because gen_test.sh uses relative
  # paths for dependent files.
  cd "${SCRIPT_DIR}"

  #
  # run_performance_test
  #   <node_depth> <node_unused> <node_count> <append_count> <override_count>

  # Test case #1: node=x append=100 override=100
  for t in $(seq 200 50 1000); do
    run_performance_test 2 0 $t 100 100
  done
  # Test case #2: node=x append=100 override=100
  for t in $(seq 200 50 1000); do
    run_performance_test 2 $t $t 100 100
  done
  # Test case #3: node=2000 append=X
  for t in $(seq 50 50 1000); do
    run_performance_test 2 0 2000 $t 0
  done
  # Test case #4: node=2000 override=X
  for t in $(seq 50 50 1000); do
    run_performance_test 2 0 2000 0 $t
  done
}

main "$@"
