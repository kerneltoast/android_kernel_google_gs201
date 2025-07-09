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

alert() {
  echo "$*" >&2
}

die() {
  echo "ERROR: $@"
  exit 1
}

command_exists () {
  type "$1" &> /dev/null;
}

remove_local_fixups() {
  sed '/__local_fixups__/ {s/^\s*__local_fixups__\s*//; :again;N; s/{[^{}]*};//; /^$/ !b again; d}' $1
}

remove_overlay_stuff() {
  # remove __symbols__, phandle, "linux,phandle" and __local_fixups__
  sed "/__symbols__/,/[}];/d" $1 | sed "/\(^[ \t]*phandle\)/d" | sed "/\(^[ \t]*linux,phandle\)/d" | sed '/^\s*$/d' | remove_local_fixups
}

dts_diff () {
  diff -u <(cat "$1" | remove_overlay_stuff) <(cat "$2" | remove_overlay_stuff)
}
