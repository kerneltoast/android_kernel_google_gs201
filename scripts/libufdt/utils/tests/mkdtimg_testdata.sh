#!/bin/bash
# Copyright (C) 2017 The Android Open Source Project
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

SRCDIR="data"
DTS_LIST="
  board1v1.dts
  board1v1_1.dts
  board2v1.dts
"
DTB_LIST=(
  "board1v1.dts.dtb"
  "board1v1_1.dts.dtb"
  "board2v1.dts.dtb"
  "board1v1.dts.dtb"
)
CONFIG="${SRCDIR}/mkdtimg.cfg"

ALIGN=4

OUTDIR="out"
OUTDTB_CFG="${OUTDIR}/dump_cfg.dtb"
OUTDTB="${OUTDIR}/dump.dtb"

mkdir -p "$OUTDIR"
for dts in ${DTS_LIST}; do
  echo "Building $dts..."
  src_dts="${SRCDIR}/${dts}"
  out_dtb="${OUTDIR}/${dts}.dtb"
  dtc -O dtb -@ -qq -a "$ALIGN" -o "$out_dtb" "$src_dts"
done

IMG="${OUTDIR}/cfg_create.img"
mkdtimg cfg_create "$IMG" "${CONFIG}" --dtb-dir="$OUTDIR"
mkdtimg dump "$IMG" -b "$OUTDTB_CFG" | tee "${OUTDIR}/cfg_create.dump"
for index in ${!DTB_LIST[@]}; do
  diff ${OUTDIR}/${DTB_LIST[$index]} ${OUTDTB_CFG}.$index
done

IMG="${OUTDIR}/create.img"
mkdtimg create "$IMG" \
  --page_size=4096 --id=/:board_id --rev=/:board_rev --custom0=0xabc --version=1 \
  "${OUTDIR}/board1v1.dts.dtb" \
  "${OUTDIR}/board1v1_1.dts.dtb" --id=/:another_board_id \
  "${OUTDIR}/board2v1.dts.dtb" --rev=0x201 \
  "${OUTDIR}/board1v1.dts.dtb" --custom0=0xdef
mkdtimg dump "$IMG" -b "$OUTDTB" | tee "${OUTDIR}/create.dump"
for index in ${!DTB_LIST[@]}; do
  diff ${OUTDIR}/${DTB_LIST[$index]} ${OUTDTB}.$index
done

diff "${OUTDIR}/cfg_create.dump" "${OUTDIR}/create.dump"
