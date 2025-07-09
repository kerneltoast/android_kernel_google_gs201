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

if ! hash iasl; then
  echo "Please install 'acpica-tools' first."
  exit 1
fi

SRCDIR="data"
ASL_LIST="
  ssdt1.asl
  ssdt2.asl
  ssdt3.asl
"
AML_LIST=(
  "ssdt1.aml"
  "ssdt2.aml"
  "ssdt3.aml"
)
CONFIG="${SRCDIR}/mkdtimg_acpi.cfg"

OUTDIR="out"
OUTDTB_CFG="${OUTDIR}/dump_cfg.aml"
OUTDTB="${OUTDIR}/dump.aml"

mkdir -p "$OUTDIR"
for asl in ${ASL_LIST}; do
  echo "Building $asl..."
  src_asl="${SRCDIR}/${asl}"
  out_aml_pfx="${OUTDIR}/${asl%.asl}"
  iasl -p "${out_aml_pfx}" "$src_asl"
done

IMG="${OUTDIR}/cfg_create_acpi.img"
mkdtimg cfg_create "$IMG" "${CONFIG}" --dtb-dir="$OUTDIR"
mkdtimg dump "$IMG" -b "$OUTDTB_CFG" | grep -v 'FDT' | tee "${OUTDIR}/cfg_create_acpi.dump"
for index in "${!AML_LIST[@]}"; do
  diff ${OUTDIR}/${AML_LIST[$index]} ${OUTDTB_CFG}.$index
done

IMG="${OUTDIR}/create_acpi.img"
mkdtimg create "$IMG" --dt_type=acpi --page_size=2048 --id=0x100 --rev=0x100 --version=1 \
  "${OUTDIR}/ssdt1.aml" "${OUTDIR}/ssdt2.aml" "${OUTDIR}/ssdt3.aml"
mkdtimg dump "$IMG" -b "$OUTDTB" | grep -v 'FDT' | tee "${OUTDIR}/create_acpi.dump"
for index in "${!AML_LIST[@]}"; do
  diff ${OUTDIR}/${AML_LIST[$index]} ${OUTDTB}.$index
done

diff "${OUTDIR}/cfg_create_acpi.dump" "${OUTDIR}/create_acpi.dump"
