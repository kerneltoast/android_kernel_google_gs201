#! /bin/bash
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
set -e

if ! hash iasl; then
    echo "Please install 'acpica-tools' first"
    exit 1
fi

SRCDIR="data"
ASL_LIST="
  ssdt1.asl
  ssdt2.asl
  ssdt3.asl
"
PYCONFIG="${SRCDIR}/mkdtboimg_acpi.cfg"

OUTDIR="out"
MKDTIMG_OUT="${OUTDIR}/mkdtimg_acpi_out"
MKDTIMG_DUMP="${MKDTIMG_OUT}"/dump.aml

MKDTBOIMG_OUT="${OUTDIR}/mkdtboimg_acpi_out"
MKDTBOIMG_OUTCREATE="${MKDTBOIMG_OUT}/create"
MKDTBOIMG_OUTCFG="${MKDTBOIMG_OUT}/cfg_create"
MKDTBOIMG_CREATEDUMP="${MKDTBOIMG_OUTCREATE}"/dump.aml
MKDTBOIMG_CFGDUMP="${MKDTBOIMG_OUTCFG}"/dump.aml

mkdir -p "$MKDTIMG_OUT"
mkdir -p "$MKDTBOIMG_OUTCREATE"
mkdir -p "$MKDTBOIMG_OUTCFG"

for asl in ${ASL_LIST}; do
  echo "Building $asl..."
  src_asl="${SRCDIR}/${asl}"
  out_aml_pfx="${OUTDIR}/${asl%.asl}"
  iasl -p "${out_aml_pfx}" "$src_asl"
done

echo "Creating acpi image with mkdtbimg"
mkdtimg create ${MKDTIMG_OUT}/create_acpi.img --dt_type=acpi --page_size=2048 --id=0x100 --rev=0x100 --version=1 \
    "${OUTDIR}/ssdt1.aml" "${OUTDIR}/ssdt2.aml" "${OUTDIR}/ssdt3.aml" > /dev/null

echo "Creating acpi image with mkdtboimg"
../src/mkdtboimg.py create  ${MKDTBOIMG_OUTCREATE}/create_acpi.img --dt_type=acpi --page_size=2048 --id=0x200 --rev=0x200 --version=2 \
    "${OUTDIR}/ssdt1.aml" "${OUTDIR}/ssdt2.aml" "${OUTDIR}/ssdt3.aml" > /dev/null

echo "Creating acpi image with ${PYCONFIG} config file"
../src/mkdtboimg.py cfg_create ${MKDTBOIMG_OUTCFG}/create_acpi.img ${PYCONFIG} -d "${OUTDIR}"

echo "Dumping fragments from mkdtimg tool image"
mkdtimg dump ${MKDTIMG_OUT}/create_acpi.img -b "${MKDTIMG_DUMP}"| grep -v 'FDT' > ${MKDTIMG_OUT}/create.dump

echo "Dumping fragments from mkdtboimg.py tool for image generated with 'create'"
../src/mkdtboimg.py dump ${MKDTBOIMG_OUTCREATE}/create_acpi.img --output ${MKDTBOIMG_OUTCREATE}/create.dump -b "${MKDTBOIMG_CREATEDUMP}" --decompress

echo "Dumping fragments from mkdtboimg.py tool for image generated with 'cfg_create'"
../src/mkdtboimg.py dump ${MKDTBOIMG_OUTCFG}/create_acpi.img --output ${MKDTBOIMG_OUTCFG}/create.dump -b "${MKDTBOIMG_CFGDUMP}" --decompress

echo "======================================================================================"
echo "Testing differences between image created by 'create' for 'mkdtimg' and 'mkdtboimg.py'"
echo "======================================================================================"
for x in ${MKDTIMG_DUMP}.*
do
    file=$(basename $x)
    if [ ! -e ${MKDTBOIMG_OUTCREATE}/$file ]
    then
        continue
    fi
    echo "diff $x vs ${MKDTBOIMG_OUTCREATE}/$file"
    diff $x ${MKDTBOIMG_OUTCREATE}/$file
done
echo "=========================================================================================="
echo "Testing differences between image created by 'cfg_create' for 'mkdtimg' and 'mkdtboimg.py'"
echo "=========================================================================================="
for x in ${MKDTIMG_DUMP}.*
do
    file=$(basename $x)
    if [ ! -e ${MKDTBOIMG_OUTCFG}/$file ]
    then
        continue
    fi
    echo "diff $x vs ${MKDTBOIMG_OUTCFG}/$file"
    diff $x ${MKDTBOIMG_OUTCFG}/$file
done
