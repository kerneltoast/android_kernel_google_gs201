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
PYCONFIG="${SRCDIR}/mkdtboimg.cfg"

ALIGN=4

OUTDIR="out"
MKDTIMG_OUT="${OUTDIR}/mkdtimg_out"
MKDTIMG_DUMP="${MKDTIMG_OUT}"/dump.dtb

MKDTBOIMG_OUT="${OUTDIR}/mkdtboimg_out"
MKDTBOIMG_OUTCREATE="${MKDTBOIMG_OUT}/create"
MKDTBOIMG_OUTCFG="${MKDTBOIMG_OUT}/cfg_create"
MKDTBOIMG_CREATEDUMP="${MKDTBOIMG_OUTCREATE}"/dump.dtb
MKDTBOIMG_CFGDUMP="${MKDTBOIMG_OUTCFG}"/dump.dtb

mkdir -p "$MKDTIMG_OUT"
mkdir -p "$MKDTBOIMG_OUTCREATE"
mkdir -p "$MKDTBOIMG_OUTCFG"

for dts in ${DTS_LIST}; do
  echo "Building $dts..."
  src_dts="${SRCDIR}/${dts}"
  out_dtb="${OUTDIR}/${dts}.dtb"
  dtc -O dtb -@ -qq -a "$ALIGN" -o "$out_dtb" "$src_dts"
done

echo "Creating dtbo image with mkdtbimg"
mkdtimg create ${MKDTIMG_OUT}/create.img --page_size=4096 --id=0x100 --version=1\
    --rev=0x100 --custom0=0xabc "${OUTDIR}/board1v1.dts.dtb" "${OUTDIR}/board1v1_1.dts.dtb" \
    --id=0xddccbbaa --rev=0x01000100 "${OUTDIR}/board2v1.dts.dtb" --id=0x200 \
    --rev=0x201 "${OUTDIR}/board1v1.dts.dtb" --custom0=0xdef \
    "${OUTDIR}/board1v1.dts.dtb" --custom0=0xdef > /dev/null

echo "Creating dtbo image with mkdtboimg"
../src/mkdtboimg.py create  ${MKDTBOIMG_OUTCREATE}/create.img --page_size=4096 \
    --id=0x100 --rev=0x100 --flags=0xabc0 --version=1 "${OUTDIR}/board1v1.dts.dtb" \
    "${OUTDIR}/board1v1_1.dts.dtb" --id=0xddccbbaa --rev=0x01000100 \
    "${OUTDIR}/board2v1.dts.dtb" --id=0x200 --rev=0x201 \
    "${OUTDIR}/board1v1.dts.dtb" --flags=0xd01 \
    "${OUTDIR}/board1v1.dts.dtb" --flags=0xd02 > /dev/null

echo "Creating dtbo image with ${PYCONFIG} config file"
../src/mkdtboimg.py cfg_create ${MKDTBOIMG_OUTCFG}/create.img ${PYCONFIG} --dtb-dir "${OUTDIR}"

echo "Dumping fragments from mkdtimg tool image"
mkdtimg dump ${MKDTIMG_OUT}/create.img -b "${MKDTIMG_DUMP}"| grep -v 'FDT' > ${MKDTIMG_OUT}/create.dump

echo "Dumping fragments from mkdtboimg.py tool for image generated with 'create'"
../src/mkdtboimg.py dump ${MKDTBOIMG_OUTCREATE}/create.img --output ${MKDTBOIMG_OUTCREATE}/create.dump -b "${MKDTBOIMG_CREATEDUMP}" --decompress

echo "Dumping fragments from mkdtboimg.py tool for image generated with 'cfg_create'"
../src/mkdtboimg.py dump ${MKDTBOIMG_OUTCFG}/create.img --output ${MKDTBOIMG_OUTCFG}/create.dump -b "${MKDTBOIMG_CFGDUMP}" --decompress

echo "======================================================================================"
echo "Testing differences between image created by 'create' for 'mkdtimg' and 'mkdtboimg.py'"
echo "======================================================================================"
for x in `ls -1 ${MKDTIMG_DUMP}.*`
do
    file=`basename $x`
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
for x in `ls -1 ${MKDTIMG_DUMP}.*`
do
    file=`basename $x`
    if [ ! -e ${MKDTBOIMG_OUTCFG}/$file ]
    then
        continue
    fi
    echo "diff $x vs ${MKDTBOIMG_OUTCFG}/$file"
    diff $x ${MKDTBOIMG_OUTCFG}/$file
done
