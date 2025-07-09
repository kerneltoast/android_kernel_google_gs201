#!/bin/bash

if [ -z "${ANDROID_HOST_OUT}" ]; then
  echo 'ANDROID_HOST_OUT not set. Please run lunch'
  exit 1
fi

# Global variables
BASE_DT_NAME="apply_fragment-base"
OVERLAY_DT_1="multiple_overlay_1"
OVERLAY_DT_2="multiple_overlay_2"
OVERLAY_DT_3="multiple_overlay_3"
OVERLAY_DT_4="multiple_overlay_4"
ANDROID_HOST_BIN_LOCATION=${ANDROID_HOST_OUT}/bin
IN_DATA_DIR="testdata"

tmpdir=$(mktemp -d)
trap 'rm -rf ${tmpdir};' EXIT

dtc -@ -qq -O dtb -o "${tmpdir}/${BASE_DT_NAME}.dtb" "${IN_DATA_DIR}/${BASE_DT_NAME}.dts"
dtc -@ -qq -O dtb -o "${tmpdir}/${OVERLAY_DT_1}.dtb" "${IN_DATA_DIR}/${OVERLAY_DT_1}.dts"
dtc -@ -qq -O dtb -o "${tmpdir}/${OVERLAY_DT_2}.dtb" "${IN_DATA_DIR}/${OVERLAY_DT_2}.dts"
dtc -@ -qq -O dtb -o "${tmpdir}/${OVERLAY_DT_3}.dtb" "${IN_DATA_DIR}/${OVERLAY_DT_3}.dts"
dtc -@ -qq -O dtb -o "${tmpdir}/${OVERLAY_DT_4}.dtb" "${IN_DATA_DIR}/${OVERLAY_DT_4}.dts"

cd $tmpdir

apply_bin_path="${ANDROID_HOST_BIN_LOCATION}/ufdt_apply_overlay"
$apply_bin_path "${BASE_DT_NAME}.dtb" "${OVERLAY_DT_1}.dtb" final_dt.dtb > /dev/null
$apply_bin_path final_dt.dtb "${OVERLAY_DT_2}.dtb" final_dt.dtb > /dev/null
$apply_bin_path final_dt.dtb "${OVERLAY_DT_3}.dtb" final_dt.dtb > /dev/null

#verify that the overlay was correctly applied

verify_bin_path="${ANDROID_HOST_BIN_LOCATION}/ufdt_verify_overlay_host"

overlay_sets=(
                "${OVERLAY_DT_1}.dtb ${OVERLAY_DT_2}.dtb ${OVERLAY_DT_3}.dtb"
                "${OVERLAY_DT_1}.dtb ${OVERLAY_DT_2}.dtb ${OVERLAY_DT_3}.dtb ${OVERLAY_DT_4}.dtb"
                "${OVERLAY_DT_2}.dtb ${OVERLAY_DT_3}.dtb ${OVERLAY_DT_1}.dtb"
                "${OVERLAY_DT_3}.dtb ${OVERLAY_DT_1}.dtb ${OVERLAY_DT_2}.dtb"
                "${OVERLAY_DT_1}.dtb ${OVERLAY_DT_3}.dtb ${OVERLAY_DT_2}.dtb"
                "${OVERLAY_DT_2}.dtb ${OVERLAY_DT_1}.dtb ${OVERLAY_DT_3}.dtb"
                "${OVERLAY_DT_3}.dtb ${OVERLAY_DT_2}.dtb ${OVERLAY_DT_1}.dtb"
                "${OVERLAY_DT_1}.dtb ${OVERLAY_DT_2}.dtb"
                "${OVERLAY_DT_1}.dtb ${OVERLAY_DT_3}.dtb"
                "${OVERLAY_DT_2}.dtb ${OVERLAY_DT_3}.dtb"
                "${OVERLAY_DT_3}.dtb ${OVERLAY_DT_1}.dtb"
                "${OVERLAY_DT_2}.dtb ${OVERLAY_DT_1}.dtb"
                "${OVERLAY_DT_1}.dtb"
                "${OVERLAY_DT_2}.dtb"
                "${OVERLAY_DT_3}.dtb"
                "${OVERLAY_DT_4}.dtb"
                ""
              )

expected_out=(0 1 1 1 1 1 1 1 1 0 1 1 1 1 0 1 1)

for((i=0;i<${#overlay_sets[@]};i++))
do
  $verify_bin_path final_dt.dtb ${overlay_sets[$i]} >& /dev/null
  result=$?
  if [[ "$result" -ne "${expected_out[$i]}" ]]
  then
    echo "Incorrect overlay application/verification for"  ${overlay_sets[$i]}
    exit 1
  fi
done

echo "Overlays Verified Successfully"
exit 0
