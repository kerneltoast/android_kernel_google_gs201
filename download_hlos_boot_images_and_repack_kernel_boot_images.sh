#!/bin/bash
CUR_DIR=$(pwd)
HLOS_BUILD_ID=$1
HLOS_BASE_TARGET=$2
KERNEL_BUILD_ID=$3
KERNEL_BASE_TARGET=$4
DEST_DIR=$5
DOWNLOAD_FROM_KERNEL=${DEST_DIR}/DOWNLOAD_FROM_KERNEL
OUTPUT_UNPACK=${DOWNLOAD_FROM_KERNEL}/output
OUTPUT_DIST=${DEST_DIR}/out/mixed/dist
IMG_ZIP_BASE="${HLOS_BASE_TARGET}-img-${HLOS_BUILD_ID}"
FETCH_ARTIFACT_CMD="/google/data/ro/projects/android/fetch_artifact"

#-----------------------------------------------------------------------
# Usage 1 : download/repack boot images from ab/hlos and ab/kernel
#
# ./download_hlos_boot_images_and_repack_kernel_boot_images.sh <HLOS_BUILD_ID> <HLOS_BASE_TARGET> <KERNEL_BUILD_ID> <KERNEL_BASE_TARGET> <DEST_DIR>
# e.g. ./download_hlos_boot_images_and_repack_kernel_boot_images.sh 8033893 raven 8032883 slider_gki out_slider_gki
#
#
# Usage 2 : download/repack boot images from ab/hlos and local/kernel
#
# ./download_hlos_boot_images_and_repack_kernel_boot_images.sh <HLOS_BUILD_ID> <HLOS_BASE_TARGET> <KERNEL_DIST_DIR> <KERNEL_BASE_TARGET> <DEST_DIR>
# ./download_hlos_boot_images_and_repack_kernel_boot_images.sh 8033893 raven out/mixed/dist slider_gki out_slider_gki

#-----------------------------------------------------------------------
echo
echo "Downloading HLOS bootimages from ${HLOS_BUILD_ID} ${HLOS_BASE_TARGET}-userdebug into "$DEST_DIR"/DUMP_FROM_DEVICE/ ..."
mkdir -p ${DEST_DIR}/DUMP_FROM_DEVICE
cd ${DEST_DIR}/DUMP_FROM_DEVICE
${FETCH_ARTIFACT_CMD} \
--bid ${HLOS_BUILD_ID} \
--target ${HLOS_BASE_TARGET}-userdebug \
--zip_entry boot.img \
--zip_entry dtbo.img \
--zip_entry vendor_boot.img \
--zip_entry vendor_dlkm.img \
${IMG_ZIP_BASE}.zip
cd ${CUR_DIR}

#-----------------------------------------------------------------------
if [ -n "${KERNEL_BUILD_ID}" ] && [ -d "${KERNEL_BUILD_ID}" ]; then
	echo "Folder ${HLOS_BUILD_ID} exist, use this folder as the source of kernel boot images"
	DOWNLOAD_FROM_KERNEL=$KERNEL_BUILD_ID
else
	echo "Folder ${HLOS_BUILD_ID} does not exist, download kernel boot images from ab/${HLOS_BUILD_ID}"
	echo "Downloading KERNEL bootimages from ${KERNEL_BUILD_ID} ${KERNEL_BASE_TARGET}-userdebug into ${DOWNLOAD_FROM_KERNEL}/ ..."
	mkdir -p ${DOWNLOAD_FROM_KERNEL}
	cd ${DOWNLOAD_FROM_KERNEL}
	${FETCH_ARTIFACT_CMD} --bid ${KERNEL_BUILD_ID} --target ${KERNEL_BASE_TARGET} 'boot.img'
	${FETCH_ARTIFACT_CMD} --bid ${KERNEL_BUILD_ID} --target ${KERNEL_BASE_TARGET} 'dtbo.img'
	${FETCH_ARTIFACT_CMD} --bid ${KERNEL_BUILD_ID} --target ${KERNEL_BASE_TARGET} 'vendor_boot.img'
	${FETCH_ARTIFACT_CMD} --bid ${KERNEL_BUILD_ID} --target ${KERNEL_BASE_TARGET} 'vendor_dlkm.img'
	cd ${CUR_DIR}
fi

#-----------------------------------------------------------------------
echo "Unpacking bootimages into ${OUTPUT_UNPACK} ..."
mkdir -p ${OUTPUT_UNPACK}
tools/mkbootimg/unpack_bootimg.py --boot_img ${DOWNLOAD_FROM_KERNEL}/boot.img --out ${OUTPUT_UNPACK}
tools/mkbootimg/unpack_bootimg.py --boot_img ${DOWNLOAD_FROM_KERNEL}/vendor_boot.img --out ${OUTPUT_UNPACK}

#-----------------------------------------------------------------------
echo "Copying Image.lz4, initramfs.img, .dtb, dtbo.img, vendor_dlkm.img into $OUTPUT_DIST for repack script ..."
mkdir -p ${OUTPUT_DIST}
cp ${OUTPUT_UNPACK}/kernel ${OUTPUT_DIST}/Image.lz4
cp ${OUTPUT_UNPACK}/vendor_ramdisk01 ${OUTPUT_DIST}/initramfs.img
cp ${OUTPUT_UNPACK}/dtb ${OUTPUT_DIST}/gs.dtb
cp ${DOWNLOAD_FROM_KERNEL}/dtbo.img ${OUTPUT_DIST}/dtbo.img
cp ${DOWNLOAD_FROM_KERNEL}/vendor_dlkm.img ${OUTPUT_DIST}/vendor_dlkm.img

#-----------------------------------------------------------------------
echo "Repacking KERNEL bootimages based on HLOS bootimages into "$DEST_DIR"/ ..."
./repack_kernel_boot_images.sh 11223344 ${OUTPUT_DIST} ${DEST_DIR} Not_To_Replace

#-----------------------------------------------------------------------
echo "Done."
