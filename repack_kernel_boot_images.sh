#!/bin/bash -e
usage () {
    set +x
cat <<EOF
Usage: $0 device_serialno kernel_dist_dir output_dir

e.g. $0 123456789ABCDE out/mixed/dist output_dir

1) device_serialno is device serial-number can be found from "adb devices".

2) kernel_dist_dir is the path to where the kernel binaries can be found
    Image.lz4
    initramfs.img
    *.dtb
    dtbo.img
    vendor_dlkm.img

3) out_dir is the path to where the new build will be placed.

************************************************************************
This script is intended to avoid the dependency issue between device base-rom
and kernel prebuilt ramdisks under prebuilt/boot-artifacts/ramdisk/. To
achieve this, this script will dump/extract ramdisks from device via ADB
and recreate boot.img and vendor_boot.img based on it.

This script does not re-create any of the vbmeta partition images which
dm-verity has to be disabled prior to flashing these images.

Please refer to b/203499549 for the details.
************************************************************************

EOF
}

# Print error message and exit.
# Usage: exit_badparam message
#
# message is a string to be displayed before exit.
exit_badparam () {
    echo "ERROR: $1" >&2
    echo
    usage
    exit 1
}

cleanup_and_exit () {
    readonly result="$?"
    rm -rf "${TEMP_DIR}"
    exit "${result}"
}

# $1 header file to parse
# $2 key to get
get_bootimg_header_field() {
    local hfile="$1"
    local key="$2"
    if [ ! -f "${hfile}" ]; then
        echo "Boot image header file does not exist." >&2
        exit 1
    fi
    sed -ne "/^${key}: /s/^${key}: //p" "${hfile}"
}

# adb root
function adb_wait_root() {
    if ! adb -s $SERIALNO get-state >& /dev/null ; then
        echo
        echo "!!! Device $SERIALNO is not reachable via ADB !!!"
        echo "Please make sure device boot into Android and tap \"Allow\" to the \"Allow USB Debugging\" dialog."
        echo
        exit 1
    fi
    if ! adb -s $SERIALNO root | grep 'adbd is already running as root' ; then
        sleep 2
        adb -s $SERIALNO wait-for-device >& /dev/null
    fi
}

# Dump partitions from device
function dump_boot_images_from_device() {
	adb_wait_root
	for img in dtbo boot vendor_boot vendor_dlkm; do
		if [ "${img}" == "vendor_dlkm" ]; then
			adb -s $SERIALNO pull /dev/block/$(adb -s $SERIALNO shell getprop dev.mnt.blk.${img}) "${DEVICE_DIR}"/${img}.img
		else
			adb -s $SERIALNO pull /dev/block/bootdevice/by-name/${img}$(adb -s $SERIALNO shell getprop ro.boot.slot_suffix) "${DEVICE_DIR}"/${img}.img
		fi
	done
}

trap cleanup_and_exit EXIT

ARGS=()
KCOMPRESS=.lz4

while [[ $# -gt 0 ]]; do
    case "$1" in
        --*)
            exit_badparam "unrecognized option '$1'"
            shift
            ;;
        *)
            ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ ${#ARGS[*]} -gt 4 ]]; then
	echo
    exit_badparam "Unexpected number of arguments"
fi

readonly SERIALNO="${ARGS[0]}"
readonly KERNEL_DIST_DIR="${ARGS[1]}"
readonly OUTPUT_DIR="${ARGS[2]}"
readonly DEVICE_DIR="${ARGS[2]}"/DUMP_FROM_DEVICE
readonly TEMP_DIR="$(mktemp -d --tmpdir "$(basename $0)"_XXXXXXXX)"
readonly LZ4="prebuilts/kernel-build-tools/linux-x86/bin/lz4"
readonly LZ4_COMPRESS="${LZ4} -c -12 --favor-decSpeed"
readonly LZ4_DECOMP="${LZ4} -c -f -d"

# Create destination directory if need be
if [ -n "${OUTPUT_DIR}" ] && [ ! -d "${OUTPUT_DIR}" ]; then
    mkdir -p "${OUTPUT_DIR}" || true
fi

RESP=${ARGS[3]:-""}
# Create device directory and dump paritions from device if need be
if [ -n "${DEVICE_DIR}" ] && [ ! -d "${DEVICE_DIR}" ]; then
    mkdir -p "${DEVICE_DIR}" || true
    dump_boot_images_from_device
else
	if [ "${RESP}" == "" ]; then
		echo
		read -p "Previous dumped files already exist in ${DEVICE_DIR}/ !!! Do you want to dump from device and replace it? (y/N): " RESP
		echo
	fi
	case $RESP in
		[Yy]* ) dump_boot_images_from_device;	;;
	esac
fi

KERNEL_IMAGE="${KERNEL_DIST_DIR}/Image${KCOMPRESS}"

# Extract boot images
echo "Extracting boot.img from ${DEVICE_DIR}/ ..."
cp -rf "${DEVICE_DIR}"/boot.img "${TEMP_DIR}"/boot.img

echo "Unpacking boot.img ..."
tools/mkbootimg/unpack_bootimg.py --boot_img "${TEMP_DIR}"/boot.img \
    --out "${TEMP_DIR}"/bootimg > "${TEMP_DIR}"/bootimg.header

get_bhf() {
    get_bootimg_header_field "${TEMP_DIR}"/bootimg.header "$1"
}

header_version="$(get_bhf 'boot image header version')"
if ! [ "${header_version}" -ge 0 ]; then
    echo "Invalid boot image header version: ${header_version}" >&2 && exit 1
fi

if [ "${header_version}" -ge 4 ]; then
    echo "Extracting vendor_boot.img from ${DEVICE_DIR}/ ..."
    cp -rf "${DEVICE_DIR}"/vendor_boot.img "${TEMP_DIR}"/vendor_boot.img
fi

get_vendor_bhf() {
    get_bootimg_header_field "${TEMP_DIR}"/vendor_bootimg.header "$1"
}

# Concatenate provided .dtb files into dtb.img
cat "${KERNEL_DIST_DIR}"/*.dtb > "${TEMP_DIR}"/dtb.img

MKBOOTIMGARGS=()
MKBOOTIMG_VENDORARGS=()

boot_ramdisk="${TEMP_DIR}"/bootimg/ramdisk

if [ "${header_version}" -eq 4 ]; then
    echo "Unpacking vendor_boot.img ..."
    tools/mkbootimg/unpack_bootimg.py --boot_img "${TEMP_DIR}"/vendor_boot.img \
        --out "${TEMP_DIR}"/vendor_bootimg --format=mkbootimg -0 > "${TEMP_DIR}"/vendor_bootimg.args

    while IFS= read -r -d '' ARG; do
        MKBOOTIMG_VENDORARGS+=("${ARG}")
    done <"${TEMP_DIR}"/vendor_bootimg.args
    MKBOOTIMG_VENDORARGS+=("--vendor_boot" "${TEMP_DIR}"/vendor_boot.img)

    if [ ! -f "${TEMP_DIR}"/vendor_bootimg/vendor-ramdisk-by-name/ramdisk_dlkm ]; then
        echo "dlkm ramdisk fragment missing from vendor_boot.img" >&2
        exit 1
    fi
    cp "${KERNEL_DIST_DIR}"/initramfs.img \
        "${TEMP_DIR}"/vendor_bootimg/vendor-ramdisk-by-name/ramdisk_dlkm

    if [ ! -f "${TEMP_DIR}"/vendor_bootimg/dtb ]; then
        echo "dtb missing from vendor_boot.img" >&2
        exit 1
    fi
    cp "${TEMP_DIR}"/dtb.img \
        "${TEMP_DIR}"/vendor_bootimg/dtb

    MKBOOTIMGARGS+=("--ramdisk" "${boot_ramdisk}")
else
    echo "ERROR: Unsupported boot image header version: ${header_version}" >&2
    exit 1
fi

# For header_version == 4, "unpack_bootimg --format=mkbootimg" will provide the
# --dtb argument.
if [ "${header_version}" -lt 4 ]; then
    MKBOOTIMGARGS+=("--dtb" "${TEMP_DIR}"/dtb.img)
fi

MKBOOTIMGARGS+=("--header_version" "${header_version}")
MKBOOTIMGARGS+=("--cmdline" "$(get_bhf 'command line args')")

# Create boot.img and vendor_boot.img. (vendor_boot.img is only created for
# header_version is 4 or later)
echo "Creating repacked boot.img and vendor_boot.img..."
tools/mkbootimg/mkbootimg.py \
    --kernel "$KERNEL_IMAGE" \
    "${MKBOOTIMGARGS[@]}" \
    "${MKBOOTIMG_VENDORARGS[@]}" \
    --output "${TEMP_DIR}"/boot.img

BOOT_SIZE=$(stat -c%s "$DEVICE_DIR"/boot.img)
prebuilts/kernel-build-tools/linux-x86/bin/avbtool add_hash_footer \
    --image "${TEMP_DIR}"/boot.img \
    --partition_size ${BOOT_SIZE} \
    --partition_name boot

VENDOR_BOOT_SIZE=$(stat -c%s "$DEVICE_DIR"/vendor_boot.img)
if [ "${header_version}" -ge 4 ]; then
    prebuilts/kernel-build-tools/linux-x86/bin/avbtool add_hash_footer \
        --image "${TEMP_DIR}"/vendor_boot.img \
        --partition_size ${VENDOR_BOOT_SIZE} \
        --partition_name vendor_boot
fi

# Update boot.img in device archive
cp -rf "${TEMP_DIR}/boot.img" "${OUTPUT_DIR}"
if [ "${header_version}" -ge 4 ]; then
    # and vendor_boot.img (if header_version >= 4)
    echo "Updating boot.img and vendor_boot.img in ${OUTPUT_DIR}/ ..."
    cp -rf "${TEMP_DIR}/vendor_boot.img" "${OUTPUT_DIR}"
fi

# Update {dtbo.img,vendor_dlkm.img} if they are products of the kernel build
for img in dtbo.img vendor_dlkm.img; do
	if [ -f "${KERNEL_DIST_DIR}/${img}" ]; then
		echo "Copying ${img} into ${OUTPUT_DIR}/ ..."
		cp -rf "${KERNEL_DIST_DIR}/${img}" "${OUTPUT_DIR}"
	fi
done

echo
echo "Images have been repacked at: ${OUTPUT_DIR}"/
echo
echo "You may use below command to flash repacked kernel images:"
echo "-------------------------------------------------------------"
echo "cd ${OUTPUT_DIR}/"
echo "fastboot oem disable-verity"
echo "fastboot flash boot boot.img"
echo "fastboot flash dtbo dtbo.img"
echo "fastboot flash vendor_boot vendor_boot.img"
echo "fastboot reboot fastboot"
echo "fastboot flash vendor_dlkm vendor_dlkm.img"
echo "fastboot -w"
echo "fastboot reboot"
echo "-------------------------------------------------------------"
echo
