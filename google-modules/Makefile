# SPDX-License-Identifier: GPL-2.0

# Set KERNEL_SRC to google-modules so that the include paths are set correctly
# for in-tree compilation, since the modules assume that `google-modules` exists
# in the same directory as the kernel tree (one level up), whereas we've got
# `google-modules` inside the kernel tree. This way, when the modules include
# with `-I$(KERNEL_SRC)/../google-modules/SOME_MODULE`, inclusion works as-is.
export KERNEL_SRC := $(srctree)/google-modules

obj-y += amplifiers/cs35l41/
obj-y += amplifiers/cs35l45/
obj-y += amplifiers/cs40l26/
obj-y += amplifiers/drv2624/
obj-y += amplifiers/snd_soc_wm_adsp/
obj-y += aoc/
obj-y += aoc/alsa/
obj-y += aoc/usb/
obj-y += bluetooth/broadcom/
obj-y += bluetooth/qcom/
obj-y += bms/
obj-y += bms/misc/
obj-y += display/common/gs_drm/
obj-y += display/common/gs_panel/
obj-y += display/samsung/
obj-y += edgetpu/janeiro/drivers/edgetpu/
obj-y += fingerprint/goodix/fps_touch_handler/
obj-y += gps/broadcom/bcm47765/
obj-y += gpu/mali_kbase/
obj-y += gpu/mali_pixel/
obj-y += gxp/gs201/
obj-y += gxp/gs201/gcip-kernel-driver/drivers/gcip/
obj-y += lwis/
obj-y += nfc/
obj-y += power/mitigation/
obj-y += power/reset/
obj-y += radio/samsung/s5300/
obj-y += soc/gs/
obj-y += touch/common/
obj-y += touch/focaltech/ft3658/
obj-y += touch/goodix/
obj-y += touch/synaptics/syna_c10/
obj-y += trusty/
obj-y += uwb/qorvo/dw3000/kernel/
obj-y += video/gchips/
obj-y += wlan/bcm4389/
obj-y += wlan/wcn6740/cnss2/
obj-y += wlan/wcn6740/wlan/qcacld-3.0/
