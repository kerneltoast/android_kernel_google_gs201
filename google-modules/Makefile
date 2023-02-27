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
obj-y += aoc/
obj-y += aoc/alsa/
obj-y += aoc/usb/
obj-y += bluetooth/broadcom/
obj-y += bluetooth/qcom/
obj-y += bms/
obj-y += display/samsung/
obj-y += edgetpu/janeiro/drivers/edgetpu/
obj-y += gpu/mali_kbase/
obj-y += gpu/mali_pixel/
obj-y += gxp/gs201/
obj-y += lwis/
obj-y += nfc/
obj-y += power/reset/
obj-y += touch/common/
obj-y += touch/focaltech/ft3658/
obj-y += touch/goodix/
obj-y += touch/synaptics/
obj-y += uwb/kernel/
obj-y += video/gchips/
obj-y += wlan/bcmdhd/bcm4389/
obj-y += wlan/qcom/wcn6740/cnss2/
obj-y += wlan/qcom/wcn6740/wlan/qcacld-3.0/
