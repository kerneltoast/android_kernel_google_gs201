# SPDX-License-Identifier: GPL-2.0
#
# USB peripheral controller drivers
#

ccflags-y			+= -I$(srctree)/drivers/usb/gadget/function/

# USB Functions
usb_f_dm-y			:= f_dm.o
obj-$(CONFIG_USB_F_DM)		+= usb_f_dm.o
usb_f_dm1-y			:= f_dm1.o
obj-$(CONFIG_USB_F_DM)		+= usb_f_dm1.o
usb_f_etr_miu-y                 := f_etr_miu.o
obj-$(CONFIG_USB_F_ETR_MIU)     += usb_f_etr_miu.o
