# SPDX-License-Identifier: GPL-2.0-only
#ccflags-y += -I$(WLAN_PLATFORM_ROOT)/inc
obj-$(CONFIG_QRTR) += qrtr.o
qrtr-y := af_qrtr.o ns.o
qrtr-$(CONFIG_QRTR_DEBUG) += debug.o
obj-$(CONFIG_QRTR_MHI) += qrtr-mhi.o
qrtr-mhi-y	:= mhi.o

ccflags-y += -DCONFIG_WCN_GOOGLE
