# SPDX-License-Identifier: GPL-2.0-only

obj-$(CONFIG_CNSS2) += cnss2.o

ifeq ($(CONFIG_CNSS_OUT_OF_TREE),y)
ccflags-y += -I$(WLAN_PLATFORM_ROOT)/inc
ccflags-y += -I$(WLAN_PLATFORM_ROOT)/../../../soc/gs/include
ccflags-y += -I$(WLAN_PLATFORM_ROOT)/cnss_utils
ccflags-y += -DCONFIG_WCN_GOOGLE
ccflags-y += -DCONFIG_QCOM_RAMDUMP -DCONFIG_QCOM_MEMORY_DUMP_V2
else
ccflags-y += -I$(srctree)/drivers/net/wireless/cnss_utils/
endif
cnss2-y := main.o
cnss2-y += bus.o
cnss2-y += debug.o
cnss2-y += pci.o
cnss2-y += power.o
cnss2-y += genl.o
cnss2-y += qcom_ramdump.o
cnss2-$(CONFIG_SOC_GOOGLE) += pci_platform_google.o
cnss2-$(CONFIG_CNSS2_QMI) += qmi.o coexistence_service_v01.o ip_multimedia_subsystem_private_service_v01.o
