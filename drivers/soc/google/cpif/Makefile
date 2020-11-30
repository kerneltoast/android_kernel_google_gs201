# SPDX-License-Identifier: GPL-2.0
# Makefile of cpif

EXTRA_CFLAGS += -Idrivers/soc/samsung/cpif \
		-DCONFIG_OPTION_REGION=\"$(PROJECT_REGION)\"

obj-$(CONFIG_MCU_IPC) += mcu_ipc.o
obj-$(CONFIG_SHM_IPC) += shm_ipc.o
obj-$(CONFIG_BOOT_DEVICE_SPI) += boot_device_spi.o

obj-$(CONFIG_EXYNOS_DIT) += exynos_dit.o
exynos_dit-y += dit.o dit_net.o dit_hal.o

obj-$(CONFIG_EXYNOS_MODEM_IF) += cpif.o
cpif-y += modem_main.o modem_variation.o
cpif-y += modem_io_device.o net_io_device.o bootdump_io_device.o ipc_io_device.o
cpif-y += modem_utils.o modem_dump.o

cpif-y += link_device.o link_device_memory_flow_control.o
cpif-y += link_device_memory_debug.o modem_notifier.o
cpif-y += link_device_memory_snapshot.o link_device_memory_sbd.o link_device_memory_legacy.o

cpif-$(CONFIG_LINK_DEVICE_PCIE) += s51xx_pcie.o

cpif-$(CONFIG_SEC_MODEM_S5000AP) += modem_ctrl_s5000ap.o modem_ctrl.o
cpif-$(CONFIG_SEC_MODEM_S5100) += modem_ctrl_s5100.o modem_ctrl.o

cpif-$(CONFIG_CP_ZEROCOPY) += link_rx_zerocopy.o
cpif-$(CONFIG_CP_PKTPROC) += link_rx_pktproc.o
cpif-$(CONFIG_CP_PKTPROC_UL) += link_tx_pktproc.o

cpif-$(CONFIG_LINK_CONTROL_MSG_IOSM) += link_ctrlmsg_iosm.o

cpif-$(CONFIG_CP_BTL) += cp_btl.o

cpif-$(CONFIG_CPIF_TP_MONITOR) += cpif_tp_monitor.o

cpif-$(CONFIG_MODEM_IF_LEGACY_QOS) += cpif_qos_info.o

ifeq ($(CONFIG_RPS), y)
cpif-$(CONFIG_ARGOS) += modem_argos_notifier.o
endif

obj-$(CONFIG_CP_THERMAL) += cp_thermal_zone.o
