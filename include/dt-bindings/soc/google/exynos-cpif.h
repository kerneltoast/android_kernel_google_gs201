/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019-2020 Samsung Electronics Co., Ltd.
 *
 * Device Tree binding constants for Exynos CP interface
 */

#ifndef _DT_BINDINGS_EXYNOS_CPIF_H
#define _DT_BINDINGS_EXYNOS_CPIF_H

/* CP model */
#define SEC_S5000AP		0
#define SEC_S5100		1
#define MODEM_TYPE_DUMMY	2
#define MAX_MODEM_TYPE		3

/* CP protocol */
#define PROTOCOL_SIPC	0
#define PROTOCOL_SIT	1
#define MAX_PROTOCOL	2

/* SIPC version */
#define NO_SIPC_VER	0
#define SIPC_VER_40	40
#define SIPC_VER_41	41
#define SIPC_VER_42	42
#define SIPC_VER_50	50
#define MAX_SIPC_VER	51

/* IPC device format */
#define IPC_FMT		0
#define IPC_RAW		1
#define IPC_RFS		2
#define IPC_MULTI_RAW	3
#define IPC_BOOT		4
#define IPC_DUMP		5
#define IPC_CMD		6
#define IPC_DEBUG	7
#define MAX_DEV_FORMAT	8

/* Link device type */
#define LINKDEV_SHMEM		0
#define LINKDEV_PCIE		1
#define LINKDEV_UNDEFINED	2
#define LINKDEV_MAX		3

/* Interrupt type */
#define INTERRUPT_MAILBOX	0
#define INTERRUPT_GPIO		1
#define INTERRUPT_MAX		2

/* Control msg type */
#define MAILBOX_SR		1
#define DRAM_V1		2
#define DRAM_V2		3
#define GPIO			4
#define MAX_CMSG_TYPE		5

/* Mailbox interrupt */
#define CP_MBOX_IRQ_IDX_0	0
#define CP_MBOX_IRQ_IDX_1	1
#define CP_MBOX_IRQ_IDX_2	2
#define CP_MBOX_IRQ_IDX_3	3
#define CP_MBOX_IRQ_IDX_4	4
#define MAX_CP_MBOX_IRQ_IDX	5

/* PCIe MSI interrupt */
#define PCIE_CP_IRQ_IDX_0	0
#define PCIE_CP_IRQ_IDX_1	1
#define PCIE_CP_IRQ_IDX_2	2
#define PCIE_CP_IRQ_IDX_3	3
#define PCIE_CP_IRQ_IDX_4	4
#define PCIE_CP_IRQ_IDX_5	5
#define MAX_PCIE_CP_IRQ_IDX	6

/* IO device type */
#define IODEV_BOOTDUMP	0
#define IODEV_IPC		1
#define IODEV_NET		2
#define IODEV_DUMMY		3

/* Shared memory index */
#define SHMEM_CP		0
#define SHMEM_VSS		1
#define SHMEM_L2B		2
#define SHMEM_IPC		3
#define SHMEM_VPA		4
#define SHMEM_BTL		5
#define SHMEM_PKTPROC		6
#define SHMEM_PKTPROC_UL	7
#define SHMEM_ZMC		8
#define SHMEM_C2C		9
#define SHMEM_MSI		10
#define MAX_CP_SHMEM		11

/* TPMON measure */
#define TPMON_MEASURE_TP	0
#define TPMON_MEASURE_NETDEV_Q	1
#define TPMON_MEASURE_PKTPROC_DL_Q	2
#define TPMON_MEASURE_DIT_SRC_Q	3

/* TPMON target */
#define TPMON_TARGET_RPS	0
#define TPMON_TARGET_GRO	1
#define TPMON_TARGET_MIF	2
#define TPMON_TARGET_PCIE_LOW_POWER	3
#define TPMON_TARGET_IRQ_MBOX	4
#define TPMON_TARGET_IRQ_PCIE	5
#define TPMON_TARGET_IRQ_DIT	6
#define TPMON_TARGET_INT_FREQ	7
#define TPMON_TARGET_BTS 8
#define TPMON_TARGET_CPU_CL0	9
#define TPMON_TARGET_CPU_CL1	10
#define TPMON_TARGET_CPU_CL2	11
#define TPMON_TARGET_MIF_MAX	12
#define TPMON_TARGET_INT_FREQ_MAX	13
#define TPMON_TARGET_CPU_CL0_MAX	14
#define TPMON_TARGET_CPU_CL1_MAX	15
#define TPMON_TARGET_CPU_CL2_MAX	16
#define MAX_TPMON_TARGET	17

/* Protocol for TPMON */
#define TPMON_PROTO_ALL	0
#define TPMON_PROTO_TCP	1
#define TPMON_PROTO_UDP	2
#define TPMON_PROTO_OTHERS	3
#define MAX_TPMON_PROTO	4

/* Link device attr */
#define LINK_ATTR_SBD_IPC		(0x1 << 0) /* IPC over SBD (from MIPI-LLI) */
#define LINK_ATTR_IPC_ALIGNED		(0x1 << 1) /* IPC with 4-bytes alignment */
#define LINK_ATTR_IOSM_MESSAGE		(0x1 << 2) /* IOSM message */
#define LINK_ATTR_DPRAM_MAGIC		(0x1 << 3) /* DPRAM-style magic code validation */
#define LINK_ATTR_SBD_BOOT		(0x1 << 4) /* BOOT over SBD */
#define LINK_ATTR_SBD_DUMP		(0x1 << 5) /* DUMP over SBD */
#define LINK_ATTR_MEM_BOOT		(0x1 << 6) /* BOOT over legacy memory-type I/F */
#define LINK_ATTR_MEM_DUMP		(0x1 << 7) /* DUMP over legacy memory-type I/F */
#define LINK_ATTR_BOOT_ALIGNED		(0x1 << 8) /* BOOT with 4-bytes alignment */
#define LINK_ATTR_DUMP_ALIGNED		(0x1 << 9) /* DUMP with 4-bytes alignment */
#define LINK_ATTR_XMIT_BTDLR		(0x1 << 10) /* Used to download CP bootloader */
#define LINK_ATTR_XMIT_BTDLR_SPI	(0x1 << 11) /* Download CP bootloader by SPI */

/* IO device attr */
#define IO_ATTR_SIPC4			(0x1 << 0)
#define IO_ATTR_SIPC5			(0x1 << 1)
#define IO_ATTR_CDC_NCM		(0x1 << 2)
#define IO_ATTR_MULTIFMT		(0x1 << 3)
#define IO_ATTR_HANDOVER		(0x1 << 4)
#define IO_ATTR_LEGACY_RFS		(0x1 << 5)
#define IO_ATTR_RX_FRAGMENT		(0x1 << 6)
#define IO_ATTR_SBD_IPC		(0x1 << 7) /* IPC using SBD designed from MIPI-LLI */
#define IO_ATTR_NO_LINK_HEADER		(0x1 << 8) /* Link-layer header is not needed */
#define IO_ATTR_NO_CHECK_MAXQ		(0x1 << 9) /* no need to check rxq overflow condition */
#define IO_ATTR_DUALSIM		(0x1 << 10) /* support Dual SIM */
#define IO_ATTR_OPTION_REGION		(0x1 << 11) /* region & operator info */
#define IO_ATTR_ZEROCOPY		(0x1 << 12) /* support SW zerocopy on SBD */

/* SIPC channel ID */
#define SIPC_CH_ID_RAW_0		0
#define SIPC_CH_ID_CS_VT_DATA		1
#define SIPC_CH_ID_CS_VT_CONTROL	2
#define SIPC_CH_ID_CS_VT_AUDIO		3
#define SIPC_CH_ID_CS_VT_VIDEO		4
#define SIPC_CH_ID_RAW_5		5
#define SIPC_CH_ID_RAW_6		6
#define SIPC_CH_ID_CDMA_DATA		7
#define SIPC_CH_ID_PCM_DATA		8
#define SIPC_CH_ID_TRANSFER_SCREEN	9

#define SIPC_CH_ID_PDP_0	10
#define SIPC_CH_ID_PDP_1	11
#define SIPC_CH_ID_PDP_2	12
#define SIPC_CH_ID_PDP_3	13
#define SIPC_CH_ID_PDP_4	14
#define SIPC_CH_ID_PDP_5	15
#define SIPC_CH_ID_PDP_6	16
#define SIPC_CH_ID_PDP_7	17
#define SIPC_CH_ID_PDP_8	18
#define SIPC_CH_ID_PDP_9	19
#define SIPC_CH_ID_PDP_10	20
#define SIPC_CH_ID_PDP_11	21
#define SIPC_CH_ID_PDP_12	22
#define SIPC_CH_ID_PDP_13	23
#define SIPC_CH_ID_PDP_14	24
#define SIPC_CH_ID_BT_DUN	25
#define SIPC_CH_ID_CIQ_DATA	26
#define SIPC_CH_ID_PDP_17	27
#define SIPC_CH_ID_CPLOG1	28
#define SIPC_CH_ID_CPLOG2	29
#define SIPC_CH_ID_LOOPBACK1	30
#define SIPC_CH_ID_LOOPBACK2	31

#define SIPC_CH_ID_SMD4	33

#define SIPC_CH_ID_CASS	35

#define SIPC5_CH_ID_BOOT_0	215
#define SIPC5_CH_ID_BOOT_1	216
#define SIPC5_CH_ID_BOOT_2	217
#define SIPC5_CH_ID_BOOT_3	218
#define SIPC5_CH_ID_BOOT_4	219
#define SIPC5_CH_ID_BOOT_5	220
#define SIPC5_CH_ID_BOOT_6	221
#define SIPC5_CH_ID_BOOT_7	222
#define SIPC5_CH_ID_BOOT_8	223
#define SIPC5_CH_ID_BOOT_9	224

#define SIPC5_CH_ID_DUMP_0	225
#define SIPC5_CH_ID_DUMP_1	226
#define SIPC5_CH_ID_DUMP_2	227
#define SIPC5_CH_ID_DUMP_3	228
#define SIPC5_CH_ID_DUMP_4	229
#define SIPC5_CH_ID_DUMP_5	230
#define SIPC5_CH_ID_DUMP_6	231
#define SIPC5_CH_ID_DUMP_7	232
#define SIPC5_CH_ID_DUMP_8	233
#define SIPC5_CH_ID_DUMP_9	234

#define SIPC5_CH_ID_FMT_0	235
#define SIPC5_CH_ID_FMT_1	236
#define SIPC5_CH_ID_FMT_2	237
#define SIPC5_CH_ID_FMT_3	238
#define SIPC5_CH_ID_FMT_4	239
#define SIPC5_CH_ID_FMT_5	240
#define SIPC5_CH_ID_FMT_6	241
#define SIPC5_CH_ID_FMT_7	242
#define SIPC5_CH_ID_FMT_8	243
#define SIPC5_CH_ID_FMT_9	244

#define SIPC5_CH_ID_RFS_0	245
#define SIPC5_CH_ID_RFS_1	246
#define SIPC5_CH_ID_RFS_2	247
#define SIPC5_CH_ID_RFS_3	248
#define SIPC5_CH_ID_RFS_4	249
#define SIPC5_CH_ID_RFS_5	250
#define SIPC5_CH_ID_RFS_6	251
#define SIPC5_CH_ID_RFS_7	252
#define SIPC5_CH_ID_RFS_8	253
#define SIPC5_CH_ID_RFS_9	254

#define SIPC5_CH_ID_MAX	255

/* SIT channel ID */
#define EXYNOS_CH_ID_MULTIPDP	0
#define EXYNOS_CH_ID_PDP_0	1
#define EXYNOS_CH_ID_PDP_1	2
#define EXYNOS_CH_ID_PDP_2	3
#define EXYNOS_CH_ID_PDP_3	4
#define EXYNOS_CH_ID_PDP_4	5
#define EXYNOS_CH_ID_PDP_5	6
#define EXYNOS_CH_ID_PDP_6	7
#define EXYNOS_CH_ID_PDP_7	8
#define EXYNOS_CH_ID_PDP_8	9
#define EXYNOS_CH_ID_PDP_9	10
#define EXYNOS_CH_ID_PDP_10	11
#define EXYNOS_CH_ID_PDP_11	12
#define EXYNOS_CH_ID_PDP_12	13
#define EXYNOS_CH_ID_PDP_13	14
#define EXYNOS_CH_ID_PDP_14	15
#define EXYNOS_CH_ID_PDP_15	16

#define EXYNOS_CH_ID_BT_DUN	21 /* umts_router */

#define EXYNOS_CH_ID_UTS	23 /* umts_atc0 */

#define EXYNOS_CH_ID_EMBMS_0	30
#define EXYNOS_CH_ID_EMBMS_1	31

#define EXYNOS_CH_ID_RFS_0	41 /*umts_rfs0 */
#define EXYNOS_CH_ID_RFS_1	42
#define EXYNOS_CH_ID_RFS_2	43
#define EXYNOS_CH_ID_RFS_3	44
#define EXYNOS_CH_ID_RFS_4	45
#define EXYNOS_CH_ID_RFS_5	46
#define EXYNOS_CH_ID_RFS_6	47
#define EXYNOS_CH_ID_RFS_7	48
#define EXYNOS_CH_ID_RFS_8	49
#define EXYNOS_CH_ID_RFS_9	50

#define EXYNOS_CH_ID_CPLOG	81 /* umts_dm0 */
#define EXYNOS_CH_ID_LOOPBACK	82 /* umts_loopback */

#define EXYNOS_CH_ID_RCS_0	91
#define EXYNOS_CH_ID_RCS_1	92

#define EXYNOS_CH_ID_WFS_0	93 /* umts_wfc0 */
#define EXYNOS_CH_ID_WFS_1	94 /* umts_wfc1 */

#define EXYNOS_CH_ID_OEM_0	129 /* oem_ipc0 */
#define EXYNOS_CH_ID_OEM_1	130 /* oem_ipc1 */
#define EXYNOS_CH_ID_OEM_2	131 /* oem_ipc2 */
#define EXYNOS_CH_ID_OEM_3	132 /* oem_ipc3 */
#define EXYNOS_CH_ID_OEM_4	133 /* oem_ipc4 */
#define EXYNOS_CH_ID_OEM_5	134 /* oem_ipc5 */
#define EXYNOS_CH_ID_OEM_6	135 /* oem_ipc6 */
#define EXYNOS_CH_ID_OEM_7	136 /* oem_ipc7 */

#define EXYNOS_CH_ID_BOOT	241
#define EXYNOS_CH_ID_DUMP	242

#define EXYNOS_CH_ID_FMT_0	245 /* umts_ipc0 */
#define EXYNOS_CH_ID_FMT_1	246 /* umts_ipc1 */

#define EXYNOS_CH_ID_MAX	255


#endif /* _DT_BINDINGS_EXYNOS_CPIF_H */
