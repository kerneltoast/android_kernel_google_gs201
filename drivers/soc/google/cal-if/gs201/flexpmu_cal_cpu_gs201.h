/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 */

struct cpu_inform pmucal_cpuinform_list[] = {
	PMUCAL_CPU_INFORM(0, 0x18060000, 0x860),
	PMUCAL_CPU_INFORM(1, 0x18060000, 0x864),
	PMUCAL_CPU_INFORM(2, 0x18060000, 0x868),
	PMUCAL_CPU_INFORM(3, 0x18060000, 0x86C),
	PMUCAL_CPU_INFORM(4, 0x18060000, 0x870),
	PMUCAL_CPU_INFORM(5, 0x18060000, 0x874),
	PMUCAL_CPU_INFORM(6, 0x18060000, 0x878),
	PMUCAL_CPU_INFORM(7, 0x18060000, 0x87C),
};
unsigned int cpu_inform_list_size = ARRAY_SIZE(pmucal_cpuinform_list);
#ifndef ACPM_FRAMEWORK
/* individual sequence descriptor for each core - on, off, status */
struct pmucal_seq core00_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 0), (0x0 << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP2_INTR_BID_CLEAR", 0x18070000, 0x020c, (0x1 << 0), (0x1 << 0), 0x18070000, 0x0208, (0x1 << 0), (0x1 << 0) | (0x1 << 0)),
};

struct pmucal_seq core00_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 0), (0x1 << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 0), (0x1 << 0), 0x18070000, 0x0108, (0x1 << 0), (0x1 << 0) | (0x1 << 0)),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 8), (0x1 << 8), 0x18070000, 0x0108, (0x1 << 8), (0x1 << 8) | (0x1 << 8)),
};

struct pmucal_seq core00_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU0_STATUS", 0x18060000, 0x1004, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core01_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 1), (0x0 << 1), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP2_INTR_BID_CLEAR", 0x18070000, 0x020c, (0x1 << 1), (0x1 << 1), 0x18070000, 0x0208, (0x1 << 1), (0x1 << 1) | (0x1 << 1)),
};

struct pmucal_seq core01_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 1), (0x1 << 1), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 1), (0x1 << 1), 0x18070000, 0x0108, (0x1 << 1), (0x1 << 1) | (0x1 << 1)),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 9), (0x1 << 9), 0x18070000, 0x0108, (0x1 << 9), (0x1 << 9) | (0x1 << 9)),
};

struct pmucal_seq core01_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU1_STATUS", 0x18060000, 0x1084, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core02_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 2), (0x0 << 2), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP2_INTR_BID_CLEAR", 0x18070000, 0x020c, (0x1 << 2), (0x1 << 2), 0x18070000, 0x0208, (0x1 << 2), (0x1 << 2) | (0x1 << 2)),
};

struct pmucal_seq core02_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 2), (0x1 << 2), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 2), (0x1 << 2), 0x18070000, 0x0108, (0x1 << 2), (0x1 << 2) | (0x1 << 2)),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 10), (0x1 << 10), 0x18070000, 0x0108, (0x1 << 10), (0x1 << 10) | (0x1 << 10)),
};

struct pmucal_seq core02_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU2_STATUS", 0x18060000, 0x1104, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core03_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 3), (0x0 << 3), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP2_INTR_BID_CLEAR", 0x18070000, 0x020c, (0x1 << 3), (0x1 << 3), 0x18070000, 0x0208, (0x1 << 3), (0x1 << 3) | (0x1 << 3)),
};

struct pmucal_seq core03_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 3), (0x1 << 3), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 3), (0x1 << 3), 0x18070000, 0x0108, (0x1 << 3), (0x1 << 3) | (0x1 << 3)),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 11), (0x1 << 11), 0x18070000, 0x0108, (0x1 << 11), (0x1 << 11) | (0x1 << 11)),
};

struct pmucal_seq core03_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU3_STATUS", 0x18060000, 0x1184, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core10_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP2_INTR_BID_CLEAR", 0x18070000, 0x020c, (0x1 << 4), (0x1 << 4), 0x18070000, 0x0208, (0x1 << 4), (0x1 << 4) | (0x1 << 4)),
};

struct pmucal_seq core10_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 4), (0x1 << 4), 0x18070000, 0x0108, (0x1 << 4), (0x1 << 4) | (0x1 << 4)),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 12), (0x1 << 12), 0x18070000, 0x0108, (0x1 << 12), (0x1 << 12) | (0x1 << 12)),
};

struct pmucal_seq core10_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER1_CPU0_STATUS", 0x18060000, 0x1304, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core11_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 5), (0x0 << 5), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP2_INTR_BID_CLEAR", 0x18070000, 0x020c, (0x1 << 5), (0x1 << 5), 0x18070000, 0x0208, (0x1 << 5), (0x1 << 5) | (0x1 << 5)),
};

struct pmucal_seq core11_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 5), (0x1 << 5), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 5), (0x1 << 5), 0x18070000, 0x0108, (0x1 << 5), (0x1 << 5) | (0x1 << 5)),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 13), (0x1 << 13), 0x18070000, 0x0108, (0x1 << 13), (0x1 << 13) | (0x1 << 13)),
};

struct pmucal_seq core11_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER1_CPU1_STATUS", 0x18060000, 0x1384, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core20_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 6), (0x0 << 6), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP2_INTR_BID_CLEAR", 0x18070000, 0x020c, (0x1 << 6), (0x1 << 6), 0x18070000, 0x0208, (0x1 << 6), (0x1 << 6) | (0x1 << 6)),
};

struct pmucal_seq core20_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 6), (0x1 << 6), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 6), (0x1 << 6), 0x18070000, 0x0108, (0x1 << 6), (0x1 << 6) | (0x1 << 6)),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 14), (0x1 << 14), 0x18070000, 0x0108, (0x1 << 14), (0x1 << 14) | (0x1 << 14)),
};

struct pmucal_seq core20_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER2_CPU0_STATUS", 0x18060000, 0x1504, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core21_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 7), (0x0 << 7), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP2_INTR_BID_CLEAR", 0x18070000, 0x020c, (0x1 << 7), (0x1 << 7), 0x18070000, 0x0208, (0x1 << 7), (0x1 << 7) | (0x1 << 7)),
};

struct pmucal_seq core21_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "GRP2_INTR_BID_ENABLE", 0x18070000, 0x0200, (0x1 << 7), (0x1 << 7), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 7), (0x1 << 7), 0x18070000, 0x0108, (0x1 << 7), (0x1 << 7) | (0x1 << 7)),
	PMUCAL_SEQ_DESC(PMUCAL_CLEAR_PEND, "GRP1_INTR_BID_CLEAR", 0x18070000, 0x010c, (0x1 << 15), (0x1 << 15), 0x18070000, 0x0108, (0x1 << 15), (0x1 << 15) | (0x1 << 15)),
};

struct pmucal_seq core21_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER2_CPU1_STATUS", 0x18060000, 0x1584, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq cluster0_on[] = {
};

struct pmucal_seq cluster0_off[] = {
};

struct pmucal_seq cluster0_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_NONCPU_STATUS", 0x18060000, 0x1204, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq cluster1_on[] = {
};

struct pmucal_seq cluster1_off[] = {
};

struct pmucal_seq cluster1_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER1_NONCPU_STATUS", 0x18060000, 0x1404, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq cluster2_on[] = {
};

struct pmucal_seq cluster2_off[] = {
};

struct pmucal_seq cluster2_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER2_NONCPU_STATUS", 0x18060000, 0x1604, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

enum pmucal_cpu_corenum {
	CPU_CORE00,
	CPU_CORE01,
	CPU_CORE02,
	CPU_CORE03,
	CPU_CORE10,
	CPU_CORE11,
	CPU_CORE20,
	CPU_CORE21,
	PMUCAL_NUM_CORES,
};

struct pmucal_cpu pmucal_cpu_list[PMUCAL_NUM_CORES] = {
	[CPU_CORE00] = {
		.id = CPU_CORE00,
		.release = 0,
		.on = core00_on,
		.off = core00_off,
		.status = core00_status,
		.num_release = 0,
		.num_on = ARRAY_SIZE(core00_on),
		.num_off = ARRAY_SIZE(core00_off),
		.num_status = ARRAY_SIZE(core00_status),
	},
	[CPU_CORE01] = {
		.id = CPU_CORE01,
		.release = 0,
		.on = core01_on,
		.off = core01_off,
		.status = core01_status,
		.num_release = 0,
		.num_on = ARRAY_SIZE(core01_on),
		.num_off = ARRAY_SIZE(core01_off),
		.num_status = ARRAY_SIZE(core01_status),
	},
	[CPU_CORE02] = {
		.id = CPU_CORE02,
		.release = 0,
		.on = core02_on,
		.off = core02_off,
		.status = core02_status,
		.num_release = 0,
		.num_on = ARRAY_SIZE(core02_on),
		.num_off = ARRAY_SIZE(core02_off),
		.num_status = ARRAY_SIZE(core02_status),
	},
	[CPU_CORE03] = {
		.id = CPU_CORE03,
		.release = 0,
		.on = core03_on,
		.off = core03_off,
		.status = core03_status,
		.num_release = 0,
		.num_on = ARRAY_SIZE(core03_on),
		.num_off = ARRAY_SIZE(core03_off),
		.num_status = ARRAY_SIZE(core03_status),
	},
	[CPU_CORE10] = {
		.id = CPU_CORE10,
		.release = 0,
		.on = core10_on,
		.off = core10_off,
		.status = core10_status,
		.num_release = 0,
		.num_on = ARRAY_SIZE(core10_on),
		.num_off = ARRAY_SIZE(core10_off),
		.num_status = ARRAY_SIZE(core10_status),
	},
	[CPU_CORE11] = {
		.id = CPU_CORE11,
		.release = 0,
		.on = core11_on,
		.off = core11_off,
		.status = core11_status,
		.num_release = 0,
		.num_on = ARRAY_SIZE(core11_on),
		.num_off = ARRAY_SIZE(core11_off),
		.num_status = ARRAY_SIZE(core11_status),
	},
	[CPU_CORE20] = {
		.id = CPU_CORE20,
		.release = 0,
		.on = core20_on,
		.off = core20_off,
		.status = core20_status,
		.num_release = 0,
		.num_on = ARRAY_SIZE(core20_on),
		.num_off = ARRAY_SIZE(core20_off),
		.num_status = ARRAY_SIZE(core20_status),
	},
	[CPU_CORE21] = {
		.id = CPU_CORE21,
		.release = 0,
		.on = core21_on,
		.off = core21_off,
		.status = core21_status,
		.num_release = 0,
		.num_on = ARRAY_SIZE(core21_on),
		.num_off = ARRAY_SIZE(core21_off),
		.num_status = ARRAY_SIZE(core21_status),
	},
};

unsigned int pmucal_cpu_list_size = ARRAY_SIZE(pmucal_cpu_list);

enum pmucal_cpu_clusternum {
	CPU_CLUSTER0,
	CPU_CLUSTER1,
	CPU_CLUSTER2,
	PMUCAL_NUM_CLUSTERS,
};

struct pmucal_cpu pmucal_cluster_list[PMUCAL_NUM_CLUSTERS] = {
	[CPU_CLUSTER0] = {
		.id = CPU_CLUSTER0,
		.on = cluster0_on,
		.off = cluster0_off,
		.status = cluster0_status,
		.num_on = ARRAY_SIZE(cluster0_on),
		.num_off = ARRAY_SIZE(cluster0_off),
		.num_status = ARRAY_SIZE(cluster0_status),
	},
	[CPU_CLUSTER1] = {
		.id = CPU_CLUSTER1,
		.on = cluster1_on,
		.off = cluster1_off,
		.status = cluster1_status,
		.num_on = ARRAY_SIZE(cluster1_on),
		.num_off = ARRAY_SIZE(cluster1_off),
		.num_status = ARRAY_SIZE(cluster1_status),
	},
	[CPU_CLUSTER2] = {
		.id = CPU_CLUSTER2,
		.on = cluster2_on,
		.off = cluster2_off,
		.status = cluster2_status,
		.num_on = ARRAY_SIZE(cluster2_on),
		.num_off = ARRAY_SIZE(cluster2_off),
		.num_status = ARRAY_SIZE(cluster2_status),
	},
};

unsigned int pmucal_cluster_list_size = ARRAY_SIZE(pmucal_cluster_list);

enum pmucal_opsnum {
	NUM_PMUCAL_OPTIONS,
};

struct pmucal_cpu pmucal_pmu_ops_list[] = {};

unsigned int pmucal_option_list_size = ARRAY_SIZE(pmucal_pmu_ops_list);

#else

struct pmucal_seq core00_release[] = {
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_cpu_reset_release", 0, 0, 0x20, 0x0, 0, 0, 0, 0),
};

struct pmucal_seq core00_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_cpu_up", 0, 0, 0x21, 0x0, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core00_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_cpu_down", 0, 0, 0x22, 0x0, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core00_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU0_STATUS", 0x18060000, 0x1004, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core01_release[] = {
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_cpu_reset_release", 0, 0, 0x20, 0x1, 0, 0, 0, 0),
};

struct pmucal_seq core01_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_cpu_up", 0, 0, 0x21, 0x1, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core01_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_cpu_down", 0, 0, 0x22, 0x1, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core01_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU1_STATUS", 0x18060000, 0x1084, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core02_release[] = {
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_cpu_reset_release", 0, 0, 0x20, 0x2, 0, 0, 0, 0),
};

struct pmucal_seq core02_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_cpu_up", 0, 0, 0x21, 0x2, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core02_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_cpu_down", 0, 0, 0x22, 0x2, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core02_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU2_STATUS", 0x18060000, 0x1104, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core03_release[] = {
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_cpu_reset_release", 0, 0, 0x20, 0x3, 0, 0, 0, 0),
};

struct pmucal_seq core03_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_cpu_up", 0, 0, 0x21, 0x3, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core03_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_cpu_down", 0, 0, 0x22, 0x3, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core03_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU3_STATUS", 0x18060000, 0x1184, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core10_release[] = {
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster1_cpu_reset_release", 0, 0, 0x30, 0x0, 0, 0, 0, 0),
};

struct pmucal_seq core10_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL1_HCHGEN_CLKMUX_CPU", 0x20c10000, 0x0854, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster1_cpu_up", 0, 0, 0x31, 0x0, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL1_HCHGEN_CLKMUX_CPU", 0x20c10000, 0x0854, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core10_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL1_HCHGEN_CLKMUX_CPU", 0x20c10000, 0x0854, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster1_cpu_down", 0, 0, 0x32, 0x0, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL1_HCHGEN_CLKMUX_CPU", 0x20c10000, 0x0854, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core10_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER1_CPU0_STATUS", 0x18060000, 0x1304, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core11_release[] = {
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster1_cpu_reset_release", 0, 0, 0x30, 0x1, 0, 0, 0, 0),
};

struct pmucal_seq core11_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL1_HCHGEN_CLKMUX_CPU", 0x20c10000, 0x0854, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster1_cpu_up", 0, 0, 0x31, 0x1, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL1_HCHGEN_CLKMUX_CPU", 0x20c10000, 0x0854, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core11_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL1_HCHGEN_CLKMUX_CPU", 0x20c10000, 0x0854, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster1_cpu_down", 0, 0, 0x32, 0x1, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL1_HCHGEN_CLKMUX_CPU", 0x20c10000, 0x0854, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core11_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER1_CPU1_STATUS", 0x18060000, 0x1384, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core20_release[] = {
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster2_cpu_reset_release", 0, 0, 0x40, 0x0, 0, 0, 0, 0),
};

struct pmucal_seq core20_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL2_HCHGEN_CLKMUX_CPU", 0x20c20000, 0x0854, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster2_cpu_up", 0, 0, 0x41, 0x0, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL2_HCHGEN_CLKMUX_CPU", 0x20c20000, 0x0854, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core20_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL2_HCHGEN_CLKMUX_CPU", 0x20c20000, 0x0854, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster2_cpu_down", 0, 0, 0x42, 0x0, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL2_HCHGEN_CLKMUX_CPU", 0x20c20000, 0x0854, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core20_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER2_CPU0_STATUS", 0x18060000, 0x1504, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core21_release[] = {
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster2_cpu_reset_release", 0, 0, 0x40, 0x1, 0, 0, 0, 0),
};

struct pmucal_seq core21_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL2_HCHGEN_CLKMUX_CPU", 0x20c20000, 0x0854, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster2_cpu_up", 0, 0, 0x41, 0x1, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL2_HCHGEN_CLKMUX_CPU", 0x20c20000, 0x0854, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core21_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL2_HCHGEN_CLKMUX_CPU", 0x20c20000, 0x0854, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster2_cpu_down", 0, 0, 0x42, 0x1, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL2_HCHGEN_CLKMUX_CPU", 0x20c20000, 0x0854, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq core21_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER2_CPU1_STATUS", 0x18060000, 0x1584, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq cluster0_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_noncpu_up", 0, 0, 0x28, 0x0, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "QCH_CON_CLUSTER0_QCH_SCLK", 0x20c00000, 0x3144, (0x7 << 0), (0x6 << 0), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq cluster0_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "QCH_CON_CLUSTER0_QCH_SCLK", 0x20c00000, 0x3144, (0x7 << 0), (0x2 << 0), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster0_noncpu_down", 0, 0, 0x29, 0x0, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL0_HCHGEN_CLKMUX_CPU", 0x20c00000, 0x0844, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq cluster0_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_NONCPU_STATUS", 0x18060000, 0x1204, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq cluster1_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL1_HCHGEN_CLKMUX_CPU", 0x20c10000, 0x0854, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster1_noncpu_up", 0, 0, 0x38, 0x0, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL1_HCHGEN_CLKMUX_CPU", 0x20c10000, 0x0854, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq cluster1_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL1_HCHGEN_CLKMUX_CPU", 0x20c10000, 0x0854, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster1_noncpu_down", 0, 0, 0x39, 0x0, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL1_HCHGEN_CLKMUX_CPU", 0x20c10000, 0x0854, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq cluster1_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER1_NONCPU_STATUS", 0x18060000, 0x1404, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

struct pmucal_seq cluster2_on[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL2_HCHGEN_CLKMUX_CPU", 0x20c20000, 0x0854, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster2_noncpu_up", 0, 0, 0x48, 0x0, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL2_HCHGEN_CLKMUX_CPU", 0x20c20000, 0x0854, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq cluster2_off[] = {
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL2_HCHGEN_CLKMUX_CPU", 0x20c20000, 0x0854, (0x1 << 4), (0x1 << 4), 0, 0, 0xffffffff, 0),
	PMUCAL_SEQ_DESC(PMUCAL_EXT_FUNC, "cluster2_noncpu_down", 0, 0, 0x49, 0x0, 0, 0, 0, 0),
	PMUCAL_SEQ_DESC(PMUCAL_WRITE, "CPUCL2_HCHGEN_CLKMUX_CPU", 0x20c20000, 0x0854, (0x1 << 4), (0x0 << 4), 0, 0, 0xffffffff, 0),
};

struct pmucal_seq cluster2_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER2_NONCPU_STATUS", 0x18060000, 0x1604, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};

enum pmucal_cpu_corenum {
	CPU_CORE00,
	CPU_CORE01,
	CPU_CORE02,
	CPU_CORE03,
	CPU_CORE10,
	CPU_CORE11,
	CPU_CORE20,
	CPU_CORE21,
	PMUCAL_NUM_CORES,
};

struct pmucal_cpu pmucal_cpu_list[PMUCAL_NUM_CORES] = {
	[CPU_CORE00] = {
		.id = CPU_CORE00,
		.release = core00_release,
		.on = core00_on,
		.off = core00_off,
		.status = core00_status,
		.num_release = ARRAY_SIZE(core00_release),
		.num_on = ARRAY_SIZE(core00_on),
		.num_off = ARRAY_SIZE(core00_off),
		.num_status = ARRAY_SIZE(core00_status),
	},
	[CPU_CORE01] = {
		.id = CPU_CORE01,
		.release = core01_release,
		.on = core01_on,
		.off = core01_off,
		.status = core01_status,
		.num_release = ARRAY_SIZE(core01_release),
		.num_on = ARRAY_SIZE(core01_on),
		.num_off = ARRAY_SIZE(core01_off),
		.num_status = ARRAY_SIZE(core01_status),
	},
	[CPU_CORE02] = {
		.id = CPU_CORE02,
		.release = core02_release,
		.on = core02_on,
		.off = core02_off,
		.status = core02_status,
		.num_release = ARRAY_SIZE(core02_release),
		.num_on = ARRAY_SIZE(core02_on),
		.num_off = ARRAY_SIZE(core02_off),
		.num_status = ARRAY_SIZE(core02_status),
	},
	[CPU_CORE03] = {
		.id = CPU_CORE03,
		.release = core03_release,
		.on = core03_on,
		.off = core03_off,
		.status = core03_status,
		.num_release = ARRAY_SIZE(core03_release),
		.num_on = ARRAY_SIZE(core03_on),
		.num_off = ARRAY_SIZE(core03_off),
		.num_status = ARRAY_SIZE(core03_status),
	},
	[CPU_CORE10] = {
		.id = CPU_CORE10,
		.release = core10_release,
		.on = core10_on,
		.off = core10_off,
		.status = core10_status,
		.num_release = ARRAY_SIZE(core10_release),
		.num_on = ARRAY_SIZE(core10_on),
		.num_off = ARRAY_SIZE(core10_off),
		.num_status = ARRAY_SIZE(core10_status),
	},
	[CPU_CORE11] = {
		.id = CPU_CORE11,
		.release = core11_release,
		.on = core11_on,
		.off = core11_off,
		.status = core11_status,
		.num_release = ARRAY_SIZE(core11_release),
		.num_on = ARRAY_SIZE(core11_on),
		.num_off = ARRAY_SIZE(core11_off),
		.num_status = ARRAY_SIZE(core11_status),
	},
	[CPU_CORE20] = {
		.id = CPU_CORE20,
		.release = core20_release,
		.on = core20_on,
		.off = core20_off,
		.status = core20_status,
		.num_release = ARRAY_SIZE(core20_release),
		.num_on = ARRAY_SIZE(core20_on),
		.num_off = ARRAY_SIZE(core20_off),
		.num_status = ARRAY_SIZE(core20_status),
	},
	[CPU_CORE21] = {
		.id = CPU_CORE21,
		.release = core21_release,
		.on = core21_on,
		.off = core21_off,
		.status = core21_status,
		.num_release = ARRAY_SIZE(core21_release),
		.num_on = ARRAY_SIZE(core21_on),
		.num_off = ARRAY_SIZE(core21_off),
		.num_status = ARRAY_SIZE(core21_status),
	},
};

unsigned int pmucal_cpu_list_size = ARRAY_SIZE(pmucal_cpu_list);

enum pmucal_cpu_clusternum {
	CPU_CLUSTER0,
	CPU_CLUSTER1,
	CPU_CLUSTER2,
	PMUCAL_NUM_CLUSTERS,
};

struct pmucal_cpu pmucal_cluster_list[PMUCAL_NUM_CLUSTERS] = {
	[CPU_CLUSTER0] = {
		.id = CPU_CLUSTER0,
		.on = cluster0_on,
		.off = cluster0_off,
		.status = cluster0_status,
		.num_on = ARRAY_SIZE(cluster0_on),
		.num_off = ARRAY_SIZE(cluster0_off),
		.num_status = ARRAY_SIZE(cluster0_status),
	},
	[CPU_CLUSTER1] = {
		.id = CPU_CLUSTER1,
		.on = cluster1_on,
		.off = cluster1_off,
		.status = cluster1_status,
		.num_on = ARRAY_SIZE(cluster1_on),
		.num_off = ARRAY_SIZE(cluster1_off),
		.num_status = ARRAY_SIZE(cluster1_status),
	},
	[CPU_CLUSTER2] = {
		.id = CPU_CLUSTER2,
		.on = cluster2_on,
		.off = cluster2_off,
		.status = cluster2_status,
		.num_on = ARRAY_SIZE(cluster2_on),
		.num_off = ARRAY_SIZE(cluster2_off),
		.num_status = ARRAY_SIZE(cluster2_status),
	},
};

unsigned int pmucal_cluster_list_size = ARRAY_SIZE(pmucal_cluster_list);

enum pmucal_opsnum {
	NUM_PMUCAL_OPTIONS,
};

struct pmucal_cpu pmucal_pmu_ops_list[NUM_PMUCAL_OPTIONS] = {
};
unsigned int pmucal_option_list_size = ARRAY_SIZE(pmucal_pmu_ops_list);

struct cpu_info cpuinfo[] = {
	[0] = {
		.min = 0,
		.max = 3,
		.total = 4,
	},
	[1] = {
		.min = 4,
		.max = 5,
		.total = 2,
	},
	[2] = {
		.min = 6,
		.max = 7,
		.total = 2,
	},
};
#endif
