/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Samsung Electronics Co., Ltd.
 */

#ifndef ACPM_FRAMEWORK

struct cpu_inform pmucal_cpuinform_list[] = {
	PMUCAL_CPU_INFORM(0, 0x15460000, 0x0860),
	PMUCAL_CPU_INFORM(1, 0x15460000, 0x0864),
	PMUCAL_CPU_INFORM(2, 0x15460000, 0x0868),
	PMUCAL_CPU_INFORM(3, 0x15460000, 0x086c),
	PMUCAL_CPU_INFORM(4, 0x15460000, 0x0870),
	PMUCAL_CPU_INFORM(5, 0x15460000, 0x0874),
	PMUCAL_CPU_INFORM(6, 0x15460000, 0x0878),
	PMUCAL_CPU_INFORM(7, 0x15460000, 0x087c),
	PMUCAL_CPU_INFORM(8, 0x15460000, 0x0880),
};
unsigned int cpu_inform_list_size = ARRAY_SIZE(pmucal_cpuinform_list);

/* Kernel O/S side */
/* individual sequence descriptor for each core - on, off, status */
struct pmucal_seq core00_on[] = {
};
struct pmucal_seq core00_off[] = {
};
struct pmucal_seq core00_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU0_STATUS", 0x15460000, 0x1004, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core01_on[] = {
};
struct pmucal_seq core01_off[] = {
};
struct pmucal_seq core01_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU0_STATUS", 0x15460000, 0x1084, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core02_on[] = {
};
struct pmucal_seq core02_off[] = {
};
struct pmucal_seq core02_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU0_STATUS", 0x15460000, 0x1104, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core03_on[] = {
};
struct pmucal_seq core03_off[] = {
};
struct pmucal_seq core03_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU0_STATUS", 0x15460000, 0x1184, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core10_on[] = {
};
struct pmucal_seq core10_off[] = {
};
struct pmucal_seq core10_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU0_STATUS", 0x15460000, 0x1304, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core11_on[] = {
};
struct pmucal_seq core11_off[] = {
};
struct pmucal_seq core11_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU0_STATUS", 0x15460000, 0x1384, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core12_on[] = {
};
struct pmucal_seq core12_off[] = {
};
struct pmucal_seq core12_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU0_STATUS", 0x15460000, 0x1404, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core13_on[] = {
};
struct pmucal_seq core13_off[] = {
};
struct pmucal_seq core13_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU0_STATUS", 0x15460000, 0x1484, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq core20_on[] = {
};
struct pmucal_seq core20_off[] = {
};
struct pmucal_seq core20_status[] = {
	PMUCAL_SEQ_DESC(PMUCAL_READ, "CLUSTER0_CPU0_STATUS", 0x15460000, 0x1604, (0x1 << 0), 0, 0, 0, 0xffffffff, 0),
};
struct pmucal_seq cluster0_status[] = {
};
struct pmucal_seq cluster1_status[] = {
};
struct pmucal_seq cluster2_status[] = {
};

enum pmucal_cpu_corenum {
	CPU_CORE00,
	CPU_CORE01,
	CPU_CORE02,
	CPU_CORE03,
	CPU_CORE10,
	CPU_CORE11,
	CPU_CORE12,
	CPU_CORE13,
	CPU_CORE20,
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
	[CPU_CORE12] = {
		.id = CPU_CORE12,
		.release = 0,
		.on = core12_on,
		.off = core12_off,
		.status = core12_status,
		.num_release = 0,
		.num_on = ARRAY_SIZE(core12_on),
		.num_off = ARRAY_SIZE(core12_off),
		.num_status = ARRAY_SIZE(core10_status),
	},
	[CPU_CORE13] = {
		.id = CPU_CORE13,
		.release = 0,
		.on = core13_on,
		.off = core13_off,
		.status = core13_status,
		.num_release = 0,
		.num_on = ARRAY_SIZE(core13_on),
		.num_off = ARRAY_SIZE(core13_off),
		.num_status = ARRAY_SIZE(core13_status),
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
		.on = 0,
		.off = 0,
		.status = cluster0_status,
		.num_on = 0,
		.num_off = 0,
		.num_status = ARRAY_SIZE(cluster0_status),
	},
	[CPU_CLUSTER1] = {
		.id = CPU_CLUSTER1,
		.on = 0,
		.off = 0,
		.status = cluster1_status,
		.num_on = 0,
		.num_off = 0,
		.num_status = ARRAY_SIZE(cluster1_status),
	},
	[CPU_CLUSTER2] = {
		.id = CPU_CLUSTER2,
		.on = 0,
		.off = 0,
		.status = cluster2_status,
		.num_on = 0,
		.num_off = 0,
		.num_status = ARRAY_SIZE(cluster2_status),
	},
};
unsigned int pmucal_cluster_list_size = ARRAY_SIZE(pmucal_cluster_list);

enum pmucal_opsnum {
	NUM_PMUCAL_OPTIONS,
};

struct pmucal_cpu pmucal_pmu_ops_list[] = {};

unsigned int pmucal_option_list_size;

#else
/* ACPM F/W side */
/* individual sequence descriptor for each core - on, off, status */
#endif
