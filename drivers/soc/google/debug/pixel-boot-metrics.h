/* SPDX-License-Identifier: GPL-2.0 */
/*
 * boot_metrics.h - Get boot metrics information
 *
 * Copyright 2021 Google LLC
 */

#ifndef BOOT_METRICS_H
#define BOOT_METRICS_H

#define BOOT_METRICS_MAX	30

enum metrics_phase_type_t {
	METRICS_PHASE_BL1 = 0x1,
	METRICS_PHASE_PBL,
	METRICS_PHASE_BL2,
	METRICS_PHASE_EL3,
	METRICS_PHASE_MAX
};

enum metrics_data_type_t {
	METRICS_DATA_LOAD = 0x0,
	METRICS_DATA_WAIT,
	METRICS_DATA_CHECK,
	METRICS_DATA_MAX
};

struct metrics_header_t {
	unsigned int index;
	unsigned int maximum;
	unsigned int metrics[];
} __packed;

#define METRICS_PHASE_TYPE_MASK	0x7U
#define METRICS_PHASE_TYPE_SHIFT	29
#define METRICS_TYPE_MASK		0xFU
#define METRICS_TYPE_SHIFT		25
#define METRICS_DATA_TYPE_MASK		0x3U
#define METRICS_DATA_TYPE_SHIFT	23
#define METRICS_DATA_MASK		0x7FFFFFU
#define METRICS_DATA_SHIFT		0
#define METRICS_DATA_ALIGN		3
#define METRICS_DATA_MAX_VALUE		(METRICS_DATA_MASK << METRICS_DATA_ALIGN)
#define METRICS_DATA_OVERFLOW		(METRICS_DATA_MAX_VALUE + 1)

enum bl1_metrics_type_t {
	METRICS_BL1_ENTRY = 0x1,
	METRICS_PBL_LOAD,
	/* Add normal boot metrics here */

	METRICS_BL1_SLEEP_GO_START = 0xA,
	/* Add SleepGo metrics here */

	METRICS_BL1_EXIT = 0xF
};

enum pbl_metrics_type_t {
	METRICS_PBL_ENTRY = 0x1,
	METRICS_BL2_LOAD,
	METRICS_TZSW_LOAD,
	METRICS_LDFW_LOAD,
	METRICS_BL31_LOAD,
	METRICS_GSA_COPY_DIARY_START,
	METRICS_GSA_COPY_DIARY_END,
	/* Add normal boot metrics here */

	METRICS_PBL_SLEEP_GO_START = 0xA,
	/* Add SleepGo metrics here */

	METRICS_PBL_EXIT = 0xF,
};

enum bl2_metrics_type_t {
	METRICS_BL2_ENTRY = 0x1,
	METRICS_ECT_LOAD,
	METRICS_ACPM_LOAD,
	METRICS_ACPM_ENABLE_START,
	METRICS_ACPM_ENABLE_END,
	METRICS_DRAM_TRAINING_LOAD,
	METRICS_ACPM_INIT_START,
	METRICS_ACPM_INIT_END,
#if defined(CONFIG_SOC_GS101) || defined(CONFIG_SOC_GS201)
	METRICS_GSA_LOAD,
#else
	METRICS_GSABL1_LOAD,
	METRICS_GSAFW_LOAD,
#endif
	METRICS_UBOOT_LOAD,
	METRICS_HYPERVISOR_LOAD,
	/* Add normal boot metrics here */

	METRICS_BL2_WARMBOOT_START = 0xE,
	/* Add warmboot metrics here */

	METRICS_BL2_EXIT = 0xF,
};

enum el3_metrics_type_t {
	METRICS_EL3_ENTRY = 0x1,
	/* Add normal boot metrics here */

	METRICS_EL3_PSCI_SYSTEM_SUSPEND_START = 0x5,
	METRICS_EL3_MON_SMC_SLEEP_START,
	METRICS_EL3_MON_SMC_SLEEP_END,
	/* Add sleep metrics here */

	METRICS_EL3_SLEEP_GO_START = 0xA,
	METRICS_EL3_MON_SMC_WARMBOOT_START,
	METRICS_EL3_MON_SMC_WARMBOOT_END,
	/* Add wake metrics here */

	METRICS_EL3_EXIT = 0xF
};

#endif /* BOOT_METRICS_H */
