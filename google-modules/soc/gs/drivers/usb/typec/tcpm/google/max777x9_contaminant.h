// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 Google LLC
 *
 */
#ifndef __MAX777x9_CONTAMINANT_H
#define __MAX777x9_CONTAMINANT_H

#include <linux/extcon.h>
#include <linux/types.h>

#define MAXQ_DETECT_TYPE_CC_AND_SBU     0x10
#define MAXQ_DETECT_TYPE_SBU_ONLY       0x30

#define READ1_SLEEP_MS                  10
#define READ2_SLEEP_MS                  5

/* To be kept in sync with TCPC_VENDOR_ADC_CTRL1.ADCINSEL */
enum adc_select {
	CC1_SCALE1 = 1,
	CC1_SCALE2,
	CC2_SCALE1,
	CC2_SCALE2,
	SBU1,
	SBU2,
};

enum contamiant_state {
	NOT_DETECTED,
	DETECTED,
	FLOATING_CABLE,
	SINK,
	DISABLED,
};

struct max77759_plat;

struct max777x9_contaminant {
	struct max77759_plat *chip;
	enum contamiant_state state;
	bool auto_ultra_low_power_mode_disabled;
	bool contaminant_detect_maxq;
	bool is_max77779;
};

int maxq_query_contaminant(u8 cc1_raw, u8 cc2_raw, u8 sbu1_raw, u8 sbu2_raw,
			   u8 cc1_rd, u8 cc2_rd, u8 type, u8 cc_adc_skipped,
			   u8 *response, u8 length);
int __attribute__((weak)) maxq_query_contaminant(u8 cc1_raw, u8 cc2_raw, u8 sbu1_raw, u8 sbu2_raw,
						 u8 cc1_rd, u8 cc2_rd, u8 type, u8 cc_adc_skipped,
						 u8 *response, u8 length)
{
	return -EINVAL;
}

struct max77759_plat;
struct max777x9_contaminant *max777x9_contaminant_init(struct max77759_plat *plat, bool enable,
						       bool is_max77779);
int max777x9_process_contaminant_alert(struct max777x9_contaminant *contaminant, bool debounce_path,
				       bool tcpm_toggling, bool *cc_status_handled,
				       bool *port_clean);
int max777x9_enable_contaminant_detection(struct max77759_plat *chip, bool maxq);
int max777x9_disable_contaminant_detection(struct max77759_plat *chip);
bool max777x9_is_contaminant_detected(struct max77759_plat *chip);
bool max777x9_is_floating_cable_or_sink_detected(struct max77759_plat *chip);
void max777x9_disable_auto_ultra_low_power_mode(struct max77759_plat *chip, bool disable);

static inline bool status_check(u8 reg, u8 mask, u8 val)
{
	return ((reg) & (mask)) == (val);
}
#endif /* __MAX777x9_CONTAMINANT_H */
