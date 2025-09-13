// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 Google LLC
 *
 * MAX77759 contaminant detection specific helpers
 */

#ifndef __MAX77759_CONTAMINANT_H
#define __MAX77759_CONTAMINANT_H

#include <misc/logbuffer.h>

#include "max777x9_contaminant.h"
#include "tcpci_max77759.h"

enum adc_select;
int max77759_enable_contaminant_detection(struct max77759_plat *chip);
int max77759_disable_contaminant_detection(struct max77759_plat *chip);
void max77759_disable_auto_ultra_low_power_mode(struct max77759_plat *chip, bool disable);
int max77759_detect_contaminant(struct max777x9_contaminant *contaminant);
int max77759_enable_dry_detection(struct max777x9_contaminant *contaminant);
int max77759_read_comparators(struct max777x9_contaminant *contaminant, u8 *vendor_cc_status2_cc1,
			      u8 *vendor_cc_status2_cc2);
int max77759_read_resistance_kohm(struct max777x9_contaminant *contaminant,
				  enum adc_select channel, int sleep_msec,
				  bool raw);
#endif /* __MAX77779_CONTAMINANT_H */
