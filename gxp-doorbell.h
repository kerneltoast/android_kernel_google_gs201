/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GXP doorbell interface.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_DOORBELL_H__
#define __GXP_DOORBELL_H__

#include "gxp-internal.h"

/**
 * gxp_doorbell_enable_for_core() - Enable a core to receive interrupts from
 *                                  the specified doorbell
 * @gxp: The GXP device to enable the doorbell for
 * @doorbell_num: The index of the doorbell to enable
 * @core: The core which will receive interrupts from the enabled doorbell
 *
 * This function can only be called before firmware is running on @core or as
 * part of halting firmware on it. Firmware may read or modify which doorbells
 * it is listening to and accesses are not synchronized between the driver and
 * firmware.
 */
void gxp_doorbell_enable_for_core(struct gxp_dev *gxp, u32 doorbell_num,
				  uint core);

/**
 * gxp_doorbell_set() - Ring the specified doorbell
 * @gxp: The GXP device to ring the doorbell for
 * @doorbell_num: The index of the doorbell to ring
 *
 * Any cores this doorbell is enabled for willexecute their handler.
 */
void gxp_doorbell_set(struct gxp_dev *gxp, u32 doorbell_num);

/**
 * gxp_doorbell_clear() - Clear the specified doorbell's interrupt
 * @gxp: The GXP device to clear the doorbell for
 * @doorbell_num: The index of the doorbell to clear
 */
void gxp_doorbell_clear(struct gxp_dev *gxp, u32 doorbell_num);

/**
 * gxp_doorbell_status() - Returns whether the specified doorbell is rung
 * @gxp: The GXP device containing the doorbell
 * @doorbell_num: The index of the doorbell to check
 *
 * Return: The status register of the doorbell, 1 for rung, 0 for cleared
 */
u32 gxp_doorbell_status(struct gxp_dev *gxp, u32 doorbell_num);

#endif /* __GXP_DOORBELL_H__ */
