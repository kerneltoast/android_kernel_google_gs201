/*
 * =============================================================================
 * Copyright (c) 2016  Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details
 *
 * File:
 *     tas25xx-algo-intf.h
 *
 * Description:
 *     Header file for Wrapper on interface to algorithm.
 *
 * =============================================================================
 */
#ifndef __TAS25XX_ALGO_INTF__
#define __TAS25XX_ALGO_INTF__

enum module_id_t {
	TISA_MOD_RX = 0,
	TISA_MOD_TX = 1,
	TISA_MOD_END = 2
};

void tas25xx_smartamp_alg_intf_init(void);
void tas25xx_smartamp_alg_intf_deinit(void);
int tas25xx_smartamp_algo_ctrl(uint8_t *user_data, uint32_t param_id,
		uint8_t get_set, uint32_t length, enum module_id_t module_id);

/* bypass set and get */
int tas25xx_get_algo_bypass(void);
void tas25xx_set_algo_bypass(int bypass);

void tas25xx_smartamp_set_card(struct snd_soc_card *card);
int tas25xx_smartamp_get_set(uint8_t *user_data, uint32_t param_id,
		uint8_t get_set, uint32_t length, enum module_id_t mod_id_i);

#endif /* __TAS25XX_ALGO_INTF__ */

