/*
 * ALSA SoC Texas Instruments TAS25XX High Performance 4W Smart Amplifier
 *
 * Copyright (C) 2021 Texas Instruments, Inc.
 *
 * Author: Vijeth P O
 * Modified: 10-07-21
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#ifndef __TAS25XX_REGBIN_PARSER__
#define __TAS25XX_REGBIN_PARSER__

#define SampleRate_48000	0
#define SampleRate_44100	1
#define SampleRate_96000	2

#define FMT_INV_NB_NF	0
#define FMT_INV_IB_NF	1
#define FMT_INV_NB_IF	2
#define FMT_INV_IB_IF	3

#define FMT_MASK_I2S	0
#define FMT_MASK_DSP_A	1
#define FMT_MASK_DSP_B	2
#define FMT_MASK_LEFT_J	3

#define RX_SLOTS_16	0
#define RX_SLOTS_24	1
#define RX_SLOTS_32	2

#define TX_SLOTS_16	0
#define TX_SLOTS_24	1
#define TX_SLOTS_32	2

#define RX_BITWIDTH_16	0
#define RX_BITWIDTH_24	1
#define RX_BITWIDTH_32	2

#define RX_SLOTLEN_16	0
#define RX_SLOTLEN_24	1
#define RX_SLOTLEN_32	2

#define TX_SLOTLEN_16	0
#define TX_SLOTLEN_24	1
#define TX_SLOTLEN_32	2

#define SUPPORTED_BIN_VERSION 7

#define TAS25XX_DEFAULT	0xFFFFFFFF

#define TAS25XX_EM_REG		0x03
#define TAS25XX_EM_SHIFT	6
#define TAS25XX_EM_MASK		0xC0

enum kcntl_during_t {
	KCNTR_ANYTIME = 0, /* instant update */
	KCNTR_PRE_POWERUP = 1, /* during pre-power up */
	KCNTR_POST_POWERUP = 2, /* during post-power up */
};

struct default_hw_params {
	char sample_rate;
	char fmt_inv;
	char fmt_mask;
	char rx_slots;
	char tx_slots;
	char rx_bitwidth;
	char rx_slotlen;
	char tx_slotlen;
};

struct tas_dev {
	u8 device;
	u8 i2c_addr;
};

struct bin_header {
	u32 version;
	u8 name[64];
	u32 timestamp;
	u32 size;
	u32 channels;
	struct tas_dev dev[MAX_CHANNELS];
	u8 iv_width;
	u8 vbat_mon;
	u32 features;
} __attribute__((packed));

int32_t tas25xx_load_firmware(struct tas25xx_priv *p_tas25xx, int max_retry_count);
int32_t tas25xx_create_kcontrols(struct tas25xx_priv *p_tas25xx);
int32_t tas25xx_remove_binfile(struct tas25xx_priv *p_tas25xx);
int32_t tas25xx_set_init_params(struct tas25xx_priv *p_tas25xx, int32_t ch);
int32_t tas25xx_set_sample_rate(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t sample_rate);
int32_t tas25xx_set_fmt_inv(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t fmt_inv);
int32_t tas25xx_set_fmt_mask(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t fmt_mask);
int32_t tas25xx_set_rx_slots(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t rx_slot);
int32_t tas25xx_set_tx_slots(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t tx_slot);
int32_t tas25xx_set_rx_bitwidth(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t rx_bitwidth);
int32_t tas25xx_set_rx_slotlen(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t rx_slotlen);
int32_t tas25xx_set_tx_slotlen(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t tx_slotlen);
int32_t tas25xx_set_pre_powerup(struct tas25xx_priv *p_tas25xx, int32_t ch);
int32_t tas25xx_set_post_powerup(struct tas25xx_priv *p_tas25xx, int32_t ch);
int32_t tas25xx_set_pre_powerdown(struct tas25xx_priv *p_tas25xx, int32_t ch);
int32_t tas25xx_set_post_powerdown(struct tas25xx_priv *p_tas25xx, int32_t ch);

int32_t tas25xx_process_block(struct tas25xx_priv *p_tas25xx, char *mem, int32_t chn);

int32_t tas25xx_check_if_powered_on(struct tas25xx_priv *p_tas25xx, int *state, int ch);
int tas_write_init_config_params(struct tas25xx_priv *p_tas25xx, int number_of_channels);

int32_t tas25xx_update_kcontrol_data(struct tas25xx_priv *p_tas25xx, enum kcntl_during_t cur_state,
	uint32_t chmask);

int32_t tas25xx_update_playback_volume(struct tas25xx_priv *p_tas25xx, int32_t ch);

void tas25xx_prep_dev_for_calib(int start);
#endif /* __TAS25XX_REGBIN_PARSER__ */
