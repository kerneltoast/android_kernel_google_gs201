/*
 * ALSA SoC Texas Instruments TAS25XX High Performance 4W Smart Amplifier
 *
 * Copyright (C) 2022 Texas Instruments, Inc.
 *
 * Author: Niranjan H Y, Vijeth P O
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
#ifndef __TAS25XX__
#define __TAS25XX__

#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <sound/soc.h>
#include <linux/version.h>

#define TAS25XX_DRIVER_TAG  "UDA_0.0.20_K6.1"

#define MAX_CHANNELS	2

/* Page Control Register */
#define TAS25XX_PAGECTL_REG  0

/* Book Control Register (available in page0 of each book) */
#define TAS25XX_BOOKCTL_PAGE  0
#define TAS25XX_BOOKCTL_REG  127

#define TAS25XX_REG(book, page, reg)  (((book * 256 * 128) + \
					(page * 128)) + reg)

#define TAS25XX_BOOK_ID(reg)  (reg / (256 * 128))
#define TAS25XX_PAGE_ID(reg)  ((reg % (256 * 128)) / 128)
#define TAS25XX_BOOK_REG(reg)  (reg % (256 * 128))
#define TAS25XX_PAGE_REG(reg)  ((reg % (256 * 128)) % 128)
#define TAS25XX_REG_NO_BOOK(reg) (reg - (TAS25XX_BOOK_ID(reg) * 256 * 128))

/* Book */
#define TAS25XX_BOOK  TAS25XX_REG(0x0, 0x0, 0x7F)
#define TAS25XX_BOOK_BOOK70_MASK  (0xff << 0)

/* Rev Info */
#define TAS25XX_REVID_REG TAS25XX_REG(0x0, 0x0, 0x78)

/* data format */
#define TAS25XX_DATAFORMAT_SHIFT  2
#define TAS25XX_DATAFORMAT_I2S  0x0
#define TAS25XX_DATAFORMAT_DSP  0x1
#define TAS25XX_DATAFORMAT_RIGHT_J  0x2
#define TAS25XX_DATAFORMAT_LEFT_J  0x3

#define TAS25XX_DAI_FMT_MASK  (0x7 << TAS25XX_DATAFORMAT_SHIFT)


#define ERROR_NONE  0x0000000
#define ERROR_PLL_ABSENT  0x0000001
#define ERROR_DEVA_I2C_COMM  0x0000002
#define ERROR_DEVB_I2C_COMM  0x0000004
#define ERROR_CLOCK  0x0000008
#define ERROR_YRAM_CRCCHK  0x0000010
#define ERROR_OVER_CURRENT  0x0000020
#define ERROR_DIE_OVERTEMP  0x0000040
#define ERROR_OVER_VOLTAGE  0x0000080
#define ERROR_UNDER_VOLTAGE  0x0000100
#define ERROR_BROWNOUT  0x0000200
#define ERROR_CLASSD_PWR  0x0000400
#define ERROR_FAILSAFE  0x0000800

#define TAS25XX_IRQ_DET_TIMEOUT		30000
#define TAS25XX_IRQ_DET_CNT_LIMIT	500

/* 5 second */
#define CHECK_PERIOD  5000

#define TAS25XX_I2C_RETRY_COUNT  3
#define ERROR_I2C_SUSPEND  -1
#define ERROR_I2C_FAILED  -2

#define TAS25XX_SWITCH  0x10000001
#define TAS25XX_RIGHT_SWITCH  0x10000002
#define RX_SCFG_LEFT  0x10000003
#define RX_SCFG_RIGHT  0x10000004

#define RESTART_MAX  3
#define MAX_CMD_LIST 5 /* sysfs cmd nodes */

struct tas25xx_priv;
struct snd_soc_component;

/* REGBIN related */
#define TAS25XX_CONFIG_SIZE  (10)
#define TAS25XX_DEVICE_SUM  (8)

#define TAS25XX_CMD_SING_W  (0x1)
#define TAS25XX_CMD_BURST  (0x2)
#define TAS25XX_CMD_DELAY  (0x3)
#define TAS25XX_CMD_FIELD_W  (0x4)

#define SMS_HTONS(a, b)  ((((a)&0x00FF)<<8) | \
				((b)&0x00FF))
#define SMS_HTONL(a, b, c, d) ((((a)&0x000000FF)<<24) |\
					(((b)&0x000000FF)<<16) | \
					(((c)&0x000000FF)<<8) | \
					((d)&0x000000FF))


#define CMD_SINGLE_WRITE	0
#define CMD_BURST_WRITES	1
#define CMD_UPDATE_BITS		2
#define CMD_DELAY		3

#define CMD_SINGLE_WRITE_SZ 6
#define CMD_BURST_WRITES_SZ 9
#define CMD_UPDATE_BITS_SZ  7
#define CMD_DELAY_SZ		5

#define DSP_FW_LOAD_NTRIES  20

#define INTERRUPT_TYPE_CLOCK_BASED (1<<0)
#define INTERRUPT_TYPE_NON_CLOCK_BASED (1<<1)

enum tas_power_states_t {
	TAS_POWER_ACTIVE = 0,
	TAS_POWER_MUTE = 1,
	TAS_POWER_SHUTDOWN = 2,
};

enum tas25xx_dsp_fw_state {
	TAS25XX_DSP_FW_NONE = 0,
	TAS25XX_DSP_FW_TRYLOAD,
	TAS25XX_DSP_FW_PARSE_FAIL,
	TAS25XX_DSP_FW_LOAD_FAIL,
	TAS25XX_DSP_FW_OK,
};

enum tas25xx_bin_blk_type {
	TAS25XX_BIN_BLK_COEFF = 1,
	TAS25XX_BIN_BLK_POST_POWER_UP,
	TAS25XX_BIN_BLK_PRE_SHUTDOWN,
	TAS25XX_BIN_BLK_PRE_POWER_UP,
	TAS25XX_BIN_BLK_POST_SHUTDOWN
};

enum tas_int_action_t {
	TAS_INT_ACTION_NOACTION = 0,
	TAS_INT_ACTION_SW_RESET = 1 << 0,
	TAS_INT_ACTION_HW_RESET = 1 << 1,
	TAS_INT_ACTION_POWER_ON = 1 << 2,
};

struct tas25xx_block_data {
	unsigned char dev_idx;
	unsigned char block_type;
	unsigned short yram_checksum;
	unsigned int block_size;
	unsigned int nSublocks;
	unsigned char *regdata;
};

struct tas25xx_config_info {
	unsigned int nblocks;
	unsigned int real_nblocks;
	struct tas25xx_block_data **blk_data;
};

void tas25xx_select_cfg_blk(void *pContext, int conf_no,
	unsigned char block_type);
int tas25xx_load_container(struct tas25xx_priv *pTAS256x);
void tas25xx_config_info_remove(void *pContext);

struct tas25xx_register {
	int book;
	int page;
	int reg;
};

/* Used for Register Dump */
struct tas25xx_reg {
	unsigned int reg_index;
	unsigned int reg_val;
};

struct tas25xx_dai_cfg {
unsigned int dai_fmt;
unsigned int tdm_delay;
};

/*struct tas25xx_buf_cfg {
 *	unsigned short bufSize;
 *	unsigned char *buf;
 *};
 */

//This should be removed or modified
enum ch_bitmask {
	channel_left = 0x1 << 0,
	channel_right = 0x1 << 1,
	channel_both = channel_left|channel_right
};

/*
 * device ops function structure
 */
struct tas_device_ops {
	/**< init typically for loading optimal settings */
	int (*tas_init)(struct tas25xx_priv *p_tas25xx, int chn);
	int (*tas_probe)(struct tas25xx_priv *p_tas25xx,
		struct snd_soc_component *codec, int chn);
/*TODO:*/
};


struct tas_device {
	int mn_chip_id;
	int mn_current_book;
	int mn_addr;
	int reset_gpio;
	int irq_gpio;
	int irq_no;
	int device_id;
	int channel_no;
	int rx_mode;
	int dvc_pcm;
	int bst_vltg;
	int bst_ilm;
	int ampoutput_lvl;
	int lim_switch;
	int lim_max_attn;
	int lim_thr_max;
	int lim_thr_min;
	int lim_att_rate;
	int lim_rel_rate;
	int lim_att_stp_size;
	int lim_rel_stp_size;
	int lim_max_thd;
	int lim_min_thd;
	int lim_infl_pt;
	int lim_trk_slp;
	int bop_enable;
	int bop_thd;
	int bop_att_rate;
	int bop_att_stp_size;
	int bop_hld_time;
	int bop_mute;
	int bosd_enable;
	int bosd_thd;
	int vbat_lpf;
	int rx_cfg;
	int classh_timer;
	int reciever_enable;
	int icn_sw;
	int icn_thr;
	int icn_hyst;
	struct tas_device_ops dev_ops;
	struct delayed_work init_work;
	struct tas25xx_priv *prv_data;
	/* interrupt count since interrupt enable */
	uint16_t irq_count;
	unsigned long jiffies;
	/* Fail Safe */
	unsigned int mn_restart;
};

struct tas25xx_intr_info {
	char name[64];
	int32_t reg;
	int32_t mask;
	int32_t action;
	int32_t detected;
	int32_t is_clock_based;
	uint32_t count;
	uint64_t count_persist;
	struct device_attribute *dev_attr;
};

struct tas25xx_interrupts {
	uint8_t count;
	uint8_t *buf_intr_enable;
	uint8_t *buf_intr_disable;
	uint8_t *buf_intr_clear;
	struct tas25xx_intr_info *intr_info;
	uint32_t processing_delay;
};

struct tas_regbin_read_blok_t {
	int32_t reg;
	int32_t mask;
	int32_t value;
};

struct tas_block_op_data_t {
	uint32_t no_of_rx_blks;
	uint32_t no_of_tx_blks;
	uint8_t *sw_reset;
	uint8_t *power_check;
	uint8_t *mute;
	uint8_t *cal_init;
	uint8_t *cal_deinit;
	uint8_t *rx_fmt_data;
	uint8_t *tx_fmt_data;
};

#if IS_ENABLED(CONFIG_TAS25XX_IRQ_BD)
struct irq_bigdata {
	struct device_attribute *p_dev_attr;
	struct attribute **p_attr_arr;
	struct device *irq_dev;
};
#endif

struct cmd_data {
	struct device_attribute *p_dev_attr;
	struct attribute **p_attr_arr;
	struct device *cmd_dev;
};

struct tas25xx_priv {
	struct linux_platform *platform_data;
	struct kobject *k_obj;
	int m_power_state;
	int mn_frame_size;
	int mn_ppg;
	int mn_ch_size;
	int mn_rx_width;
	int mn_tx_slot_width;
	int sample_rate;
	int mn_iv_width;
	int curr_mn_iv_width;
	int mn_vbat;
	int curr_mn_vbat;
	int ch_count;
	int ch_failed[MAX_CHANNELS];
	int mn_slots;
	int mn_rx_slot_map[2];
	unsigned int mn_fmt;
	int mn_fmt_mode;
	int mn_frame_start;
	int mn_rx_edge;
	int mn_rx_offset;
	int mn_tx_edge;
	int mn_tx_offset;
	int dac_power;	/* this is set based on the DAC events */
	struct tas_device **devs;
	int (*read)(struct tas25xx_priv *p_tas25xx, int32_t chn,
		unsigned int reg, unsigned int *pValue);
	int (*write)(struct tas25xx_priv *p_tas25xx, int32_t chn,
		unsigned int reg, unsigned int Value);
	int (*bulk_read)(struct tas25xx_priv *p_tas25xx, int32_t chn,
		unsigned int reg, unsigned char *p_data, unsigned int len);
	int (*bulk_write)(struct tas25xx_priv *p_tas25xx, int32_t chn,
		unsigned int reg, unsigned char *p_data, unsigned int len);
	int (*update_bits)(struct tas25xx_priv *p_tas25xx, int32_t chn,
		unsigned int reg, unsigned int mask, unsigned int value);
	void (*hw_reset)(struct tas25xx_priv *p_tas25xx);
	void (*clear_irq)(struct tas25xx_priv *p_tas25xx);
	void (*enable_irq)(struct tas25xx_priv *p_tas25xx);
	void (*disable_irq)(struct tas25xx_priv *p_tas25xx);
	unsigned int mn_err_code;
	int (*plat_write)(void *plat_data,
		unsigned int i2c_addr, unsigned int reg, unsigned int value,
		unsigned int channel);
	int (*plat_bulk_write)(void *plat_data, unsigned int i2c_addr,
		unsigned int reg, unsigned char *pData,
		unsigned int nLength, unsigned int channel);
	int (*plat_read)(void *plat_data, unsigned int i2c_addr,
		unsigned int reg, unsigned int *value, unsigned int channel);
	int (*plat_bulk_read)(void *plat_data, unsigned int i2c_addr,
		unsigned int reg, unsigned char *pData,
		unsigned int nLength, unsigned int channel);
	int (*plat_update_bits)(void *plat_data, unsigned int i2c_addr,
		unsigned int reg, unsigned int mask,
		unsigned int value, unsigned int channel);
	void (*schedule_init_work)(struct tas25xx_priv *p_tas25xx, int ch);
	void (*cancel_init_work) (struct tas25xx_priv *p_tas25xx, int ch);
	struct mutex dev_lock;
	struct mutex codec_lock;
	struct mutex file_lock;
	struct delayed_work irq_work;
	struct delayed_work dc_work;
	int iv_enable;
	uint32_t dev_revid;
	uint32_t fw_size;
	uint8_t *fw_data;
	struct delayed_work post_fw_load_work;
	struct delayed_work fw_load_work;
	wait_queue_head_t fw_wait;
	int fw_load_retry_count;
	atomic_t fw_state;
	atomic_t fw_wait_complete;
	wait_queue_head_t dev_init_wait;
	atomic_t dev_init_status;
	int device_used;
	int irq_registered[MAX_CHANNELS];
	int irq_enabled[MAX_CHANNELS];
	struct tas25xx_interrupts intr_data[MAX_CHANNELS];
#if IS_ENABLED(CONFIG_TAS25XX_IRQ_BD)
	struct irq_bigdata irqdata;
#endif
	struct class *class;
	struct cmd_data cmd_data;
	struct tas_block_op_data_t block_op_data[MAX_CHANNELS];
	int32_t playback_volume_left_index;
	int32_t playback_volume_right_index;
	int irqz_value;
	int left_em_mode;
	int right_em_mode;
};

static inline int is_power_up_state(enum tas_power_states_t state)
{
	if (state == TAS_POWER_ACTIVE)
		return 1;

	return 0;
}

#endif /* __TAS25XX__ */

