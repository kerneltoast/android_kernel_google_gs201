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

#include <linux/firmware.h>
#include <linux/wait.h>
#include <linux/i2c.h>
#include "inc/tas25xx.h"
#include "inc/tas25xx-regmap.h"
#include "inc/tas25xx-regbin-parser.h"
#include "inc/tas25xx-device.h"
#include "algo/inc/tas_smart_amp_v2.h"

#define TAS25XX_BINFILE_NAME	"tismartpa_driver_tuning.bin"

#define MDATA_HDR_SZ 9
#define ONE_BIN_MD_SZ 20
#define HDR_STR_SZ	5
#define MAIN_BLOCK_SIZE 5
#define ANY_CHANNEL 0xffffffff
#define MDATA "MDATA"
#define HEADER "HEADR"
#define INITP_STR "INITP"
#define INTRP_STR  "INTRP"
#define HW_PRMS_STR "HWPRM"
#define PROFP_STR   "PROFP"
#define KCNTRL_STR  "KCNTR"
#define BLK_OP_STR  "BLKOP"
#define ALGOP_STR   "ALGOP"

enum block_types_t {
	BLK_SW_RST,
	BLK_POWER_CHECK,
	BLK_MUTE,
	BLK_CAL_INIT,
	BLK_CAL_DEINIT,
	BLK_RX_FMT,
	BLK_TX_FMT,
	BLK_INVALID,
};

const char *SampleRate[3] = { "48000", "44100", "96000"};
const char *FMT_INV[4] = { "NB_NF", "IB_NF", "NB_IF", "IB_IF" };
const char *FMT_MASK[4] = { "I2S", "DSP_A", "DSP_B", "LEFT_J"};
const char *RX_SLOTS[3] = { "16", "24", "32" };
const char *TX_SLOTS[3] = { "16", "24", "32" };
const char *RX_BITWIDTH[3] = { "16", "24", "32" };
const char *RX_SLOTLEN[3] = { "16", "24", "32" };
const char *TX_SLOTLEN[3] = { "16", "24", "32" };
const char *CMD_ID[4] = { "CMD_SINGLE_WRITE", "CMD_BURST_WRITES", "CMD_UPDATE_BITS", "CMD_DELAY"};

struct hw_params_t {
	char *SampleRate[3];
	char *FMT_INV[4];
	char *FMT_MASK[4];
	char *RX_SLOTS[3];
	char *TX_SLOTS[3];
	char *RX_BITWIDTH[3];
	char *RX_SLOTLEN[3];
	char *TX_SLOTLEN[3];
};

struct regbin_parser {
	struct bin_header head;
	struct default_hw_params def_hw_params;
	char *init_params[MAX_CHANNELS];
	struct hw_params_t hw_params[MAX_CHANNELS];
};

static struct regbin_parser s_rbin;

static uint32_t g_no_of_profiles;
static int32_t g_tas25xx_profile;

struct tas25xx_profiles {
	uint8_t name[64];
	uint32_t misc_control;
	uint8_t *pre_power_up;
	uint8_t *post_power_up;
	uint8_t *pre_power_down;
	uint8_t *post_power_down;
};

struct tas25xx_profiles_channel {
	struct tas25xx_profiles *tas_profiles;
};

static struct tas25xx_profiles_channel g_profile_data_list[MAX_CHANNELS];

static char **g_profile_list;
static struct soc_enum tas25xx_switch_enum;
static struct snd_kcontrol_new tas25xx_profile_ctrl;


struct tas25xx_kcontrol_int {
	char *name;
	char channel;
	int32_t reg;
	char reg_type;
	int32_t mask;
	int32_t range_min;
	int32_t range_max;
	int32_t step;
	int32_t def;
	int32_t count;
	int32_t curr_val;
	uint32_t misc_info; /* additional info regarding kcontrol */
	char *chardata;
	int32_t *intdata;
	struct soc_mixer_control mctl;
};

struct tas25xx_kcontrol_enum_data {
	char name[64];
	char *data;
};

struct tas25xx_kcontrol_enum {
	char *name;
	char channel;
	char def;
	int32_t count;
	int32_t curr_val;
	uint32_t misc_info; /* additional info regarding kcontrol */
	struct tas25xx_kcontrol_enum_data *data;
	char **enum_texts;
	struct soc_enum tas25xx_kcontrol_enum;
};

union tas25xx_kcontrols_types {
	struct tas25xx_kcontrol_int int_type;
	struct tas25xx_kcontrol_enum enum_type;
};

struct tas25xx_kcontrols {
	char type;
	union tas25xx_kcontrols_types kcontrol;
};

static int32_t g_no_of_kcontrols;
/* bin file parsed data */
static struct tas25xx_kcontrols *g_kctrl_data;
/* creating kcontrols */
static struct snd_kcontrol_new *g_kctrl_ctrl;
static struct tas25xx_priv *g_tas25xx;

static uint8_t *tas25xx_read_size_bytes(uint8_t *in, uint8_t **out);
static uint32_t get_block_size_noadvance(uint8_t *mem_in);

#if IS_ENABLED(CONFIG_TAS25XX_ALGO)
void tas25xx_parse_algo_bin(int ch_count, u8 *buf);
#endif /* CONFIG_TAS25XX_ALGO */

static int32_t change_endian(void *data, int32_t size)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t *in;
	char *data_l;
	char c;

	if (size%4 != 0) {
		pr_err("tas25xx: %s size %d are not 4bytes aligned!!!",
			__func__, size);
	} else {
		in = (int32_t *)data;
		c = 0;
		for (i = 0; i < size/4; i++) {
			data_l = (char *)&in[i];
			for (j = 0; j < 2; j++) {
				c = data_l[3-j];
				data_l[3-j] = data_l[j];
				data_l[j] = c;
			}
		}
	}
	return 0;
}

static bool header_check(struct tas25xx_priv *p_tas25xx,
	const uint8_t *s1, const uint8_t *s2)
{
	bool success = false;
	//struct linux_platform *plat_data = NULL;

	//plat_data = p_tas25xx->platform_data;

	if (memcmp(s1, s2, HDR_STR_SZ) == 0)
		success = true;

	return success;
}

static int8_t *find_block_for_channel(struct tas25xx_priv *p_tas25xx,
	uint8_t *inp, uint8_t *blk_name, uint32_t ch)
{
	int32_t sz;
	uint8_t *outp = NULL;
	uint8_t *buf;
	uint32_t count = -1;
	uint32_t any_channel = 0;
	struct linux_platform *plat_data = NULL;

	plat_data = p_tas25xx->platform_data;

	if (inp && (inp + HDR_STR_SZ < p_tas25xx->fw_data + p_tas25xx->fw_size) &&
		header_check(p_tas25xx, inp, blk_name))
		return inp;

	/* start from begginging */
	buf = p_tas25xx->fw_data;

	if (ch == ANY_CHANNEL)
		any_channel = 1;

	while (buf < (p_tas25xx->fw_data + p_tas25xx->fw_size - 5)) {
		if (header_check(p_tas25xx, buf, INITP_STR)) {
			dev_info(plat_data->dev,
				"block %s found, incrementing count", INITP_STR);
			count++;
		} else {
			dev_info(plat_data->dev,
				"block check, found %.5s(@%p) count=%d", buf, buf, count);
		}
		if (header_check(p_tas25xx, buf, blk_name)) {
			if (any_channel || (count == ch)) {
				outp = buf;
				break;
			}
		}

		buf += 5; /* header */
		sz = get_block_size_noadvance(buf);
		buf += 4;
		buf += sz;
	}

	if (outp) {
		dev_warn(plat_data->dev,
			"found block %s @%p, ch=%d", blk_name, outp, count);
	} else {
		dev_err(plat_data->dev, "block %s not found", blk_name);
	}

	return outp;
}

/* For profile KControls */
static int32_t tas25xx_profile_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	if (!ucontrol) {
		pr_err("tas25xx: %s:ucontrol is NULL\n", __func__);
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = g_tas25xx_profile;

	return 0;
}

static int32_t tas25xx_profile_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	if (!ucontrol)
		return -EINVAL;

	g_tas25xx_profile = ucontrol->value.integer.value[0];
	pr_info("tas25xx: setting profile %d", g_tas25xx_profile);

	return 0;
}

static int32_t tas25xx_create_profile_controls(struct tas25xx_priv *p_tas25xx)
{
	struct tas25xx_profiles *tas_profiles_t = (struct tas25xx_profiles *)g_profile_data_list[0].tas_profiles;
	struct linux_platform *plat_data =
		(struct linux_platform *)p_tas25xx->platform_data;
	int32_t ret = 0;
	int32_t i = 0;

	g_profile_list = kzalloc(g_no_of_profiles * sizeof(char *), GFP_KERNEL);
	if (!g_profile_list) {
		ret = -ENOMEM;
		goto EXIT;
	}

	for (i = 0; i < g_no_of_profiles; i++) {
		g_profile_list[i] = kzalloc(64, GFP_KERNEL);
		if (!g_profile_list[i]) {
			ret = -ENOMEM;
			goto EXIT;
		}
		memcpy(g_profile_list[i], tas_profiles_t[i].name, 64);
	}

	tas25xx_switch_enum.items = g_no_of_profiles;
	tas25xx_switch_enum.texts = (const char * const *)g_profile_list;

	tas25xx_profile_ctrl.iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	tas25xx_profile_ctrl.name = "TAS25XX CODEC PROFILE";
	tas25xx_profile_ctrl.info = snd_soc_info_enum_double;
	tas25xx_profile_ctrl.get = tas25xx_profile_get;
	tas25xx_profile_ctrl.put = tas25xx_profile_put;
	tas25xx_profile_ctrl.private_value = (unsigned long)(&tas25xx_switch_enum);

	ret = snd_soc_add_component_controls(plat_data->codec,
			&tas25xx_profile_ctrl, 1);
EXIT:
	return ret;
}

static int32_t tas25xx_em_left_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas25xx_priv *p_tas25xx = snd_soc_component_get_drvdata(component);
	struct soc_enum *soc_enum = (struct soc_enum *)kcontrol->private_value;
	uint32_t reg;
	int value = 0;
	int ret = 0;

	if (!ucontrol) {
		pr_err("tas25xx: %s:ucontrol is NULL\n", __func__);
		return -EINVAL;
	}

	reg = TAS25XX_REG(0, 0, soc_enum->reg);
	ret = p_tas25xx->read(p_tas25xx, 0, reg, &value);
	ucontrol->value.integer.value[0] = (value & TAS25XX_EM_MASK) >> soc_enum->shift_l;
	pr_info ("%s: read value = %x  ret = %d\n", __func__, value, ret);
	return ret;
}

static int32_t tas25xx_em_left_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas25xx_priv *p_tas25xx = snd_soc_component_get_drvdata(component);
	struct soc_enum *soc_enum = (struct soc_enum *)kcontrol->private_value;
	uint32_t reg;
	int value = 0;
	int ret = 0;

	if (!ucontrol)
		return -EINVAL;

	p_tas25xx->left_em_mode = ucontrol->value.integer.value[0];
	value = p_tas25xx->left_em_mode << soc_enum->shift_l;
	reg = TAS25XX_REG(0, 0, soc_enum->reg);
	ret = p_tas25xx->update_bits(p_tas25xx, 0, reg, TAS25XX_EM_MASK, value);
	pr_info("%s: update value %d ret = %d", __func__, value, ret);
	return ret;
}

static int32_t tas25xx_em_right_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas25xx_priv *p_tas25xx = snd_soc_component_get_drvdata(component);
	struct soc_enum *soc_enum = (struct soc_enum *)kcontrol->private_value;
	uint32_t reg;
	int value = 0;
	int ret = 0;

	if (!ucontrol) {
		pr_err("tas25xx: %s:ucontrol is NULL\n", __func__);
		return -EINVAL;
	}

	reg = TAS25XX_REG(0, 0, soc_enum->reg);
	ret = p_tas25xx->read(p_tas25xx, 1, reg, &value);
	ucontrol->value.integer.value[0] = (value & TAS25XX_EM_MASK) >> soc_enum->shift_l;
	pr_info ("%s: read value = %x  ret = %d\n", __func__, value, ret);
	return ret;
}

static int32_t tas25xx_em_right_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
	struct tas25xx_priv *p_tas25xx = snd_soc_component_get_drvdata(component);
	struct soc_enum *soc_enum = (struct soc_enum *)kcontrol->private_value;
	uint32_t reg;
	int value = 0;
	int ret = 0;

	if (!ucontrol)
		return -EINVAL;

	p_tas25xx->right_em_mode = ucontrol->value.integer.value[0];
	value = p_tas25xx->right_em_mode << soc_enum->shift_l;
	reg = TAS25XX_REG(0, 0, soc_enum->reg);
	ret = p_tas25xx->update_bits(p_tas25xx, 1, reg, TAS25XX_EM_MASK, value);
	pr_info("%s: update value %x ret = %d", __func__, value, ret);
	return ret;
}

static const char * const tas25xx_em_text[] = {"OFF", "NG_ON", "ME_ON", "NG_ME_ON"};
static SOC_ENUM_SINGLE_DECL(em_enum, TAS25XX_EM_REG, TAS25XX_EM_SHIFT, tas25xx_em_text);

static struct snd_kcontrol_new tas25xx_custom_ctrls [] = {
	SOC_ENUM_EXT("TAS25XX EM LEFT", em_enum, tas25xx_em_left_get, tas25xx_em_left_put),
	SOC_ENUM_EXT("TAS25XX EM RIGHT", em_enum, tas25xx_em_right_get, tas25xx_em_right_put),
};

static int32_t tas25xx_create_custom_controls(struct tas25xx_priv *p_tas25xx)
{
	struct linux_platform *plat_data =
		(struct linux_platform *)p_tas25xx->platform_data;
	int32_t ret = 0;

	ret = snd_soc_add_component_controls(plat_data->codec,
			&tas25xx_custom_ctrls[0], 2);
	return ret;
}

static uint8_t *process_block_get_cmd(uint8_t *mem_in, int8_t *cmd)
{
	*cmd = *mem_in;
	mem_in++;
	return mem_in;
}

static uint32_t get_block_size_noadvance(uint8_t *mem_in)
{
	int32_t sz;
	int32_t *ptr = (int32_t *)mem_in;

	sz = *ptr;
	ptr++;
	return sz;
}

static uint8_t *process_block_get_single_write_data(uint8_t *mem_in, int32_t *reg, uint8_t *val)
{
	uint8_t *data8b;
	int32_t *data32b;

	data32b = (int32_t *)mem_in;
	*reg = *data32b;
	data32b++;

	data8b = (uint8_t *)data32b;
	*val = *data8b;
	data8b++;

	return data8b;
}

static uint8_t *process_block_get_burst_write_data(uint8_t *mem_in,
	int32_t *reg, int32_t *count, uint8_t **bulk_buffer)
{
	uint8_t *data8b;
	int32_t *data32b;

	data32b = (int32_t *)mem_in;
	*reg = *data32b;
	data32b++;

	*count = *data32b;
	data32b++;

	data8b = (uint8_t *)data32b;
	*bulk_buffer = data8b;

	data8b = data8b + (*count);

	return data8b;
}

static uint8_t *process_block_get_bit_update_data(uint8_t *mem_in,
	int32_t *reg, uint8_t *mask, uint8_t *value)
{
	uint8_t *data8b;
	int32_t *data32b;

	data32b = (int32_t *)mem_in;
	*reg = *data32b;
	data32b++;

	data8b = (uint8_t *)data32b;
	*mask = *data8b;
	data8b++;
	*value = *data8b;
	data8b++;

	return data8b;
}

static uint8_t *process_block_get_delay_data(uint8_t *mem_in, int32_t *delay)
{
	uint8_t *data8b;
	int32_t *data32b;

	data32b = (int32_t *)mem_in;
	*delay = *data32b;
	data32b++;

	data8b = (uint8_t *)data32b;

	return data8b;
}

int32_t tas25xx_process_block(struct tas25xx_priv *p_tas25xx, char *mem, int32_t chn)
{
	int32_t i = 0;
	int32_t block_size = 0;
	struct linux_platform *plat_data = NULL;
	int32_t ret = 0;
	int32_t ret_i = 0;
	int32_t reg;
	int32_t count;
	int32_t delay;
	int fw_state;
	int8_t cmd;
	uint8_t mask;
	uint8_t val;
	uint8_t *buffer = NULL;
	uint8_t *ptr = NULL;

	fw_state = atomic_read(&p_tas25xx->fw_state);
	if (fw_state != TAS25XX_DSP_FW_OK)
		return -EINVAL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;
	memcpy(&block_size, mem, sizeof(int32_t));
	mem += sizeof(int32_t);
	ptr = mem;

	while (i < block_size) {
		ptr = process_block_get_cmd(ptr, &cmd);
		switch (cmd) {
		case CMD_SINGLE_WRITE:
			ptr = process_block_get_single_write_data(ptr, &reg, &val);
			i += CMD_SINGLE_WRITE_SZ;
			ret_i = p_tas25xx->write(p_tas25xx, chn, reg, val);
			dev_dbg(plat_data->dev, "ch=%d Cmd = %s B:P:R %02x:%02x:%02x, value=%02x, ret=%d\n",
				chn, CMD_ID[cmd], TAS25XX_BOOK_ID(reg), TAS25XX_PAGE_ID(reg),
				TAS25XX_PAGE_REG(reg), val, ret_i);
			ret |= ret_i;
			break;

		case CMD_BURST_WRITES:
			ptr = process_block_get_burst_write_data(ptr, &reg, &count, &buffer);
			i += CMD_BURST_WRITES_SZ + count;
			ret_i = p_tas25xx->bulk_write(p_tas25xx, chn, reg, buffer, count);
			dev_dbg(plat_data->dev,
				"ch=%d Cmd = %s B:P:R %02x:%02x:%02x, count=%d, buf=%02x %02x %02x %02x ret=%d\n",
				chn, CMD_ID[cmd], TAS25XX_BOOK_ID(reg), TAS25XX_PAGE_ID(reg), TAS25XX_PAGE_REG(reg),
				count, buffer[0], buffer[1], buffer[2], buffer[3], ret_i);
			ret |= ret_i;
			break;

		case CMD_UPDATE_BITS:
			ptr = process_block_get_bit_update_data(ptr, &reg, &mask, &val);
			i += CMD_UPDATE_BITS_SZ;
			ret_i = p_tas25xx->update_bits(p_tas25xx, chn, reg, mask, val);
			dev_dbg(plat_data->dev,
				"ch=%d Cmd = %s B:P:R %02x:%02x:%02x mask=%02x, val=%02x, ret=%d",
				chn, CMD_ID[cmd], TAS25XX_BOOK_ID(reg), TAS25XX_PAGE_ID(reg), TAS25XX_PAGE_REG(reg),
				mask, val, ret_i);
			ret |= ret_i;
			break;

		case CMD_DELAY:
			ptr = process_block_get_delay_data(ptr, &delay);
			i += CMD_DELAY_SZ;
			dev_dbg(plat_data->dev, "ch=%d Cmd = %s delay=%x(%d)\n",
				chn, CMD_ID[cmd], delay, delay);
			ret |= 0;
			usleep_range(delay*1000, delay*1000);
			break;
		}
	}

	return ret;
}

int32_t tas25xx_check_if_powered_on(struct tas25xx_priv *p_tas25xx, int *state, int chn)
{
	int8_t cmd;
	uint8_t expected_val = 0;
	uint8_t mask;
	uint32_t val;
	int32_t reg;
	int32_t i = 0;
	int32_t block_size = 0;
	struct linux_platform *plat_data = NULL;
	int32_t ret = 0;
	uint8_t *ptr = p_tas25xx->block_op_data[chn].power_check;
	bool expected = true;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;
	if (!ptr) {
		dev_err(plat_data->dev, "%s null buffer recieved for ch=%d",
			__func__, chn);
		return -EINVAL;
	}

	memcpy(&block_size, ptr, sizeof(int32_t));
	ptr += sizeof(int32_t);

	while (i < block_size) {
		ptr = process_block_get_cmd(ptr, &cmd);
		switch (cmd) {
		case CMD_SINGLE_WRITE:
			ptr = process_block_get_single_write_data(ptr, &reg, &expected_val);
			i += CMD_SINGLE_WRITE_SZ;
			ret = p_tas25xx->read(p_tas25xx, chn, reg, &val);
			dev_dbg(plat_data->dev,
				"Chn=%d Cmd = %s reg=%x(%d), value read=%02x, expected=%02x\n",
				chn, CMD_ID[cmd], reg, reg, val, expected_val);
			if ((val & 0xFFFF) != expected_val)
				expected = expected && false;
			break;

		case CMD_UPDATE_BITS:
			ptr = process_block_get_bit_update_data(ptr, &reg, &mask, &expected_val);
			i += CMD_UPDATE_BITS_SZ;
			dev_dbg(plat_data->dev, "Cmd = %s reg=%x(%d), mask=%02x, expected_val=%02x\n",
				CMD_ID[cmd], reg, reg, mask, expected_val);
			ret = p_tas25xx->read(p_tas25xx, chn, reg, &val);
			dev_dbg(plat_data->dev,
				"Chn=%d Cmd = %s reg=%x(%d), value read=%02x, expected=%02x\n",
				chn, CMD_ID[cmd], reg, reg, val, expected_val);
			if ((val & 0xFFFF) != expected_val)
				expected = expected && false;
			break;

		default:
			dev_dbg(plat_data->dev, "Chn=%d default cmd=%d", chn, cmd);
			break;
		}
	}

	if (expected)
		*state = 1;
	else
		*state = 0;

	return ret;
}

static int32_t tas25xx_parse_init_params(struct tas25xx_priv *p_tas25xx, uint8_t **buf, int32_t ch)
{
	int32_t ret = 0;
	int32_t size = 0;
	//struct linux_platform *plat_data = NULL;
	uint8_t *data = *buf;

	//plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	data = find_block_for_channel(p_tas25xx, data, INITP_STR, ch);
	if (!data)
		return -EINVAL;

	data += HDR_STR_SZ;

	/* Parsing Init Params */
	memcpy(&size, data, sizeof(size));
	s_rbin.init_params[ch] = data;
	data += (size + sizeof(size));

	*buf = data;

	return ret;
}

static uint8_t *tas25xx_read_size_bytes(uint8_t *in, uint8_t **out)
{
	uint8_t *buf;
	int32_t size = 0;

	if (!in || !out)
		return NULL;

	buf = in;
	*out = buf;

	memcpy(&size, buf, sizeof(size));
	buf += size + sizeof(int32_t);

	/*buf_data = in + sizeof(int32_t);
	dev_info(plat_data->dev, "tas25xx: dump buffer, size=%d\n", size);
	for (i = 0; i < size; i++) {
		dev_info(plat_data->dev, "tas25xx: buf[%d] = 0x%02x,\t", i, buf_data[i]);
	}
	dev_info(plat_data->dev, "\n");
	*/

	return buf;
}

static int32_t tas25xx_parse_hw_params(struct tas25xx_priv *p_tas25xx, uint8_t **inbuf, int32_t ch)
{
	int32_t ret = 0;
	uint8_t *next = *inbuf;
	uint8_t *first = *inbuf;
	uint8_t *out = NULL;
	int32_t size = 0;
	int32_t no_of_hw_params = 0;
	int count = 1;
	int end = 0;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;

	next = find_block_for_channel(p_tas25xx, next, HW_PRMS_STR, ch);
	if (!next)
		return -EINVAL;

	next += HDR_STR_SZ;
	size = get_block_size_noadvance(next);
	next += 4;
	no_of_hw_params = get_block_size_noadvance(next);
	next += 4;

	dev_dbg(plat_data->dev, "parsing HW params, inbuf=0x%p, size=%d, no_of_hw_params=%d",
		next, size, no_of_hw_params);

	next = tas25xx_read_size_bytes(next, &out);
	while (next) {
		switch (count) {
		case 1:
			s_rbin.hw_params[ch].SampleRate[SampleRate_48000] = out;
			break;

		case 2:
			s_rbin.hw_params[ch].SampleRate[SampleRate_44100] = out;
			break;

		case 3:
			s_rbin.hw_params[ch].SampleRate[SampleRate_96000] = out;
			break;

		case 4:
			/* Parsing FMT_INV_NB_NF */
			s_rbin.hw_params[ch].FMT_INV[FMT_INV_NB_NF] = out;
			break;

		case 5:
			/* Parsing FMT_INV_IB_NF */
			s_rbin.hw_params[ch].FMT_INV[FMT_INV_IB_NF] = out;
			break;

		case 6:
			/* Parsing FMT_INV_NB_IF */
			s_rbin.hw_params[ch].FMT_INV[FMT_INV_NB_IF] = out;
			break;

		case 7:
			/* Parsing FMT_INV_IB_IF */
			s_rbin.hw_params[ch].FMT_INV[FMT_INV_IB_IF] = out;
			break;

		case 8:
			/* Parsing FMT_MASK_I2S */
			s_rbin.hw_params[ch].FMT_MASK[FMT_MASK_I2S] = out;
			break;

		case 9:
			/* Parsing FMT_MASK_DSP_A */
			s_rbin.hw_params[ch].FMT_MASK[FMT_MASK_DSP_A] = out;
			break;

		case 10:
			/* Parsing FMT_MASK_DSP_B */
			s_rbin.hw_params[ch].FMT_MASK[FMT_MASK_DSP_B] = out;
			break;

		case 11:
			/* Parsing FMT_MASK_LEFT_J */
			s_rbin.hw_params[ch].FMT_MASK[FMT_MASK_LEFT_J] = out;
			break;


		case 12:
			/* Parsing RX_SLOTS_16 */
			s_rbin.hw_params[ch].RX_SLOTS[RX_SLOTS_16] = out;
			break;


		case 13:
			/* Parsing RX_SLOTS_24 */
			s_rbin.hw_params[ch].RX_SLOTS[RX_SLOTS_24] = out;
			break;


		case 14:
			/* Parsing RX_SLOTS_32 */
			s_rbin.hw_params[ch].RX_SLOTS[RX_SLOTS_32] = out;
			break;


		case 15:
			/* Parsing TX_SLOTS_16 */
			s_rbin.hw_params[ch].TX_SLOTS[TX_SLOTS_16] = out;
			break;


		case 16:
			/* Parsing TX_SLOTS_24 */
			s_rbin.hw_params[ch].TX_SLOTS[TX_SLOTS_24] = out;
			break;


		case 17:
			/* Parsing TX_SLOTS_32 */
			s_rbin.hw_params[ch].TX_SLOTS[TX_SLOTS_32] = out;
			break;


		case 18:
			/* Parsing RX_BITWIDTH_16 */
			s_rbin.hw_params[ch].RX_BITWIDTH[RX_BITWIDTH_16] = out;
			break;


		case 19:
			/* Parsing RX_BITWIDTH_24 */
			s_rbin.hw_params[ch].RX_BITWIDTH[RX_BITWIDTH_24] = out;
			break;


		case 20:
			/* Parsing RX_BITWIDTH_32 */
			s_rbin.hw_params[ch].RX_BITWIDTH[RX_BITWIDTH_32] = out;
			break;


		case 21:
			/* Parsing RX_SLOTLEN_16 */
			s_rbin.hw_params[ch].RX_SLOTLEN[RX_SLOTLEN_16] = out;
			break;


		case 22:
			/* Parsing RX_SLOTLEN_24 */
			s_rbin.hw_params[ch].RX_SLOTLEN[RX_SLOTLEN_24] = out;
			break;

		case 23:
			/* Parsing RX_SLOTLEN_32 */
			s_rbin.hw_params[ch].RX_SLOTLEN[RX_SLOTLEN_32] = out;
			break;

		case 24:
			/* Parsing TX_SLOTLEN_16 */
			s_rbin.hw_params[ch].TX_SLOTLEN[TX_SLOTLEN_16] = out;
			break;

		case 25:
			/* Parsing TX_SLOTLEN_24 */
			s_rbin.hw_params[ch].TX_SLOTLEN[TX_SLOTLEN_24] = out;
			break;

		case 26:
			/* Parsing TX_SLOTLEN_32 */
			s_rbin.hw_params[ch].TX_SLOTLEN[TX_SLOTLEN_32] = out;
			end = 1;
			break;

		default:
			end = 1;
			break;
		} /*<<switch>>*/

		if (!end) {
			next = tas25xx_read_size_bytes(next, &out);
			count++;
		} else {
			break;
		}
	} /*<<while>>*/

	size = (int32_t)(next - first);
	if (size < 0)
		size = -1 * size;
	*inbuf = next;

	dev_dbg(plat_data->dev, "parsing HW params exit, next=0x%p, size=%d", next, size);

	return ret;
}

int tas25xx_check_if_algo_ctrl_bypassed(int ch)
{
	struct tas25xx_profiles *tas_profiles_t;
	if (ch >= s_rbin.head.channels)
		return -EINVAL;

	tas_profiles_t = (struct tas25xx_profiles *)g_profile_data_list[ch].tas_profiles;

	/* bit5, 1 = algo control bypass, 0 = use algo */
	return (tas_profiles_t[g_tas25xx_profile].misc_control & 0x10);
}
EXPORT_SYMBOL(tas25xx_check_if_algo_ctrl_bypassed);

int tas25xx_get_drv_channel_opmode(void)
{
	struct tas25xx_profiles *tas_profiles_t;

	tas_profiles_t = (struct tas25xx_profiles *)g_profile_data_list[0].tas_profiles;
	return tas_profiles_t[g_tas25xx_profile].misc_control;
}
EXPORT_SYMBOL(tas25xx_get_drv_channel_opmode);


static int32_t tas25xx_parse_profiles(struct tas25xx_priv *p_tas25xx, uint8_t **inbuf, int32_t ch)
{
	int32_t ret = 0;
	int32_t size;
	int32_t i = 0;
	uint8_t *buf = *inbuf;
	uint8_t *out = NULL;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	buf = find_block_for_channel(p_tas25xx, buf, PROFP_STR, ch);
	if (!buf)
		return -EINVAL;

	buf += HDR_STR_SZ;
	size = get_block_size_noadvance(buf);
	buf += 4;
	dev_info(plat_data->dev, "parsing profiles, total size=%d", size);

	memcpy(&g_no_of_profiles, buf, 1);
	buf++;

	dev_info(plat_data->dev, "Number of profiles=%d, allocation=%u", (int)g_no_of_profiles,
		(uint32_t)(g_no_of_profiles * sizeof(struct tas25xx_profiles)));

	g_profile_data_list[ch].tas_profiles =
		kzalloc(g_no_of_profiles*(sizeof(struct tas25xx_profiles)), GFP_KERNEL);
	if (!g_profile_data_list[ch].tas_profiles)
		return -ENOMEM;

	for (i = 0; i < g_no_of_profiles; i++) {
		struct tas25xx_profiles *tas_profiles_t =
			(struct tas25xx_profiles *)g_profile_data_list[ch].tas_profiles;
		/* Parsing Name */
		memcpy(tas_profiles_t[i].name, buf, 64);
		buf += 64;

		/* bitmask indicating misc control for profile */
		memcpy(&tas_profiles_t[i].misc_control, buf, sizeof(uint32_t));
		buf += 4;

		/* pre_power_up */
		buf = tas25xx_read_size_bytes(buf, &out);
		tas_profiles_t[i].pre_power_up = out;

		/* post_power_up */
		buf = tas25xx_read_size_bytes(buf, &out);
		tas_profiles_t[i].post_power_up = out;

		/* pre_power_down */
		buf = tas25xx_read_size_bytes(buf, &out);
		tas_profiles_t[i].pre_power_down = out;

		/* post_power_down */
		buf = tas25xx_read_size_bytes(buf, &out);
		tas_profiles_t[i].post_power_down = out;
	}

	*inbuf = buf;
	return ret;
}

void tas25xx_prep_dev_for_calib(int start)
{
	int i, ret;
	uint8_t *mem;
	struct linux_platform *plat_data = NULL;

	if (unlikely(!g_tas25xx))
		return;

	plat_data = (struct linux_platform *) g_tas25xx->platform_data;

	for (i = 0; i < g_tas25xx->ch_count; i++) {
		if (start)
			mem = g_tas25xx->block_op_data[i].cal_init;
		else
			mem = g_tas25xx->block_op_data[i].cal_deinit;

		if (mem) {
			ret = tas25xx_process_block(g_tas25xx, mem, i);
			if (ret)
				dev_err(plat_data->dev, "ch=%d failed to %s device for calibration",
					i, start ? "init" : "deinit");
		}
	}
}
EXPORT_SYMBOL(tas25xx_prep_dev_for_calib);


static int32_t tas25xx_parse_block_data(struct tas25xx_priv *p_tas25xx, uint8_t **inbuf, int32_t ch)
{
	int32_t i = 0;
	int32_t size;
	int32_t ret = 0;
	uint32_t number_of_blocks = 0;
	enum block_types_t type;
	uint8_t *out = NULL;
	//uint8_t *start;
	uint8_t *end;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;

	//start = *inbuf;
	end = *inbuf;

	end = find_block_for_channel(p_tas25xx, end, BLK_OP_STR, ch);
	if (!end)
		return -EINVAL;

	end += HDR_STR_SZ;
	size = get_block_size_noadvance(end);
	end += 4;

	number_of_blocks = *((uint32_t *)end);
	end += sizeof(uint32_t);

	dev_info(plat_data->dev, "parsing block op data, total size=%d, toal blocks=%u",
		size, number_of_blocks);

	for (i = 0; i < number_of_blocks; i++) {
		if (memcmp(end, "SWRST", MAIN_BLOCK_SIZE) == 0) {
			type = BLK_SW_RST;
		} else if (memcmp(end, "PWRCK", MAIN_BLOCK_SIZE) == 0) {
			type = BLK_POWER_CHECK;
		} else if (memcmp(end, "MUTE0", MAIN_BLOCK_SIZE) == 0) {
			type = BLK_MUTE;
		} else if (memcmp(end, "RXFMT", MAIN_BLOCK_SIZE) == 0) {
			type = BLK_RX_FMT;
		} else if (memcmp(end, "TXFMT", MAIN_BLOCK_SIZE) == 0) {
			type = BLK_TX_FMT;
		} else if (memcmp(end, "CLINI", MAIN_BLOCK_SIZE) == 0) {
			type = BLK_CAL_INIT;
		} else if (memcmp(end, "CLDIN", MAIN_BLOCK_SIZE) == 0) {
			type = BLK_CAL_DEINIT;
		} else {
			type = BLK_INVALID;
			dev_err(plat_data->dev, "%s, number of block=%u\n",
				__func__, number_of_blocks);
		}
		end += 5;

		switch (type) {
		case BLK_SW_RST:
			end = tas25xx_read_size_bytes(end, &out);
			if (out)
				p_tas25xx->block_op_data[ch].sw_reset = out;
			else
				ret = -EINVAL;
			break;

		case BLK_POWER_CHECK:
			end = tas25xx_read_size_bytes(end, &out);
			if (out)
				p_tas25xx->block_op_data[ch].power_check = out;
			else
				ret = -EINVAL;
			break;

		case BLK_MUTE:
			end = tas25xx_read_size_bytes(end, &out);
			if (out)
				p_tas25xx->block_op_data[ch].mute = out;
			else
				ret = -EINVAL;
			break;

		case BLK_CAL_INIT:
			end = tas25xx_read_size_bytes(end, &out);
			if (out)
				p_tas25xx->block_op_data[ch].cal_init = out;
			else
				ret = -EINVAL;
			break;

		case BLK_CAL_DEINIT:
			end = tas25xx_read_size_bytes(end, &out);
			if (out)
				p_tas25xx->block_op_data[ch].cal_deinit = out;
			else
				ret = -EINVAL;
			break;

		case BLK_RX_FMT:
			end = tas25xx_read_size_bytes(end, &out);
			dev_info(plat_data->dev, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
				out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7], out[8], out[9], out[10], out[11]);
			if (out)
				p_tas25xx->block_op_data[ch].rx_fmt_data = out;
			else
				ret = -EINVAL;
			break;

		case BLK_TX_FMT:
			end = tas25xx_read_size_bytes(end, &out);
			dev_info(plat_data->dev, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
				out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7], out[8], out[9], out[10], out[11]);
			if (out)
				p_tas25xx->block_op_data[ch].tx_fmt_data = out;
			else
				ret = -EINVAL;
			break;

		case BLK_INVALID:
		default:
			break;

		}
	}

	dev_info(plat_data->dev, "%s reset=%p, power-check=%p mute=%p\n", __func__,
		p_tas25xx->block_op_data[ch].sw_reset, p_tas25xx->block_op_data[ch].power_check,
		p_tas25xx->block_op_data[ch].mute);

	*inbuf = end;

	return ret;
}

static int32_t tas25xx_parse_interrupts(struct tas25xx_priv *p_tas25xx, uint8_t **inbuf, int32_t ch)
{
	int32_t ret = 0;
	int32_t size = 0;
	int32_t i = 0;
	uint8_t *out = NULL;
	uint8_t *start;
	uint8_t *end;
	int32_t dummy;
	struct tas25xx_intr_info *intr_info = NULL;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;

	start = *inbuf;
	end = *inbuf;

	end = find_block_for_channel(p_tas25xx, end, INTRP_STR, ch);
	if (!end)
		return -EINVAL;

	end += HDR_STR_SZ;
	size = get_block_size_noadvance(end);
	end += 4;
	dev_info(plat_data->dev, "parsing interrupt data, total size=%d, ptr=%p",
		size, end - 4 - 5);


	p_tas25xx->intr_data[ch].count = *end;
	end++;

	dev_info(plat_data->dev, "INTR parsing interrupt buffer, interrupt count=%d",
		p_tas25xx->intr_data[ch].count);

	end = tas25xx_read_size_bytes(end, &out);
	if (out)
		p_tas25xx->intr_data[ch].buf_intr_enable = out;
	else
		return -EINVAL;

	end = tas25xx_read_size_bytes(end, &out);
	if (out)
		p_tas25xx->intr_data[ch].buf_intr_disable = out;
	else
		return -EINVAL;

	end = tas25xx_read_size_bytes(end, &out);
	if (out)
		p_tas25xx->intr_data[ch].buf_intr_clear = out;
	else
		return -EINVAL;

	dev_info(plat_data->dev, "INTR en=%p dis=%p clr=%p\n",
		p_tas25xx->intr_data[ch].buf_intr_enable,
		p_tas25xx->intr_data[ch].buf_intr_disable,
		p_tas25xx->intr_data[ch].buf_intr_clear);

	dev_info(plat_data->dev, "INTR %c %c %c %c", end[0], end[1], end[2], end[3]);

	size = (p_tas25xx->intr_data[ch].count) * sizeof(struct tas25xx_intr_info);
	dev_info(plat_data->dev, "%s:%u: ALLOC Size %d\n",
		__func__, __LINE__, size);

	intr_info = kzalloc(size, GFP_KERNEL);
	if (!intr_info)
		return -ENOMEM;

	for (i = 0; i < p_tas25xx->intr_data[ch].count; i++) {
		memcpy(intr_info[i].name, end, 64);
		intr_info[i].name[63] = 0;
		end += 64;
		memcpy(&(intr_info[i].reg), end, 4);
		end += 4;
		memcpy(&(intr_info[i].mask), end, 4);
		end += 4;
		memcpy(&(intr_info[i].action), end, 4);
		end += 4;
		memcpy(&(intr_info[i].is_clock_based), end, 4);
		end += 4;
		/* 3 dummy ints for future reference */
		memcpy(&dummy, end, 4);
		end += 4;
		memcpy(&dummy, end, 4);
		end += 4;
		memcpy(&dummy, end, 4);
		end += 4;

		dev_info(plat_data->dev,
			"INTR %s, REG=%x, MASK=%x, action=%d, clk based=%d", intr_info[i].name,
			intr_info[i].reg, intr_info[i].mask, intr_info[i].action,
			intr_info[i].is_clock_based);
	}
	p_tas25xx->intr_data[ch].intr_info = intr_info;

	p_tas25xx->intr_data[ch].processing_delay = *((uint32_t *)end);
	end += 4;

	/* 5 dummy ints for future reference */
	dummy = *((uint32_t *)end);
	end += 4;
	dummy = *((uint32_t *)end);
	end += 4;
	dummy = *((uint32_t *)end);
	end += 4;
	dummy = *((uint32_t *)end);
	end += 4;
	dummy = *((uint32_t *)end);
	end += 4;

	if (!ch)
		dev_info(plat_data->dev, "INTR processing_delay=%d",
			p_tas25xx->intr_data[ch].processing_delay);

	size = end - start;
	if (size < 0)
		size = -1 * size;

	dev_info(plat_data->dev, "INTR bin buf size=%d (end=%p - start=%p)",
		size, end, start);
	*inbuf = end;

	return ret;
}

static int32_t tas25xx_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int32_t ret = -EINVAL;
	int32_t curr_kcontrol = 0;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;

	if (!mc) {
		pr_err("%s:codec or control is NULL\n", __func__);
		return ret;
	}

	curr_kcontrol = mc->reg;
	if (curr_kcontrol < g_no_of_kcontrols)
		ucontrol->value.integer.value[0] =
			g_kctrl_data[curr_kcontrol].kcontrol.int_type.curr_val;
	else
		return ret;
	return 0;
}

static int32_t tas25xx_int_put_idx_value(struct tas25xx_priv *p_tas25xx,
	int32_t ctrl_idx, int32_t value_idx)
{
	int ret = 0;
	int32_t reg_w;
	int32_t count_w;
	int32_t chn;
	int32_t mask_w;
	int32_t value_w;
	struct linux_platform *plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	if (ctrl_idx < g_no_of_kcontrols) {
		reg_w = g_kctrl_data[ctrl_idx].kcontrol.int_type.reg;
		count_w = g_kctrl_data[ctrl_idx].kcontrol.int_type.count;
		chn = g_kctrl_data[ctrl_idx].kcontrol.int_type.channel;
		mask_w = g_kctrl_data[ctrl_idx].kcontrol.int_type.mask;
		value_w = 0;

		dev_info(plat_data->dev, "%s kcontrol=%s with value index=%d", __func__,
			g_kctrl_data[ctrl_idx].kcontrol.int_type.name, value_idx);

		if (value_idx >= 0 && value_idx < count_w) {
			switch (g_kctrl_data[ctrl_idx].kcontrol.int_type.reg_type) {
			case CMD_SINGLE_WRITE:
				value_w = g_kctrl_data[ctrl_idx].kcontrol.int_type.chardata[value_idx];
				ret = p_tas25xx->write(p_tas25xx, chn, reg_w,
						value_w);
				if (ret) {
					dev_err(plat_data->dev, "tas25xx:%s failed ret = %d\n", __func__, ret);
				} else {
					dev_info(plat_data->dev,
						"ch=%d Cmd = %s B:P:R %02x:%02x:%02x, value=%02x, ret=%d\n", chn,
						CMD_ID[CMD_SINGLE_WRITE], TAS25XX_BOOK_ID(reg_w), TAS25XX_PAGE_ID(reg_w),
						TAS25XX_PAGE_REG(reg_w), value_w, ret);
				}
				break;

			case CMD_BURST_WRITES:
				value_w = g_kctrl_data[ctrl_idx].kcontrol.int_type.intdata[value_idx];
				change_endian(&value_w, sizeof(int32_t));
				ret = p_tas25xx->bulk_write(p_tas25xx, chn,
						reg_w,
						(char *)(&value_w), sizeof(int32_t));
				if (ret) {
					dev_err(plat_data->dev, "tas25xx:%s failed ret = %d\n", __func__, ret);
				} else {
					dev_info(plat_data->dev, "ch=%d Cmd = %s B:P:R %02x:%02x:%02x, value=%02x, ret=%d\n",
						chn, CMD_ID[CMD_BURST_WRITES], TAS25XX_BOOK_ID(reg_w), TAS25XX_PAGE_ID(reg_w),
						TAS25XX_PAGE_REG(reg_w), value_w, ret);
				}
				break;

			case CMD_UPDATE_BITS:
				value_w = g_kctrl_data[ctrl_idx].kcontrol.int_type.chardata[value_idx];
				ret = p_tas25xx->update_bits(p_tas25xx,
						chn, reg_w, mask_w, value_w);
				if (ret) {
					dev_err(plat_data->dev, "tas25xx:%s failed ret = %d\n", __func__, ret);
				} else {
					dev_info(plat_data->dev,
						"ch=%d Cmd = %s B:P:R %02x:%02x:%02x, mask=%02x, value=%02x, ret=%d\n",
						chn, CMD_ID[CMD_UPDATE_BITS], TAS25XX_BOOK_ID(reg_w), TAS25XX_PAGE_ID(reg_w),
						TAS25XX_PAGE_REG(reg_w), mask_w, value_w, ret);
				}
				break;
			}
		} else {
			dev_err(plat_data->dev, "tas25xx:%s out of bound, value_idx=%d\n", __func__, value_idx);
			ret = -EINVAL;
		}
	} else {
		dev_err(plat_data->dev, "tas25xx:%s out of bound ctrl_idx=%d\n", __func__, ctrl_idx);
		ret = -EINVAL;
	}

	return ret;
}

static int32_t tas25xx_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct tas25xx_priv *p_tas25xx = NULL;
	struct linux_platform *plat_data = NULL;
	int32_t ret = -EINVAL;
	int32_t curr_kcontrol = 0;
	uint32_t misc_info;

	if ((codec == NULL) || (mc == NULL)) {
		pr_err("tas25xx: %s:codec or control is NULL\n", __func__);
		return ret;
	}

	p_tas25xx = snd_soc_component_get_drvdata(codec);
	if (p_tas25xx == NULL) {
		pr_err("tas25xx: %s:p_tas25xx is NULL\n", __func__);
		return ret;
	}

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	mutex_lock(&p_tas25xx->codec_lock);

	curr_kcontrol = mc->reg;
	if (curr_kcontrol < g_no_of_kcontrols) {
		int32_t value_idx = ucontrol->value.integer.value[0];
		misc_info = g_kctrl_data[curr_kcontrol].kcontrol.int_type.misc_info;
		if (((misc_info & 0xf) == KCNTR_ANYTIME) ||
						(p_tas25xx->m_power_state == TAS_POWER_ACTIVE))
			ret = tas25xx_int_put_idx_value(p_tas25xx, curr_kcontrol, value_idx);
		else
			ret = 0;

		if (!ret)
			g_kctrl_data[curr_kcontrol].kcontrol.int_type.curr_val = value_idx;
	} else {
		dev_err(plat_data->dev, "tas25xx:%s invalid kcontrol %d\n", __func__, curr_kcontrol);
	}
	mutex_unlock(&p_tas25xx->codec_lock);

	return ret;
}

static int32_t tas25xx_enum_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int32_t ret = -EINVAL;
	int32_t i = 0;

	if (ucontrol == NULL) {
		pr_err("%s:ucontrol is NULL\n", __func__);
		return ret;
	}

	while (i < g_no_of_kcontrols) {
		if (g_kctrl_data[i].type == 1) {
			if (strnstr(ucontrol->id.name,
						g_kctrl_data[i].kcontrol.enum_type.name, 64)) {
				ucontrol->value.integer.value[0] =
					g_kctrl_data[i].kcontrol.enum_type.curr_val;
				ret = 0;
			}
		}
		i++;
	}
	return ret;
}

static int32_t tas25xx_enum_put_idx_value(struct tas25xx_priv *p_tas25xx,
	int32_t ctrl_idx, int32_t value_idx)
{
	int ret = 0;
	int32_t count_w = g_kctrl_data[ctrl_idx].kcontrol.enum_type.count;
	int32_t channel = g_kctrl_data[ctrl_idx].kcontrol.enum_type.channel;
	struct linux_platform *plat_data =
		(struct linux_platform *) p_tas25xx->platform_data;

	dev_info(plat_data->dev, "%s kcontrol=%s with value index=%d", __func__,
		g_kctrl_data[ctrl_idx].kcontrol.enum_type.name, value_idx);

	if (value_idx >= 0 && value_idx < count_w) {
		char *mem = g_kctrl_data[ctrl_idx].kcontrol.enum_type.data[value_idx].data;
		ret = tas25xx_process_block(p_tas25xx, mem, channel);
		if (ret)
			dev_err(plat_data->dev,
				"tas25xx:%s failed ret = %d\n", __func__, ret);
	} else {
		dev_err(plat_data->dev,
			"tas25xx:%s out of bound, value_idx=%d\n", __func__, value_idx);
		ret = -EINVAL;
	}

	return ret;
}

static int32_t tas25xx_enum_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *codec = snd_soc_kcontrol_component(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct tas25xx_priv *p_tas25xx = NULL;
	//struct linux_platform *plat_data = NULL;
	int32_t ret = -EINVAL;
	int32_t i = 0;
	uint32_t misc_info;

	if ((codec == NULL) || (mc == NULL)) {
		pr_err("%s:codec or control is NULL\n", __func__);
		return ret;
	}

	p_tas25xx = snd_soc_component_get_drvdata(codec);
	if (p_tas25xx == NULL) {
		pr_err("%s:p_tas25xx is NULL\n", __func__);
		return ret;
	}
	//plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	mutex_lock(&p_tas25xx->codec_lock);

	while (i < g_no_of_kcontrols) {
		if (g_kctrl_data[i].type == 1) {
			if (strnstr(ucontrol->id.name,
						g_kctrl_data[i].kcontrol.enum_type.name, 64)) {
				int32_t v_idx = ucontrol->value.integer.value[0];
				misc_info = g_kctrl_data[i].kcontrol.enum_type.misc_info;

				/* check if it needs to be updated now */
				if (((misc_info & 0xf) == KCNTR_ANYTIME) ||
						(p_tas25xx->m_power_state == TAS_POWER_ACTIVE))
					ret = tas25xx_enum_put_idx_value(p_tas25xx, i, v_idx);
				else
					ret = 0; /* mixer control will be updated during power up */

				if (!ret)
					g_kctrl_data[i].kcontrol.enum_type.curr_val = v_idx;
			}
		}
		i++;
	}

	mutex_unlock(&p_tas25xx->codec_lock);

	return ret;
}

/*
 * Helper function to update the kcontrol local values to device
 * when something like SW/HW reset happens.
 */
int32_t tas25xx_update_kcontrol_data(struct tas25xx_priv *p_tas25xx,
	enum kcntl_during_t cur_state, uint32_t chmask)
{
	int ret = -EINVAL;
	int i = 0, kcntrl_type, misc_info;
	int chn;
	char *name = NULL;
	struct linux_platform *plat_data = NULL;
	struct tas25xx_kcontrol_int *int_type;
	struct tas25xx_kcontrol_enum *enum_type;

	if (!p_tas25xx || !g_kctrl_data) {
		pr_err("%s: p_tas25xx=%p g_kctrl_data=%p\n",
			__func__, p_tas25xx, g_kctrl_data);
		ret = -EINVAL;
		return ret;
	}

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;
	if (!plat_data)
		return ret;

	for (i = 0; i < g_no_of_kcontrols; ++i) {
		kcntrl_type = g_kctrl_data[i].type;
		ret = 0;

		switch (kcntrl_type) {
		/* integer */
		case 0:
			int_type = &g_kctrl_data[i].kcontrol.int_type;
			chn = g_kctrl_data[i].kcontrol.int_type.channel;
			if ((1 << chn) & chmask) {
				misc_info = int_type->misc_info;
				name = int_type->name;
				if (cur_state == (misc_info & 0xf))
					ret = tas25xx_int_put_idx_value(p_tas25xx, i,
						int_type->curr_val);
			}
			break;

		/* enum */
		case 1:
			enum_type = &g_kctrl_data[i].kcontrol.enum_type;
			chn = g_kctrl_data[i].kcontrol.enum_type.channel;
			if ((1 << chn) & chmask) {
				misc_info = enum_type->misc_info;
				name = enum_type->name;
				if (cur_state == (misc_info & 0xf))
					ret = tas25xx_enum_put_idx_value(p_tas25xx, i,
						enum_type->curr_val);
			}
			break;

		default:
			name = NULL;
			dev_err(plat_data->dev,
				"%s non supported kcontrol type\n", __func__);
			ret = -EINVAL;
			break;

		}

		if (ret) {
			dev_err(plat_data->dev, "%s %s mixer ctrl update err=%d\n",
				__func__, name ? name : "", ret);
			continue;
		}
	}

	return ret;
}


static int32_t tas25xx_create_common_kcontrols(struct tas25xx_priv *p_tas25xx)
{
	struct linux_platform *plat_data =
		(struct linux_platform *)p_tas25xx->platform_data;
	int32_t ret = 0;
	int32_t i = 0;

	if (!g_no_of_kcontrols) {
		dev_info(plat_data->dev, "%s no kcontrols found", __func__);
		return 0;
	}

	g_kctrl_ctrl = kzalloc(g_no_of_kcontrols * sizeof(struct snd_kcontrol_new), GFP_KERNEL);
	if (!g_kctrl_ctrl) {
		ret = -ENOMEM;
		goto EXIT;
	}
	for (i = 0; i < g_no_of_kcontrols; i++) {
		/* Integer Type */
		if (g_kctrl_data[i].type == 0) {
			g_kctrl_data[i].kcontrol.int_type.mctl.reg = i;
			g_kctrl_data[i].kcontrol.int_type.mctl.rreg = i;
			g_kctrl_data[i].kcontrol.int_type.mctl.shift = 0;
			g_kctrl_data[i].kcontrol.int_type.mctl.rshift = 0;
			g_kctrl_data[i].kcontrol.int_type.mctl.max = g_kctrl_data[i].kcontrol.int_type.count-1;
			g_kctrl_data[i].kcontrol.int_type.mctl.platform_max = g_kctrl_data[i].kcontrol.int_type.count-1;
			g_kctrl_data[i].kcontrol.int_type.mctl.invert = 0;
			g_kctrl_data[i].kcontrol.int_type.mctl.autodisable = 0;

			g_kctrl_ctrl[i].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
			g_kctrl_ctrl[i].name = g_kctrl_data[i].kcontrol.int_type.name;
			g_kctrl_ctrl[i].info = snd_soc_info_volsw;
			g_kctrl_ctrl[i].get = tas25xx_get;
			g_kctrl_ctrl[i].put = tas25xx_put;
			g_kctrl_ctrl[i].private_value =
				(unsigned long)(&(g_kctrl_data[i].kcontrol.int_type.mctl));
		} else { /* Enum Type */
			int32_t count = g_kctrl_data[i].kcontrol.enum_type.count;

			g_kctrl_data[i].kcontrol.enum_type.tas25xx_kcontrol_enum.items = count;
			g_kctrl_data[i].kcontrol.enum_type.tas25xx_kcontrol_enum.texts =
				(const char * const *)g_kctrl_data[i].kcontrol.enum_type.enum_texts;

			g_kctrl_ctrl[i].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
			g_kctrl_ctrl[i].name = g_kctrl_data[i].kcontrol.enum_type.name;
			g_kctrl_ctrl[i].info = snd_soc_info_enum_double;
			g_kctrl_ctrl[i].get = tas25xx_enum_get;
			g_kctrl_ctrl[i].put = tas25xx_enum_put;
			g_kctrl_ctrl[i].private_value =
				(unsigned long)(&(g_kctrl_data[i].kcontrol.enum_type.tas25xx_kcontrol_enum));
		}
	}
	ret = snd_soc_add_component_controls(plat_data->codec,
			g_kctrl_ctrl, g_no_of_kcontrols);
EXIT:
	return ret;
}

static int32_t tas25xx_parse_kcontrols(struct tas25xx_priv *p_tas25xx, uint8_t **inbuf)
{
	int32_t ret = 0;
	int32_t size = 0;
	int32_t j = 0, i = 0;
	int32_t count;
	struct linux_platform *plat_data = NULL;
	uint8_t *buf = *inbuf;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	buf = find_block_for_channel(p_tas25xx, buf, KCNTRL_STR, ANY_CHANNEL);
	if (!buf)
		return -EINVAL;

	buf += HDR_STR_SZ;
	size = get_block_size_noadvance(buf);
	buf += 4;
	dev_info(plat_data->dev, "parsing kcontrol data, total size=%d", size);

	memcpy(&g_no_of_kcontrols, buf, sizeof(g_no_of_kcontrols));
	buf += sizeof(g_no_of_kcontrols);

	dev_info(plat_data->dev, "%s: g_no_of_kcontrols %d\n", __func__, g_no_of_kcontrols);

	g_kctrl_data = kzalloc(g_no_of_kcontrols * sizeof(struct tas25xx_kcontrols), GFP_KERNEL);
	if (!g_kctrl_data) {
		ret = -ENOMEM;
		g_no_of_kcontrols = 0;
		goto EXIT;
	}

	p_tas25xx->playback_volume_left_index = -1;
	p_tas25xx->playback_volume_right_index = -1;

	for (i = 0; i < g_no_of_kcontrols; i++) {
		memcpy(&(g_kctrl_data[i].type), buf, 1);
		buf += 1;
		/* Integer Type */
		if (g_kctrl_data[i].type == 0) {
			if (p_tas25xx->playback_volume_left_index == -1
				&& !strcmp(buf, "PLAYBACK_VOLUME_LEFT")) {
				p_tas25xx->playback_volume_left_index = i;
				dev_dbg(plat_data->dev,
					"%s: update playback_volume_left_index = %d\n",
					__func__, p_tas25xx->playback_volume_left_index);
			}
			if (p_tas25xx->playback_volume_right_index == -1
				&& !strcmp(buf, "PLAYBACK_VOLUME_RIGHT")) {
				p_tas25xx->playback_volume_right_index = i;
				dev_dbg(plat_data->dev,
					"%s: update playback_volume_right_index = %d\n",
					__func__, p_tas25xx->playback_volume_right_index);
			}

			g_kctrl_data[i].kcontrol.int_type.name = kasprintf(GFP_KERNEL, "%s %s",
					"TAS25XX", (char *)buf);
			buf += 64;
			memcpy(&(g_kctrl_data[i].kcontrol.int_type.channel), buf, 1);
			buf += 1;
			memcpy(&(g_kctrl_data[i].kcontrol.int_type.reg), buf, 4);
			buf += 4;
			memcpy(&(g_kctrl_data[i].kcontrol.int_type.reg_type), buf, 1);
			buf += 1;
			if (g_kctrl_data[i].kcontrol.int_type.reg_type == 2) {
				memcpy(&(g_kctrl_data[i].kcontrol.int_type.mask), buf, 4);
				buf += 4;
			}
			memcpy(&(g_kctrl_data[i].kcontrol.int_type.range_min), buf, 1);
			buf += 4;
			memcpy(&(g_kctrl_data[i].kcontrol.int_type.range_max), buf, 4);
			buf += 4;
			memcpy(&(g_kctrl_data[i].kcontrol.int_type.step), buf, 4);
			buf += 4;
			memcpy(&(g_kctrl_data[i].kcontrol.int_type.def), buf, 4);
			buf += 4;
			if (s_rbin.head.version > 6) {
				memcpy(&(g_kctrl_data[i].kcontrol.int_type.misc_info), buf, 4);
				buf += 4;
			}

			/* Assign default value to current value */
			g_kctrl_data[i].kcontrol.int_type.curr_val = g_kctrl_data[i].kcontrol.int_type.def;
			dev_info(plat_data->dev, "%s: curr_kcontrol [%s] = %d\n", __func__,
					g_kctrl_data[i].kcontrol.int_type.name,
					g_kctrl_data[i].kcontrol.int_type.curr_val);

			memcpy(&(g_kctrl_data[i].kcontrol.int_type.count), buf, 4);
			buf += 4;
			if (g_kctrl_data[i].kcontrol.int_type.reg_type == 1) {
				g_kctrl_data[i].kcontrol.int_type.intdata =
						kzalloc(g_kctrl_data[i].kcontrol.int_type.count * sizeof(int32_t), GFP_KERNEL);
				for (j = 0; j < g_kctrl_data[i].kcontrol.int_type.count; j++) {
					memcpy(&(g_kctrl_data[i].kcontrol.int_type.intdata[j]), buf, 4);
					buf += 4;
				}
			} else {
				g_kctrl_data[i].kcontrol.int_type.chardata = kzalloc(g_kctrl_data[i].kcontrol.int_type.count * sizeof(char), GFP_KERNEL);
				for (j = 0; j < g_kctrl_data[i].kcontrol.int_type.count; j++) {
					memcpy(&(g_kctrl_data[i].kcontrol.int_type.chardata[j]), buf, 1);
					buf += 1;
				}
			}
		} else { /* Enum type */
			g_kctrl_data[i].kcontrol.enum_type.name = kasprintf(GFP_KERNEL, "%s %s",
					"TAS25XX", (char *)buf);
			buf += 64;
			memcpy(&(g_kctrl_data[i].kcontrol.enum_type.channel), buf, 1);
			buf += 1;
			memcpy(&(g_kctrl_data[i].kcontrol.enum_type.def), buf, 1);
			buf += 1;
			if (s_rbin.head.version > 6) {
				memcpy(&(g_kctrl_data[i].kcontrol.enum_type.misc_info), buf, 4);
				buf += 4;
			}
			g_kctrl_data[i].kcontrol.enum_type.curr_val =
				g_kctrl_data[i].kcontrol.enum_type.def;
			memcpy(&count, buf, 4);
			buf += 4;
			g_kctrl_data[i].kcontrol.enum_type.count = count;

			g_kctrl_data[i].kcontrol.enum_type.data =
				kzalloc(count * sizeof(struct tas25xx_kcontrol_enum_data), GFP_KERNEL);
			if (!g_kctrl_data[i].kcontrol.enum_type.data) {
				ret = -ENOMEM;
				goto EXIT;
			}
			for (j = 0; j < count; j++) {
				memcpy(g_kctrl_data[i].kcontrol.enum_type.data[j].name, buf, 64);
				buf += 64;
				memcpy(&size, buf, 4);
				g_kctrl_data[i].kcontrol.enum_type.data[j].data = buf;
				buf += (size + 4);
			}

			g_kctrl_data[i].kcontrol.enum_type.enum_texts = kzalloc(count * sizeof(char *), GFP_KERNEL);
			if (!g_kctrl_data[i].kcontrol.enum_type.enum_texts) {
				ret = -ENOMEM;
				goto EXIT;
			}
			for (j = 0; j < count; j++) {
				g_kctrl_data[i].kcontrol.enum_type.enum_texts[j] = kzalloc(64, GFP_KERNEL);
				if (!g_kctrl_data[i].kcontrol.enum_type.enum_texts[j]) {
					ret = -ENOMEM;
					goto EXIT;
				}
				memcpy(g_kctrl_data[i].kcontrol.enum_type.enum_texts[j],
						g_kctrl_data[i].kcontrol.enum_type.data[j].name, 64);
			}
		}
	}
EXIT:
	*inbuf = buf;
	return ret;
}

static int tas_write_init_default_config_params(struct tas25xx_priv *p_tas25xx, int number_of_channels)
{
	int i;
	int ret = 0;

	for (i = 0; i < number_of_channels; i++) {
		if (p_tas25xx->ch_failed[i]) {
			continue;
		}
		ret |= tas25xx_set_sample_rate(p_tas25xx, i, TAS25XX_DEFAULT);
		ret |= tas25xx_set_fmt_inv(p_tas25xx, i, TAS25XX_DEFAULT);
		ret |= tas25xx_set_fmt_mask(p_tas25xx, i, TAS25XX_DEFAULT);
		ret |= tas25xx_set_rx_slots(p_tas25xx, i, TAS25XX_DEFAULT);
		ret |= tas25xx_set_tx_slots(p_tas25xx, i, TAS25XX_DEFAULT);
		ret |= tas25xx_set_rx_bitwidth(p_tas25xx, i, TAS25XX_DEFAULT);
		ret |= tas25xx_set_rx_slotlen(p_tas25xx, i, TAS25XX_DEFAULT);
		ret |= tas25xx_set_tx_slotlen(p_tas25xx, i, TAS25XX_DEFAULT);
	}

	return ret;
}

int tas_write_init_config_params(struct tas25xx_priv *p_tas25xx, int number_of_channels)
{
	return tas_write_init_default_config_params(p_tas25xx, number_of_channels);
}

static int32_t tas25xx_parse_algo_data(struct tas25xx_priv *p_tas25xx, uint8_t **inbuf)
{
	int32_t ret = 0;
	struct linux_platform *plat_data = NULL;
	uint8_t *buf = *inbuf;
	uint32_t size;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	buf = find_block_for_channel(p_tas25xx, buf, ALGOP_STR, ANY_CHANNEL);
	if (!buf)
		return -EINVAL;

	buf += HDR_STR_SZ;
	size = get_block_size_noadvance(buf);
	buf += 4;
	dev_info(plat_data->dev, "parsing algo data, total size=%d", size);

#if IS_ENABLED(CONFIG_TAS25XX_ALGO)
	tas25xx_parse_algo_bin(p_tas25xx->ch_count, buf);
#endif /* CONFIG_TAS25XX_ALGO */

	return ret;
}

static void tas25xx_fw_ready(const struct firmware *pFW, void *pContext)
{
	int32_t ret;
	uint32_t size;
	int32_t i;
	int32_t number_of_channels;
	uint32_t rev_count;
	uint32_t rev_id;
	uint64_t file_offset;
	struct tas25xx_priv *p_tas25xx = (struct tas25xx_priv *) pContext;
	struct linux_platform *plat_data = NULL;
	struct i2c_client *p_client = NULL;
	uint8_t *bin_data = NULL;
	uint8_t *mdata;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;
	p_client = container_of(plat_data->dev,
		struct i2c_client, dev);

	if (unlikely(!pFW) || unlikely(!pFW->data)) {
		dev_err(plat_data->dev,
			"firmware is not ready, %s\n", TAS25XX_BINFILE_NAME);
		atomic_set(&p_tas25xx->fw_state, TAS25XX_DSP_FW_TRYLOAD);
		goto EXIT;
	}

	dev_info(plat_data->dev, "%s: pFW->size %d", __func__, (int32_t)pFW->size);
	atomic_set(&p_tas25xx->fw_state, TAS25XX_DSP_FW_PARSE_FAIL);

	mdata = (uint8_t *)pFW->data;
	if (header_check(p_tas25xx, MDATA, mdata)) {
		dev_info(plat_data->dev, "%s: Found metadata, check against rev-id %d",
			__func__, p_tas25xx->dev_revid);
		file_offset = 0;
		size = 0;
		mdata += 5; /* header */
		memcpy(&rev_count, mdata, sizeof(uint32_t));
		mdata += 4; /* count */
		for (i = 0; i < rev_count; i++) {
			file_offset += size;

			memcpy(&rev_id, mdata, sizeof(uint32_t));
			mdata += 4; /* id */
			memcpy(&size, mdata, sizeof(uint32_t));
			mdata += 4; /* file sz */
			/* dummy data */
			mdata += 4;
			mdata += 4;
			mdata += 4;
			dev_info(plat_data->dev, "%s: revid 0x%x, size=%d, offset=%llu",
				__func__, rev_id, size, file_offset);
			if (p_tas25xx->dev_revid == rev_id)
				break;
		}

		if (i == rev_count) {
			dev_info(plat_data->dev,
				"%s: metadata parse failure...", __func__);
			ret = -EINVAL;
			goto EXIT;
		} else {
			dev_info(plat_data->dev,
				"%s: using idx=%d revid 0x%x, size=%d", __func__, i, rev_id, size);
			p_tas25xx->fw_data = kzalloc(size, GFP_KERNEL);
			if (!p_tas25xx->fw_data) {
				ret = -ENOMEM;
				goto EXIT;
			}

			bin_data = (uint8_t *)pFW->data;
			bin_data += MDATA_HDR_SZ;

			/* first bin file */
			bin_data += (rev_count * ONE_BIN_MD_SZ) + file_offset;


			p_tas25xx->fw_size = size;
			memcpy(p_tas25xx->fw_data, bin_data, size);
		}

	} else {
		dev_info(plat_data->dev, "%s: metadata not found, regular fw", __func__);
			p_tas25xx->fw_data = kzalloc(pFW->size, GFP_KERNEL);
		if (!p_tas25xx->fw_data) {
			ret = -ENOMEM;
			goto EXIT;
		}
		p_tas25xx->fw_size = pFW->size;
		memcpy(p_tas25xx->fw_data, pFW->data, pFW->size);
	}

	bin_data = p_tas25xx->fw_data;

	if (!header_check(p_tas25xx, HEADER, bin_data)) {
		ret = -EINVAL;
		goto EXIT;
	} else {
		bin_data += HDR_STR_SZ;
		size = get_block_size_noadvance(bin_data);
		dev_info(plat_data->dev, "%s: Header size %d", __func__, size);
		bin_data += 4;
	}

	memcpy(&s_rbin.head, bin_data, sizeof(s_rbin.head));
	bin_data += sizeof(s_rbin.head);

	memcpy(&s_rbin.def_hw_params, bin_data, sizeof(struct default_hw_params));
	bin_data += sizeof(struct default_hw_params);

	/* skip dummy data */
	bin_data += (sizeof(int32_t) * 5);

	number_of_channels = s_rbin.head.channels;

	dev_info(plat_data->dev, "%s: version 0x%x", __func__, s_rbin.head.version);
	dev_info(plat_data->dev, "%s: name %s", __func__, s_rbin.head.name);
	dev_info(plat_data->dev, "%s: timestamp %d", __func__, s_rbin.head.timestamp);
	dev_info(plat_data->dev, "%s: channels %d", __func__, number_of_channels);

	for (i = 0; i < number_of_channels; i++) {
		dev_info(plat_data->dev, "%s: device_%d %d", __func__, i,
				s_rbin.head.dev[i].device);
		if (p_tas25xx->devs[i]->mn_addr < 0) {
			dev_info(plat_data->dev, "%s: Updating the device_%d i2c address to 0x%x from invalid",
				__func__, i, s_rbin.head.dev[i].i2c_addr);
			p_tas25xx->devs[i]->mn_addr = s_rbin.head.dev[i].i2c_addr;
		}
	}

	for (i = 0; i < number_of_channels; i++)
		if (p_tas25xx->devs[i]->mn_addr == p_client->addr)
			break;
	if (i == number_of_channels)
		dev_warn(plat_data->dev,
			"Atlest one address should be %d", p_client->addr);

	dev_info(plat_data->dev, "%s: IVSenseWidth %d", __func__, (int)s_rbin.head.iv_width);
	dev_info(plat_data->dev, "%s: Vbat-mon %d", __func__, (int)s_rbin.head.vbat_mon);
	p_tas25xx->mn_iv_width = p_tas25xx->curr_mn_iv_width = (int)s_rbin.head.iv_width;
	p_tas25xx->mn_vbat = p_tas25xx->curr_mn_vbat = (int)s_rbin.head.vbat_mon;
	dev_dbg(plat_data->dev, "%s: updating number of channels %d -> %d",
		__func__, p_tas25xx->ch_count, number_of_channels);
	p_tas25xx->ch_count = number_of_channels;

	dev_dbg(plat_data->dev, "%s: features %d", __func__, s_rbin.head.features);
	dev_dbg(plat_data->dev, "%s: sample_rate %s", __func__, SampleRate[(int32_t)s_rbin.def_hw_params.sample_rate]);
	dev_dbg(plat_data->dev, "%s: fmt_inv %s", __func__, FMT_INV[(int32_t)s_rbin.def_hw_params.fmt_inv]);
	dev_dbg(plat_data->dev, "%s: fmt_mask %s", __func__, FMT_MASK[(int32_t)s_rbin.def_hw_params.fmt_mask]);
	dev_dbg(plat_data->dev, "%s: rx_slots %s", __func__, RX_SLOTS[(int32_t)s_rbin.def_hw_params.rx_slots]);
	dev_dbg(plat_data->dev, "%s: tx_slots %s", __func__, TX_SLOTS[(int32_t)s_rbin.def_hw_params.tx_slots]);
	dev_dbg(plat_data->dev, "%s: rx_bitwidth %s", __func__, RX_SLOTS[(int32_t)s_rbin.def_hw_params.rx_bitwidth]);
	dev_dbg(plat_data->dev, "%s: rx_slotlen %s", __func__, RX_SLOTS[(int32_t)s_rbin.def_hw_params.rx_slotlen]);
	dev_dbg(plat_data->dev, "%s: tx_slotlen %s", __func__, TX_SLOTS[(int32_t)s_rbin.def_hw_params.tx_slotlen]);

	if (s_rbin.head.version > SUPPORTED_BIN_VERSION) {
		dev_err(plat_data->dev,
			"%s: version not compatible. Supported version <= 0x%x", __func__,
			SUPPORTED_BIN_VERSION);
		goto EXIT;
	}

	for (i = 0; i < number_of_channels; i++) {
		/* Parsing Init Params */
		ret = tas25xx_parse_init_params(p_tas25xx, &bin_data, i);
		if (ret) {
			dev_err(plat_data->dev, "%s: Init Parsing failed", __func__);
			goto EXIT;
		}
		ret = tas25xx_parse_hw_params(p_tas25xx, &bin_data, i);
		if (ret) {
			dev_err(plat_data->dev,
				"%s: HW params parsing failed, ignoring..", __func__);
		}
		ret = tas25xx_parse_profiles(p_tas25xx, &bin_data, i);
		if (ret) {
			dev_err(plat_data->dev, "%s: profiles failed", __func__);
			goto EXIT;
		}
		ret = tas25xx_parse_interrupts(p_tas25xx, &bin_data, i);
		if (ret) {
			dev_info(plat_data->dev,
				"%s: interrupt parsing failed, ignoring..", __func__);
		}

		ret = tas25xx_parse_block_data(p_tas25xx, &bin_data, i);
		if (ret) {
			dev_err(plat_data->dev, "%s: block data", __func__);
			goto EXIT;
		}
	}

	ret = tas25xx_parse_kcontrols(p_tas25xx, &bin_data);
	if (ret)
		dev_info(plat_data->dev,
			"%s: Error while parsing the kcontrols, ignored..", __func__);

	ret = tas25xx_parse_algo_data(p_tas25xx, &bin_data);
	if (ret)
		dev_info(plat_data->dev,
			"%s: Error while parsing the algo data, ignored..", __func__);

	dev_dbg(plat_data->dev, "%s: Firmware init complete\n", __func__);

	atomic_set(&p_tas25xx->fw_state, TAS25XX_DSP_FW_OK);

EXIT:
	atomic_set(&p_tas25xx->fw_wait_complete, 1);
	wake_up(&p_tas25xx->fw_wait);

	if (pFW)
		release_firmware(pFW);
}


int32_t tas25xx_load_firmware(struct tas25xx_priv *p_tas25xx, int max_fw_tryload_count)
{
	int32_t ret;
	uint32_t retry_count = 0;
	struct linux_platform *plat_data = NULL;

	g_tas25xx = p_tas25xx;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	if (!max_fw_tryload_count)
		max_fw_tryload_count = DSP_FW_LOAD_NTRIES;
	atomic_set(&p_tas25xx->fw_wait_complete, 0);

	do {
		ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_UEVENT,
				TAS25XX_BINFILE_NAME, plat_data->dev, GFP_KERNEL,
				p_tas25xx, tas25xx_fw_ready);
		if (ret) {
			dev_err(plat_data->dev, "request_firmware_nowait failed, err=%d", ret);
			msleep(100);
		} else {
			dev_info(plat_data->dev, "fw load requested, trail=%d", retry_count);
			/* wait for either firmware to be loaded or failed */
			ret = wait_event_interruptible(p_tas25xx->fw_wait,
				atomic_read(&p_tas25xx->fw_wait_complete));
			if (ret)
				dev_err(plat_data->dev, "wait failed with error %d", ret);

			/* any other state other than retry */
			if (atomic_read(&p_tas25xx->fw_state) != TAS25XX_DSP_FW_TRYLOAD)
				break;

			atomic_set(&p_tas25xx->fw_wait_complete, 0);
			msleep(100);
		}

		retry_count++;
	} while (retry_count < max_fw_tryload_count);

	if (retry_count == max_fw_tryload_count)
		atomic_set(&p_tas25xx->fw_state, TAS25XX_DSP_FW_LOAD_FAIL);

	switch (atomic_read(&p_tas25xx->fw_state)) {
	case TAS25XX_DSP_FW_LOAD_FAIL:
		ret = -ENOENT;
		break;
	case TAS25XX_DSP_FW_OK:
		ret = 0;
		break;
	case TAS25XX_DSP_FW_PARSE_FAIL:
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int32_t tas25xx_create_kcontrols(struct tas25xx_priv *p_tas25xx)
{
	int32_t ret = -1;
	int fw_state;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	fw_state = atomic_read(&p_tas25xx->fw_state);
	if (fw_state == TAS25XX_DSP_FW_OK) {
		ret = tas25xx_create_common_kcontrols(p_tas25xx);
		if (ret) {
			dev_err(plat_data->dev, "%s: kcontrols failed", __func__);
			goto EXIT;
		}
		ret = tas25xx_create_profile_controls(p_tas25xx);
		if (ret) {
			dev_err(plat_data->dev, "%s: kcontrols failed", __func__);
			goto EXIT;
		}
		ret = tas25xx_create_custom_controls(p_tas25xx);
		if (ret) {
			dev_err(plat_data->dev, "%s: custom kcontrols failed", __func__);
			goto EXIT;
		}
	} else {
		dev_err(plat_data->dev, "%s: firmware not loaded\n", __func__);
	}
EXIT:
	return ret;
}

int32_t tas25xx_set_init_params(struct tas25xx_priv *p_tas25xx, int32_t ch)
{
	int32_t ret = -1;
	int fw_state;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	dev_info(plat_data->dev, "%s: ch-%d\n", __func__, ch);
	fw_state = atomic_read(&p_tas25xx->fw_state);
	if (fw_state == TAS25XX_DSP_FW_OK)
		ret = tas25xx_process_block(p_tas25xx, s_rbin.init_params[ch], ch);
	else
		dev_err(plat_data->dev, "%s: firmware not loaded\n", __func__);

	return ret;
}

int32_t tas25xx_set_sample_rate(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t sample_rate)
{
	int32_t ret;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	p_tas25xx->sample_rate = sample_rate;

	switch (sample_rate) {
	case 44100:
		dev_info(plat_data->dev, "%s: ch-%d SampleRate 44100\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].SampleRate[SampleRate_44100], ch);
		break;

	case 96000:
		dev_info(plat_data->dev, "%s: ch-%d SampleRate 96000\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].SampleRate[SampleRate_96000], ch);
		break;

	case 48000:
		dev_info(plat_data->dev, "%s: ch-%d SampleRate 48000\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].SampleRate[SampleRate_48000], ch);
		break;

	case TAS25XX_DEFAULT:
	default:
		p_tas25xx->sample_rate = TAS25XX_DEFAULT;
		dev_info(plat_data->dev, "%s:[default] ch-%d SampleRate %s [default]\n", __func__, ch,
				SampleRate[(int32_t)s_rbin.def_hw_params.sample_rate]);
		ret = tas25xx_process_block(p_tas25xx,
			s_rbin.hw_params[ch].SampleRate[(int32_t)s_rbin.def_hw_params.sample_rate], ch);
		break;
	}
	return ret;
}

int32_t tas25xx_set_fmt_inv(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t fmt_inv)
{
	int32_t ret = -1;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	switch (fmt_inv) {
	case FMT_INV_NB_NF:
		dev_info(plat_data->dev, "%s: ch-%d FMT_INV_NB_NF\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].FMT_INV[FMT_INV_NB_NF], ch);
		break;
	case FMT_INV_IB_NF:
		dev_info(plat_data->dev, "%s: ch-%d FMT_INV_IB_NF\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].FMT_INV[FMT_INV_IB_NF], ch);
		break;
	case FMT_INV_NB_IF:
		dev_info(plat_data->dev, "%s: ch-%d FMT_INV_NB_IF\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].FMT_INV[FMT_INV_NB_IF], ch);
		break;
	case FMT_INV_IB_IF:
		dev_info(plat_data->dev, "%s: ch-%d FMT_INV_IB_IF\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].FMT_INV[FMT_INV_IB_IF], ch);
		break;
	case TAS25XX_DEFAULT:
		dev_info(plat_data->dev, "%s:[default] ch-%d %s\n", __func__, ch,
			FMT_INV[(int32_t)s_rbin.def_hw_params.fmt_inv]);
		ret = tas25xx_process_block(p_tas25xx,
			s_rbin.hw_params[ch].FMT_INV[(int32_t)s_rbin.def_hw_params.fmt_inv], ch);
		break;
	default:
		dev_info(plat_data->dev, "%s: ch-%d Invalid FMT_INV %d\n", __func__, ch,
				fmt_inv);
		break;
	}
	return ret;
}

int32_t tas25xx_set_fmt_mask(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t fmt_mask)
{
	int32_t ret = -1;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	switch (fmt_mask) {
	case FMT_MASK_I2S:
		dev_info(plat_data->dev, "%s: ch-%d FMT_MASK_I2S\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx,
			s_rbin.hw_params[ch].FMT_MASK[FMT_MASK_I2S], ch);
		break;
	case FMT_MASK_DSP_A:
		dev_info(plat_data->dev, "%s: ch-%d FMT_MASK_DSP_A\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx,
			s_rbin.hw_params[ch].FMT_MASK[FMT_MASK_DSP_A], ch);
		break;
	case FMT_MASK_DSP_B:
		dev_info(plat_data->dev, "%s: ch-%d FMT_MASK_DSP_B\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx,
			s_rbin.hw_params[ch].FMT_MASK[FMT_MASK_DSP_B], ch);
		break;
	case FMT_MASK_LEFT_J:
		dev_info(plat_data->dev, "%s: ch-%d FMT_MASK_LEFT_J\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx,
			s_rbin.hw_params[ch].FMT_MASK[FMT_MASK_LEFT_J], ch);
		break;
	case TAS25XX_DEFAULT:
		dev_info(plat_data->dev, "%s:[default] ch-%d %s\n", __func__, ch,
				FMT_MASK[(int32_t)s_rbin.def_hw_params.fmt_mask]);
		ret = tas25xx_process_block(p_tas25xx,
			s_rbin.hw_params[ch].FMT_MASK[(int32_t)s_rbin.def_hw_params.fmt_mask], ch);
		break;
	default:
		dev_info(plat_data->dev, "%s: ch-%d Invalid FMT_MASK %d\n", __func__, ch,
				fmt_mask);
		break;
	}
	return ret;
}

int32_t tas25xx_set_rx_slots(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t rx_slot)
{
	int32_t ret = -1;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	switch (rx_slot) {
	case RX_SLOTS_16:
		dev_info(plat_data->dev, "%s: ch-%d RX_SLOTS_16\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].RX_SLOTS[RX_SLOTS_16], ch);
		break;
	case RX_SLOTS_24:
		dev_info(plat_data->dev, "%s: ch-%d RX_SLOTS_24\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].RX_SLOTS[RX_SLOTS_24], ch);
		break;
	case RX_SLOTS_32:
		dev_info(plat_data->dev, "%s: ch-%d RX_SLOTS_32\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].RX_SLOTS[RX_SLOTS_32], ch);
		break;
	case TAS25XX_DEFAULT:
		dev_info(plat_data->dev, "%s:[default] ch-%d %s\n", __func__, ch,
				RX_SLOTS[(int32_t)s_rbin.def_hw_params.rx_slots]);
		ret = tas25xx_process_block(p_tas25xx,
			s_rbin.hw_params[ch].RX_SLOTS[(int32_t)s_rbin.def_hw_params.rx_slots], ch);
		break;
	default:
		dev_info(plat_data->dev, "%s: ch-%d Invalid RX_SLOTS %d\n", __func__, ch,
				rx_slot);
		break;
	}
	return ret;
}

int32_t tas25xx_set_tx_slots(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t tx_slot)
{
	int32_t ret = -1;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	switch (tx_slot) {
	case TX_SLOTS_16:
		dev_info(plat_data->dev, "%s: ch-%d TX_SLOTS_16\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].TX_SLOTS[TX_SLOTS_16], ch);
		break;
	case TX_SLOTS_24:
		dev_info(plat_data->dev, "%s: ch-%d TX_SLOTS_24\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].TX_SLOTS[TX_SLOTS_24], ch);
		break;
	case TX_SLOTS_32:
		dev_info(plat_data->dev, "%s: ch-%d TX_SLOTS_32\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].TX_SLOTS[TX_SLOTS_32], ch);
		break;
	case TAS25XX_DEFAULT:
		dev_info(plat_data->dev, "%s:[default] ch-%d %s\n", __func__, ch,
				RX_SLOTS[(int32_t)s_rbin.def_hw_params.tx_slots]);
		ret = tas25xx_process_block(p_tas25xx,
			s_rbin.hw_params[ch].TX_SLOTS[(int32_t)s_rbin.def_hw_params.tx_slots], ch);
		break;
	default:
		dev_info(plat_data->dev, "%s: ch-%d Invalid TX_SLOTS %d\n", __func__, ch,
				tx_slot);
		break;
	}
	return ret;
}

int32_t tas25xx_set_rx_bitwidth(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t rx_bitwidth)
{
	int32_t ret = -1;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	switch (rx_bitwidth) {
	case RX_BITWIDTH_16:
		dev_info(plat_data->dev, "%s: ch-%d RX_BITWIDTH_16\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].RX_BITWIDTH[RX_BITWIDTH_16], ch);
		break;
	case RX_BITWIDTH_24:
		dev_info(plat_data->dev, "%s: ch-%d RX_BITWIDTH_24\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].RX_BITWIDTH[RX_BITWIDTH_24], ch);
		break;
	case RX_BITWIDTH_32:
		dev_info(plat_data->dev, "%s: ch-%d RX_BITWIDTH_32\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].RX_BITWIDTH[RX_BITWIDTH_32], ch);
		break;
	case TAS25XX_DEFAULT:
		dev_info(plat_data->dev, "%s:[default]  ch-%d %s\n", __func__, ch,
				RX_SLOTS[(int32_t)s_rbin.def_hw_params.rx_bitwidth]);
		ret = tas25xx_process_block(p_tas25xx,
			s_rbin.hw_params[ch].RX_BITWIDTH[(int32_t)s_rbin.def_hw_params.rx_bitwidth], ch);
		break;
	default:
		dev_info(plat_data->dev, "%s: ch-%d Invalid RX_BITWIDTH %d\n", __func__, ch,
				rx_bitwidth);
		break;
	}
	return ret;
}

int32_t tas25xx_set_rx_slotlen(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t rx_slotlen)
{
	int32_t ret = -1;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	switch (rx_slotlen) {
	case RX_SLOTLEN_16:
		dev_info(plat_data->dev, "%s: ch-%d RX_SLOTLEN_16\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].RX_SLOTLEN[RX_SLOTLEN_16], ch);
		break;
	case RX_SLOTLEN_24:
		dev_info(plat_data->dev, "%s: ch-%d RX_SLOTLEN_24\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].RX_SLOTLEN[RX_SLOTLEN_24], ch);
		break;
	case RX_SLOTLEN_32:
		dev_info(plat_data->dev, "%s: ch-%d RX_SLOTLEN_32\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].RX_SLOTLEN[RX_SLOTLEN_32], ch);
		break;
	case TAS25XX_DEFAULT:
		dev_info(plat_data->dev, "%s:[default]  ch-%d %s\n", __func__, ch,
				RX_SLOTS[(int32_t)s_rbin.def_hw_params.rx_slotlen]);
		ret = tas25xx_process_block(p_tas25xx,
			s_rbin.hw_params[ch].RX_SLOTLEN[(int32_t)s_rbin.def_hw_params.rx_slotlen], ch);
		break;
	default:
		dev_info(plat_data->dev, "%s: ch-%d Invalid RX_SLOTLEN %d\n", __func__, ch,
				rx_slotlen);
		break;
	}
	return ret;
}

int32_t tas25xx_set_tx_slotlen(struct tas25xx_priv *p_tas25xx, int32_t ch, int32_t tx_slotlen)
{
	int32_t ret = -1;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	switch (tx_slotlen) {
	case TX_SLOTLEN_16:
		dev_info(plat_data->dev, "%s: ch-%d TX_SLOTLEN_16\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].TX_SLOTLEN[TX_SLOTLEN_16], ch);
		break;
	case TX_SLOTLEN_24:
		dev_info(plat_data->dev, "%s: ch-%d TX_SLOTLEN_24\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].TX_SLOTLEN[TX_SLOTLEN_24], ch);
		break;
	case TX_SLOTLEN_32:
		dev_info(plat_data->dev, "%s: ch-%d TX_SLOTLEN_32\n", __func__, ch);
		ret = tas25xx_process_block(p_tas25xx, s_rbin.hw_params[ch].TX_SLOTLEN[TX_SLOTLEN_32], ch);
		break;
	case TAS25XX_DEFAULT:
		dev_info(plat_data->dev, "%s:[default] ch-%d %s\n", __func__, ch,
				RX_SLOTS[(int32_t)s_rbin.def_hw_params.tx_slotlen]);
		ret = tas25xx_process_block(p_tas25xx,
			s_rbin.hw_params[ch].TX_SLOTLEN[(int32_t)s_rbin.def_hw_params.tx_slotlen], ch);
		break;
	default:
		dev_info(plat_data->dev, "%s: ch-%d Invalid TX_SLOTLEN %d\n", __func__, ch,
				tx_slotlen);
		break;
	}
	return ret;
}

int32_t tas25xx_set_pre_powerup(struct tas25xx_priv *p_tas25xx, int32_t ch)
{
	int32_t ret = 0;
	struct linux_platform *plat_data = NULL;
	struct tas25xx_profiles *tas_profiles_t =
		(struct tas25xx_profiles *)g_profile_data_list[ch].tas_profiles;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	dev_info(plat_data->dev, "ch-%d %s profile %s pre_power_up\n", ch, __func__,
			tas_profiles_t[g_tas25xx_profile].name);
	ret = tas25xx_process_block(p_tas25xx, tas_profiles_t[g_tas25xx_profile].pre_power_up, ch);
	return ret;
}

int32_t tas25xx_set_post_powerup(struct tas25xx_priv *p_tas25xx, int32_t ch)
{
	int32_t ret = -1;
	struct linux_platform *plat_data = NULL;
	struct tas25xx_profiles *tas_profiles_t =
		(struct tas25xx_profiles *)g_profile_data_list[ch].tas_profiles;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	dev_info(plat_data->dev, "ch-%d %s profile %s post_power_up\n", ch,
		__func__, tas_profiles_t[g_tas25xx_profile].name);
	ret = tas25xx_process_block(p_tas25xx, tas_profiles_t[g_tas25xx_profile].post_power_up, ch);
	return ret;
}

int32_t tas25xx_set_pre_powerdown(struct tas25xx_priv *p_tas25xx, int32_t ch)
{
	int32_t ret = -1;
	struct linux_platform *plat_data = NULL;
	struct tas25xx_profiles *tas_profiles_t = (struct tas25xx_profiles *)g_profile_data_list[ch].tas_profiles;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	dev_info(plat_data->dev, "ch-%d %s profile %s pre_power_down\n",
		ch, __func__, tas_profiles_t[g_tas25xx_profile].name);
	ret = tas25xx_process_block(p_tas25xx, tas_profiles_t[g_tas25xx_profile].pre_power_down, ch);
	return ret;
}

int32_t tas25xx_set_post_powerdown(struct tas25xx_priv *p_tas25xx, int32_t ch)
{
	int32_t ret = -1;
	struct linux_platform *plat_data = NULL;
	struct tas25xx_profiles *tas_profiles_t = (struct tas25xx_profiles *)g_profile_data_list[ch].tas_profiles;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;

	dev_info(plat_data->dev, "ch-%d %s profile %s post_power_down\n",
		ch, __func__, tas_profiles_t[g_tas25xx_profile].name);
	ret = tas25xx_process_block(p_tas25xx, tas_profiles_t[g_tas25xx_profile].post_power_down, ch);
	return ret;
}

int32_t tas25xx_remove_binfile(struct tas25xx_priv *p_tas25xx)
{
	int32_t i = 0, j = 0;
	int32_t count;

	/* firmware not loaded */
	if (p_tas25xx)
		atomic_set(&p_tas25xx->fw_state, TAS25XX_DSP_FW_NONE);

	if (g_profile_list) {
		for (i = 0; i < g_no_of_profiles; i++) {
			kfree(g_profile_list[i]);
			g_profile_list[i] = NULL;
		}
		kfree(g_profile_list);
		g_profile_list = NULL;
	}

	for (i = 0; i < s_rbin.head.channels; i++) {
		kfree(g_profile_data_list[i].tas_profiles);
		g_profile_data_list[i].tas_profiles = NULL;
	}

	if (p_tas25xx) {
		for (i = 0; i < s_rbin.head.channels; i++) {
			kfree(p_tas25xx->intr_data[i].intr_info);
			p_tas25xx->intr_data[i].intr_info = NULL;
		}
	}

	if (g_kctrl_data) {
		for (i = 0; i < g_no_of_kcontrols; i++) {
			if (g_kctrl_data[i].type != 0) {
				count = g_kctrl_data[i].kcontrol.enum_type.count;
				for (j = 0; j < count; j++) {
					kfree(g_kctrl_data[i].kcontrol.enum_type.enum_texts[j]);
					g_kctrl_data[i].kcontrol.enum_type.enum_texts[j] = NULL;
				}
				kfree(g_kctrl_data[i].kcontrol.enum_type.enum_texts);
				g_kctrl_data[i].kcontrol.enum_type.enum_texts = NULL;

				kfree(g_kctrl_data[i].kcontrol.enum_type.data);
				g_kctrl_data[i].kcontrol.enum_type.data = NULL;
			} else {
				kfree(g_kctrl_data[i].kcontrol.int_type.intdata);
				g_kctrl_data[i].kcontrol.int_type.intdata = NULL;
				kfree(g_kctrl_data[i].kcontrol.int_type.chardata);
				g_kctrl_data[i].kcontrol.int_type.chardata = NULL;
			}
		}

		kfree(g_kctrl_data);
		g_kctrl_data = NULL;
	}

	kfree(g_kctrl_ctrl);
	g_kctrl_ctrl = NULL;

	kfree(p_tas25xx->fw_data);
	p_tas25xx->fw_data = NULL;

	g_no_of_kcontrols = 0;
	g_no_of_profiles = 0;
	g_tas25xx_profile = 0;
	memset(&s_rbin, 0, sizeof(s_rbin));

	g_tas25xx = NULL;

	return 0;
}

int32_t tas25xx_update_playback_volume(struct tas25xx_priv *p_tas25xx, int32_t ch)
{
	int32_t kcontrol_index;
	int32_t v_idx;
	int32_t misc_info;
	int32_t ret = 0;
	struct linux_platform *plat_data = NULL;

	plat_data = (struct linux_platform *) p_tas25xx->platform_data;
	kcontrol_index = (ch == 0)?p_tas25xx->playback_volume_left_index:
			p_tas25xx->playback_volume_right_index;

	if (kcontrol_index < 0) {
		dev_err(plat_data->dev, "ch-%d %s invalid kcontrol index %d\n",
				ch, __func__, kcontrol_index);
		return -EINVAL;
	}
	v_idx = g_kctrl_data[kcontrol_index].kcontrol.int_type.curr_val;
	misc_info = g_kctrl_data[kcontrol_index].kcontrol.int_type.misc_info;

	if (((misc_info & 0xf) == KCNTR_ANYTIME) ||
		(p_tas25xx->m_power_state == TAS_POWER_ACTIVE))
		ret = tas25xx_int_put_idx_value(p_tas25xx, kcontrol_index, v_idx);

	return ret;
}

