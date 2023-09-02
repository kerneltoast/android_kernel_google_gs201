/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */
#ifndef __DW3000_CALIB_H
#define __DW3000_CALIB_H

/**
 * DW3000_CALIBRATION_ANTENNA_MAX - number of antenna
 */
#define DW3000_CALIBRATION_ANTENNA_MAX 4

/**
 * enum dw3000_calibration_channels - calibration channel number.
 * @DW3000_CALIBRATION_CHANNEL_5: index in array for channel 5
 * @DW3000_CALIBRATION_CHANNEL_9: index in array for channel 9
 * @DW3000_CALIBRATION_CHANNEL_MAX: channel array size
 */
enum dw3000_calibration_channels {
	DW3000_CALIBRATION_CHANNEL_5,
	DW3000_CALIBRATION_CHANNEL_9,

	DW3000_CALIBRATION_CHANNEL_MAX
};

/**
 * enum dw3000_calibration_prfs - calibration Pulse Repetition Frequency.
 * @DW3000_CALIBRATION_PRF_16MHZ: index in array for prf 16
 * @DW3000_CALIBRATION_PRF_64MHZ: index in array for prf 64
 * @DW3000_CALIBRATION_PRF_MAX: prf array size
 */
enum dw3000_calibration_prfs {
	DW3000_CALIBRATION_PRF_16MHZ,
	DW3000_CALIBRATION_PRF_64MHZ,

	DW3000_CALIBRATION_PRF_MAX
};

/**
 * DW3000_CALIBRATION_PDOA_LUT_MAX - number of value in PDOA LUT table
 */
#define DW3000_CALIBRATION_PDOA_LUT_MAX 31

/* Intermediate types to fix following error:
 * [kernel-doc ERROR] : can't parse typedef!
 */
typedef s16 pdoa_lut_entry_t[2];
typedef pdoa_lut_entry_t pdoa_lut_table_t[DW3000_CALIBRATION_PDOA_LUT_MAX];

/**
 * typedef dw3000_pdoa_lut_t - PDoA LUT array type
 */
typedef pdoa_lut_table_t dw3000_pdoa_lut_t;

/* Default LUTs, theorical values for Monalisa antenna (20.8mm) */
extern const dw3000_pdoa_lut_t dw3000_default_lut_ch5;
extern const dw3000_pdoa_lut_t dw3000_default_lut_ch9;

/**
 * DW3000_DEFAULT_ANT_DELAY - antenna delay default value
 */
#define DW3000_DEFAULT_ANT_DELAY 16450

/**
 * struct dw3000_channel_calib - per-channel dependent calibration parameters
 * @pll_locking_code: PLL locking code
 * @wifi_coex_enabled: WiFi coexistence activation
 */
struct dw3000_channel_calib {
	/* chY.pll_locking_code */
	u32 pll_locking_code;
	bool wifi_coex_enabled;
};

/**
 * struct dw3000_antenna_calib_prf - antenna calibration parameters
 * @ant_delay: antenna delay
 * @tx_power: tx power
 * @pg_count: PG count
 * @pg_delay: PG delay
 */
struct dw3000_antenna_calib_prf {
	u32 ant_delay;
	u32 tx_power;
	u8 pg_count;
	u8 pg_delay;
};

/**
 * struct dw3000_antenna_calib - per-antenna dependent calibration parameters
 * @ch: table of channels dependent calibration values
 * @ch.prf: table of PRF dependent calibration values
 * @port: port value this antenna belong to (0 for RF1, 1 for RF2)
 * @selector_gpio: GPIO number to select this antenna
 * @selector_gpio_value: GPIO value to select this antenna
 * @caps: antenna capabilities
 */
struct dw3000_antenna_calib {
	/* antX.chY.prfZ.* */
	struct {
		struct dw3000_antenna_calib_prf prf[DW3000_CALIBRATION_PRF_MAX];
	} ch[DW3000_CALIBRATION_CHANNEL_MAX];
	/* antX.* */
	u8 port, selector_gpio, selector_gpio_value, caps;
};

/**
 * struct dw3000_antenna_pair_calib_chan - per-channel antennas pair calibration
 *  parameters
 * @pdoa_offset: PDOA offset
 * @pdoa_lut: PDOA LUT
 */
struct dw3000_antenna_pair_calib_chan {
	s16 pdoa_offset;
	dw3000_pdoa_lut_t pdoa_lut;
};

/**
 * struct dw3000_antenna_pair_calib - antenna pair dependent calibration values
 * @ch: table of channels dependent calibration values
 */
struct dw3000_antenna_pair_calib {
	/* antX.antW.chY.* */
	struct dw3000_antenna_pair_calib_chan ch[DW3000_CALIBRATION_CHANNEL_MAX];
};

/* Just to ease reading of the following formulas. */
#define ANTMAX DW3000_CALIBRATION_ANTENNA_MAX

/**
 * ANTPAIR_MAX - calculated antpair table size
 */
#define ANTPAIR_MAX ((ANTMAX * (ANTMAX - 1)) / 2)

/**
 * ANTPAIR_OFFSET - calculate antpair table indexes row offset
 * @x: first antenna index
 *
 * Return: An index for the first element in antpair table for the given value.
 */
#define ANTPAIR_OFFSET(x) ((((2 * (ANTMAX - 1)) + 1 - (x)) * (x)) / 2)

/**
 * ANTPAIR_IDX - calculate antpair table indexes
 * @x: first antenna index
 * @w: second antenna index, must be > @x
 *
 * Return: An index for the antpair table in [0;ANTPAIR_MAX-1] interval.
 */
#define ANTPAIR_IDX(x, w) (ANTPAIR_OFFSET(x) + ((w) - (x)-1))

/**
 * ANTSET_ID_MAX - calculated antenna set id table size
 */
#define ANTSET_ID_MAX (ANTPAIR_MAX + ANTMAX)

/**
 * dw3000_calib_ant_set_id_to_ant - convert antenna set id pair to corresponding antennas
 * @ant_set_id: antenna set id
 * @ant_idx1: first antenna
 * @ant_idx2: second antenna
 */
static inline void dw3000_calib_ant_set_id_to_ant(int ant_set_id, s8 *ant_idx1,
						  s8 *ant_idx2)
{
	static const s8 set_id_to_ant[ANTSET_ID_MAX][2] = {
		{ 0, 1 }, { 0, 2 },  { 0, 3 },	{ 1, 2 },  { 1, 3 },
		{ 2, 3 }, { 0, -1 }, { 1, -1 }, { 2, -1 }, { 3, -1 }
	};

	*ant_idx1 = set_id_to_ant[ant_set_id][0];
	*ant_idx2 = set_id_to_ant[ant_set_id][1];
}

/**
 * struct dw3000_calibration_data - all per-antenna and per-channel calibration
 * parameters
 * @ant: table of antenna dependent calibration values
 * @antpair: table of antenna pair dependent calibration values
 * @ch: table of channel dependent calibration values
 */
struct dw3000_calibration_data {
	struct dw3000_antenna_calib ant[ANTMAX];
	struct dw3000_antenna_pair_calib antpair[ANTPAIR_MAX];
	struct dw3000_channel_calib ch[DW3000_CALIBRATION_CHANNEL_MAX];
};

struct dw3000;

/**
 * dw3000_calib_parse_key - parse key and find corresponding param
 * @dw: the DW device
 * @key: pointer to NUL terminated string to retrieve param address and len
 * @param: pointer where to store the corresponding parameter address
 *
 * This function lookup the NULL terminated table @dw3000_calib_keys and
 * if specified key is found, store the corresponding address in @param and
 *
 * Return: length of corresponding parameter if found, else a -ENOENT error.
 */
int dw3000_calib_parse_key(struct dw3000 *dw, const char *key, void **param);

const char *const *dw3000_calib_list_keys(struct dw3000 *dw);

int dw3000_calib_update_config(struct dw3000 *dw);

#endif /* __DW3000_CALIB_H */
