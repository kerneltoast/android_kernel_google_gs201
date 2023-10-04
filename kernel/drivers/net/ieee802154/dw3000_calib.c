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
#include "dw3000.h"
#include "dw3000_txpower_adjustment.h"

/* clang-format off */
#define CHAN_PRF_PARAMS (4 * DW3000_CALIBRATION_PRF_MAX)
#define ANT_CHAN_PARAMS (CHAN_PRF_PARAMS * DW3000_CALIBRATION_CHANNEL_MAX)
#define ANT_OTHER_PARAMS (4) /* port, selector_gpio... */
#define ANTPAIR_CHAN_PARAMS (2 * DW3000_CALIBRATION_CHANNEL_MAX + 1)

#define OTHER_PARAMS (13) /* xtal_trim, temperature_reference,
			    smart_tx_power, spi_pid, dw3000_pid,
			    auto_sleep_margin, restricted_channels
			    phrMode, alternate_pulse_shape,
			    wificoex_gpio, wificoex_delay_us,
			    wificoex_interval_us, wificoex_margin_us */

#define MAX_CALIB_KEYS ((ANTMAX * (ANT_CHAN_PARAMS + ANT_OTHER_PARAMS)) + \
			(ANTPAIR_MAX * ANTPAIR_CHAN_PARAMS) +		\
			(DW3000_CALIBRATION_CHANNEL_MAX * 2) +		\
			OTHER_PARAMS)

#define DW_OFFSET(m) offsetof(struct dw3000, m)
#define DW_SIZE(m) sizeof_field(struct dw3000, m)
#define DW_INFO(m) { .offset = DW_OFFSET(m), .length = DW_SIZE(m) }

#define CAL_INFO(m) DW_INFO(calib_data.m)
#define OTP_INFO(m) DW_INFO(otp_data.m)

#define PRF_CAL_INFO(b,x)			\
	CAL_INFO(b.prf[x].ant_delay),		\
	CAL_INFO(b.prf[x].tx_power),		\
	CAL_INFO(b.prf[x].pg_count),		\
	CAL_INFO(b.prf[x].pg_delay)

#define ANTENNA_CAL_INFO(x)			\
	PRF_CAL_INFO(ant[x].ch[0], 0),		\
	PRF_CAL_INFO(ant[x].ch[0], 1),		\
	PRF_CAL_INFO(ant[x].ch[1], 0),		\
	PRF_CAL_INFO(ant[x].ch[1], 1),		\
	CAL_INFO(ant[x].port),			\
	CAL_INFO(ant[x].selector_gpio),		\
	CAL_INFO(ant[x].selector_gpio_value),	\
	CAL_INFO(ant[x].caps)

#define ANTPAIR_CAL_INFO(x,y)					\
	CAL_INFO(antpair[ANTPAIR_IDX(x, y)].ch[0].pdoa_offset),	\
	CAL_INFO(antpair[ANTPAIR_IDX(x, y)].ch[0].pdoa_lut),	\
	CAL_INFO(antpair[ANTPAIR_IDX(x, y)].ch[1].pdoa_offset),	\
	CAL_INFO(antpair[ANTPAIR_IDX(x, y)].ch[1].pdoa_lut)

static const struct {
	unsigned int offset;
	unsigned int length;
} dw3000_calib_keys_info[MAX_CALIB_KEYS] = {
	/* ant0.* */
	ANTENNA_CAL_INFO(0),
	/* ant1.* */
	ANTENNA_CAL_INFO(1),
	/* ant0.* */
	ANTENNA_CAL_INFO(2),
	/* ant1.* */
	ANTENNA_CAL_INFO(3),
	/* antX.antW.* */
	ANTPAIR_CAL_INFO(0,1),
	ANTPAIR_CAL_INFO(0,2),
	ANTPAIR_CAL_INFO(0,3),
	ANTPAIR_CAL_INFO(1,2),
	ANTPAIR_CAL_INFO(1,3),
	ANTPAIR_CAL_INFO(2,3),
	/* chY.* */
	CAL_INFO(ch[0].pll_locking_code),
	CAL_INFO(ch[0].wifi_coex_enabled),
	CAL_INFO(ch[1].pll_locking_code),
	CAL_INFO(ch[1].wifi_coex_enabled),
	/* other with direct access in struct dw3000 */
	DW_INFO(txconfig.smart),
	DW_INFO(auto_sleep_margin_us),
	DW_INFO(spi_pid),
	DW_INFO(dw3000_pid),
	DW_INFO(restricted_channels),
	DW_INFO(config.phrMode),
	DW_INFO(coex_gpio),
	DW_INFO(coex_delay_us),
	DW_INFO(coex_margin_us),
	DW_INFO(coex_interval_us),
	/* country */
	DW_INFO(config.alternate_pulse_shape),
	/* other with defaults from OTP */
	OTP_INFO(xtal_trim),
	OTP_INFO(tempP),
};

#define PRF_CAL_LABEL(a,c,p)				\
	"ant" #a ".ch" #c ".prf" #p ".ant_delay",	\
	"ant" #a ".ch" #c ".prf" #p ".tx_power",	\
	"ant" #a ".ch" #c ".prf" #p ".pg_count",	\
	"ant" #a ".ch" #c ".prf" #p ".pg_delay"

#define ANTENNA_CAL_LABEL(x)			\
	PRF_CAL_LABEL(x, 5, 16),		\
	PRF_CAL_LABEL(x, 5, 64),		\
	PRF_CAL_LABEL(x, 9, 16),		\
	PRF_CAL_LABEL(x, 9, 64),		\
	"ant" #x ".port",			\
	"ant" #x ".selector_gpio",		\
	"ant" #x ".selector_gpio_value",	\
	"ant" #x ".caps"

#define PDOA_CAL_LABEL(a, b, c)				\
	"ant" #a ".ant" #b ".ch" #c ".pdoa_offset",	\
	"ant" #a ".ant" #b ".ch" #c ".pdoa_lut"

#define ANTPAIR_CAL_LABEL(x,y)			\
	PDOA_CAL_LABEL(x, y, 5),		\
	PDOA_CAL_LABEL(x, y, 9)

/*
 * calibration parameters keys table
 */
static const char *const dw3000_calib_keys[MAX_CALIB_KEYS + 1] = {
	/* antX */
	ANTENNA_CAL_LABEL(0),
	ANTENNA_CAL_LABEL(1),
	ANTENNA_CAL_LABEL(2),
	ANTENNA_CAL_LABEL(3),
	/* antX.antY.* */
	ANTPAIR_CAL_LABEL(0,1),
	ANTPAIR_CAL_LABEL(0,2),
	ANTPAIR_CAL_LABEL(0,3),
	ANTPAIR_CAL_LABEL(1,2),
	ANTPAIR_CAL_LABEL(1,3),
	ANTPAIR_CAL_LABEL(2,3),
	/* chY.* */
	"ch5.pll_locking_code",
	"ch5.wifi_coex-enabled",
	"ch9.pll_locking_code",
	"ch9.wifi_coex-enabled",
	/* other */
	"smart_tx_power",
	"auto_sleep_margin",
	"spi_pid",
	"dw3000_pid",
	"restricted_channels",
	"phr_mode",
	"coex_gpio",
	"coex_delay_us",
	"coex_margin_us",
	"coex_interval_us",
	/* country */
	"alternate_pulse_shape",
	/* other (OTP) */
	"xtal_trim",
	"temperature_reference",
	/* NULL terminated array for caller of dw3000_calib_list_keys(). */
	NULL
};
/* clang-format on */

const dw3000_pdoa_lut_t dw3000_default_lut_ch5 = {
	/* clang-format off */
	{ 0xe6de, 0xf36f },
	{ 0xe88b, 0xf36f },
	{ 0xea38, 0xf5b0 },
	{ 0xebe5, 0xf747 },
	{ 0xed92, 0xf869 },
	{ 0xef3f, 0xf959 },
	{ 0xf0ec, 0xfa2e },
	{ 0xf299, 0xfaf1 },
	{ 0xf445, 0xfba7 },
	{ 0xf5f2, 0xfc53 },
	{ 0xf79f, 0xfcf9 },
	{ 0xf94c, 0xfd9a },
	{ 0xfaf9, 0xfe36 },
	{ 0xfca6, 0xfed0 },
	{ 0xfe53, 0xff69 },
	{ 0x0000, 0x0000 },
	{ 0x01ad, 0x0097 },
	{ 0x035a, 0x0130 },
	{ 0x0507, 0x01ca },
	{ 0x06b4, 0x0266 },
	{ 0x0861, 0x0307 },
	{ 0x0a0e, 0x03ad },
	{ 0x0bbb, 0x0459 },
	{ 0x0d67, 0x050f },
	{ 0x0f14, 0x05d2 },
	{ 0x10c1, 0x06a7 },
	{ 0x126e, 0x0797 },
	{ 0x141b, 0x08b9 },
	{ 0x15c8, 0x0a50 },
	{ 0x1775, 0x0c91 },
	{ 0x1922, 0x0c91 }
	/* clang-format on */
};

const dw3000_pdoa_lut_t dw3000_default_lut_ch9 = {
	/* clang-format off */
	{ 0xe6de, 0xf701 },
	{ 0xe88b, 0xf7ff },
	{ 0xea38, 0xf8d2 },
	{ 0xebe5, 0xf98d },
	{ 0xed92, 0xfa38 },
	{ 0xef3f, 0xfad7 },
	{ 0xf0ec, 0xfb6d },
	{ 0xf299, 0xfbfc },
	{ 0xf445, 0xfc86 },
	{ 0xf5f2, 0xfd0c },
	{ 0xf79f, 0xfd8f },
	{ 0xf94c, 0xfe0f },
	{ 0xfaf9, 0xfe8d },
	{ 0xfca6, 0xff09 },
	{ 0xfe53, 0xff85 },
	{ 0x0000, 0x0000 },
	{ 0x01ad, 0x007b },
	{ 0x035a, 0x00f7 },
	{ 0x0507, 0x0173 },
	{ 0x06b4, 0x01f1 },
	{ 0x0861, 0x0271 },
	{ 0x0a0e, 0x02f4 },
	{ 0x0bbb, 0x037a },
	{ 0x0d67, 0x0404 },
	{ 0x0f14, 0x0493 },
	{ 0x10c1, 0x0529 },
	{ 0x126e, 0x05c8 },
	{ 0x141b, 0x0673 },
	{ 0x15c8, 0x072e },
	{ 0x1775, 0x0801 },
	{ 0x1922, 0x08ff }
	/* clang-format on */
};

int dw3000_calib_parse_key(struct dw3000 *dw, const char *key, void **param)
{
	int i;

	for (i = 0; dw3000_calib_keys[i]; i++) {
		const char *k = dw3000_calib_keys[i];

		if (strcmp(k, key) == 0) {
			/* Key found, calculate parameter address */
			*param = (void *)dw + dw3000_calib_keys_info[i].offset;
			return dw3000_calib_keys_info[i].length;
		}
	}
	return -ENOENT;
}

/**
 * dw3000_calib_list_keys - return the @dw3000_calib_keys known key table
 * @dw: the DW device
 *
 * Return: pointer to known keys table.
 */
const char *const *dw3000_calib_list_keys(struct dw3000 *dw)
{
	return dw3000_calib_keys;
}

/**
 * dw3000_calib_update_config - update running configuration
 * @dw: the DW device
 *
 * This function update the required fields in struct dw3000_txconfig according
 * the channel and PRF and the corresponding calibration values.
 *
 * Also update RX/TX RMARKER offset according calibrated antenna delay.
 *
 * Other calibration parameters aren't used yet.
 *
 * Return: zero on success, else a negative error code.
 */
int dw3000_calib_update_config(struct dw3000 *dw)
{
	struct dw3000_config *config = &dw->config;
	struct dw3000_txconfig *txconfig = &dw->txconfig;
	const struct dw3000_antenna_calib *ant_calib;
	const struct dw3000_antenna_calib_prf *ant_calib_prf;
	const struct dw3000_antenna_pair_calib *antpair_calib;
	int ant_rf1, ant_rf2, antpair;
	int chanidx, prfidx;

	dw->llhw->hw->phy->supported.channels[4] = DW3000_SUPPORTED_CHANNELS &
						   ~dw->restricted_channels;
	/* Change channel if the current one is restricted. */
	if ((1 << dw->llhw->hw->phy->current_channel) &
	    dw->restricted_channels) {
		config->chan =
			ffs(dw->llhw->hw->phy->supported.channels[4]) - 1;
		dw->llhw->hw->phy->current_channel = config->chan;
	}

	ant_rf1 = config->ant[0];
	ant_rf2 = config->ant[1];
	if (ant_rf1 < 0 && ant_rf2 < 0) {
		/* Not configured yet, does nothing. */
		return 0;
	}
	/* Since both calibs can be used at this time, both needs to be safe.  */
	if (ant_rf1 >= ANTMAX || ant_rf2 >= ANTMAX)
		return -1;
	if (ant_rf1 >= 0)
		ant_calib = &dw->calib_data.ant[ant_rf1];
	else
		ant_calib = &dw->calib_data.ant[ant_rf2];

	/* Convert config into index of array. */
	chanidx = config->chan == 9 ? DW3000_CALIBRATION_CHANNEL_9 :
				      DW3000_CALIBRATION_CHANNEL_5;
	prfidx = config->txCode >= 9 ? DW3000_CALIBRATION_PRF_64MHZ :
				       DW3000_CALIBRATION_PRF_16MHZ;

	/* Shortcut pointers to reduce line length. */
	ant_calib_prf = &ant_calib->ch[chanidx].prf[prfidx];

	/* WiFi coexistence according to current channel */
	dw->coex_enabled = dw->calib_data.ch[chanidx].wifi_coex_enabled;

	/* Update TX configuration */
	txconfig->power = ant_calib_prf->tx_power ? ant_calib_prf->tx_power :
						    0xfefefefe;
	txconfig->PGdly = ant_calib_prf->pg_delay ? ant_calib_prf->pg_delay :
						    0x34;
	txconfig->PGcount = ant_calib_prf->pg_count ? ant_calib_prf->pg_count :
						      0;
	/* Update RMARKER offsets */
	config->rmarkerOffset = ant_calib_prf->ant_delay;

	/* Early exit if RF2 isn't configured yet. */
	if (ant_rf2 < 0)
		return 0;
	if (ant_rf1 >= 0 && ant_rf2 >= 0) {
		/* RF2 and RF1 port has a valid antenna, so antpair can be used */
		antpair = ant_rf2 > ant_rf1 ? ANTPAIR_IDX(ant_rf1, ant_rf2) :
					      ANTPAIR_IDX(ant_rf2, ant_rf1);

		antpair_calib = &dw->calib_data.antpair[antpair];
		/* Update PDOA offset */
		config->pdoaOffset = antpair_calib->ch[chanidx].pdoa_offset;
		config->pdoaLut = &antpair_calib->ch[chanidx].pdoa_lut;
	}
	/* Smart TX power */
	/* When deactivated, reset register to default value (if change occurs
	   while already started) */
	if (!txconfig->smart && dw3000_is_active(dw))
		dw3000_set_tx_power_register(dw, txconfig->power);

	/* Update idle_dtu in case auto_sleep_margin_us changed */
	dw->llhw->idle_dtu = dw->auto_sleep_margin_us > 0 ?
				     US_TO_DTU(dw->auto_sleep_margin_us) :
				     DW3000_DTU_FREQ;
	return 0;
}
