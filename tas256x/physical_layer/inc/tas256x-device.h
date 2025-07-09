#ifndef __TAS256X_DEVICE_
#define __TAS256X_DEVICE_

#include "tas256x.h"
#define DEVICE_TAS2558	0x2558
#define DEVICE_TAS2562	0x2562
#define DEVICE_TAS2564	0x2564

#define DVC_PCM				0x1
#define LIM_MAX_ATN			0x2
#define LIMB_TH_MAX			0x3
#define LIMB_TH_MIN			0x4
#define LIMB_INF_PT			0x5
#define LIMB_SLOPE			0x6
#define BOP_TH				0x7
#define BOSD_TH				0x8
#define LIMB_ATK_RT			0x9
#define	LIMB_RLS_RT			0xA
#define LIMB_ATK_ST			0xB
#define LIMB_RLS_ST			0xC
#define BOP_ATK_RT			0xD
#define BOP_ATK_ST			0xE
#define BOP_HLD_TM			0xF
#define BST_VREG			0x10
#define BST_ILIM			0x11
#define CLASSH_TIMER			0x12
#define AMPOUTPUT_LVL			0x13
#define ICN_THR				0x14
#define ICN_HYST			0x15

#define TX_SLOT0	0
#define TX_SLOT1	1
#define TX_SLOT2	2
#define TX_SLOT3	3
#define TX_SLOT4	4
#define TX_SLOT5	5
#define TX_SLOT6	6
#define TX_SLOT7	7
#define TX_SLOT8	8
#define TX_SLOT9	9
#define TX_SLOTa	0xa
#define TX_SLOTb	0xb
#define TX_SLOTc	0xc
#define TX_SLOTd	0xd
#define TX_SLOTe	0xe
#define TX_SLOTf	0xf

int tas56x_software_reset(struct tas256x_priv *p_tas256x, int ch);

int tas56x_get_chipid(struct tas256x_priv *p_tas256x, int *chipid, int ch);

/* Power Up related functions */
int tas256x_set_power_up(struct tas256x_priv *p_tas256x,
	enum channel chn);

int tas256x_set_power_mute(struct tas256x_priv *p_tas256x,
	enum channel chn);

int tas256x_set_power_shutdown(struct tas256x_priv *p_tas256x,
	enum channel chn);

int tas256x_power_check(struct tas256x_priv *p_tas256x, int *state,
	int ch);

/* IV Sense Format Related functions */
int tas256x_iv_sense_enable_set(struct tas256x_priv *p_tas256x,
	bool enable, int ch);

bool tas256x_iv_sense_enable_get(struct tas256x_priv *p_tas256x, int ch);

int tas256x_set_iv_slot(struct tas256x_priv *p_tas256x, int ch, int vslot, int islot);

int tas256x_set_vbat_slot(struct tas256x_priv *p_tas256x, int ch, int slot);

int tas256x_iv_bitwidth_config(struct tas256x_priv *p_tas256x, int bitwidth, int ch);

int tas256x_tx_set_edge(struct tas256x_priv *p_tas256x,
	unsigned int tx_edge, int ch);

int tas256x_tx_set_start_slot(struct tas256x_priv *p_tas256x,
	unsigned int tx_start_slot, int ch);

int tas256x_set_tx_config(struct tas256x_priv *p_tas256x, int value, int ch);

/* Rx Format Related functions */
int tas256x_set_samplerate(struct tas256x_priv *p_tas256x,
			int samplerate, int ch);

int tas256x_rx_set_edge(struct tas256x_priv *p_tas256x,
	unsigned int rx_edge, int ch);

int tas256x_rx_set_start_slot(struct tas256x_priv *p_tas256x,
	unsigned int rx_start_slot, int ch);

int tas256x_rx_set_slot_len(struct tas256x_priv *p_tas256x,
	int slot_width, int ch);

int tas256x_rx_set_slot(struct tas256x_priv *p_tas256x,
	int slot, int ch);

int tas256x_rx_set_bitwidth(struct tas256x_priv *p_tas256x,
	int bitwidth, int ch);

/* Interrupt Related Functions */
int tas256x_interrupt_clear(struct tas256x_priv *p_tas256x, int ch);

int tas256x_interrupt_enable(struct tas256x_priv *p_tas256x,
	int val, int ch);

int tas256x_interrupt_read(struct tas256x_priv *p_tas256x,
	int *intr1, int *intr2, int ch);

int tas256x_interrupt_determine(struct tas256x_priv *p_tas256x, int ch,
	int int1status, int int2status);

/* ICN Related Functions */
int tas256x_icn_disable(struct tas256x_priv *p_tas256x,
	int disable, int ch);


int tas256x_icn_config(struct tas256x_priv *p_tas256x, int value, int ch);

/* Boost/Volt Related functions */
int tas256x_boost_volt_update(struct tas256x_priv *p_tas256x,
	int value, int ch);

int tas256x_set_misc_config(struct tas256x_priv *p_tas256x, int value, int ch);

/* Clock Configuration */
int tas256x_set_clock_config(struct tas256x_priv *p_tas256x,
	int value, int ch);

/* ClassH Configuration*/
int tas256x_set_classH_config(struct tas256x_priv *p_tas256x,
	int value, int ch);

/* HPF Related functions*/
int tas256x_HPF_FF_Bypass(struct tas256x_priv *p_tas256x, int value, int ch);

int tas256x_HPF_FB_Bypass(struct tas256x_priv *p_tas256x, int value, int ch);

/*Speaker Receiever mode - specific to TAS2564*/
int tas2564_rx_mode_update(struct tas256x_priv *p_tas256x,
	int rx_mode, int ch);

/*Playback Volume*/
int tas256x_update_playback_volume(struct tas256x_priv *p_tas256x,
	int value, int ch);

/*Feature Control Functions*/
int tas256x_update_lim_max_attenuation(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_lim_max_thr(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_lim_min_thr(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_lim_inflection_point(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_lim_slope(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_bop_thr(struct tas256x_priv *p_tas256x, int value, int ch);
int tas256x_update_bosd_thr(struct tas256x_priv *p_tas256x, int value, int ch);
int tas256x_update_boost_voltage(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_current_limit(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_limiter_enable(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_limiter_attack_rate(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_limiter_attack_step_size(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_limiter_release_rate(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_limiter_release_step_size(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_bop_enable(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_bop_mute(struct tas256x_priv *p_tas256x, int value, int ch);
int tas256x_update_bop_shutdown_enable(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_bop_attack_rate(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_bop_attack_step_size(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_bop_hold_time(struct tas256x_priv *p_tas256x,
	int value, int ch);
int tas256x_update_vbat_lpf(struct tas256x_priv *p_tas256x, int value, int ch);
int tas256x_update_rx_cfg(struct tas256x_priv *p_tas256x, int value, int ch);
int tas256x_rx_set_frame_start(struct tas256x_priv *p_tas256x,
	unsigned int frame_start, int ch);
int tas256x_update_classh_timer(struct tas256x_priv *p_tas256x, int value, int ch);
int tas256x_set_auto_detect_clock(struct tas256x_priv *p_tas256x,
			int value, int ch);
int tas256x_enable_reciever_mode(struct tas256x_priv *p_tas256x, int enable,
	int ch);
int	tas256x_update_ampoutput_level(struct tas256x_priv *p_tas256x,
	int value, int ch);

/*Initialize to defaults*/
int tas256x_update_default_params(struct tas256x_priv *p_tas256x, int ch);

/* ICN Related*/
int tas256x_update_icn_threshold(struct tas256x_priv *p_tas256x, int value,
	int ch);

int tas256x_update_icn_hysterisis(struct tas256x_priv *p_tas256x, int value,
	int sample_rate, int ch);

/* Noise Gate Enable/Disable */
int tas256x_enable_noise_gate(struct tas256x_priv *p_tas256x, int enable,
	int ch);

#endif /* __TAS256X_DEVICE_ */
