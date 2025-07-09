#ifndef __TAS256X_LOGIC_
#define __TAS256X_LOGIC_

#include "physical_layer/inc/tas256x.h"

#define TAS256X_STREAM_PLAYBACK 0
#define TAS256X_STREAM_CAPTURE 1

int tas256x_register_device(struct tas256x_priv  *p_tas256x);
int tas256x_probe(struct tas256x_priv *p_tas256x);
void tas256x_remove(struct tas256x_priv *p_tas256x);
int tas256x_load_init(struct tas256x_priv *p_tas256x);
int tas256x_irq_work_func(struct tas256x_priv *p_tas256x);
void tas256x_load_config(struct tas256x_priv *p_tas256x);
int tas256x_init_work_func(struct tas256x_priv *p_tas256x);
int tas256x_dc_work_func(struct tas256x_priv *p_tas256x, int ch);
void tas_reload(struct tas256x_priv *p_tas256x, int chn);
int tas256x_set_power_state(struct tas256x_priv *p_tas256x,
			int state);
int tas256x_iv_vbat_slot_config(struct tas256x_priv *p_tas256x,
	int mn_slot_width);
int tas256x_set_bitwidth(struct tas256x_priv *p_tas256x,
	int bitwidth, int stream);
int tas256x_set_tdm_rx_slot(struct tas256x_priv *p_tas256x,
	int slots, int slot_width);
int tas256x_set_tdm_tx_slot(struct tas256x_priv *p_tas256x,
	int slots, int slot_width);

#endif /*__TAS256X_LOGIC_*/