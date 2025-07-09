#ifndef __TAS25XX_LOGIC_
#define __TAS25XX_LOGIC_

#include "tas25xx.h"

#define TAS25XX_STREAM_PLAYBACK 0
#define TAS25XX_STREAM_CAPTURE 1

int tas25xx_register_device(struct tas25xx_priv  *p_tas25xx);
int tas25xx_probe(struct tas25xx_priv *p_tas25xx);
void tas25xx_remove(struct tas25xx_priv *p_tas25xx);
int tas25xx_load_init(struct tas25xx_priv *p_tas25xx, int chn);
int tas25xx_irq_work_func(struct tas25xx_priv *p_tas25xx);
void tas25xx_load_config(struct tas25xx_priv *p_tas25xx, int chn);
int tas25xx_init_work_func(struct tas25xx_priv *p_tas25xx,
	struct tas_device *dev_tas25xx);
int tas25xx_dc_work_func(struct tas25xx_priv *p_tas25xx, int chn);
void tas_reload(struct tas25xx_priv *p_tas25xx, int chn);
int tas25xx_set_power_state(struct tas25xx_priv *p_tas25xx,
		enum tas_power_states_t state, uint32_t chbitmask);
int tas25xx_iv_vbat_slot_config(struct tas25xx_priv *p_tas25xx,
	int mn_slot_width);
int tas25xx_set_bitwidth(struct tas25xx_priv *p_tas25xx,
	int bitwidth, int stream);
int tas25xx_set_dai_fmt_for_fmt(struct tas25xx_priv *p_tas25xx,
	unsigned int fmt);
int tas25xx_set_tdm_rx_slot(struct tas25xx_priv *p_tas25xx,
	int slots, int slot_width);
int tas25xx_set_tdm_tx_slot(struct tas25xx_priv *p_tas25xx,
	int slots, int slot_width);

int tas25xx_change_book(struct tas25xx_priv *p_tas25xx,
	int32_t chn, int book);

#if IS_ENABLED(CONFIG_TAS25XX_IRQ_BD)
void tas25xx_log_interrupt_stats(struct tas25xx_priv *p_tas25xx);
void tas25xx_clear_interrupt_stats(struct tas25xx_priv *p_tas25xx);
#endif /* CONFIG_TAS25XX_IRQ_BD */

int tas25xx_get_drv_channel_opmode(void);

#endif /*__TAS25XX_LOGIC_*/
