#ifndef __TAS25XX_DEVICE_
#define __TAS25XX_DEVICE_

#include "tas25xx.h"

int tas25xx_software_reset(struct tas25xx_priv *p_tas25xx, int ch);
int tas25xx_set_power_mute(struct tas25xx_priv *p_tas25xx, int ch);
int tas25xx_tx_set_edge(struct tas25xx_priv *p_tas25xx,
	unsigned int tx_edge, int ch);
int tas25xx_tx_set_start_slot(struct tas25xx_priv *p_tas25xx,
	unsigned int tx_start_slot, int ch);
int tas25xx_rx_set_edge(struct tas25xx_priv *p_tas25xx,
	unsigned int rx_edge, int ch);
int tas25xx_rx_set_bitwidth(struct tas25xx_priv *p_tas25xx,
	int bitwidth, int ch);
/* Interrupt Related Functions */
int tas_dev_interrupt_clear(struct tas25xx_priv *p_tas25xx, int ch);
int tas_dev_interrupt_enable(struct tas25xx_priv *p_tas25xx, int ch);
int tas_dev_interrupt_disable(struct tas25xx_priv *p_tas25xx, int ch);
int tas_dev_interrupt_read(struct tas25xx_priv *p_tas25xx, int ch, int *type);

#endif /* __TAS25XX_DEVICE_ */
