#ifndef __TAS25XX_ALGO_SYSFS_INTF_H__
#define __TAS25XX_ALGO_SYSFS_INTF_H__

#include <linux/version.h>
#include <sound/soc.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>

int tas25xx_parse_algo_bin_sysfs(int ch_count, u8 *buf);
void tas25xx_algo_bump_oc_count(int channel, int reset);
int tas_smartamp_add_algo_controls_debugfs(struct snd_soc_component *c,
	int number_of_channels);
void tas_smartamp_remove_algo_controls_debugfs(struct snd_soc_component *c);

#endif /* __TAS25XX_ALGO_SYSFS_INTF_H__ */
