#ifndef __TAS25XX_REGMAP__
#define __TAS25XX_REGMAP__
#include <linux/version.h>
#include "tas25xx.h"

struct linux_platform {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap[MAX_CHANNELS];
	struct hrtimer mtimer;
	struct snd_soc_component *codec;
	/* device is working, but system is suspended */
	int (*runtime_suspend)(struct tas25xx_priv *p_tas25xx);
	int (*runtime_resume)(struct tas25xx_priv *p_tas25xx);
	bool mb_runtime_suspend;
	bool i2c_suspend;
};

void tas25xx_select_cfg_blk(void *pContext, int conf_no,
	unsigned char block_type);
void tas25xx_dump_regs(struct tas25xx_priv  *p_tas25xx, int chn);


void tas25xx_register_i2c_error_callback(void (*i2c_err_cb)(uint32_t));
#endif /*__TAS25XX_REGMAP__*/
