/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MAX77759 MAXQ interface.
 *
 * Copyright 2020 Google, LLC
 *
 */

#if IS_ENABLED(CONFIG_MAXQ_MAX77759)

struct max77759_maxq;
extern struct max77759_maxq *maxq_init(struct device *dev,
				       struct regmap *regmap,
				       bool poll);
extern void maxq_remove(struct max77759_maxq *maxq);
extern void maxq_irq(struct max77759_maxq *maxq);
/* Helpers */
extern int maxq_query_contaminant(struct max77759_maxq *maxq, u8 cc1_raw,
				  u8 cc2_raw, u8 sbu1_raw,
				  u8 sbu2_raw, u8 cc1_rd, u8 cc2_rd,
				  u8 type, u8 cc_adc_skipped,
				  u8 *response, u8 response_len);
extern int maxq_gpio_control_read(struct max77759_maxq *maxq, u8 *gpio);
extern int maxq_gpio_control_write(struct max77759_maxq *maxq, u8 gpio);
extern int maxq_gpio_trigger_read(struct max77759_maxq *maxq, u8 gpio, bool *trigger_falling);
extern int maxq_gpio_trigger_write(struct max77759_maxq *maxq, u8 gpio, bool trigger_falling);
# else
static inline int maxq_gpio_trigger_read(struct max77759_maxq *maxq, u8 gpio, bool *trigger_falling)
{
	return -EINVAL;
}

static inline int maxq_gpio_trigger_write(struct max77759_maxq *maxq, u8 gpio,
					  bool trigger_falling)
{
	return -EINVAL;
}
static inline struct max77759_maxq *maxq_init(struct device *dev,
					      struct regmap *regmap,
					      bool poll)
{
	return NULL;
}
static inline void maxq_remove(struct max77759_maxq *maxq) {}
static inline void maxq_irq(struct max77759_maxq *maxq) {}
/* Helpers */
static inline int maxq_query_contaminant(struct max77759_maxq *maxq,
					 u8 cc1_raw, u8 cc2_raw,
					 u8 sbu1_raw, u8 sbu2_raw,
					 u8 cc1_rd, u8 cc2_rd,
					 u8 type, u8 cc_adc_skipped,
					 u8 *response, u8 response_len)
{
	return -EINVAL;
}
extern int maxq_gpio_control_read(struct max77759_maxq *maxq, u8 *gpio)
{
	return -EINVAL;
}
extern int maxq_gpio_control_write(struct max77759_maxq *maxq, u8 gpio)
{
	return -EINVAL;
}
#endif
