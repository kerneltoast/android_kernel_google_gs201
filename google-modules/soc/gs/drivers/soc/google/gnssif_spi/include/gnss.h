/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022, Samsung Electronics.
 *
 */

#ifndef __GNSS_IF_H__
#define __GNSS_IF_H__

#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#define MAX_NAME_LEN		64

struct gnss_pdata {
	char	*name;	/* device name (ex. KEPLER) */
	char	*node_name;	/* device node (ex. gnss_ipc) */

	int	irq_gnss2ap_spi;
	int	irq_gnss2ap_wdt;

	struct gnss_io_t	*iodev;
	struct gnss_link_dev	*link_dev;

};

struct gnss_irq {
		spinlock_t lock;
		unsigned int num;
		char name[MAX_NAME_LEN];
		unsigned long flags;
		bool active;
		bool registered;
};

struct gnss_gpio {
	unsigned int num;
	const char *label;
};

#define gif_dt_read_enum(np, prop, dest) \
		do { \
				u32 val; \
				if (of_property_read_u32(np, prop, &val)) \
						return -EINVAL; \
				dest = (__typeof__(dest))(val); \
		} while (0)

#define gif_dt_read_bool(np, prop, dest) \
		do { \
				u32 val; \
				if (of_property_read_u32(np, prop, &val)) \
						return -EINVAL; \
				dest = val ? true : false; \
		} while (0)

#define gif_dt_read_string(np, prop, dest) \
		do { \
				if (of_property_read_string(np, prop, \
								(const char **)&dest)) \
				return -EINVAL; \
		} while (0)

#define gif_dt_read_u32(np, prop, dest) \
		do { \
				u32 val; \
				if (of_property_read_u32(np, prop, &val)) \
						return -EINVAL; \
								dest = val; \
		} while (0)
#define gif_dt_read_u32_array(np, prop, dest, sz) \
		do { \
				if (of_property_read_u32_array(np, prop, dest, (sz))) \
						return -EINVAL; \
		} while (0)

#define LOG_TAG "gif: "
#define CALLEE  (__func__)
#define CALLER  (__builtin_return_address(0))

#define gif_err_limited(fmt, ...) \
		 printk_ratelimited(KERN_ERR LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define gif_err(fmt, ...) \
		pr_err(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define gif_debug(fmt, ...) \
		pr_debug(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define gif_info(fmt, ...) \
		pr_info(LOG_TAG "%s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define gif_trace(fmt, ...) \
		printk(KERN_DEBUG "gif: %s: %d: called(%pF): " fmt, \
				__func__, __LINE__, __builtin_return_address(0), ##__VA_ARGS__)

int gnss_set_coredump(const char *buf, int buf_len, const char *info);

#endif /* __GNSS_IF_H__ */
