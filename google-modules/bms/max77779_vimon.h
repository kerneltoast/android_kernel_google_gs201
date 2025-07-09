/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2023 Google LLC
 */

#ifndef MAX77779_VIMON_H_
#define MAX77779_VIMON_H_

#include <linux/regmap.h>

#define MAX77779_VIMON_SIZE 0xFF
#define MAX77779_VIMON_DEFAULT_MAX_CNT 256
#define MAX77779_VIMON_DEFAULT_MAX_TRIGGERS 1

#define MAX77779_VIMON_BUFFER_SIZE 0x80
#define MAX77779_VIMON_OFFSET_BASE 0x80
#define MAX77779_VIMON_PAGE_CNT 4
#define MAX77779_VIMON_PAGE_SIZE 0x80
#define MAX77779_VIMON_LAST_PAGE_SIZE 0x70
#define MAX77779_VIMON_BYTES_PER_ENTRY 2
#define MAX77779_VIMON_ENTRIES_PER_VI_PAIR 2

#define MAX77779_VIMON_SMPL_CNT 64
#define MAX77779_VIMON_DATA_RETRIEVE_DELAY 0

/*
 * TODO: b/376771907: needs to be removed
 * - MAX77779_VIMON_DATA_RETRY_DELAY_MS
 * - MAX77779_VIMON_NV_PER_LSB
 * - MAX77779_VIMON_NA_PER_LSB
 * - MILLI_UNITS_TO_NANO_UNITS
 */
#define MAX77779_VIMON_DATA_RETRY_DELAY_MS 100

#define MAX77779_VIMON_NV_PER_LSB 78122
#define MAX77779_VIMON_NA_PER_LSB 781250
#define MILLI_UNITS_TO_NANO_UNITS 1000000
#define VIMON_CLIENT_ALWAYS_RUN -1

enum max77779_vimon_state {
	MAX77779_VIMON_ERROR = -1,
	MAX77779_VIMON_DISABLED = 0,
	MAX77779_VIMON_IDLE,
	MAX77779_VIMON_RUNNING,
	MAX77779_VIMON_DATA_AVAILABLE,
};

enum vimon_trigger_source {
	VIMON_IMMEDIATE_TRIGGER            = BIT(0),
	VIMON_BATOILO1_TRIGGER             = BIT(1),
	VIMON_BATOILO2_TRIGGER             = BIT(2),
	VIMON_SYSUVLO1_TRIGGER             = BIT(3),
	VIMON_SYSUVLO2_TRIGGER             = BIT(4),
	VIMON_VBATT_MEASUREMENT_TRIGGER    = BIT(5),
	VIMON_IBATT_MEASUREMENT_TRIGGER    = BIT(6),
	VIMON_VBATT_SAMPLE_TRIGGER_AVERAGE = BIT(7),
	VIMON_VBATT_SAMPLE_TRIGGER_MINIMUM = BIT(8),
	VIMON_VBATT_SAMPLE_TRIGGER_MAXIMUM = BIT(9),
	VIMON_IBATT_SAMPLE_TRIGGER_AVERAGE = BIT(10),
	VIMON_IBATT_SAMPLE_TRIGGER_MINIMUM = BIT(11),
	VIMON_IBATT_SAMPLE_TRIGGER_MAXIMUM = BIT(12),
	VIMON_CLIENT_REQUEST               = BIT(15), /* if extra_trigger returns true */
};

struct vimon_client_callbacks {
	/* on_sample_ready: required */
	void (*on_sample_ready)(void *private, const enum vimon_trigger_source reason,
				const u16 *data, const size_t len);

	/* on_unregistered: required */
	void (*on_removed)(void *private);

	/* extra_trigger: optional */
	bool (*extra_trigger)(void *private, const u16 *data, const size_t len);
};

int vimon_register_callback(struct device *dev, const u16 mask, const int count, void *private,
			    struct vimon_client_callbacks *cb);
void vimon_unregister_callback(struct device *dev, struct vimon_client_callbacks *cb);

struct max77779_vimon_data {
	struct device *dev;
	int irq;
	struct regmap *regmap;
	struct dentry *de;

	struct notifier_block	reboot_notifier;
	bool run_in_offmode;

	struct mutex vimon_lock;
	struct mutex vimon_cb_lock;
	unsigned max_cnt;
	unsigned max_triggers;
	enum max77779_vimon_state state;
	uint16_t *buf;
	size_t buf_size;
	size_t buf_len;

	/* debug interface, register to read or write */
	u32 debug_reg_address;
	u8 debug_buffer_page;

	struct delayed_work read_data_work;

	int (*direct_reg_read)(struct max77779_vimon_data *data, u8 reg, unsigned int *val);
	int (*direct_reg_write)(struct max77779_vimon_data *data, u8 reg, unsigned int val);
	u16 trigger_src;
};

int max77779_vimon_init(struct max77779_vimon_data *data);
void max77779_vimon_remove(struct max77779_vimon_data *data);
bool max77779_vimon_is_reg(struct device *dev, unsigned int reg);
#endif
