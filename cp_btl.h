/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019, Samsung Electronics.
 *
 */

#ifndef __CP_BTL_H__
#define __CP_BTL_H__

enum cp_btl_id {
	BTL_ID_0,
	BTL_ID_1,
	MAX_BTL_ID,
};

struct cp_btl_mem_region {
	unsigned long p_base;
	void __iomem *v_base;
	unsigned long cp_p_base;
	u32 size;
};

struct cp_btl {
	char *name;
	u32 id;

	bool enabled;
	atomic_t active;

	u32 link_type;
	struct cp_btl_mem_region mem;
#if IS_ENABLED(CONFIG_LINK_DEVICE_PCIE)
	int last_pcie_atu_grp;
#endif

	struct mem_link_device *mld;
	struct miscdevice miscdev;

	bool support_extension;
	u32 extension_dram_size;
	bool extension_enabled;
};

#if IS_ENABLED(CONFIG_CP_BTL)
extern int cp_btl_create(struct cp_btl *btl, struct device *dev);
extern int cp_btl_destroy(struct cp_btl *btl);
#else
static inline int cp_btl_create(struct cp_btl *btl, struct device *dev) { return 0; }
static inline int cp_btl_destroy(struct cp_btl *btl) { return 0; }
#endif

#endif /* __CP_BTL_H__ */
