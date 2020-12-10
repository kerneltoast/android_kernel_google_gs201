// SPDX-License-Identifier: GPL-2.0-only
/*
 * SLC operations for BigOcean
 *
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/io.h>
#include <linux/module.h>

#include "bigo_slc.h"

#define SID_S3_TEMPORAL 3
#define SID_COMP_INFO 6
#define SID_S4_SECONDARY_COLBUF 19
#define SID_S4_COMP_TILE_COL 20
#define SID_S4_CDEC_TILE_COL 21

void bigo_bypass_ssmt_pid(struct bigo_core *core)
{
	int sid;
	unsigned int offset;
	const int cache_off = 0x400;
	const int rd_alloc_off = 0x600;
	const int wr_alloc_off = 0x800;

	if (!core->slc.ssmt_pid_base)
		return;

	for (sid = 0; sid < 32; sid++) {
		offset = sid * 4;
		if (sid == SID_S3_TEMPORAL || sid == SID_COMP_INFO ||
			sid == SID_S4_SECONDARY_COLBUF || sid == SID_S4_COMP_TILE_COL ||
			sid == SID_S4_CDEC_TILE_COL) {
			writel(core->slc.pid, core->slc.ssmt_pid_base + offset);
		}
		writel(0xe, core->slc.ssmt_pid_base + cache_off + offset);
		writel(0x80000000, core->slc.ssmt_pid_base + rd_alloc_off + offset);
		writel(0x80000000, core->slc.ssmt_pid_base + wr_alloc_off + offset);
	}
}

int bigo_pt_client_enable(struct bigo_core *core)
{
	int rc = 0;

	if (!core->slc.pt_hnd)
		return 0;

	core->slc.pid = pt_client_enable_size(core->slc.pt_hnd, 0, &core->slc.size);
	if (core->slc.pid < 0) {
		pr_warn("Failed to get BO pid\n");
		rc = core->slc.pid;
	}
	return rc;
}

void bigo_pt_client_disable(struct bigo_core *core)
{
	if (core->slc.pt_hnd)
		pt_client_disable(core->slc.pt_hnd, 0);
}

void bigo_get_cache_info(struct bigo_core *core, struct bigo_cache_info *cinfo)
{
	cinfo->size = core->slc.size;
	cinfo->pid = core->slc.pid;
}

static void bigo_pt_resize_cb(void *data, int id, size_t size_allocated)
{
	struct bigo_core *core = (struct bigo_core *)data;

	pr_debug("Received SLC resize callback, id: %d, size: %zu\n", id, size_allocated);
	core->slc.size = size_allocated;
}

void bigo_pt_client_register(struct device_node *node, struct bigo_core *core)
{
	core->slc.pt_hnd = pt_client_register(node, (void *)core, bigo_pt_resize_cb);
	if (IS_ERR(core->slc.pt_hnd)) {
		core->slc.pt_hnd = NULL;
		pr_warn("Failed to register pt_client.\n");
	}
}

void bigo_pt_client_unregister(struct bigo_core *core)
{
	pt_client_unregister(core->slc.pt_hnd);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vinay Kalia <vinaykalia@google.com>");
