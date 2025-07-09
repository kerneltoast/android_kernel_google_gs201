// SPDX-License-Identifier: GPL-2.0-only
/*
 * Parses BigOcean device tree node
 *
 * Copyright 2020 Google LLC.
 *
 * Author: Vinay Kalia <vinaykalia@google.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <soc/google/bts.h>

#include "bigo_of.h"

static int bigo_of_get_resource(struct bigo_core *core)
{
	struct platform_device *pdev = to_platform_device(core->dev);
	struct resource *res;
	int rc = 0;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	if (IS_ERR_OR_NULL(res)) {
		rc = PTR_ERR(res);
		pr_err("Failed to find bw register base: %d\n", rc);
		goto err;
	}
	core->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(core->base)) {
		rc = PTR_ERR(core->base);
		if (rc == 0)
			rc = -EIO;
		pr_err("Failed to map bw register base: %d\n", rc);
		core->base = NULL;
		goto err;
	}
	core->regs_size = res->end - res->start + 1;
	core->paddr = (phys_addr_t)res->start;

#if IS_ENABLED(ENABLE_SLC)
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ssmt_pid");
	if (IS_ERR_OR_NULL(res)) {
		rc = PTR_ERR(res);
		pr_err("Failed to find ssmt register base: %d\n", rc);
		goto err;
	}
	core->slc.ssmt_pid_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(core->slc.ssmt_pid_base)) {
		pr_warn("Failed to map ssmt register base: %ld\n",
			PTR_ERR(core->slc.ssmt_pid_base));
		core->slc.ssmt_pid_base = NULL;
	}
#endif
	core->irq = platform_get_irq(pdev, 0);

	if (core->irq < 0) {
		rc = core->irq;
		pr_err("platform_get_irq failed: %d\n", rc);
		goto err;
	}

	rc = of_property_read_u32(core->dev->of_node, "ip_ver", &core->ip_ver);
	if (rc < 0) {
		core->ip_ver = 1;
		pr_debug("ip_ver is not specified\n");
		rc = 0;
	}
err:
	return rc;
}

static void bigo_of_remove_opp_table(struct bigo_core *core)
{
	struct bigo_opp *opp, *tmp;

	list_for_each_entry_safe(opp, tmp, &core->pm.opps, list) {
		list_del(&opp->list);
		kfree(opp);
	}
}

static void bigo_of_remove_bw_table(struct bigo_core *core)
{
	struct bigo_bw *bw, *tmp;

	list_for_each_entry_safe(bw, tmp, &core->pm.bw, list) {
		list_del(&bw->list);
		kfree(bw);
	}
}

static int bigo_of_parse_opp_table(struct bigo_core *core)
{
	int rc = 0;
	struct device_node *np;
	struct bigo_opp *opp;

	struct device_node *opp_np =
		of_parse_phandle(core->dev->of_node, "vpu-opp-table", 0);
	if (!opp_np) {
		return -ENOENT;
		goto err_add_table;
	}
	for_each_available_child_of_node(opp_np, np) {
		opp = kmalloc(sizeof(*opp), GFP_KERNEL);
		if (!opp) {
			rc = -ENOMEM;
			goto err_entry;
		}
		rc = of_property_read_u32(np, "load-pps", &opp->load_pps);
		if (rc < 0) {
			kfree(opp);
			goto err_entry;
		}
		core->pm.max_load = opp->load_pps;
		rc = of_property_read_u32(np, "freq-khz", &opp->freq_khz);
		if (rc < 0) {
			kfree(opp);
			goto err_entry;
		}

		list_add_tail(&opp->list, &core->pm.opps);
	}
	return rc;
err_entry:
	bigo_of_remove_opp_table(core);
err_add_table:
	return rc;
}

static int bigo_of_parse_bw_table(struct bigo_core *core)
{
	int rc = 0;
	struct device_node *np;
	struct bigo_bw *bw;

	struct device_node *bw_np =
		of_parse_phandle(core->dev->of_node, "vpu-bw-table", 0);
	if (!bw_np) {
		return -ENOENT;
		goto err_add_table;
	}
	for_each_available_child_of_node(bw_np, np) {
		bw = kmalloc(sizeof(*bw), GFP_KERNEL);
		if (!bw) {
			rc = -ENOMEM;
			goto err_entry;
		}
		rc = of_property_read_u32(np, "load-pps", &bw->load_pps);
		if (rc < 0) {
			kfree(bw);
			goto err_entry;
		}
		rc = of_property_read_u32(np, "rd-bw", &bw->rd_bw);
		if (rc < 0) {
			kfree(bw);
			goto err_entry;
		}
		rc = of_property_read_u32(np, "wr-bw", &bw->wr_bw);
		if (rc < 0) {
			kfree(bw);
			goto err_entry;
		}
		rc = of_property_read_u32(np, "pk-bw", &bw->pk_bw);
		if (rc < 0) {
			kfree(bw);
			goto err_entry;
		}
		/* *-bw-afbc entry is optional */
		if (of_property_read_u32(np, "rd-bw-afbc", &bw->rd_bw_afbc) < 0)
			bw->rd_bw_afbc = bw->rd_bw;

		if (of_property_read_u32(np, "wr-bw-afbc", &bw->wr_bw_afbc) < 0)
			bw->wr_bw_afbc = bw->wr_bw;

		if (of_property_read_u32(np, "pk-bw-afbc", &bw->pk_bw_afbc) < 0)
			bw->pk_bw_afbc = bw->pk_bw;

		/* *-bw-enc entry is optional */
		if (of_property_read_u32(np, "rd-bw-enc", &bw->rd_bw_enc) < 0)
			bw->rd_bw_enc = bw->rd_bw;

		if (of_property_read_u32(np, "wr-bw-enc", &bw->wr_bw_enc) < 0)
			bw->wr_bw_enc = bw->wr_bw;

		if (of_property_read_u32(np, "pk-bw-enc", &bw->pk_bw_enc) < 0)
			bw->pk_bw_enc = bw->pk_bw;

		list_add_tail(&bw->list, &core->pm.bw);
	}
	return rc;
err_entry:
	bigo_of_remove_bw_table(core);
err_add_table:
	return rc;
}

int bigo_of_dt_parse(struct bigo_core *core)
{
	int rc = 0;

	rc = bigo_of_get_resource(core);
	if (rc < 0) {
		pr_err("failed to get respource: %d\n", rc);
		goto err_get_res;
	}

	rc = bigo_of_parse_opp_table(core);
	if (rc < 0) {
		pr_err("failed to parse bigocean OPP table\n");
		goto err_parse_opp_table;
	}

	rc = bigo_of_parse_bw_table(core);
	if (rc < 0) {
		pr_err("failed to parse bigocean bandwidth table\n");
		goto err_parse_bw_table;
	}

	core->pm.bwindex = bts_get_bwindex("bo");
	if (core->pm.bwindex < 0) {
		rc = core->pm.bwindex;
		goto err_bwindex;
	}
	return rc;
err_bwindex:
	bigo_of_remove_bw_table(core);
err_parse_bw_table:
	bigo_of_remove_opp_table(core);
err_parse_opp_table:
err_get_res:
	return rc;
}

void bigo_of_dt_release(struct bigo_core *core)
{
	if (!core)
		return;
	bigo_of_remove_opp_table(core);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vinay Kalia <vinaykalia@google.com>");
