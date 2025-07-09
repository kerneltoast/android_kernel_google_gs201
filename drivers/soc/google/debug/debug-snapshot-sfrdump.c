// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/panic_notifier.h>

#include <soc/google/exynos-pd.h>
#include <soc/google/cal-if.h>
#include "debug-snapshot-local.h"

struct dbg_snapshot_sfrdump {
	const char *name;
	const char *pd_name;
	void __iomem *reg;
	unsigned int phy_reg;
	unsigned int size;
	struct list_head list;
	bool pwr_mode;
};

static LIST_HEAD(sfrdump_list);
static struct device *sfrdump_dev;

static bool dbg_snapshot_pd_is_power_up(struct dbg_snapshot_sfrdump *sfrdump)
{
	struct exynos_pm_domain *pd_domain;

	if (!sfrdump->pwr_mode)
		return true;

	pd_domain = exynos_pd_lookup_name(sfrdump->pd_name);
	if (!pd_domain) {
		dev_err(sfrdump_dev, "failed to get power domain - %s\n",
				sfrdump->pd_name);
		return false;
	}

	dev_emerg(sfrdump_dev, "%s: power %s\n", sfrdump->pd_name,
		  exynos_pd_status(pd_domain) ? "on" : "off");

	return (bool)exynos_pd_status(pd_domain);
}

static void dbg_snapshot_dump_sfr(void)
{
	struct dbg_snapshot_sfrdump *sfrdump;
	struct list_head *entry;
	unsigned int i, val, offset;
	int dump_size = 0, size;
	char buf[SZ_64];
	dma_addr_t daddr;
	void *vaddr;

	if (!sfrdump_dev) {
		dev_err(sfrdump_dev, "%s sfrdump is disabled\n", __func__);
		return;
	}

	if (list_empty(&sfrdump_list)) {
		dev_emerg(sfrdump_dev, "No sfrdump information\n");
		return;
	}

	dma_set_mask_and_coherent(sfrdump_dev, DMA_BIT_MASK(36));
	vaddr = dma_alloc_coherent(sfrdump_dev, SZ_1M, &daddr, GFP_KERNEL);
	if (!vaddr) {
		dev_err(sfrdump_dev, "failed to alloc dma\n");
		return;
	}

	list_for_each(entry, &sfrdump_list) {
		sfrdump = list_entry(entry, struct dbg_snapshot_sfrdump, list);
		if (!dbg_snapshot_pd_is_power_up(sfrdump))
			continue;

		for (i = 0; i < (sfrdump->size >> 2); i++) {
			offset = i * 4;
			val = __raw_readl(sfrdump->reg + offset);
			scnprintf(buf, SZ_64, "0x%X: 0x%0X\n",
					sfrdump->phy_reg + offset, val);
			size = strlen(buf);
			if (dump_size + size > SZ_1M) {
				dev_err(sfrdump_dev, "log is full\n");
				break;
			}

			memcpy(vaddr + dump_size, buf, size);
			dump_size += size;
		}
		dev_info(sfrdump_dev, "complete to dump %s\n", sfrdump->name);
	}
}

static int dbg_snapshot_sfrdump_panic_handler(struct notifier_block *nb,
			unsigned long l, void *buf)
{
	dbg_snapshot_dump_sfr();
	return 0;
}

static struct notifier_block nb_sfrdump = {
	.notifier_call = dbg_snapshot_sfrdump_panic_handler,
};

static int dbg_snapshot_sfrdump_probe(struct platform_device *pdev)
{
	struct device_node *parent_np, *dump_np;
	struct dbg_snapshot_sfrdump *sfrdump;
	u32 phy_regs[2];
	int ret = 0;

	sfrdump_dev = &pdev->dev;
	INIT_LIST_HEAD(&sfrdump_list);

	parent_np = of_get_child_by_name(pdev->dev.of_node, "dump-info");
	for_each_child_of_node(parent_np, dump_np) {
		sfrdump = devm_kzalloc(&pdev->dev, sizeof(struct dbg_snapshot_sfrdump),
				GFP_KERNEL);
		if (!sfrdump) {
			ret = -ENOMEM;
			break;
		}
		ret = of_property_read_u32_array(dump_np, "reg", phy_regs, 2);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to get register information(%s)\n",
					dump_np->name);
			devm_kfree(&pdev->dev, sfrdump);
			continue;
		}

		sfrdump->reg = devm_ioremap(&pdev->dev, phy_regs[0], phy_regs[1]);
		if (!sfrdump->reg) {
			dev_err(&pdev->dev, "failed to get i/o address %s node\n",
					dump_np->name);
			devm_kfree(&pdev->dev, sfrdump);
			continue;
		}

		/* check 4 bytes alignment */
		if ((phy_regs[0] & 0x3) || (phy_regs[1] & 0x3)) {
			dev_err(&pdev->dev, "(%s) Invalid alignments (4 bytes)\n",
					dump_np->name);
			devm_kfree(&pdev->dev, sfrdump);
			continue;
		}

		sfrdump->phy_reg = phy_regs[0];
		sfrdump->size = phy_regs[1];
		sfrdump->name = dump_np->name;

		ret = of_property_read_string_array(dump_np, "pd-name",
				&sfrdump->pd_name, 0);
		if (ret < 0)
			sfrdump->pwr_mode = false;
		else
			sfrdump->pwr_mode = true;

		list_add(&sfrdump->list, &sfrdump_list);
		dev_info(&pdev->dev, "success to register %s\n", sfrdump->name);
		ret = 0;
	}

	if (list_empty(&sfrdump_list))
		dev_info(&pdev->dev, "There is no sfr dump list.\n");
	else
		atomic_notifier_chain_register(&panic_notifier_list, &nb_sfrdump);

	dev_info(&pdev->dev, "%s successful.\n", __func__);

	return ret;
}

static const struct of_device_id dbg_snapshot_sfrdump_matches[] = {
	{ .compatible = "google,debug-snapshot-sfrdump", },
	{},
};
MODULE_DEVICE_TABLE(of, dbg_snapshot_sfrdump_matches);

static struct platform_driver dbg_snapshot_sfrdump_driver = {
	.probe		= dbg_snapshot_sfrdump_probe,
	.driver		= {
		.name	= "debug-snapshot-sfrdump",
		.of_match_table	= of_match_ptr(dbg_snapshot_sfrdump_matches),
	},
};
module_platform_driver(dbg_snapshot_sfrdump_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DebugSnapshot SFRdump Driver");
