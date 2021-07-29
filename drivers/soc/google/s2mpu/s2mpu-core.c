// SPDX-License-Identifier: GPL-2.0-only
/*
 * Platform device driver for S2MPU.
 *
 * Copyright (C) 2020 Google LLC.
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <soc/google/s2mpu.h>
#include "s2mpu-lib.h"

static LIST_HEAD(info_list);
static DEFINE_SPINLOCK(s2mpu_driver_lock);

#ifdef S2MPU_TEST
static ssize_t parse_addrs(char *buf, u64 *start, u64 *end)
{
	char *from, *to;
	ssize_t ret;

	to = strchr(buf, '-');
	if (!to)
		return -EINVAL;

	*to = 0;
	to++;
	if (!to)
		return -EINVAL;

	from = buf;

	ret = kstrtou64(from, 16, start);
	if (ret)
		return ret;
	ret = kstrtou64(to, 16, end);
	if (ret)
		return ret;

	return ret;
}

static ssize_t s2mpu_win_open_write(struct file *file, const char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	u64 start, end, len;
	struct s2mpu_info *info;
	ssize_t ret;
	char *buf;

	if (*ppos != 0 || count == 0)
		return -EINVAL;

	buf = memdup_user(ubuf, count);
	if (IS_ERR(buf))
		return (PTR_ERR(buf));

	buf[count - 1] = 0;

	ret = parse_addrs(buf, &start, &end);
	if (ret)
		goto out;

	len = end - start;
	info = file->f_inode->i_private;

	s2mpu_lib_open_close(info, start, len, true, DMA_BIDIRECTIONAL);

	*ppos = count;
	ret = count;
out:
	kfree(buf);
	return ret;
}

static ssize_t s2mpu_win_close_write(struct file *file, const char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	u64 start, end, len;
	struct s2mpu_info *info;
	ssize_t ret;
	char *buf;

	if (*ppos != 0 || count == 0)
		return -EINVAL;

	buf = memdup_user(ubuf, count);
	if (IS_ERR(buf))
		return (PTR_ERR(buf));

	buf[count - 1] = 0;

	ret = parse_addrs(buf, &start, &end);
	if (ret)
		goto out;

	len = end - start;
	info = file->f_inode->i_private;
	s2mpu_lib_open_close(info, start, len, false, DMA_BIDIRECTIONAL);

	*ppos = count;
	ret = count;
out:
	kfree(buf);
	return ret;
}

static int s2mpu_enable_read(void *data, u64 *val)
{
	struct s2mpu_info *info = data;

	*val = s2mpu_get_enabled(info);

	return 0;
}

static int s2mpu_enable_write(void *data, u64 val)
{
	s2mpu_set_enabled(data, !!val);

	return 0;
}

static const struct file_operations s2mpu_win_open_fops = {
	.open   = simple_open,
	.write  = s2mpu_win_open_write,
};

static const struct file_operations s2mpu_win_close_fops = {
	.open   = simple_open,
	.write  = s2mpu_win_close_write,
};

DEFINE_SIMPLE_ATTRIBUTE(s2mpu_enable_fops, s2mpu_enable_read,
			s2mpu_enable_write, "%llu\n");

static void s2mpu_debugfs(struct s2mpu_info *info, const char *dir_name)
{
	struct dentry *win_open;
	struct dentry *win_close;
	struct dentry *enable;
	struct device *dev = info->dev;
	struct dentry *s2mpu_dentry;

	s2mpu_dentry = debugfs_create_dir(dir_name, NULL);
	if (!s2mpu_dentry) {
		dev_warn(dev, "debugfs init failed\n");
		return;
	}

	info->debugfs_dentry = s2mpu_dentry;

	win_open = debugfs_create_file("win_open", 0200, s2mpu_dentry, info,
				       &s2mpu_win_open_fops);
	if (!win_open)
		goto err_out;

	win_close = debugfs_create_file("win_close", 0200, s2mpu_dentry, info,
					&s2mpu_win_close_fops);
	if (!win_close)
		goto err_out;

	enable = debugfs_create_file("enable", 0600, s2mpu_dentry, info,
				     &s2mpu_enable_fops);
	if (!enable)
		goto err_out;

	dev_info(dev, "debugfs init done\n");

	return;

err_out:
	dev_err(dev, "debugfs init failed\n");
	debugfs_remove_recursive(s2mpu_dentry);
	s2mpu_dentry = NULL;
	info->debugfs_dentry = NULL;
}
#endif

int s2mpu_open(struct s2mpu_info *info, phys_addr_t start_pa, size_t len,
	       enum dma_data_direction dir)
{
	if (!info)
		return -EINVAL;

	return s2mpu_lib_open_close(info, start_pa, len, true, dir);
}
EXPORT_SYMBOL_GPL(s2mpu_open);

int s2mpu_close(struct s2mpu_info *info, phys_addr_t start_pa, size_t len,
		enum dma_data_direction dir)
{
	if (!info)
		return -EINVAL;

	return s2mpu_lib_open_close(info, start_pa, len, false, dir);
}
EXPORT_SYMBOL_GPL(s2mpu_close);

int s2mpu_restore(struct s2mpu_info *info)
{
	if (!info)
		return -EINVAL;

	return s2mpu_lib_restore(info);
}
EXPORT_SYMBOL_GPL(s2mpu_restore);

struct s2mpu_info *s2mpu_fwnode_to_info(struct fwnode_handle *fwnode)
{
	struct s2mpu_info *iter, *info = NULL;

	spin_lock(&s2mpu_driver_lock);
	list_for_each_entry(iter, &info_list, list)
		if (iter->dev->fwnode == fwnode) {
			info = iter;
			break;
		}

	spin_unlock(&s2mpu_driver_lock);

	return info;
}
EXPORT_SYMBOL_GPL(s2mpu_fwnode_to_info);

static int s2mpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpu_info *info;
	void __iomem *ssmt_base;
	struct resource *res;
	void __iomem *base;
	int sidcount;
	int irq_num;
	u32 *sids;
	u32 vid;
	int ret;

	dma_set_mask(dev, DMA_BIT_MASK(36));

	/* s2mpu config */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "s2mpu");
	if (!res) {
		dev_warn(dev, "failed to read s2mpu addr from device tree\n");
		return -EINVAL;
	}

	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base)) {
		dev_err(dev, "failed to map s2mpu csr\n");
		return PTR_ERR(base);
	}

	/* ssmt config */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ssmt");
	if (!res) {
		dev_err(dev, "failed to read ssmt addr from device tree\n");
		return -EINVAL;
	}

	ssmt_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ssmt_base)) {
		dev_err(dev, "failed to map ssmt csr\n");
		return PTR_ERR(ssmt_base);
	}

	ret = of_property_read_u32(dev->of_node, "vid", &vid);
	if (ret) {
		dev_err(dev, "failed to read property vid\n");
		return ret;
	}

	if (vid >= 8) {
		dev_err(dev, "vid (%u) out of range\n", vid);
		return -EINVAL;
	}

	sidcount = of_property_count_u32_elems(dev->of_node, "sids");
	if (sidcount < 0) {
		dev_err(dev, "failed to get sids\n");
		return -EINVAL;
	}

	sids = devm_kcalloc(dev, sidcount, sizeof(*sids), GFP_KERNEL);
	if (!sids)
		return -ENOMEM;

	ret = of_property_read_u32_array(dev->of_node, "sids", sids, sidcount);
	if (ret) {
		dev_err(dev, "failed to read in sids\n");
		return ret;
	}

	info = s2mpu_lib_init(dev, base, ssmt_base, vid, sids, sidcount);
	if (IS_ERR(info))
		return PTR_ERR(info);

	/* register s2mpu fault handler interrupt */
	irq_num = platform_get_irq(pdev, 0);
	if (irq_num < 0)
		goto list_add;

	ret = devm_request_irq(dev, irq_num, s2mpu_lib_irq_handler, IRQF_TRIGGER_NONE,
			       dev_name(dev), info);
	if (ret)
		dev_err(dev, "request_irq failed %d\n", ret);

list_add:
	spin_lock(&s2mpu_driver_lock);
	list_add_tail(&info->list, &info_list);
	spin_unlock(&s2mpu_driver_lock);

	platform_set_drvdata(pdev, info);

#ifdef S2MPU_TEST
	s2mpu_debugfs(info, pdev->name);
#endif
	return 0;
}

static int s2mpu_remove(struct platform_device *pdev)
{
	struct s2mpu_info *info;

	info = platform_get_drvdata(pdev);
#ifdef S2MPU_TEST
	debugfs_remove_recursive(info->debugfs_dentry);
#endif
	s2mpu_lib_deinit(info);

	return 0;
}

static const struct of_device_id s2mpu_of_match[] = {
	{ .compatible = "google,gs101-s2mpu-v1", },
	{},
};
MODULE_DEVICE_TABLE(of, s2mpu_of_match);

static struct platform_driver s2mpu_driver = {
	.probe = s2mpu_probe,
	.remove = s2mpu_remove,
	.driver = {
		.name = "s2mpu",
		.of_match_table = s2mpu_of_match,
	},
};

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Google S2MPU driver");
module_platform_driver(s2mpu_driver);
