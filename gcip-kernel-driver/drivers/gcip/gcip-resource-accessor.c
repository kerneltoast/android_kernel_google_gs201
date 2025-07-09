// SPDX-License-Identifier: GPL-2.0-only
/*
 * GCIP helpers for accessing resources for debugging.
 *
 * Copyright (C) 2023 Google LLC
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#include <gcip/gcip-resource-accessor.h>

#define RESOURCE_ACCESSOR "resource-accessor"

static int gcip_resource_accessor_get_type(struct gcip_resource_accessor *accessor,
					   phys_addr_t addr)
{
	struct gcip_resource_list_element *cur;
	unsigned long flags;
	int ret = -EFAULT;

	spin_lock_irqsave(&accessor->resource_list_lock, flags);
	list_for_each_entry(cur, &accessor->resource_list, list) {
		if (cur->resource.start <= addr && cur->resource.end >= addr) {
			ret = resource_type(&cur->resource);
			break;
		}
	}
	spin_unlock_irqrestore(&accessor->resource_list_lock, flags);
	return ret;
}

static inline bool is_valid_width(unsigned int width)
{
	switch (width) {
	case 1:
	case 2:
	case 4:
	case 8:
		return true;
	}

	return false;
}

static int gcip_resource_accessor_addr_read(struct gcip_resource_accessor *accessor,
					    phys_addr_t addr, unsigned int width)
{
	int type = gcip_resource_accessor_get_type(accessor, addr);

	if (type < 0) {
		dev_warn(accessor->dev, "Failed to find a registered resource for %pap\n", &addr);
		return -EINVAL;
	}

	if (!is_valid_width(width)) {
		dev_warn(accessor->dev, "%u-byte access is invalid\n", width);
		return -EINVAL;
	}

	accessor->last_query_addr = addr;
	accessor->last_query_width = width;
	return 0;
}

static int gcip_resource_accessor_addr_write(struct gcip_resource_accessor *accessor,
					     phys_addr_t addr, unsigned int width, u64 val)
{
	void __iomem *vaddr;
	int type = gcip_resource_accessor_get_type(accessor, addr);

	if (!is_valid_width(width)) {
		dev_warn(accessor->dev, "%u-byte access is invalid\n", width);
		return -EINVAL;
	}

	if (type == IORESOURCE_MEM) {
		vaddr = memremap(addr, width, MEMREMAP_WC);
	} else if (type == IORESOURCE_IO) {
		vaddr = ioremap(addr, width);
	} else {
		dev_warn(accessor->dev, "Failed to find a registered resource for %pap\n", &addr);
		return -EINVAL;
	}

	if (!vaddr) {
		dev_warn(accessor->dev, "Failed to map %pap\n", &addr);
		return -EFAULT;
	}

	switch (width) {
	case 1:
		writeb(val, vaddr);
		break;
	case 2:
		writew(val, vaddr);
		break;
	case 4:
		writel(val, vaddr);
		break;
	case 8:
		writeq(val, vaddr);
		break;
	}
	/*
	 * Also records the written address. It would be helpful to check if we write the address
	 * successfully.
	 */
	accessor->last_query_addr = addr;
	accessor->last_query_width = width;

	if (type == IORESOURCE_MEM)
		memunmap(vaddr);
	else if (type == IORESOURCE_IO)
		iounmap(vaddr);

	return 0;
}

static ssize_t gcip_resource_accessor_read(struct file *file, char __user *user_buf, size_t len,
					   loff_t *ppos)
{
	/* 64 is enough for an 8-byte address, 8-byte value, the space and the separator. */
	char buf[64];
	ssize_t ret;
	struct gcip_resource_accessor *accessor;
	void __iomem *vaddr;
	phys_addr_t addr;
	unsigned int width;
	u64 val = 0;
	int type, size;

	accessor = file->private_data;
	addr = accessor->last_query_addr;
	width = accessor->last_query_width;
	if (addr == 0) {
		dev_warn(accessor->dev, "No available query address\n");
		return -EINVAL;
	}

	if (!is_valid_width(width)) {
		dev_warn(accessor->dev, "%u-byte access is invalid\n", width);
		return -EINVAL;
	}

	type = gcip_resource_accessor_get_type(accessor, addr);
	if (type == IORESOURCE_MEM) {
		vaddr = memremap(addr, width, MEMREMAP_WC);
	} else if (type == IORESOURCE_IO) {
		vaddr = ioremap(addr, width);
	} else {
		dev_warn(accessor->dev, "Failed to find a registered resource for %pap\n", &addr);
		return -EINVAL;
	}

	if (!vaddr) {
		dev_warn(accessor->dev, "Failed to map %pap\n", &addr);
		return -EFAULT;
	}

	switch (width) {
	case 1:
		val = readb(vaddr);
		break;
	case 2:
		val = readw(vaddr);
		break;
	case 4:
		val = readl(vaddr);
		break;
	case 8:
		val = readq(vaddr);
		break;
	}
	/* width-byte value is width * 2 long in hex, + 2 for '0x'. */
	size = scnprintf(buf, sizeof(buf), "%pap: %#0*llx\n", &addr, width * 2 + 2, val);

	if (type == IORESOURCE_MEM)
		memunmap(vaddr);
	else if (type == IORESOURCE_IO)
		iounmap(vaddr);

	ret = simple_read_from_buffer(user_buf, len, ppos, buf, size);
	return ret;
}

static ssize_t gcip_resource_accessor_write(struct file *file, const char __user *user_buf,
					    size_t len, loff_t *ppos)
{
	/* 64 is enough for two 8-byte hex, a 4-byte decimal, and spaces. */
	char buf[64] = {};
	size_t size;
	struct gcip_resource_accessor *accessor;
	u64 addr;
	unsigned int width;
	u64 value;
	int num_parsed;
	int err;

	size = min(sizeof(buf) - 1, len);
	if (copy_from_user(buf, user_buf, size))
		return -EFAULT;

	accessor = file->private_data;
	num_parsed = sscanf(buf, "%llx %u %llx", &addr, &width, &value);
	if (num_parsed == 2) {
		err = gcip_resource_accessor_addr_read(accessor, addr, width);
		if (err)
			return err;
	} else if (num_parsed == 3) {
		err = gcip_resource_accessor_addr_write(accessor, addr, width, value);
		if (err)
			return err;
	} else {
		dev_warn(
			accessor->dev,
			"The input format: <address in hex> <1|2|4|8|> [value in hex]\n");
		return -EINVAL;
	}

	return size;
}

static int gcip_resource_accessor_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return nonseekable_open(inode, file);
}

static const struct file_operations fops_gcip_resource_accessor = {
	.owner = THIS_MODULE,
	.read = gcip_resource_accessor_read,
	.write = gcip_resource_accessor_write,
	.open = gcip_resource_accessor_open,
};

struct gcip_resource_accessor *gcip_resource_accessor_create(struct device *dev,
							     struct dentry *parent_dentry)
{
	struct gcip_resource_accessor *accessor = devm_kzalloc(dev, sizeof(*accessor), GFP_KERNEL);

	if (!accessor)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&accessor->resource_list);
	spin_lock_init(&accessor->resource_list_lock);
	accessor->dev = dev;

	accessor->dentry = debugfs_create_file(RESOURCE_ACCESSOR, 0600, parent_dentry, accessor,
					       &fops_gcip_resource_accessor);

	if (IS_ERR(accessor->dentry)) {
		dev_warn(dev, "Failed to create debugfs for resource accessor (ret=%ld)\n",
			 PTR_ERR(accessor->dentry));
		return ERR_CAST(accessor->dentry);
	}

	return accessor;
}

/* No need to release resource lists since those memories are device managed. */
void gcip_resource_accessor_destroy(struct gcip_resource_accessor *accessor)
{
	debugfs_remove(debugfs_lookup(RESOURCE_ACCESSOR, accessor->dentry));
}

int gcip_register_accessible_resource(struct gcip_resource_accessor *accessor,
				      const struct resource *r)
{
	unsigned long flags;
	struct gcip_resource_list_element *gcip_res_list =
		devm_kzalloc(accessor->dev, sizeof(*gcip_res_list), GFP_KERNEL);

	if (!gcip_res_list)
		return -ENOMEM;

	memcpy(&gcip_res_list->resource, r, sizeof(*r));

	spin_lock_irqsave(&accessor->resource_list_lock, flags);
	list_add_tail(&gcip_res_list->list, &accessor->resource_list);
	spin_unlock_irqrestore(&accessor->resource_list_lock, flags);

	return 0;
}
