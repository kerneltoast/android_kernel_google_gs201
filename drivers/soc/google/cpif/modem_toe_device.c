// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * MODEM TOE device support
 *
 */

#include "modem_toe_device.h"
#include "dit.h"
#include "link_device.h"

#define TOE_DEV_NAME	"umts_toe0"

static struct toe_ctrl_t *tc;

void toe_set_iod_clat_netdev(struct io_device *iod, void *args)
{
	struct clat_info *clat = (struct clat_info *) args;
	struct net_device *ndev = NULL;
	struct link_device *ld = get_current_link(iod);
	unsigned long flags;

	if (strncmp(iod->name, clat->ipv6_iface, IFNAMSIZ) != 0)
		return;

	if (clat->ipv4_iface[0])
		ndev = dev_get_by_name(&init_net, clat->ipv4_iface);

	if (!clat->ipv4_iface[0] || ndev) {
		spin_lock_irqsave(&iod->clat_lock, flags);
		if (iod->clat_ndev)
			dev_put(iod->clat_ndev);

		if (ndev)
			ndev->features |= NETIF_F_GRO_FRAGLIST;
		iod->clat_ndev = ndev;
		spin_unlock_irqrestore(&iod->clat_lock, flags);

#if IS_ENABLED(CONFIG_CPIF_TP_MONITOR)
		if (iod->clat_ndev) {
			struct mem_link_device *mld = to_mem_link_device(ld);

			mif_info("set RPS again\n");
			mld->tpmon->reset_data("RPS");
		}
#endif

		mif_info("%s clat netdev[%d] ch: %d, iface v6/v4: %s/%s\n",
			(ndev ? "set" : "clear"), clat->clat_index, iod->ch,
			clat->ipv6_iface, clat->ipv4_iface);
	}
}

static int toe_dev_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int toe_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}

static unsigned int toe_dev_poll(struct file *filp, struct poll_table_struct *wait)
{
	return 0;
}

static ssize_t toe_dev_read(struct file *filp, char *buf, size_t count, loff_t *fpos)
{
	return 0;
}

static long toe_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct clat_info clat;
	u32 ready, num;
	struct mem_link_device *mld = tc->mld;

	switch (cmd) {
	case IOCTL_TOE_SET_CLAT_READY:
		if (copy_from_user(&ready, (const void __user *)arg, sizeof(ready)))
			return -EFAULT;

		tc->clat_hal_ready = (ready ? true : false);
		break;
	case IOCTL_TOE_SET_CLAT_IFACES_NUM:
		if (copy_from_user(&num, (const void __user *)arg, sizeof(num)))
			return -EFAULT;

		tc->clat_ifaces_num = num;
		break;
	case IOCTL_TOE_SET_CLAT_INFO:
		if (copy_from_user(&clat, (const void __user *)arg, sizeof(struct clat_info)))
			return -EFAULT;

		if (!tc->set_clat_info || !tc->set_clat_info(mld, &clat))
			return -EINVAL;
		break;
	default:
		mif_err("unknown command: 0x%X\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;

	count += scnprintf(&buf[count], PAGE_SIZE - count,
			   "hal_ready:%d ifaces_num:%d dev_support:%d\n",
			   tc->clat_hal_ready, tc->clat_ifaces_num, tc->clat_dev_support);

	return count;
}

static DEVICE_ATTR_RO(status);

static struct attribute *toe_attrs[] = {
	&dev_attr_status.attr,
	NULL,
};

static const struct attribute_group toe_group = {
	.attrs = toe_attrs,
	.name = "toe",
};

static const struct file_operations toe_dev_fops = {
	.owner		= THIS_MODULE,
	.open		= toe_dev_open,
	.poll		= toe_dev_poll,
	.read		= toe_dev_read,
	.release	= toe_dev_release,
	.compat_ioctl	= toe_dev_ioctl,
	.unlocked_ioctl	= toe_dev_ioctl,
};

static struct miscdevice toe_dev_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= TOE_DEV_NAME,
	.fops	= &toe_dev_fops,
};

int toe_dev_init(struct mem_link_device *mld)
{
	if (unlikely(!tc)) {
		mif_err("toe not created\n");
		return -EPERM;
	}

	tc->mld = mld;
	tc->clat_dev_support = false;
	tc->set_clat_info = NULL;
	tc->set_iod_clat_netdev = toe_set_iod_clat_netdev;

	/* Can add the other devs or change the ordering */
	if (dit_support_clat()) {
		tc->clat_dev_support = true;
		tc->set_clat_info = dit_hal_set_clat_info;
	}
#if IS_ENABLED(CONFIG_CP_PKTPROC_CLAT)
	else {
		tc->clat_dev_support = true;
		tc->set_clat_info = shmem_ap2cp_write_clatinfo;
	}
#endif
	mld->tc = tc;

	return 0;
}

int toe_dev_create(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = 0;

	tc = devm_kzalloc(dev, sizeof(struct toe_ctrl_t), GFP_KERNEL);
	if (!tc) {
		mif_err("toe ctrl alloc failed\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(&dev->kobj, &toe_group);
	if (ret != 0) {
		mif_err("sysfs_create_group() error %d\n", ret);
		goto error;
	}

	ret = misc_register(&toe_dev_misc);
	if (ret) {
		mif_err("misc register error\n");
		goto error;
	}

	return 0;

error:
	if (tc) {
		devm_kfree(dev, tc);
		tc = NULL;
	}

	return ret;
}
