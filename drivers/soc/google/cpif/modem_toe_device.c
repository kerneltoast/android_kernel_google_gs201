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

#define TOE_DEV_NAME	"umts_toe0"

static struct toe_ctrl_t *tc;

/*
 * bpf_skb_proto_6_to_4 does not support UDP GRO with fraglist.
 * check if the BPF tc is not needed to avoid edge cases.
 */
bool toe_check_6_to_4_ready(void)
{
	if (!tc->clat_hal_ready)
		return false;

	if (!tc->clat_ifaces_num)
		return true;

	return tc->clat_dev_support;
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

		if (!tc->set_clat_info || !tc->set_clat_info(&clat))
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
			   "6_to_4_ready:%d\n", toe_check_6_to_4_ready());
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

int toe_dev_init(void)
{
	if (unlikely(!tc)) {
		mif_err("toe not created\n");
		return -EPERM;
	}

	tc->clat_dev_support = false;
	tc->set_clat_info = NULL;

	/* Can add the other devs or change the ordering */
	if (dit_support_clat()) {
		tc->clat_dev_support = true;
		tc->set_clat_info = dit_hal_set_clat_info;
	}

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
