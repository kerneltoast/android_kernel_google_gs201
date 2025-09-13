#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/serial_core.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include "smartpa-debug-common.h"

#define AM_DEV_NAME   "smartpa"
#define MSGS_SIZE     256
#define RESERVED_SIZE 252

struct smartpa_msg {
	char msgs[MSGS_SIZE];
	char reserved[RESERVED_SIZE];
	int msg_result;
};

struct smartpa_prars {
	int fRes_max;
	int fRes_min;
	int Qt;
	int impedance_max;
	int impedance_min;
};

#define TFA_CTL_IOC_MAGIC	'T'
#define TFA_IOCTL_SPK_REST	_IOW(TFA_CTL_IOC_MAGIC, 0x01, int)
#define TFA_IOCTL_SPK_INTS	_IOR(TFA_CTL_IOC_MAGIC, 0x02, struct smartpa_msg)
#define TFA_IOCTL_SPK_INTT	_IOW(TFA_CTL_IOC_MAGIC, 0x03, int)
#define TFA_IOCTL_SPK_RFDES	_IOR(TFA_CTL_IOC_MAGIC, 0x04, struct smartpa_msg)
#define TFA_IOCTL_SPK_CHCK	_IOR(TFA_CTL_IOC_MAGIC, 0x05, int)
#define TFA_IOCTL_SPK_PRARS	_IOR(TFA_CTL_IOC_MAGIC, 0x06, struct smartpa_prars)
#define TFA_IOCTL_SPK_ADDR	_IOW(TFA_CTL_IOC_MAGIC, 0x07, unsigned char)
#define TFA_IOCTL_SPK_MTP_BACKUP	_IOR(TFA_CTL_IOC_MAGIC, 0x08, int)
#define TFA_IOCTL_SPK_SET   _IOW(TFA_CTL_IOC_MAGIC, 0x09, struct smartpa_msg)

extern int smartpa_check_calib_dbg(void);
extern int smartpa_init_dbg(char *buffer, char *buffer1, int size);
extern int smartpa_read_freq_dbg(char *buffer, int size);
extern void smartpa_read_prars_dbg(int temp[5], unsigned char addr);
extern void smartpa_get_client(struct i2c_client **client, unsigned char addr);
extern void smartpa_init_re_value(uint32_t *re_value, int channels);

static struct i2c_client *smartpa_sysfs_client;
static unsigned char last_addr;

static ssize_t smartpa_sysfs_read(struct file *file,
	char __user *buf, size_t count, loff_t *offset)
{
	char *tmp;
	int ret;

	smartpa_get_client(&smartpa_sysfs_client, last_addr);

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	ret = i2c_master_recv(smartpa_sysfs_client, tmp, count);
	if (ret >= 0)
		ret = copy_to_user(buf, tmp, count) ? -EFAULT : ret;
	else
		pr_err("[SmartPA]%s: transfer error %d\n", __func__, ret);

	kfree(tmp);
	return ret;
}

static ssize_t smartpa_sysfs_write(struct file *file,
	const char __user *buf, size_t count, loff_t *offset)
{
	char *tmp;
	int ret;

	smartpa_get_client(&smartpa_sysfs_client, last_addr);
	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp))
		return PTR_ERR(tmp);

	ret = i2c_master_send(smartpa_sysfs_client, tmp, count);
	if (ret < 0)
		pr_err("[SmartPA]%s: transfer error %d\n", __func__, ret);

	kfree(tmp);
	return ret;
}

static long smartpa_sysfs_shared_ioctl(struct file *file,
	unsigned int cmd, void __user *arg)
{
	int ret = 0, check = 0;
	int temp[5] = {0};
	struct smartpa_msg msg = {{0}, {0} };
	struct smartpa_prars prars = {0};
	uint32_t re_calib[2] = {0};

	memset(&prars, 0, sizeof(struct smartpa_prars));
	memset(&msg, 0, sizeof(struct smartpa_msg));

	switch (cmd) {
	/* Reset MTP */
	case TFA_IOCTL_SPK_REST:
		pr_debug("%s:smartpa_ioctl SPK_REST\n", __func__);
		break;
		/* calibrate */

	case TFA_IOCTL_SPK_INTS:
		pr_debug("%s:smartpa_ioctl SPK_INTS\n", __func__);
		check = smartpa_init_dbg(msg.msgs, msg.reserved, MSGS_SIZE);
		msg.msg_result = check;
		ret = copy_to_user((void *)arg, &msg,
				sizeof(struct smartpa_msg));
		break;

	case TFA_IOCTL_SPK_INTT:
		pr_debug("%s:smartpa_ioctl SPK_INT\n", __func__);
		break;

	case TFA_IOCTL_SPK_SET:
		pr_debug("%s:smartpa_ioctl SPK_SET\n", __func__);
		ret = copy_from_user(&msg, (void __user *)arg,
				sizeof(struct smartpa_msg));
		if (ret) {
			pr_info("%s: Could not copy arg value from user\n", __func__);
			ret = -EFAULT;
		} else {
			ret = 0;
			memcpy(re_calib, msg.reserved, sizeof(re_calib));
			smartpa_init_re_value(re_calib, ARRAY_SIZE(re_calib));
			pr_info("%s,%s,calib: %x,%x,ret=%d\n", __func__, msg.msgs,
					re_calib[0], re_calib[1], ret);
		}
		break;

	case TFA_IOCTL_SPK_RFDES:
		usleep_range(10*1000, 10*1000);
		pr_debug("smartpa_ioctl SPK_ReadFDes\n");
		ret = smartpa_read_freq_dbg(msg.msgs, MSGS_SIZE);
		ret = copy_to_user((void *)arg, &msg,
				sizeof(struct smartpa_msg));
		break;
		/* checkmtp */

	case TFA_IOCTL_SPK_CHCK:
		pr_debug("smartpa_ioctl SPK Check MtpEx\n");
		check = smartpa_check_calib_dbg();
		pr_info("%s check %d.\n", __func__, check);
		ret = copy_to_user((__user int *)arg, &check, sizeof(int));
		break;

	case TFA_IOCTL_SPK_PRARS:
		pr_debug("smartpa_ioctl SPK Read f0 and Qt\n");
		smartpa_read_prars_dbg(temp, last_addr);
		prars.fRes_max = temp[0];
		prars.fRes_min = temp[1];
		prars.Qt = temp[2];
		prars.impedance_max = temp[3];
		prars.impedance_min = temp[4];
		ret = copy_to_user((void *)arg, &prars,
				sizeof(struct smartpa_prars));
		pr_info("smartpa_ioctl %d %d %d\n", temp[0], temp[1], temp[2]);
		break;

	case TFA_IOCTL_SPK_ADDR:
		ret = copy_from_user(&last_addr, (void __user *)arg,
				sizeof(unsigned char));
		pr_info("smartpa_ioctl addr %x\n", last_addr);
		break;

	case TFA_IOCTL_SPK_MTP_BACKUP:
		pr_info("%s mtp backup %d.\n", __func__, check);

	default:
		pr_err("smartpa Fail IOCTL command no such ioctl cmd = %x\n",
				cmd);
		ret = -1;
		break;
	}

	return ret;
}

static long smartpa_sysfs_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	pr_debug("%s...\n", __func__);
	return smartpa_sysfs_shared_ioctl(file, cmd, (void __user *)arg);
}

#ifdef CONFIG_COMPAT
static long smartpa_compact_sysfs_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	pr_debug("%s...\n", __func__);
	ret = smartpa_sysfs_shared_ioctl(file, cmd, compat_ptr(arg));
	return ret;
}
#endif

static int smartpa_sysfs_open(struct inode *inode, struct file *file)
{
	pr_info("[SmartPA]%s\n", __func__);
	return 0;
}

int smartpa_sysfs_release(struct inode *inode, struct file *file)
{
	pr_info("[SmartPA]%s\n", __func__);
	return 0;
}

static const struct file_operations smartpa_sysfs_fileops = {
	.owner = THIS_MODULE,
	.open  = smartpa_sysfs_open,
	.read  = smartpa_sysfs_read,
	.write = smartpa_sysfs_write,
	.unlocked_ioctl = smartpa_sysfs_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl =   smartpa_compact_sysfs_ioctl,
#endif
	.release = smartpa_sysfs_release,
};

static struct miscdevice smartpa_sysfs_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AM_DEV_NAME,
	.fops = &smartpa_sysfs_fileops,
};

int smartpa_debug_probe(struct i2c_client *client)
{
	int err = 0;

	pr_info("%s\n", __func__);

	if (smartpa_sysfs_client)
		return 0;

	err = misc_register(&smartpa_sysfs_device);
	if (err) {
		pr_err("%s: smartpa_device register failed\n", __func__);
		return err;
	}

	smartpa_sysfs_client = client;

	return 0;
}

MODULE_DESCRIPTION("smartpa sysfs driver");
MODULE_AUTHOR("chenjinquan <chenjinquan@vivo.com>");
MODULE_LICENSE("GPL");
