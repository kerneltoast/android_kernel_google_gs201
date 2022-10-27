// SPDX-License-Identifier: GPL-2.0-only
/*
 * Android BootControl Service Support
 *
 * Copyright 2022 Google LLC
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/arm-smccc.h>

#define SMC_CM_OTP_CONTROL (0xC2001014UL)

#define CMD_W_ANTIRBK_NS_AP (0x7UL)
#define CMD_W_ANTIRBK_S_AP (0x9UL)

static u32 blow_ar_done;

static ssize_t blow_ar_val_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", blow_ar_done);
}

static ssize_t blow_ar_val_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned long val = 0;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return -EINVAL;

	if (blow_ar_done)
		return -EINVAL;

	if (val) {
		struct arm_smccc_res res;

		arm_smccc_smc(SMC_CM_OTP_CONTROL, CMD_W_ANTIRBK_NS_AP,
			      0, 0, 0, 0, 0, 0, &res);
		if (res.a0) {
			pr_err("Fail to blow ap_ns AR %lx\n", res.a0);
			return -EIO;
		}
		arm_smccc_smc(SMC_CM_OTP_CONTROL, CMD_W_ANTIRBK_S_AP,
			      0, 0, 0, 0, 0, 0, &res);
		if (res.a0) {
			pr_err("Fail to blow ap_s AR %lx\n", res.a0);
			return -EIO;
		}

		blow_ar_done = !!val;
	}

	return count;
}

static struct kobj_attribute blow_ar_attribute =
	__ATTR(blow_ar, 0644, blow_ar_val_show, blow_ar_val_store);
static struct attribute *attrs[] = {
	&blow_ar_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static int boot_control_init(void)
{
	struct kobject *boot_control_kobj;
	int ret;

	boot_control_kobj = kobject_create_and_add("boot_control", kernel_kobj);
	if (!boot_control_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(boot_control_kobj, &attr_group);
	if (ret)
		goto put_mm_kobj;

	return 0;

put_mm_kobj:
	kobject_put(boot_control_kobj);

	return ret;
}

module_init(boot_control_init);
MODULE_LICENSE("GPL");
