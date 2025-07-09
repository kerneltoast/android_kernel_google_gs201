// SPDX-License-Identifier: GPL-2.0-only
/*
 * The helper functions for fault injection.
 *
 * Copyright (C) 2023 Google LLC
 */

#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <gcip/gcip-fault-injection.h>
#include <gcip/gcip-kci.h>
#include <gcip/gcip-pm.h>

static int gcip_fault_inject_send_locked(struct gcip_fault_inject *injection)
{
	int i, ret, offset = 0;
	char *buf;

	lockdep_assert_held(&injection->lock);

	if (injection->progress != GCIP_FAULT_INJECT_PROGRESS_PENDING)
		return 0;

	buf = kzalloc(FAULT_INJECT_BUF_SIZE, GFP_KERNEL);
	if (buf) {
		for (i = 0; i < GCIP_FAULT_INJECT_OPAQUE_SIZE; i++)
			offset += scnprintf(buf + offset, FAULT_INJECT_BUF_SIZE - offset, " %u",
					    injection->opaque[i]);
		dev_info(injection->dev, "Inserting fault:%s", buf);
	}
	kfree(buf);

	injection->progress = GCIP_FAULT_INJECT_PROGRESS_INJECTED;

	ret = injection->send_kci(injection);
	if (!ret) {
		injection->fw_support_status = GCIP_FAULT_INJECT_STATUS_SUPPORTED;
	} else if (ret == GCIP_KCI_ERROR_UNIMPLEMENTED) {
		injection->fw_support_status = GCIP_FAULT_INJECT_STATUS_UNSUPPORTED;
	} else {
		injection->fw_support_status = GCIP_FAULT_INJECT_STATUS_ERROR;
		if (ret > 0) {
			dev_warn(injection->dev,
				 "Fault injection KCI not accepted by the firmware: %d\n", ret);
		} else {
			/* Keep pending if the failure was in the driver side. */
			injection->progress = GCIP_FAULT_INJECT_PROGRESS_PENDING;
			dev_warn(injection->dev, "Failed to send the fault injection KCI: %d", ret);
		}
	}

	return ret;
}

/**
 * gcip_fault_injection_set() - Set the fault-injection values received from the DebugFS.
 * @filp: The file pointer.
 * @buff: The user buffer holding the data to be written.
 * @count: The size of the requested data transfer.
 * @offp: The file position the user is accessing.
 *
 * Return: A non-negative return value represents the number of bytes successfully read.
 */
static ssize_t gcip_fault_injection_set(struct file *filp, const char __user *buff, size_t count,
					loff_t *offp)
{
	struct gcip_fault_inject *injection = filp->f_inode->i_private;
	bool mcu_ready = !gcip_pm_get_if_powered(injection->pm, false);
	char *input = NULL;
	int ret;
	int i;
	int start = 0;
	uint32_t val;
	int consume;

	if (*offp || (count + 1 >= FAULT_INJECT_BUF_SIZE)) {
		ret = -EINVAL;
		goto out;
	}

	input = kzalloc(count + 1, GFP_KERNEL);
	if (!input) {
		ret = -ENOMEM;
		goto out;
	}

	if (copy_from_user(input, buff, count)) {
		ret = -EFAULT;
		goto out;
	}

	mutex_lock(&injection->lock);

	if (injection->progress == GCIP_FAULT_INJECT_PROGRESS_PENDING)
		dev_warn(injection->dev, "Ignore pending fault injection.\n");

	memset(injection->opaque, 0, sizeof(injection->opaque));
	for (i = 0; i < ARRAY_SIZE(injection->opaque); i++) {
		if (sscanf(input + start, "%u%n", &val, &consume) <= 0)
			break;

		start += consume;
		injection->opaque[i] = val;
	}

	injection->progress = GCIP_FAULT_INJECT_PROGRESS_PENDING;

	if (!mcu_ready)
		dev_dbg(injection->dev, "MCU is not ready, pend sending fault injection");
	else
		gcip_fault_inject_send_locked(injection);

	ret = count;

	mutex_unlock(&injection->lock);

out:
	kfree(input);

	if (mcu_ready)
		gcip_pm_put(injection->pm);

	return ret;
}

/* Write the fault injection progress rate and content. */
static void write_injection_content(struct gcip_fault_inject *injection, char *output, loff_t *offp,
				    const size_t buf_size)
{
	int i;

	if (injection->progress == GCIP_FAULT_INJECT_PROGRESS_NONE) {
		*offp += scnprintf(output + *offp, buf_size - *offp, "none\n");
		return;
	}
	*offp += scnprintf(output + *offp, buf_size - *offp, "%s\n",
			   injection->progress == GCIP_FAULT_INJECT_PROGRESS_PENDING ? "pending" :
										       "injected");
	for (i = 0; i < GCIP_FAULT_INJECT_OPAQUE_SIZE; i++)
		*offp += scnprintf(output + *offp, buf_size - *offp, "%.*s%u", i, " ",
				   injection->opaque[i]);
	*offp += scnprintf(output + *offp, buf_size - *offp, "\n");
}

/**
 * gcip_fault_injection_get() - Get the fault-injection values.
 * @filp: The file pointer.
 * @buff: The empty buffer where the newly read data should be placed.
 * @count: The size of the requested data transfer.
 * @offp: The file position the user is accessing.
 *
 * Return: A non-negative return value represents the number of bytes successfully written.
 */
static ssize_t gcip_fault_injection_get(struct file *filp, char __user *buff, size_t count,
					loff_t *offp)
{
	struct gcip_fault_inject *injection = filp->f_inode->i_private;
	const size_t buf_size = min_t(size_t, count, FAULT_INJECT_BUF_SIZE);
	int ret;
	char *output;

	if (*offp)
		return 0;

	output = kzalloc(buf_size, GFP_KERNEL);
	if (!output)
		return -ENOMEM;

	mutex_lock(&injection->lock);

	switch (injection->fw_support_status) {
	case GCIP_FAULT_INJECT_STATUS_UNKNOWN:
		*offp += scnprintf(output + *offp, buf_size - *offp, "unknown\n");
		write_injection_content(injection, output, offp, buf_size);
		break;
	case GCIP_FAULT_INJECT_STATUS_ERROR:
		*offp += scnprintf(output + *offp, buf_size - *offp, "error\n");
		break;
	case GCIP_FAULT_INJECT_STATUS_UNSUPPORTED:
		*offp += scnprintf(output + *offp, buf_size - *offp, "unsupported\n");
		break;
	default:
		*offp += scnprintf(output + *offp, buf_size - *offp, "supported\n");
		write_injection_content(injection, output, offp, buf_size);
	}

	mutex_unlock(&injection->lock);

	if (copy_to_user(buff, output, *offp))
		ret = -EFAULT;
	else
		ret = *offp;

	kfree(output);

	return ret;
}

static const struct file_operations fault_inject_fops = { .write = gcip_fault_injection_set,
							  .read = gcip_fault_injection_get };

struct gcip_fault_inject *gcip_fault_inject_create(const struct gcip_fault_inject_args *args)
{
	struct gcip_fault_inject *injection =
		devm_kzalloc(args->dev, sizeof(*injection), GFP_KERNEL);

	if (!injection)
		return ERR_PTR(-ENOMEM);

	injection->dev = args->dev;
	injection->pm = args->pm;
	injection->send_kci = args->send_kci;
	injection->kci_data = args->kci_data;
	injection->progress = GCIP_FAULT_INJECT_PROGRESS_NONE;
	injection->fw_support_status = GCIP_FAULT_INJECT_STATUS_UNKNOWN;

	mutex_init(&injection->lock);
	injection->d_entry = debugfs_create_file(DEBUGFS_FAULT_INJECTION, 0600, args->parent_dentry,
						 injection, &fault_inject_fops);

	return injection;
}

void gcip_fault_inject_destroy(struct gcip_fault_inject *injection)
{
	if (injection) {
		debugfs_remove(injection->d_entry);
		devm_kfree(injection->dev, injection);
	}
}

int gcip_fault_inject_send(struct gcip_fault_inject *injection)
{
	int ret;

	if (!injection)
		return -ENODEV;

	mutex_lock(&injection->lock);
	ret = gcip_fault_inject_send_locked(injection);
	mutex_unlock(&injection->lock);

	return ret;
}
