// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2019 Google LLC. All Rights Reserved.
 *
 * Character device interface for AoC services
 * ACD - AoC Character Device
 */

#define pr_fmt(fmt) "aoc_char: " fmt

#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "aoc.h"
#include "aoc_ipc_core.h"

#define ACD_CHARDEV_NAME "aoc_char"

static int acd_major = -1;
static int acd_major_dev;
static unsigned int acd_next_minor; /* protected by acd_devices_lock */
static struct class *acd_class;

struct acd_device_entry {
	struct device *acd_device;
	struct aoc_service_dev *service;
	bool opened;
	struct list_head list;
	struct kref refcount;
};

static LIST_HEAD(acd_devices_list); /* protected by acd_devices_lock */
static DEFINE_MUTEX(acd_devices_lock);

/* Driver methods */
static int acd_probe(struct aoc_service_dev *dev);
static int acd_remove(struct aoc_service_dev *dev);

static struct aoc_driver aoc_char_driver = {
	.drv = {
			.name = "aoc_char",
		},
	.probe = acd_probe,
	.remove = acd_remove,
};

/* File methods */
static int acd_open(struct inode *inode, struct file *file);
static int acd_release(struct inode *inode, struct file *file);
static long acd_unlocked_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg);
static ssize_t acd_read(struct file *file, char __user *buf, size_t count,
			loff_t *off);
static ssize_t acd_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *off);
static unsigned int acd_poll(struct file *file, poll_table *wait);

static const struct file_operations fops = {
	.open = acd_open,
	.release = acd_release,
	.unlocked_ioctl = acd_unlocked_ioctl,
	.read = acd_read,
	.write = acd_write,
	.poll = acd_poll,

	.owner = THIS_MODULE,
};

static void acd_device_entry_release(struct kref *ref)
{
	kfree(container_of(ref, struct acd_device_entry, refcount));
}

/*
 * Caller must hold acd_devices_lock.
 */
static struct acd_device_entry *acd_device_entry_for_inode(struct inode *inode)
{
	struct acd_device_entry *entry;

	list_for_each_entry(entry, &acd_devices_list, list) {
		/* entries with service->dead == true can't be in acd_devices_list */
		if (entry->acd_device->devt == inode->i_rdev)
			return entry;
	}

	return NULL;
}

static char *acd_devnode(struct device *dev, umode_t *mode)
{
	if (!mode || !dev)
		return NULL;

	if (MAJOR(dev->devt) == acd_major)
		*mode = 0666;

	return kasprintf(GFP_KERNEL, "%s", dev_name(dev));
}

static int create_character_device(struct aoc_service_dev *dev)
{
	int rc = 0;
	struct acd_device_entry *new_entry;

	new_entry = kmalloc(sizeof(*new_entry), GFP_KERNEL);
	if (!new_entry) {
		rc = -ENOMEM;
		goto err_kmalloc;
	}

	mutex_lock(&acd_devices_lock);
	new_entry->acd_device = device_create(acd_class, &dev->dev,
					      MKDEV(acd_major, acd_next_minor), NULL,
					      "acd-%s", dev_name(&dev->dev));
	if (IS_ERR(new_entry->acd_device)) {
		pr_err("device_create failed: %ld\n", PTR_ERR(new_entry->acd_device));
		rc = PTR_ERR(new_entry->acd_device);
		goto err_device_create;
	}
	get_device(&dev->dev);
	new_entry->service = dev;
	acd_next_minor++;
	new_entry->opened = false;
	kref_init(&new_entry->refcount);
	list_add(&new_entry->list, &acd_devices_list);
	mutex_unlock(&acd_devices_lock);
	return 0;

err_device_create:
	mutex_unlock(&acd_devices_lock);
	kfree(new_entry);
err_kmalloc:
	return rc;
}

static int acd_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	struct acd_device_entry *entry;

	pr_debug("attempt to open major:%d minor:%d\n", MAJOR(inode->i_rdev),
		 MINOR(inode->i_rdev));

	mutex_lock(&acd_devices_lock);
	entry = acd_device_entry_for_inode(inode);
	if (!entry) {
		rc = -ENODEV;
		goto err_acd_device_entry;
	}

	if (entry->opened) {
		rc = -EBUSY;
		goto err_open;
	}

	kref_get(&entry->refcount);
	get_device(&entry->service->dev);
	entry->opened = true;
	file->private_data = entry;
	mutex_unlock(&acd_devices_lock);
	return 0;

err_open:
err_acd_device_entry:
	mutex_unlock(&acd_devices_lock);
	return rc;
}

static int acd_release(struct inode *inode, struct file *file)
{
	struct acd_device_entry *entry = file->private_data;

	if (!entry)
		return -ENODEV;

	entry->opened = false;
	put_device(&entry->service->dev);
	kref_put(&entry->refcount, acd_device_entry_release);
	file->private_data = NULL;

	return 0;
}

static long acd_unlocked_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	/*
	 * struct file_prvdata *private = file->private_data;
	 *
	 * if (!private)
	 *	return -ENODEV;
	 */

	return -EINVAL;
}

static ssize_t acd_read(struct file *file, char __user *buf, size_t count,
			loff_t *off)
{
	struct acd_device_entry *entry = file->private_data;
	char *buffer;
	size_t leftover;
	ssize_t retval = 0;
	bool should_block = ((file->f_flags & O_NONBLOCK) == 0);

	if (!entry)
		return -ENODEV;

	buffer = kmalloc(count, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	retval =
		aoc_service_read(entry->service, buffer, count, should_block);
	if (retval >= 0) {
		leftover = copy_to_user(buf, buffer, retval);
		retval = retval - leftover;
	}

	kfree(buffer);
	return retval;
}

static ssize_t acd_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *off)
{
	struct acd_device_entry *entry = file->private_data;
	bool should_block = ((file->f_flags & O_NONBLOCK) == 0);
	char *buffer;
	size_t leftover;
	ssize_t retval = 0;

	if (!entry)
		return -ENODEV;

	buffer = kmalloc(count, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	leftover = copy_from_user(buffer, buf, count);
	if (leftover == 0) {
		retval = aoc_service_write(entry->service, buffer, count,
					   should_block);
	} else {
		retval = -ENOMEM;
	}

	kfree(buffer);
	return retval;
}

static unsigned int acd_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	struct acd_device_entry *entry = file->private_data;
	struct aoc_service_dev *service;
	bool acd_device_dead;

	acd_device_dead = entry->service->dead;

	if (acd_device_dead)
		return mask | POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM;

	service = entry->service;
	poll_wait(file, aoc_service_get_read_queue(service), wait);
	poll_wait(file, aoc_service_get_write_queue(service), wait);
	aoc_service_set_read_blocked(service);
	aoc_service_set_write_blocked(service);
	if (aoc_service_can_read(service))
		mask |= POLLIN | POLLRDNORM;
	if (aoc_service_can_write(service))
		mask |= POLLOUT | POLLWRNORM;

	return mask;
}

static int acd_probe(struct aoc_service_dev *dev)
{
	int ret;

	pr_debug("probe service with name %s\n", dev_name(&dev->dev));
	ret = create_character_device(dev);

	return ret;
}

static int acd_remove(struct aoc_service_dev *dev)
{
	struct acd_device_entry *entry;
	struct acd_device_entry *tmp;

	mutex_lock(&acd_devices_lock);
	list_for_each_entry_safe(entry, tmp, &acd_devices_list, list) {
		if (entry->acd_device->parent == &dev->dev) {
			pr_debug("remove service with name %s\n",
				 dev_name(&dev->dev));

			list_del_init(&entry->list);
			put_device(&entry->service->dev);
			device_destroy(acd_class, entry->acd_device->devt);
			kref_put(&entry->refcount, acd_device_entry_release);
			break;
		}
	}
	acd_next_minor = 0;
	mutex_unlock(&acd_devices_lock);

	return 0;
}

static void cleanup_resources(void)
{
	aoc_driver_unregister(&aoc_char_driver);

	if (acd_class) {
		class_destroy(acd_class);
		acd_class = NULL;
	}

	if (acd_major >= 0) {
		unregister_chrdev(acd_major, ACD_CHARDEV_NAME);
		acd_major = -1;
	}
}

static int __init acd_init(void)
{
	pr_debug("driver init\n");

	acd_major = register_chrdev(0, ACD_CHARDEV_NAME, &fops);
	if (acd_major < 0) {
		pr_err("Failed to register character major number\n");
		goto fail;
	}

	acd_major_dev = MKDEV(acd_major, 0);

	acd_class = class_create(THIS_MODULE, ACD_CHARDEV_NAME);
	if (!acd_class) {
		pr_err("Failed to create class\n");
		goto fail;
	}

	acd_class->devnode = acd_devnode;

	aoc_driver_register(&aoc_char_driver);

	return 0;

fail:
	cleanup_resources();
	return -1;
}

static void __exit acd_exit(void)
{
	pr_debug("driver exit\n");

	cleanup_resources();
}

module_init(acd_init);
module_exit(acd_exit);

MODULE_LICENSE("GPL v2");
