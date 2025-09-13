// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2019 Google LLC. All Rights Reserved.
 *
 * Channelized character IPC device interface for AoC services
 * AOCC - AoC Channelized Comms
 */

#define pr_fmt(fmt) "aoc_chan: " fmt

#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <uapi/linux/sched/types.h>

#include "aoc.h"
#include "uapi/aoc_channel_dev.h"

#define AOCC_CHARDEV_NAME "aoc_chan"

static int aocc_major = -1;
static int aocc_major_dev;
static unsigned int aocc_next_minor; /* protected by aocc_devices_lock */
static struct class *aocc_class;
static long received_msg_count = 0;
static long sent_msg_count = 0;

module_param(received_msg_count, long, S_IRUGO);
module_param(sent_msg_count, long, S_IRUGO);

struct chan_prvdata {
	struct wakeup_source *user_wakelock;
	struct task_struct *demux_task;
};

struct aocc_device_entry {
	struct device *aocc_device;
	struct aoc_service_dev *service;
	struct list_head list;
	struct kref refcount;
	int sh_mem_doorbell_count;
	bool sh_mem_doorbell_available;
	struct work_struct sh_mem_doorbell_work;
};

static LIST_HEAD(aocc_devices_list);
static DEFINE_MUTEX(aocc_devices_lock);
static DEFINE_MUTEX(aocc_write_lock);
static DEFINE_MUTEX(s_open_files_lock);

#define AOCC_MAX_MSG_SIZE 1024

static atomic_t channel_index_counter = ATOMIC_INIT(1);

static u32 aocc_max_pending_msgs;
static u32 aocc_block_channel_threshold;

/* Driver methods */
static int aocc_probe(struct aoc_service_dev *dev);
static int aocc_remove(struct aoc_service_dev *dev);

static const char * const channel_service_names[] = {
	"com.google.usf",
	"com.google.usf.non_wake_up",
	"com.google.chre",
	"com.google.chre.non_wake_up",
	"com.google.bt",
	"com.google.bt.non_wake_up",
	"usf_sh_mem_doorbell",
	NULL,
};

static struct aoc_driver aoc_chan_driver = {
	.drv = {
		.name = "aoc_chan",
	},
	.service_names = channel_service_names,
	.probe = aocc_probe,
	.remove = aocc_remove,
};

/* Message definitions. */
/* TODO: These should be synchronized with EFW source. b/140593553 */
/*
 * Maximum channel index for a 31-bit channel index value. It's set at half of
 * the actual maximum to avoid a race condition where two threads could pass the
 * maximum channel index check and then each allocate and increment the channel
 * index and push it over the maximum 31-bit value.
 */
#define AOCC_MAX_CHANNEL_INDEX ((1 << 30) - 1)
struct aoc_channel_message {
	uint32_t channel_index : 31;
	uint32_t non_wake_up : 1;
	char payload[AOCC_MAX_MSG_SIZE - sizeof(uint32_t)];
} __attribute__((packed));

struct aoc_message_node {
	struct list_head msg_list;
	size_t msg_size;
	union {
		char msg_buffer[AOCC_MAX_MSG_SIZE];
		struct aoc_channel_message msg;
	};
};

enum aoc_cmd_code {
	AOCC_CMD_OPEN_CHANNEL = 0,
	AOCC_CMD_CLOSE_CHANNEL,
	AOCC_CMD_BLOCK_CHANNEL,
	AOCC_CMD_UNBLOCK_CHANNEL,
	AOCC_CMD_SUSPEND_PREPARE,
	AOCC_CMD_WAKEUP_COMPELTE
};

struct aocc_channel_control_msg {
	int channel_index; /* Should always be 0 for CMD messages */
	int command_code;
	int channel_to_modify;
} __packed;

struct file_prvdata {
	struct aocc_device_entry *aocc_device_entry;
	int channel_index;
	wait_queue_head_t read_queue;
	struct list_head open_files_list;
	struct list_head pending_aoc_messages;
	struct mutex pending_msg_lock;
	atomic_t pending_msg_count;
	bool is_channel_blocked;
	int prev_sh_mem_doorbell_count;
	bool sh_mem_doorbell_enabled;
};

/* Globals */
/* TODO(b/141396548): Move these to drv_data. */
static LIST_HEAD(s_open_files);

/* Shared memory transport doorbell globals. */
/* TODO (b/184637825): Use mailbox device for AoC shared memory transport. */
static struct aoc_service_dev *sh_mem_doorbell_service_dev;
static struct aocc_device_entry* sh_mem_doorbell_channel_device;

/* Message related services */
static int aocc_send_cmd_msg(aoc_service *service_id, enum aoc_cmd_code code,
			      int channel_to_modify);

static int aocc_demux_kthread(void *data)
{
	ssize_t retval = 0;
	int rc = 0;
	struct aoc_service_dev *service = (struct aoc_service_dev *)data;
	struct chan_prvdata *service_prvdata = service->prvdata;

	dev_info(&(service->dev), "Demux handler started!");

	while (!kthread_should_stop()) {
		int handler_found = 0;
		int channel = 0;
		bool take_wake_lock = false;
		struct file_prvdata *entry;
		struct aoc_message_node *node =
			kmalloc(sizeof(struct aoc_message_node), GFP_KERNEL);

		INIT_LIST_HEAD(&node->msg_list);

		/* Attempt to read from the service, and block if we can't. */
		retval = aoc_service_read(service, node->msg_buffer,
					  AOCC_MAX_MSG_SIZE, true);

		if (retval < 0 || retval < sizeof(int)) {
			pr_err_ratelimited("Read failed with %ld", retval);
			kfree(node);

			if (retval == -ENODEV) {
				/*
				 * ENODEV indicates that the device is going
				 * away (most likely due to a firmware crash).
				 * At this point it is a race between the
				 * kthread and aocc_remove(), so we need to be
				 * careful to not miss the thread_stop event.
				 * By setting ourselves as INTERRUPTIBLE, that
				 * window is closed since a kthread_stop() will
				 * set this thread back to runnable before
				 * schedule is allowed to block.
				 */

				set_current_state(TASK_INTERRUPTIBLE);
				if (!kthread_should_stop())
					schedule();

				set_current_state(TASK_RUNNING);
			}

			continue;
		}

		received_msg_count++;
		node->msg_size = retval;
		channel = node->msg.channel_index;

		/* Find the open file with the correct matching ID. */
		mutex_lock(&s_open_files_lock);
		list_for_each_entry(entry, &s_open_files, open_files_list) {
			if (channel == entry->channel_index) {
				handler_found = 1;
				if (!node->msg.non_wake_up &&
				    (strcmp(dev_name(&service->dev),"com.google.usf") == 0 ||
				     strcmp(dev_name(&service->dev),"com.google.chre") == 0 ||
				     strcmp(dev_name(&service->dev),"com.google.bt") == 0)) {
					take_wake_lock = true;
				}

				if (atomic_read(&entry->pending_msg_count) >
				    aocc_max_pending_msgs) {
					pr_err_ratelimited(
						"Too many pending messages on channel %d.  More than %d allocated",
						channel, aocc_max_pending_msgs);
					kfree(node);
					break;
				}

				/* append message to the list of messages. */
				mutex_lock(&entry->pending_msg_lock);
				list_add_tail(&node->msg_list,
					      &entry->pending_aoc_messages);
				atomic_inc(&entry->pending_msg_count);
				if (atomic_read(&entry->pending_msg_count) >
				    aocc_block_channel_threshold &&
				    !entry->is_channel_blocked) {
					rc = aocc_send_cmd_msg(service,
						AOCC_CMD_BLOCK_CHANNEL, channel);
					if (rc >= 0)
						entry->is_channel_blocked = true;
				}
				mutex_unlock(&entry->pending_msg_lock);

				/* wake up anyone blocked on reading */
				wake_up(&entry->read_queue);
				break;
			}
		}
		mutex_unlock(&s_open_files_lock);

		/*
		 * If the message is "waking", take wakelock to allow userspace to dequeue
                 * the message.
		 */
		if (take_wake_lock) {
			pm_wakeup_ws_event(service_prvdata->user_wakelock, 200, true);
		}

		if (!handler_found) {
			pr_warn_ratelimited("Could not find handler for channel %d",
					    channel);
			/* Notifies AOC the channel is closed. */
			aocc_send_cmd_msg(service, AOCC_CMD_CLOSE_CHANNEL, channel);
			kfree(node);
			continue;
		}
	}

	return 0;
}

static void sh_mem_doorbell_service_handler(struct aoc_service_dev *dev)
{
	struct aocc_device_entry *aocc_device =
		(struct aocc_device_entry*) dev->prvdata;
	schedule_work(&(aocc_device->sh_mem_doorbell_work));
}

static void aocc_sh_mem_handle_doorbell(struct work_struct *work)
{
	struct file_prvdata *entry;
	struct aocc_device_entry *aocc_device =
		container_of(work, struct aocc_device_entry, sh_mem_doorbell_work);

	/* One more doorbell. */
	aocc_device->sh_mem_doorbell_count++;

	/*
	 * Wake up all read queue waiters that have the shared memory transport
	 * doorbell enabled.
	 */
	mutex_lock(&s_open_files_lock);
	list_for_each_entry(entry, &s_open_files, open_files_list) {
		if (entry->sh_mem_doorbell_enabled) {
			wake_up(&entry->read_queue);
		}
	}
	mutex_unlock(&s_open_files_lock);
}

static int aocc_send_cmd_msg(aoc_service *service_id, enum aoc_cmd_code code,
			      int channel_to_modify)
{
	struct aocc_channel_control_msg msg;
	int ret;

	msg.channel_index = 0;
	msg.command_code = code;
	msg.channel_to_modify = channel_to_modify;

	mutex_lock(&aocc_write_lock);
	ret = aoc_service_write(service_id, (char *)&msg, sizeof(msg), false);
	mutex_unlock(&aocc_write_lock);
	return ret;
}

/* File methods */
static int aocc_open(struct inode *inode, struct file *file);
static int aocc_release(struct inode *inode, struct file *file);
static ssize_t aocc_read(struct file *file, char __user *buf, size_t count,
			 loff_t *off);
static ssize_t aocc_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *off);
static unsigned int aocc_poll(struct file *file, poll_table *wait);
static long aocc_unlocked_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg);

static const struct file_operations fops = {
	.open = aocc_open,
	.release = aocc_release,
	.read = aocc_read,
	.write = aocc_write,
	.poll = aocc_poll,
	.unlocked_ioctl = aocc_unlocked_ioctl,

	.owner = THIS_MODULE,
};

static void aocc_device_entry_release(struct kref *ref)
{
	kfree(container_of(ref, struct aocc_device_entry, refcount));
}

/*
 * Caller must hold aocc_devices_lock.
 */
static struct aocc_device_entry *aocc_device_entry_for_inode(struct inode *inode)
{
	struct aocc_device_entry *entry;

	list_for_each_entry(entry, &aocc_devices_list, list) {
		/* entries with service->dead == true can't be in aocc_devices_list */
		if (entry->aocc_device->devt == inode->i_rdev)
			return entry;
	}

	return NULL;
}

static char *aocc_devnode(struct device *dev, umode_t *mode)
{
	if (!mode || !dev)
		return NULL;

	if (dev->devt == aocc_major_dev)
		*mode = 0666;

	return NULL;
}

static int create_character_device(struct aoc_service_dev *dev)
{
	int rc = 0;
	struct aocc_device_entry *new_entry;

	new_entry = kmalloc(sizeof(*new_entry), GFP_KERNEL);
	if (!new_entry) {
		rc = -ENOMEM;
		goto err_kmalloc;
	}

	mutex_lock(&aocc_devices_lock);
	new_entry->aocc_device = device_create(aocc_class, &dev->dev,
					       MKDEV(aocc_major, aocc_next_minor), NULL,
					       "acd-%s", dev_name(&dev->dev));
	if (IS_ERR(new_entry->aocc_device)) {
		pr_err("device_create failed: %ld\n", PTR_ERR(new_entry->aocc_device));
		rc = PTR_ERR(new_entry->aocc_device);
		goto err_device_create;
	}
	get_device(&dev->dev);
	new_entry->service = dev;
	aocc_next_minor++;
	kref_init(&new_entry->refcount);
	list_add(&new_entry->list, &aocc_devices_list);
	mutex_unlock(&aocc_devices_lock);
	return 0;

err_device_create:
	mutex_unlock(&aocc_devices_lock);
	kfree(new_entry);
err_kmalloc:
	return rc;
}

static int aocc_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	struct file_prvdata *prvdata;
	struct aocc_device_entry *entry;

	pr_debug("attempt to open major:%d minor:%d\n", MAJOR(inode->i_rdev),
		 MINOR(inode->i_rdev));

	prvdata = kmalloc(sizeof(struct file_prvdata), GFP_KERNEL);
	if (!prvdata) {
		rc = -ENOMEM;
		goto err_kmalloc;
	}
	mutex_init(&prvdata->pending_msg_lock);
	prvdata->prev_sh_mem_doorbell_count = 0;
	prvdata->sh_mem_doorbell_enabled = false;

	mutex_lock(&aocc_devices_lock);
	entry = aocc_device_entry_for_inode(inode);
	if (!entry) {
		rc = -ENODEV;
		mutex_unlock(&aocc_devices_lock);
		goto err_aocc_device_entry;
	}

	/* Check if our simple allocation scheme has overflowed */
	if (atomic_read(&channel_index_counter) >= AOCC_MAX_CHANNEL_INDEX) {
		pr_err("Too many channels have been opened.");
		rc = -EMFILE;
		mutex_unlock(&aocc_devices_lock);
		goto err_channel_index_counter;
	}

	kref_get(&entry->refcount);
	get_device(&entry->service->dev);
	prvdata->aocc_device_entry = entry;
	file->private_data = prvdata;
	mutex_unlock(&aocc_devices_lock);

	/* Allocate a unique index to represent this open file. */
	prvdata->channel_index = atomic_inc_return(&channel_index_counter);
	dev_info(&(entry->service->dev), "New client with channel ID %d", prvdata->channel_index);

	/* Start a new empty message list for this channel's message queue. */
	INIT_LIST_HEAD(&prvdata->pending_aoc_messages);
	atomic_set(&prvdata->pending_msg_count, 0);

	init_waitqueue_head(&prvdata->read_queue);

	/* Add this item to the open files list for the dispatcher thread. */
	mutex_lock(&s_open_files_lock);
	INIT_LIST_HEAD(&prvdata->open_files_list);
	list_add(&prvdata->open_files_list, &s_open_files);
	mutex_unlock(&s_open_files_lock);

	/*Send a message to AOC to register a new channel. */
	rc = aocc_send_cmd_msg(prvdata->aocc_device_entry->service,
			  AOCC_CMD_OPEN_CHANNEL, prvdata->channel_index);
	if (rc < 0) {
		pr_err("send AOCC_CMD_OPEN_CHANNEL fail %d", rc);
		goto err_send_cmd_msg;
	}

	return 0;

err_send_cmd_msg:
	mutex_lock(&s_open_files_lock);
	list_del(&prvdata->open_files_list);
	mutex_unlock(&s_open_files_lock);

	mutex_lock(&aocc_devices_lock);
	put_device(&entry->service->dev);
	kref_put(&entry->refcount, aocc_device_entry_release);
	mutex_unlock(&aocc_devices_lock);
err_channel_index_counter:
err_aocc_device_entry:
	kfree(prvdata);
err_kmalloc:
	return rc;
}

static int aocc_release(struct inode *inode, struct file *file)
{
	struct file_prvdata *private = file->private_data;
	struct aoc_message_node *entry;
	struct aoc_message_node *temp;
	int scrapped = 0;
	bool aocc_device_dead;

	if (!private)
		return -ENODEV;

	mutex_lock(&aocc_devices_lock);
	aocc_device_dead = private->aocc_device_entry->service->dead;
	mutex_unlock(&aocc_devices_lock);

	/*Remove this file from from the list of active channels. */
	mutex_lock(&s_open_files_lock);
	list_del(&private->open_files_list);
	mutex_unlock(&s_open_files_lock);

	/*Clear all pending messages. */
	mutex_lock(&private->pending_msg_lock);
	list_for_each_entry_safe(entry, temp, &private->pending_aoc_messages,
				 msg_list) {
		kfree(entry);
		scrapped++;
		atomic_dec(&private->pending_msg_count);
	}
	mutex_unlock(&private->pending_msg_lock);

	if (!aocc_device_dead) {
		/*Send a message to AOC to close the channel. */
		aocc_send_cmd_msg(private->aocc_device_entry->service, AOCC_CMD_CLOSE_CHANNEL,
				  private->channel_index);
	}

	if (scrapped)
		pr_warn("Destroyed channel %d with %d unread messages",
			private->channel_index, scrapped);
	else
		pr_debug("Destroyed channel %d with no unread messages",
			 private->channel_index);

	put_device(&private->aocc_device_entry->service->dev);
	kref_put(&private->aocc_device_entry->refcount, aocc_device_entry_release);
	mutex_destroy(&private->pending_msg_lock);
	kfree(private);
	file->private_data = NULL;

	return 0;
}

static bool aocc_are_messages_pending(struct file_prvdata *private)
{
	bool retval = !!atomic_read(&private->pending_msg_count);
	return retval;
}

static ssize_t aocc_read(struct file *file, char __user *buf, size_t count,
			 loff_t *off)
{
	struct file_prvdata *private = file->private_data;
	struct aoc_message_node *node = NULL;
	ssize_t retval = 0;
	bool aocc_device_dead;
	int rc = 0;

	mutex_lock(&aocc_devices_lock);
	aocc_device_dead = private->aocc_device_entry->service->dead;
	mutex_unlock(&aocc_devices_lock);

	if (aocc_device_dead)
		return -ESHUTDOWN;

	/*Block while there are no messages pending. */
	while (!aocc_are_messages_pending(private)) {
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}

		retval = wait_event_interruptible(
			private->read_queue,
			aocc_are_messages_pending(private));

		if (retval == -ERESTARTSYS) {
			/* If the wait was interrupted, we are probably
			 * being killed. Quit.
			 */
			return -EINTR;
		}
	}

	/* pop pending message from the list. */
	mutex_lock(&private->pending_msg_lock);
	node = list_first_entry_or_null(&private->pending_aoc_messages,
					struct aoc_message_node, msg_list);
	mutex_unlock(&private->pending_msg_lock);

	if (!node) {
		pr_err("No messages available.");
		return retval;
	}

	/* is message too big to fit into read buffer? */
	if (count < (node->msg_size - sizeof(int))) {
		pr_err("Message size %zu bytes, read size %zu", node->msg_size,
		       count);
		node->msg_size = count + sizeof(int);
	}

	/* copy message payload to userspace, minus the channel ID */
	retval = copy_to_user(buf, node->msg.payload, node->msg_size - sizeof(uint32_t));

	/* copy_to_user returns bytes that couldn't be copied */
	retval = node->msg_size - retval;

	mutex_lock(&private->pending_msg_lock);
	list_del(&node->msg_list);
	atomic_dec(&private->pending_msg_count);
	if (atomic_read(&private->pending_msg_count) <
	    aocc_block_channel_threshold && private->is_channel_blocked) {
		rc = aocc_send_cmd_msg(private->aocc_device_entry->service,
				       AOCC_CMD_UNBLOCK_CHANNEL, private->channel_index);
		if (rc >= 0)
			private->is_channel_blocked = false;
	}
	mutex_unlock(&private->pending_msg_lock);

	kfree(node);

	return retval;
}

static ssize_t aocc_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *off)
{
	struct file_prvdata *private = file->private_data;
	char *buffer;
	size_t leftover;
	ssize_t retval = 0;
	bool should_block = ((file->f_flags & O_NONBLOCK) == 0);
	bool aocc_device_dead;

	if (!private)
		return -ENODEV;

	buffer = kmalloc(count + sizeof(int), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	mutex_lock(&aocc_devices_lock);
	aocc_device_dead = private->aocc_device_entry->service->dead;
	mutex_unlock(&aocc_devices_lock);

	if (aocc_device_dead) {
		retval = -ESHUTDOWN;
		goto err_aocc_device_dead;
	}

	/*Prepend the appropriate channel index to the message. */
	((int *)buffer)[0] = private->channel_index;

	leftover = copy_from_user(buffer + sizeof(int), buf, count);
	if (leftover == 0) {
		mutex_lock(&aocc_write_lock);
		retval = aoc_service_write(private->aocc_device_entry->service, buffer,
					   count + sizeof(int), should_block);
		mutex_unlock(&aocc_write_lock);
		if (retval > 0)
			sent_msg_count++;
	} else {
		retval = -ENOMEM;
	}

err_aocc_device_dead:
	if (retval < 0 && retval != -EAGAIN) {
		pr_err("Write failed for channel %d with code %zd\n", private->channel_index, retval);
	}

	kfree(buffer);
	return retval;
}

static unsigned int aocc_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	struct file_prvdata *private = file->private_data;
	bool aocc_device_dead;

	mutex_lock(&aocc_devices_lock);
	aocc_device_dead = private->aocc_device_entry->service->dead;
	mutex_unlock(&aocc_devices_lock);

	if (aocc_device_dead)
		return mask | POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM;

	poll_wait(file, &private->read_queue, wait);

	if (aocc_are_messages_pending(private)) {
		mask |= POLLIN | POLLRDNORM;
	}

	/*
	 * If the shared memory doorbell is enabled and the doorbell has been
	 * rung, indicate that data may be read.
	 */
	if (private->sh_mem_doorbell_enabled &&
	    (private->prev_sh_mem_doorbell_count != private->aocc_device_entry->sh_mem_doorbell_count)) {
		mask |= POLLIN | POLLRDNORM;
		private->prev_sh_mem_doorbell_count = private->aocc_device_entry->sh_mem_doorbell_count;
	}

	return mask;
}

static long aocc_unlocked_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct file_prvdata *private = file->private_data;
	long rv = 0;

	switch (cmd) {
	case AOCC_IOCTL_ENABLE_SH_MEM_DOORBELL:
		if (!private->aocc_device_entry->sh_mem_doorbell_available) {
			rv = -ENOSYS;
			break;
		}
		private->sh_mem_doorbell_enabled = true;
	break;

	default:
		/*
		 * ioctl(2) The specified request does not apply to the kind of
		 * object that the file descriptor fd references
		 */
		pr_err("Received IOCTL with invalid ID (%d) returning ENOTTY",
		       cmd);
		rv = -ENOTTY;
		break;
	}

	return rv;
}

static void aocc_sh_mem_doorbell_probe(struct aoc_service_dev *dev)
{
	/* Get the shared memory doorbell service device. */
	if (strcmp(dev_name(&dev->dev), "usf_sh_mem_doorbell") == 0) {
		sh_mem_doorbell_service_dev = dev;
	}

	/* Use the first channel device with the shared memory doorbell. */
	sh_mem_doorbell_channel_device =
		list_entry(aocc_devices_list.next, struct aocc_device_entry, list);

	/*
	 * If both a shared memory doorbell service device and a channel device
	 * are available, initialize the shared memory doorbell.
	 */
	if ((sh_mem_doorbell_channel_device != NULL) &&
	    (sh_mem_doorbell_service_dev != NULL)) {
		/* Install the shared memory doorbell service handler. */
		sh_mem_doorbell_service_dev->handler =
			sh_mem_doorbell_service_handler;
		sh_mem_doorbell_service_dev->prvdata = sh_mem_doorbell_channel_device;

		/* Initialize the shared memory transport doorbell work task. */
		INIT_WORK(&(sh_mem_doorbell_channel_device->sh_mem_doorbell_work),
			  aocc_sh_mem_handle_doorbell);

		/*
		 * Initialize the shared memory doorbell count and mark the
		 * doorbell as available.
		 */
		sh_mem_doorbell_channel_device->sh_mem_doorbell_count = 0;
		sh_mem_doorbell_channel_device->sh_mem_doorbell_available = true;
	}
}

static int aocc_probe(struct aoc_service_dev *dev)
{
	struct chan_prvdata *prvdata;
	int ret = 0;
	struct sched_param param = {
		.sched_priority = 10,
	};

	pr_notice("probe service with name %s\n", dev_name(&dev->dev));

	prvdata = devm_kzalloc(&dev->dev, sizeof(*prvdata), GFP_KERNEL);
	if (!prvdata)
		return -ENOMEM;

	aocc_max_pending_msgs = dt_property(dev->dev.parent->of_node, "channel-max-pending-msgs");
	if (aocc_max_pending_msgs == DT_PROPERTY_NOT_FOUND) {
		dev_err(&dev->dev, "AOC DT missing property channel-max-pending-msgs");
		return -EINVAL;
	}
	aocc_block_channel_threshold = dt_property(dev->dev.parent->of_node,
						"block-channel-threshold");
	if (aocc_block_channel_threshold == DT_PROPERTY_NOT_FOUND) {
		dev_err(&dev->dev, "AOC DT missing property block_channel_threshold");
		return -EINVAL;
	}

	if (strcmp(dev_name(&dev->dev), "usf_sh_mem_doorbell") != 0) {
		ret = create_character_device(dev);
		if (ret)
			return ret;
		prvdata->user_wakelock = wakeup_source_register(&dev->dev, dev_name(&dev->dev));
		dev->prvdata = prvdata;
		prvdata->demux_task =  kthread_run(&aocc_demux_kthread, dev, dev_name(&dev->dev));
		sched_setscheduler(prvdata->demux_task, SCHED_FIFO, &param);
		if (IS_ERR(prvdata->demux_task))
			ret = PTR_ERR(prvdata->demux_task);
	}

	aocc_sh_mem_doorbell_probe(dev);

	return ret;
}

static int aocc_remove(struct aoc_service_dev *dev)
{
	struct aocc_device_entry *entry;
	struct aocc_device_entry *tmp;
	struct chan_prvdata *prvdata;

	/* Uninstall the shared memory doorbell service. */
	if (dev == sh_mem_doorbell_service_dev) {
		sh_mem_doorbell_service_dev->handler = NULL;
		sh_mem_doorbell_service_dev = NULL;
	} else {
		prvdata = dev->prvdata;
		kthread_stop(prvdata->demux_task);
		if (prvdata->user_wakelock) {
			wakeup_source_unregister(prvdata->user_wakelock);
			prvdata->user_wakelock = NULL;
		}
	}

	mutex_lock(&aocc_devices_lock);
	list_for_each_entry_safe(entry, tmp, &aocc_devices_list, list) {
		if (entry->aocc_device->parent == &dev->dev) {
			pr_debug("remove service with name %s\n",
				 dev_name(&dev->dev));

			/* Disable shared memory transport doorbell. */
			if (entry == sh_mem_doorbell_channel_device) {
				sh_mem_doorbell_channel_device = NULL;
			}
			entry->sh_mem_doorbell_available = false;

			list_del_init(&entry->list);
			put_device(&entry->service->dev);
			device_destroy(aocc_class, entry->aocc_device->devt);
			kref_put(&entry->refcount, aocc_device_entry_release);
			break;
		}
	}
	aocc_next_minor = 0;
	mutex_unlock(&aocc_devices_lock);

	return 0;
}

static void cleanup_resources(void)
{
	aoc_driver_unregister(&aoc_chan_driver);

	if (aocc_class) {
		class_destroy(aocc_class);
		aocc_class = NULL;
	}

	if (aocc_major >= 0) {
		unregister_chrdev(aocc_major, AOCC_CHARDEV_NAME);
		aocc_major = -1;
	}
}

static int aocc_prepare(struct device *dev)
{
	struct device *parent = dev->parent;
	struct aoc_service_dev *service = container_of(parent, struct aoc_service_dev, dev);
	int rc;

	if (strcmp(dev_name(&service->dev), "com.google.usf") != 0)
		return 0;

	rc = aocc_send_cmd_msg(service, AOCC_CMD_SUSPEND_PREPARE, 0);
	if (rc < 0)
		dev_err(dev, "failed to send suspend message: %d\n", rc);

	return 0;
}

static void aocc_complete(struct device *dev)
{
	struct device *parent = dev->parent;
	struct aoc_service_dev *service = container_of(parent, struct aoc_service_dev, dev);
	int rc;

	if (strcmp(dev_name(&service->dev), "com.google.usf") != 0)
		return;

	rc = aocc_send_cmd_msg(service, AOCC_CMD_WAKEUP_COMPELTE, 0);
	if (rc < 0)
		dev_err(dev, "failed to send resume message: %d\n", rc);
}

static const struct dev_pm_ops aocc_pm_ops = {
	.prepare = aocc_prepare,
	.complete = aocc_complete,
};

static int __init aocc_init(void)
{
	pr_debug("driver init\n");

	aocc_major = register_chrdev(0, AOCC_CHARDEV_NAME, &fops);
	if (aocc_major < 0) {
		pr_err("Failed to register character major number\n");
		goto fail;
	}

	aocc_major_dev = MKDEV(aocc_major, 0);

	aocc_class = class_create(THIS_MODULE, AOCC_CHARDEV_NAME);
	if (!aocc_class) {
		pr_err("Failed to create class\n");
		goto fail;
	}

	aocc_class->devnode = aocc_devnode;
	aocc_class->pm = &aocc_pm_ops;

	aoc_driver_register(&aoc_chan_driver);

	return 0;

fail:
	cleanup_resources();
	return -1;
}

static void __exit aocc_exit(void)
{
	pr_debug("driver exit\n");

	cleanup_resources();
}

module_init(aocc_init);
module_exit(aocc_exit);

MODULE_LICENSE("GPL v2");
