// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/poll.h>

#include "touch_offload.h"

static int touch_offload_open(struct inode *inode, struct file *file)
{
	struct touch_offload_context *context =
		container_of(inode->i_cdev, struct touch_offload_context, dev);

	file->private_data = context;
	pr_debug("%s\n", __func__);

	mutex_lock(&context->file_lock);
	if (context->file_in_use) {
		mutex_unlock(&context->file_lock);
		return -EBUSY;
	}
	context->file_in_use = true;
	mutex_unlock(&context->file_lock);

	/* Prepare context for consumer. E.g., event queue */
	return 0;
}

static int touch_offload_release(struct inode *inode, struct file *file)
{
	struct touch_offload_context *context = file->private_data;

	mutex_lock(&context->file_lock);
	context->file_in_use = false;
	mutex_unlock(&context->file_lock);

	pr_debug("%s\n", __func__);
	return 0;
}

static int pack_frame(struct touch_offload_context *context,
		      struct touch_offload_frame *frame, u8 **packed)
{
	u8 *ptr = context->prealloc_packed_frame;
	int channel_size;
	int i = 0;

	if (!ptr || !frame || frame->num_channels > MAX_CHANNELS)
		return -EINVAL;

	/* Copy the header */
	memcpy(ptr, &frame->header, sizeof(frame->header));
	ptr += sizeof(frame->header);

	/* Copy frame data */
	for (i = 0; i < frame->num_channels; i++) {
		if (frame->channel_type[i] == TOUCH_DATA_TYPE_COORD)
			channel_size = TOUCH_OFFLOAD_FRAME_SIZE_COORD;
		else if ((frame->channel_type[i] & TOUCH_SCAN_TYPE_MUTUAL) != 0)
			channel_size =
			    TOUCH_OFFLOAD_FRAME_SIZE_2D(context->caps.heatmap_height,
							context->caps.heatmap_width);
		else if ((frame->channel_type[i] & TOUCH_SCAN_TYPE_SELF) != 0)
			channel_size =
			    TOUCH_OFFLOAD_FRAME_SIZE_1D(context->caps.heatmap_height,
							context->caps.heatmap_width);
		else if (frame->channel_type[i] ==
				CONTEXT_CHANNEL_TYPE_DRIVER_STATUS)
			channel_size = TOUCH_OFFLOAD_FRAME_SIZE_DRIVER_STATUS;
		else if (frame->channel_type[i] ==
				CONTEXT_CHANNEL_TYPE_STYLUS_STATUS)
			channel_size = TOUCH_OFFLOAD_FRAME_SIZE_STYLUS_STATUS;
		else {
			pr_err("%s: Invalid channel_type = 0x%08X", __func__,
			       frame->channel_type[i]);
			return -EINVAL;
		}
		memcpy(ptr, frame->channel_data[i], channel_size);
		ptr += channel_size;
	}

	*packed = context->prealloc_packed_frame;
	return (ptr - *packed);
}

static ssize_t touch_offload_read(struct file *file, char __user *user_buffer,
				  size_t size, loff_t *offset)
{
	struct touch_offload_context *context = file->private_data;
	struct touch_offload_frame *frame;
	size_t copy_size;
	int result;
	unsigned long remaining;

	pr_debug("%s\n", __func__);

	if (context->num_buffers == 0)
		return -EINVAL;

	/* Block until touch events occur */
	/* Block (on completion?) until len >= 0. */
	/* Lock the event buffer */
	/* Copy contents of event buffer to service */
	/* Reset completion since len=0 */
	/* Unlock and return */

	/* If the end of the data is reached, free the packed_frame and
	 * return 0
	 */

	mutex_lock(&context->buffer_lock);

	if (context->packed_frame != NULL &&
	    *offset == context->packed_frame_size) {
		pr_err("%s: [Unexpected!] The buffer should have been recycled after the previous read.\n",
		       __func__);
		context->packed_frame = NULL;
		*offset = 0;
		mutex_unlock(&context->buffer_lock);
		return 0;
	} else if (context->packed_frame == NULL) {
		/* Process the next queued buffer */

		while (list_empty(&context->frame_queue)) {
			/* Presumably more buffers on the way, so block */
			mutex_unlock(&context->buffer_lock);

			/* Non-blocking read? */
			if (file->f_flags & O_NONBLOCK)
				return -EAGAIN;

			/* Block until data is available */
			if (wait_event_interruptible(context->read_queue,
					!list_empty(&context->frame_queue)))
				return -ERESTARTSYS;

			/* Check that the pipeline is still running */
			mutex_lock(&context->buffer_lock);
		}

		frame = list_entry(context->frame_queue.next,
				   struct touch_offload_frame, entry);
		list_del(&frame->entry);

		result = pack_frame(context, frame, &context->packed_frame);
		list_add_tail(&frame->entry, &context->free_pool);
		if (result <= 0) {
			pr_err("%s: Error packing frame! Result = %d.\n",
			       __func__, result);
			mutex_unlock(&context->buffer_lock);
			return -EINVAL;
		}
		if (result != context->packed_frame_size) {
			pr_err("%s: Packed frame size (%d) does not match size allocated per frame(%d)!\n",
			       __func__, result, context->packed_frame_size);
		}

	}

	/* Transfer the maximum amount of data */
	copy_size = min((long long)size, context->packed_frame_size - *offset);
	remaining = copy_to_user(user_buffer, context->packed_frame + *offset,
			      copy_size);
	if (remaining != 0)
		pr_err("%s: copy_to_user unexpectedly failed to copy %lu bytes.\n",
		       __func__, remaining);
	*offset += copy_size - remaining;

	/* Recycle the frame if transfer was complete */
	if (*offset == context->packed_frame_size) {
		context->packed_frame = NULL;
		*offset = 0;
	}

	mutex_unlock(&context->buffer_lock);

	return copy_size - remaining;
}

static unsigned int touch_offload_poll(struct file *file,
				       struct poll_table_struct *wait)
{
	struct touch_offload_context *context = file->private_data;
	unsigned int flags = 0;

	pr_debug("%s\n", __func__);

	poll_wait(file, &context->read_queue, wait);

	mutex_lock(&context->buffer_lock);

	if (context->packed_frame || !list_empty(&context->frame_queue))
		flags |= POLLIN | POLLRDNORM;

	mutex_unlock(&context->buffer_lock);
	return flags;
}

static int touch_offload_allocate_buffers(struct touch_offload_context *context,
					  int nb)
{
	struct touch_offload_frame *frame = NULL;
	int i;
	int num_channels;
	__u32 mask;
	__u32 size = 0;
	int max_packed_frame_size =
		sizeof(struct touch_offload_frame) +
		TOUCH_OFFLOAD_DATA_SIZE_2D(context->caps.heatmap_height,
					   context->caps.heatmap_width) *
		MAX_CHANNELS;

	pr_debug("%s\n", __func__);

	num_channels = (context->config.read_coords ? 1 : 0) +
		       hweight_long(context->config.mutual_data_types) +
		       hweight_long(context->config.self_data_types) +
		       hweight_long(context->config.context_channel_types);
	if (num_channels == 0 || num_channels > MAX_CHANNELS) {
		pr_err("%s: Configuration enables more (%d) than %d channels!\n",
		       __func__, num_channels, MAX_CHANNELS);
		return -EINVAL;
	}


	mutex_lock(&context->buffer_lock);
	context->prealloc_packed_frame =
		devm_kzalloc(context->device, max_packed_frame_size, GFP_KERNEL);
	if (context->prealloc_packed_frame == NULL) {
		mutex_unlock(&context->buffer_lock);
		return -ENOMEM;
	}
	context->prealloc_packed_frame_size = max_packed_frame_size;

	/* Add new buffers to the free_pool */
	for (i = 0; i < nb; i++) {
		int chan = 0;
		struct touch_offload_frame *frame =
		    kzalloc(sizeof(struct touch_offload_frame), GFP_KERNEL);

		if (frame == NULL) {
			mutex_unlock(&context->buffer_lock);
			return -ENOMEM;
		}

		frame->header.frame_size = sizeof(frame->header);

		/* Allocate component buffers */

		if (context->config.read_coords) {
			struct TouchOffloadDataCoord *coord;

			frame->channel_type[chan] = TOUCH_DATA_TYPE_COORD;
			size = TOUCH_OFFLOAD_FRAME_SIZE_COORD;
			frame->channel_data[chan] = kzalloc(size,
							       GFP_KERNEL);
			if (frame->channel_data[chan] == NULL)
				goto kzalloc_channel_fail;
			coord = (struct TouchOffloadDataCoord *)
					frame->channel_data[chan];
			coord->header.channel_type = TOUCH_DATA_TYPE_COORD;
			coord->header.channel_size = size;
			frame->channel_data_size[chan] = size;
			frame->header.frame_size += size;
			chan++;
		}
		for (mask = 0x01; mask < 0x100; mask <<= 1) {
			if ((context->config.mutual_data_types & mask) != 0) {
				struct TouchOffloadData2d *data;

				frame->channel_type[chan] =
				    TOUCH_SCAN_TYPE_MUTUAL | mask;
				size = TOUCH_OFFLOAD_FRAME_SIZE_2D(
						context->caps.heatmap_height,
						context->caps.heatmap_width);
				frame->channel_data[chan] = kzalloc(size,
								    GFP_KERNEL);
				if (frame->channel_data[chan] == NULL)
					goto kzalloc_channel_fail;
				data = (struct TouchOffloadData2d *)
						frame->channel_data[chan];
				data->header.channel_type =
				    TOUCH_SCAN_TYPE_MUTUAL | mask;
				data->header.channel_size = size;
				frame->channel_data_size[chan] = size;
				frame->header.frame_size += size;
				chan++;
			}
		}
		for (mask = 0x01; mask < 0x100; mask <<= 1) {
			if ((context->config.self_data_types & mask) != 0) {
				struct TouchOffloadData1d *data;

				frame->channel_type[chan] =
					TOUCH_SCAN_TYPE_SELF | mask;
				size = TOUCH_OFFLOAD_FRAME_SIZE_1D(
						context->caps.heatmap_height,
						context->caps.heatmap_width);
				frame->channel_data[chan] = kzalloc(size,
								GFP_KERNEL);
				if (frame->channel_data[chan] == NULL)
					goto kzalloc_channel_fail;
				data = (struct TouchOffloadData1d *)
						frame->channel_data[chan];
				data->header.channel_type =
				    TOUCH_SCAN_TYPE_SELF | mask;
				data->header.channel_size = size;
				frame->channel_data_size[chan] = size;
				frame->header.frame_size += size;
				chan++;
			}
		}
		for (mask = CONTEXT_CHANNEL_BIT_START;
		     mask <= CONTEXT_CHANNEL_BIT_END;
		     mask <<= 1) {
			struct TouchOffloadChannelHeader *chan_header;

			if (!(context->config.context_channel_types & mask))
				continue;

			frame->channel_type[chan] = (__u32)mask;
			size = 0;
			switch (mask) {
			case CONTEXT_CHANNEL_TYPE_DRIVER_STATUS:
				size =
				    TOUCH_OFFLOAD_FRAME_SIZE_DRIVER_STATUS;
				break;
			case CONTEXT_CHANNEL_TYPE_STYLUS_STATUS:
				size =
				    TOUCH_OFFLOAD_FRAME_SIZE_STYLUS_STATUS;
				break;
			default:
				pr_err("%s: Invalid channel_type = 0x%08X\n",
					__func__, mask);
				goto invalid_context_channel;
			}
			frame->channel_data[chan] = kzalloc(size,
							GFP_KERNEL);
			if (frame->channel_data[chan] == NULL)
				goto kzalloc_channel_fail;

			chan_header =
				(struct TouchOffloadChannelHeader *)
					frame->channel_data[chan];
			chan_header->channel_type = (__u32)mask;
			chan_header->channel_size = size;
			frame->channel_data_size[chan] = size;
			frame->header.frame_size += size;
			chan++;
		}

		frame->num_channels = chan;
		frame->header.num_channels = chan;

		if (context->packed_frame_size == 0)
			context->packed_frame_size = frame->header.frame_size;
		if (context->packed_frame_size != frame->header.frame_size)
			pr_err("%s: Frame size mismatch! %d != %d.\n", __func__,
			       context->packed_frame_size,
			       frame->header.frame_size);

		list_add_tail(&frame->entry, &context->free_pool);
		context->num_buffers++;
	}

	mutex_unlock(&context->buffer_lock);
	return 0;

invalid_context_channel:
kzalloc_channel_fail:
	/* Free all channels of "frame" before returning */
	if (frame)
		for (i = 0; i < MAX_CHANNELS; i++)
			kfree(frame->channel_data[i]);
	kfree(frame);

	mutex_unlock(&context->buffer_lock);
	return -ENOMEM;
}

static int touch_offload_free_buffers(struct touch_offload_context *context)
{
	int freed = 0;
	int chan = 0;

	pr_debug("%s\n", __func__);

	mutex_lock(&context->buffer_lock);

	/* Ensure there is no outstanding reserved_frame before continuing */
	while (context->reserved_frame != NULL) {
		mutex_unlock(&context->buffer_lock);
		wait_for_completion(&context->reserve_returned);
		mutex_lock(&context->buffer_lock);
	}

	if (context->num_buffers > 0) {
		while (!list_empty(&context->free_pool)) {
			struct list_head *next = context->free_pool.next;
			struct touch_offload_frame *frame =
			    list_entry(next, struct touch_offload_frame, entry);

			list_del(next);
			for (chan = 0; chan < frame->num_channels; chan++)
				kfree(frame->channel_data[chan]);
			kfree(frame);
			freed++;
		}

		while (!list_empty(&context->frame_queue)) {
			struct list_head *next = context->frame_queue.next;
			struct touch_offload_frame *frame =
			    list_entry(next, struct touch_offload_frame, entry);

			list_del(next);
			for (chan = 0; chan < frame->num_channels; chan++)
				kfree(frame->channel_data[chan]);
			kfree(frame);
			freed++;
		}
	}
	if (freed != context->num_buffers)
		pr_err("%s: mismatch between the number of buffers allocated(%d) and freed(%d)!",
		       __func__, context->num_buffers, freed);

	/* clean up */
	context->num_buffers = 0;
	context->reserved_frame = NULL;
	INIT_LIST_HEAD(&context->free_pool);
	INIT_LIST_HEAD(&context->frame_queue);
	context->packed_frame_size = 0;

	mutex_unlock(&context->buffer_lock);

	return 0;
}

static long touch_offload_ioctl(struct file *file, unsigned int ioctl_num,
				unsigned long ioctl_param)
{
	struct touch_offload_context *context = file->private_data;
	unsigned long err = 0;

	pr_debug("%s: ioctl_num=0x%08X, ioctl_param=0x%08lX\n", __func__,
		 ioctl_num, ioctl_param);

	switch (ioctl_num) {
	case TOUCH_OFFLOAD_IOC_RD_GETCAPS:
	{
		struct TouchOffloadIocGetCaps getCaps;

		/* Copy previously-populated caps */
		memcpy(&getCaps.caps, &context->caps,
		       sizeof(context->caps));

		err = copy_to_user((void *)ioctl_param, &getCaps,
				   sizeof(getCaps));
		if (err != 0) {
			pr_err("%s: copy_to_failed with err=0x%08lX",
			       __func__, err);
			return err;
		}
		break;
	}

	case TOUCH_OFFLOAD_IOC_WR_CONFIGURE:
	{
		struct TouchOffloadIocConfigure configure;
		int num_channels;

		err = copy_from_user(&configure, (void *)ioctl_param,
				     sizeof(configure));
		if (err != 0) {
			pr_err("%s: copy_from_user failed with err=0x%08lX",
			       __func__, err);
			return err;
		}

		/* TODO: stop any active streaming */

		/* Purge any previously-allocated buffers */
		touch_offload_free_buffers(context);

		memset(&context->config, 0, sizeof(context->config));
		if ((configure.config.continuous_reporting &&
		     !context->caps.continuous_reporting) ||
		    (configure.config.noise_reporting &&
		     !context->caps.noise_reporting) ||
		    (configure.config.cancel_reporting &&
		     !context->caps.cancel_reporting) ||
		    (configure.config.filter_grip &&
		     !context->caps.filter_grip) ||
		    (configure.config.filter_palm &&
		     !context->caps.filter_palm) ||
		    (configure.config.auto_reporting &&
		     !context->caps.auto_reporting) ||
		    (configure.config.coord_filter &&
		     !context->caps.coord_filter)) {
			pr_err("%s: Invalid configuration enables unsupported features!\n",
			       __func__);
			err = -EINVAL;
			return err;
		} else if (configure.config.sensitivity_setting >=
			   context->caps.num_sensitivity_settings) {
			pr_err("%s: Invalid configuration enables unsupported sensitivity setting!\n",
			       __func__);
			err = -EINVAL;
			return err;
		} else if ((configure.config.mutual_data_types &
			    ~context->caps.touch_data_types) != 0 ||
			   (configure.config.self_data_types &
			    ~context->caps.touch_data_types) != 0) {
			pr_err("%s: Invalid configuration enables unsupported data types!\n",
			       __func__);
			err = -EINVAL;
			return err;
		} else if ((configure.config.context_channel_types &
			    ~context->caps.context_channel_types) != 0) {
			pr_err("%s: Invalid configuration enables unsupported context types!\n",
			       __func__);
			err = -EINVAL;
			return err;
		}

		num_channels = (configure.config.read_coords ? 1 : 0) +
			       hweight_long(
				   configure.config.mutual_data_types) +
			       hweight_long(
				   configure.config.self_data_types) +
			       hweight_long(
				   configure.config.context_channel_types);
		if (num_channels <= 0 || num_channels > MAX_CHANNELS) {
			pr_err("%s: Invalid configuration enables more (%d) than %d channels!\n",
			       __func__, num_channels, MAX_CHANNELS);
			err = -EINVAL;
			return err;
		}

		/* Copy sanitized config */
		memcpy(&context->config, &configure.config,
		       sizeof(context->config));

		/* Allocate frames */
		err = touch_offload_allocate_buffers(context, TOUCH_OFFLOAD_BUFFER_NUM);
		if (err != 0) {
			pr_err("%s: failed to allocate buffers. err = 0x%08X.\n",
			       __func__, (unsigned int)err);
			return err;
		}

		break;
	}

	case TOUCH_OFFLOAD_IOC_WR_REPORT:
	{
		struct TouchOffloadIocReport report;

		err = copy_from_user(&report, (void *)ioctl_param,
				     sizeof(report));
		if (err != 0) {
			pr_err("%s: copy_from_user failed with err=0x%08lx.\n",
			       __func__, err);
			return err;
		}

		context->report_cb(context->hcallback, &report);
		break;
	}

	default:
		return -EINVAL;
	}

	return err;
}

const struct file_operations touch_offload_fops = {
	.owner	  = THIS_MODULE,
	.open	  = touch_offload_open,
	.release  = touch_offload_release,
	.read	  = touch_offload_read,
	.poll	  = touch_offload_poll,
	.unlocked_ioctl	  = touch_offload_ioctl
};

int touch_offload_reserve_frame(struct touch_offload_context *context,
				struct touch_offload_frame **frame)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

	if (!context || !frame)
		return -EINVAL;
	*frame = NULL;

	mutex_lock(&context->buffer_lock);

	if (context->num_buffers == 0 || list_empty(&context->free_pool) ||
	    context->reserved_frame != NULL) {
		pr_debug("%s: buffer not available.\n", __func__);
		ret = -EINVAL;
	} else {
		reinit_completion(&context->reserve_returned);
		context->reserved_frame =
			list_entry(context->free_pool.next,
				   struct touch_offload_frame, entry);
		list_del(&context->reserved_frame->entry);
		*frame = context->reserved_frame;
	}

	mutex_unlock(&context->buffer_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(touch_offload_reserve_frame);

int touch_offload_queue_frame(struct touch_offload_context *context,
			      struct touch_offload_frame *frame)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

	if (!context || !frame)
		return -EINVAL;

	mutex_lock(&context->buffer_lock);

	if (context->num_buffers == 0 || frame != context->reserved_frame) {
		pr_err("%s: incorrect or NULL buffer submitted.\n", __func__);
		ret = -EINVAL;
	} else {
		/* TODO: Restore important constant fields */

		list_add_tail(&frame->entry, &context->frame_queue);
		context->reserved_frame = NULL;
		complete_all(&context->reserve_returned);

		wake_up(&context->read_queue);
	}

	mutex_unlock(&context->buffer_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(touch_offload_queue_frame);

int touch_offload_init(struct touch_offload_context *context)
{
	int ret = 0;

	/* Initialize ioctl interface */
	context->file_in_use = false;
	mutex_init(&context->file_lock);

	/* Initialize the buffer pool */
	context->num_buffers = 0;
	context->reserved_frame = NULL;
	INIT_LIST_HEAD(&context->free_pool);
	INIT_LIST_HEAD(&context->frame_queue);
	mutex_init(&context->buffer_lock);

	init_waitqueue_head(&context->read_queue);

	init_completion(&context->reserve_returned);
	complete_all(&context->reserve_returned);

	if (!strnlen(context->device_name, sizeof(context->device_name)))
		scnprintf(context->device_name, sizeof(context->device_name), "%s", DEVICE_NAME);

	pr_info("%s: %s.\n", __func__, context->device_name);

	/* Initialize char device */
	cdev_init(&context->dev, &touch_offload_fops);

	ret = alloc_chrdev_region(&context->dev_num, 0, 1, context->device_name);
	if (ret < 0) {
		pr_err("%s: register_chrdev failed with error = %u\n",
		       __func__, ret);
		return ret;
	}

	ret = cdev_add(&context->dev, context->dev_num, 1);
	if (ret < 0) {
		pr_err("%s: cdev_add failed with error = %u\n",
		        __func__, ret);
		goto err_cdev_add;
	}

	context->cls = class_create(THIS_MODULE, context->device_name);
	if (IS_ERR(context->cls)) {
		pr_err("%s: class_create failed with error = %ld.\n",
		       __func__, PTR_ERR(context->cls));
		ret = PTR_ERR(context->cls);
		goto err_class_create;
	}

	context->device = device_create(context->cls, NULL, context->dev_num,
		NULL, context->device_name);
	if (IS_ERR(context->device)) {
		pr_err("%s: device_create failed with error = %ld.\n",
		       __func__, PTR_ERR(context->device));
		ret = PTR_ERR(context->device);
		goto err_device_create;
	}

	return ret;

err_device_create:
	class_destroy(context->cls);
err_class_create:
	cdev_del(&context->dev);
err_cdev_add:
	unregister_chrdev_region(context->dev_num, 1);
	return ret;
}
EXPORT_SYMBOL_GPL(touch_offload_init);

int touch_offload_cleanup(struct touch_offload_context *context)
{
	pr_debug("%s\n", __func__);

	device_destroy(context->cls, context->dev_num);

	class_destroy(context->cls);

	cdev_del(&context->dev);

	unregister_chrdev_region(context->dev_num, 1);

	complete_all(&context->reserve_returned);

	touch_offload_free_buffers(context);

	return 0;
}
EXPORT_SYMBOL_GPL(touch_offload_cleanup);

MODULE_DESCRIPTION("Touch Offload to AP");
MODULE_AUTHOR("Steve Pfetsch <spfetsch@google.com>");
MODULE_LICENSE("GPL v2");
