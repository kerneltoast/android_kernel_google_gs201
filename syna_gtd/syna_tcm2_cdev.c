// SPDX-License-Identifier: GPL-2.0
/*
 * Synaptics TouchCom touchscreen driver
 *
 * Copyright (C) 2017-2020 Synaptics Incorporated. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/*
 * @file syna_tcm2_cdev.c
 *
 * This file implements cdev and ioctl interface in the reference driver.
 */

#include <linux/string.h>

#include "syna_tcm2.h"
#include "syna_tcm2_cdev.h"
#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_base.h"

#if (KERNEL_VERSION(5, 9, 0) <= LINUX_VERSION_CODE) || \
	defined(HAVE_UNLOCKED_IOCTL)
#define USE_UNLOCKED_IOCTL
#endif

#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL)
#define USE_COMPAT_IOCTL
#endif

#if (KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE)
#define REPLACE_KTIME
#endif


/* #define ENABLE_PID_TASK */

#define SIG_ATTN (46)

/* structure for IOCTLs
 */
struct syna_ioctl_data {
	unsigned int data_length;
	unsigned int buf_size;
	unsigned char __user *buf;
};

#ifdef USE_COMPAT_IOCTL
struct syna_tcm_ioctl_data_compat {
	unsigned int data_length;
	unsigned int buf_size;
	compat_uptr_t __user *buf;
};
#endif

/* structure for the use of sw interface
 */
struct syna_cdev_data {
	/* This is used to backup the pointer of given platform_device
	 */
	struct platform_device *dev;
	/* This is a temporary buffer storing the data from userspace
	 */
	struct tcm_buffer buffer;
	/* These are used to protect the access from the userspace application
	 */
	syna_pal_mutex_t mutex;
	syna_pal_mutex_t queue_mutex;

	/* This variable is used to set the polling interval for the use
	 * of syna_tcm_send_command from syna_cdev_ioctl_send_message.
	 *
	 * The way to update this variable is through the
	 * syna_cdev_ioctl_enable_irq.
	 */
	unsigned int io_polling_interval;
	/* This variable allows caller to ask extra bytes to read and
	 * append at the end of the package.
	 */
	int extra_bytes;

	/* This is used to define the number of frames being queued in the
	 * kernel FIFO. This value can be set via the field of feature.depth_of_fifo
	 * inside struct drv_param.
	 * If set to '0', it represents no limitation and the depth of FIFO
	 * is equal to FIFO_QUEUE_MAX_FRAMES.
	 */
	unsigned int fifo_depth;
	/* In case that the write/read chunk size will be changed at the runtime,
	 * these variables are used to keep the original values
	 */
	unsigned int origin_max_wr_size;
	unsigned int origin_max_rd_size;
};

static struct syna_cdev_data g_cdev_data;

/* a buffer to record the streaming report
 * considering touch report and another reports may be co-enabled
 * at the same time, give a little buffer here (3 sec x 300 fps)
 */
#define FIFO_QUEUE_MAX_FRAMES		(1200)
#define SEND_MESSAGE_HEADER_LENGTH	(3)

/* Indicate the interrupt status especially for sysfs using */
#define SYSFS_DISABLED_INTERRUPT		(0)
#define SYSFS_ENABLED_INTERRUPT			(1)

/* Define a data structure that contains a list_head */
struct fifo_queue {
	struct list_head next;
	unsigned char *fifo_data;
	unsigned int data_length;
#ifdef REPLACE_KTIME
	struct timespec64 timestamp;
#else
	struct timeval timestamp;
#endif
};


#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
/*
 * syna_cdev_insert_fifo()
 *
 * Insert/Push the data to the queue.
 *
 * This function is called by syna_cdev_update_report_queue(),
 * where the event data will be placed as the below format in byte
 * and use this function to store the data in queue.
 *     [0        ] : status / report code
 *     [1 :   2  ] : length of data frame
 *     [3 : N + 3] : N bytes of data payload
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] buf_ptr:  points to a data going to push
 *    [ in] length:   data length
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_insert_fifo(struct syna_tcm *tcm,
		unsigned char *buf_ptr, unsigned int length)
{
	int retval = 0;
	struct fifo_queue *pfifo_data;
	struct fifo_queue *pfifo_data_temp;
	static int pre_remaining_frames = -1;

	syna_pal_mutex_lock(&g_cdev_data.queue_mutex);

	/* check queue buffer limit */
	if (tcm->fifo_remaining_frame >= FIFO_QUEUE_MAX_FRAMES) {
		if (tcm->fifo_remaining_frame != pre_remaining_frames)
			LOGI("FIFO is full drop the first frame\n");

		pfifo_data_temp = list_first_entry(&tcm->frame_fifo_queue,
						struct fifo_queue, next);

		list_del(&pfifo_data_temp->next);
		kfree(pfifo_data_temp->fifo_data);
		kfree(pfifo_data_temp);
		pre_remaining_frames = tcm->fifo_remaining_frame;
		tcm->fifo_remaining_frame--;
	} else if (pre_remaining_frames >= FIFO_QUEUE_MAX_FRAMES) {
		LOGI("FIFO is still full\n");
		pre_remaining_frames = tcm->fifo_remaining_frame;
	}

	pfifo_data = kmalloc(sizeof(*pfifo_data), GFP_KERNEL);
	if (!(pfifo_data)) {
		LOGE("Failed to allocate memory\n");
		LOGE("Allocation size = %zu\n", (sizeof(*pfifo_data)));
		retval = -ENOMEM;
		goto exit;
	}

	pfifo_data->fifo_data = kmalloc(length, GFP_KERNEL);
	if (!(pfifo_data->fifo_data)) {
		LOGE("Failed to allocate memory, size = %d\n", length);
		retval = -ENOMEM;
		goto exit;
	}

	pfifo_data->data_length = length;

	memcpy((void *)pfifo_data->fifo_data, (void *)buf_ptr, length);
#ifdef REPLACE_KTIME
	ktime_get_real_ts64(&(pfifo_data->timestamp));
#else
	do_gettimeofday(&(pfifo_data->timestamp));
#endif
	/* append the data to the tail for FIFO queueing */
	list_add_tail(&pfifo_data->next, &tcm->frame_fifo_queue);
	tcm->fifo_remaining_frame++;
	retval = 0;

	LOGD("Frames %d queued in FIFO\n", tcm->fifo_remaining_frame);

	/* once reaching the queue size, stop to queue data in FIFO */
	if (g_cdev_data.fifo_depth != 0) {
		if (tcm->fifo_remaining_frame >= g_cdev_data.fifo_depth) {
			if (tcm->hw_if->ops_enable_irq)
				tcm->hw_if->ops_enable_irq(tcm->hw_if, false);
		}
	}

exit:
	syna_pal_mutex_unlock(&g_cdev_data.queue_mutex);
	return retval;
}
#endif
/*
 * syna_cdev_ioctl_do_hw_reset()
 *
 * Perform the hardware reset with the selected reset method. The reset
 * option depends on the hardware design. The user has to add the
 * corresponding implementation in this function for the userspace
 * application.
 *
 * Arguments:
 *    byte 0   : skip the followed identify report
 *    byte 1-2 : active time
 *    byte 3-4 : delay time
 *
 * @param
 *    [ in] tcm:           the driver handle
 *    [ in] ubuf_ptr:      points to a memory space from userspace
 *    [ in] buf_size:      size of given space
 *    [ in] data_size:     input data size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_do_hw_reset(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	unsigned char arguments[5] = {0};
	unsigned int active_ms;
	unsigned int delay_ms;
	unsigned int original_active_ms;
	unsigned int original_delay_ms;

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return -ENXIO;
	}

	if (buf_size < sizeof(arguments) || data_size < sizeof(arguments)) {
		LOGE("Invalid sync data size, buf_size: %u\n", buf_size);
		retval = -EINVAL;
		goto exit;
	}

	if (!tcm->hw_if->ops_hw_reset) {
		LOGE("No hardware reset support\n");
		retval = -ENODEV;
		goto exit;
	}

	original_active_ms = tcm->hw_if->bdata_rst.reset_active_ms;
	original_delay_ms = tcm->hw_if->bdata_rst.reset_delay_ms;

	retval = copy_from_user(arguments, ubuf_ptr, sizeof(arguments));
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	active_ms = syna_pal_le2_to_uint(&arguments[1]);
	delay_ms  = syna_pal_le2_to_uint(&arguments[3]);

	if (active_ms > 0)
		tcm->hw_if->bdata_rst.reset_active_ms = active_ms;
	if (delay_ms > 0)
		tcm->hw_if->bdata_rst.reset_delay_ms = delay_ms;

	LOGD("HW reset arguments, skip identify report:%s active time:%d, delay time:%d\n",
		(arguments[0] == 1) ? "no" : "yes",
		tcm->hw_if->bdata_rst.reset_active_ms,
		tcm->hw_if->bdata_rst.reset_delay_ms);

	tcm->hw_if->ops_hw_reset(tcm->hw_if);

	tcm->hw_if->bdata_rst.reset_active_ms = original_active_ms;
	tcm->hw_if->bdata_rst.reset_delay_ms = original_delay_ms;

	retval = 0;
	if (arguments[0] == 1)
		goto exit;

	/* process the followed identify report */
	if (!tcm->hw_if->bdata_attn.irq_enabled) {
		tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
		syna_pal_sleep_ms(tcm->hw_if->bdata_rst.reset_delay_ms);
		tcm->hw_if->ops_enable_irq(tcm->hw_if, false);
	}

	/* check the fw setup in case the settings is changed */
	retval = tcm->dev_set_up_app_fw(tcm);
	if (retval < 0) {
		LOGE("HW reset: failed to set up the app fw\n");
		goto exit;
	}

exit:
	return retval;
}
/*
 * syna_cdev_ioctl_application_info()
 *
 * To keep the userspace application information, the user shall apply
 * the corresponding defined format on userspace. Otherwise, data will
 * be void type.
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:  size of given space
 *    [ in] data_size: size of actual data
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_application_info(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	void *data = NULL;

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return -ENXIO;
	}

	if ((buf_size < 1) || (buf_size < data_size)) {
		LOGE("Invalid input buffer size, buf_size:%u, data_size:%u\n",
			buf_size, data_size);
		return -EINVAL;
	}

	/* free the allocated memory*/
	if (tcm->userspace_app_info != NULL)
		syna_pal_mem_free(tcm->userspace_app_info);

	tcm->userspace_app_info = syna_pal_mem_alloc(1, data_size);
	if (!(tcm->userspace_app_info)) {
		LOGE("Failed to allocate user app info memory, size = %u\n",
			data_size);
		retval = -ENOMEM;
		goto exit;
	}

	syna_pal_mem_set(tcm->userspace_app_info, 0, data_size);
	data = tcm->userspace_app_info;

	retval = copy_from_user(data, ubuf_ptr, data_size);
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	/*
	 * The user shall cast the retrieved data to the format defined
	 * on userspace for the application.
	 */

exit:
	return retval;
}
/*
 * syna_cdev_ioctl_check_frame()
 *
 * Check the queuing status and wait for the data if it's empty.
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in/out] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:  size of given space
 *    [ in] data_size: timeout value for queue waiting
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_check_frame(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	int result = 0;
	unsigned int timeout = 0;
	unsigned int frames = 0;
	unsigned char data[4] = {0};

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return -ENXIO;
	}

	if (tcm->pwr_state == BARE_MODE) {
		LOGN("In bare connection mode, no frame forwarding support\n");
		return 0;
	}

	if (buf_size < sizeof(data) || data_size < sizeof(data)) {
		LOGE("Invalid sync data size, buf_size: %u\n", buf_size);
		retval = -EINVAL;
		goto exit;
	}

	result = copy_from_user(data, ubuf_ptr,
			sizeof(data));
	if (result) {
		LOGE("Fail to copy data from user space\n");
		retval = -EBADE;
		goto exit;
	}

	/* Store the waiting duration length */
	timeout = syna_pal_le4_to_uint(&data[0]);
	LOGD("Time out: %d\n", timeout);

	if (list_empty(&tcm->frame_fifo_queue)) {
		LOGD("The queue is empty, wait for the frames\n");
		result = wait_event_interruptible_timeout(tcm->wait_frame,
				(tcm->fifo_remaining_frame > 0),
				msecs_to_jiffies(timeout));
		if (result == 0) {
			LOGD("Queue waiting timed out after %dms\n", timeout);
			retval = -ETIMEDOUT;
			goto exit;
		}
		LOGD("Data queued\n");
	}

	retval = data_size;

exit:
	if (retval > 0) {
		frames = tcm->fifo_remaining_frame;
		data[0] = (unsigned char)(frames & 0xff);
		data[1] = (unsigned char)((frames >> 8) & 0xff);
		data[2] = (unsigned char)((frames >> 16) & 0xff);
		data[3] = (unsigned char)((frames >> 24) & 0xff);
		result = copy_to_user((void *)ubuf_ptr,
				data, sizeof(data));
		if (result) {
			LOGE("Fail to copy data to user space\n");
			retval = -EBADE;
		}
	}

	return retval;
}

/*
 * syna_cdev_clean_queue()
 *
 * Clean the data queue.
 * All data in the queue will be cleaned up in every time of device
 * open and close.
 *
 * @param
 *    [ in] tcm:       the driver handle
 *
 * @return
 *    void.
 */
static void syna_cdev_clean_queue(struct syna_tcm *tcm)
{
	struct fifo_queue *pfifo_data;
	unsigned int frames_to_del = tcm->fifo_remaining_frame;

	syna_pal_mutex_lock(&g_cdev_data.queue_mutex);

	while (!list_empty(&tcm->frame_fifo_queue)) {
		pfifo_data = list_first_entry(&tcm->frame_fifo_queue,
				struct fifo_queue, next);
		list_del(&pfifo_data->next);
		kfree(pfifo_data->fifo_data);
		kfree(pfifo_data);
		if (tcm->fifo_remaining_frame != 0)
			tcm->fifo_remaining_frame--;
	}

	LOGD("Kernel fifo cleaned, %d frames removed\n", frames_to_del);

	syna_pal_mutex_unlock(&g_cdev_data.queue_mutex);
}
/*
 * syna_cdev_ioctl_get_frame()
 *
 * Read the data from the queue and return to userspace if data is
 * copied or the specified timeout is expired.
 *
 * Please be noted that the retrieved data is formatted as follows.
 *     [0        ] : status / report code
 *     [1 :   2  ] : length of data frame
 *     [3 : N + 3] : N bytes of data payload
 *
 * @param
 *    [ in] tcm:           the driver handle
 *    [in/out] ubuf_ptr:   points to a memory space from userspace
 *    [ in] buf_size:      size of given space
 *    [out] frame_size:    frame size returned
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_get_frame(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int *frame_size)
{
	int retval = 0;
	int timeout = 0;
	unsigned char timeout_data[4] = {0};
	struct fifo_queue *pfifo_data;

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return -ENXIO;
	}

	if (tcm->pwr_state == BARE_MODE) {
		LOGN("In bare connection mode, no frame forwarding support\n");
		return 0;
	}

	if (buf_size < sizeof(timeout_data)) {
		LOGE("Invalid sync data size, buf_size:%d\n", buf_size);
		retval = -EINVAL;
		goto exit;
	}

#if !defined(ENABLE_EXTERNAL_FRAME_PROCESS)
	LOGE("ENABLE_EXTERNAL_FRAME_PROCESS is not enabled\n");
	return -EINVAL;
#endif

	retval = copy_from_user(timeout_data, ubuf_ptr, sizeof(timeout_data));
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	/* get the waiting duration */
	timeout = syna_pal_le4_to_uint(&timeout_data[0]);
	LOGD("Wait time: %dms\n", timeout);

	/* wait for the available frame if fifo is empty */
	if (list_empty(&tcm->frame_fifo_queue)) {
		LOGD("The queue is empty, wait for the frame\n");
		retval = wait_event_interruptible_timeout(tcm->wait_frame,
				(tcm->fifo_remaining_frame > 0),
				msecs_to_jiffies(timeout));
		if (retval == 0) {
			LOGD("Queue waiting timed out after %dms\n", timeout);
			retval = -ETIMEDOUT;
			*frame_size = 0;
			goto exit;
		}
	}

    /* confirm the queue is not empty */
	if (list_empty(&tcm->frame_fifo_queue)) {
		LOGD("Is queue empty? The remaining frame = %d\n",
			tcm->fifo_remaining_frame);
		retval = -ENODATA;
		goto exit;
	}

	/* start to pop up a frame from fifo */
	syna_pal_mutex_lock(&g_cdev_data.queue_mutex);

	pfifo_data = list_first_entry(&tcm->frame_fifo_queue,
			struct fifo_queue, next);

	LOGD("Popping data from the queue, data size:%d\n",
		pfifo_data->data_length);

	if (buf_size >= pfifo_data->data_length) {
		retval = copy_to_user((void *)ubuf_ptr,
				pfifo_data->fifo_data,
				pfifo_data->data_length);
		if (retval) {
			LOGE("Fail to copy data to user space, size:%d\n",
				retval);
			retval = -EBADE;
		}

		*frame_size = pfifo_data->data_length;

	} else {
		LOGE("No enough space for data copy, buf_size:%d data:%d\n",
			buf_size, pfifo_data->data_length);

		retval = -EOVERFLOW;
		goto exit;
	}

	LOGD("Data popped: 0x%02x, 0x%02x, 0x%02x ...\n",
		pfifo_data->fifo_data[0], pfifo_data->fifo_data[1],
		pfifo_data->fifo_data[2]);

	list_del(&pfifo_data->next);

	if (retval >= 0)
		retval = pfifo_data->data_length;

	kfree(pfifo_data->fifo_data);
	kfree(pfifo_data);
	if (tcm->fifo_remaining_frame != 0)
		tcm->fifo_remaining_frame--;

	/* re-activate kernel FIFO if it was full */
	if (tcm->fifo_remaining_frame < g_cdev_data.fifo_depth) {
		if (!tcm->hw_if->bdata_attn.irq_enabled) {
			if (tcm->hw_if->ops_enable_irq)
				tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
		}
	}

	LOGD("Frames %d remaining in FIFO\n", tcm->fifo_remaining_frame);

	syna_pal_mutex_unlock(&g_cdev_data.queue_mutex);

exit:
	return retval;
}

/*
 * syna_cdev_ioctl_set_reports()
 *
 * Assign the report types for queuing. The enabled reports will be queued
 * into the FIFO queue.
 *
 * @param
 *    [ in] tcm:            the driver handle
 *    [ in] ubuf_ptr:       points to a memory space from userspace
 *    [ in] buf_size:       size of given space
 *    [ in] report_size:    report types data size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_set_reports(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int report_size)
{
	int retval = 0;
	unsigned char data[REPORT_TYPES] = {0};
	unsigned int reports = 0;
	unsigned int report_set = 0;

	if (tcm->pwr_state == BARE_MODE) {
		LOGN("In bare connection mode, no report forwarding support\n");
		return 0;
	}

	if (buf_size < sizeof(data)) {
		LOGE("Invalid sync data size, buf_size:%d, expected:%d\n",
			buf_size, (unsigned int)sizeof(data));
		return -EINVAL;
	}

#if !defined(ENABLE_EXTERNAL_FRAME_PROCESS)
	LOGE("ENABLE_EXTERNAL_FRAME_PROCESS is not enabled\n");
	return -EINVAL;
#endif

	if (report_size == 0) {
		LOGE("Invalid written size\n");
		return -EINVAL;
	}

	retval = copy_from_user(data, ubuf_ptr, report_size);
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	retval = syna_pal_mem_cpy(tcm->report_to_queue, REPORT_TYPES,
			data, sizeof(data), REPORT_TYPES);
	for (reports = 0 ; reports < REPORT_TYPES ; reports++) {
		if (tcm->report_to_queue[reports] == EFP_ENABLE)
			report_set++;
	}
	if (report_set < 16) {
		for (reports = 0 ; reports < REPORT_TYPES ; reports++) {
			if (tcm->report_to_queue[reports] == EFP_ENABLE)
				LOGD("Set report 0x%02x for queue\n", reports);
		}
	}

	LOGD("Forward %d types of reports to the Queue.\n", report_set);

	retval = report_set;

exit:
	return retval;
}
/*
 * syna_cdev_ioctl_send_message()
 *
 * Send the command/message from userspace.
 *
 * For updating the io_polling_interval, it need to be configured
 * by syna_cdev_ioctl_enable_irq from userspace.
 *
 * @param
 *    [ in] tcm:           the driver handle
 *    [ in/out] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:      size of given space
 *    [ in/out] msg_size:  size of message
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_send_message(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int *msg_size)
{
	int retval = 0;
	struct tcm_dev *tcm_dev = tcm->tcm_dev;
	unsigned int offset;
	unsigned int size;
	unsigned char *data_ptr = NULL;
	unsigned char resp_code = 0;
	unsigned int length_in_header = 0;
	unsigned int actual_length = 0;
	unsigned int delay_ms_resp = RESP_IN_POLLING;
	struct tcm_buffer *caller;
	struct tcm_buffer resp_data_buf;
	unsigned short val;

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return -ENXIO;
	}

	if (tcm->pwr_state == BARE_MODE) {
		LOGN("In bare connection mode, no command handler support\n");
		return 0;
	}

	if (buf_size < SEND_MESSAGE_HEADER_LENGTH + g_cdev_data.extra_bytes) {
		LOGE("Invalid sync data size, buf_size:%d\n", buf_size);
		return -EINVAL;
	}

	if (*msg_size < 3) {
		LOGE("Invalid size of message %d, the min length is three\n",
			*msg_size);
		return -EINVAL;
	}

	caller = &g_cdev_data.buffer;
	syna_tcm_buf_lock(caller);

	size = (g_cdev_data.extra_bytes > 0) ?
		(buf_size + g_cdev_data.extra_bytes) : buf_size;
	retval = syna_tcm_buf_alloc(caller, size);
	if (retval < 0) {
		LOGE("Fail to allocate memory for caller buf, size: %d\n",
			buf_size);
		goto exit;
	}

	data_ptr = caller->buf;

	retval = copy_from_user(data_ptr, ubuf_ptr, *msg_size);
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", *msg_size);
		retval = -EBADE;
		goto exit;
	}

	tcm->is_attn_asserted = false;

	length_in_header = syna_pal_le2_to_uint(&data_ptr[1]);
	actual_length = (*msg_size) - 3;

	if (length_in_header < actual_length)
		actual_length = length_in_header;

	LOGD("Write Command: 0x%02x, 0x%02x, 0x%02x (payload size:%d)\n",
		data_ptr[0], data_ptr[1], data_ptr[2], (*msg_size));
	if (length_in_header != actual_length)
		LOGD("Size of payload to write:%d (size in header:%d)\n",
			actual_length, length_in_header);

	/* init a buffer for the response data */
	syna_tcm_buf_init(&resp_data_buf);

	if (g_cdev_data.io_polling_interval == RESP_IN_ATTN)
		delay_ms_resp = RESP_IN_ATTN;
	else
		delay_ms_resp = g_cdev_data.io_polling_interval;

	retval = syna_tcm_send_command(tcm_dev,
			data_ptr[0],
			&data_ptr[3],
			actual_length,
			length_in_header,
			&resp_code,
			&resp_data_buf,
			delay_ms_resp);
	if (retval < 0) {
		LOGE("Fail to run command 0x%02x with payload len %d\n",
			data_ptr[0], actual_length);
		/* even if resp_code returned is not success
		 * this ioctl shall return the packet to caller
		 */
	}

	syna_pal_mem_set(data_ptr, 0, buf_size);
	/* status code */
	data_ptr[0] = resp_code;
	/* the length for response data */
	data_ptr[1] = (unsigned char)(resp_data_buf.data_length & 0xff);
	data_ptr[2] = (unsigned char)((resp_data_buf.data_length >> 8) & 0xff);

	offset = SEND_MESSAGE_HEADER_LENGTH;
	LOGD("Resp data: 0x%02x 0x%02x 0x%02x\n",
		data_ptr[0], data_ptr[1], data_ptr[2]);

	if (caller->buf_size < resp_data_buf.data_length) {
		LOGE("No enough space for data copy, buf_size:%d data:%d\n",
			caller->buf_size, resp_data_buf.data_length);
		retval = -EOVERFLOW;
		goto exit;
	}

	/* response data returned */
	if (resp_data_buf.data_length > 0) {
		retval = syna_pal_mem_cpy(&data_ptr[offset],
			(caller->buf_size - offset),
			resp_data_buf.buf,
			resp_data_buf.buf_size,
			resp_data_buf.data_length);
		if (retval < 0) {
			LOGE("Fail to copy resp data\n");
			goto exit;
		}

		offset += resp_data_buf.data_length;

		if (g_cdev_data.extra_bytes >= TCM_MSG_CRC_LENGTH) {
			val = tcm_dev->msg_data.crc_bytes;
			data_ptr[offset] = (unsigned char)val;
			data_ptr[offset + 1] = (unsigned char)(val >> 8);

			val = g_cdev_data.extra_bytes - TCM_MSG_CRC_LENGTH;
			if (val >= TCM_EXTRA_RC_LENGTH)
				data_ptr[offset + TCM_MSG_CRC_LENGTH] =
					tcm_dev->msg_data.rc_byte;
		}
	}

#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
    /* It's for queuing the data when user is polling the command
     * response for the selected responses. The response will not be
     * queued if the user doesn't set the report/response types through
     * syna_cdev_ioctl_set_reports.
     */
	if (!tcm->is_attn_asserted) {
		if (tcm->report_to_queue[resp_code] == EFP_ENABLE) {
			syna_cdev_update_report_queue(tcm, resp_code,
				&resp_data_buf);
		}
	}
#endif

	*msg_size = resp_data_buf.data_length + SEND_MESSAGE_HEADER_LENGTH;
	if (g_cdev_data.extra_bytes > 0)
		*msg_size += g_cdev_data.extra_bytes;
	retval = copy_to_user((void *)ubuf_ptr, data_ptr, *msg_size);
	if (retval) {
		LOGE("Fail to copy data to user space\n");
		retval = -EBADE;
		goto exit;
	}

	retval = *msg_size;

exit:
	tcm->is_attn_asserted = false;

	syna_tcm_buf_unlock(caller);

	syna_tcm_buf_release(&resp_data_buf);

	return retval;
}

/*
 * syna_cdev_ioctl_enable_irq()
 *
 * Enable or disable the irq via IOCTL.
 *
 * Expect to get 4 bytes unsigned int parameter from userspace:
 *    0:         disable the irq.
 *    1:         enable the irq and set io_polling_interval
 *               to RESP_IN_ATTN
 *    otherwise: enable the irq and also assign the polling interval
 *               to a specific time, which will be used when calling
 *               syna_cdev_ioctl_send_message.
 *               the min. polling time is RESP_IN_POLLING
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:  size of given space
 *    [ in] data_size: size of actual data
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_enable_irq(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	unsigned int data;

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return -ENXIO;
	}

	if (tcm->pwr_state == BARE_MODE) {
		LOGN("In bare connection mode, no irq support\n");
		return 0;
	}

	if ((buf_size < sizeof(data)) || (data_size < sizeof(data))) {
		LOGE("Invalid sync data size, buf_size:%d, data_size:%d\n",
		    buf_size, data_size);
		return -EINVAL;
	}

	if (!tcm->hw_if->ops_enable_irq) {
		LOGW("Not support irq control\n");
		return -EINVAL;
	}

	retval = copy_from_user(&data, ubuf_ptr, buf_size);
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		return -EBADE;
	}

	switch (data) {
	case SYSFS_DISABLED_INTERRUPT:
		if (tcm->hw_if->bdata_attn.irq_enabled)
			LOGI("IRQ is disabled by userspace application\n");

		retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, false);
		if (retval < 0) {
			LOGE("Fail to disable interrupt\n");
			return retval;
		}

		g_cdev_data.io_polling_interval =
			tcm->tcm_dev->msg_data.default_resp_reading;

		break;
	case SYSFS_ENABLED_INTERRUPT:
		if (!tcm->hw_if->bdata_attn.irq_enabled)
			LOGI("IRQ is enabled by userspace application\n");

		retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
		if (retval < 0) {
			LOGE("Fail to enable interrupt\n");
			return retval;
		}

		g_cdev_data.io_polling_interval = RESP_IN_ATTN;

		break;
	default:
		/* recover the interrupt and also assign the polling interval */
		retval = tcm->hw_if->ops_enable_irq(tcm->hw_if, true);
		if (retval < 0) {
			LOGE("Fail to enable interrupt\n");
			return retval;
		}

		g_cdev_data.io_polling_interval = data;
		if (g_cdev_data.io_polling_interval < RESP_IN_POLLING)
			g_cdev_data.io_polling_interval = RESP_IN_POLLING;

		LOGI("IRQ is enabled by userspace application\n");
		LOGI("Set polling interval is %d ms\n",
			g_cdev_data.io_polling_interval);

		break;
	}

	return 0;
}
/*
 * syna_cdev_ioctl_store_pid()
 *
 * Save PID through IOCTL interface
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in] ubuf_ptr:  points to a memory space from userspace
 *    [ in] buf_size:  size of given space
 *    [ in] data_size: size of actual data
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_store_pid(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int data_size)
{
	int retval = 0;
	unsigned char *data = NULL;
	struct tcm_buffer *caller;

	if (!tcm->is_connected) {
		LOGE("Not connected\n");
		return -ENXIO;
	}

	if (buf_size < 4) {
		LOGE("Invalid sync data size, buf_size:%d\n", buf_size);
		return -EINVAL;
	}

	if (data_size < 4) {
		LOGE("Invalid data_size\n");
		return -EINVAL;
	}

	caller = &g_cdev_data.buffer;
	syna_tcm_buf_lock(caller);

	retval = syna_tcm_buf_alloc(caller, buf_size);
	if (retval < 0) {
		LOGE("Fail to allocate memory for caller buf, size: %d\n",
			buf_size);
		goto exit;
	}

	data = caller->buf;
	retval = copy_from_user(data, ubuf_ptr, data_size);
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	tcm->proc_pid = syna_pal_le4_to_uint(&data[0]);

	LOGD("PID: %d\n", (unsigned int)tcm->proc_pid);
#ifdef ENABLE_PID_TASK
	if (tcm->proc_pid) {
		tcm->proc_task = pid_task(
				find_vpid(tcm->proc_pid),
				PIDTYPE_PID);
		if (!tcm->proc_task) {
			LOGE("Fail to locate task, pid: %d\n",
				(unsigned int)tcm->proc_pid);
			retval = -ESRCH;
			goto exit;
		}
	}
#endif
exit:
	syna_tcm_buf_unlock(caller);

	return retval;
}
/*
 * syna_cdev_ioctl_raw_read()
 *
 * Read the data from device directly without routing to command wrapper
 * interface.
 *
 * @param
 *    [ in] tcm:         the driver handle
 *    [in/out] ubuf_ptr: ubuf_ptr: points to a memory space from userspace
 *    [ in] buf_size:    size of given space
 *    [ in] rd_size:     reading size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_raw_read(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int rd_size)
{
	int retval = 0;
	unsigned char *data = NULL;
	struct tcm_buffer *caller;

	if ((buf_size < 0) || (rd_size > buf_size)) {
		LOGE("Invalid sync data size, buf_size:%d, rd_size:%d\n",
			buf_size, rd_size);
		return -EINVAL;
	}

	if (rd_size == 0) {
		LOGE("The read length is 0\n");
		return 0;
	}

	syna_pal_mutex_lock(&tcm->tcm_dev->msg_data.rw_mutex);

	caller = &g_cdev_data.buffer;
	syna_tcm_buf_lock(caller);

	retval = syna_tcm_buf_alloc(caller, rd_size);
	if (retval < 0) {
		LOGE("Fail to allocate memory for caller buf, size: %d\n",
			rd_size);
		goto exit;
	}

	data = caller->buf;
	retval = syna_tcm_read(tcm->tcm_dev,
			data,
			rd_size);
	if (retval < 0) {
		LOGE("Fail to read raw data, size: %d\n", rd_size);
		goto exit;
	}

	if (copy_to_user((void *)ubuf_ptr, data, rd_size)) {
		LOGE("Fail to copy data to user space\n");
		retval = -EBADE;
		goto exit;
	}

	retval = rd_size;

exit:
	syna_tcm_buf_unlock(caller);

	syna_pal_mutex_unlock(&tcm->tcm_dev->msg_data.rw_mutex);

	return retval;
}
/*
 * syna_cdev_ioctl_raw_write()
 *
 * Write the given data to device directly without routing to command wrapper
 * interface.
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] ubuf_ptr: points to a memory space from userspace
 *    [ in] buf_size: size of given space
 *    [ in] wr_size:  size to write
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_raw_write(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int wr_size)
{
	int retval = 0;
	unsigned char *data = NULL;
	struct tcm_buffer *caller;

	if ((buf_size < 0) || (wr_size > buf_size)) {
		LOGE("Invalid sync data size, buf_size:%d, wr_size:%d\n",
			buf_size, wr_size);
		return -EINVAL;
	}

	if (wr_size == 0) {
		LOGE("Invalid written size\n");
		return -EINVAL;
	}

	syna_pal_mutex_lock(&tcm->tcm_dev->msg_data.rw_mutex);

	caller = &g_cdev_data.buffer;
	syna_tcm_buf_lock(caller);

	retval = syna_tcm_buf_alloc(caller, wr_size);
	if (retval < 0) {
		LOGE("Fail to allocate memory for caller buf, size: %d\n",
			wr_size);
		goto exit;
	}

	data = caller->buf;
	retval = copy_from_user(data, ubuf_ptr, wr_size);
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	LOGD("Write data: 0x%02x, 0x%02x, 0x%02x (length:%d)\n",
		data[0], data[1], data[2], wr_size);

	retval = syna_tcm_write(tcm->tcm_dev,
			data,
			wr_size);
	if (retval < 0) {
		LOGE("Fail to write raw data, size: %d\n", wr_size);
		goto exit;
	}

	retval = wr_size;

exit:
	syna_tcm_buf_unlock(caller);

	syna_pal_mutex_unlock(&tcm->tcm_dev->msg_data.rw_mutex);

	return retval;
}
/*
 * syna_cdev_ioctl_get_config_params()
 *
 * Return current configuration settings to user-space
 * The returned buffer array should be same as struct drv_param
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] ubuf_ptr: points to a memory space from userspace
 *    [ in] buf_size: size of given space
 *    [ in] size:     size of array
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_get_config_params(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int size)
{
	int retval = 0;
	struct drv_param *param;
	struct tcm_buffer *caller;

	if (buf_size < 0) {
		LOGE("Invalid sync data size, out of range\n");
		return -EINVAL;
	}

	if (size < sizeof(struct drv_param)) {
		LOGE("Invalid data input, size: %d (expected: %zd)\n",
			size, sizeof(struct drv_param));
		return -EINVAL;
	}

	caller = &g_cdev_data.buffer;
	syna_tcm_buf_lock(caller);

	retval = syna_tcm_buf_alloc(caller, sizeof(struct drv_param));
	if (retval < 0) {
		LOGE("Fail to allocate memory for caller buf, size: %zd\n",
			sizeof(struct drv_param));
		goto exit;
	}

	syna_pal_mem_set(&caller->buf[0], 0x00, sizeof(struct drv_param));

	param = (struct drv_param *)&caller->buf[0];


	param->bus.chunk_wr_size =
		(unsigned short)tcm->tcm_dev->max_wr_size;
	param->bus.chunk_rd_size =
		(unsigned short)tcm->tcm_dev->max_rd_size;

	param->connection.activate = (tcm->is_connected) ? 1 : 0;
	param->connection.inactivate = (tcm->is_connected) ? 0 : 1;
	param->connection.bare = (tcm->pwr_state == BARE_MODE) ? 1 : 0;

	if (tcm->tcm_dev->id_info.version > 0)
		param->connection.touchcomm_version =
			(unsigned char)tcm->tcm_dev->id_info.version;

	param->feature.predict_reads = (tcm->tcm_dev->msg_data.predict_reads & 0x01);
	param->feature.extra_bytes_to_read = (unsigned char)g_cdev_data.extra_bytes;
	param->feature.depth_of_fifo = (g_cdev_data.fifo_depth >> 2);

	/* copy the info to user-space */
	retval = copy_to_user((void *)ubuf_ptr,
		(unsigned char *)param,
		sizeof(struct drv_param));
	if (retval) {
		LOGE("Fail to copy data to user space\n");
		retval = -EBADE;
		goto exit;
	}

	retval = sizeof(struct drv_param);

exit:
	syna_tcm_buf_unlock(caller);

	return retval;
}
/*
 * syna_cdev_ioctl_set_config()
 *
 * Set up and connect to touch controller.
 * The given buffer array should be same as struct drv_param
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] ubuf_ptr: points to a memory space from userspace
 *    [ in] buf_size: size of given space
 *    [ in] in_size:  input data size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_set_config(struct syna_tcm *tcm,
		const unsigned char *ubuf_ptr, unsigned int buf_size,
		unsigned int in_size)
{
	int retval = 0;
	struct tcm_dev *tcm_dev = tcm->tcm_dev;
	struct drv_param *param;
	bool predict_read = false;
	bool chunks_update = false;
	int extra_bytes = 0;
	struct tcm_buffer *caller;

	if (buf_size < 0) {
		LOGE("Invalid sync data size, out of range\n");
		return -EINVAL;
	}

	if (in_size < sizeof(struct drv_param)) {
		LOGE("Invalid data input, size: %d (expected: %zd)\n",
			in_size, sizeof(struct drv_param));
		return -EINVAL;
	}

	caller = &g_cdev_data.buffer;
	syna_tcm_buf_lock(caller);

	retval = syna_tcm_buf_alloc(caller, sizeof(struct drv_param));
	if (retval < 0) {
		LOGE("Fail to allocate memory for caller buf, size: %zd\n",
			sizeof(struct drv_param));
		goto exit;
	}

	retval = copy_from_user(&caller->buf[0], ubuf_ptr,
		sizeof(struct drv_param));
	if (retval) {
		LOGE("Fail to copy data from user space, size:%d\n", retval);
		retval = -EBADE;
		goto exit;
	}

	param = (struct drv_param *)&caller->buf[0];


	/* config the feature of legacy firmware */
	tcm_dev->msg_data.legacy = (param->feature.legacy_firmware == 1);


	/* set up driver features */
	if (tcm->is_connected) {
		/* change the chunk size */
		if (param->bus.chunk_rd_size > 0) {
			if (tcm_dev->max_rd_size != param->bus.chunk_rd_size) {
				tcm_dev->max_rd_size = param->bus.chunk_rd_size;
				chunks_update = true;
			}
		}
		if (param->bus.chunk_wr_size > 0) {
			if (tcm_dev->max_wr_size != param->bus.chunk_wr_size) {
				tcm_dev->max_wr_size = param->bus.chunk_wr_size;
				chunks_update = true;
			}
		}
		if (chunks_update)
			tcm_dev->set_max_rw_size(tcm_dev);

		/* change the feature of predict reading */
		predict_read = (param->feature.predict_reads == 1);
		if (tcm_dev->msg_data.predict_reads != predict_read) {
			LOGI("request to %s predict reading\n", (predict_read) ? "enable":"disable");
			syna_tcm_enable_predict_reading(tcm_dev, predict_read);
		}
		/* change the feature of extra bytes reading */
		extra_bytes = param->feature.extra_bytes_to_read;
		if (g_cdev_data.extra_bytes != extra_bytes) {
			g_cdev_data.extra_bytes = extra_bytes;
			LOGI("request to read in %d extra bytes\n", extra_bytes);
		}
		/* change the depth of kernel fifo */
		g_cdev_data.fifo_depth = param->feature.depth_of_fifo << 2;
		if (g_cdev_data.fifo_depth > FIFO_QUEUE_MAX_FRAMES)
			g_cdev_data.fifo_depth = 0;
		if (g_cdev_data.fifo_depth != 0)
			LOGI("request to adjust kernel fifo size to %d\n", g_cdev_data.fifo_depth);

	}

exit:
	syna_tcm_buf_unlock(caller);

	return retval;
}
/*
 * syna_cdev_ioctl_dispatch()
 *
 * Dispatch the IOCTLs operation based on the given code
 *
 * @param
 *    [ in] tcm:       the driver handle
 *    [ in] code:      code for the target operation
 *    [ in] ubuf_ptr:  points to a memory space from userspace
 *    [ in] ubuf_size: size of given space
 *    [ in] wr_size:   written data size
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_dispatch(struct syna_tcm *tcm,
		unsigned int code, const unsigned char *ubuf_ptr,
		unsigned int ubuf_size, unsigned int *data_size)
{
	int retval = 0;

	switch (code) {
	case STD_SET_PID_ID:
		retval = syna_cdev_ioctl_store_pid(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_ENABLE_IRQ_ID:
		retval = syna_cdev_ioctl_enable_irq(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_RAW_WRITE_ID:
		retval = syna_cdev_ioctl_raw_write(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_RAW_READ_ID:
		retval = syna_cdev_ioctl_raw_read(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_GET_FRAME_ID:
		retval = syna_cdev_ioctl_get_frame(tcm,
				ubuf_ptr, ubuf_size, data_size);
		break;
	case STD_SEND_MESSAGE_ID:
		retval = syna_cdev_ioctl_send_message(tcm,
				ubuf_ptr, ubuf_size, data_size);
		break;
	case STD_SET_REPORTS_ID:
		retval = syna_cdev_ioctl_set_reports(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_CHECK_FRAMES_ID:
		retval = syna_cdev_ioctl_check_frame(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_CLEAN_OUT_FRAMES_ID:
		syna_cdev_clean_queue(tcm);
		retval = 0;
		break;
	case STD_APPLICATION_INFO_ID:
		retval = syna_cdev_ioctl_application_info(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_DO_HW_RESET_ID:
		retval = syna_cdev_ioctl_do_hw_reset(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_DRIVER_CONFIG_ID:
		retval = syna_cdev_ioctl_set_config(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	case STD_DRIVER_GET_CONFIG_ID:
		retval = syna_cdev_ioctl_get_config_params(tcm,
				ubuf_ptr, ubuf_size, *data_size);
		break;
	default:
		LOGE("Unknown ioctl code: 0x%x\n", code);
		return -EINVAL;
	}

	return retval;
}
/*
 * syna_cdev_ioctl_old_dispatch()
 *
 * Dispatch the old IOCTLs operation based on the given code
 *
 * @param
 *    [ in] tcm:      the driver handle
 *    [ in] code:     code for the target operation
 *    [ in] arg:      argument passed from user-space
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_ioctl_old_dispatch(struct syna_tcm *tcm,
		unsigned int code, unsigned long arg)
{
	int retval = 0;

	switch (code) {
	case OLD_RESET_ID:
		retval = syna_tcm_reset(tcm->tcm_dev);
		if (retval < 0) {
			LOGE("Fail to do reset\n");
			break;
		}

		retval = tcm->dev_set_up_app_fw(tcm);
		if (retval < 0) {
			LOGE("Fail to set up app fw\n");
			break;
		}

		break;
	case OLD_SET_IRQ_MODE_ID:
		if (!tcm->hw_if->ops_enable_irq) {
			retval = -EINVAL;
			break;
		}

		if (arg == 0)
			retval = tcm->hw_if->ops_enable_irq(tcm->hw_if,
					false);
		else if (arg == 1)
			retval = tcm->hw_if->ops_enable_irq(tcm->hw_if,
					true);
		break;
	case OLD_SET_RAW_MODE_ID:
		retval = 0;
		break;
	case OLD_CONCURRENT_ID:
		retval = 0;
		break;

	default:
		LOGE("Unknown ioctl code: 0x%x\n", code);
		retval = -EINVAL;
		break;
	}

	return retval;
}

/*
 * syna_cdev_ioctls()
 *
 * Used to implements the IOCTL operations
 *
 * @param
 *    [ in] filp: represents the file descriptor
 *    [ in] cmd:  command code sent from userspace
 *    [ in] arg:  arguments sent from userspace
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
#ifdef USE_UNLOCKED_IOCTL
static long syna_cdev_ioctls(struct file *filp, unsigned int cmd,
		unsigned long arg)
#else
static int syna_cdev_ioctls(struct inode *inp, struct file *filp,
		unsigned int cmd, unsigned long arg)
#endif
{
	int retval = 0;
	struct syna_tcm *tcm = platform_get_drvdata(g_cdev_data.dev);
	struct syna_ioctl_data ioc_data;
	unsigned char *ptr = NULL;

	syna_pal_mutex_lock(&g_cdev_data.mutex);

	retval = 0;

	LOGD("%s (ID:0x%02X) received\n",
		syna_cdev_ioctl_get_name((unsigned int)_IOC_NR(cmd)),
		(unsigned int)_IOC_NR(cmd));

	/* handle the old IOCTLs */
	if ((_IOC_NR(cmd)) < STD_IOCTL_BEGIN) {
		retval = syna_cdev_ioctl_old_dispatch(tcm,
			(unsigned int)_IOC_NR(cmd), arg);

		goto exit;
	} else if ((_IOC_NR(cmd)) == STD_IOCTL_BEGIN) {
		retval = 1;
		goto exit;
	}

	retval = copy_from_user(&ioc_data,
			(void __user *) arg,
			sizeof(struct syna_ioctl_data));
	if (retval) {
		LOGE("Fail to copy ioctl_data from user space, size:%d\n",
			retval);
		retval = -EBADE;
		goto exit;
	}

	ptr = ioc_data.buf;

	retval = syna_cdev_ioctl_dispatch(tcm,
			(unsigned int)_IOC_NR(cmd),
			(const unsigned char *)ptr,
			ioc_data.buf_size,
			&ioc_data.data_length);
	if (retval < 0)
		goto exit;

	retval = copy_to_user((void __user *) arg,
			&ioc_data,
			sizeof(struct syna_ioctl_data));
	if (retval) {
		LOGE("Fail to update ioctl_data to user space, size:%d\n",
			retval);
		retval = -EBADE;
		goto exit;
	}

exit:
	syna_pal_mutex_unlock(&g_cdev_data.mutex);

	return retval;
}

#ifdef USE_COMPAT_IOCTL
/*
 * syna_cdev_compat_ioctls()
 *
 * Used to implements the IOCTL compatible operations
 *
 * @param
 *    [ in] filp: represents the file descriptor
 *    [ in] cmd: command code sent from userspace
 *    [ in] arg: arguments sent from userspace
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static long syna_cdev_compat_ioctls(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	struct syna_tcm *tcm = platform_get_drvdata(g_cdev_data.dev);
	struct syna_tcm_ioctl_data_compat ioc_data;
	unsigned char *ptr = NULL;

	syna_pal_mutex_lock(&g_cdev_data.mutex);

	retval = 0;

	/* handle the old IOCTLs */
	if ((_IOC_NR(cmd)) < STD_IOCTL_BEGIN) {
		retval = syna_cdev_ioctl_old_dispatch(tcm,
			(unsigned int)_IOC_NR(cmd), arg);

		goto exit;
	} else if ((_IOC_NR(cmd)) == STD_IOCTL_BEGIN) {
		retval = 1;
		goto exit;
	}

	retval = copy_from_user(&ioc_data,
		(struct syna_tcm_ioctl_data_compat __user *) compat_ptr(arg),
		sizeof(struct syna_tcm_ioctl_data_compat));
	if (retval) {
		LOGE("Fail to copy ioctl_data from user space, size:%d\n",
			retval);
		retval = -EBADE;
		goto exit;
	}

	ptr = compat_ptr((unsigned long)ioc_data.buf);

	retval = syna_cdev_ioctl_dispatch(tcm,
			(unsigned int)_IOC_NR(cmd),
			(const unsigned char *)ptr,
			ioc_data.buf_size,
			&ioc_data.data_length);
	if (retval < 0)
		goto exit;

	retval = copy_to_user(compat_ptr(arg),
			&ioc_data,
			sizeof(struct syna_tcm_ioctl_data_compat));
	if (retval) {
		LOGE("Fail to update ioctl_data to user space, size:%d\n",
			retval);
		retval = -EBADE;
		goto exit;
	}

exit:
	syna_pal_mutex_unlock(&g_cdev_data.mutex);

	return retval;
}
#endif

/*
 * syna_cdev_llseek()
 *
 * Used to change the current position in a file.
 *
 * @param
 *    [ in] filp:   represents the file descriptor
 *    [ in] off:    the file position
 *    [ in] whence: flag for seeking
 *
 * @return
 *    not support
 */
static loff_t syna_cdev_llseek(struct file *filp,
		loff_t off, int whence)
{
	return -EINVAL;
}
/*
 * syna_cdev_read()
 *
 * Used to read data through the device file.
 * Function will use raw write approach.
 *
 * @param
 *    [ in] filp:  represents the file descriptor
 *    [out] buf:   given buffer from userspace
 *    [ in] count: size of buffer
 *    [ in] f_pos: the file position
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static ssize_t syna_cdev_read(struct file *filp,
		char __user *buf, size_t count, loff_t *f_pos)
{
	int retval = 0;
	struct syna_tcm *tcm = platform_get_drvdata(g_cdev_data.dev);

	if (count == 0)
		return 0;

	syna_pal_mutex_lock(&g_cdev_data.mutex);

	retval = syna_cdev_ioctl_raw_read(tcm,
			(const unsigned char *)buf, count, count);
	if (retval != count) {
		LOGE("Invalid read operation, request:%d, return:%d\n",
			(unsigned int)count, retval);
	}

	syna_pal_mutex_unlock(&g_cdev_data.mutex);

	return retval;
}
/*
 * syna_cdev_write()
 *
 * Used to send data to device through the device file.
 * Function will use raw write approach.
 *
 * @param
 *    [ in] filp:  represents the file descriptor
 *    [ in] buf:   given buffer from userspace
 *    [ in] count: size of buffer
 *    [ in] f_pos: the file position
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static ssize_t syna_cdev_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *f_pos)
{
	int retval = 0;
	struct syna_tcm *tcm = platform_get_drvdata(g_cdev_data.dev);

	if (count == 0)
		return 0;

	syna_pal_mutex_lock(&g_cdev_data.mutex);

	retval = syna_cdev_ioctl_raw_write(tcm,
			(const unsigned char *)buf, count, count);
	if (retval != count) {
		LOGE("Invalid write operation, request:%d, return:%d\n",
			(unsigned int)count, retval);
	}

	syna_pal_mutex_unlock(&g_cdev_data.mutex);

	return retval;
}
/*
 * syna_cdev_open()
 *
 * Invoked when the device file is being open, which should be
 * always the first operation performed on the device file
 *
 * @param
 *    [ in] inp:  represents a file in rootfs
 *    [ in] filp: represents the file descriptor
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_open(struct inode *inp, struct file *filp)
{
	struct syna_tcm *tcm = platform_get_drvdata(g_cdev_data.dev);

	syna_pal_mutex_lock(&g_cdev_data.mutex);

	if (tcm->char_dev_ref_count != 0) {
		LOGN("CDevice already open, %d\n", tcm->char_dev_ref_count);
	}

	tcm->char_dev_ref_count++;

	g_cdev_data.io_polling_interval = 0;
	g_cdev_data.fifo_depth = 0;
	g_cdev_data.extra_bytes = 0;

	g_cdev_data.origin_max_rd_size = tcm->tcm_dev->max_rd_size;
	g_cdev_data.origin_max_wr_size = tcm->tcm_dev->max_wr_size;

	tcm->tcm_dev->msg_data.predict_reads = false;
	tcm->tcm_dev->msg_data.legacy = false;

#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
	syna_cdev_clean_queue(tcm);
#endif
	syna_pal_mutex_unlock(&g_cdev_data.mutex);

	/* Force to use CPU mode in case some command cannot fit the 4 bytes alignment */
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE) && IS_ENABLED(CONFIG_SPI_S3C64XX_GS)
	if (goog_check_spi_dma_enabled(tcm->hw_if->pdev) && tcm->hw_if->s3c64xx_sci) {
		tcm->hw_if->dma_mode = 0;
		tcm->hw_if->s3c64xx_sci->dma_mode = CPU_MODE;
	}
#endif

	LOGI("CDevice open\n");

	return 0;
}
/*
 * syna_cdev_release()
 *
 * Invoked when the device file is being released
 *
 * @param
 *    [ in] inp:  represents a file in rootfs
 *    [ in] filp: represents the file descriptor
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_cdev_release(struct inode *inp, struct file *filp)
{
	struct syna_tcm *tcm = platform_get_drvdata(g_cdev_data.dev);

	syna_pal_mutex_lock(&g_cdev_data.mutex);

	if (tcm->char_dev_ref_count <= 0) {
		LOGN("CDevice already closed, %d\n", tcm->char_dev_ref_count);
		return 0;
	}

	tcm->char_dev_ref_count--;

	tcm->is_attn_asserted = false;
	syna_pal_mem_set(tcm->report_to_queue, 0, REPORT_TYPES);
#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
	syna_cdev_clean_queue(tcm);
#endif
	syna_pal_mutex_unlock(&g_cdev_data.mutex);

	g_cdev_data.io_polling_interval = 0;
	g_cdev_data.fifo_depth = 0;
	g_cdev_data.extra_bytes = 0;

	/* Restore DMA mode */
#if IS_ENABLED(CONFIG_GOOG_TOUCH_INTERFACE) && IS_ENABLED(CONFIG_SPI_S3C64XX_GS)
	if (goog_check_spi_dma_enabled(tcm->hw_if->pdev) && tcm->hw_if->s3c64xx_sci) {
		tcm->hw_if->dma_mode = 1;
		tcm->hw_if->s3c64xx_sci->dma_mode = DMA_MODE;
	}
#endif

	LOGI("CDevice close\n");

	/* recover the max read write size */
	if (tcm->tcm_dev->max_wr_size != g_cdev_data.origin_max_wr_size)
		tcm->tcm_dev->max_wr_size = g_cdev_data.origin_max_wr_size;
	if (tcm->tcm_dev->max_rd_size != g_cdev_data.origin_max_rd_size)
		tcm->tcm_dev->max_rd_size = g_cdev_data.origin_max_rd_size;

	return 0;
}

/*
 * Declare the operations of TouchCom device file
 */
static const struct file_operations device_fops = {
	.owner = THIS_MODULE,
#ifdef USE_UNLOCKED_IOCTL
	.unlocked_ioctl = syna_cdev_ioctls,
#ifdef USE_COMPAT_IOCTL
	.compat_ioctl = syna_cdev_compat_ioctls,
#endif
#else
	.ioctl = syna_cdev_ioctls,
#endif
	.llseek = syna_cdev_llseek,
	.read = syna_cdev_read,
	.write = syna_cdev_write,
	.open = syna_cdev_open,
	.release = syna_cdev_release,
};
#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
/*
 * syna_cdev_update_report_queue()
 *
 * Push the selected data to the queue.
 * The followings are the format of reported data
 *
 *      [Bytes]     [ Description         ]
 *      -------------------------------------------
 *      [   0   ]  status of report code
 *      [ 1 - 2 ]  length of payload data
 *      [ 3 -N+3]  N bytes of payload data
 *
 * If the extra bytes are requested, the format will become
 *      [N+3]       the original packet with N bytes of payload
 *           [0-1]  crc bytes
 *           [ 2 ]  extra rc byte
 *
 * @param
 *    [ in] tcm:         the driver handle
 *    [ in] code:        report type
 *    [ in] pevent_data: report payload
 *
 * @return
 *    none.
 */
void syna_cdev_update_report_queue(struct syna_tcm *tcm,
		unsigned char code, struct tcm_buffer *pevent_data)
{
	int retval;
	struct tcm_dev *tcm_dev = tcm->tcm_dev;
	unsigned char *frame_buffer = NULL;
	unsigned int size = 0;
	unsigned short val;
	unsigned char *extrabytes = NULL;
	unsigned char *extraptr = NULL;
	int offset;
	const int header_size = 3;

	if (pevent_data == NULL) {
		LOGE("Invalid event data pointer\n");
		return;
	}

	size = pevent_data->data_length + header_size;
	if (g_cdev_data.extra_bytes > 0)
		size += g_cdev_data.extra_bytes;

	LOGD("Pushing data size:%d, total:%d\n",
		pevent_data->data_length, size);

	frame_buffer = (unsigned char *)syna_pal_mem_alloc(size,
					sizeof(unsigned char));
	if (!frame_buffer) {
		LOGE("Fail to allocate buffer, size: %d, data_length: %d\n",
			size, pevent_data->data_length);
		return;
	}

	if (g_cdev_data.extra_bytes > 0) {
		extrabytes = (unsigned char *)syna_pal_mem_alloc(
					g_cdev_data.extra_bytes,
					sizeof(unsigned char));
		if (!extrabytes) {
			syna_pal_mem_free((void *)frame_buffer);

			LOGE("Fail to allocate extra buffer, size: %d\n",
				g_cdev_data.extra_bytes);
			return;
		}
	}

	frame_buffer[0] = code;
	frame_buffer[1] = (unsigned char)pevent_data->data_length;
	frame_buffer[2] = (unsigned char)(pevent_data->data_length >> 8);

	if (pevent_data->data_length > 0) {
		retval = syna_pal_mem_cpy(&frame_buffer[header_size],
				(size - header_size),
				pevent_data->buf,
				pevent_data->data_length,
				pevent_data->data_length);
		if (retval < 0) {
			LOGE("Fail to copy data to buffer, size: %d\n",
				pevent_data->data_length);
			goto exit;
		}
	}

	if (g_cdev_data.extra_bytes >= TCM_MSG_CRC_LENGTH) {
		val = tcm_dev->msg_data.crc_bytes;
		extrabytes[0] = (unsigned char)val;
		extrabytes[1] = (unsigned char)(val >> 8);

		val = g_cdev_data.extra_bytes - TCM_MSG_CRC_LENGTH;
		if (val >= TCM_EXTRA_RC_LENGTH)
			extrabytes[TCM_MSG_CRC_LENGTH] = tcm_dev->msg_data.rc_byte;

		offset = pevent_data->data_length + header_size;
		extraptr = &frame_buffer[offset];
		retval = syna_pal_mem_cpy(extraptr,
				(size - offset),
				extrabytes,
				g_cdev_data.extra_bytes,
				g_cdev_data.extra_bytes);
		if (retval < 0) {
			LOGE("Fail to copy extra bytes to buffer\n");
			goto exit;
		}
	}

	LOGD("Pushing data starting by code 0x%02x to queue (size:%d)\n",
		code, size);

	retval = syna_cdev_insert_fifo(tcm, frame_buffer, size);
	if (retval < 0) {
		LOGE("Fail to push data to fifo\n");
		goto exit;
	}

	wake_up_interruptible(&(tcm->wait_frame));

exit:
	syna_pal_mem_free((void *)extrabytes);
	syna_pal_mem_free((void *)frame_buffer);
}
#endif
/*
 * syna_cdev_devnode()
 *
 * Provide the declaration of devtmpfs
 *
 * @param
 *    [ in] dev:  an instance of device
 *    [ in] mode: mode of created node
 *
 * @return
 *    the string of devtmpfs
 */
static char *syna_cdev_devnode(struct device *dev, umode_t *mode)
{
	if (!mode)
		return NULL;

	/* S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH */
	*mode = CHAR_DEVICE_MODE;

	return kasprintf(GFP_KERNEL, "%s", dev_name(dev));
}
/*
 * syna_cdev_create()
 *
 * Create a device node and register it with sysfs.
 *
 * @param
 *    [ in] tcm: the driver handle
 *    [ in] pdev: an instance of platform device
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_cdev_create(struct syna_tcm *tcm,
		struct platform_device *pdev)
{
	int retval = 0;
	struct class *device_class = NULL;
	struct device *device = NULL;
	static int cdev_major_num;

	syna_pal_mem_set(&g_cdev_data, 0x00, sizeof(struct syna_cdev_data));

	g_cdev_data.dev = pdev;

	tcm->device_class = NULL;
	tcm->device = NULL;

	tcm->is_attn_asserted = false;

	syna_pal_mutex_alloc(&g_cdev_data.mutex);
#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
	syna_pal_mutex_alloc(&g_cdev_data.queue_mutex);
#endif
	syna_tcm_buf_init(&g_cdev_data.buffer);

	if (cdev_major_num) {
		tcm->char_dev_num = MKDEV(cdev_major_num, 0);
		retval = register_chrdev_region(tcm->char_dev_num, 1,
				PLATFORM_DRIVER_NAME);
		if (retval < 0) {
			LOGE("Fail to register char device\n");
			goto err_register_chrdev_region;
		}
	} else {
		retval = alloc_chrdev_region(&tcm->char_dev_num, 0, 1,
				PLATFORM_DRIVER_NAME);
		if (retval < 0) {
			LOGE("Fail to allocate char device\n");
			goto err_alloc_chrdev_region;
		}

		cdev_major_num = MAJOR(tcm->char_dev_num);
	}

	cdev_init(&tcm->char_dev, &device_fops);
	tcm->char_dev.owner = THIS_MODULE;

	retval = cdev_add(&tcm->char_dev, tcm->char_dev_num, 1);
	if (retval < 0) {
		LOGE("Fail to add cdev_add\n");
		goto err_add_chardev;
	}

	device_class = class_create(THIS_MODULE, PLATFORM_DRIVER_NAME);
	if (IS_ERR(device_class)) {
		LOGE("Fail to create device class\n");
		retval = PTR_ERR(device_class);
		goto err_create_class;
	}

	device_class->devnode = syna_cdev_devnode;

	device = device_create(device_class, NULL,
			tcm->char_dev_num, NULL,
			CHAR_DEVICE_NAME"%d", MINOR(tcm->char_dev_num));
	if (IS_ERR(device)) {
		LOGE("Fail to create character device\n");
		retval = -ENOENT;
		goto err_create_device;
	}

	tcm->device_class = device_class;

	tcm->device = device;

	tcm->char_dev_ref_count = 0;
	tcm->proc_pid = 0;

	g_cdev_data.extra_bytes = 0;

#ifdef ENABLE_EXTERNAL_FRAME_PROCESS
	INIT_LIST_HEAD(&tcm->frame_fifo_queue);
	init_waitqueue_head(&tcm->wait_frame);
#endif
	syna_pal_mem_set(tcm->report_to_queue, 0, REPORT_TYPES);

#ifdef HAS_SYSFS_INTERFACE
	retval = syna_sysfs_create_dir(tcm, pdev);
	if (retval < 0) {
		LOGE("Fail to create sysfs dir\n");
		retval = -ENOTDIR;
		goto err_create_dir;
	}
#endif
	return 0;

#ifdef HAS_SYSFS_INTERFACE
err_create_dir:
	device_destroy(device_class, tcm->char_dev_num);
#endif
err_create_device:
	class_destroy(device_class);
err_create_class:
	cdev_del(&tcm->char_dev);
err_add_chardev:
	unregister_chrdev_region(tcm->char_dev_num, 1);
err_alloc_chrdev_region:
err_register_chrdev_region:
	return retval;
}
/*
 * syna_cdev_remove()
 *
 * Remove the allocate cdev device node and release the resource
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    none.
 */
void syna_cdev_remove(struct syna_tcm *tcm)
{
	if (!tcm) {
		LOGE("Invalid tcm driver handle\n");
		return;
	}

#ifdef HAS_SYSFS_INTERFACE
	syna_sysfs_remove_dir(tcm);
#endif

	syna_pal_mem_set(tcm->report_to_queue, 0, REPORT_TYPES);
	syna_cdev_clean_queue(tcm);
	syna_pal_mutex_free(&g_cdev_data.queue_mutex);

	tcm->char_dev_ref_count = 0;
	tcm->proc_pid = 0;

	if (tcm->device) {
		device_destroy(tcm->device_class, tcm->char_dev_num);
		class_destroy(tcm->device_class);
		cdev_del(&tcm->char_dev);
		unregister_chrdev_region(tcm->char_dev_num, 1);
	}

	syna_tcm_buf_release(&g_cdev_data.buffer);

	syna_pal_mutex_free(&g_cdev_data.mutex);

	tcm->device_class = NULL;
	tcm->device = NULL;

	g_cdev_data.dev = NULL;
}


