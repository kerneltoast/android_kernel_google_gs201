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
 * @file synaptics_touchcom_func_base.c
 *
 * This file implements generic and foundational functions supported in
 * Synaptics TouchComm communication protocol.
 *
 * The declarations are in synaptics_touchcom_func_base.h.
 */

#include "synaptics_touchcom_func_base.h"
#include "synaptics_touchcom_func_touch.h"


/*
 * syna_tcm_change_resp_read()
 *
 * Helper to change the default method to read the response packet.
 *
 * @param
 *    [in] tcm_dev: the device handle
 *    [in] request: resp reading method to change
 *                  set '0' or 'RESP_IN_ATTN' for ATTN-driven; otherwise,
 *                  assign a positive value standing for the polling time
 * @return
 *    none.
 */
void syna_tcm_change_resp_read(struct tcm_dev *tcm_dev, unsigned int request)
{
	if (request == RESP_IN_ATTN) {
		tcm_dev->msg_data.default_resp_reading = RESP_IN_ATTN;

		LOGI("Change default resp reading method by attn\n");
	} else {
		if (request < RESP_IN_POLLING)
			request = RESP_IN_POLLING;

		tcm_dev->msg_data.default_resp_reading = request;

		LOGI("Change default resp reading method by polling (%dms)\n",
			tcm_dev->msg_data.default_resp_reading);
	}
}

/*
 * syna_tcm_init_message_wrap()
 *
 * Initialize the TouchComm message wrapper interface.
 * Setup internal buffers and the relevant structures for command processing.
 *
 * @param
 *    [in] tcm_msg: message wrapper structure
 *    [in] resp_reading: default method to retrieve the resp data
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_init_message_wrap(struct tcm_message_data_blob *tcm_msg,
		unsigned int resp_reading)
{
	/* initialize internal buffers */
	syna_tcm_buf_init(&tcm_msg->in);
	syna_tcm_buf_init(&tcm_msg->out);
	syna_tcm_buf_init(&tcm_msg->temp);

	/* allocate the completion event for command processing */
	if (syna_pal_completion_alloc(&tcm_msg->cmd_completion) < 0) {
		LOGE("Fail to allocate cmd completion event\n");
		return -ERR_INVAL;
	}

	/* allocate the cmd_mutex for command protection */
	if (syna_pal_mutex_alloc(&tcm_msg->cmd_mutex) < 0) {
		LOGE("Fail to allocate cmd_mutex\n");
		return -ERR_INVAL;
	}

	/* allocate the rw_mutex for rw protection */
	if (syna_pal_mutex_alloc(&tcm_msg->rw_mutex) < 0) {
		LOGE("Fail to allocate rw_mutex\n");
		return -ERR_INVAL;
	}

	/* set default state of command_status  */
	ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);
	tcm_msg->command = CMD_NONE;
	tcm_msg->status_report_code = STATUS_IDLE;
	tcm_msg->payload_length = 0;
	tcm_msg->response_code = 0;
	tcm_msg->report_code = 0;
	tcm_msg->seq_toggle = 0;

	/* allocate the internal buffer.in at first */
	syna_tcm_buf_lock(&tcm_msg->in);

	if (syna_tcm_buf_alloc(&tcm_msg->in, MESSAGE_HEADER_SIZE) < 0) {
		LOGE("Fail to allocate memory for buf.in (size = %d)\n",
			MESSAGE_HEADER_SIZE);
		tcm_msg->in.buf_size = 0;
		tcm_msg->in.data_length = 0;
		syna_tcm_buf_unlock(&tcm_msg->in);
		return -ERR_NOMEM;
	}
	tcm_msg->in.buf_size = MESSAGE_HEADER_SIZE;

	syna_tcm_buf_unlock(&tcm_msg->in);

	tcm_msg->default_resp_reading = resp_reading;

	LOGI("Set default resp. reading method in %s\n",
		(resp_reading == RESP_IN_ATTN) ? "ATTN" : "Polling");

	/* initialize the features of message handling */
	tcm_msg->predict_reads = false;
	tcm_msg->predict_length = 0;
	tcm_msg->has_crc = false;
	tcm_msg->crc_bytes = 0;
	tcm_msg->has_extra_rc = false;
	tcm_msg->rc_byte = 0;

	return 0;
}

/*
 * syna_tcm_del_message_wrap()
 *
 * Remove message wrapper interface and internal buffers.
 * Call the function once the message wrapper is no longer needed.
 *
 * @param
 *    [in] tcm_msg: message wrapper structure
 *
 * @return
 *    none.
 */
static void syna_tcm_del_message_wrap(struct tcm_message_data_blob *tcm_msg)
{
	/* release the mutex */
	syna_pal_mutex_free(&tcm_msg->rw_mutex);
	syna_pal_mutex_free(&tcm_msg->cmd_mutex);

	/* release the completion event */
	syna_pal_completion_free(&tcm_msg->cmd_completion);

	/* release internal buffers  */
	syna_tcm_buf_release(&tcm_msg->temp);
	syna_tcm_buf_release(&tcm_msg->out);
	syna_tcm_buf_release(&tcm_msg->in);
}

/*
 * syna_tcm_allocate_device()
 *
 * Create the TouchCom core device handle.
 * This function must be called in order to allocate the main device handle,
 * structure syna_tcm_dev, which will be passed to all other operations and
 * functions within the entire source code.
 *
 * Meanwhile, caller has to prepare specific syna_tcm_hw_interface structure,
 * so that all the implemented functions can access hardware components
 * through syna_tcm_hw_interface.
 *
 * @param
 *    [out] ptcm_dev_ptr: a pointer to the device handle returned
 *    [ in] hw_if:        hardware-specific data on target platform
 *    [ in] resp_reading: default resp reading method
 *                        set 'RESP_IN_ATTN' to apply ATTN-driven method;
 *                        set 'RESP_IN_POLLING' to read in resp by polling
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_allocate_device(struct tcm_dev **ptcm_dev_ptr,
		struct syna_hw_interface *hw_if, unsigned int resp_reading)
{
	int retval = 0;
	struct tcm_dev *tcm_dev = NULL;

	if (!hw_if) {
		LOGE("Invalid parameter of hw_if\n");
		return -ERR_INVAL;
	}

	if ((!hw_if->ops_read_data) || (!hw_if->ops_write_data)) {
		LOGE("Invalid hw read write operation\n");
		return -ERR_INVAL;
	}

	LOGI("Prepare to allocate TouchComm core module ...\n");

	*ptcm_dev_ptr = NULL;

	/* allocate the core device handle */
	tcm_dev = (struct tcm_dev *)syna_pal_mem_alloc(
			1,
			sizeof(struct tcm_dev));
	if (!tcm_dev) {
		LOGE("Fail to create tcm device handle\n");
		return -ERR_NOMEM;
	}

	/* link to the given hardware data */
	tcm_dev->hw_if = hw_if;

	/* initialize the default setup */
	tcm_dev->max_rd_size = hw_if->bdata_io.rd_chunk_size;
	tcm_dev->max_wr_size = hw_if->bdata_io.wr_chunk_size;

	tcm_dev->write_message = NULL;
	tcm_dev->read_message = NULL;

	tcm_dev->cb_custom_touch_entity = NULL;
	tcm_dev->cbdata_touch_entity = NULL;
	tcm_dev->cb_custom_gesture = NULL;
	tcm_dev->cbdata_gesture = NULL;
	tcm_dev->cb_reset_occurrence = NULL;
	tcm_dev->cbdata_reset = NULL;

	tcm_dev->dev_mode = MODE_UNKNOWN;

	/* allocate internal buffers */
	syna_tcm_buf_init(&tcm_dev->report_buf);
	syna_tcm_buf_init(&tcm_dev->resp_buf);
	syna_tcm_buf_init(&tcm_dev->external_buf);
	syna_tcm_buf_init(&tcm_dev->touch_config);

	/* initialize the command wrapper interface */
	retval = syna_tcm_init_message_wrap(&tcm_dev->msg_data,
			resp_reading);
	if (retval < 0) {
		LOGE("Fail to initialize command interface\n");
		goto err_init_message_wrap;
	}

	/* return the created device handle */
	*ptcm_dev_ptr = tcm_dev;

	LOGI("TouchComm core module created, ver.: %d.%02d\n",
		(unsigned char)(SYNA_TCM_CORE_LIB_VERSION >> 8),
		(unsigned char)SYNA_TCM_CORE_LIB_VERSION & 0xff);

	return 0;

err_init_message_wrap:
	syna_tcm_buf_release(&tcm_dev->touch_config);
	syna_tcm_buf_release(&tcm_dev->external_buf);
	syna_tcm_buf_release(&tcm_dev->report_buf);
	syna_tcm_buf_release(&tcm_dev->resp_buf);

	tcm_dev->hw_if = NULL;

	syna_pal_mem_free((void *)tcm_dev);

	return retval;
}

/*
 * syna_tcm_remove_device()
 *
 * Remove the TouchCom core device handler.
 * This function must be invoked when the device is no longer needed.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    none.
 */
void syna_tcm_remove_device(struct tcm_dev *tcm_dev)
{
	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	/* release the command interface */
	syna_tcm_del_message_wrap(&tcm_dev->msg_data);

	/* release buffers */
	syna_tcm_buf_release(&tcm_dev->touch_config);
	syna_tcm_buf_release(&tcm_dev->external_buf);
	syna_tcm_buf_release(&tcm_dev->report_buf);
	syna_tcm_buf_release(&tcm_dev->resp_buf);

	tcm_dev->hw_if = NULL;

	/* release the device handle */
	syna_pal_mem_free((void *)tcm_dev);

	LOGI("TouchComm core module removed\n");
}
/*
 * syna_tcm_detect_device()
 *
 * Determine the type of device being connected, and distinguish which
 * version of TouchCom firmware running on the device.
 *
 * This function should be called as the first step of initialization.
 *
 * The start-up packet has an important data to identify the attached device
 * so it's recommended to process the startup packet in default.
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [ in] protocol: protocol to detect
 *                    0 - auto detection (default)
 *                    1 - TouchComm version 1
 *                    2 - TouchComm version 1
 *    [ in] startup:  request to handle the startup packet
 *                    set to 'True' if uncertainty
 *
 * @return
 *    on success, the current mode running on the device is returned;
 *    otherwise, negative value on error.
 */
int syna_tcm_detect_device(struct tcm_dev *tcm_dev, int protocol,
		bool startup)
{
	int retval = 0;
	unsigned char data[4] = { 0 };

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_dev->dev_mode = MODE_UNKNOWN;
	tcm_dev->protocol = 0;

	/* read in the bare data once willing to handle the startup packet */
	if (startup) {
		data[0] = 0x02;
		retval = syna_tcm_write(tcm_dev, &data[0], 1);
		if (retval < 0) {
			LOGE("Fail to write magic to bus\n");
			return retval;
		}

		retval = syna_tcm_read(tcm_dev,
				data, (unsigned int)sizeof(data));
		if (retval < 0) {
			LOGE("Fail to retrieve 4-byte data from bus\n");
			return retval;
		}

		LOGD("start-up data: %02x %02x %02x %02x\n",
				data[0], data[1], data[2], data[3]);
	}

	switch (protocol) {
	case 1: /* force to use v1 operations */
		if (startup)
			retval = syna_tcm_v1_detect(tcm_dev, data,
				(unsigned int)sizeof(data));
		else
			syna_tcm_v1_set_ops(tcm_dev);
		LOGI("Communicate to TouchComm ver.1 forcibly\n");
		break;
	case 2: /* force to use v2 operations */
		if (startup)
			retval = syna_tcm_v2_detect(tcm_dev, data,
				(unsigned int)sizeof(data));
		else
			syna_tcm_v2_set_ops(tcm_dev);
		LOGI("Communicate to TouchComm ver.2 forcibly\n");
		break;

	default:
		if (!startup) {
			LOGE("Fail to detect device without startup packet\n");
			return -ERR_INVAL;
		}
		/* perform the protocol detection */
		retval = syna_tcm_v2_detect(tcm_dev,
			data, (unsigned int)sizeof(data));
		if (retval < 0)
			retval = syna_tcm_v1_detect(tcm_dev,
				data, (unsigned int)sizeof(data));
		if (retval < 0) {
			LOGE("Fail to detect TouchCom device, %02x %02x %02x %02x\n",
				data[0], data[1], data[2], data[3]);
			return retval;
		}

		break;
	}

	if ((!tcm_dev->write_message) || (!tcm_dev->read_message)) {
		LOGE("Invalid TouchCom R/W operations\n");
		return -ERR_NODEV;
	}

	/* skip the recognition of device mode for some specific scenarios */
	if (!startup)
		return 0;

	/* check the running mode */
	switch (tcm_dev->dev_mode) {
	case MODE_APPLICATION_FIRMWARE:
		LOGI("Device in Application FW, build id: %d, %s\n",
			tcm_dev->packrat_number,
			tcm_dev->id_info.part_number);
		break;
	case MODE_BOOTLOADER:
	case MODE_TDDI_BOOTLOADER:
		LOGI("Device in Bootloader\n");
		break;
	case MODE_ROMBOOTLOADER:
		LOGI("Device in ROMBoot Bootloader\n");
		break;
	case MODE_MULTICHIP_TDDI_BOOTLOADER:
		LOGI("Device in multi-chip TDDI Bootloader\n");
		break;
	default:
		LOGW("Found TouchCom device, but unknown mode:0x%02x detected\n",
			tcm_dev->dev_mode);
		break;
	}

	retval = tcm_dev->dev_mode;
	return retval;
}

/*
 * syna_tcm_get_event_data()
 *
 * Helper to read TouchComm messages when ATTN signal is asserted.
 * After returning, the ATTN signal should be no longer asserted.
 *
 * The 'code' returned will guide the caller on the next action.
 * For example, do touch reporting once returned code is equal to REPORT_TOUCH.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [out] code:    received report code
 *    [out] data:    a user buffer for data returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_event_data(struct tcm_dev *tcm_dev,
		unsigned char *code, struct tcm_buffer *data)
{
	int retval = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (!code) {
		LOGE("Invalid parameter\n");
		return -ERR_INVAL;
	}

	/* retrieve the event data */
	retval = tcm_dev->read_message(tcm_dev,
			code);
	if (retval < 0) {
		LOGE("Fail to read messages\n");
		return retval;
	}

	/* exit if no buffer provided */
	if (!data)
		goto exit;

	/* if gathering a report, copy to the user buffer */
	if ((*code >= REPORT_IDENTIFY) && (*code != STATUS_INVALID)) {
		if (tcm_dev->report_buf.data_length == 0)
			goto exit;

		syna_tcm_buf_lock(&tcm_dev->report_buf);

		retval = syna_tcm_buf_copy(data, &tcm_dev->report_buf);
		if (retval < 0) {
			LOGE("Fail to copy data, report type: %x\n", *code);
			syna_tcm_buf_unlock(&tcm_dev->report_buf);
			goto exit;
		}

		syna_tcm_buf_unlock(&tcm_dev->report_buf);
	}

	/* if gathering a response, copy to the user buffer */
	if ((*code > STATUS_IDLE) && (*code <= STATUS_ERROR)) {
		if (tcm_dev->resp_buf.data_length == 0)
			goto exit;

		syna_tcm_buf_lock(&tcm_dev->resp_buf);

		retval = syna_tcm_buf_copy(data, &tcm_dev->resp_buf);
		if (retval < 0) {
			LOGE("Fail to copy data, status code: %x\n", *code);
			syna_tcm_buf_unlock(&tcm_dev->resp_buf);
			goto exit;
		}

		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
	}

exit:
	return retval;
}

/*
 * syna_tcm_identify()
 *
 * Implement the standard command code to request an IDENTIFY report.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [out] id_info: the identification info packet returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_identify(struct tcm_dev *tcm_dev,
		struct tcm_identification_info *id_info)
{
	int retval = 0;
	unsigned char resp_code;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_IDENTIFY,
			NULL,
			0,
			0,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n", CMD_IDENTIFY);
		goto exit;
	}

	if (id_info == NULL)
		goto show_info;

	/* copy identify info to caller */
	retval = syna_pal_mem_cpy((unsigned char *)id_info,
			sizeof(struct tcm_identification_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			MIN(sizeof(*id_info), tcm_dev->resp_buf.data_length));
	if (retval < 0) {
		LOGE("Fail to copy identify info to caller\n");
		goto exit;
	}

show_info:
	LOGI("TCM Fw mode: 0x%02x, TCM ver.: %d\n",
		tcm_dev->id_info.mode, tcm_dev->id_info.version);

	tcm_dev->dev_mode = tcm_dev->id_info.mode;

exit:
	return retval;
}

/*
 * syna_tcm_reset()
 *
 * Implement the standard command code, which is used to perform a sw reset
 * immediately. After a successful reset, an IDENTIFY report is received to
 * indicate that device is ready.
 *
 * Caller shall be aware that the firmware will be reloaded after reset.
 * Therefore, if expecting that a different firmware version is loaded, please
 * do app firmware setup after reset.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_reset(struct tcm_dev *tcm_dev)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int board_setting;
	unsigned int resp_handling;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	resp_handling = tcm_dev->msg_data.default_resp_reading;

	/* gather the board settings of reset delay time */
	board_setting = tcm_dev->hw_if->bdata_rst.reset_delay_ms;
	if (board_setting == 0)
		board_setting = RESET_DELAY_MS;

	/* select the proper period to handle the resp of reset */
	if (resp_handling != RESP_IN_ATTN) {
		if (board_setting > resp_handling) {
			resp_handling = board_setting;
			LOGI("Use board settings %dms to poll resp of reset\n",
				resp_handling);
		}
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_RESET,
			NULL,
			0,
			0,
			&resp_code,
			resp_handling);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n", CMD_RESET);
		goto exit;
	}

	/* current device mode is expected to be updated
	 * because identification report will be received after reset
	 */
	tcm_dev->dev_mode = tcm_dev->id_info.mode;
	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGI("Device mode 0x%02X running after reset\n",
			tcm_dev->dev_mode);
	}

	retval = 0;
exit:
	return retval;
}

/*
 * syna_tcm_enable_report()
 *
 * Implement the application fw command code to enable or disable the
 * specific TouchComm report.
 *
 * @param
 *    [ in] tcm_dev:     the device handle
 *    [ in] report_code: the requested report code being generated
 *    [ in] en:          '1' for enabling; '0' for disabling
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_enable_report(struct tcm_dev *tcm_dev,
		unsigned char report_code, bool en)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned char command;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	command = (en) ? CMD_ENABLE_REPORT : CMD_DISABLE_REPORT;

	retval = tcm_dev->write_message(tcm_dev,
			command,
			&report_code,
			1,
			1,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x to %s 0x%02x report\n",
			command, (en)?"enable":"disable", report_code);
		goto exit;
	}

	if (resp_code != STATUS_OK) {
		LOGE("Fail to %s 0x%02x report, resp_code:%x\n",
			(en) ? "enable" : "disable", report_code, resp_code);
	} else {
		LOGD("Report 0x%x %s\n", report_code,
			(en) ? "enabled" : "disabled");
	}

exit:
	return retval;
}

/*
 * syna_tcm_run_display_rom_bootloader_fw()
 *
 * Requests to run the display rombootloader firmware.
 * Once the completion of switching display rombootloader firmware, an
 * IDENTIFY report will be received.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_run_display_rom_bootloader_fw(struct tcm_dev *tcm_dev,
		unsigned int fw_switch_delay)
{
	int retval = 0;
	unsigned char resp_code;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_REBOOT_TO_DISPLAY_ROM_BOOTLOADER,
			NULL,
			0,
			0,
			&resp_code,
			fw_switch_delay);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_REBOOT_TO_DISPLAY_ROM_BOOTLOADER);
		goto exit;
	}

	if (!IS_DISPLAY_ROM_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		LOGE("Fail to enter display rom bootloader, mode: %x\n",
			tcm_dev->dev_mode);
		retval = -ERR_TCMMSG;
		goto exit;
	}

	LOGI("Display ROM Bootloader (mode 0x%x) activated\n",
		tcm_dev->dev_mode);

	retval = 0;

exit:
	return retval;
}

/*
 * syna_tcm_run_rom_bootloader_fw()
 *
 * Requests to run the rombootloader firmware.
 * Once the completion of switching rombootloader firmware, an IDENTIFY report
 * will be received.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_run_rom_bootloader_fw(struct tcm_dev *tcm_dev,
		unsigned int fw_switch_delay)
{
	int retval = 0;
	unsigned char resp_code;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_REBOOT_TO_ROM_BOOTLOADER,
			NULL,
			0,
			0,
			&resp_code,
			fw_switch_delay);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_REBOOT_TO_ROM_BOOTLOADER);
		goto exit;
	}

	if (!IS_ROM_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		LOGE("Fail to enter rom bootloader, mode: %x\n",
			tcm_dev->dev_mode);
		retval = -ERR_TCMMSG;
		goto exit;
	}

	LOGI("ROM Bootloader (mode 0x%x) activated\n",
		tcm_dev->dev_mode);

	retval = 0;

exit:
	return retval;
}

/*
 * syna_tcm_run_bootloader_fw()
 *
 * Requests to run the bootloader firmware.
 * Once the completion of switching bootloader firmware, an IDENTIFY report
 * will be received.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_run_bootloader_fw(struct tcm_dev *tcm_dev,
		unsigned int fw_switch_delay)
{
	int retval = 0;
	unsigned char resp_code;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_RUN_BOOTLOADER_FIRMWARE,
			NULL,
			0,
			0,
			&resp_code,
			fw_switch_delay);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_RUN_BOOTLOADER_FIRMWARE);
		goto exit;
	}

	if (!IS_BOOTLOADER_MODE(tcm_dev->dev_mode)) {
		LOGE("Fail to enter bootloader, mode: %x\n",
			tcm_dev->dev_mode);
		retval = -ERR_TCMMSG;
		goto exit;
	}

	LOGI("Bootloader Firmware (mode 0x%x) activated\n",
		tcm_dev->dev_mode);

	retval = 0;

exit:
	return retval;
}

/*
 * syna_tcm_run_application_fw()
 *
 * Requests to run the application firmware.
 * Once the completion of switching application firmware, an IDENTIFY report
 * will be received.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_run_application_fw(struct tcm_dev *tcm_dev,
		unsigned int fw_switch_delay)
{
	int retval = 0;
	unsigned char resp_code;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_RUN_APPLICATION_FIRMWARE,
			NULL,
			0,
			0,
			&resp_code,
			fw_switch_delay);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_RUN_APPLICATION_FIRMWARE);
		goto exit;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGW("Fail to enter application fw, mode: %x\n",
			tcm_dev->dev_mode);
		retval = -ERR_TCMMSG;
		goto exit;
	}

	LOGI("Application Firmware (mode 0x%x) activated\n",
		tcm_dev->dev_mode);

	retval = 0;

exit:
	return retval;
}

/*
 * syna_tcm_switch_fw_mode()
 *
 * Request to switch the firmware mode running on.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] mode:    target firmware mode
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_switch_fw_mode(struct tcm_dev *tcm_dev,
		unsigned char mode, unsigned int fw_switch_delay)
{
	int retval = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	switch (mode) {
	case MODE_APPLICATION_FIRMWARE:
		retval = syna_tcm_run_application_fw(tcm_dev,
				fw_switch_delay);
		if (retval < 0) {
			LOGE("Fail to switch to application mode\n");
			goto exit;
		}
		break;
	case MODE_BOOTLOADER:
	case MODE_TDDI_BOOTLOADER:
	case MODE_TDDI_HDL_BOOTLOADER:
	case MODE_MULTICHIP_TDDI_BOOTLOADER:
		retval = syna_tcm_run_bootloader_fw(tcm_dev,
				fw_switch_delay);
		if (retval < 0) {
			LOGE("Fail to switch to bootloader mode\n");
			goto exit;
		}
		break;
	case MODE_ROMBOOTLOADER:
		retval = syna_tcm_run_rom_bootloader_fw(tcm_dev,
				fw_switch_delay);
		if (retval < 0) {
			LOGE("Fail to switch to rom bootloader mode\n");
			goto exit;
		}
		break;
	case MODE_DISPLAY_ROMBOOTLOADER:
		retval = syna_tcm_run_display_rom_bootloader_fw(tcm_dev,
				fw_switch_delay);
		if (retval < 0) {
			LOGE("Fail to switch to display rom bootloader mode\n");
			goto exit;
		}
		break;
	default:
		LOGE("Invalid firmware mode requested\n");
		retval = -ERR_INVAL;
		goto exit;
	}

	retval = 0;

exit:
	return retval;
}

/*
 * syna_tcm_get_boot_info()
 *
 * Implement the bootloader command code to request the bootloader
 * information.
 *
 * @param
 *    [ in] tcm_dev:   the device handle
 *    [out] boot_info: the boot info packet returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_boot_info(struct tcm_dev *tcm_dev,
		struct tcm_boot_info *boot_info)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int resp_data_len = 0;
	unsigned int copy_size;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_BOOT_INFO,
			NULL,
			0,
			0,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_GET_BOOT_INFO);
		goto exit;
	}

	resp_data_len = tcm_dev->resp_buf.data_length;
	copy_size = MIN(sizeof(struct tcm_boot_info), resp_data_len);

	/* save the boot_info */
	retval = syna_pal_mem_cpy((unsigned char *)&tcm_dev->boot_info,
			sizeof(struct tcm_boot_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			copy_size);
	if (retval < 0) {
		LOGE("Fail to copy boot info\n");
		goto exit;
	}

	if (boot_info == NULL)
		goto exit;

	/* copy boot_info to caller */
	retval = syna_pal_mem_cpy((unsigned char *)boot_info,
			sizeof(struct tcm_boot_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			copy_size);
	if (retval < 0) {
		LOGE("Fail to copy boot info to caller\n");
		goto exit;
	}

exit:
	return retval;
}

/*
 * syna_tcm_get_app_info()
 *
 * Implement the application fw command code to request an application
 * information from device.
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [out] app_info: the application info packet returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_app_info(struct tcm_dev *tcm_dev,
		struct tcm_application_info *app_info)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int app_status;
	unsigned int resp_data_len = 0;
	unsigned int copy_size;
	struct tcm_application_info *info;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_APPLICATION_INFO,
			NULL,
			0,
			0,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_GET_APPLICATION_INFO);
		goto exit;
	}

	resp_data_len = tcm_dev->resp_buf.data_length;
	copy_size = MIN(sizeof(tcm_dev->app_info), resp_data_len);

	info = &tcm_dev->app_info;

	/* save the app_info */
	retval = syna_pal_mem_cpy((unsigned char *)info,
			sizeof(struct tcm_application_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			copy_size);
	if (retval < 0) {
		LOGE("Fail to copy application info\n");
		goto exit;
	}

	if (app_info == NULL)
		goto show_info;

	/* copy app_info to caller */
	retval = syna_pal_mem_cpy((unsigned char *)app_info,
			sizeof(struct tcm_application_info),
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			copy_size);
	if (retval < 0) {
		LOGE("Fail to copy application info to caller\n");
		goto exit;
	}

show_info:
	app_status = syna_pal_le2_to_uint(tcm_dev->app_info.status);

	if (app_status == APP_STATUS_BAD_APP_CONFIG) {
		LOGE("Bad application firmware, status: 0x%x\n", app_status);
		retval = -ERR_TCMMSG;
		goto exit;
	} else if (app_status != APP_STATUS_OK) {
		LOGE("Incorrect application status, 0x%x\n", app_status);
		retval = -ERR_TCMMSG;
		goto exit;
	}

	tcm_dev->max_objects = syna_pal_le2_to_uint(info->max_objects);
	tcm_dev->max_x = syna_pal_le2_to_uint(info->max_x);
	tcm_dev->max_y = syna_pal_le2_to_uint(info->max_y);

	tcm_dev->cols = syna_pal_le2_to_uint(info->num_of_image_cols);
	tcm_dev->rows = syna_pal_le2_to_uint(info->num_of_image_rows);
	syna_pal_mem_cpy((unsigned char *)tcm_dev->config_id,
			MAX_SIZE_CONFIG_ID,
			info->customer_config_id,
			MAX_SIZE_CONFIG_ID,
			MAX_SIZE_CONFIG_ID);

	LOGD("App info version: %d, status: %d\n",
		syna_pal_le2_to_uint(info->version), app_status);
	LOGD("App info: max_objs: %d, max_x:%d, max_y: %d, trx: %dx%d\n",
		tcm_dev->max_objects, tcm_dev->max_x, tcm_dev->max_y,
		tcm_dev->rows, tcm_dev->cols);

exit:
	return retval;
}

/*
 * syna_tcm_get_static_config()
 *
 * Implement the application fw command code to retrieve the contents of
 * the static configuration.
 *
 * The size of static configuration is available from the app info.
 *
 * @param
 *    [ in] tcm_dev:   the device handle
 *    [out] buf:       buffer stored the static configuration
 *    [ in] buf_size:  the size of given buffer
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_static_config(struct tcm_dev *tcm_dev,
		unsigned char *buf, unsigned int buf_size)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int size;
	struct tcm_application_info *app_info;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	app_info = &tcm_dev->app_info;

	size = syna_pal_le2_to_uint(app_info->static_config_size);

	if (size > buf_size) {
		LOGE("Invalid buffer input, given size: %d (actual: %d)\n",
			buf_size, size);
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_STATIC_CONFIG,
			NULL,
			0,
			0,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_GET_STATIC_CONFIG);
		goto exit;
	}

	if (buf == NULL)
		goto exit;

	/* copy app_info to caller */
	retval = syna_pal_mem_cpy((unsigned char *)buf,
			buf_size,
			tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			tcm_dev->resp_buf.data_length);
	if (retval < 0) {
		LOGE("Fail to copy static config data to caller\n");
		goto exit;
	}

exit:
	return retval;
}

/*
 * syna_tcm_set_static_config()
 *
 * Implement the application fw command code to set the contents of
 * the static configuration. When the write is completed, the device will
 * restart touch sensing with the new settings.
 *
 * The size of static configuration is available from the app info.
 *
 * @param
 *    [ in] tcm_dev:          the device handle
 *    [ in] config_data:      the data of static configuration
 *    [ in] config_data_size: the size of given data
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_set_static_config(struct tcm_dev *tcm_dev,
		unsigned char *config_data, unsigned int config_data_size)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int size;
	struct tcm_application_info *app_info;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	app_info = &tcm_dev->app_info;

	size = syna_pal_le2_to_uint(app_info->static_config_size);

	if (size != config_data_size) {
		LOGE("Invalid static config size, given: %d (actual: %d)\n",
			config_data_size, size);
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_SET_STATIC_CONFIG,
			config_data,
			config_data_size,
			config_data_size,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_SET_STATIC_CONFIG);
		goto exit;
	}

	retval = 0;
exit:
	return retval;
}

/*
 * syna_tcm_get_dynamic_config()
 *
 * Implement the application fw command code to get the value from the a single
 * field of the dynamic configuration.
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [ in] id:       target field id
 *    [out] value:    the value returned
 *    [ in] delay_ms_resp: delay time for response reading.
 *                         a positive value presents the time for polling;
 *                         or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_dynamic_config(struct tcm_dev *tcm_dev,
		unsigned char id, unsigned short *value,
		unsigned int delay_ms_resp)
{
	int retval = 0;
	unsigned char out;
	unsigned char resp_code;
	unsigned int resp_handling;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	resp_handling = tcm_dev->msg_data.default_resp_reading;
	if (resp_handling != delay_ms_resp)
		resp_handling = delay_ms_resp;

	out = (unsigned char)id;

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_DYNAMIC_CONFIG,
			&out,
			sizeof(out),
			sizeof(out),
			&resp_code,
			resp_handling);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x to get dynamic field 0x%x\n",
			CMD_GET_DYNAMIC_CONFIG, (unsigned char)id);
		goto exit;
	}

	/* return dynamic config data */
	if (tcm_dev->resp_buf.data_length < 2) {
		LOGE("Invalid resp data size, %d\n",
			tcm_dev->resp_buf.data_length);
		retval = -ERR_INVAL;
		goto exit;
	}

	*value = (unsigned short)syna_pal_le2_to_uint(tcm_dev->resp_buf.buf);

	LOGD("Get %d from dynamic field 0x%x\n", *value, id);

	retval = 0;

exit:
	return retval;
}

/*
 * syna_tcm_set_dynamic_config()
 *
 * Implement the application fw command code to set the specified value to
 * the selected field of the dynamic configuration.
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [ in] id:       target field id
 *    [ in] value:    the value to the selected field
 *    [ in] delay_ms_resp: delay time for response reading.
 *                          a positive value presents the time for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_set_dynamic_config(struct tcm_dev *tcm_dev,
		unsigned char id, unsigned short value,
		unsigned int delay_ms_resp)
{
	int retval = 0;
	unsigned char out[3];
	unsigned char resp_code;
	unsigned int resp_handling;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	resp_handling = tcm_dev->msg_data.default_resp_reading;
	if (resp_handling != delay_ms_resp)
		resp_handling = delay_ms_resp;

	LOGD("Set %d to dynamic field 0x%x\n", value, id);

	out[0] = (unsigned char)id;
	out[1] = (unsigned char)value;
	out[2] = (unsigned char)(value >> 8);

	retval = tcm_dev->write_message(tcm_dev,
			CMD_SET_DYNAMIC_CONFIG,
			out,
			sizeof(out),
			sizeof(out),
			&resp_code,
			resp_handling);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x to set %d to field 0x%x\n",
			CMD_SET_DYNAMIC_CONFIG, value, (unsigned char)id);
		goto exit;
	}

	retval = 0;

exit:
	return retval;
}

/*
 * syna_tcm_rezero()
 *
 * Implement the application fw command code to force the device to rezero its
 * baseline estimate.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_rezero(struct tcm_dev *tcm_dev)
{
	int retval = 0;
	unsigned char resp_code;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_REZERO,
			NULL,
			0,
			0,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_REZERO);
		goto exit;
	}

	retval = 0;
exit:
	return retval;
}

/*
 * syna_tcm_set_config_id()
 *
 * Implement the application fw command code to set the 16-byte config id
 * defined in the app info.
 *
 * @param
 *    [ in] tcm_dev:   the device handle
 *    [ in] config_id: config id to be set
 *    [ in] size:      size of input data
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_set_config_id(struct tcm_dev *tcm_dev,
		unsigned char *config_id, unsigned int size)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int config_id_len = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	config_id_len = sizeof(tcm_dev->app_info.customer_config_id);

	if (size != config_id_len) {
		LOGE("Invalid config id input, given size: %d (%d)\n",
			size, config_id_len);
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_SET_CONFIG_ID,
			config_id,
			size,
			size,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_SET_CONFIG_ID);
		goto exit;
	}

	retval = 0;
exit:
	return retval;
}

/*
 * syna_tcm_sleep()
 *
 * Implement the application fw command code to put the device into low power
 * deep sleep mode or the normal active mode.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] en:      '1' to low power deep sleep mode; '0' to active mode
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_sleep(struct tcm_dev *tcm_dev, bool en)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned char command;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	command = (en) ? CMD_ENTER_DEEP_SLEEP : CMD_EXIT_DEEP_SLEEP;

	retval = tcm_dev->write_message(tcm_dev,
			command,
			NULL,
			0,
			0,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%x\n", command);
		goto exit;
	}

	retval = 0;
exit:
	return retval;
}

/*
 * syna_tcm_get_features()
 *
 * Implement the application fw command code to query the supported features.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [out] info:    the features description packet returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_features(struct tcm_dev *tcm_dev,
		struct tcm_features_info *info)
{
	int retval = 0;
	unsigned char resp_code;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_GET_FEATURES,
			NULL,
			0,
			0,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_GET_FEATURES);
		goto exit;
	}

	if (info == NULL)
		goto exit;

	/* copy features_info to caller */
	retval = syna_pal_mem_cpy((unsigned char *)info,
		sizeof(struct tcm_features_info),
		tcm_dev->resp_buf.buf,
		tcm_dev->resp_buf.buf_size,
		MIN(sizeof(*info), tcm_dev->resp_buf.data_length));
	if (retval < 0) {
		LOGE("Fail to copy features_info to caller\n");
		goto exit;
	}

exit:
	return retval;
}

/*
 * syna_tcm_run_production_test()
 *
 * Implement the application fw command code to request the device to run
 * the production test.
 *
 * Production tests are listed at enum test_code (PID$).
 *
 * @param
 *    [ in] tcm_dev:    the device handle
 *    [ in] test_item:  the requested testing item
 *    [out] tdata:      testing data returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_run_production_test(struct tcm_dev *tcm_dev,
		unsigned char test_item, struct tcm_buffer *tdata)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned char test_code;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGE("Device is not in application fw mode, mode: %x\n",
			tcm_dev->dev_mode);
		return -ERR_INVAL;
	}

	test_code = (unsigned char)test_item;

	retval = tcm_dev->write_message(tcm_dev,
			CMD_PRODUCTION_TEST,
			&test_code,
			1,
			1,
			&resp_code,
			tcm_dev->msg_data.default_resp_reading);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n",
			CMD_PRODUCTION_TEST);
		goto exit;
	}

	if (tdata == NULL)
		goto exit;

	/* copy testing data to caller */
	retval = syna_tcm_buf_copy(tdata, &tcm_dev->resp_buf);
	if (retval < 0) {
		LOGE("Fail to copy testing data\n");
		goto exit;
	}
exit:
	return retval;
}
/*
 * syna_tcm_send_command()
 *
 * Helper to execute the custom command.
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] command:        TouchComm command
 *    [ in] payload:        data payload, if any
 *    [ in] payload_length: length of data payload, if any
 *    [ in] total_length:   length of total payload
 *    [out] resp_code:      response code returned
 *    [out] resp:           buffer to store the response data
 *    [ in] delay_ms_resp: delay time for response reading.
 *                          a positive value presents the time for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_send_command(struct tcm_dev *tcm_dev, unsigned char command,
		unsigned char *payload, unsigned int payload_length,
		unsigned int total_length, unsigned char *code,
		struct tcm_buffer *resp, unsigned int delay_ms_resp)
{
	int retval = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (!code) {
		LOGE("Invalid parameter\n");
		return -ERR_INVAL;
	}

	retval = tcm_dev->write_message(tcm_dev,
			command,
			payload,
			total_length,
			payload_length,
			code,
			delay_ms_resp);
	if (retval < 0)
		LOGE("Fail to run command 0x%02x\n", command);

	LOGD("Status code returned: 0x%02x\n", *code);

	/* exit if no buffer provided */
	if (!resp)
		goto exit;

	/* if gathering a report, copy to the user buffer */
	if ((*code >= REPORT_IDENTIFY) && (*code != STATUS_INVALID)) {
		if (tcm_dev->report_buf.data_length == 0)
			goto exit;

		syna_tcm_buf_lock(&tcm_dev->report_buf);

		if (syna_tcm_buf_copy(resp, &tcm_dev->report_buf) < 0) {
			LOGE("Fail to copy data, report type: %x\n",
				*code);
			syna_tcm_buf_unlock(&tcm_dev->report_buf);
			retval = -ERR_NOMEM;
			goto exit;
		}

		syna_tcm_buf_unlock(&tcm_dev->report_buf);
	}

	/* if gathering a response, copy to the user buffer */
	if ((*code > STATUS_IDLE) && (*code <= STATUS_ERROR)) {
		if (tcm_dev->resp_buf.data_length == 0)
			goto exit;

		syna_tcm_buf_lock(&tcm_dev->resp_buf);

		if (syna_tcm_buf_copy(resp, &tcm_dev->resp_buf) < 0) {
			LOGE("Fail to copy resp data, status code: %x\n",
				*code);
			syna_tcm_buf_unlock(&tcm_dev->resp_buf);
			retval = -ERR_NOMEM;
			goto exit;
		}

		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
	}

exit:
	return retval;
}

/*
 * syna_tcm_enable_predict_reading()
 *
 * Helper to enable the feature of predict reading.
 *
 * This feature aims to read in all data at one bus transferring.
 * In contrast to the predict reading, standard reads require two transfers
 * to separately read the header and the payload data.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] en:      '1' to low power deep sleep mode; '0' to active mode
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_enable_predict_reading(struct tcm_dev *tcm_dev, bool en)
{
	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_dev->msg_data.predict_reads = en;
	tcm_dev->msg_data.predict_length = 0;

	LOGI("Predicted reading is %s\n",
		(en) ? "enabled":"disabled");

	return 0;
}

/*
 * syna_tcm_set_reset_occurrence_callback()
 *
 * Set up callback function once an unexpected identify report is received.
 *
 * This callback can help the shell implementations to handle unexpected event.
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [ in] p_cb:     the pointer of callback function
 *    [ in] p_cbdata: pointer to caller data
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_set_reset_occurrence_callback(struct tcm_dev *tcm_dev,
		tcm_reset_occurrence_callback_t p_cb, void *p_cbdata)
{
	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_dev->cb_reset_occurrence = p_cb;
	tcm_dev->cbdata_reset = p_cbdata;

	LOGI("reset callback enabled\n");

	return 0;
}

/*
 * syna_tcm_smart_bridge_reset()
 *
 * Implement the specific command code to reset the smart bride entirely.
 * After a successful reset, wait at least 200 ms before reading the IDENTIFY
 * report.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] delay:   specific delay time to read in the response
 *                   set '0' to apply the default delay time which is 200 ms
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_smart_bridge_reset(struct tcm_dev *tcm_dev, int delay)
{
	int retval = 0;
	unsigned char resp_code;
	unsigned int resp_handling;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	resp_handling = tcm_dev->msg_data.default_resp_reading;

	/* select the proper period to handle the resp of reset */
	if ((resp_handling != RESP_IN_ATTN) && (delay != RESP_IN_ATTN)) {
		resp_handling = delay;
		if (resp_handling < 200)
			resp_handling = 200;
	}

	retval = tcm_dev->write_message(tcm_dev,
			CMD_SMART_BRIDGE_RESET,
			NULL,
			0,
			0,
			&resp_code,
			resp_handling);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x\n", CMD_SMART_BRIDGE_RESET);
		goto exit;
	}

	/* current device mode is expected to be updated
	 * because identification report will be received after reset
	 */
	tcm_dev->dev_mode = tcm_dev->id_info.mode;
	if (IS_NOT_APP_FW_MODE(tcm_dev->dev_mode)) {
		LOGI("Device mode 0x%02X running after reset\n",
			tcm_dev->dev_mode);
	}

	retval = 0;
exit:
	return retval;
}

