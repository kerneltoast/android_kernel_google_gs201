// SPDX-License-Identifier: GPL-2.0
/*
 * Synaptics TCM touchscreen driver
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
 * @file synaptics_touchcom_core_v1.c
 *
 * This file implements the TouchComm version 1 command-response protocol.
 */

#include "synaptics_touchcom_core_dev.h"

#define TCM_V1_MESSAGE_MARKER 0xa5
#define TCM_V1_MESSAGE_PADDING 0x5a

/*
 * @section: Header of TouchComm v1 Message Packet
 *
 * The 4-byte header in the TouchComm v1 packet
 */
struct tcm_v1_message_header {
	union {
		struct {
			unsigned char marker;
			unsigned char code;
			unsigned char length[2];
		};
		unsigned char data[MESSAGE_HEADER_SIZE];
	};
};

/*
 * syna_tcm_v1_update_crc()
 *
 * Helper to update the CRC bytes from the internal in_buf
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static void syna_tcm_v1_update_crc(struct tcm_dev *tcm_dev)
{
	struct tcm_message_data_blob *tcm_msg = NULL;
	unsigned int offset;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	tcm_msg = &tcm_dev->msg_data;

	if (!tcm_msg->has_crc)
		return;

	if (tcm_msg->payload_length == 0)
		return;

	syna_tcm_buf_lock(&tcm_msg->in);

	offset = MESSAGE_HEADER_SIZE + tcm_msg->payload_length;
	if (tcm_msg->in.buf_size <= offset + 1)
		return;

	if (tcm_msg->has_crc) {
		/* copy crc bytes which are followed by EOM (0x5a) */
		tcm_msg->crc_bytes = (unsigned short)syna_pal_le2_to_uint(
			&tcm_msg->in.buf[offset + 1]); /* skip EOM */
		LOGD("CRC retrieved: 0x%04X\n", tcm_msg->crc_bytes);

		if (tcm_msg->has_extra_rc) {
			/* copy an extra rc byte which is appended after crc */
			offset += TCM_MSG_CRC_LENGTH;
			if (tcm_msg->in.buf_size >= offset + 1) {
				tcm_msg->rc_byte =
					tcm_msg->in.buf[offset + 1];
				LOGD("Extra RC byte retrieved: 0x%02X\n",
					tcm_msg->rc_byte);
			}

		}
	}

	syna_tcm_buf_unlock(&tcm_msg->in);
}

/*
 * syna_tcm_v1_set_max_rw_size()
 *
 * Configure the max length for message reading and writing.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    none.
 */
static int syna_tcm_v1_set_max_rw_size(struct tcm_dev *tcm_dev)
{
	unsigned int wr_size = 0;
	struct tcm_identification_info *id_info;
	unsigned int build_id = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	id_info = &tcm_dev->id_info;
	build_id = syna_pal_le4_to_uint(id_info->build_id);

	if (build_id == 0) {
		LOGE("Invalid identify report stored\n");
		return -ERR_INVAL;
	}

	wr_size = syna_pal_le2_to_uint(id_info->max_write_size);

	if (wr_size == 0) {
		LOGE("Invalid max write size from identify report\n");
		return -ERR_INVAL;
	}

	/* set max write size */
	if (wr_size != tcm_dev->max_wr_size) {
		if (tcm_dev->max_wr_size == 0)
			tcm_dev->max_wr_size = wr_size;
		else
			tcm_dev->max_wr_size =
				MIN(wr_size, tcm_dev->max_wr_size);

		LOGD("Set max write length to %d bytes\n",
			tcm_dev->max_wr_size);
	}

	tcm_dev->max_rd_size = tcm_dev->max_rd_size;

	LOGD("Set max read length to %d bytes\n", tcm_dev->max_rd_size);

	return 0;
}

/*
 * syna_tcm_v1_parse_idinfo()
 *
 * Copy the given data to the identification info structure
 * and parse the basic information, e.g. fw build id.
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [ in] data:     data buffer
 *    [ in] size:     size of given data buffer
 *    [ in] data_len: length of actual data
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_v1_parse_idinfo(struct tcm_dev *tcm_dev,
		unsigned char *data, unsigned int size, unsigned int data_len)
{
	int retval;
	unsigned int build_id = 0;
	struct tcm_identification_info *id_info;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if ((!data) || (data_len == 0)) {
		LOGE("Invalid given data buffer\n");
		return -ERR_INVAL;
	}

	id_info = &tcm_dev->id_info;

	retval = syna_pal_mem_cpy((unsigned char *)id_info,
			sizeof(struct tcm_identification_info),
			data,
			size,
			MIN(sizeof(*id_info), data_len));
	if (retval < 0) {
		LOGE("Fail to copy identification info\n");
		return retval;
	}

	build_id = syna_pal_le4_to_uint(id_info->build_id);

	if (tcm_dev->packrat_number != build_id)
		tcm_dev->packrat_number = build_id;

	LOGI("TCM Fw mode: 0x%02x\n", id_info->mode);

	tcm_dev->dev_mode = id_info->mode;

	return 0;
}

/*
 * syna_tcm_v1_dispatch_report()
 *
 * Handle the TouchCom report packet being received.
 *
 * If it's an identify report, parse the identification packet and signal
 * the command completion just in case.
 * Otherwise, copy the data from internal buffer.in to internal buffer.report
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    none.
 */
static void syna_tcm_v1_dispatch_report(struct tcm_dev *tcm_dev)
{
	int retval;
	struct tcm_message_data_blob *tcm_msg = NULL;
	syna_pal_completion_t *cmd_completion = NULL;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	tcm_msg = &tcm_dev->msg_data;
	cmd_completion = &tcm_msg->cmd_completion;

	tcm_msg->report_code = tcm_msg->status_report_code;

	if (tcm_msg->payload_length == 0) {
		tcm_dev->report_buf.data_length = 0;
		goto exit;
	}

	/* store the received report into the internal buffer.report */
	syna_tcm_buf_lock(&tcm_dev->report_buf);

	retval = syna_tcm_buf_alloc(&tcm_dev->report_buf,
			tcm_msg->payload_length);
	if (retval < 0) {
		LOGE("Fail to allocate memory for internal buf.report\n");
		syna_tcm_buf_unlock(&tcm_dev->report_buf);
		goto exit;
	}

	syna_tcm_buf_lock(&tcm_msg->in);

	retval = syna_pal_mem_cpy(tcm_dev->report_buf.buf,
			tcm_dev->report_buf.buf_size,
			&tcm_msg->in.buf[MESSAGE_HEADER_SIZE],
			tcm_msg->in.buf_size - MESSAGE_HEADER_SIZE,
			tcm_msg->payload_length);
	if (retval < 0) {
		LOGE("Fail to copy payload to buf_report\n");
		syna_tcm_buf_unlock(&tcm_msg->in);
		syna_tcm_buf_unlock(&tcm_dev->report_buf);
		goto exit;
	}

	tcm_dev->report_buf.data_length = tcm_msg->payload_length;

	syna_tcm_buf_unlock(&tcm_msg->in);
	syna_tcm_buf_unlock(&tcm_dev->report_buf);

	/* The identify report may be resulted from reset or fw mode switching
	 */
	if (tcm_msg->report_code == REPORT_IDENTIFY) {

		syna_tcm_buf_lock(&tcm_msg->in);

		retval = syna_tcm_v1_parse_idinfo(tcm_dev,
				&tcm_msg->in.buf[MESSAGE_HEADER_SIZE],
				tcm_msg->in.buf_size - MESSAGE_HEADER_SIZE,
				tcm_msg->payload_length);
		if (retval < 0) {
			LOGE("Fail to identify device\n");
			syna_tcm_buf_unlock(&tcm_msg->in);
			return;
		}

		syna_tcm_buf_unlock(&tcm_msg->in);

		/* in case, the identify info packet is caused by the command */
		if (ATOMIC_GET(tcm_msg->command_status) == CMD_STATE_BUSY) {
			switch (tcm_msg->command) {
			case CMD_RESET:
				LOGD("Reset by command 0x%02X\n",
					tcm_msg->command);
				tcm_msg->response_code = STATUS_OK;
				ATOMIC_SET(tcm_msg->command_status,
					CMD_STATE_IDLE);
				syna_pal_completion_complete(cmd_completion);
				goto exit;
			case CMD_REBOOT_TO_ROM_BOOTLOADER:
			case CMD_RUN_BOOTLOADER_FIRMWARE:
			case CMD_RUN_APPLICATION_FIRMWARE:
			case CMD_ENTER_PRODUCTION_TEST_MODE:
			case CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE:
				tcm_msg->response_code = STATUS_OK;
				ATOMIC_SET(tcm_msg->command_status,
					CMD_STATE_IDLE);
				syna_pal_completion_complete(cmd_completion);
				goto exit;
			default:
				LOGI("Unexpected 0x%02X report received\n",
					REPORT_IDENTIFY);
				ATOMIC_SET(tcm_msg->command_status,
					CMD_STATE_ERROR);
				syna_pal_completion_complete(cmd_completion);
				goto exit;
			}
		} else {
			LOGN("Device has been reset\n");
			/* invoke callback to handle unexpected reset if doesn't
			 * result from command
			 */
			if (tcm_dev->cb_reset_occurrence)
				tcm_dev->cb_reset_occurrence(
					tcm_dev->cbdata_reset);
		}
	}

exit:
	return;
}

/*
 * syna_tcm_v1_dispatch_response()
 *
 * Handle the response packet.
 *
 * Copy the data from internal buffer.in to internal buffer.resp,
 * and then signal the command completion.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    none.
 */
static void syna_tcm_v1_dispatch_response(struct tcm_dev *tcm_dev)
{
	int retval;
	struct tcm_message_data_blob *tcm_msg = NULL;
	syna_pal_completion_t *cmd_completion = NULL;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	tcm_msg = &tcm_dev->msg_data;
	cmd_completion = &tcm_msg->cmd_completion;

	tcm_msg->response_code = tcm_msg->status_report_code;

	if (ATOMIC_GET(tcm_msg->command_status) != CMD_STATE_BUSY)
		return;

	if (tcm_msg->payload_length == 0) {
		tcm_dev->resp_buf.data_length = tcm_msg->payload_length;
		ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);
		goto exit;
	}

	/* copy the received resp data into the internal buffer.resp */
	syna_tcm_buf_lock(&tcm_dev->resp_buf);

	retval = syna_tcm_buf_alloc(&tcm_dev->resp_buf,
			tcm_msg->payload_length);
	if (retval < 0) {
		LOGE("Fail to allocate memory for internal buf.resp\n");
		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
		goto exit;
	}

	syna_tcm_buf_lock(&tcm_msg->in);

	retval = syna_pal_mem_cpy(tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			&tcm_msg->in.buf[MESSAGE_HEADER_SIZE],
			tcm_msg->in.buf_size - MESSAGE_HEADER_SIZE,
			tcm_msg->payload_length);
	if (retval < 0) {
		LOGE("Fail to copy payload to internal resp_buf\n");
		syna_tcm_buf_unlock(&tcm_msg->in);
		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
		goto exit;
	}

	tcm_dev->resp_buf.data_length = tcm_msg->payload_length;

	syna_tcm_buf_unlock(&tcm_msg->in);
	syna_tcm_buf_unlock(&tcm_dev->resp_buf);

	ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);

exit:
	syna_pal_completion_complete(cmd_completion);
}


/*
 * syna_tcm_v1_read()
 *
 * Read in a TouchCom packet from device.
 *
 * @param
 *    [ in] tcm_dev:    the device handle
 *    [ in] rd_length:  number of reading bytes;
 *                      '0' means to read the message header only
 *    [in/out] buf:     pointer to a buffer which is stored the retrieved data
 *    [out] buf_size:   size of the buffer pointed
 *    [ in] extra_crc:  flag to read in extra crc bytes
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_v1_read(struct tcm_dev *tcm_dev, unsigned int rd_length,
		unsigned char *buf, unsigned int buf_size, bool extra_crc)
{
	int retval;
	unsigned int max_rd_size;
	int retry;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if (rd_length == 0)
		return 0;

	if (rd_length > buf_size) {
		LOGE("Invalid read length, len: %d, buf_size: %d\n",
			rd_length, buf_size);
		return -ERR_INVAL;
	}

	max_rd_size = tcm_dev->max_rd_size;

	if ((max_rd_size != 0) && (rd_length > max_rd_size)) {
		LOGE("Invalid read length, len: %d, max_rd_size: %d\n",
			rd_length, max_rd_size);
		return -ERR_INVAL;
	}

	/* read in the message header from device
	 * will do retry if the packet is not expected
	 */
	for (retry = 0; retry < 10; retry++) {
		retval = syna_tcm_read(tcm_dev,
				buf,
				rd_length
				);
		if (retval < 0) {
			LOGE("Fail to read %d bytes to device\n", rd_length);
			goto exit;
		}

		/* check the message header */
		if (buf[0] == TCM_V1_MESSAGE_MARKER)
			break;

		LOGE("Incorrect header marker, 0x%02x (retry:%d)\n",
			buf[0], retry);

		retval = -ERR_TCMMSG;
		syna_pal_sleep_us(RD_RETRY_US_MIN, RD_RETRY_US_MAX);
	}

exit:
	return retval;
}

/*
 * syna_tcm_v1_write()
 *
 * Construct the TouchCom v1 packet and send it to device.
 *
 * @param
 *    [ in] tcm_dev:     the device handle
 *    [ in] command:     command code
 *    [ in] payload:     data payload if any
 *    [ in] payload_len: length of data payload if any
 *    [ in] crc_append:  flag to send extra crc bytes
 *    [ in] extra_crc:   two bytes crc value
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_v1_write(struct tcm_dev *tcm_dev, unsigned char command,
		unsigned char *payload, unsigned int payload_len,
		bool extra_crc, unsigned short crc)
{
	int retval = 0;
	struct tcm_message_data_blob *tcm_msg = NULL;
	int size, buf_size;
	unsigned char crc16[TCM_MSG_CRC_LENGTH] = { 0 };

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;

	syna_tcm_buf_lock(&tcm_msg->out);

	/* allocate the space storing the written data */
	buf_size = payload_len + 3;
	if (extra_crc) {
		crc16[0] = (unsigned char) crc & 0xff;
		crc16[1] = (unsigned char) (crc >> 8);

		buf_size += sizeof(crc16);
	}

	retval = syna_tcm_buf_alloc(&tcm_msg->out, buf_size);
	if (retval < 0) {
		LOGE("Fail to allocate memory for internal buf.out\n");
		goto exit;
	}

	if (command != CMD_CONTINUE_WRITE) {
		/* construct the command packet
		 * size = 1-byte command + 2-byte length + payload
		 */
		size = payload_len + 3;

		tcm_msg->out.buf[0] = command;
		tcm_msg->out.buf[1] = (unsigned char)payload_len;
		tcm_msg->out.buf[2] = (unsigned char)(payload_len >> 8);

		if (payload_len > 0) {
			retval = syna_pal_mem_cpy(&tcm_msg->out.buf[3],
					tcm_msg->out.buf_size - 3,
					payload,
					payload_len,
					payload_len
					);
			if (retval < 0) {
				LOGE("Fail to copy payload\n");
				goto exit;
			}
		}
	} else {
		/* construct the continued writes packet
		 * size = 1-byte continued write + payload
		 */
		size = payload_len + 1;

		tcm_msg->out.buf[0] = CMD_CONTINUE_WRITE;

		retval = syna_pal_mem_cpy(&tcm_msg->out.buf[1],
				tcm_msg->out.buf_size - 1,
				payload,
				payload_len,
				payload_len
				);
		if (retval < 0) {
			LOGE("Fail to copy continued write\n");
			goto exit;
		}
	}

	/* append the crc16 value at the end */
	if (extra_crc) {
		retval = syna_pal_mem_cpy(&tcm_msg->out.buf[size],
				tcm_msg->out.buf_size - size,
				crc16,
				sizeof(crc16),
				sizeof(crc16)
				);
		if (retval < 0) {
			LOGE("Fail to append crc16\n");
			goto exit;
		}

		size += sizeof(crc16);
	}

	/* write command packet to the device */
	retval = syna_tcm_write(tcm_dev,
			tcm_msg->out.buf,
			size
			);
	if (retval < 0) {
		LOGE("Fail to write %d bytes to device\n", size);
		goto exit;
	}

exit:
	syna_tcm_buf_unlock(&tcm_msg->out);

	return retval;
}

/*
 * syna_tcm_v1_continued_read()
 *
 * The remaining data payload is read in continuously until the end of data.
 * All the retrieved data is appended to the internal buffer.in.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] length:  remaining length of payload data
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_v1_continued_read(struct tcm_dev *tcm_dev,
		unsigned int length)
{
	int retval = 0;
	unsigned char code;
	unsigned int idx;
	unsigned int offset;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int total_length;
	unsigned int remaining_length;
	struct tcm_message_data_blob *tcm_msg = NULL;
	bool last = false;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;

	if ((length == 0) || (tcm_msg->payload_length == 0))
		return 0;

	if ((length & 0xffff) == 0xffff) {
		LOGE("Invalid length to read\n");
		return -ERR_INVAL;
	}

	/* continued read packet contains the header, payload, and a padding */
	total_length = MESSAGE_HEADER_SIZE + tcm_msg->payload_length + 1;
	/* length to read, remember a padding at the end */
	remaining_length = length + 1;

	/* read extra crc if supported */
	if (tcm_msg->has_crc) {
		total_length += TCM_MSG_CRC_LENGTH;
		remaining_length += TCM_MSG_CRC_LENGTH;
	}
	if (tcm_msg->has_extra_rc) {
		total_length += TCM_EXTRA_RC_LENGTH;
		remaining_length += TCM_EXTRA_RC_LENGTH;
	}
	  /* read in one more EOM when crc bytes appending */
	if (tcm_msg->has_crc || tcm_msg->has_extra_rc) {
		total_length += 1;
		remaining_length += 1;
	}

	syna_tcm_buf_lock(&tcm_msg->in);

	/* in case the current buf.in is smaller than requested size */
	retval = syna_tcm_buf_realloc(&tcm_msg->in,
			total_length + 1);
	if (retval < 0) {
		LOGE("Fail to allocate memory for internal buf_in\n");
		syna_tcm_buf_unlock(&tcm_msg->in);
		return -ERR_NOMEM;
	}

	/* available chunk space for payload =
	 *     total chunk size - (marker + status code)
	 */
	if (tcm_dev->max_rd_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = tcm_dev->max_rd_size - 2;

	chunks = syna_pal_ceil_div(remaining_length, chunk_space);
	chunks = chunks == 0 ? 1 : chunks;

	offset = MESSAGE_HEADER_SIZE + (tcm_msg->payload_length - length);

	syna_tcm_buf_lock(&tcm_msg->temp);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		last = ((idx + 1) == chunks);

		if (xfer_length == 1) {
			tcm_msg->in.buf[offset] = TCM_V1_MESSAGE_PADDING;
			offset += xfer_length;
			remaining_length -= xfer_length;
			continue;
		}

		/* delay for the bus turnaround time */
		syna_pal_sleep_us(TAT_DELAY_US_MIN, TAT_DELAY_US_MAX);

		/* allocate the internal temp buffer */
		retval = syna_tcm_buf_alloc(&tcm_msg->temp,
				xfer_length + 2);
		if (retval < 0) {
			LOGE("Fail to allocate memory for internal buf.temp\n");
			goto exit;
		}
		/* retrieve data from the bus
		 * data should include header marker and status code
		 */
		retval = syna_tcm_v1_read(tcm_dev,
				xfer_length + 2,
				tcm_msg->temp.buf,
				tcm_msg->temp.buf_size,
				(tcm_msg->has_crc) && last);
		if (retval < 0) {
			LOGE("Fail to read %d bytes from device\n",
				xfer_length + 2);
			goto exit;
		}

		tcm_msg->temp.data_length = xfer_length + 2;

		/* check the data content */
		code = tcm_msg->temp.buf[1];

		if (code != STATUS_CONTINUED_READ) {
			LOGE("Incorrect status code 0x%02x at %d out of %d\n",
					code, idx, chunks);
			retval = -ERR_TCMMSG;
			goto exit;
		}

		/* copy data from internal buffer.temp to buffer.in */
		retval = syna_pal_mem_cpy(&tcm_msg->in.buf[offset],
				tcm_msg->in.buf_size - offset,
				&tcm_msg->temp.buf[2],
				tcm_msg->temp.buf_size - 2,
				xfer_length);
		if (retval < 0) {
			LOGE("Fail to copy payload\n");
			goto exit;
		}

		offset += xfer_length;
		remaining_length -= xfer_length;
	}

exit:
	syna_tcm_buf_unlock(&tcm_msg->temp);
	syna_tcm_buf_unlock(&tcm_msg->in);

	return retval;
}

/*
 * syna_tcm_v1_read_message()
 *
 * Read in a TouchCom packet from device.
 * The packet including its payload is read in from device and stored in
 * the internal buffer.resp or buffer.report based on the code received.
 *
 * @param
 *    [ in] tcm_dev:            the device handle
 *    [out] status_report_code: status code or report code received
 *
 * @return
 *    0 or positive value on success; otherwise, on error.
 */
static int syna_tcm_v1_read_message(struct tcm_dev *tcm_dev,
		unsigned char *status_report_code)
{
	int retval = 0;
	struct tcm_v1_message_header *header;
	struct tcm_message_data_blob *tcm_msg = NULL;
	syna_pal_mutex_t *rw_mutex = NULL;
	syna_pal_completion_t *cmd_completion = NULL;
	unsigned int len = 0;
	bool do_predict = false;
	unsigned int tmp_len;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;
	rw_mutex = &tcm_msg->rw_mutex;
	cmd_completion = &tcm_msg->cmd_completion;

	/* predict reading is applied when doing report streaming only */
	do_predict = (ATOMIC_GET(tcm_msg->command_status) == CMD_STATE_IDLE);

	if (status_report_code)
		*status_report_code = STATUS_INVALID;

	syna_pal_mutex_lock(rw_mutex);

	syna_tcm_buf_lock(&tcm_msg->in);

	/* read in the message header */
	len = MESSAGE_HEADER_SIZE;

	/* if predict read enabled, plus the predicted length
	 * and determine the length to read
	 */
	if (tcm_msg->predict_reads && do_predict) {
		if (tcm_msg->predict_length > 0) {
			len += tcm_msg->predict_length;
			if (tcm_msg->has_crc)
				len += TCM_MSG_CRC_LENGTH;
			if (tcm_msg->has_extra_rc)
				len += TCM_EXTRA_RC_LENGTH;
			/* end of message byte */
			len += 1;
		}
	}

	/* ensure the size of in.buf, do re-allocate if needed */
	if (len > tcm_msg->in.buf_size) {
		retval = syna_tcm_buf_alloc(&tcm_msg->in, len);
		if (retval < 0) {
			LOGE("Fail to allocate memory for buf_in\n");
			syna_tcm_buf_unlock(&tcm_msg->in);

			tcm_msg->status_report_code = STATUS_INVALID;
			tcm_msg->payload_length = 0;
			goto exit;
		}
	}

	/* read in the message from device */
	retval = syna_tcm_v1_read(tcm_dev,
			len,
			tcm_msg->in.buf,
			tcm_msg->in.buf_size,
			false);
	if (retval < 0) {
		LOGE("Fail to read message header from device\n");
		syna_tcm_buf_unlock(&tcm_msg->in);

		tcm_msg->status_report_code = STATUS_INVALID;
		tcm_msg->payload_length = 0;
		goto exit;
	}

	/* check the message header */
	header = (struct tcm_v1_message_header *)tcm_msg->in.buf;

	tcm_msg->status_report_code = header->code;

	tcm_msg->payload_length = syna_pal_le2_to_uint(header->length);

	if ((tcm_msg->status_report_code != STATUS_IDLE) ||
		(tcm_msg->payload_length > 0)) {
		LOGD("Status code: 0x%02x, length: %d (%02x %02x %02x %02x)\n",
			tcm_msg->status_report_code, tcm_msg->payload_length,
			header->data[0], header->data[1], header->data[2],
			header->data[3]);
	}

	syna_tcm_buf_unlock(&tcm_msg->in);

	/* do dispatch if this's the full message */
	if (tcm_msg->payload_length == 0)
		goto do_dispatch;

	/* calculate the remaining length for continued reads
	 *
	 * if enabling predict read or having extra bytes to read,
	 * the value of 'len' shall be larger than the message header;
	 * so removing the size of pre-read was the actual size to read.
	 *
	 * otherwise, the total length of payload shall be retrieved
	 */
	tmp_len = len - MESSAGE_HEADER_SIZE;
	if (len > MESSAGE_HEADER_SIZE)
		len = (tcm_msg->payload_length > tmp_len) ?
			(tcm_msg->payload_length - tmp_len) : 0;
	else
		len = tcm_msg->payload_length;

	/* retrieve the remaining data, if any */
	retval = syna_tcm_v1_continued_read(tcm_dev, len);
	if (retval < 0) {
		LOGE("Fail to do continued read\n");
		goto exit;
	}

do_dispatch:
	/* refill the header for dispatching */
	syna_tcm_buf_lock(&tcm_msg->in);

	tcm_msg->in.buf[0] = TCM_V1_MESSAGE_MARKER;
	tcm_msg->in.buf[1] = tcm_msg->status_report_code;
	tcm_msg->in.buf[2] = (unsigned char)tcm_msg->payload_length;
	tcm_msg->in.buf[3] = (unsigned char)(tcm_msg->payload_length >> 8);

	syna_tcm_buf_unlock(&tcm_msg->in);

	/* update crc data if needed */
	syna_tcm_v1_update_crc(tcm_dev);

	/* duplicate the data to external buffer */
	syna_tcm_buf_lock(&tcm_dev->external_buf);
	if (tcm_msg->payload_length > 0) {
		retval = syna_tcm_buf_alloc(&tcm_dev->external_buf,
				tcm_msg->payload_length);
		if (retval < 0) {
			LOGE("Fail to allocate memory, external_buf invalid\n");
			syna_tcm_buf_unlock(&tcm_dev->external_buf);
			goto exit;
		}
		retval = syna_pal_mem_cpy(&tcm_dev->external_buf.buf[0],
			tcm_msg->payload_length,
			&tcm_msg->in.buf[MESSAGE_HEADER_SIZE],
			tcm_msg->in.buf_size - MESSAGE_HEADER_SIZE,
			tcm_msg->payload_length);
		if (retval < 0) {
			LOGE("Fail to copy data to external buffer\n");
			syna_tcm_buf_unlock(&tcm_dev->external_buf);
			goto exit;
		}
	}
	tcm_dev->external_buf.data_length = tcm_msg->payload_length;
	syna_tcm_buf_unlock(&tcm_dev->external_buf);

	if ((tcm_msg->status_report_code <= STATUS_ERROR) ||
		(tcm_msg->status_report_code == STATUS_INVALID)) {
		switch (tcm_msg->status_report_code) {
		case STATUS_OK:
			break;
		case STATUS_CONTINUED_READ:
			LOGE("Out-of-sync continued read\n");
			retval = -ERR_TCMMSG;
			goto exit;
		case STATUS_IDLE:
			retval = 0;
			goto exit;
		default:
			LOGE("Incorrect Status code, 0x%02x\n",
				tcm_msg->status_report_code);
			break;
		}
	}

	/* process the retrieved packet */
	if (tcm_msg->status_report_code >= REPORT_IDENTIFY)
		syna_tcm_v1_dispatch_report(tcm_dev);
	else
		syna_tcm_v1_dispatch_response(tcm_dev);

	/* copy the status report code to caller */
	if (status_report_code)
		*status_report_code = tcm_msg->status_report_code;

	/* update the length for predict reading */
	if (tcm_msg->predict_reads && do_predict) {
		if (tcm_dev->max_rd_size == 0)
			tcm_msg->predict_length = tcm_msg->payload_length;
		else
			tcm_msg->predict_length = MIN(tcm_msg->payload_length,
				tcm_dev->max_rd_size - MESSAGE_HEADER_SIZE - 1);

		if (tcm_msg->status_report_code < REPORT_IDENTIFY)
			tcm_msg->predict_length = 0;
	}

	retval = 0;

exit:
	/* raise the completion event when errors out */
	if (retval < 0) {
		if (ATOMIC_GET(tcm_msg->command_status) == CMD_STATE_BUSY) {
			ATOMIC_SET(tcm_msg->command_status, CMD_STATE_ERROR);
			syna_pal_completion_complete(cmd_completion);
		}
	}

	syna_pal_mutex_unlock(rw_mutex);

	return retval;
}

/*
 * syna_tcm_v1_write_message()
 *
 * Write message including command and its payload to TouchCom device.
 * Then, the response of the command generated by the device will be
 * read in and stored in internal buffer.resp.
 *
 * @param
 *    [ in] tcm_dev:       the device handle
 *    [ in] command:       TouchComm command
 *    [ in] payload:       data payload, if any
 *    [ in] length_total:   length of total payload
 *    [ in] length:         length of payload data, if any
 *    [out] resp_code:     response code returned
 *    [ in] delay_ms_resp: delay time for response reading.
 *                         a positive value presents the time for polling;
 *                         or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 *
 * @return
 *    0 or positive value on success; otherwise, on error.
 */
static int syna_tcm_v1_write_message(struct tcm_dev *tcm_dev,
	unsigned char command, unsigned char *payload,
	unsigned int length_total, unsigned int length,
	unsigned char *resp_code, unsigned int delay_ms_resp)
{
	int retval = 0;
	unsigned int idx;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int remaining_length;
	int timeout = 0;
	int polling_ms = 0;
	struct tcm_message_data_blob *tcm_msg = NULL;
	syna_pal_mutex_t *cmd_mutex = NULL;
	syna_pal_mutex_t *rw_mutex = NULL;
	syna_pal_completion_t *cmd_completion = NULL;
	bool has_irq_ctrl = false;
	bool in_polling = false;
	bool last = false;
	unsigned char tmp;
	unsigned short crc16 = 0xFFFF;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;
	cmd_mutex = &tcm_msg->cmd_mutex;
	rw_mutex = &tcm_msg->rw_mutex;
	cmd_completion = &tcm_msg->cmd_completion;

	if (resp_code)
		*resp_code = STATUS_INVALID;

	/* indicate which mode is used */
	in_polling = (delay_ms_resp != RESP_IN_ATTN);

	/* irq control is enabled only when the operations is implemented
	 * and the current status of irq is enabled.
	 * do not enable irq if it is disabled by someone.
	 */
	has_irq_ctrl = (bool)(tcm_dev->hw_if->ops_enable_irq != NULL);
	has_irq_ctrl &= tcm_dev->hw_if->bdata_attn.irq_enabled;

	/* disable irq when using polling mode */
	if (has_irq_ctrl && in_polling && tcm_dev->hw_if->ops_enable_irq)
		tcm_dev->hw_if->ops_enable_irq(tcm_dev->hw_if, false);

	syna_pal_mutex_lock(cmd_mutex);

	syna_pal_mutex_lock(rw_mutex);

	ATOMIC_SET(tcm_msg->command_status, CMD_STATE_BUSY);

	/* reset the command completion */
	syna_pal_completion_reset(cmd_completion);

	tcm_msg->command = command;

	remaining_length = length;

	if (length != length_total) {
		LOGD("Command: 0x%02x, payload length: %d / %d\n",
			command, length, length_total);
	} else {
		LOGD("Command: 0x%02x, payload length: %d\n",
			command, length);
	}

	/* calculate the crc if supported */
	if (tcm_msg->has_crc) {
		crc16 = syna_tcm_crc16(&command, 1, crc16);
		tmp = (unsigned char)length & 0xff;
		crc16 = syna_tcm_crc16(&tmp, 1, crc16);
		tmp = (unsigned char)(length >> 8) & 0xff;
		crc16 = syna_tcm_crc16(&tmp, 1, crc16);
		if (length > 0)
			crc16 = syna_tcm_crc16(payload, length, crc16);

		LOGD("CRC to write: 0x%04X\n", crc16);
	}

	/* available space for payload = total size - command byte */
	if (tcm_dev->max_wr_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = tcm_dev->max_wr_size - 1;

	chunks = syna_pal_ceil_div(remaining_length, chunk_space);
	chunks = chunks == 0 ? 1 : chunks;

	/* send out command packets
	 *
	 * separate into several sub-packets if the overall size is over
	 * than the maximum write size.
	 */
	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		last = ((idx + 1) == chunks);

		if (idx == 0) {
			retval = syna_tcm_v1_write(tcm_dev,
					tcm_msg->command,
					&payload[0],
					xfer_length,
					(tcm_msg->has_crc) && last,
					crc16);
		} else {
			retval = syna_tcm_v1_write(tcm_dev,
					CMD_CONTINUE_WRITE,
					&payload[idx * chunk_space],
					xfer_length,
					(tcm_msg->has_crc) && last,
					crc16);
		}

		if (retval < 0) {
			LOGE("Fail to write %d bytes to device\n",
				xfer_length);
			syna_pal_mutex_unlock(rw_mutex);
			goto exit;
		}

		remaining_length -= xfer_length;

		if (chunks > 1)
			syna_pal_sleep_us(WR_DELAY_US_MIN, WR_DELAY_US_MAX);
	}

	syna_pal_mutex_unlock(rw_mutex);

	/* handle the command response
	 *
	 * assuming to select the polling mode, the while-loop below will
	 * repeatedly read in the respose data based on the given polling
	 * time; otherwise, wait until receiving a completion event from
	 * interrupt thread.
	 */
	timeout = 0;
	if (!in_polling)
		polling_ms = CMD_RESPONSE_TIMEOUT_MS;
	else
		polling_ms = delay_ms_resp;

	do {
		if (!in_polling) {
			/* wait for the completion event triggered by read_message from irq */
			retval = syna_pal_completion_wait_for(cmd_completion, polling_ms);
		} else {
			/* otherwise, keep in polling */
			syna_pal_sleep_ms(polling_ms);
			ATOMIC_SET(tcm_msg->command_status, CMD_STATE_BUSY);

			/* retrieve the message packet back */
			retval = syna_tcm_v1_read_message(tcm_dev, NULL);
			/* keep in polling if still not having a valid resp */
			if (retval < 0)
				syna_pal_completion_reset(cmd_completion);
		}

		/* break the loop once a resp was ready */
		if (ATOMIC_GET(tcm_msg->command_status) == CMD_STATE_IDLE)
			goto check_response;

		timeout += polling_ms;

	} while (timeout < CMD_RESPONSE_TIMEOUT_MS);

	/* check the status of response data
	 * according to the touchcomm spec, each command message
	 * should have an associated response message.
	 */
check_response:
	if (ATOMIC_GET(tcm_msg->command_status) != CMD_STATE_IDLE) {
		if (timeout >= CMD_RESPONSE_TIMEOUT_MS) {
			LOGE("Timed out wait for response of command 0x%02x\n",
				command);
			retval = -ERR_TIMEDOUT;
			goto exit;
		} else {
			LOGE("Fail to get valid response of command 0x%02x\n",
				command);
			retval = -ERR_TCMMSG;
			goto exit;
		}
	}

	/* copy response code to the caller */
	if (resp_code)
		*resp_code = tcm_msg->status_report_code;

	retval = 0;
	if (tcm_msg->response_code != STATUS_OK) {
		LOGE("Received code 0x%02x (command 0x%02x)\n",
			tcm_msg->status_report_code, tcm_msg->command);
		retval = -ERR_TCMMSG;
	}

exit:
	tcm_msg->command = CMD_NONE;

	ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);

	syna_pal_mutex_unlock(cmd_mutex);

	/* recovery the irq if using polling mode */
	if (has_irq_ctrl && in_polling && tcm_dev->hw_if->ops_enable_irq)
		tcm_dev->hw_if->ops_enable_irq(tcm_dev->hw_if, true);

	return retval;
}
/*
 * syna_tcm_v1_set_ops()
 *
 * Assign read / write operations
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    none
 */
void syna_tcm_v1_set_ops(struct tcm_dev *tcm_dev)
{
	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	/* expose the read / write operations */
	tcm_dev->read_message = syna_tcm_v1_read_message;
	tcm_dev->write_message = syna_tcm_v1_write_message;
	tcm_dev->set_max_rw_size = syna_tcm_v1_set_max_rw_size;

	tcm_dev->msg_data.predict_length = 0;
	tcm_dev->protocol = TOUCHCOMM_V1;
}
/*
 * syna_tcm_v1_detect()
 *
 * Function to process the startup packet of TouchComm V1 firmware
 *
 * For TouchCom v1 protocol, the given raw data must start with a specific
 * maker code. If so, read the remaining packet from TouchCom device.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] data:    raw 4-byte data
 *    [ in] size:    length of input data in bytes
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_v1_detect(struct tcm_dev *tcm_dev, unsigned char *data,
		unsigned int size)
{
	int retval;
	struct tcm_v1_message_header *header;
	struct tcm_message_data_blob *tcm_msg = NULL;
	unsigned int payload_length = 0;
	unsigned char resp_code = 0;
	unsigned short default_crc = 0x5a5a;
	unsigned short default_rc = 0x5a;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if ((!data) || (size != MESSAGE_HEADER_SIZE)) {
		LOGE("Invalid parameters\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;

	header = (struct tcm_v1_message_header *)data;

	if (header->marker != TCM_V1_MESSAGE_MARKER)
		return -ERR_NODEV;

	/* assume having crc appended, will determine whether feature
	 * is enabled or not
	 */
	tcm_msg->has_crc = true;
	tcm_dev->msg_data.crc_bytes = default_crc;
	tcm_msg->has_extra_rc = true;
	tcm_dev->msg_data.rc_byte = (unsigned char)default_rc;

	/* the identify report should be the first packet at startup */
	if (header->code == REPORT_IDENTIFY) {
		payload_length = syna_pal_le2_to_uint(header->length);
		tcm_msg->payload_length = payload_length;

		/* retrieve the startup packet */
		retval = syna_tcm_v1_continued_read(tcm_dev,
				payload_length);
		if (retval < 0) {
			LOGE("Fail to read in identify info packet\n");
			return -ERR_TCMMSG;
		}
	} else {
		/* if not, send an identify command instead */
		retval = syna_tcm_v1_write_message(tcm_dev,
				CMD_IDENTIFY,
				NULL,
				0,
				0,
				&resp_code,
				RESP_IN_POLLING);
		if (retval < 0) {
			/* if still not working, do reset */
			retval = syna_tcm_v1_write_message(tcm_dev,
					CMD_RESET,
					NULL,
					0,
					0,
					&resp_code,
					RESET_DELAY_MS);
			if (retval < 0) {
				LOGE("Fail to identify at startup\n");
				return -ERR_TCMMSG;
			}
		}

		payload_length = tcm_msg->payload_length;
	}


	/* parse the identify info packet if needed */
	if (tcm_dev->dev_mode == MODE_UNKNOWN) {
		syna_tcm_buf_lock(&tcm_msg->in);

		retval = syna_tcm_v1_parse_idinfo(tcm_dev,
				&tcm_msg->in.buf[MESSAGE_HEADER_SIZE],
				tcm_msg->in.buf_size - MESSAGE_HEADER_SIZE,
				payload_length);
		if (retval < 0) {
			LOGE("Fail to parse identify report at startup\n");
			syna_tcm_buf_unlock(&tcm_msg->in);
			return -ERR_TCMMSG;
		}

		syna_tcm_buf_unlock(&tcm_msg->in);
	}

	/* set up the max. reading length at startup */
	retval = syna_tcm_v1_set_max_rw_size(tcm_dev);
	if (retval < 0) {
		LOGE("Fail to setup the max length to read/write\n");
		return -ERR_TCMMSG;
	}

	/* detect whether has crc data appended */
	syna_tcm_v1_update_crc(tcm_dev);
	/* if all crc bytes belong to EOM, crc feature is yet enabled */
	if (tcm_dev->msg_data.crc_bytes == default_crc)
		tcm_msg->has_crc = false;
	/* if rc byte belongs to EOM, extra rc feature is yet enabled */
	if (tcm_dev->msg_data.rc_byte == default_rc)
		tcm_msg->has_extra_rc = false;

	LOGI("Message including CRC:(%s) extra RC:(%s)\n",
			(tcm_msg->has_crc) ? "yes" : "no",
			(tcm_msg->has_extra_rc) ? "yes" : "no");

	/* set up read/write operations */
	syna_tcm_v1_set_ops(tcm_dev);

	return 0;
}
