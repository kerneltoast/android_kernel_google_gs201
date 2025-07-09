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
 * @file synaptics_touchcom_core_v2.c
 *
 * This file implements the TouchComm version 2 command-response protocol
 */

#include "synaptics_touchcom_core_dev.h"

#define BITS_IN_MESSAGE_HEADER (MESSAGE_HEADER_SIZE * 8)

#define HOST_PRIMARY (0)

#define COMMAND_RETRY_TIMES (5)

#define CHECK_PACKET_CRC

/*
 * @section: Header of TouchComm v2 Message Packet
 *
 * The 4-byte header in the TouchComm v2 packet
 */
struct tcm_v2_message_header {
	union {
		struct {
			unsigned char code;
			unsigned char length[2];
			unsigned char byte3;
		};
		unsigned char data[MESSAGE_HEADER_SIZE];
	};
};

/* helper to execute a tcm v2 command
 */
static int syna_tcm_v2_execute_cmd_request(struct tcm_dev *tcm_dev,
	unsigned char command, unsigned char *payload,
	unsigned int total_length, unsigned int length);

/*
 * syna_tcm_v2_set_max_rw_size()
 *
 * Configure the max length for message reading and writing.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    none.
 */
static int syna_tcm_v2_set_max_rw_size(struct tcm_dev *tcm_dev)
{
	int retval;
	unsigned int possible_rd_size = 0;
	unsigned int rd_size = 0;
	unsigned int wr_size = 0;
	struct tcm_identification_info *id_info;
	unsigned int build_id = 0;
	unsigned char data[2] = { 0 };

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

	rd_size = syna_pal_le2_to_uint(id_info->max_read_size);
	wr_size = syna_pal_le2_to_uint(id_info->max_write_size);
	possible_rd_size = syna_pal_le2_to_uint(id_info->max_possible_read_size);
	/* legacy firmware doesn't provide the possible size to read */
	if (tcm_dev->msg_data.legacy)
		possible_rd_size = rd_size;

	rd_size = MIN(rd_size, possible_rd_size);

	if ((wr_size == 0) || (rd_size == 0)) {
		LOGE("Invalid max read:%d or write:%d size\n",
			rd_size, wr_size);
		retval = 0;
		goto exit;
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

	/* set max read size */
	if (rd_size != tcm_dev->max_rd_size) {
		if (tcm_dev->max_rd_size == 0)
			tcm_dev->max_rd_size = rd_size;
		else
			tcm_dev->max_rd_size =
				MIN(rd_size, tcm_dev->max_rd_size);

		data[0] = (unsigned char)tcm_dev->max_rd_size;
		data[1] = (unsigned char)(tcm_dev->max_rd_size >> 8);

		retval = syna_tcm_v2_execute_cmd_request(tcm_dev,
				CMD_TCM2_SET_MAX_READ_LENGTH,
				data,
				sizeof(data),
				sizeof(data));
		if (retval < 0) {
			LOGE("Fail to set max read size\n");
			goto exit;
		}

		LOGD("Set max read length to %d bytes\n", tcm_dev->max_rd_size);
	}
	retval = 0;
exit:
	return retval;
}

/*
 * syna_tcm_v2_parse_idinfo()
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
static int syna_tcm_v2_parse_idinfo(struct tcm_dev *tcm_dev,
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
 * syna_tcm_v2_dispatch_report()
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
static void syna_tcm_v2_dispatch_report(struct tcm_dev *tcm_dev)
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
		tcm_dev->report_buf.data_length = tcm_msg->payload_length;
		ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);
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

		retval = syna_tcm_v2_parse_idinfo(tcm_dev,
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
			case CMD_TCM2_GET_REPORT:
				tcm_msg->response_code = STATUS_OK;
				ATOMIC_SET(tcm_msg->command_status,
					CMD_STATE_IDLE);
				syna_pal_completion_complete(cmd_completion);
				goto exit;
			case CMD_RESET:
			case CMD_SMART_BRIDGE_RESET:
				LOGD("Reset by command 0x%02X\n",
					tcm_msg->command);
				tcm_msg->response_code = STATUS_OK;
				ATOMIC_SET(tcm_msg->command_status,
					CMD_STATE_IDLE);
				syna_pal_completion_complete(cmd_completion);
				goto exit;
			case CMD_REBOOT_TO_DISPLAY_ROM_BOOTLOADER:
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
	} else {
		if (tcm_msg->command == CMD_TCM2_GET_REPORT) {
			LOGD("Report %2X received\n", tcm_msg->report_code);
			tcm_msg->response_code = STATUS_OK;
			ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);
			syna_pal_completion_complete(cmd_completion);
		}
	}

exit:
	return;
}

/*
 * syna_tcm_v2_dispatch_response()
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
static void syna_tcm_v2_dispatch_response(struct tcm_dev *tcm_dev)
{
	int retval;
	unsigned int resp_data_length;
	struct tcm_message_data_blob *tcm_msg = NULL;
	syna_pal_completion_t *cmd_completion = NULL;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	tcm_msg = &tcm_dev->msg_data;
	cmd_completion = &tcm_msg->cmd_completion;

	if (ATOMIC_GET(tcm_msg->command_status) != CMD_STATE_BUSY)
		return;

	resp_data_length = tcm_msg->payload_length;

	if (tcm_msg->response_code == STATUS_NO_REPORT_AVAILABLE) {
		if (tcm_msg->command == CMD_TCM2_GET_REPORT)
			tcm_msg->response_code = STATUS_OK;
	}

	if (resp_data_length == 0) {
		tcm_dev->resp_buf.data_length = resp_data_length;
		ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);
		goto exit;
	}

	/* store the received report into the temporary buffer */
	syna_tcm_buf_lock(&tcm_dev->resp_buf);

	retval = syna_tcm_buf_alloc(&tcm_dev->resp_buf,
			resp_data_length + 1);
	if (retval < 0) {
		LOGE("Fail to allocate memory for internal buf.resp\n");
		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
		ATOMIC_SET(tcm_msg->command_status, CMD_STATE_ERROR);
		goto exit;
	}

	syna_tcm_buf_lock(&tcm_msg->in);

	retval = syna_pal_mem_cpy(tcm_dev->resp_buf.buf,
			tcm_dev->resp_buf.buf_size,
			&tcm_msg->in.buf[MESSAGE_HEADER_SIZE],
			tcm_msg->in.buf_size - MESSAGE_HEADER_SIZE,
			resp_data_length);
	if (retval < 0) {
		LOGE("Fail to copy payload to internal resp_buf\n");
		syna_tcm_buf_unlock(&tcm_msg->in);
		syna_tcm_buf_unlock(&tcm_dev->resp_buf);
		ATOMIC_SET(tcm_msg->command_status, CMD_STATE_ERROR);
		goto exit;
	}

	tcm_dev->resp_buf.data_length = resp_data_length;

	syna_tcm_buf_unlock(&tcm_msg->in);
	syna_tcm_buf_unlock(&tcm_dev->resp_buf);

	ATOMIC_SET(tcm_msg->command_status, CMD_STATE_IDLE);

exit:
	syna_pal_completion_complete(cmd_completion);
}

/*
 * syna_tcm_v2_read()
 *
 * Read in a TouchCom packet from device.
 * Checking the CRC is necessary to ensure a valid message received.
 *
 * @param
 *    [ in] tcm_dev:    the device handle
 *    [ in] rd_length:  number of reading bytes;
 *                     '0' means to read the message header only
 *    [out] buf:        pointer to a buffer which is stored the retrieved data
 *    [out] buf_size:   size of the buffer pointed
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_v2_read(struct tcm_dev *tcm_dev, unsigned int rd_length,
		unsigned char **buf, unsigned int *buf_size)
{
	int retval;
	struct tcm_v2_message_header *header;
	int max_rd_size;
	int xfer_len;
	unsigned char seq;
	unsigned char crc6 = 0;
	unsigned short crc16 = 0xFFFF;
	struct tcm_message_data_blob *tcm_msg = NULL;
#ifdef CHECK_PACKET_CRC
	unsigned int header_len;
#endif

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;
	max_rd_size = tcm_dev->max_rd_size;

	/* continued packet crc if containing payload data */
	if (rd_length > 0)
		xfer_len = rd_length + TCM_MSG_CRC_LENGTH;
	else
		xfer_len = rd_length;
	xfer_len += sizeof(struct tcm_v2_message_header);

	if ((max_rd_size != 0) && (xfer_len > max_rd_size)) {
		LOGE("Invalid xfer length, len: %d, max_rd_size: %d\n",
			xfer_len, max_rd_size);
		tcm_msg->status_report_code = STATUS_INVALID;
		return -ERR_INVAL;
	}

	syna_tcm_buf_lock(&tcm_msg->temp);

	/* allocate the internal temp buffer */
	retval = syna_tcm_buf_alloc(&tcm_msg->temp, xfer_len);
	if (retval < 0) {
		LOGE("Fail to allocate memory for internal buf.temp\n");
		goto exit;
	}
	/* read data from the bus */
	retval = syna_tcm_read(tcm_dev,
			tcm_msg->temp.buf,
			xfer_len);
	if (retval < 0) {
		LOGE("Fail to read from device\n");
		goto exit;
	}

	header = (struct tcm_v2_message_header *)tcm_msg->temp.buf;

	/* check header crc always */
	crc6 = syna_tcm_crc6(header->data, BITS_IN_MESSAGE_HEADER);
	if (crc6 != 0) {
		LOGE("Invalid header crc: 0x%02x\n", (header->byte3 & 0x3f));
		tcm_msg->status_report_code = STATUS_PACKET_CORRUPTED;
		retval = 0;
		goto exit;
	}
	seq = (tcm_msg->seq_toggle & 0x01);
	if ((((header->byte3) >> 6) & 0x01) != seq) {
		LOGE("Mismatched sequence number, expected:%d\n", seq);
		tcm_msg->status_report_code = STATUS_PACKET_CORRUPTED;
		retval = 0;
		goto exit;
	}

#ifdef CHECK_PACKET_CRC
	/* check payload crc */
	header_len = syna_pal_le2_to_uint(header->length);
	if ((rd_length > 0) && (header_len > 0)) {
		tcm_msg->crc_bytes = (unsigned short)syna_pal_le2_to_uint(
			&tcm_msg->temp.buf[xfer_len - 2]);
		crc16 = syna_tcm_crc16(&tcm_msg->temp.buf[0],
				xfer_len, 0xFFFF);
		if (crc16 != 0) {
			LOGE("Invalid payload crc: %04X\n",
				tcm_msg->crc_bytes);

			tcm_msg->status_report_code =
				STATUS_PACKET_CORRUPTED;
			goto exit;
		}
	}
#endif

	tcm_msg->status_report_code = header->code;

	tcm_msg->payload_length = syna_pal_le2_to_uint(header->length);

	if (tcm_msg->status_report_code != STATUS_IDLE)
		LOGD("Status code: 0x%02x, length: %d (%02x %02x %02x %02x)\n",
			tcm_msg->status_report_code, tcm_msg->payload_length,
			header->data[0], header->data[1], header->data[2],
			header->data[3]);

	*buf = tcm_msg->temp.buf;
	*buf_size = tcm_msg->temp.buf_size;

exit:
	syna_tcm_buf_unlock(&tcm_msg->temp);

	return retval;
}

/*
 * syna_tcm_v2_write()
 *
 * Construct the TouchCom v2 packet and send it to device.
 * Add 4-byte header at the beginning of a message and appended crc if needed.
 *
 * @param
 *    [ in] tcm_dev:     the device handle
 *    [ in] command:     command code
 *    [ in] payload:     data payload if any
 *    [ in] payload_len: length of data payload to write
 *    [ in] header_len:  length in packet header
 *    [ in] resend:      flag for re-sending the packet
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_v2_write(struct tcm_dev *tcm_dev, unsigned char command,
		unsigned char *payload, unsigned int payload_len,
		unsigned int header_len, bool resend)
{
	int retval;
	struct tcm_v2_message_header *header;
	unsigned char bits = BITS_IN_MESSAGE_HEADER - 6;
	int total_xfer_size = sizeof(struct tcm_v2_message_header);
	unsigned short crc16;
	struct tcm_message_data_blob *tcm_msg = NULL;
	int offset;
	bool do_predict = false;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;

	if (!tcm_msg) {
		LOGE("Invalid tcm message blob\n");
		return -ERR_INVAL;
	}

	/* predict reading is applied only when getting the report */
	do_predict = (command == CMD_TCM2_GET_REPORT);

	if (payload_len > 0) {
		total_xfer_size += payload_len;
		total_xfer_size += TCM_MSG_CRC_LENGTH;
	}

	syna_tcm_buf_lock(&tcm_msg->out);

	/* allocate the internal out buffer */
	retval = syna_tcm_buf_alloc(&tcm_msg->out, total_xfer_size);
	if (retval < 0) {
		LOGE("Fail to allocate memory for internal buf.out\n");
		goto exit;
	}

	/* construct packet header */
	header = (struct tcm_v2_message_header *)tcm_msg->out.buf;

	if (!resend)
		tcm_msg->seq_toggle++;

	header->code = command;
	header->length[0] = (unsigned char)header_len;
	header->length[1] = (unsigned char)(header_len >> 8);
	header->byte3 = ((HOST_PRIMARY & 0x01) << 7);
	header->byte3 |= ((tcm_msg->seq_toggle & 0x01) << 6);
	header->byte3 |= syna_tcm_crc6(header->data, bits);

	LOGD("Command packet: %02x %02x %02x %02x, length:%d\n",
		header->data[0], header->data[1],
		header->data[2], header->data[3], payload_len);

	/* copy payload, if any */
	if (payload_len > 0) {
		retval = syna_pal_mem_cpy(
				&tcm_msg->out.buf[MESSAGE_HEADER_SIZE],
				tcm_msg->out.buf_size - MESSAGE_HEADER_SIZE,
				payload,
				payload_len,
				payload_len);
		if (retval < 0) {
			LOGE("Fail to copy payload data\n");
			goto exit;
		}

		/* append payload crc */
		offset = MESSAGE_HEADER_SIZE + payload_len;
		crc16 = syna_tcm_crc16(&tcm_msg->out.buf[0], offset, 0xFFFF);
		tcm_msg->out.buf[offset] = (unsigned char)((crc16 >> 8) & 0xFF);
		tcm_msg->out.buf[offset + 1] = (unsigned char)(crc16 & 0xFF);
	}

	/* write command packet to the bus */
	retval = syna_tcm_write(tcm_dev,
			tcm_msg->out.buf,
			total_xfer_size);
	if (retval < 0) {
		LOGE("Fail to write to device\n");
		goto exit;
	}

	/* update the length for predict reading */
	if ((tcm_msg->predict_reads) && do_predict) {
		tcm_msg->predict_length = MIN(tcm_msg->payload_length,
			tcm_dev->max_rd_size - MESSAGE_HEADER_SIZE - 2);
	} else {
		tcm_msg->predict_length = 0;
	}

exit:
	syna_tcm_buf_unlock(&tcm_msg->out);

	return retval;
}

/*
 * syna_tcm_v2_continued_read()
 *
 * Write a CMD_ACK to read in the remaining data payload continuously
 * until the end of data. All the retrieved data is appended to the
 * internal buffer.in.
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [ in] length:   remaining data length in bytes
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_v2_continued_read(struct tcm_dev *tcm_dev,
		unsigned int length)
{
	int retval;
	unsigned char *tmp_buf;
	unsigned int tmp_buf_size;
	int retry_cnt = 0;
	unsigned int idx;
	unsigned int offset;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int total_length;
	unsigned int remaining_length;
	unsigned char command;
	struct tcm_message_data_blob *tcm_msg = NULL;
	struct tcm_v2_message_header *header;
	bool need_ack_cmd;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;

	/* continued read packet contains the header and its payload */
	total_length = MESSAGE_HEADER_SIZE + tcm_msg->payload_length;

	remaining_length = length;

	offset = tcm_msg->payload_length - length;

	syna_tcm_buf_lock(&tcm_msg->in);

	/* extend the internal buf_in if needed */
	retval = syna_tcm_buf_realloc(&tcm_msg->in,
			total_length + 1);
	if (retval < 0) {
		LOGE("Fail to allocate memory for internal buf_in\n");
		goto exit;
	}

	/* available space for payload = total chunk size - header - crc */
	chunk_space = tcm_dev->max_rd_size;
	if (chunk_space == 0)
		chunk_space = remaining_length;
	else
		chunk_space -= (MESSAGE_HEADER_SIZE + TCM_MSG_CRC_LENGTH);

	chunks = syna_pal_ceil_div(remaining_length, chunk_space);
	chunks = chunks == 0 ? 1 : chunks;

	offset += MESSAGE_HEADER_SIZE;

	/* send CMD_ACK for a continued read */
	command = CMD_TCM2_ACK;

	for (idx = 0; idx < chunks; idx++) {
retry:
		/* ack command is required if
		 *  1. for the new implementations, not the first packet
		 *  2. for the legacy v2 firmware, ack is always needed
		 */
		if (tcm_msg->legacy)
			need_ack_cmd = true;
		else
			need_ack_cmd = ((idx > 0) | (offset > MESSAGE_HEADER_SIZE));

		if (need_ack_cmd || (retry_cnt > 0)) {
			retval = syna_tcm_v2_write(tcm_dev,
					command,
					NULL,
					0,
					0,
					(retry_cnt > 0));
			if (retval < 0) {
				LOGE("Fail to send ACK in continued read\n");
				goto exit;
			}
		}

		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		/* read in the requested size of data */
		retval = syna_tcm_v2_read(tcm_dev,
				xfer_length,
				&tmp_buf,
				&tmp_buf_size);
		if (retval < 0) {
			LOGE("Fail to read %d bytes from device\n",
					xfer_length);
			goto exit;
		}

		/* If see an error, retry the previous read transaction
		 * Send RETRY instead of CMD_ACK
		 */
		if (tcm_msg->status_report_code == STATUS_PACKET_CORRUPTED) {
			if (retry_cnt > COMMAND_RETRY_TIMES) {
				LOGE("Continued read packet corrupted\n");
				goto exit;
			}

			retry_cnt += 1;

			LOGW("Read corrupted, retry %d\n", retry_cnt);
			goto retry;
		}

		retry_cnt = 0;
		command = CMD_TCM2_ACK;

		/* append data from temporary buffer to in_buf */
		syna_tcm_buf_lock(&tcm_msg->temp);

		/* copy data from internal buffer.temp to buffer.in */
		retval = syna_pal_mem_cpy(&tcm_msg->in.buf[offset],
				tcm_msg->in.buf_size - offset,
				&tmp_buf[MESSAGE_HEADER_SIZE],
				tmp_buf_size - MESSAGE_HEADER_SIZE,
				xfer_length);
		if (retval < 0) {
			LOGE("Fail to copy payload to internal buf_in\n");
			syna_tcm_buf_unlock(&tcm_msg->temp);
			goto exit;
		}

		syna_tcm_buf_unlock(&tcm_msg->temp);

		remaining_length -= xfer_length;

		offset += xfer_length;
	}

	header = (struct tcm_v2_message_header *)tcm_msg->in.buf;
	tcm_msg->payload_length = syna_pal_le2_to_uint(header->length);

	tcm_msg->in.data_length = offset;
	retval = 0;

exit:
	syna_tcm_buf_unlock(&tcm_msg->in);

	return retval;
}

/*
 * syna_tcm_v2_get_response()
 *
 * Read in the response packet from device.
 * If containing payload data, use continued_read() function and read the
 * remaining payload data.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_v2_get_response(struct tcm_dev *tcm_dev)
{
	int retval;
	struct tcm_v2_message_header *header;
	unsigned char *tmp_buf;
	unsigned int tmp_buf_size;
	struct tcm_message_data_blob *tcm_msg = NULL;
	unsigned int len = 0;
	unsigned int remaining = 0;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;

	/* if predict read enabled, plus the predicted length
	 * and determine the length to read
	 */
	if ((tcm_msg->predict_reads) && (tcm_msg->predict_length > 0))
		len += tcm_msg->predict_length;

	/* read in the message header at first */
	retval = syna_tcm_v2_read(tcm_dev,
			len,
			&tmp_buf,
			&tmp_buf_size);
	if (retval < 0) {
		LOGE("Fail to read message header from device\n");
		return retval;
	}

	/* error out once the response packet is corrupted */
	if (tcm_msg->status_report_code == STATUS_PACKET_CORRUPTED)
		return 0;

	/* allocate the required space = header + payload */
	syna_tcm_buf_lock(&tcm_msg->in);

	retval = syna_tcm_buf_alloc(&tcm_msg->in,
			MESSAGE_HEADER_SIZE + tcm_msg->payload_length);
	if (retval < 0) {
		LOGE("Fail to reallocate memory for internal buf.in\n");
		syna_tcm_buf_unlock(&tcm_msg->in);
		return retval;
	}

	retval = syna_pal_mem_cpy(tcm_msg->in.buf,
			tcm_msg->in.buf_size,
			tmp_buf,
			tmp_buf_size,
			len + MESSAGE_HEADER_SIZE);
	if (retval < 0) {
		LOGE("Fail to copy data to internal buf_in\n");
		syna_tcm_buf_unlock(&tcm_msg->in);
		return retval;
	}

	syna_tcm_buf_unlock(&tcm_msg->in);

	/* read in payload, if any */
	remaining = tcm_msg->payload_length - len;
	if ((tcm_msg->payload_length) && (remaining > 0)) {
		retval = syna_tcm_v2_continued_read(tcm_dev, remaining);
		if (retval < 0) {
			LOGE("Fail to read in payload data, size: %d)\n",
				tcm_msg->payload_length);
			return retval;
		}
	}

	syna_tcm_buf_lock(&tcm_msg->in);

	header = (struct tcm_v2_message_header *)tcm_msg->in.buf;

	tcm_msg->payload_length = syna_pal_le2_to_uint(header->length);
	tcm_msg->status_report_code = header->code;

	syna_tcm_buf_unlock(&tcm_msg->in);

	return retval;
}

/*
 * syna_tcm_v2_send_cmd()
 *
 * Forward the given command and payload to syna_tcm_v2_write().
 *
 * @param
 *    [ in] tcm_dev:     the device handle
 *    [ in] command:     command code
 *    [ in] payload:     data payload if any
 *    [ in] length:      total length of payload if have any
 *    [ in] payload_len: length to transfer
 *    [ in] resend:      flag for re-sending the packet
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static inline int syna_tcm_v2_send_cmd(struct tcm_dev *tcm_dev,
		unsigned char command, unsigned char *payload,
		unsigned int length, unsigned int payload_len, bool resend)
{
	return syna_tcm_v2_write(tcm_dev,
			command,
			payload,
			payload_len,
			length,
			resend);
}

/*
 * syna_tcm_v2_execute_cmd_request()
 *
 * Process the command message.
 * The helper is responsible for sending the given command and its payload,
 * to device. Once the total size of message is over the wr_chunk, divide
 * into continued writes
 *
 * In addition, the response to the command generated by the device will be
 * read in immediately.
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] command:        command code
 *    [ in] payload:        data payload if any
 *    [ in] total_length:   total length of payload
 *    [ in] length:         length of data payload to write
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
static int syna_tcm_v2_execute_cmd_request(struct tcm_dev *tcm_dev,
		unsigned char command, unsigned char *payload,
		unsigned int total_length, unsigned int length)
{
	int retval;
	unsigned int idx;
	unsigned int offset;
	unsigned int chunks;
	unsigned int xfer_size;
	unsigned int remaining_size;
	int retry_cnt = 0;
	unsigned int chunk_space;
	struct tcm_message_data_blob *tcm_msg = NULL;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;
	chunk_space = tcm_dev->max_wr_size;

	remaining_size = length;

	/* available space for payload = total size - header - crc */
	if (chunk_space == 0)
		chunk_space = remaining_size;
	else
		chunk_space = chunk_space - (MESSAGE_HEADER_SIZE + 2);

	chunks = syna_pal_ceil_div(remaining_size, chunk_space);
	chunks = chunks == 0 ? 1 : chunks;

	offset = 0;

	/* process the command message and handle the response
	 * to the command
	 */
	for (idx = 0; idx < chunks; idx++) {
		if (remaining_size > chunk_space)
			xfer_size = chunk_space;
		else
			xfer_size = remaining_size;

retry:
		/* send command to device */
		command = (idx == 0) ? command : CMD_CONTINUE_WRITE;
		retval = syna_tcm_v2_send_cmd(tcm_dev,
				command,
				&payload[offset],
				total_length,
				xfer_size,
				(retry_cnt > 0));
		if (retval < 0) {
			LOGE("Fail to send command 0x%02x\n", command);
			goto exit;
		}

		/* bus turnaround delay */
		syna_pal_sleep_us(TAT_DELAY_US_MIN, TAT_DELAY_US_MAX);

		/* get the response to the command immediately */
		retval = syna_tcm_v2_get_response(tcm_dev);
		if (retval < 0) {
			LOGE("Fail to get the response to command 0x%02x\n",
				command);
			goto exit;
		}

		/* check the response code */
		tcm_msg->response_code = tcm_msg->status_report_code;
		switch (tcm_msg->response_code) {
		case STATUS_NO_REPORT_AVAILABLE:
		case STATUS_OK:
		case STATUS_ACK:
			retry_cnt = 0;
			/* typically, the total length and payload length should
			 * be the same. if not, it probably belong to continuous
			 * write operation and be controlled out of core lib
			 */
			if ((idx + 1 == chunks) && (total_length != length))
				tcm_msg->response_code = STATUS_OK;
			break;
		case STATUS_PACKET_CORRUPTED:
		case STATUS_RETRY_REQUESTED:
			retry_cnt += 1;
			break;
		default:
			/* go to next if returned status belongs to a report */
			if (tcm_msg->response_code >= REPORT_IDENTIFY)
				goto next;
			/* otherwise, unknown error */
			LOGE("Incorrect status code 0x%02x of command 0x%02x\n",
				tcm_msg->response_code, command);
			goto exit;
		}

		if (retry_cnt > 0) {
			if (command == CMD_RESET) {
				LOGE("Command CMD_RESET corrupted, exit\n");
				/* assume ACK and wait for interrupt assertion
				 * once the response of reset is corrupted
				 */
				tcm_msg->response_code = STATUS_ACK;
				goto exit;
			} else if (retry_cnt > COMMAND_RETRY_TIMES) {
				LOGE("Command 0x%02x corrupted\n", command);
				goto exit;
			}

			LOGN("Command 0x%02x, retry %d\n", command, retry_cnt);
			syna_pal_sleep_us(WR_DELAY_US_MIN, WR_DELAY_US_MAX);

			goto retry;
		}
next:
		offset += xfer_size;

		remaining_size -= xfer_size;

		if (chunks > 1)
			syna_pal_sleep_us(WR_DELAY_US_MIN, WR_DELAY_US_MAX);
	}

exit:
	return retval;
}

/*
 * syna_tcm_v2_read_message()
 *
 * Send a CMD_GET_REPORT to acquire a TouchCom v2 report packet from device.
 * Meanwhile, the retrieved data will be stored in the internal buffer.resp
 * or buffer.report.
 *
 * @param
 *    [ in] tcm_dev:            the device handle
 *    [out] status_report_code: status code or report code received
 *
 * @return
 *    0 or positive value on success; otherwise, on error.
 */
static int syna_tcm_v2_read_message(struct tcm_dev *tcm_dev,
		unsigned char *status_report_code)
{
	int retval;
	struct tcm_message_data_blob *tcm_msg = NULL;
	syna_pal_mutex_t *rw_mutex = NULL;
	syna_pal_completion_t *cmd_completion = NULL;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	tcm_msg = &tcm_dev->msg_data;
	rw_mutex = &tcm_msg->rw_mutex;
	cmd_completion = &tcm_msg->cmd_completion;

	if (status_report_code)
		*status_report_code = STATUS_INVALID;

	tcm_msg->crc_bytes = 0;

	syna_pal_mutex_lock(rw_mutex);

	/* request a command */
	retval = syna_tcm_v2_execute_cmd_request(tcm_dev,
			CMD_TCM2_GET_REPORT,
			NULL,
			0, 0);
	if (retval < 0) {
		LOGE("Fail to send command CMD_TCM2_GET_REPORT\n");

		if (ATOMIC_GET(tcm_msg->command_status) == CMD_STATE_BUSY) {
			ATOMIC_SET(tcm_msg->command_status, CMD_STATE_ERROR);
			syna_pal_completion_complete(cmd_completion);
		}
		goto exit;
	}

	/* duplicate the data to external buffer */
	syna_tcm_buf_lock(&tcm_dev->external_buf);
	if (tcm_msg->payload_length > 0) {
		retval = syna_tcm_buf_alloc(&tcm_dev->external_buf,
				tcm_msg->payload_length);
		if (retval < 0) {
			LOGE("Fail to allocate memory, external_buf invalid\n");
			syna_tcm_buf_unlock(&tcm_dev->external_buf);
			goto exit;
		} else {
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
	}
	tcm_dev->external_buf.data_length = tcm_msg->payload_length;
	syna_tcm_buf_unlock(&tcm_dev->external_buf);

	if (tcm_msg->response_code == STATUS_NO_REPORT_AVAILABLE)
		goto exit;

	/* process the retrieved packet */
	if (tcm_msg->status_report_code >= REPORT_IDENTIFY)
		syna_tcm_v2_dispatch_report(tcm_dev);
	else
		syna_tcm_v2_dispatch_response(tcm_dev);

	/* copy the status report code to caller */
	if (status_report_code)
		*status_report_code = tcm_msg->status_report_code;

exit:
	syna_pal_mutex_unlock(rw_mutex);

	return retval;
}

/*
 * syna_tcm_v2_write_message()
 *
 * Write message including command and its payload to TouchCom device.
 * Then, the response of the command generated by the device will be
 * read in and stored in internal buffer.resp.
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] command:        TouchComm command
 *    [ in] payload:        data payload, if any
 *    [ in] length_total:   length of total payload
 *    [ in] length:         length of payload data, if any
 *    [out] resp_code:      response code returned
 *    [ in] delay_ms_resp:  delay time for response reading.
 *                          a positive value presents the time for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 *
 * @return
 *    0 or positive value on success; otherwise, on error.
 */
static int syna_tcm_v2_write_message(struct tcm_dev *tcm_dev,
		unsigned char command, unsigned char *payload,
		unsigned int length_total, unsigned int length,
		unsigned char *resp_code, unsigned int delay_ms_resp)
{
	int retval;
	int timeout = 0;
	int polling_ms = 0;
	struct tcm_message_data_blob *tcm_msg = NULL;
	syna_pal_mutex_t *cmd_mutex = NULL;
	syna_pal_mutex_t *rw_mutex = NULL;
	syna_pal_completion_t *cmd_completion = NULL;
	bool has_irq_ctrl = false;
	bool in_polling = false;

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
	if (in_polling)
		delay_ms_resp = (delay_ms_resp < 100) ? 100 : delay_ms_resp;

	/* irq control is enabled only when the operations is implemented
	 * and the current status of irq is enabled.
	 * do not enable irq if it is disabled by someone.
	 */
	has_irq_ctrl = (bool)(tcm_dev->hw_if->ops_enable_irq != NULL);
	has_irq_ctrl &= tcm_dev->hw_if->bdata_attn.irq_enabled;

	/* disable irq when using polling mode */
	if (has_irq_ctrl && in_polling && tcm_dev->hw_if->ops_enable_irq)
		tcm_dev->hw_if->ops_enable_irq(tcm_dev->hw_if, false);

	if (length_total != length) {
		LOGD("Command: 0x%02x, payload length: %d (given: %d)\n",
			command, length_total, length);
	} else {
		LOGD("Command: 0x%02x, payload length: %d\n", command, length);
	}

	syna_pal_mutex_lock(cmd_mutex);

	syna_pal_mutex_lock(rw_mutex);

	ATOMIC_SET(tcm_msg->command_status, CMD_STATE_BUSY);

	/* reset the command completion */
	syna_pal_completion_reset(cmd_completion);

	tcm_msg->command = command;

	/* request a command execution */
	retval = syna_tcm_v2_execute_cmd_request(tcm_dev,
			command,
			payload,
			length_total,
			length);
	if (retval < 0) {
		LOGE("Fail to send command 0x%02x to device\n", command);
		syna_pal_mutex_unlock(rw_mutex);
		goto exit;
	}

	syna_pal_mutex_unlock(rw_mutex);

	/* waiting for the resp data only at STATUS_ACK */
	if (tcm_msg->response_code >= REPORT_IDENTIFY) {
		syna_tcm_v2_dispatch_report(tcm_dev);
		goto check_response;
	}

	if (tcm_msg->response_code != STATUS_ACK) {
		syna_tcm_v2_dispatch_response(tcm_dev);
		goto check_response;
	}

	/* handle the report generated by the command
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
		/* wait for the completion event triggered by read_message */
		retval = syna_pal_completion_wait_for(cmd_completion,
				polling_ms);

		/* break when getting a valid resp; otherwise, do polling */
		if (ATOMIC_GET(tcm_msg->command_status) == CMD_STATE_IDLE)
			goto check_response;
		if (in_polling) {
			ATOMIC_SET(tcm_msg->command_status, CMD_STATE_BUSY);

			/* retrieve the message packet back */
			retval = syna_tcm_v2_read_message(tcm_dev, NULL);
			/* keep in polling if still not having a valid resp */
			if (retval < 0)
				syna_pal_completion_reset(cmd_completion);
		}

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

	if (tcm_msg->response_code != STATUS_OK) {
		LOGE("Error code 0x%02x of command 0x%02x\n",
			tcm_msg->response_code, tcm_msg->command);
		retval = -ERR_TCMMSG;
	} else {
		retval = 0;
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
 * syna_tcm_v2_set_ops()
 *
 * Assign read / write operations
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    none
 */
void syna_tcm_v2_set_ops(struct tcm_dev *tcm_dev)
{
	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return;
	}

	/* expose the read / write operations */
	tcm_dev->read_message = syna_tcm_v2_read_message;
	tcm_dev->write_message = syna_tcm_v2_write_message;
	tcm_dev->set_max_rw_size = syna_tcm_v2_set_max_rw_size;

	tcm_dev->msg_data.predict_length = 0;
	tcm_dev->protocol = TOUCHCOMM_V2;

	if (tcm_dev->msg_data.legacy)
		LOGI("Apply legacy TouchComm V2 firmware\n");
}
 /*
  * syna_tcm_v2_detect()
  *
  * Function to process the startup packet of TouchComm V2 firmware
  *
  * For TouchCom v2 protocol, each packet must have a valid crc-6 at the end.
  * If so, send an identify command to identify the device and complete the
  * pre-initialization.
  *
  * @param
  *    [ in] tcm_dev: the device handle
  *    [ in] data:    raw 4-byte data
  *    [ in] size:    length of input data in bytes
  *
  * @return
  *    on success, 0 or positive value; otherwise, negative value on error.
  */
int syna_tcm_v2_detect(struct tcm_dev *tcm_dev, unsigned char *data,
		unsigned int size)
{
	int retval;
	struct tcm_message_data_blob *tcm_msg = NULL;
	unsigned char resp_code = 0;
	unsigned short default_rc = 0x5a;

	if (!tcm_dev) {
		LOGE("Invalid tcm device handle\n");
		return -ERR_INVAL;
	}

	if ((!data) || (size < MESSAGE_HEADER_SIZE)) {
		LOGE("Invalid parameters\n");
		return -ERR_INVAL;
	}

	if (syna_tcm_crc6(data, BITS_IN_MESSAGE_HEADER) != 0)
		return -ERR_NODEV;

	tcm_msg = &tcm_dev->msg_data;

	/* send 'GET_REPORT' command to read in the startup packet */
	retval = syna_tcm_v2_read_message(tcm_dev, &resp_code);
	if ((retval < 0) || (resp_code != REPORT_IDENTIFY)) {
		/* if not, send an identify command instead */
		retval = syna_tcm_v2_write_message(tcm_dev,
				CMD_IDENTIFY,
				NULL,
				0,
				0,
				&resp_code,
				RESP_IN_POLLING);
		if ((retval < 0) || (resp_code != STATUS_OK)) {
			/* if still not working, do reset */
			retval = syna_tcm_v2_write_message(tcm_dev,
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
	}

	/* parse the identify info packet if needed */
	if (tcm_dev->dev_mode == MODE_UNKNOWN) {
		syna_tcm_buf_lock(&tcm_msg->in);

		retval = syna_tcm_v2_parse_idinfo(tcm_dev,
				&tcm_msg->in.buf[MESSAGE_HEADER_SIZE],
				tcm_msg->in.buf_size,
				tcm_msg->payload_length);
		if (retval < 0) {
			LOGE("Fail to parse identify report at startup\n");
			syna_tcm_buf_unlock(&tcm_msg->in);
			return -ERR_TCMMSG;
		}

		syna_tcm_buf_unlock(&tcm_msg->in);
	}

	/* set up the max. reading length at startup */
	retval = syna_tcm_v2_set_max_rw_size(tcm_dev);
	if (retval < 0) {
		LOGE("Fail to setup the max length to read/write\n");
		return -ERR_TCMMSG;
	}

	/* tcm v2 always has crc appended */
	tcm_dev->msg_data.has_crc = true;
	/* tcm v2 not support extra rc appending so far */
	tcm_dev->msg_data.has_extra_rc = false;
	tcm_dev->msg_data.rc_byte = (unsigned char)default_rc;

	LOGI("Message including CRC:(%s) extra RC:(%s)\n",
			(tcm_msg->has_crc) ? "yes" : "no",
			(tcm_msg->has_extra_rc) ? "yes" : "no");

	/* set up read/write operations */
	syna_tcm_v2_set_ops(tcm_dev);

	return 0;
}
