/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */

#include "dw3000_nfcc_coex_msg.h"
#include "dw3000_nfcc_coex_buffer.h"
#include "dw3000_nfcc_coex_core.h"
#include "dw3000.h"
#include "dw3000_trc.h"
#include "dw3000_core.h"

/* TLVs len helpers. */
#define TLV_TYPELEN_LEN 2 /* type 1 byte, len 1 byte. */
#define TLV_U32_LEN (4 + 1) /* u32 + ack/nack. */
#define TLV_SLOTS_LEN(nbslots) \
	(1 + (8 * (nbslots)) + 1) /* nslots + slots + ack/nack. */
#define TLV_SLOTS_LIST_SIZE_MAX (1 + (8 * (TLV_MAX_NB_SLOTS)))
#define MSG_NEXT_TLV(buffer, offset) \
	(struct dw3000_nfcc_coex_tlv *)((buffer)->msg.tlvs + (offset))

/**
 * struct dw3000_nfcc_coex_tlv - Type Length Value.
 */
struct dw3000_nfcc_coex_tlv {
	/**
	 * @type: Identifier of TLV.
	 */
	u8 type;
	/**
	 * @len: Number of byte of TLV array.
	 */
	u8 len;
	/**
	 * @tlv: Value of the TLV.
	 */
	u8 tlv[];
} __attribute__((packed));

/**
 * dw3000_nfcc_coex_header_put() - Fill NFCC frame header.
 * @dw: Driver context.
 * @buffer: Message buffer to fill.
 */
void dw3000_nfcc_coex_header_put(struct dw3000 *dw,
				 struct dw3000_nfcc_coex_buffer *buffer)
{
	struct dw3000_nfcc_coex_msg *msg = &buffer->msg;

	trace_dw3000_nfcc_coex_header_put(dw, dw->nfcc_coex.version,
					  dw->nfcc_coex.tx_seq_num);
	memcpy(msg->signature, DW3000_NFCC_COEX_SIGNATURE_STR,
	       DW3000_NFCC_COEX_SIGNATURE_LEN);
	msg->ver_id = dw->nfcc_coex.version;
	msg->seqnum = dw->nfcc_coex.tx_seq_num;
	msg->nb_tlv = 0;
	buffer->tlvs_len = 0;
}

/**
 * dw3000_nfcc_coex_tlv_u32_put() - Fill buffer payload for a TLV.
 * @buffer: Message buffer to fill.
 * @type: Type id of the TLV.
 * @value: Value of TLV.
 */
static void dw3000_nfcc_coex_tlv_u32_put(struct dw3000_nfcc_coex_buffer *buffer,
					 enum dw3000_nfcc_coex_tlv_type type,
					 u32 value)
{
	struct dw3000_nfcc_coex_msg *msg = &buffer->msg;
	struct dw3000_nfcc_coex_tlv *tlv;
	u32 *v;

	tlv = MSG_NEXT_TLV(buffer, buffer->tlvs_len);
	msg->nb_tlv++;
	tlv->type = type;
	tlv->len = 4;
	v = (u32 *)&tlv->tlv;
	*v = value;
	buffer->tlvs_len += TLV_TYPELEN_LEN + TLV_U32_LEN;
}

/**
 * dw3000_nfcc_coex_clock_sync_payload_put() - Fill clock sync frame payload.
 * @dw: Driver context.
 * @buffer: Buffer to set with help of handle_access.
 */
static void
dw3000_nfcc_coex_clock_sync_payload_put(struct dw3000 *dw,
					struct dw3000_nfcc_coex_buffer *buffer)
{
	u32 session_time0_sys_time =
		dw3000_dtu_to_sys_time(dw, dw->nfcc_coex.access_start_dtu);

	/* Update clock reference. */
	dw->nfcc_coex.session_time0_dtu = dw->nfcc_coex.access_start_dtu;
	/* Prepare message. */
	trace_dw3000_nfcc_coex_clock_sync_payload_put(dw,
						      session_time0_sys_time);
	dw3000_nfcc_coex_header_put(dw, buffer);
	dw3000_nfcc_coex_tlv_u32_put(buffer,
				     DW3000_NFCC_COEX_TLV_TYPE_SESSION_TIME0,
				     session_time0_sys_time);
}

/**
 * dw3000_nfcc_coex_clock_offset_payload_put() - Fill clock offset payload.
 * @dw: Driver context.
 * @buffer: Buffer to set with help of handle_access.
 * @clock_offset_sys_time: Offset to add to next nfcc_coex schedule.
 */
static void dw3000_nfcc_coex_clock_offset_payload_put(
	struct dw3000 *dw, struct dw3000_nfcc_coex_buffer *buffer,
	u32 clock_offset_sys_time)
{
	trace_dw3000_nfcc_coex_clock_offset_payload_put(dw,
							clock_offset_sys_time);
	dw3000_nfcc_coex_header_put(dw, buffer);
	dw3000_nfcc_coex_tlv_u32_put(buffer,
				     DW3000_NFCC_COEX_TLV_TYPE_TLV_UWBCNT_OFFS,
				     clock_offset_sys_time);
}

/**
 * dw3000_nfcc_coex_stop_session_payload_put() - Fill stop session payload.
 * @dw: Driver context.
 * @buffer: Buffer to set with help of handle_access.
 * @session_id: Session id to stop.
 */
static void dw3000_nfcc_coex_stop_session_payload_put(
	struct dw3000 *dw, struct dw3000_nfcc_coex_buffer *buffer,
	u32 session_id)
{
	trace_dw3000_nfcc_coex_stop_session_payload_put(dw, session_id);

	dw3000_nfcc_coex_header_put(dw, buffer);

	dw3000_nfcc_coex_tlv_u32_put(
		buffer, DW3000_NFCC_COEX_TLV_TYPE_STOP_SESSION, session_id);
}

/**
 * dw3000_nfcc_coex_message_send() - Write message for NFCC and release SPI1.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_message_send(struct dw3000 *dw)
{
	struct dw3000_nfcc_coex_buffer buffer = {};
	/* Build the absolute sys time offset. */
	u32 offset_sys_time =
		(dw->dtu_sync << DW3000_DTU_PER_SYS_POWER) - dw->sys_time_sync;
	u32 clock_offset_sys_time;

	switch (dw->nfcc_coex.send) {
	case DW3000_NFCC_COEX_SEND_CLK_SYNC:
		dw3000_nfcc_coex_clock_sync_payload_put(dw, &buffer);
		break;
	default:
	case DW3000_NFCC_COEX_SEND_CLK_OFFSET:
		/* Compute the clock correction to forward to NFCC. */
		clock_offset_sys_time =
			offset_sys_time - dw->nfcc_coex.prev_offset_sys_time;
		/* Build the message with the clock update to forward. */
		dw3000_nfcc_coex_clock_offset_payload_put(
			dw, &buffer, -clock_offset_sys_time);
		break;
	case DW3000_NFCC_COEX_SEND_STOP:
		dw3000_nfcc_coex_stop_session_payload_put(
			dw, &buffer, DW3000_NFCC_COEX_SESSION_ID_DEFAULT);
		break;
	}

	dw->nfcc_coex.prev_offset_sys_time = offset_sys_time;
	/* Write message to NFCC and release SP1. */
	return dw3000_nfcc_coex_write_buffer(dw, &buffer, MSG_LEN(buffer));
}

/**
 * dw3000_nfcc_coex_header_check() - Check header message.
 * @dw: Driver context.
 * @buffer: Buffer to check.
 *
 * Return: 0 when buffer contain a valid message, else an error.
 */
static int
dw3000_nfcc_coex_header_check(struct dw3000 *dw,
			      const struct dw3000_nfcc_coex_buffer *buffer)
{
	const struct dw3000_nfcc_coex_msg *msg = &buffer->msg;

	trace_dw3000_nfcc_coex_header_check(dw, msg->signature, msg->ver_id,
					    msg->seqnum, msg->nb_tlv);
	/* Check signature. */
	if (memcmp(msg->signature, DW3000_NFCC_COEX_SIGNATURE_STR,
		   DW3000_NFCC_COEX_SIGNATURE_LEN)) {
		return -EINVAL;
	}
	/* Check AP_NFCC Interface Version ID. */
	if (msg->ver_id != dw->nfcc_coex.version) {
		return -EINVAL;
	}
	/* Read number of TLVs. */
	if (msg->nb_tlv > DW3000_NFCC_COEX_MAX_NB_TLV) {
		return -EINVAL;
	}
	/* Check if message is a new one with the sequence number. */
	if (!dw->nfcc_coex.first_rx_message &&
	    (msg->seqnum - dw->nfcc_coex.rx_seq_num) > 0) {
		/* TODO: Reject message with bad seqnum. */
	}

	dw->nfcc_coex.rx_seq_num = msg->seqnum;
	dw->nfcc_coex.first_rx_message = false;
	return 0;
}

/**
 * dw3000_nfcc_coex_tlvs_check() - Set information on a received message.
 * @dw: Driver context.
 * @buffer: Buffer to read.
 * @rx_msg_info: information updated on valid message.
 *
 * Return: 0 on success, else an error.
 */
static int
dw3000_nfcc_coex_tlvs_check(struct dw3000 *dw,
			    const struct dw3000_nfcc_coex_buffer *buffer,
			    struct dw3000_nfcc_coex_rx_msg_info *rx_msg_info)
{
	static const int tlvs_len_max =
		DW3000_NFCC_COEX_MSG_MAX_SIZE - MSG_HEADER_LEN;
	const struct dw3000_nfcc_coex_msg *msg = &buffer->msg;
	const struct dw3000_nfcc_coex_tlv_slot_list *slot_list = NULL;
	int tlvs_len = 0; /* Start parsing at first TLV. */
	int i;

	/* Process tlvs. */
	for (i = 0; i < msg->nb_tlv; i++) {
		struct dw3000_nfcc_coex_tlv *tlv;

		if ((tlvs_len + sizeof(*tlv)) > tlvs_len_max)
			return -EINVAL;

		tlv = MSG_NEXT_TLV(buffer, tlvs_len);
		tlvs_len += tlv->len;

		if (tlvs_len > tlvs_len_max)
			return -EINVAL;

		trace_dw3000_nfcc_coex_tlv_check(dw, tlv->type, tlv->len,
						 tlv->tlv);
		if (tlv->type == DW3000_NFCC_COEX_TLV_TYPE_SLOT_LIST_UUS) {
			/* Reject a new TLV with same type. Behavior not defined. */
			if (slot_list)
				return -EINVAL;
			/* Check if the tlv size isn't exceeding the list max size */
			if (tlv->len > TLV_SLOTS_LIST_SIZE_MAX)
				return -EINVAL;
			slot_list = (const struct dw3000_nfcc_coex_tlv_slot_list
					     *)&tlv->tlv;
			/* Update rx_msg_info. */
			if (slot_list->nb_slots > 0) {
				const struct dw3000_nfcc_coex_tlv_slot *slot =
					&slot_list->slots[0];
				u32 next_in_session_dtu =
					slot->t_start_uus
					<< DW3000_NFCC_COEX_DTU_PER_UUS_POWER;

				rx_msg_info->next_slot_found = true;
				rx_msg_info->next_timestamp_dtu =
					dw->nfcc_coex.session_time0_dtu +
					next_in_session_dtu;
				rx_msg_info->next_duration_dtu =
					(slot->t_end_uus - slot->t_start_uus)
					<< DW3000_NFCC_COEX_DTU_PER_UUS_POWER;
			}
		}
	}

	return 0;
}

/**
 * dw3000_nfcc_coex_message_check() - Check and read message.
 * @dw: Driver context.
 * @buffer: Buffer to read.
 * @rx_msg_info: Result of message parsed updated on success.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_message_check(
	struct dw3000 *dw, const struct dw3000_nfcc_coex_buffer *buffer,
	struct dw3000_nfcc_coex_rx_msg_info *rx_msg_info)
{
	int r;

	r = dw3000_nfcc_coex_header_check(dw, buffer);
	if (r)
		return r;

	r = dw3000_nfcc_coex_tlvs_check(dw, buffer, rx_msg_info);
	if (r)
		return r;

	if (rx_msg_info->next_slot_found)
		trace_dw3000_nfcc_coex_rx_msg_info(
			dw, rx_msg_info->next_timestamp_dtu,
			rx_msg_info->next_duration_dtu);
	return 0;
}
