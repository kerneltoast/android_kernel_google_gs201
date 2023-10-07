/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
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

#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/ieee802154.h>
#include <linux/module.h>
#include <net/mcps802154_frame.h>

/**
 * enum mcps802154_ie_state - State of information element writing.
 * @MCPS802154_IE_STATE_INIT: Initial state, no IE written.
 * @MCPS802154_IE_STATE_HEADER_IE: At least one header element written.
 * @MCPS802154_IE_STATE_PAYLOAD_IE: At least one payload element written.
 * @MCPS802154_IE_STATE_NESTED_MLME_IE: At least one payload element written,
 * the last one is a MLME IE containing at least one nested IE.
 */
enum mcps802154_ie_state {
	MCPS802154_IE_STATE_INIT,
	MCPS802154_IE_STATE_HEADER_IE,
	MCPS802154_IE_STATE_PAYLOAD_IE,
	MCPS802154_IE_STATE_NESTED_MLME_IE,
};

/**
 * struct mcps802154_ie_cb - Control buffer used in sk_buff to store information
 * while building IE.
 */
struct mcps802154_ie_cb {
	/**
	 * @ie_state: State of IEs writing, used to know whether a terminator is
	 * needed or not.
	 */
	enum mcps802154_ie_state ie_state;
	/**
	 * @mlme_ie_index: Index in buffer of the MLME IE header. Valid in the
	 * corresponding state only, used to increment IE payload length.
	 */
	u16 mlme_ie_index;
	/**
	 * @header_len: Length of frame header. Can be used for frame
	 * encryption, the header is not encrypted but only used for
	 * authentication.
	 */
	u16 header_len;
};

void mcps802154_ie_put_begin(struct sk_buff *skb)
{
	struct mcps802154_ie_cb *cb = (struct mcps802154_ie_cb *)&skb->cb;

	cb->ie_state = MCPS802154_IE_STATE_INIT;
	cb->header_len = skb->len;
}
EXPORT_SYMBOL(mcps802154_ie_put_begin);

int mcps802154_ie_put_end(struct sk_buff *skb, bool data_payload)
{
	struct mcps802154_ie_cb *cb = (struct mcps802154_ie_cb *)&skb->cb;

	if (data_payload) {
		bool err = false;

		if (cb->ie_state == MCPS802154_IE_STATE_HEADER_IE)
			err = !mcps802154_ie_put_header_ie(
				skb, IEEE802154_IE_HEADER_TERMINATION_2_ID, 0);
		else if (cb->ie_state >= MCPS802154_IE_STATE_PAYLOAD_IE)
			err = !mcps802154_ie_put_payload_ie(
				skb, IEEE802154_IE_PAYLOAD_TERMINATION_GID, 0);
		if (unlikely(err))
			return -ENOBUFS;
	}
	return cb->header_len;
}
EXPORT_SYMBOL(mcps802154_ie_put_end);

void *mcps802154_ie_put_header_ie(struct sk_buff *skb, int element_id,
				  unsigned int len)
{
	u16 ie_header;
	u8 *ie;
	struct mcps802154_ie_cb *cb = (struct mcps802154_ie_cb *)&skb->cb;

	if (unlikely(len > FIELD_MAX(IEEE802154_HEADER_IE_HEADER_LENGTH)))
		return NULL;
	if (unlikely(skb_availroom(skb) < IEEE802154_IE_HEADER_LEN + len))
		return NULL;

	if (cb->ie_state == MCPS802154_IE_STATE_INIT)
		skb->data[1] |= IEEE802154_FC_IE_PRESENT >> 8;
	cb->ie_state = MCPS802154_IE_STATE_HEADER_IE;

	ie_header =
		FIELD_PREP(IEEE802154_HEADER_IE_HEADER_LENGTH, len) |
		FIELD_PREP(IEEE802154_HEADER_IE_HEADER_ELEMENT_ID, element_id) |
		IEEE802154_HEADER_IE_HEADER_TYPE;

	ie = skb_put(skb, IEEE802154_IE_HEADER_LEN + len);
	put_unaligned_le16(ie_header, ie);

	cb->header_len = skb->len;

	return ie + IEEE802154_IE_HEADER_LEN;
}
EXPORT_SYMBOL(mcps802154_ie_put_header_ie);

void *mcps802154_ie_put_payload_ie(struct sk_buff *skb, int group_id,
				   unsigned int len)
{
	u16 ie_header;
	u8 *ie;
	struct mcps802154_ie_cb *cb = (struct mcps802154_ie_cb *)&skb->cb;
	bool need_terminator = false;

	if (cb->ie_state < MCPS802154_IE_STATE_PAYLOAD_IE)
		need_terminator = true;

	if (unlikely(len > FIELD_MAX(IEEE802154_PAYLOAD_IE_HEADER_LENGTH)))
		return NULL;
	if (unlikely(skb_availroom(skb) <
		     (need_terminator ? IEEE802154_IE_HEADER_LEN : 0) +
			     IEEE802154_IE_HEADER_LEN + len))
		return NULL;

	if (cb->ie_state == MCPS802154_IE_STATE_INIT)
		skb->data[1] |= IEEE802154_FC_IE_PRESENT >> 8;
	if (need_terminator)
		mcps802154_ie_put_header_ie(
			skb, IEEE802154_IE_HEADER_TERMINATION_1_ID, 0);
	cb->ie_state = MCPS802154_IE_STATE_PAYLOAD_IE;

	ie_header =
		FIELD_PREP(IEEE802154_PAYLOAD_IE_HEADER_LENGTH, len) |
		FIELD_PREP(IEEE802154_PAYLOAD_IE_HEADER_GROUP_ID, group_id) |
		IEEE802154_PAYLOAD_IE_HEADER_TYPE;

	ie = skb_put(skb, IEEE802154_IE_HEADER_LEN + len);
	put_unaligned_le16(ie_header, ie);

	return ie + IEEE802154_IE_HEADER_LEN;
}
EXPORT_SYMBOL(mcps802154_ie_put_payload_ie);

void *mcps802154_ie_put_nested_mlme_ie(struct sk_buff *skb, int sub_id,
				       unsigned int len)
{
	u16 ie_header;
	u8 *ie;
	struct mcps802154_ie_cb *cb = (struct mcps802154_ie_cb *)&skb->cb;

	if (sub_id < IEEE802154_IE_NESTED_SHORT_MIN_SID) {
		if (unlikely(
			    len >
			    FIELD_MAX(IEEE802154_LONG_NESTED_IE_HEADER_LENGTH)))
			return NULL;
		ie_header = FIELD_PREP(IEEE802154_LONG_NESTED_IE_HEADER_LENGTH,
				       len) |
			    FIELD_PREP(IEEE802154_LONG_NESTED_IE_HEADER_SUB_ID,
				       sub_id) |
			    IEEE802154_LONG_NESTED_IE_HEADER_TYPE;
	} else {
		if (unlikely(len >
			     FIELD_MAX(
				     IEEE802154_SHORT_NESTED_IE_HEADER_LENGTH)))
			return NULL;
		ie_header = FIELD_PREP(IEEE802154_SHORT_NESTED_IE_HEADER_LENGTH,
				       len) |
			    FIELD_PREP(IEEE802154_SHORT_NESTED_IE_HEADER_SUB_ID,
				       sub_id) |
			    IEEE802154_SHORT_NESTED_IE_HEADER_TYPE;
	}

	if (cb->ie_state != MCPS802154_IE_STATE_NESTED_MLME_IE) {
		ie = mcps802154_ie_put_payload_ie(
			skb, IEEE802154_IE_PAYLOAD_MLME_GID,
			IEEE802154_IE_HEADER_LEN + len);
		if (unlikely(!ie))
			return NULL;
		cb->ie_state = MCPS802154_IE_STATE_NESTED_MLME_IE;
		cb->mlme_ie_index = ie - IEEE802154_IE_HEADER_LEN - skb->data;
	} else {
		u8 *mlme_ie = skb->data + cb->mlme_ie_index;
		u16 mlme_ie_header = get_unaligned_le16(mlme_ie);
		int mlme_len = FIELD_GET(IEEE802154_PAYLOAD_IE_HEADER_LENGTH,
					 mlme_ie_header);

		mlme_len += IEEE802154_IE_HEADER_LEN + len;
		if (unlikely(mlme_len >
			     FIELD_MAX(IEEE802154_PAYLOAD_IE_HEADER_LENGTH)))
			return NULL;
		mlme_ie_header = (mlme_ie_header &
				  ~IEEE802154_PAYLOAD_IE_HEADER_LENGTH) |
				 FIELD_PREP(IEEE802154_PAYLOAD_IE_HEADER_LENGTH,
					    mlme_len);
		put_unaligned_le16(mlme_ie_header, mlme_ie);

		ie = skb_put(skb, IEEE802154_IE_HEADER_LEN + len);
	}

	put_unaligned_le16(ie_header, ie);

	return ie + IEEE802154_IE_HEADER_LEN;
}
EXPORT_SYMBOL(mcps802154_ie_put_nested_mlme_ie);

int mcps802154_ie_get(struct sk_buff *skb,
		      struct mcps802154_ie_get_context *context)
{
	u16 ie_header;
	bool header;
	bool last = false;

	if (skb->len < IEEE802154_IE_HEADER_LEN) {
		if (context->mlme_len)
			/* This could only happen if caller made a mistake. */
			return -EINVAL;
		if (skb->len > 0)
			/* Not enough for a header, but too much for nothing. */
			return -EBADMSG;
		context->in_payload = true;
		context->kind = MCPS802154_IE_GET_KIND_NONE;
		context->id = 0;
		context->len = 0;
		return 1;
	}

	ie_header = get_unaligned_le16(skb->data);
	skb_pull(skb, sizeof(ie_header));

	if (context->mlme_len) {
		context->kind = MCPS802154_IE_GET_KIND_MLME_NESTED;
		if ((ie_header & IEEE802154_IE_HEADER_TYPE) ==
		    IEEE802154_LONG_NESTED_IE_HEADER_TYPE) {
			context->id = FIELD_GET(
				IEEE802154_LONG_NESTED_IE_HEADER_SUB_ID,
				ie_header);
			context->len = FIELD_GET(
				IEEE802154_LONG_NESTED_IE_HEADER_LENGTH,
				ie_header);
		} else {
			context->id = FIELD_GET(
				IEEE802154_SHORT_NESTED_IE_HEADER_SUB_ID,
				ie_header);
			context->len = FIELD_GET(
				IEEE802154_SHORT_NESTED_IE_HEADER_LENGTH,
				ie_header);
		}
		if (context->mlme_len < IEEE802154_IE_HEADER_LEN + context->len)
			return -EBADMSG;
		context->mlme_len -= IEEE802154_IE_HEADER_LEN + context->len;
	} else {
		header = (ie_header & IEEE802154_IE_HEADER_TYPE) ==
			 IEEE802154_HEADER_IE_HEADER_TYPE;
		if (header != !context->in_payload)
			return -EBADMSG;
		if (header) {
			context->kind = MCPS802154_IE_GET_KIND_HEADER;
			context->id = FIELD_GET(
				IEEE802154_HEADER_IE_HEADER_ELEMENT_ID,
				ie_header);
			context->len = FIELD_GET(
				IEEE802154_HEADER_IE_HEADER_LENGTH, ie_header);
			if (context->id ==
			    IEEE802154_IE_HEADER_TERMINATION_1_ID)
				context->in_payload = true;
			if (context->id ==
			    IEEE802154_IE_HEADER_TERMINATION_2_ID) {
				context->in_payload = true;
				last = true;
			}
		} else {
			context->kind = MCPS802154_IE_GET_KIND_PAYLOAD;
			context->id =
				FIELD_GET(IEEE802154_PAYLOAD_IE_HEADER_GROUP_ID,
					  ie_header);
			context->len = FIELD_GET(
				IEEE802154_PAYLOAD_IE_HEADER_LENGTH, ie_header);
			if (context->id ==
			    IEEE802154_IE_PAYLOAD_TERMINATION_GID)
				last = true;
			else if (context->id == IEEE802154_IE_PAYLOAD_MLME_GID)
				context->mlme_len = context->len;
		}
		if (skb->len < context->len)
			return -EBADMSG;
	}

	return last ? 1 : 0;
}
EXPORT_SYMBOL(mcps802154_ie_get);
