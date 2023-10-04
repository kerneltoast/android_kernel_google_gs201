/*
* This file is part of the UWB stack for linux.
*
* Copyright (c) 2022 Qorvo US, Inc.
*
* This software is provided under the GNU General Public License, version 2
* (GPLv2), as well as under a Qorvo commercial license.
*
* You may choose to use this software under the terms of the GPLv2 License,
* version 2 ("GPLv2"), as published by the Free Software Foundation.
* You should have received a copy of the GPLv2 along with this program.  If
* not, see <http://www.gnu.org/licenses/>.
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

#include "fira_session.h"
#include "fira_sts.h"
#include "fira_crypto.h"

#include <asm/unaligned.h>
#include <linux/errno.h>
#include <linux/string.h>

#define FIRA_CONCATENATED_PARAMS_SIZE 17

/**
* fira_sts_concatenate_params() - Concatenate the session parameters to compute
* the digest.
* @session: FiRa session for which we need to concatenate the parameter.
* @channel: Channel parameter coming from the LLHW.
* @slot_duration_us: duration of a FiRa slot in us (according to session config).
* @concat_params: output buffer.
* @concat_params_size: size of the output buffer.
*/
static void
fira_sts_concatenate_params(const struct fira_session *session,
			    const struct mcps802154_channel *channel,
			    int slot_duration_us, u8 *concat_params,
			    u8 concat_params_size)
{
	u8 *p;

	p = concat_params;
	*p++ = session->params.ranging_round_usage;
	*p++ = session->params.sts_config;
	*p++ = session->params.multi_node_mode;
	*p++ = session->params.channel_number != 0 ?
		       session->params.channel_number :
		       channel->channel;
	put_unaligned_be16(slot_duration_us, p);
	p += sizeof(u16);
	*p++ = session->params.mac_fcs_type;
	*p++ = session->params.rframe_config;
	*p++ = session->params.preamble_code_index != 0 ?
		       session->params.preamble_code_index :
		       channel->preamble_code;
	*p++ = session->params.sfd_id;
	*p++ = session->params.psdu_data_rate;
	*p++ = session->params.preamble_duration;
	*p++ = 0x03;
	put_unaligned_be32(session->id, p);
}

/**
* fira_sts_get_crypto_sts_index() - Compute the current crypto STS index.
* @session: The session for which the crypto sts index is needed
* @slot_index: index to the current slot.
*
* Return: crypto_sts_index depending on the sts mode.
*/
static u32 fira_sts_get_crypto_sts_index(struct fira_session *session,
					 u32 slot_index)
{
	if (session->params.sts_config == FIRA_STS_MODE_STATIC) {
		return slot_index;
	}
	return fira_sts_get_phy_sts_index(session, slot_index);
}

int fira_sts_init(struct fira_session *session, int slot_duration_us,
		  const struct mcps802154_channel *current_channel)
{
	int r = 0;

	u32 crypto_sts_index;
	u8 concat_params[FIRA_CONCATENATED_PARAMS_SIZE];
	struct fira_crypto_params crypto_params;

	fira_sts_concatenate_params(session, current_channel, slot_duration_us,
				    concat_params, sizeof(concat_params));

	crypto_params.session_id = session->id;
	crypto_params.sts_config = session->params.sts_config;
	crypto_params.concat_params = concat_params;
	crypto_params.concat_params_size = sizeof(concat_params);
	crypto_params.vupper64 = session->params.vupper64;
	crypto_params.prov_session_key = session->params.session_key;
	crypto_params.prov_session_key_len = session->params.session_key_len;

	r = fira_crypto_context_init(&crypto_params, &session->crypto);
	if (r)
		return r;

	r = fira_crypto_build_phy_sts_index_init(
		session->crypto, &session->sts.phy_sts_index_init);
	if (r)
		goto error_out;

	session->sts.last_rotation_block_index = 0;
	crypto_sts_index = fira_sts_get_crypto_sts_index(session, 0);
	r = fira_crypto_rotate_elements(session->crypto, crypto_sts_index);
	if (r)
		goto error_out;

	return 0;

error_out:
	fira_crypto_context_deinit(session->crypto);
	session->crypto = NULL;
	return r;
}

void fira_sts_deinit(struct fira_session *session)
{
	if (session->crypto)
		fira_crypto_context_deinit(session->crypto);
}

int fira_sts_rotate_keys(struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;
	u32 rotation_period;
	u32 n_slots_per_block;
	u32 crypto_sts_index;
	bool time_to_rotate;
	bool rotation_after_resync;
	int rotation_block_index;
	int r = 0;

	if (params->sts_config != FIRA_STS_MODE_STATIC &&
	    params->key_rotation) {
		/* Key rotation is triggered after rotation_period expires or
		 * by a resync at controlee side.
		 */
		rotation_period = (1 << params->key_rotation_rate);
		time_to_rotate = (session->block_index -
				  session->sts.last_rotation_block_index) >=
				 rotation_period;
		rotation_after_resync = session->block_index <
					session->sts.last_rotation_block_index;
		if (time_to_rotate || rotation_after_resync) {
			n_slots_per_block = (params->block_duration_dtu /
					     params->slot_duration_dtu);
			/* Remove extra blocks following resynchronization
			 * rotation_block_index should be power of 2 and multiple of
			 * rotation_period.
			 *
			 * crypto_sts_index shall be calculated at the block triggering rotation.
			 */
			rotation_block_index =
				session->block_index -
				(session->block_index % rotation_period);
			crypto_sts_index =
				session->sts.phy_sts_index_init +
				(rotation_block_index * n_slots_per_block);
			r = fira_crypto_rotate_elements(session->crypto,
							crypto_sts_index);
			session->sts.last_rotation_block_index =
				rotation_block_index;
		}
	}

	return r;
}

int fira_sts_get_sts_params(struct fira_session *session, u32 slot_index,
			    u8 *sts_v, u32 sts_v_size, u8 *sts_key,
			    u32 sts_key_size)
{
	u32 crypto_sts_index =
		fira_sts_get_crypto_sts_index(session, slot_index);
	return fira_crypto_get_sts_params(session->crypto, crypto_sts_index,
					  sts_v, sts_v_size, sts_key,
					  sts_key_size);
}

u32 fira_sts_get_phy_sts_index(const struct fira_session *session,
			       const u32 slot_index)
{
	return session->sts.phy_sts_index_init +
	       (session->block_index * (session->params.block_duration_dtu /
					session->params.slot_duration_dtu)) +
	       (session->round_index * session->params.round_duration_slots) +
	       slot_index;
}

int fira_sts_convert_phy_sts_idx_to_time_indexes(
	const struct fira_session *session, const u32 current_phy_sts_index,
	u32 *block_idx, u32 *round_idx, u32 *slot_idx)
{
	u32 remaining_slots, absolute_phy_sts_index, n_slots_per_block;
	const struct fira_session_params *params = &session->params;

	if (current_phy_sts_index < session->sts.phy_sts_index_init)
		return -EINVAL;
	n_slots_per_block =
		params->block_duration_dtu / params->slot_duration_dtu;
	absolute_phy_sts_index =
		current_phy_sts_index - session->sts.phy_sts_index_init;
	*block_idx = (u32)(absolute_phy_sts_index / n_slots_per_block);
	remaining_slots = absolute_phy_sts_index % n_slots_per_block;
	*round_idx = (u32)(remaining_slots / params->round_duration_slots);
	*slot_idx = remaining_slots % params->round_duration_slots;

	return 0;
}

int fira_sts_prepare_decrypt(struct fira_session *session, struct sk_buff *skb)
{
	return fira_crypto_prepare_decrypt(session->crypto, skb);
}

int fira_sts_encrypt_frame(struct fira_session *session, struct sk_buff *skb,
			   int header_len, __le16 src_short_addr,
			   u32 slot_index)
{
	u32 crypto_sts_index =
		fira_sts_get_crypto_sts_index(session, slot_index);
	return fira_crypto_encrypt_frame(session->crypto, skb, header_len,
					 src_short_addr, crypto_sts_index);
}

int fira_sts_decrypt_frame(struct fira_session *session, struct sk_buff *skb,
			   int header_len, __le16 src_short_addr,
			   u32 slot_index)
{
	u32 crypto_sts_index =
		fira_sts_get_crypto_sts_index(session, slot_index);
	return fira_crypto_decrypt_frame(session->crypto, skb, header_len,
					 src_short_addr, crypto_sts_index);
}

int fira_sts_decrypt_hie(struct fira_session *session, struct sk_buff *skb,
			 int hie_offset, int hie_len)
{
	return fira_crypto_decrypt_hie(session->crypto, skb, hie_offset,
				       hie_len);
}

int fira_sts_encrypt_hie(struct fira_session *session, struct sk_buff *skb,
			 int hie_offset, int hie_len)
{
	return fira_crypto_encrypt_hie(session->crypto, skb, hie_offset,
				       hie_len);
}
