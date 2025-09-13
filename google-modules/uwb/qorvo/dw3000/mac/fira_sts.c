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

#include "fira_region.h"
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
 * fira_sts_get_absolute_slot_index() - Compute the absolute index of the current slot.
 * @session: The session requesting absolute slot index.
 * @slot_index: index to the current slot in the round.
 *
 * Return: crypto_sts_index depending on the sts mode.
 */
static u32 fira_sts_get_absolute_slot_index(const struct fira_session *session,
					    const u32 slot_index)
{
	return (session->block_index * (session->params.block_duration_dtu /
					session->params.slot_duration_dtu)) +
	       (session->round_index * session->params.round_duration_slots) +
	       slot_index;
}

/**
 * fira_sts_get_crypto_sts_index() - Compute the current crypto STS index.
 * @session: The session requesting crypto sts index.
 * @crypto: The crypto context containing initial value of phy sts index.
 * @slot_index: index to the current slot in the round.
 *
 * Return: crypto_sts_index depending on the sts mode.
 */
static u32 fira_sts_get_crypto_sts_index(const struct fira_session *session,
					 const struct fira_crypto *crypto,
					 const u32 slot_index)
{
	u32 phy_sts_index_init, absolute_slot_index;

	/*
	 * In static sts, crypto_sts_index is the current slot index in the ranging round.
	 * For other sts configurations, crypto_sts_index shall be the current phy_sts_index.
	 */
	if (session->params.sts_config == FIRA_STS_MODE_STATIC)
		return slot_index;

	phy_sts_index_init = fira_crypto_get_phy_sts_index_init(crypto);
	absolute_slot_index =
		fira_sts_get_absolute_slot_index(session, slot_index);
	return phy_sts_index_init + absolute_slot_index;
}

/**
 * fira_sts_crypto_init() - Initialize crypto related variables for the current
 * session or sub-session.
 * @session: The FiRa session the current crypto context belongs to.
 * @crypto_params: Parameters to initialize the crypto context.
 * @crypto: Crypto related variables.
 *
 * Return: 0 or error.
*/
static int fira_sts_crypto_init(struct fira_session *session,
				struct fira_crypto_params *crypto_params,
				struct fira_crypto **crypto)
{
	int r;
	u32 crypto_sts_index = 0;
	u32 phy_sts_index_init, block_duration_in_slots;

	r = fira_crypto_context_init(crypto_params, crypto);
	if (r)
		return r;

	r = fira_crypto_build_phy_sts_index_init(*crypto);
	if (r)
		goto error_out;

	/*
	 * crypto_sts_index shall be calculated at the block where the last key rotation occurred,
	 * not at the exact slot where crypto_context is being initialized. For Static STS, as key
	 * rotation is not supported, crypto_sts_index shall be 0 (first slot, RR and block).
	 */
	if (session->params.sts_config != FIRA_STS_MODE_STATIC) {
		phy_sts_index_init =
			fira_crypto_get_phy_sts_index_init(*crypto);
		block_duration_in_slots = session->params.block_duration_dtu /
					  session->params.slot_duration_dtu;
		crypto_sts_index = phy_sts_index_init +
				   (session->sts.last_rotation_block_index *
				    block_duration_in_slots);
	}

	r = fira_crypto_rotate_elements(*crypto, crypto_sts_index);
	if (r)
		goto error_out;

	return 0;
error_out:
	fira_crypto_context_deinit(*crypto);
	return r;
}
/**
 * fira_sts_responder_specific_crypto_init() - Initialize sub-session's crypto context
 * in case of a controlee device with a sts_config being FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY.
 * @session: The FiRa session the current crypto context belongs to.
 * @crypto_params: Parameters to initialize the crypto context.
 * @concat_params: Subsession's parameters concatenation used to calculate configDigest.
 * @concat_params_size: Size of the parameter concatenation buffer.
 *
 * Return: 0 or error.
 */
static int fira_sts_responder_specific_crypto_init(
	struct fira_session *session, struct fira_crypto_params *crypto_params,
	u8 *concat_params, u8 concat_params_size)
{
	bool sts_mode_responder_specific_key, device_controlee_responder;

	sts_mode_responder_specific_key =
		session->params.sts_config ==
		FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY;
	device_controlee_responder = session->params.device_type ==
		FIRA_DEVICE_TYPE_CONTROLEE;

	/* Replace by params.device_role == FIRA_DEVICE_ROLE_RESPONDER when roles
	 * are implemented in FiRa region. Currently, FIRA_DEVICE_TYPE_CONTROLEE == FIRA_DEVICE_ROLE_RESPONDER
	 * and FIRA_DEVICE_TYPE_CONTROLLER == FIRA_DEVICE_ROLE_INITIATOR.
	 */
	if (!sts_mode_responder_specific_key || !device_controlee_responder)
		return 0;

	/* Reused most part of crypto_params except for sub-session specific parameters. */
	crypto_params->sub_session_id = session->params.sub_session_id;
	crypto_params->key = session->params.sub_session_key;
	crypto_params->key_len = session->params.sub_session_key_len;
	return fira_sts_crypto_init(
		session, crypto_params,
		&session->controlee.responder_specific_crypto);
}

int fira_sts_init(struct fira_session *session, int slot_duration_us,
		const struct mcps802154_channel *current_channel)
{
	int r = 0;
	u8 concat_params[FIRA_CONCATENATED_PARAMS_SIZE];
	struct fira_crypto_params crypto_params;

	if ((session->params.sts_config ==
                     FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY) &&
            session->params.multi_node_mode == FIRA_MULTI_NODE_MODE_UNICAST)
                return -EINVAL;

	fira_sts_concatenate_params(session, current_channel,
					slot_duration_us, concat_params,
					sizeof(concat_params));

	crypto_params.session_id = session->id;
	crypto_params.sts_config = session->params.sts_config;
	crypto_params.concat_params = concat_params;
	crypto_params.concat_params_size = sizeof(concat_params);
	crypto_params.vupper64 = session->params.vupper64;
	crypto_params.key = session->params.session_key;
	crypto_params.key_len = session->params.session_key_len;

	r = fira_sts_crypto_init(session, &crypto_params, &session->crypto);
	if (r)
		return r;

	r = fira_sts_responder_specific_crypto_init(
		session, &crypto_params, concat_params, sizeof(concat_params));
	if (r)
		goto error_out;

	session->sts.last_rotation_block_index = 0;
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
	if (session->controlee.responder_specific_crypto)
		fira_crypto_context_deinit(
			session->controlee.responder_specific_crypto);

	session->crypto = session->controlee.responder_specific_crypto = NULL;
}

int fira_sts_controlee_init(struct fira_session *session,
			    struct fira_controlee *controlee,
			    int slot_duration_us,
			    const struct mcps802154_channel *current_channel)
{
	int r;
	u8 concat_params[FIRA_CONCATENATED_PARAMS_SIZE];
	struct fira_crypto_params crypto_params = { 0 };

	if (session->params.sts_config !=
		    FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY) {
		controlee->crypto = NULL;
		return 0;
	}

	/*
	 * Session should not be unicast if STS is configured in
	 * FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY
	 */
	if (session->params.multi_node_mode == FIRA_MULTI_NODE_MODE_UNICAST)
		return -EINVAL;

	fira_sts_concatenate_params(session, current_channel,
					slot_duration_us, concat_params,
					sizeof(concat_params));

	crypto_params.session_id = session->id;
	crypto_params.sub_session_id = controlee->sub_session_id;
	crypto_params.sts_config = session->params.sts_config;
	crypto_params.concat_params = concat_params;
	crypto_params.concat_params_size = sizeof(concat_params);
	crypto_params.vupper64 = NULL;
	crypto_params.key = controlee->sub_session_key;
	crypto_params.key_len = controlee->sub_session_key_len;

	r = fira_sts_crypto_init(session, &crypto_params, &controlee->crypto);
	if (r)
		return r;

	session->sts.last_rotation_block_index = 0;
	return 0;
}

void fira_sts_controlee_deinit(struct fira_controlee *controlee)
{
	if (!controlee->crypto)
	  fira_crypto_context_deinit(controlee->crypto);
}

/**
 * fira_sts_rotate_elements() - Rotate intermediate keys.
 * @session: The FiRa session requesting key rotation.
 * @crypto: crypto related variables.
 * @n_slots_per_block: Amount of slots contained in a FiRa block.
 * @rotation_block_index: Block index where key rotation shall be placed.
 */
static void fira_sts_rotate_elements(const struct fira_session *session,
				     struct fira_crypto *crypto,
				     const u32 n_slots_per_block,
				     const int rotation_block_index)
{
	u32 crypto_sts_index, phy_sts_index_init;

	phy_sts_index_init = fira_crypto_get_phy_sts_index_init(crypto);
	/* crypto_sts_index shall be calculated at the block triggering key rotation. */
	crypto_sts_index =
		phy_sts_index_init + (rotation_block_index * n_slots_per_block);
	fira_crypto_rotate_elements(crypto, crypto_sts_index);
}

void fira_sts_rotate_keys(struct fira_session *session)
{
	const struct fira_session_params *params = &session->params;
	struct fira_controlee *controlee, *tmp_controlee;
	u32 next_block = session->block_index + session->block_stride_len + 1;
	u32 rotation_period, n_slots_per_block;
	bool time_to_rotate, rotation_after_resync;
	int rotation_block_index;

	if (params->sts_config == FIRA_STS_MODE_STATIC || !params->key_rotation)
		return;

	/* Rotation is triggered after period expires or by resynchronization in the controlee. */
	rotation_period = (1 << params->key_rotation_rate);
	time_to_rotate =
		(next_block - session->sts.last_rotation_block_index) >=
		rotation_period;
	rotation_after_resync = next_block <
				session->sts.last_rotation_block_index;
	if (!time_to_rotate && !rotation_after_resync)
		return;

	/* Remove extra blocks following resynchronization. */
	rotation_block_index = next_block - (next_block % rotation_period);
	n_slots_per_block =
		(params->block_duration_dtu / params->slot_duration_dtu);
	fira_sts_rotate_elements(session, session->crypto, n_slots_per_block,
				 rotation_block_index);

	if (params->device_type == FIRA_DEVICE_TYPE_CONTROLEE &&
	    session->controlee.responder_specific_crypto)
		fira_sts_rotate_elements(
			session, session->controlee.responder_specific_crypto,
			n_slots_per_block, rotation_block_index);
	else if (params->device_type == FIRA_DEVICE_TYPE_CONTROLLER &&
		 session->n_current_controlees > 0)
		list_for_each_entry_safe (controlee, tmp_controlee,
					  &session->current_controlees, entry) {
			if (!controlee->crypto)
				continue;
			fira_sts_rotate_elements(session, controlee->crypto,
						 n_slots_per_block,
						 rotation_block_index);
			}
	session->sts.last_rotation_block_index = rotation_block_index;
}

/**
 * fira_sts_get_fira_crypto_context() - Get the corresponding fira crypto context to be used in the current
 * slot depending on sts configuration and device type.
 * @session: The FiRa session requesting crypto context retrieval.
 * @slot: The slot on which desired fira crypto context will be used.
 *
 * Return: FiRa crypto context to use.
 */
static struct fira_crypto *
fira_sts_get_fira_crypto_context(const struct fira_session *session,
				 const struct fira_slot *slot)
{
	bool sts_mode_responder_specific_key =
	session->params.sts_config ==
		FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY;

	if (!sts_mode_responder_specific_key || slot->controller_tx)
		return session->crypto;

	return session->params.device_type == FIRA_DEVICE_TYPE_CONTROLEE ?
		       session->controlee.responder_specific_crypto :
		       slot->controlee->crypto;
}

int fira_sts_get_sts_params(const struct fira_session *session,
			    const struct fira_slot *slot, u8 *sts_v,
			    u32 sts_v_size, u8 *sts_key, u32 sts_key_size)
{
	struct fira_crypto *crypto =
		fira_sts_get_fira_crypto_context(session, slot);

	u32 crypto_sts_index =
		fira_sts_get_crypto_sts_index(session, crypto, slot->index);

	return fira_crypto_get_sts_params(crypto, crypto_sts_index, sts_v,
					  sts_v_size, sts_key, sts_key_size);
}

u32 fira_sts_get_phy_sts_index(const struct fira_session *session,
			       const struct fira_slot *slot)
{
	struct fira_crypto *crypto =
		fira_sts_get_fira_crypto_context(session, slot);
	u32 phy_sts_index_init = fira_crypto_get_phy_sts_index_init(crypto);
	u32 absolute_slot_index =
		fira_sts_get_absolute_slot_index(session, slot->index);

	return phy_sts_index_init + absolute_slot_index;
}

int fira_sts_convert_phy_sts_idx_to_time_indexes(
	const struct fira_session *session, const u32 current_phy_sts_index,
	u32 *block_idx, u32 *round_idx, u32 *slot_idx)
{
	u32 remaining_slots, absolute_phy_sts_index, n_slots_per_block,
		phy_sts_index_init;
	const struct fira_session_params *params = &session->params;

	phy_sts_index_init =
		fira_crypto_get_phy_sts_index_init(session->crypto);

	if (current_phy_sts_index < phy_sts_index_init)
		return -EINVAL;
	n_slots_per_block =
		params->block_duration_dtu / params->slot_duration_dtu;
	absolute_phy_sts_index = current_phy_sts_index - phy_sts_index_init;
	*block_idx = (u32)(absolute_phy_sts_index / n_slots_per_block);
	remaining_slots = absolute_phy_sts_index % n_slots_per_block;
	*round_idx = (u32)(remaining_slots / params->round_duration_slots);
	*slot_idx = remaining_slots % params->round_duration_slots;

	return 0;
}

int fira_sts_prepare_decrypt(struct fira_session *session,
			     const struct fira_slot *slot, struct sk_buff *skb)
{
	struct fira_crypto *crypto =
		fira_sts_get_fira_crypto_context(session, slot);

	return fira_crypto_prepare_decrypt(crypto, skb);
}

int fira_sts_encrypt_frame(const struct fira_session *session,
			   const struct fira_slot *slot, struct sk_buff *skb,
			   int header_len, __le16 src_short_addr)
{
	struct fira_crypto *crypto =
		fira_sts_get_fira_crypto_context(session, slot);
	u32 crypto_sts_index =
		fira_sts_get_crypto_sts_index(session, crypto, slot->index);

	return fira_crypto_encrypt_frame(crypto, skb, header_len,
					 src_short_addr, crypto_sts_index);
}

int fira_sts_decrypt_frame(const struct fira_session *session,
			   const struct fira_slot *slot, struct sk_buff *skb,
			   int header_len, __le16 src_short_addr)
{
	struct fira_crypto *crypto =
		fira_sts_get_fira_crypto_context(session, slot);
	u32 crypto_sts_index =
		fira_sts_get_crypto_sts_index(session, crypto, slot->index);

	return fira_crypto_decrypt_frame(crypto, skb, header_len,
					 src_short_addr, crypto_sts_index);
}

int fira_sts_decrypt_hie(const struct fira_session *session,
			 const struct fira_slot *slot, struct sk_buff *skb,
			 int hie_offset, int hie_len)
{
	struct fira_crypto *crypto =
		fira_sts_get_fira_crypto_context(session, slot);

	return fira_crypto_decrypt_hie(crypto, skb, hie_offset, hie_len);

}

int fira_sts_encrypt_hie(const struct fira_session *session,
			const struct fira_slot *slot, struct sk_buff *skb,
			 int hie_offset, int hie_len)
{
	struct fira_crypto *crypto =
		fira_sts_get_fira_crypto_context(session, slot);

	return fira_crypto_encrypt_hie(crypto, skb, hie_offset, hie_len);
}
