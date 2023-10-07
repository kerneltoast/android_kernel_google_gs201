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

#ifndef NET_MCPS802154_FIRA_STS_H
#define NET_MCPS802154_FIRA_STS_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/ieee802154.h>

#include <net/fira_region_params.h>
#include <net/mcps802154_frame.h>

struct fira_session;
struct fira_local;

/**
 * fira_sts_init() - Initialization of STS context for a given
 * session.
 * @session: The session for which the context shall be initialized.
 * @slot_duration_us: Duration of a slot in us.
 * @current_channel: Current channel used by the LLHW.
 *
 * Return: 0 or error.
 */
int fira_sts_init(struct fira_session *session, int slot_duration_us,
		  const struct mcps802154_channel *current_channel);

/**
 * fira_sts_deinit() - Deinitialize STS context and release its resources.
 * @session: The session for which the STS shall be de-initialized.
 */
void fira_sts_deinit(struct fira_session *session);

/**
 *  * fira_sts_controlee_init() - Initialize sub-session's STS context for a given controlee.
 * @session: The session requesting sub-session context init.
 * @controlee: The controlee containing the sub-session context init.
 * @slot_duration_us: Duration of a slot in us.
 * @current_channel: Current channel used by the llhw.
 *
 * Return: 0 or error.
 */
int fira_sts_controlee_init(struct fira_session *session,
			    struct fira_controlee *controlee,
			    int slot_duration_us,
			    const struct mcps802154_channel *current_channel);

/**
 * fira_sts_controlee_deinit() - Deinitialize sub-session's STS context for a given controlee.
 * @controlee: The controlee containing the sub-session context to deinit.
 */
void fira_sts_controlee_deinit(struct fira_controlee *controlee);

/**
 * fira_sts_rotate_keys() - Rotate intermediate keys if needed.
 * @session: The session requesting key rotation.
 */
void fira_sts_rotate_keys(struct fira_session *session);

/**
 * fira_sts_get_sts_params() - To fetch sts_params in order to configure the
 * current frame.
 * @session: The session for which the sts params are requested.
 * @slot: The index of the slot for which the STS shall be computed.
 * @sts_v: STS Vector to be filled using the context.
 * @sts_v_size: Size of the requested STS vector.
 * @sts_key: STS Key to be set using the context.
 * @sts_key_size: Size of the requested STS key.
 *
 * Return: 0 or error.
 */
int fira_sts_get_sts_params(const struct fira_session *session,
			    const struct fira_slot *slot, u8 *sts_v,
			    u32 sts_v_size, u8 *sts_key, u32 sts_key_size);

/**
 * fira_sts_get_phy_sts_index() - Compute phy sts index on a specific slot for a given session.
 * @session: The session requesting phy sts index.
 * @slot: The slot associated with the desired phy sts index.
 *
 *  Return: phy sts index on the current slot.
 */
u32 fira_sts_get_phy_sts_index(const struct fira_session *session,
				const struct fira_slot *slot);

/**
 * fira_sts_convert_phy_sts_idx_to_time_indexes() - Convert a given phy
 * sts index to the corresponding time related indexes (block, round and slot
 * indexes).
 * @session: The session to for which the indexes are needed.
 * @current_phy_sts_index: phy_sts_index used for time synchronization.
 * @block_idx: The block index pointed by the given phy sts index.
 * @round_idx: The block index pointed by the given phy sts index.
 * @slot_idx: The block index pointed by the given phy sts index.
 *
 * Return: 0 or error.
 */
int fira_sts_convert_phy_sts_idx_to_time_indexes(
	const struct fira_session *session, const u32 current_phy_sts_index,
	u32 *block_idx, u32 *round_idx, u32 *slot_idx);

/**
* fira_sts_prepare_decrypt() - Prepare skb for header decryption and verification.
* @session: The session to for which the indexes are needed.
* @slot: The slot for which the frame should be decrypted.
* @skb: Buffer containing the frame to decrypt.
*
* Return: 0 or error.
*/
int fira_sts_prepare_decrypt(struct fira_session *session,
			     const struct fira_slot *slot, struct sk_buff *skb);

/**
* fira_sts_encrypt_frame() - Encrypt the given FiRa 802154 frame.
* @session: The session to use to encrypt the frame.
* @slot: The slot with the frame to encrypt.
* @skb: Buffer containing the frame to encrypt.
* @header_len: Length of the 802154 header. Can be used to find the start of the
* payload (relative to skb->data).
* @src_short_addr: Source short address.
* @slot_index: The slot index of the frame to encrypt.
*
* Return: 0 or error.
*/
int fira_sts_encrypt_frame(const struct fira_session *session,
			   const struct fira_slot *slot, struct sk_buff *skb,
			   int header_len, __le16 src_short_addr);
/**
* fira_sts_decrypt_frame() - Decrypt the given FiRa 802154 frame.
* @session: The session to use to decrypt the frame.
* @skb: Buffer containing the frame to decrypt.
* @header_len: Length of the 802154 header. Used to find the start of the
* frame payload (relative to skb->data).
* @src_short_addr: Source short address.
*
* Return: 0 or error.
*/
int fira_sts_decrypt_frame(const struct fira_session *session,
			   const struct fira_slot *slot, struct sk_buff *skb,
			   int header_len, __le16 src_short_addr);

/**
* fira_sts_encrypt_hie() - Encrypt the HIE stored in a FiRa 802154
* frame.
* @session: The session to attached to the HIE.
* @slot: The slot with the HIE to encrypt.
* @skb: Buffer containing the frame to encrypt.
* @hie_offset: Offset to the start of the HIE (relative to skb->data) to encrypt.
* @hie_len: Length of the HIE to encrypt.
*
* Return: 0 or error.
*/
int fira_sts_encrypt_hie(const struct fira_session *session,
			 const struct fira_slot *slot, struct sk_buff *skb,
			 int hie_offset, int hie_len);

/**
* fira_sts_decrypt_hie() - Decrypt the HIE stored in a FiRa 802154 frame.
* @session: The session attached to the HIE
* @slot: The slot with the HIE to decrypt.
* @skb: Buffer containing the frame to decrypt.
* @hie_offset: Offset to the start of the HIE (relative to skb->data) to decrypt.
* @hie_len: Length of the HIE to decrypt.
*
* Return: 0 or error.
*/

int fira_sts_decrypt_hie(const struct fira_session *session,
			 const struct fira_slot *slot, struct sk_buff *skb,
			 int hie_offset, int hie_len);
#endif /* NET_MCPS802154_FIRA_STS_H */
