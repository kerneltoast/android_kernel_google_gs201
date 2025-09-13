/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2022 Qorvo US, Inc.
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

#ifndef NET_MCPS802154_FIRA_CRYPTO_H
#define NET_MCPS802154_FIRA_CRYPTO_H

#include <linux/skbuff.h>
#include <linux/kernel.h>

#include <linux/errno.h>
#include <asm/unaligned.h>
#include <linux/string.h>
#include <linux/ieee802154.h>
#include <linux/printk.h>
#include <net/fira_region_params.h>

struct fira_crypto;

/**
 * fira_crypto_init() - Callback to initialize crypto module implementation.
 *
 * @key_manager: Handler to key manager.
 *
 * Return: 0 or error.
 */
int fira_crypto_init(void *key_manager);

/**
 * struct fira_crypto_params - Arguments grouping structure for the crypto context
 * initialization function.
 */
struct fira_crypto_params {
	/**
	 * @session_id: Id of the session using the current fira_crypto.
	 */
	u32 session_id;
	/**
	 * @sub_session_id: Id of the sub-session using the current fira_crypto.
	 * It is only used in case of Responder Specific Sub-session Key STS mode 0x04.
	 */
	u32 sub_session_id;
	/**
	 * @sts_config: The type of STS requested for this crypto.
	 */
	enum fira_sts_mode sts_config;
	/**
	 * @concat_params: The concatenated parameters of the session/sub-session according
	 * to the FiRa specs.
	 */
	const u8 *concat_params;
	/**
	 * @concat_params_size: The size of the concatenated parameters.
	 */
	int concat_params_size;
	/**
	 * @vupper64: The vupper 64 to use when static STS is used.
	 */
	const u8 *vupper64;
	/**
	 * @key: The key when provisioned STS is used.
	 */
	const u8 *key;
	/**
	 * @key_len: The length of the key when provisioned STS is used.
	 */
	u8 key_len;
};

/**
 * fira_crypto_get_capabilities() - Query FiRa STS capabilities
 *
 * Return: FiRa crypto backend capabilities as a bitfield
 * (see &enum fira_sts_mode).
 */
u32 fira_crypto_get_capabilities(void);

/**
 * fira_crypto_context_init() - Initialize a crypto context containing the crypto
 * elements for a session or sub-session.
 * @crypto_params: Parameters to initialize the crypto context.
 * @crypto: The initialized crypto context.
 *
 * Return: 0 or error.
 */
int fira_crypto_context_init(const struct fira_crypto_params *crypto_params,
			     struct fira_crypto **crypto);

/**
 * fira_crypto_context_deinit() - Deinitialize a crypto context.
 * @crypto: The crypto context to deinitialize.
 */
void fira_crypto_context_deinit(struct fira_crypto *crypto);

/**
 * fira_crypto_rotate_elements() - Rotate the crypto elements contained in the
 * crypto context.
 *
 * NOTE: After calling this function, all active crypto elements will be the latest
 * rotated ones.
 *
 * @crypto: The context containing the elements to rotate.
 * @crypto_sts_index: The crypto STS index to use to rotate the elements.
 *
 * Return: 0 or error.
 */
int fira_crypto_rotate_elements(struct fira_crypto *crypto,
				const u32 crypto_sts_index);

/**
 * fira_crypto_build_phy_sts_index_init() - Build the phy STS index init value
 * related to the given crypto context.
 *
 * @crypto: The context to use to compute the phy STS index init value.
 *
 * Return: 0 or error.
 */
int fira_crypto_build_phy_sts_index_init(struct fira_crypto *crypto);

/**
 * fira_crypto_get_phy_sts_index_init() - Get phy_sts_index_init related to the current crypto context.
 * @crypto: Context storing the crypto-related parameters.
 *
 * Return: phy_sts_index_init deduced at crypto context initialization.
 */
u32 fira_crypto_get_phy_sts_index_init(const struct fira_crypto *crypto);

/**
 * fira_crypto_get_sts_params() - Build and get the STS parameters according to
 * a specific crypto context.
 *
 * NOTE: The elements built are the STS value and the STS key. Their construction
 * depends on the STS config and is described in the FiRa MAC specification.
 *
 * @crypto: The context to use to build the STS parameters.
 * @crypto_sts_index: The crypto STS index to use to build the STS parameters.
 * @sts_v: The output buffer for STS V.
 * @sts_v_size: The size of the output buffer for STS V.
 * @sts_key: The output buffer for STS key.
 * @sts_key_size: The size of the output buffer for STS key.
 *
 * Return: 0 or error.
 */
int fira_crypto_get_sts_params(struct fira_crypto *crypto, u32 crypto_sts_index,
			       u8 *sts_v, u32 sts_v_size, u8 *sts_key,
			       u32 sts_key_size);

/**
 * fira_crypto_prepare_decrypt() - Prepare skb for header decryption and verification.
 * @crypto: The crypto context used to decrypt the frame.
 * @skb: Buffer containing the frame to decrypt.
 *
 * Return: 0 or error.
 */
int fira_crypto_prepare_decrypt(struct fira_crypto *crypto,
				struct sk_buff *skb);

/**
 * fira_crypto_encrypt_frame() - Encrypt a 802154 frame using a given context.
 *
 * NOTE: The src address is given as an argument as it is a part of the nonce needed
 * to encrypt the frame and it is not present in the 802154 frame.
 * The length of the header is given because only the payload is encrypted even if
 * the encryption algorithm needs the whole 802154 frame.
 * Encryption is done in-place.
 * When called this function shall increase the size of the skb of
 * FIRA_CRYPTO_AEAD_AUTHSIZE and set the correct bits in the 802154 frame SCF.
 *
 * @crypto: The context to use to encrypt the frame.
 * @skb: The buffer containing the whole frame, skb->data points to the start of
 * the 802154 frame header.
 * @header_len: The length of the 802154 frame header. Can be used to find the
 * position of the 802154 frame payload relative to skb->data.
 * @src_short_addr: The short source address attached to the frame.
 * @crypto_sts_index: The crypto STS index attached to the frame.
 *
 * Return: 0 or error.
 */
int fira_crypto_encrypt_frame(struct fira_crypto *crypto, struct sk_buff *skb,
			      int header_len, __le16 src_short_addr,
			      u32 crypto_sts_index);

/**
 * fira_crypto_decrypt_frame() - Decrypt a 802.15.4 frame using the given context.
 *
 * NOTE: The src address is given as an argument as it is a part of the nonce needed to perform the
 * decryption and it is not present in the 802.15.4 frame. The length of the header is given as
 * only the payload should be decrypted but the algorithm needs the whole 802.15.4 frame.
 * Decryption is performed in-place. When calling this function, it shall decrease the size of the
 * skb in FIRA_CRYPTO_AEAD_AUTHSIZE bytes and verify the correct bits in the 802.15.4 frame SCF.
 *
 * @crypto: The context used for frame decryption.
 * @skb: The buffer containing the whole frame, skb->data points to the start of the 802.15.4
 * frame payload.
 * @header_len: The length of the 802.15.4 frame header. Can be used to find the start of the
 * 802.15.4 frame payload relative to skb->data.
 * @src_short_addr: The short source address in the current frame.
 * @crypto_sts_index: The current crypto STS index.
 *
 * Return: 0 or error.
 */
int fira_crypto_decrypt_frame(struct fira_crypto *crypto, struct sk_buff *skb,
			      int header_len, __le16 src_short_addr,
			      u32 crypto_sts_index);

/**
 * fira_crypto_encrypt_hie() - Encrypt a 802154 header using a given context.
 *
 * @crypto: The crypto to use to encrypt the frame.
 * @skb: The buffer containing the whole frame, skb->data points to the start of
 * the 802154 frame header.
 * @hie_offset: Offset of the FiRa HIE relative to skb->data.
 * @hie_len: The length of the FiRa HIE.
 *
 * Return: 0 or error.
 */
int fira_crypto_encrypt_hie(struct fira_crypto *crypto, struct sk_buff *skb,
			    int hie_offset, int hie_len);

/**
 * fira_crypto_decrypt_hie() - Decrypt a 802154 header using a given context.
 *
 * @crypto: The crypto to use to encrypt the frame.
 * @skb: The buffer containing the whole frame, skb->data points to the start of
 * the 802154 frame payload.
 * @hie_offset: Offset of the FiRa HIE relative to skb->data.
 * @hie_len: The length of 802154 header.
 *
 * Return: 0 or error.
 */
int fira_crypto_decrypt_hie(struct fira_crypto *crypto, struct sk_buff *skb,
			    int hie_offset, int hie_len);

/**
 * fira_crypto_test() - Autotest for FiRa crypto.
 *
 * Return: 0 or error.
 */
int fira_crypto_test(void);

#endif /* NET_MCPS802154_FIRA_CRYPTO_H */
