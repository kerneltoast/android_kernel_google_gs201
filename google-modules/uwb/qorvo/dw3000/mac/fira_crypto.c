/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2022 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation. You should
 * have received a copy of the GPLv2 along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
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

#ifdef __KERNEL__
#define pr_fmt(fmt) "%s:%s: " fmt, KBUILD_MODNAME, __func__
#endif

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <crypto/aes.h>

#include <asm/unaligned.h>

#include "fira_crypto.h"
#include <net/mcps802154_frame.h>
#include "mcps_crypto.h"

#ifdef CONFIG_FIRA_CRYPTO_HAVE_SE
#include "key_manager.h"
#endif

#ifdef __KERNEL__
static inline void *platform_malloc(size_t s) { return kmalloc(s, GFP_KERNEL); }
static inline void platform_free(void *p) { kfree(p); }
#else
#include "trace/define_trace_specific.h"
#define pr_info(...) print_trace(__VA_ARGS__)
#define pr_err(...) print_trace(__VA_ARGS__)
#include "platform_alloc.h"
#endif

#define FIRA_CRYPTO_KDF_LABEL_LEN	8
#define FIRA_CRYPTO_KDF_CONTEXT_LEN	16
#define FIRA_CRYPTO_KEY_STS_MASK	0x7FFFFFFF

#define FIRA_IE_VENDOR_OUI_LEN		3

#define FIRA_CRYPTO_AEAD_AUTHSIZE	8

/**
 * struct fira_crypto_aead - Context for payload encryption/decryption.
 */
struct fira_crypto_aead {
	/**
	 * @ctx: The context to be used during aead encryption & decryption.
	 */
	struct mcps_aes_ccm_star_128_ctx *ctx;
};

/**
 * struct fira_crypto - Context containing all crypto related
 * information.
 *
 * NOTE: It is regularly used by the FiRa region core to produce the STS
 * parameters for a given a session and to encrypt/decrypt frames.
 */
struct fira_crypto_base {
	/**
	 * @key_size: Size of the key used in the AES derivation.
	 */
	u8 key_size;

	/**
	 * @config_digest: Digest of the configuration, used as input for keys
	 * derivation.
	 */
	u8 config_digest[AES_BLOCK_SIZE];

	/**
	 * @data_protection_key: Derived from the session key, the label
	 * "DataPrtK" and the config_digest. The precise size is given by the
	 * key_size.
	 */
	u8 data_protection_key[FIRA_KEY_SIZE_MIN];

	/**
	 * @derived_authentication_iv: Derived from data_protection_key, the
	 * label "DerAuthI", the current value of crypto_sts_index, and the
	 * config_digest. Used to compute the STS parameters for a slot.
	 */
	u8 derived_authentication_iv[AES_BLOCK_SIZE];

	/**
	 * @derived_authentication_key: Derived from data_protection_key, the
	 * label "DerAuthK", the current value of crypto_sts_index and the
	 * config_digest. Used to compute the STS parameters for a slot.
	 */
	u8 derived_authentication_key[FIRA_KEY_SIZE_MIN];

	/**
	 * @derived_payload_key: Derived from data_protection_key, the label
	 * "DerPaylK", the current value of crypto_sts_index and the
	 * config_digest. Used to encrypt/decrypt message PIE.
	 */
	u8 derived_payload_key[FIRA_KEY_SIZE_MIN];

	/**
	 * @aead: AEAD Context for payload encryption/decryption.
	 */
	struct fira_crypto_aead aead;
};

struct fira_crypto {
	/**
	 * @phy_sts_index_init: Initial phy_sts_index deduced at context init.
	 */
	u32 phy_sts_index_init;
	/**
	 * @sts_config: The type of STS requested for this crypto.
	 */
	enum fira_sts_mode sts_config;

	/**
	 * @base: Common parameters between all types of crypto contexts.
	 */
	struct fira_crypto_base base;

	/******* Dynamic STS Only **************/

	/**
	 * @ecb_ctx: AES ECB context
	 */
	struct mcps_aes_ecb_128_ctx *ecb_ctx;

	/**
	 * @privacy_key: Derived from the session key, the label
	 * "PrivacyK" and the config_digest. Used to encrypt/decrypt message HIE.
	 */
	u8 privacy_key[FIRA_KEY_SIZE_MIN];

	/******* Static STS Only **************/
	/**
	 * @vupper64: The vupper 64 to use when static STS is used.
	 */
	u8 vupper64[FIRA_VUPPER64_SIZE];
};

static int fira_crypto_kdf(const u8 *input_key, unsigned int input_key_len,
			const char *label, const u8 *context, u8 *output_key,
			unsigned int output_key_len)
{
	u8 derivation_data[sizeof(u32) + FIRA_CRYPTO_KDF_LABEL_LEN +
		FIRA_CRYPTO_KDF_CONTEXT_LEN + sizeof(u32)];
	u8 *p;
	int r;

	if (input_key_len != AES_KEYSIZE_128) {
		pr_err("input_key_len != AES_KEYSIZE_128");
		return -EINVAL;
	}

	p = derivation_data;
	put_unaligned_be32(1, p);
	p += sizeof(u32);
	memcpy(p, label, FIRA_CRYPTO_KDF_LABEL_LEN);
	p += FIRA_CRYPTO_KDF_LABEL_LEN;
	memcpy(p, context, FIRA_CRYPTO_KDF_CONTEXT_LEN);
	p += FIRA_CRYPTO_KDF_CONTEXT_LEN;
	put_unaligned_be32(output_key_len * 8, p);

	r = mcps_crypto_cmac_aes_128_digest(input_key, derivation_data,
			sizeof(derivation_data), output_key);

	return r;
}

static int fira_crypto_aead_set_key(struct fira_crypto_aead *aead, const u8 *key)
{
	aead->ctx = mcps_crypto_aead_aes_ccm_star_128_create();

	return aead->ctx ? mcps_crypto_aead_aes_ccm_star_128_set(aead->ctx, key) : -ENOMEM;
}

static void fira_aead_fill_nonce(u8 *nonce, __le16 src_short_addr,
				 u32 crypto_sts_index)
{
	u8 *p;

	p = nonce;
	memset(p, 0, IEEE802154_EXTENDED_ADDR_LEN - IEEE802154_SHORT_ADDR_LEN);
	p += IEEE802154_EXTENDED_ADDR_LEN - IEEE802154_SHORT_ADDR_LEN;
	put_unaligned_be16(src_short_addr, p);
	p += IEEE802154_SHORT_ADDR_LEN;
	put_unaligned_be32(crypto_sts_index, p);
	p += sizeof(u32);
	*p++ = IEEE802154_SCF_SECLEVEL_ENC_MIC64;
}

static int fira_crypto_aead_encrypt(struct fira_crypto_aead *aead,
		struct sk_buff *skb, unsigned int header_len,
		__le16 src_short_addr, u32 crypto_sts_index)
{
	u8 nonce[MCPS_CRYPTO_AES_CCM_STAR_NONCE_LEN];
	u8 *header = skb->data;
	u8 *payload = skb->data + header_len;
	const int payload_len = skb->len - header_len;
	u8 *mac = skb->data + skb->len;
	int r;

	fira_aead_fill_nonce(nonce, src_short_addr, crypto_sts_index);

	skb->data[IEEE802154_FC_LEN + IEEE802154_SHORT_ADDR_LEN] =
		IEEE802154_SCF_SECLEVEL_ENC_MIC64 |
		IEEE802154_SCF_NO_FRAME_COUNTER;

	r = mcps_crypto_aead_aes_ccm_star_128_encrypt(
		aead->ctx, nonce, header, header_len, payload, payload_len, mac,
		FIRA_CRYPTO_AEAD_AUTHSIZE);

	if (!r)
		skb_put(skb, FIRA_CRYPTO_AEAD_AUTHSIZE);

	return r;
}

static int fira_crypto_aead_decrypt(struct fira_crypto_aead *aead,
		struct sk_buff *skb, unsigned int header_len,
		__le16 src_short_addr, u32 crypto_sts_index)
{
	u8 nonce[MCPS_CRYPTO_AES_CCM_STAR_NONCE_LEN];
	u8 *header;
	u8 *payload;
	unsigned int payload_len;
	u8 *mac;
	int r;

	header = skb->data - header_len;
	payload = skb->data;
	payload_len = skb->len;
	mac = skb->data + payload_len;

	fira_aead_fill_nonce(nonce, src_short_addr, crypto_sts_index);

	r = mcps_crypto_aead_aes_ccm_star_128_decrypt(
		aead->ctx, nonce, header, header_len, payload, payload_len, mac,
		FIRA_CRYPTO_AEAD_AUTHSIZE);

	if (!r) {
		header[IEEE802154_FC_LEN + IEEE802154_SHORT_ADDR_LEN] |=
			IEEE802154_SCF_NO_FRAME_COUNTER;
	}

	memzero_explicit(mac, FIRA_CRYPTO_AEAD_AUTHSIZE);

	return r;
}

static void fira_crypto_aead_destroy(struct fira_crypto_aead *aead)
{
	mcps_crypto_aead_aes_ccm_star_128_destroy(aead->ctx);
	aead->ctx = NULL;
}

static void remove_session(struct fira_crypto *session)
{
	fira_crypto_aead_destroy(&session->base.aead);
	mcps_crypto_aes_ecb_128_destroy(session->ecb_ctx);
	/* Wipe all derived keys */
	memzero_explicit(session, sizeof(*session));
	platform_free(session);
}

/*! ----------------------------------------------------------------------------------------------
 * @brief This function is used to initialise the FIRA crypto backend
 *
 * input parameters:
 * @param key_manager - key manager to use. Not used for the moment.
 *
 * output parameters
 *
 * return 0 if no error.
 */
int fira_crypto_init(void *key_manager)
{
#ifdef CONFIG_FIRA_CRYPTO_HAVE_SE
	/* Opens the UBWS - SE secure channel */
	return key_manager_init(NULL);
#else
	return 0;
#endif
}

/************************************** FIRA STS API FCTS ***************************************/

int fira_crypto_context_init(const struct fira_crypto_params *crypto_params,
		struct fira_crypto **crypto)
{
	int status = 0;
	struct fira_crypto *fira_crypto_ctx;
	u8 session_key[AES_KEYSIZE_128];

	fira_crypto_ctx = platform_malloc(sizeof(*fira_crypto_ctx));

	if (!fira_crypto_ctx)
		return -ENOMEM;

	memset(fira_crypto_ctx, 0, sizeof(*fira_crypto_ctx));

	/* Retrieve session key */
	switch (crypto_params->sts_config) {
	case FIRA_STS_MODE_STATIC:
		memcpy(session_key, "StaticTSStaticTS", AES_KEYSIZE_128);
		fira_crypto_ctx->base.key_size = AES_KEYSIZE_128;
		memcpy(fira_crypto_ctx->vupper64, crypto_params->vupper64, FIRA_VUPPER64_SIZE);
		break;

	case FIRA_STS_MODE_PROVISIONED:
	case FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY:
		if (crypto_params->key) {
			memcpy(session_key, crypto_params->key, crypto_params->key_len);
			fira_crypto_ctx->base.key_size = crypto_params->key_len;
		} else {
			/* Session key not set */
			pr_err("Session key not provisioned !\n");
			status = -1;
		}
		break;

	default:
		/* Bad value */
		pr_err("STS config unknown !\n");
		status = -1;
	}

	fira_crypto_ctx->sts_config = crypto_params->sts_config;

	if (!status) {
		/* Compute Config Digest */
		static const u8 zero_key[AES_KEYSIZE_128];

		status = mcps_crypto_cmac_aes_128_digest(zero_key,
				crypto_params->concat_params,
				crypto_params->concat_params_size,
				fira_crypto_ctx->base.config_digest);
		if (status)
			goto error_out;

		/* Compute secDataProtectionKey */
		status = fira_crypto_kdf(session_key,
				fira_crypto_ctx->base.key_size,
				"DataPrtK",
				fira_crypto_ctx->base.config_digest,
				fira_crypto_ctx->base.data_protection_key,
				FIRA_KEY_SIZE_MIN);
		if (status)
			goto error_out;

		if (crypto_params->sts_config == FIRA_STS_MODE_STATIC) {
			/* rotate keys only once for static_sts */
			status = fira_crypto_rotate_elements(
					fira_crypto_ctx,
					0);
		} else {

			/* Compute secPrivacy Key and setup AES ECB context */
			status = fira_crypto_kdf(
				session_key, fira_crypto_ctx->base.key_size,
				"PrivacyK", fira_crypto_ctx->base.config_digest,
				fira_crypto_ctx->privacy_key,
				FIRA_KEY_SIZE_MIN);
		}
		if (status)
			goto error_out;
	}

	/* Wipe session key */
	memzero_explicit(session_key, AES_KEYSIZE_128);

	*crypto = fira_crypto_ctx;

	return status;

error_out:
	/* Wipe session key */
	memzero_explicit(session_key, AES_KEYSIZE_128);
	*crypto = NULL;
	remove_session(fira_crypto_ctx);
	return status;
}

/**
 * fira_crypto_context_deinit() - De-initialize a crypto context.
 * @session_id: Pointer to the session id.
 */
void fira_crypto_context_deinit(struct fira_crypto *fira_crypto_ctx)
{
	if (fira_crypto_ctx) {
		/* Remove the context */
		remove_session(fira_crypto_ctx);
		fira_crypto_ctx = NULL;
	} else {
		/* The context doesn't exist */
		pr_err("Crypto context NULL\n");
	}
}

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
int fira_crypto_rotate_elements(struct fira_crypto *fira_crypto_ctx,
				const u32 crypto_sts_index)
{
	int r = 0;
	u8 context[AES_BLOCK_SIZE];

	if (!fira_crypto_ctx)
		goto error_out;

	memcpy(context, fira_crypto_ctx->base.config_digest + sizeof(u32),
			AES_BLOCK_SIZE - sizeof(u32));
	put_unaligned_be32(crypto_sts_index, context + AES_BLOCK_SIZE -
			sizeof(u32));

	r = fira_crypto_kdf(fira_crypto_ctx->base.data_protection_key,
			fira_crypto_ctx->base.key_size,
			"DerAuthI", context,
			fira_crypto_ctx->base.derived_authentication_iv,
			fira_crypto_ctx->base.key_size);
	if (r)
		goto error_out;

	r = fira_crypto_kdf(fira_crypto_ctx->base.data_protection_key,
			fira_crypto_ctx->base.key_size,
			"DerAuthK", context,
			fira_crypto_ctx->base.derived_authentication_key,
			fira_crypto_ctx->base.key_size);
	if (r)
		goto error_out;

	r = fira_crypto_kdf(fira_crypto_ctx->base.data_protection_key,
			fira_crypto_ctx->base.key_size,
			"DerPaylK", context,
			fira_crypto_ctx->base.derived_payload_key,
			fira_crypto_ctx->base.key_size);
	if (r)
		goto error_out;

	if (fira_crypto_ctx->base.aead.ctx)
		fira_crypto_aead_destroy(&fira_crypto_ctx->base.aead);

	r = fira_crypto_aead_set_key(&fira_crypto_ctx->base.aead,
			fira_crypto_ctx->base.derived_payload_key);

error_out:
	memzero_explicit(context, sizeof(context));
	return r;
}

u32 fira_crypto_get_phy_sts_index_init(const struct fira_crypto *crypto)
{
	if (!crypto)
		return -1;
	return crypto->phy_sts_index_init;
}

/**
 * fira_crypto_build_phy_sts_index_init() - Build the phy STS index init value
 * related to the given crypto context.
 *
 * @crypto: The context to use to compute the phy STS index init value.
 *
 * Return: 0 or error.
 */
int fira_crypto_build_phy_sts_index_init(struct fira_crypto *fira_crypto_ctx)
{
	int r = -1;
	u8 phy_sts_index_init_tmp[AES_KEYSIZE_128];

	if (!fira_crypto_ctx)
		goto error_out;

	r = fira_crypto_kdf(fira_crypto_ctx->base.data_protection_key,
			fira_crypto_ctx->base.key_size,
			"StsIndIn", fira_crypto_ctx->base.config_digest,
			phy_sts_index_init_tmp,
			fira_crypto_ctx->base.key_size);
	if (r)
		goto error_out;

	fira_crypto_ctx->phy_sts_index_init =
		get_unaligned_be32(phy_sts_index_init_tmp +
				fira_crypto_ctx->base.key_size - sizeof(u32)) &
				FIRA_CRYPTO_KEY_STS_MASK;
	return 0;

error_out:
	memzero_explicit(phy_sts_index_init_tmp,
			 sizeof(phy_sts_index_init_tmp));
	return r;

}

/**
 * fira_crypto_dynamic_get_sts_params() - Get STS parameters for a given slot
 * using a dynamic STS configuration.
 * @crypto: The context to use to get the STS parameters.
 * @crypto_sts_index: The crypto STS index related to the slot request slot.
 * @sts_v: Output buffer to store the STS V.
 * @sts_v_size: Size of the STS V buffer.
 * @sts_key: Output buffer to store the STS key.
 * @sts_key_size: Size of the STS key buffer.
 *
 * Return: 0 or error.
 */
int fira_crypto_get_sts_params(struct fira_crypto *fira_crypto_ctx,
		const u32 crypto_sts_index, u8 *sts_v, u32 sts_v_size,
		u8 *sts_key, u32 sts_key_size)
{
	u8 *vupper64 = NULL;
	u32 v_counter;
	u8 *sts_v_temp;

	if (!fira_crypto_ctx)
		return -1;

	if (fira_crypto_ctx->sts_config == FIRA_STS_MODE_STATIC)
		vupper64 = fira_crypto_ctx->vupper64;
	else
		vupper64 = fira_crypto_ctx->base.derived_authentication_iv;

	if (sts_v_size < AES_BLOCK_SIZE || sts_key_size < AES_KEYSIZE_128)
		return -EINVAL;

	sts_v_temp = sts_v;

	/* Concatenate the 128 bits of sts_v
	 * sts_v = vupper64 | crypto_sts_index | v_counter
	 */
	memcpy(sts_v_temp, vupper64, FIRA_VUPPER64_SIZE);
	sts_v_temp += FIRA_VUPPER64_SIZE;
	put_unaligned_be32(crypto_sts_index, sts_v_temp);
	sts_v_temp += sizeof(crypto_sts_index);
	v_counter = get_unaligned_be32(
			fira_crypto_ctx->base.derived_authentication_iv +
			AES_BLOCK_SIZE - sizeof(v_counter)) &
		FIRA_CRYPTO_KEY_STS_MASK;
	put_unaligned_be32(v_counter, sts_v_temp);

	memcpy(sts_key, fira_crypto_ctx->base.derived_authentication_key,
			fira_crypto_ctx->base.key_size);

	return 0;
}

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
int fira_crypto_encrypt_frame(struct fira_crypto *fira_crypto_ctx, struct sk_buff *skb,
		int header_len, __le16 src_short_addr, u32 crypto_sts_index)
{
	if (!fira_crypto_ctx)
		return -1;

	return fira_crypto_aead_encrypt(&fira_crypto_ctx->base.aead, skb, header_len,
					src_short_addr, crypto_sts_index);
}

/**
 * fira_crypto_decrypt_frame() - Decrypt a 802154 frame using a given context.
 *
 * NOTE: The src address is given as an argument as it is a part of the nonce needed
 * to decrypt the frame and it is not present in the 802154 frame.
 * The length of the header is given because only the payload is encrypted even if
 * the encryption algorithm needs the whole 802154 frame.
 * Decryption is done in-place.
 * When called, this function shall reduce the
 * size of the skb of FIRA_CRYPTO_AEAD_AUTHSIZE and verify the correct bits in the
 * 802154 frame SCF.
 *
 * @crypto: The crypto to use to decrypt the frame.
 * @skb: The buffer containing the whole frame, skb->data points to the start of
 * the 802154 frame payload.
 * @header_len: The length of the 802154 frame header. Can be used to find the
 * start of the 802154 frame payload relative to skb->data.
 * @src_short_addr: The short source address attached to the frame.
 * @crypto_sts_index: The crypto STS index attached to the frame.
 *
 * Return: 0 or error.
 */
int fira_crypto_decrypt_frame(struct fira_crypto *fira_crypto_ctx, struct sk_buff *skb,
		int header_len, __le16 src_short_addr, u32 crypto_sts_index)
{
	if (!fira_crypto_ctx)
		return -1;

	return fira_crypto_aead_decrypt(&fira_crypto_ctx->base.aead, skb, header_len,
					src_short_addr, crypto_sts_index);
}

/**
 * fira_crypto_encrypt_hie() - Encrypt the HIE in a FiRa 802154 frame.
 * @crypto: The context to use to get the STS parameters.
 * @skb: Buffer containing the frame to encrypt.
 * @hie_offset: Offset to the start of the HIE (relating to skb->data) to encrypt.
 * @hie_len: Length of the HIE to encrypt.
 *
 * Return: 0 or error.
 */
int fira_crypto_encrypt_hie(struct fira_crypto *fira_crypto_ctx, struct sk_buff *skb,
		int hie_offset, int hie_len)
{
	int rc;

	if (!fira_crypto_ctx)
		return -1;

	if (fira_crypto_ctx->sts_config == FIRA_STS_MODE_STATIC)
		return 0;

	fira_crypto_ctx->ecb_ctx = mcps_crypto_aes_ecb_128_create();
	if (!fira_crypto_ctx->ecb_ctx ||
		mcps_crypto_aes_ecb_128_set_encrypt(
			fira_crypto_ctx->ecb_ctx,
			fira_crypto_ctx->privacy_key))
		return -1;

	rc = mcps_crypto_aes_ecb_128_encrypt(fira_crypto_ctx->ecb_ctx,
			(const uint8_t *)(skb->data + hie_offset +
				IEEE802154_IE_HEADER_LEN +
				FIRA_IE_VENDOR_OUI_LEN),
			(unsigned int)hie_len - IEEE802154_IE_HEADER_LEN -
				FIRA_IE_VENDOR_OUI_LEN,
			(uint8_t *)(skb->data + hie_offset +
				IEEE802154_IE_HEADER_LEN +
				FIRA_IE_VENDOR_OUI_LEN));

	mcps_crypto_aes_ecb_128_destroy(fira_crypto_ctx->ecb_ctx);
	fira_crypto_ctx->ecb_ctx = NULL;

	return rc;
}

/**
 * fira_crypto_decrypt_hie() - Decrypt the HIE in a FiRa 802154 frame.
 * @crypto: The context to use to get the STS parameters.
 * @skb: Buffer containing the frame to decrypt.
 * @hie_offset: Offset to the start of the HIE (relative to skb->data) to decrypt.
 * @hie_len: Length of the HIE to decrypt.
 *
 * Return: 0 or error.
 */
int fira_crypto_decrypt_hie(struct fira_crypto *fira_crypto_ctx, struct sk_buff *skb,
		int hie_offset, int hie_len)
{
	int rc;

	if (!fira_crypto_ctx)
		return -1;

	if (fira_crypto_ctx->sts_config == FIRA_STS_MODE_STATIC)
		return 0;

	fira_crypto_ctx->ecb_ctx = mcps_crypto_aes_ecb_128_create();
	if (!fira_crypto_ctx->ecb_ctx ||
			mcps_crypto_aes_ecb_128_set_decrypt(
				fira_crypto_ctx->ecb_ctx,
				fira_crypto_ctx->privacy_key))
		return -1;

	rc = mcps_crypto_aes_ecb_128_encrypt(fira_crypto_ctx->ecb_ctx,
			(const uint8_t *)(skb->data + hie_offset),
			(unsigned int)hie_len,
			(uint8_t *)(skb->data + hie_offset));

	mcps_crypto_aes_ecb_128_destroy(fira_crypto_ctx->ecb_ctx);
	fira_crypto_ctx->ecb_ctx = NULL;

	return rc;
}

/**
 * fira_crypto_get_capabilities() - Get capabilities of the platform used.
 *
 * Return:
 *    bit 0 : FIRA_STS_MODE_STATIC supported
 *    bit 1 : FIRA_STS_MODE_DYNAMIC supported
 *    bit 2 : FIRA_STS_MODE_DYNAMIC_INDIVIDUAL_KEY supported
 *    bit 3 : FIRA_STS_MODE_PROVISIONED supported
 *    bit 4 : FIRA_STS_MODE_PROVISIONED_INDIVIDUAL_KEY supported
 *    other : not used
 */
u32 fira_crypto_get_capabilities(void)
{
	u32 status = 0;

	status += STS_CAP(STATIC);
	status += STS_CAP(PROVISIONED);
	status += STS_CAP(PROVISIONED_INDIVIDUAL_KEY);

#ifdef CONFIG_FIRA_CRYPTO_HAVE_SE
	status += STS_CAP(DYNAMIC);
#endif

	return status;
}

int fira_crypto_prepare_decrypt(struct fira_crypto *crypto, struct sk_buff *skb)
{
	u8 scf;
	u8 *p;

	p = skb->data - (IEEE802154_FC_LEN + IEEE802154_SHORT_ADDR_LEN +
			 IEEE802154_SCF_LEN);
	scf = p[IEEE802154_FC_LEN + IEEE802154_SHORT_ADDR_LEN];
	if (!(scf == (IEEE802154_SCF_SECLEVEL_ENC_MIC64 |
		      IEEE802154_SCF_NO_FRAME_COUNTER)) ||
	    (skb->len < FIRA_CRYPTO_AEAD_AUTHSIZE))
		return -EBADMSG;
	skb_trim(skb, skb->len - FIRA_CRYPTO_AEAD_AUTHSIZE);

	return 0;
}

static bool compare_bufs(const void *a, size_t alen, const void *b, size_t blen)
{
	if (alen != blen || memcmp(a, b, alen) != 0) {
#ifdef __KERNEL__
		print_hex_dump(KERN_ERR, "a: ", DUMP_PREFIX_OFFSET, 16, 1, a,
				alen, false);
		print_hex_dump(KERN_ERR, "b: ", DUMP_PREFIX_OFFSET, 16, 1, b,
				blen, false);
#endif
		return false;
	}

	return true;
}

/**
 * fira_crypto_test_static()
 * Run the FIRA CONSORTIUM UWB MAC TECHNICAL REQUIREMENTS
 * version 1.3.0 test vectors for Static STS
 *
 * NOTE: This APis used for unit tests only.
 *
 * Return: 0 if ok
 */
static int fira_crypto_test_static(void)
{
	/* Static STS */
	static const u8 config[] = {
		0x02, 0x00, 0x00, 0x09, 0x07, 0xd0, 0x00, 0x03,
		0x0a, 0x02, 0x00, 0x01, 0x03, 0x01, 0x23, 0x45,
		0x67
	};
	static const u8 vUpp[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	static const struct fira_crypto_params param = {
		.session_id = 0x01234567,
		.sts_config = FIRA_STS_MODE_STATIC,
		.concat_params = config,
		.concat_params_size = sizeof(config),
		.vupper64 = vUpp
	};
	static const u8 configDigest[] = {
		0xa0, 0x43, 0x90, 0xcf, 0x8a, 0x33, 0xf6, 0xeb,
		0x7e, 0x2f, 0xc3, 0x78, 0x87, 0xb6, 0xb2, 0xa3
	};
	static const u8 secDataProtectionKey[] = {
		0xf3, 0x21, 0x6c, 0x87, 0xd0, 0xc6, 0x93, 0x2e,
		0x39, 0x57, 0xb4, 0x81, 0xfa, 0xb8, 0xb2, 0x09
	};
	static const u8 derived_authentication_iv[] = {
		0x8b, 0x54, 0x37, 0x6e, 0x7c, 0xd7, 0xa5, 0xd6,
		0x6b, 0xd1, 0x20, 0x00, 0x97, 0x27, 0x41, 0x19
	};
	static const u8 derived_authentication_key[] = {
		0xdd, 0x98, 0x97, 0xf2, 0xb8, 0x5c, 0x9d, 0xc8,
		0xa7, 0xde, 0xc0, 0x1c, 0xca, 0x5b, 0x61, 0xdb
	};
	static const u8 derived_payload_key[] = {
		0xa5, 0x5f, 0xab, 0x83, 0xb6, 0x20, 0xf9, 0xf6,
		0xa4, 0x7c, 0xdb, 0x72, 0x91, 0x7c, 0x73, 0x8a
	};
	static const u8 sts_v_ref[] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x00, 0x00, 0x00, 0x00, 0x17, 0x27, 0x41, 0x19
	};
	/* build the RCM Frame */
	/* build the header */
	static const u8 RCM[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
		0x08, 0x08, 0x67, 0x45, 0x23, 0x01, 0x78, 0xbe,
		0x9b, 0x0b, 0x00, 0x3f, 0x1b, 0x90, 0xff, 0x18,
		0x5a, 0x03, 0x05, 0x00, 0x00, 0x03, 0x42, 0x55,
		0x01, 0x04, 0x44, 0x55, 0x03, 0x07, 0x42, 0x55,
		0x05, 0x09, 0x42, 0x55, 0x09, 0x0a, 0x44, 0x55,
		0x0b
	};
	static const u8 RCMRef[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
		0x08, 0x08, 0x67, 0x45, 0x23, 0x01, 0x78, 0xbe,
		0x9b, 0x0b, 0x00, 0x3f, 0xcb, 0xa4, 0xfd, 0x37,
		0xd1, 0x99, 0x44, 0x88, 0x7c, 0x2b, 0xec, 0x2e,
		0x1a, 0x99, 0x8e, 0x80, 0x61, 0x7c, 0x44, 0xb5,
		0xe8, 0xe3, 0xf3, 0x35, 0x3a, 0xb9, 0xf2, 0x29,
		0x1b, 0x80, 0x4b, 0xba, 0xe1, 0xa9, 0x2a, 0x20,
		0x28
	};
	static const u8 Header[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
		0x08, 0x08, 0x67, 0x45, 0x23, 0x01, 0x78, 0xbe,
		0x9b, 0x0b
	};
	static const u8 HeaderRef[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
		0x08, 0x08, 0x67, 0x45, 0x23, 0x01, 0x78, 0xbe,
		0x9b, 0x0b
	};
	/* Decrypt Frame */
	static const u8 RCM_Rcv_Ref[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
		0x08, 0x08, 0x67, 0x45, 0x23, 0x01, 0x78, 0xbe,
		0x9b, 0x0b, 0x00, 0x3f, 0x1b, 0x90, 0xff, 0x18,
		0x5a, 0x03, 0x05, 0x00, 0x00, 0x03, 0x42, 0x55,
		0x01, 0x04, 0x44, 0x55, 0x03, 0x07, 0x42, 0x55,
		0x05, 0x09, 0x42, 0x55, 0x09, 0x0a, 0x44, 0x55,
		0x0b
	};
	static const u8 Frame_Rcv[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
		0x08, 0x08, 0x67, 0x45, 0x23, 0x01, 0x78, 0xbe,
		0x9b, 0x0b, 0x00, 0x3f, 0xcb, 0xa4, 0xfd, 0x37,
		0xd1, 0x99, 0x44, 0x88, 0x7c, 0x2b, 0xec, 0x2e,
		0x1a, 0x99, 0x8e, 0x80, 0x61, 0x7c, 0x44, 0xb5,
		0xe8, 0xe3, 0xf3, 0x35, 0x3a, 0xb9, 0xf2, 0x29,
		0x1b, 0x80, 0x4b, 0xba, 0xe1, 0xa9, 0x2a, 0x20,
		0x28
	};
	static const u8 Header_Rcv[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
		0x08, 0x08, 0x67, 0x45, 0x23, 0x01, 0x78, 0xbe,
		0x9b, 0x0b
	};
	static const u8 HeaderRef_Rcv[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
		0x08, 0x08, 0x67, 0x45, 0x23, 0x01, 0x78, 0xbe,
		0x9b, 0x0b
	};
	int err = -1, r;
	struct fira_crypto *fira_crypto_ctx;
	const u32 crypto_sts_index = 0;
	u8 sts_v[16];
	u8 sts_key[16];
	struct sk_buff *skb = NULL;

	r = fira_crypto_context_init(&param, &fira_crypto_ctx);
	if (r != 0 || !fira_crypto_ctx) {
		pr_err("fira_crypto_context_init fail: %d\n", r);
		goto end;
	}

	if (!compare_bufs(configDigest, sizeof(configDigest),
				fira_crypto_ctx->base.config_digest,
				sizeof(fira_crypto_ctx->base.config_digest))) {
		pr_err("compare configDigest fail\n");
		goto end;
	}

	if (!compare_bufs(secDataProtectionKey, sizeof(secDataProtectionKey),
				fira_crypto_ctx->base.data_protection_key,
				sizeof(fira_crypto_ctx->base.data_protection_key))) {
		pr_err("compare secDataProtectionKey fail\n");
		goto end;
	}

	if (!compare_bufs(derived_authentication_iv,
				sizeof(derived_authentication_iv),
				fira_crypto_ctx->base.derived_authentication_iv,
				sizeof(fira_crypto_ctx->base.derived_authentication_iv))) {
		pr_err("compare derived_authentication_iv fail\n");
		goto end;
	}

	if (!compare_bufs(derived_authentication_key,
				sizeof(derived_authentication_key),
				fira_crypto_ctx->base.derived_authentication_key,
				sizeof(fira_crypto_ctx->base.derived_authentication_key))) {
		pr_err("compare derived_authentication_key fail\n");
		goto end;
	}

	if (!compare_bufs(derived_payload_key, sizeof(derived_payload_key),
				fira_crypto_ctx->base.derived_payload_key,
				sizeof(fira_crypto_ctx->base.derived_payload_key))) {
		pr_err("compare derived_payload_key fail\n");
		goto end;
	}

	r = fira_crypto_build_phy_sts_index_init(fira_crypto_ctx);
	if (r != 0) {
		pr_err("fira_crypto_build_phy_sts_index_init fail: %d\n", r);
		goto end;
	}
	if (fira_crypto_ctx->phy_sts_index_init != 0x0b9bbe78) {
		pr_err("phy_sts_index_init fail\n");
		goto end;
	}

	r = fira_crypto_get_sts_params(fira_crypto_ctx, crypto_sts_index, sts_v,
				sizeof(sts_v), sts_key, sizeof(sts_key));
	if (r != 0) {
		pr_err("fira_crypto_get_sts_params fail: %d\n", r);
		goto end;
	}
	if (!compare_bufs(derived_authentication_key,
				sizeof(derived_authentication_key),
				sts_key, sizeof(sts_key))) {
		pr_err("compare sts_key fail\n");
		goto end;
	}

	if (!compare_bufs(sts_v_ref, sizeof(sts_v_ref), sts_v, sizeof(sts_v))) {
		pr_err("compare sts_v fail\n");
		goto end;
	}

	skb = alloc_skb(128, GFP_KERNEL);
	if (!skb) {
		pr_err("cannot allocate skb\n");
		goto end;
	}

	skb_put_data(skb, Header, sizeof(Header));

	/* Encrypt Header first (NOP in Static STS) */
	r = fira_crypto_encrypt_hie(fira_crypto_ctx, skb, 5, 21);
	if (r != 0) {
		pr_err("fira_crypto_encrypt_hie fail: %d\n", r);
		goto end;
	}
	if (!compare_bufs(HeaderRef, sizeof(HeaderRef), skb->data, skb->len)) {
		pr_err("fira_crypto_encrypt_hie compare HeaderRef fail\n");
		goto end;
	}

	kfree_skb(skb);

	skb = alloc_skb(128, GFP_KERNEL);
	if (!skb) {
		pr_err("cannot allocate skb\n");
		goto end;
	}

	skb_put_data(skb, RCM, sizeof(RCM));

	r = fira_crypto_encrypt_frame(fira_crypto_ctx, skb, 28, 0xaaa1, 0);
	if (r != 0) {
		pr_err("fira_crypto_encrypt_frame fail: %d\n", r);
		goto end;
	}

	if (!compare_bufs(RCMRef, sizeof(RCMRef), skb->data, skb->len)) {
		pr_err("fira_crypto_encrypt_frame compare RCMRef fail\n");
		goto end;
	}

	kfree_skb(skb);

	skb = alloc_skb(128, GFP_KERNEL);
	if (!skb) {
		pr_err("cannot allocate skb\n");
		goto end;
	}

	skb_put_data(skb, Frame_Rcv, sizeof(Frame_Rcv));
	skb_pull(skb, 28); /* skip header */

	skb_trim(skb, skb->len - FIRA_CRYPTO_AEAD_AUTHSIZE);
	r = fira_crypto_decrypt_frame(fira_crypto_ctx, skb, 28, 0xaaa1, 0);
	if (r != 0) {
		pr_err("fira_crypto_decrypt_frame fail: %d\n", r);
		goto end;
	}

	skb_push(skb, 28); /* restore header */

	if (!compare_bufs(RCM_Rcv_Ref, sizeof(RCM_Rcv_Ref), skb->data, skb->len)) {
		pr_err("fira_crypto_decrypt_frame compare RCM_Rcv_Ref fail\n");
		goto end;
	}

	kfree_skb(skb);

	skb = alloc_skb(128, GFP_KERNEL);
	if (!skb) {
		pr_err("cannot allocate skb\n");
		goto end;
	}

	skb_put_data(skb, Header_Rcv, sizeof(Header_Rcv));

	/* Decrypt header (NOP in Static STS) */
	r = fira_crypto_decrypt_hie(fira_crypto_ctx, skb, 10, 16);
	if (r != 0) {
		pr_err("fira_crypto_decrypt_hie fail: %d\n", r);
		goto end;
	}
	if (!compare_bufs(HeaderRef_Rcv, sizeof(HeaderRef_Rcv), skb->data,
				skb->len)) {
		pr_err("fira_crypto_decrypt_hie compare HeaderRef_Rcv fail\n");
		goto end;
	}

	err = 0;

	pr_info("Static STS tests success\n");

end:
	if (skb)
		kfree_skb(skb);
	if (fira_crypto_ctx)
		fira_crypto_context_deinit(fira_crypto_ctx);

	return err;
}

/**
 * fira_crypto_test_provisioned()
 * Run the FIRA CONSORTIUM UWB MAC TECHNICAL REQUIREMENTS
 * version 1.3.0 test vectors for Dynamic STS (Provisioned STS is ran instead of
 * pure dynamic)
 *
 * NOTE: This APis used for unit tests only.
 *
 * Return: 0 if ok
 */
static int fira_crypto_test_provisioned(void)
{
	/* Provisioned STS (equivalent to D-STS) */
	static const u8 config_P_STS[] = {
		0x02, 0x01, 0x00, 0x09, 0x07, 0xD0, 0x00, 0x03,
		0x0a, 0x02, 0x00, 0x01, 0x03, 0x01, 0x23, 0x45,
		0x67
	};
	static const u8 sessionKey[] = {
		0x44, 0x79, 0x6e, 0x61, 0x6d, 0x69, 0x63, 0x53,
		0x54, 0x53, 0x4e, 0x6f, 0x52, 0x6f, 0x74, 0x30
	};
	static const struct fira_crypto_params param_p_sts = {
		.session_id = 0x01234567,
		.sts_config = FIRA_STS_MODE_PROVISIONED,
		.concat_params = config_P_STS,
		.concat_params_size = sizeof(config_P_STS),
		.key = sessionKey,
		.key_len = sizeof(sessionKey)
	};
	static const u8 configDigest_p_sts[] = {
		0x08, 0x93, 0x66, 0xba, 0xfb, 0x3b, 0x24, 0xbf,
		0xd2, 0x93, 0x33, 0x77, 0x61, 0xb8, 0x8f, 0xc3
	};
	static const u8 secDataPrivacyKey_p_sts[] = {
		0x3a, 0x4b, 0xab, 0x18, 0x74, 0x4a, 0xee, 0x93,
		0x86, 0x50, 0xf1, 0xa0, 0x3f, 0x58, 0x5a, 0x49
	};
	static const u8 secDataProtectionKey_p_sts[] = {
		0x67, 0xf7, 0x02, 0x7e, 0xa6, 0x2d, 0x84, 0xa5,
		0xe1, 0xa8, 0xd7, 0xb8, 0xb8, 0xac, 0xae, 0xaf
	};
	static const u8 derived_authentication_iv_p_sts[] = {
		0xfa, 0x32, 0x6f, 0xed, 0x87, 0xd2, 0xef, 0x7e,
		0xb6, 0x80, 0xb2, 0xd6, 0xd1, 0x19, 0xa9, 0xb8
	};
	static const u8 derived_authentication_key_p_sts[] = {
		0x91, 0xa2, 0xde, 0x58, 0xff, 0x3b, 0x5e, 0x85,
		0x15, 0x33, 0x58, 0xd6, 0x15, 0x64, 0x64, 0xff
	};
	static const u8 derived_payload_key_p_sts[] = {
		0x97, 0xe4, 0xab, 0x69, 0x61, 0x77, 0xbb, 0x39,
		0x92, 0x77, 0xb8, 0x35, 0x9f, 0xa5, 0x5d, 0x19
	};
	static const u8 sts_v_ref_p_sts[] = {
		0xfa, 0x32, 0x6f, 0xed, 0x87, 0xd2, 0xef, 0x7e,
		0x04, 0x1f, 0x3b, 0xa0, 0x51, 0x19, 0xa9, 0xb8
	};
	/* build the RCM Frame */
	static const u8 RCM_p_sts[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x84, 0x6c, 0xa4, 0x3c, 0x52, 0xfb,
		0xb0, 0x2b, 0x56, 0xa9, 0x87, 0x9d, 0xb0, 0x4e,
		0x4e, 0x03, 0x00, 0x3f, 0x1b, 0x90, 0xff, 0x18,
		0x5a, 0x03, 0x05, 0x00, 0x00, 0x03, 0x42, 0x55,
		0x01, 0x04, 0x44, 0x55, 0x03, 0x07, 0x42, 0x55,
		0x05, 0x09, 0x42, 0x55, 0x09, 0x0a, 0x44, 0x55,
		0x0b
	};
	static const u8 RCMRef_p_sts[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x84, 0x6c, 0xa4, 0x3c, 0x52, 0xfb,
		0xb0, 0x2b, 0x56, 0xa9, 0x87, 0x9d, 0xb0, 0x4e,
		0x4e, 0x03, 0x00, 0x3f, 0x82, 0x76, 0xe0, 0x44,
		0xf3, 0x78, 0xab, 0xbe, 0xd2, 0x39, 0x86, 0x7e,
		0xd2, 0xfe, 0x5c, 0x9d, 0xcd, 0x13, 0x1d, 0x1f,
		0x63, 0x38, 0xf1, 0xf7,	0x9d, 0xb1, 0x84, 0x71,
		0x72, 0x7a, 0x10, 0xfc, 0x80, 0x04, 0x7e, 0xdb,
		0x0f
	};
	static const u8 Header_p_sts[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
		0x08, 0x08, 0x67, 0x45, 0x23, 0x01, 0xa0, 0x3b,
		0x1f, 0x04
	};
	static const u8 HeaderRef_p_sts[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x84, 0x6c, 0xa4, 0x3c, 0x52, 0xfb,
		0xb0, 0x2b, 0x56, 0xa9, 0x87, 0x9d, 0xb0, 0x4e,
		0x4e, 0x03
	};
	/* Decrypt Frame */
	static const u8 Header_RCM_p_sts_Rcv[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x84, 0x6c, 0xa4, 0x3c, 0x52, 0xfb,
		0xb0, 0x2b, 0x56, 0xa9, 0x87, 0x9d, 0xb0, 0x4e,
		0x4e, 0x03, 0x00, 0x3f, 0x82, 0x76, 0xe0, 0x44,
		0xf3, 0x78, 0xab, 0xbe, 0xd2, 0x39, 0x86, 0x7e,
		0xd2, 0xfe, 0x5c, 0x9d, 0xcd, 0x13, 0x1d, 0x1f,
		0x63, 0x38, 0xf1, 0xf7, 0x9d, 0xb1, 0x84, 0x71,
		0x72, 0x7a, 0x10, 0xfc, 0x80, 0x04, 0x7e, 0xdb,
		0x0f
	};
	static const u8 RCMRef_p_sts_Rcv[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x84, 0x6c, 0xa4, 0x3c, 0x52, 0xfb,
		0xb0, 0x2b, 0x56, 0xa9, 0x87, 0x9d, 0xb0, 0x4e,
		0x4e, 0x03, 0x00, 0x3f, 0x1b, 0x90, 0xff, 0x18,
		0x5a, 0x03, 0x05, 0x00, 0x00, 0x03, 0x42, 0x55,
		0x01, 0x04, 0x44, 0x55, 0x03, 0x07, 0x42, 0x55,
		0x05, 0x09, 0x42, 0x55, 0x09, 0x0a, 0x44, 0x55,
		0x0b
	};
	static const u8 Header_p_sts_Rcv[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x84, 0x6c, 0xa4, 0x3c, 0x52, 0xfb,
		0xb0, 0x2b, 0x56, 0xa9, 0x87, 0x9d, 0xb0, 0x4e,
		0x4e, 0x03
	};
	static const u8 HeaderRef_p_sts_Rcv[] = {
		0x49, 0x2b, 0xa2, 0xaa, 0x26, 0x13, 0x00, 0xff,
		0x18, 0x5a, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
		0x08, 0x08, 0x67, 0x45, 0x23, 0x01, 0xa0, 0x3b,
		0x1f, 0x04
	};
	int err = -1, r;
	struct fira_crypto *fira_crypto_ctx;
	u32 crypto_sts_index_p_sts = 0;
	u8 sts_v[16];
	u8 sts_key[16];
	struct sk_buff *skb = NULL;

	r = fira_crypto_context_init(&param_p_sts, &fira_crypto_ctx);
	if (r != 0 || !fira_crypto_ctx) {
		pr_err("fira_crypto_context_init fail: %d\n", r);
		goto end;
	}

	if (!compare_bufs(configDigest_p_sts, sizeof(configDigest_p_sts),
				fira_crypto_ctx->base.config_digest,
				sizeof(fira_crypto_ctx->base.config_digest))) {
		pr_err("compare configDigest_p_sts fail\n");
		goto end;
	}

	if (!compare_bufs(secDataPrivacyKey_p_sts,
				sizeof(secDataPrivacyKey_p_sts),
				fira_crypto_ctx->privacy_key,
				sizeof(fira_crypto_ctx->privacy_key))) {
		pr_err("compare secDataPrivacyKey_p_sts fail\n");
		goto end;
	}

	if (!compare_bufs(secDataProtectionKey_p_sts,
				sizeof(secDataProtectionKey_p_sts),
				fira_crypto_ctx->base.data_protection_key,
				sizeof(fira_crypto_ctx->base.data_protection_key))) {
		pr_err("compare secDataProtectionKey fail\n");
		goto end;
	}

	r = fira_crypto_build_phy_sts_index_init(fira_crypto_ctx);
	if (r != 0) {
		pr_err("fira_crypto_build_phy_sts_index_init fail: %d\n", r);
		goto end;
	}
	if (fira_crypto_ctx->phy_sts_index_init != 0x041f3ba0) {
		pr_err("phy_sts_index_init fail\n");
		goto end;
	}

	r = fira_crypto_rotate_elements(fira_crypto_ctx,
					fira_crypto_ctx->phy_sts_index_init);

	if (r != 0) {
		pr_err("fira_crypto_rotate_elements fail: %d\n", r);
		goto end;
	}

	if (!compare_bufs(derived_authentication_iv_p_sts,
				sizeof(derived_authentication_iv_p_sts),
				fira_crypto_ctx->base.derived_authentication_iv,
				sizeof(fira_crypto_ctx->base.derived_authentication_iv))) {
		pr_err("compare derived_authentication_iv_p_sts fail\n");
		goto end;
	}

	if (!compare_bufs(derived_authentication_key_p_sts,
				sizeof(derived_authentication_key_p_sts),
				fira_crypto_ctx->base.derived_authentication_key,
				sizeof(fira_crypto_ctx->base.derived_authentication_key))) {
		pr_err("compare derived_authentication_key_p_sts fail\n");
		goto end;
	}

	if (!compare_bufs(derived_payload_key_p_sts,
				sizeof(derived_payload_key_p_sts),
				fira_crypto_ctx->base.derived_payload_key,
				sizeof(fira_crypto_ctx->base.derived_payload_key))) {
		pr_err("compare derived_payload_key fail\n");
		goto end;
	}

	/* Return STS parameters slot 0 */
	crypto_sts_index_p_sts = 0x041f3ba0;
	r = fira_crypto_get_sts_params(fira_crypto_ctx, crypto_sts_index_p_sts, sts_v,
				sizeof(sts_v), sts_key, sizeof(sts_key));
	if (r != 0) {
		pr_err("fira_crypto_get_sts_params fail: %d\n", r);
		goto end;
	}
	if (!compare_bufs(derived_authentication_key_p_sts,
				sizeof(derived_authentication_key_p_sts),
				sts_key, sizeof(sts_key))) {
		pr_err("compare sts_key Fail\n");
		goto end;
	}
	if (!compare_bufs(sts_v_ref_p_sts, sizeof(sts_v_ref_p_sts), sts_v,
				sizeof(sts_v))) {
		pr_err("compare sts_v_ref_p_sts Fail\n");
		goto end;
	}

	skb = alloc_skb(128, GFP_KERNEL);
	if (!skb) {
		pr_err("cannot allocate skb\n");
		goto end;
	}

	skb_put_data(skb, Header_p_sts, sizeof(Header_p_sts));

	/* Encrypt Header first */
	r = fira_crypto_encrypt_hie(fira_crypto_ctx, skb, 5, 16);
	if (r != 0) {
		pr_err("fira_crypto_encrypt_hie fail: %d\n", r);
		goto end;
	}
	if (!compare_bufs(HeaderRef_p_sts, sizeof(HeaderRef_p_sts), skb->data,
				skb->len)) {
		pr_err("compare HeaderRef_p_sts fail\n");
		goto end;
	}

	kfree_skb(skb);

	skb = alloc_skb(128, GFP_KERNEL);
	if (!skb) {
		pr_err("cannot allocate skb\n");
		goto end;
	}

	skb_put_data(skb, RCM_p_sts, sizeof(RCM_p_sts));

	r = fira_crypto_encrypt_frame(fira_crypto_ctx, skb, 28, 0xaaa1, 0x041f3ba0);

	if (r != 0) {
		pr_err("fira_crypto_encrypt_frame fail: %d\n", r);
		goto end;
	}
	if (!compare_bufs(RCMRef_p_sts, sizeof(RCMRef_p_sts), skb->data,
				skb->len)) {
		pr_err("compare RCMRef_p_sts fail\n");
		goto end;
	}

	kfree_skb(skb);

	skb = alloc_skb(128, GFP_KERNEL);
	if (!skb) {
		pr_err("cannot allocate skb\n");
		goto end;
	}

	skb_put_data(skb, Header_RCM_p_sts_Rcv, sizeof(Header_RCM_p_sts_Rcv));
	skb_pull(skb, 28); /* skip header */

	skb_trim(skb, skb->len - FIRA_CRYPTO_AEAD_AUTHSIZE);
	r = fira_crypto_decrypt_frame(fira_crypto_ctx, skb, 28, 0xaaa1, 0x041f3ba0);
	if (r != 0) {
		pr_err("fira_crypto_decrypt_frame fail: %d\n", r);
		goto end;
	}

	skb_push(skb, 28); /* restore header */

	if (!compare_bufs(RCMRef_p_sts_Rcv, sizeof(RCMRef_p_sts_Rcv), skb->data,
				skb->len)) {
		pr_err("compare RCMRef_p_sts_Rcv fail\n");
		goto end;
	}

	kfree_skb(skb);

	skb = alloc_skb(128, GFP_KERNEL);
	if (!skb) {
		pr_err("cannot allocate skb\n");
		goto end;
	}

	skb_put_data(skb, Header_p_sts_Rcv, sizeof(Header_p_sts_Rcv));

	/* Decrypt header */
	r = fira_crypto_decrypt_hie(fira_crypto_ctx, skb, 10, 16);
	if (r != 0) {
		pr_err("fira_crypto_decrypt_hie fail: %d\n", r);
		goto end;
	}
	if (!compare_bufs(HeaderRef_p_sts_Rcv, sizeof(HeaderRef_p_sts_Rcv),
				skb->data, skb->len)) {
		pr_err("compare HeaderRef_p_sts_Rcv fail\n");
		goto end;
	}

	err = 0;

	pr_info("Provisioned STS tests success\n");

end:
	if (skb)
		kfree_skb(skb);
	if (fira_crypto_ctx)
		fira_crypto_context_deinit(fira_crypto_ctx);

	return err;
}

/**
 * fira_crypto_test() - Run the FIRA CONSORTIUM UWB MAC TECHNICAL REQUIREMENTS
 * version 1.3.0 test vectors for Static STS and Dynamic STS (Provisioned STS is
 * ran instead of pure dynnamic)
 *
 *
 * NOTE: This APis used for unit tests only.
 *
 * Return: 0 if ok,
 */
int fira_crypto_test(void)
{
	int r = 0;

	r = fira_crypto_test_static() || r;
	r = fira_crypto_test_provisioned() || r;

	return r ? -1 : 0;
}
