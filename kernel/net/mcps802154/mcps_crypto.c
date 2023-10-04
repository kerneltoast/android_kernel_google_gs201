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

#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <crypto/hash.h>
#include <crypto/skcipher.h>
#include <crypto/aead.h>
#include <crypto/aes.h>

#include "mcps_crypto.h"

#if !(defined(CONFIG_CRYPTO_HASH2) && defined(CONFIG_CRYPTO_AEAD2))
#error "required CONFIG_CRYPTO_HASH2 && CONFIG_CRYPTO_AEAD2"
#endif

#define FIRA_CRYPTO_AEAD_AUTHSIZE	8


struct mcps_aes_ccm_star_128_ctx {
	struct crypto_aead *tfm;
};

struct mcps_aes_ecb_128_ctx {
	struct crypto_skcipher *tfm;
	bool decrypt;
};


int mcps_crypto_cmac_aes_128_digest(const uint8_t *key, const uint8_t *data,
		unsigned int data_len, uint8_t *out)
{
	struct crypto_shash *tfm;
	int r;

	tfm = crypto_alloc_shash("cmac(aes)", 0, 0);
	if (IS_ERR(tfm)) {
		if (PTR_ERR(tfm) == -ENOENT)
			pr_err("The crypto transform cmac(aes) seems to be missing."
			       " Please check your kernel configuration.\n");
		return PTR_ERR(tfm);
	}

	r = crypto_shash_setkey(tfm, key, AES_KEYSIZE_128);
	if (r != 0)
		goto out;

	do {
		/* tfm need to be allocated for kernel < 4.20, so don't remove
		 * this do..while block
		 */
		SHASH_DESC_ON_STACK(desc, tfm);

		desc->tfm = tfm;

		r = crypto_shash_init(desc);
		if (r != 0)
			goto out;

		r = crypto_shash_finup(desc, data, data_len, out);
	} while (0);

out:
	crypto_free_shash(tfm);

	return r;
}

struct mcps_aes_ccm_star_128_ctx *mcps_crypto_aead_aes_ccm_star_128_create(void)
{
	struct mcps_aes_ccm_star_128_ctx *ctx;
	int r;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		goto error;

	ctx->tfm = crypto_alloc_aead("ccm(aes)", 0, 0);
	if (IS_ERR(ctx->tfm)) {
		if (PTR_ERR(ctx->tfm) == -ENOENT)
			pr_err("The crypto transform ccm(aes) seems to be missing."
			       " Please check your kernel configuration.\n");
		goto error;
	}

	r = crypto_aead_setauthsize(ctx->tfm, FIRA_CRYPTO_AEAD_AUTHSIZE);
	if (r != 0)
		goto error;

	return ctx;

error:
	mcps_crypto_aead_aes_ccm_star_128_destroy(ctx);

	return NULL;
}

int mcps_crypto_aead_aes_ccm_star_128_set(struct mcps_aes_ccm_star_128_ctx *ctx,
		const uint8_t *key)
{
	if (!ctx || !key)
		return -EINVAL;

	return crypto_aead_setkey(ctx->tfm, key, AES_KEYSIZE_128);
}

void mcps_crypto_aead_aes_ccm_star_128_destroy(struct mcps_aes_ccm_star_128_ctx *ctx)
{
	if (!ctx)
		return;

	crypto_free_aead(ctx->tfm);
	kfree(ctx);
}

int mcps_crypto_aead_aes_ccm_star_128_encrypt(
		struct mcps_aes_ccm_star_128_ctx *ctx, const uint8_t *nonce,
		const uint8_t *header, unsigned int header_len,
		uint8_t *data, unsigned int data_len,
		uint8_t *mac, unsigned int mac_len)
{
	struct aead_request *req = NULL;
	struct scatterlist sg[3];
	u8 iv[AES_BLOCK_SIZE];
	DECLARE_CRYPTO_WAIT(wait);
	int r = -1;

	if (!ctx || !nonce || !header || header_len <= 0 || !data ||
			data_len <= 0 || !mac ||
			mac_len != FIRA_CRYPTO_AEAD_AUTHSIZE) {
		return -EINVAL;
	}

	req = aead_request_alloc(ctx->tfm, GFP_KERNEL);
	if (!req) {
		r = -ENOMEM;
		goto end;
	}

	sg_init_table(sg, ARRAY_SIZE(sg));
	sg_set_buf(&sg[0], header, header_len);
	sg_set_buf(&sg[1], data, data_len);
	sg_set_buf(&sg[2], mac, mac_len);

	iv[0] = sizeof(u16) - 1;
	memcpy(iv + 1, nonce, MCPS_CRYPTO_AES_CCM_STAR_NONCE_LEN);

	aead_request_set_callback(req,
			CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
			crypto_req_done, &wait);
	aead_request_set_ad(req, header_len);
	aead_request_set_crypt(req, sg, sg, data_len, iv);

	r = crypto_wait_req(crypto_aead_encrypt(req), &wait);

end:
	aead_request_free(req);

	return r;
}

int mcps_crypto_aead_aes_ccm_star_128_decrypt(
		struct mcps_aes_ccm_star_128_ctx *ctx, const uint8_t *nonce,
		const uint8_t *header, unsigned int header_len,
		uint8_t *data, unsigned int data_len,
		uint8_t *mac, unsigned int mac_len)
{
	struct aead_request *req = NULL;
	struct scatterlist sg[3];
	u8 iv[AES_BLOCK_SIZE];
	DECLARE_CRYPTO_WAIT(wait);
	int r = -1;

	if (!ctx || !nonce || !header || header_len <= 0 || !data ||
			data_len <= 0 || !mac ||
			mac_len != FIRA_CRYPTO_AEAD_AUTHSIZE) {
		return -EINVAL;
	}

	req = aead_request_alloc(ctx->tfm, GFP_KERNEL);
	if (!req) {
		r = -ENOMEM;
		goto end;
	}

	iv[0] = sizeof(u16) - 1;
	memcpy(iv + 1, nonce, MCPS_CRYPTO_AES_CCM_STAR_NONCE_LEN);

	sg_init_table(sg, ARRAY_SIZE(sg));
	sg_set_buf(&sg[0], header, header_len);
	sg_set_buf(&sg[1], data, data_len);
	sg_set_buf(&sg[2], mac, mac_len);

	aead_request_set_callback(req,
			CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
			crypto_req_done, &wait);
	aead_request_set_ad(req, header_len);
	aead_request_set_crypt(req, sg, sg, data_len + mac_len, iv);

	r = crypto_wait_req(crypto_aead_decrypt(req), &wait);

end:
	aead_request_free(req);

	return r;
}

struct mcps_aes_ecb_128_ctx *mcps_crypto_aes_ecb_128_create(void)
{
	struct mcps_aes_ecb_128_ctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		goto error;

	ctx->tfm = crypto_alloc_skcipher("ecb(aes)", 0, 0);
	if (IS_ERR(ctx->tfm)) {
		if (PTR_ERR(ctx->tfm) == -ENOENT)
			pr_err("The crypto transform ecb(aes) seems to be missing."
			       " Please check your kernel configuration.\n");
		goto error;
	}

	return ctx;

error:
	mcps_crypto_aes_ecb_128_destroy(ctx);

	return NULL;
}

int mcps_crypto_aes_ecb_128_set_encrypt(struct mcps_aes_ecb_128_ctx *ctx,
		const uint8_t *key)
{
	if (!ctx || !key)
		return -EINVAL;

	ctx->decrypt = false;

	return crypto_skcipher_setkey(ctx->tfm, key, AES_KEYSIZE_128);
}

int mcps_crypto_aes_ecb_128_set_decrypt(struct mcps_aes_ecb_128_ctx *ctx,
		const uint8_t *key)
{
	if (!ctx || !key)
		return -EINVAL;

	ctx->decrypt = true;

	return crypto_skcipher_setkey(ctx->tfm, key, AES_KEYSIZE_128);
}

void mcps_crypto_aes_ecb_128_destroy(struct mcps_aes_ecb_128_ctx *ctx)
{
	if (!ctx)
		return;

	crypto_free_skcipher(ctx->tfm);
	kfree(ctx);
}

int mcps_crypto_aes_ecb_128_encrypt(struct mcps_aes_ecb_128_ctx *ctx,
		const uint8_t *data, unsigned int data_len, uint8_t *out)
{
	struct skcipher_request *req = NULL;
	struct scatterlist sgin, sgout;
	DECLARE_CRYPTO_WAIT(wait);
	int r = -1;

	if (!ctx || !data || data_len <= 0 || !out)
		return -EINVAL;

	/* round to full cipher block */
	data_len = ((data_len - 1) & -AES_KEYSIZE_128) + AES_KEYSIZE_128;

	req = skcipher_request_alloc(ctx->tfm, GFP_KERNEL);
	if (!req) {
		r = -ENOMEM;
		goto end;
	}

	sg_init_one(&sgin, data, data_len);
	sg_init_one(&sgout, out, data_len);
	skcipher_request_set_callback(req,
			CRYPTO_TFM_REQ_MAY_BACKLOG | CRYPTO_TFM_REQ_MAY_SLEEP,
			crypto_req_done, &wait);
	skcipher_request_set_crypt(req, &sgin, &sgout, data_len, NULL);

	if (ctx->decrypt)
		r = crypto_skcipher_decrypt(req);
	else
		r = crypto_skcipher_encrypt(req);
	r = crypto_wait_req(r, &wait);

end:
	skcipher_request_free(req);

	return r;
}

