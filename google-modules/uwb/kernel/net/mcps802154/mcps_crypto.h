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

#ifndef MCPS_CRYPTO_H
#define MCPS_CRYPTO_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#define MCPS_CRYPTO_AES_CCM_STAR_NONCE_LEN 13

/**
 * struct mcps_aes_ccm_star_128_ctx - Context containing AES-128-CCM* related
 * information.
 *
 * This is an opaque structure left to the implementation.
 */
struct mcps_aes_ccm_star_128_ctx;

/**
 * struct mcps_aes_ecb_128_ctx - Context containing AES-128-ECB related
 * information.
 *
 * This is an opaque structure left to the implementation.
 */
struct mcps_aes_ecb_128_ctx;


/**
 * mcps_crypto_cmac_aes_128_digest() - Compute a cmac AES 128.
 * @key: AES key.
 * @data: Input data.
 * @data_len: Input data length in bytes.
 * @out: Output hash, with length AES_BLOCK_SIZE.
 *
 * NOTE: This API should be implemented by platform.
 *
 * Return: 0 or error.
 */
int mcps_crypto_cmac_aes_128_digest(const uint8_t *key, const uint8_t *data,
		unsigned int data_len, uint8_t *out);

/**
 * mcps_crypto_aead_aes_ccm_star_128_create() - Create a context using
 * Authenticated Encryption Associated Data with AES CCM* 128.
 *
 * NOTE: This API should be implemented by platform.
 *
 * Return: The pointer to the context that will be used to encrypt & decrypt.
 */
struct mcps_aes_ccm_star_128_ctx *mcps_crypto_aead_aes_ccm_star_128_create(void);

/**
 * mcps_crypto_aead_aes_ccm_star_128_set() - Set a context using
 * Authenticated Encryption Associated Data with AES CCM* 128.
 * @ctx: Context.
 * @key: AES key.
 *
 * NOTE: This API should be implemented by platform.
 *
 * Return: 0 or error.
 */
int mcps_crypto_aead_aes_ccm_star_128_set(struct mcps_aes_ccm_star_128_ctx *ctx, const uint8_t *key);

/**
 * mcps_crypto_aead_aes_ccm_star_128_destroy() - Destroy the Authenticated
 * Encryption Associated Data with AES CCM* 128 context.
 * @ctx: Context.
 *
 * NOTE: This API should be implemented by platform.
 */
void mcps_crypto_aead_aes_ccm_star_128_destroy(struct mcps_aes_ccm_star_128_ctx *ctx);

/**
 * mcps_crypto_aead_aes_ccm_star_128_encrypt() - Encrypt using Authenticated
 * Encryption Associated Data with AES CCM* 128.
 * @ctx: Context.
 * @nonce: Nonce, with length MCPS_CRYPTO_AES_CCM_STAR_NONCE_LEN.
 * @header: Header data.
 * @header_len: Header length in bytes.
 * @data: Data to encrypt, will be replaced with encrypted data.
 * @data_len: Data length in bytes.
 * @mac: AES CCM* MAC.
 * @mac_len: AES CCM* MAC size in bytes.
 *
 * NOTE: This API should be implemented by platform.
 *
 * Return: 0 or error.
 */
int mcps_crypto_aead_aes_ccm_star_128_encrypt(
		struct mcps_aes_ccm_star_128_ctx *ctx, const uint8_t *nonce,
		const uint8_t *header, unsigned int header_len,
		uint8_t *data, unsigned int data_len,
		uint8_t *mac, unsigned int mac_len);

/**
 * mcps_crypto_aead_aes_ccm_star_128_decrypt() - Decrypt using Authenticated
 * Encryption Associated Data with AES CCM* 128.
 * @ctx: Context.
 * @nonce: Nonce, with length MCPS_CRYPTO_AES_CCM_STAR_NONCE_LEN.
 * @header: Header data.
 * @header_len: Header length in bytes.
 * @data: Data to decrypt, will be replaced with decrypted data.
 * @data_len: Data length in bytes.
 * @mac: AES CCM* MAC.
 * @mac_len: AES CCM* MAC size in bytes.
 *
 * NOTE: This API should be implemented by platform. In case of mismatch
 * between the MAC and calculated MAC, this function should return -EBADMSG.
 *
 * Return: 0 or error.
 */
int mcps_crypto_aead_aes_ccm_star_128_decrypt(
		struct mcps_aes_ccm_star_128_ctx *ctx, const uint8_t *nonce,
		const uint8_t *header, unsigned int header_len,
		uint8_t *data, unsigned int data_len,
		uint8_t *mac, unsigned int mac_len);

/**
 * mcps_crypto_aes_ecb_128_create() - Create a context using AES ECB 128.
 *
 * NOTE: This API should be implemented by platform.
 *
 * Return: The pointer to the context that will be used to encrypt & decrypt.
 */
struct mcps_aes_ecb_128_ctx *mcps_crypto_aes_ecb_128_create(void);

/**
 * mcps_crypto_aes_ecb_128_set_encrypt() - Set a context using
 * Authenticated Encryption Associated Data with AES ECB* 128.
 * @ctx: Context.
 * @key: AES key.
 *
 * NOTE: This API should be implemented by platform.
 *
 * Return: 0 or error.
 */
int mcps_crypto_aes_ecb_128_set_encrypt(struct mcps_aes_ecb_128_ctx *ctx, const uint8_t *key);

/**
 * mcps_crypto_aes_ecb_128_set_decrypt() - Set a context using
 * Authenticated Encryption Associated Data with AES ECB* 128.
 * @ctx: Context.
 * @key: AES key.
 *
 * NOTE: This API should be implemented by platform.
 *
 * Return: 0 or error.
 */
int mcps_crypto_aes_ecb_128_set_decrypt(struct mcps_aes_ecb_128_ctx *ctx, const uint8_t *key);

/**
 * mcps_crypto_aes_ecb_128_destroy() - Destroy the AES ECB 128 context.
 * @ctx: Context.
 *
 * NOTE: This API should be implemented by platform.
 */
void mcps_crypto_aes_ecb_128_destroy(struct mcps_aes_ecb_128_ctx *ctx);

/**
 * mcps_crypto_aes_ecb_128_encrypt() - Encrypt using AES ECB 128.
 * @ctx: Context.
 * @data: Data to encrypt.
 * @data_len: Data length in bytes, should be a multiple of AES_BLOCK_SIZE.
 * @out: Ciphered data with same length as data.
 *
 * NOTE: This API should be implemented by platform.
 *
 * Return: 0 or error.
 */
int mcps_crypto_aes_ecb_128_encrypt(struct mcps_aes_ecb_128_ctx *ctx,
		const uint8_t *data, unsigned int data_len, uint8_t *out);

#endif /* MCPS_CRYPTO_H */
