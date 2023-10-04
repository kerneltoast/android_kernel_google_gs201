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

#include "fira_round_hopping_crypto_impl.h"

#include <crypto/aes.h>
#include <linux/scatterlist.h>

int fira_round_hopping_crypto_encrypt(
	const struct fira_round_hopping_sequence *round_hopping_sequence,
	const u8 *data, u8 *out)
{
	struct scatterlist sg;
	SYNC_SKCIPHER_REQUEST_ON_STACK(req, round_hopping_sequence->tfm);
	int r;

	sg_init_one(&sg, round_hopping_sequence->data, AES_BLOCK_SIZE);
	memcpy(round_hopping_sequence->data, data, AES_BLOCK_SIZE);

	skcipher_request_set_sync_tfm(req, round_hopping_sequence->tfm);
	skcipher_request_set_callback(req, 0, NULL, NULL);
	skcipher_request_set_crypt(req, &sg, &sg, AES_BLOCK_SIZE, NULL);

	r = crypto_skcipher_encrypt(req);
	skcipher_request_zero(req);

	memcpy(out, round_hopping_sequence->data, AES_BLOCK_SIZE);

	return r;
}

int fira_round_hopping_crypto_init(
	struct fira_round_hopping_sequence *round_hopping_sequence)
{
	struct crypto_sync_skcipher *tfm;
	u8 *data;
	int r;

	data = kmalloc(AES_BLOCK_SIZE, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	tfm = crypto_alloc_sync_skcipher("ecb(aes)", 0, 0);
	if (IS_ERR(tfm)) {
		r = PTR_ERR(tfm);
		if (r == -ENOENT) {
			pr_err("The crypto transform ecb(aes) seems to be missing."
			       " Please check your kernel configuration.\n");
		}
		goto err_free_data;
	}

	r = crypto_sync_skcipher_setkey(tfm, round_hopping_sequence->key,
					AES_KEYSIZE_128);
	if (r)
		goto err_free_tfm;

	round_hopping_sequence->data = data;
	round_hopping_sequence->tfm = tfm;

	return r;
err_free_tfm:
	crypto_free_sync_skcipher(tfm);
err_free_data:
	kfree(data);
	return r;
}

void fira_round_hopping_crypto_destroy(
	struct fira_round_hopping_sequence *round_hopping_sequence)
{
	kfree(round_hopping_sequence->data);
	crypto_free_sync_skcipher(round_hopping_sequence->tfm);
}
