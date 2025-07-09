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

#ifndef FIRA_ROUND_HOPPING_CRYPTO_H
#define FIRA_ROUND_HOPPING_CRYPTO_H

#include <linux/types.h>
#include <crypto/aes.h>

struct fira_round_hopping_sequence;

/**
 * fira_round_hopping_crypto_encrypt() - Compute a cipher using AES.
 * @round_hopping_sequence: Round hopping context.
 * @data: Input data, with length AES_BLOCK_SIZE.
 * @out: Output hash, with length AES_BLOCK_SIZE.
 *
 * Return: 0 or error.
 */
int fira_round_hopping_crypto_encrypt(
	const struct fira_round_hopping_sequence *round_hopping_sequence,
	const u8 *data, u8 *out);

/**
 * fira_round_hopping_crypto_init() - Initialize round hopping context.
 * @round_hopping_sequence: Round hopping context.
 *
 * Return: 0 or error.
 */
int fira_round_hopping_crypto_init(
	struct fira_round_hopping_sequence *round_hopping_sequence);

/**
 * fira_round_hopping_crypto_destroy() - Destroy round hopping context.
 * @round_hopping_sequence: Round hopping context.
 */
void fira_round_hopping_crypto_destroy(
	struct fira_round_hopping_sequence *round_hopping_sequence);

#endif /* FIRA_ROUND_HOPPING_CRYPTO_H */
