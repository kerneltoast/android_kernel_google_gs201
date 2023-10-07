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

#include "fira_session.h"
#include <linux/string.h>
#include <asm/unaligned.h>

int fira_round_hopping_sequence_init(struct fira_session *session)
{
	struct fira_round_hopping_sequence *round_hopping_sequence =
		&session->round_hopping_sequence;

	memset(round_hopping_sequence->key, 0, AES_KEYSIZE_128 - sizeof(u32));
	put_unaligned_be32(session->id, round_hopping_sequence->key +
						AES_KEYSIZE_128 - sizeof(u32));
	return fira_round_hopping_crypto_init(round_hopping_sequence);
}

void fira_round_hopping_sequence_destroy(struct fira_session *session)
{
	struct fira_round_hopping_sequence *round_hopping_sequence =
		&session->round_hopping_sequence;

	fira_round_hopping_crypto_destroy(round_hopping_sequence);
}

int fira_round_hopping_sequence_get(const struct fira_session *session,
				    int block_index)
{
	const struct fira_session_params *params = &session->params;
	const struct fira_round_hopping_sequence *round_hopping_sequence =
		&session->round_hopping_sequence;
	int block_duration_slots =
		params->block_duration_dtu / params->slot_duration_dtu;
	int n_rounds = block_duration_slots / params->round_duration_slots;
	u8 block[AES_BLOCK_SIZE];
	u8 out[AES_BLOCK_SIZE];
	int r;

	if (!block_index)
		return 0;
	memset(block, 0, AES_BLOCK_SIZE - sizeof(u32));
	put_unaligned_be32(block_index, block + AES_BLOCK_SIZE - sizeof(u32));
	r = fira_round_hopping_crypto_encrypt(round_hopping_sequence, block,
					      out);
	if (!r) {
		u16 hash =
			get_unaligned_be16(out + AES_BLOCK_SIZE - sizeof(u16));

		return hash * n_rounds >> 16;
	}
	return 0;
}
