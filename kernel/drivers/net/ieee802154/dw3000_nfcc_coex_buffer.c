/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2021 Qorvo US, Inc.
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
#include "dw3000_nfcc_coex_buffer.h"
#include "dw3000_core.h"
#include "dw3000_core_reg.h"
#include "dw3000_trc.h"

/**
 * dw3000_nfcc_coex_read_scratch_ram() - Copy from scratch ram memory to buffer.
 * @dw: Driver context.
 * @buffer: Buffer to write.
 * @len: Number of byte to copy.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_read_scratch_ram(
	struct dw3000 *dw, struct dw3000_nfcc_coex_buffer *buffer, u16 len)
{
	static const int offset = DW3000_NFCC_COEX_MSG_IN_OFFSET;
	int end_offset = len + offset;

	if (end_offset > DW3000_SCRATCH_RAM_LEN) {
		trace_dw3000_nfcc_coex_err(dw, "read scratch ram bad address");
		return -EINVAL;
	}
	return dw3000_xfer(dw, DW3000_SCRATCH_RAM_ID, offset, len, buffer->raw,
			   DW3000_SPI_RD_BIT);
}

/**
 * dw3000_nfcc_coex_write_scratch_ram() - Copy from buffer to scratch ram memory.
 * @dw: Driver context.
 * @buffer: Buffer to read.
 * @len: Number of byte to copy.
 *
 * Return: 0 on success, else an error.
 */
static int
dw3000_nfcc_coex_write_scratch_ram(struct dw3000 *dw,
				   const struct dw3000_nfcc_coex_buffer *buffer,
				   u16 len)
{
	static const int offset = DW3000_NFCC_COEX_MSG_OUT_OFFSET;
	int end_offset = len + offset;
	/* TODO: Avoid the cast, which remove the const with better code.
	 * Idea to dig: define dw3000_xfer with 2 pointers(read/write).
	 * And develop the idea of full duplex API. */
	void *raw = (void *)buffer->raw;

	if (end_offset > DW3000_SCRATCH_RAM_LEN) {
		trace_dw3000_nfcc_coex_err(dw, "write scratch ram bad address");
		return -EINVAL;
	}
	return dw3000_xfer(dw, DW3000_SCRATCH_RAM_ID, offset, len, raw,
			   DW3000_SPI_WR_BIT);
}

/**
 * dw3000_nfcc_coex_is_spi1_reserved() - Get the status of SPI1 reserved status.
 * @dw: Driver context.
 * @val: Boolean updated on success.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_is_spi1_reserved(struct dw3000 *dw, bool *val)
{
	u8 reg;
	int rc;

	/* Check if SPI1 is reserved by reading SPI_SEM register. */
	rc = dw3000_reg_read8(dw, DW3000_SPI_SEM_ID, 0, &reg);
	if (rc)
		return rc;

	*val = reg & DW3000_SPI_SEM_SPI1_RG_BIT_MASK;
	return 0;
}

/**
 * dw3000_nfcc_coex_release_spi1() - Release the SPI1.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_release_spi1(struct dw3000 *dw)
{
	int rc;
	bool is_spi1_enabled;

	rc = dw3000_write_fastcmd(dw, DW3000_CMD_SEMA_REL);
	if (rc)
		return rc;
	rc = dw3000_nfcc_coex_is_spi1_reserved(dw, &is_spi1_enabled);
	if (rc)
		return rc;

	return is_spi1_enabled ? -EBUSY : 0;
}

/**
 * dw3000_nfcc_coex_reserve_spi1() - Reserve the SPI1.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
static int dw3000_nfcc_coex_reserve_spi1(struct dw3000 *dw)
{
	int rc;
	bool is_spi1_reserved;

	rc = dw3000_write_fastcmd(dw, DW3000_CMD_SEMA_REQ);
	if (rc)
		return rc;
	/* Check if the SPI1 is really reserved.
	 * Indeed, if SPI2 is already reserved, SPI1 could not be reserved. */
	rc = dw3000_nfcc_coex_is_spi1_reserved(dw, &is_spi1_reserved);
	if (rc)
		return rc;

	return is_spi1_reserved ? 0 : -EBUSY;
}

/**
 * dw3000_nfcc_coex_write_buffer() - Write buffer into scratch memory.
 * @dw: Driver context.
 * @buffer: buffer to write in scratch memory.
 * @len: byte length of the buffer.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_write_buffer(struct dw3000 *dw,
				  const struct dw3000_nfcc_coex_buffer *buffer,
				  u16 len)
{
	int rc;

	if (!dw->nfcc_coex.enabled)
		return -EOPNOTSUPP;
	if (len > DW3000_NFCC_COEX_MSG_OUT_SIZE) {
		trace_dw3000_nfcc_coex_err(
			dw, "writing to nfcc exceed SCRATCH_AP_SIZE");
		return -EINVAL;
	}
	rc = dw3000_nfcc_coex_reserve_spi1(dw);
	if (rc)
		return rc;
	rc = dw3000_nfcc_coex_write_scratch_ram(dw, buffer, len);
	if (rc)
		return rc;
	dw->nfcc_coex.tx_seq_num++;
	/* Trigger IRQ2 to inform NFCC. */
	return dw3000_nfcc_coex_release_spi1(dw);
}

/**
 * dw3000_nfcc_coex_read_buffer() - Read buffer from scratch memory.
 * @dw: Driver context.
 * @buffer: buffer fill with content of the scratch memory.
 * @len: byte length of the buffer.
 *
 * Return: 0 on success, else an error.
 */
int dw3000_nfcc_coex_read_buffer(struct dw3000 *dw,
				 struct dw3000_nfcc_coex_buffer *buffer,
				 u16 len)
{
	int rc;

	if (!dw->nfcc_coex.enabled)
		return -EOPNOTSUPP;
	if (len > DW3000_NFCC_COEX_MSG_IN_SIZE) {
		trace_dw3000_nfcc_coex_err(
			dw, "Reading from NFCC exceed SCRATCH_AP_SIZE");
		return -EINVAL;
	}
	rc = dw3000_nfcc_coex_read_scratch_ram(dw, buffer, len);
	if (rc)
		trace_dw3000_nfcc_coex_err(
			dw, "Error while reading NFCC scratch RAM");
	return rc;
}
