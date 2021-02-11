// SPDX-License-Identifier: GPL-2.0 only
/*
 *  Emerald Hill compression engine driver
 *
 *  Copyright (C) 2020 Google LLC
 *  Author: Petri Gynther <pgynther@google.com>
 *
 *  Derived from:
 *  Hardware Compressed RAM offload driver
 *  Copyright (C) 2015 The Chromium OS Authors
 *  Sonny Rao <sonnyrao@chromium.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _EH_REGS_H_
#define _EH_REGS_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <inttypes.h>
#endif

/* Hardware related definitions */

/* fixed 4096 byte block size */
#define EH_UNCOMPRESSED_BLOCK_SIZE_LOG2 12
#define EH_UNCOMPRESSED_BLOCK_SIZE (1 << EH_UNCOMPRESSED_BLOCK_SIZE_LOG2)

/* possible sizes are 64, 128, 256, 512, 1024, 2048, 4096 */
#define EH_NUM_OF_POSSIBLE_SIZES 7

/* useful constant for converting sizes to indices */
#define EH_MIN_SIZE_SHIFT 6
#define EH_MIN_BLOCK_SIZE (1 << EH_MIN_SIZE_SHIFT)

/* 6 blocks are supplied in the command */
#define EH_NUM_OF_FREE_BLOCKS 6

/* Max compression fifo size */
#define EH_MAX_FIFO_SIZE 32768

/* Max number of buffers used */
#define EH_MAX_BUFFERS_USED 4

/* These are the possible values for the status field from the specification */
enum eh_cdesc_status {
	/* descriptor not in use */
	EH_CDESC_IDLE = 0x0,

	/* descriptor completed with compressed bytes written to target */
	EH_CDESC_COMPRESSED = 0x1,

	/*
	 * descriptor completed, incompressible page, uncompressed bytes written
	 * to target
	 */
	EH_CDESC_COPIED = 0x2,

	/* descriptor completed, incompressible page, nothing written to target
	 */
	EH_CDESC_ABORT = 0x3,

	/* descriptor completed, page was all zero, nothing written to target */
	EH_CDESC_ZERO = 0x4,

	/*
	 * descriptor count not be completed dut to an error.
	 * queue operation continued to next descriptor
	 */
	EH_CDESC_ERROR_CONTINUE = 0x5,

	/*
	 * descriptor count not be completed dut to an error.
	 * queue operation halted
	 */
	EH_CDESC_ERROR_HALTED = 0x6,

	/* descriptor in queue or being processed by hardware */
	EH_CDESC_PENDING = 0x7,
};

/* a type 0 compression descriptor */
struct eh_compr_desc_0 {
	/* word 0 */
	/* 6 bits indicating which of the 6 buffers were used */
	unsigned int buf_sel : 6;

	/* 3 bits for selecting the hash algorithm */
	unsigned int hash_algo : 3;

	/* 3 bits for selecting the encryption algorithm */
	unsigned int enc_algo : 3;

	/* 4 bits reserved for future use*/
	unsigned int reserved1 : 4;

	/* 16 bits for compressed length*/
	uint16_t compr_len;

	/* 32 bits for the hash */
	uint32_t hash;

	/* word 1, status and interrupt request are in the lower 12 bits */
	union {
		struct {
			/* 1 bit for interrupt request */
			unsigned int intr_request : 1;

			/* 3 bits for status */
			unsigned int status : 3;

			/* 4 bits to select compression algorithm */
			unsigned int compr_algo : 4;

			/* 2 bits to select number of buffers to use */
			unsigned int max_buf : 2;

			/* pad out the rest of the word */
			uint64_t padding : 54;
		} s1;
#define EH_COMPR_DESC_0_SRC_MASK ~(0xFULL)
		uint64_t src_addr;
	} u1;

	/* words 2 - 5, destination buffers */
	uint64_t dst_addr[EH_NUM_OF_FREE_BLOCKS];
};

#define EH_COMPR_DESC_0_SIZE sizeof(struct eh_compr_desc_0)

/*
 * We're using a scheme for encoding the size of an aligned buffer
 * by setting the bit equal to size/2 which is smaller than the least
 * significant bit of the address.  So, let's have a few macros to
 * convert to and from the encoded addresses.
 */

/* Convert from an encoded address to a size */
#define EH_ENCODED_ADDR_TO_SIZE(addr) (((addr) & ~((addr)-1)) << 1)

/* Convert from an encoded address to a physical address */
#define EH_ENCODED_ADDR_TO_PHYS(addr) ((addr) & ((addr)-1))

/* Convert from a physical address and size to an encoded address */
#define EH_PHYS_ADDR_TO_ENCODED(addr, size) ((addr) | (1ull << (ffs(size) - 2)))

/* register definitions */

#define EH_REGS_SIZE 0x1000

#define EH_REG_HWID             0x000
#define EH_REG_HWFEATURES       0x008
#define EH_REG_HWFEATURES2      0x010
#define EH_REG_GCTRL            0x030
#define EH_REG_INTRP_STS_ERROR  0x040
#define EH_REG_INTRP_MASK_ERROR 0x048
#define EH_REG_INTRP_STS_CMP    0x050
#define EH_REG_INTRP_MASK_CMP   0x058
#define EH_REG_INTRP_STS_DCMP   0x060
#define EH_REG_INTRP_MASK_DCMP  0x068
#define EH_REG_ERR_COND         0x080
#define EH_REG_ERR_ADDR         0x088
#define EH_REG_ERR_CMD          0x090
#define EH_REG_ERR_CNT          0x098
#define EH_REG_ERR_MSK          0x0A0
#define EH_REG_PMON_0           0x0C0
#define EH_REG_PMON_1           0x0C8
#define EH_REG_PMON_2           0x0D0
#define EH_REG_PMON_0_CFG       0x0E0
#define EH_REG_PMON_1_CFG       0x0E8
#define EH_REG_PMON_2_CFG       0x0F0
#define EH_REG_BUSCFG           0x100
#define EH_REG_BUSCFG2          0x108
#define EH_REG_PROT             0x110
#define EH_REG_INTCFG           0x118
#define EH_REG_BUSCFG3          0x120
#define EH_REG_ERRINJ           0x170
#define EH_REG_DBGCTRL          0x200
#define EH_REG_DBGSEL           0x208
#define EH_REG_DBGBUF           0x210
#define EH_REG_DBGSTMBUF        0x218
#define EH_REG_DBGMEMADDR       0x220
#define EH_REG_DBGMASK          0x228
#define EH_REG_DBGMATCH         0x230
#define EH_REG_DBGTRIGMASK      0x238
#define EH_REG_DBGTRIGMATCH     0x240

#define SHIFT_AND_MASK(val, shift, mask)                                       \
	(((val) >> (uint64_t)shift) & (uint64_t)(mask))

/* meanings of some of the register bits for global */
#define EH_HWID_VENDOR_SHIFT             0ULL
#define EH_HWID_DEVICE_SHIFT             16ULL
#define EH_FEATURES2_RESULT_WRITE_SHIFT  23ULL
#define EH_FEATURES2_WIDE_REG_SHIFT      22ULL
#define EH_FEATURES2_DESC_FIXED_SHIFT    21ULL
#define EH_FEATURES2_BUF_MAX_SHIFT       16ULL
#define EH_FEATURES2_BUF_MAX_MASK        0x7ULL
#define EH_FEATURES2_DECOMPR_CMDS_SHIFT  8ULL
#define EH_FEATURES2_DECOMPR_CMDS_MASK   0xFFULL
#define EH_FEATURES2_DESC_TYPE_SHIFT     5ULL
#define EH_FEATURES2_DESC_TYPE_MASK      0x7ULL
#define EH_GCTRL_RESET_SHIFT             0ULL

#define EH_ERR_COND_ERROR_OVERFLOW_SHIFT 36ULL
#define EH_ERR_COND_ERROR_VEC_SHIFT      8ULL
#define EH_ERR_COND_ERR_CNT_OV_SHIFT     7ULL
#define EH_ERR_COND_Q_HALT_SHIFT         6ULL
#define EH_ERR_COND_D_CMD_ERR_SHIFT      5ULL
#define EH_ERR_COND_C_CMD_ERR_SHIFT      4ULL
#define EH_ERR_COND_COR_SHIFT            3ULL
#define EH_ERR_COND_UNC_SHIFT            2ULL
#define EH_ERR_COND_DEV_FATAL_SHIFT      1ULL
#define EH_ERR_COND_SYS_FATAL_SHIFT      0ULL
#define EH_ERR_ADDR_LOG_ADDR_MASK        0xFFFFFFFFC0ULL
#define EH_ERR_ADDR_LOG_ADDR_VLD_SHIFT   1ULL
#define EH_ERR_ADDR_LOG_BUSY_SHIFT       0ULL
#define EH_ERR_CMD_LOG_BUS_ID_MASK       0xFFULL
#define EH_ERR_CMD_LOG_BUS_ID_SHIFT      40ULL
#define EH_ERR_CMD_LOG_CMD_MASK          0xFFULL
#define EH_ERR_CMD_LOG_CMD_SHIFT         32ULL
#define EH_ERR_CMD_LOG_INDEX_MASK        0xFFFFULL
#define EH_ERR_CMD_LOG_INDEX_SHIFT       16ULL
#define EH_ERR_CMD_LOG_BARRIER_SHIFT     9ULL
#define EH_ERR_CMD_LOG_BUS_ID_VLD_SHIFT  8ULL
#define EH_ERR_CMD_LOG_INDEX_VLD_SHIFT   7ULL
#define EH_ERR_CMD_LOG_CMD_VLD_SHIFT     6ULL
#define EH_ERR_CMD_LOG_PAYLOAD_SHIFT     5ULL
#define EH_ERR_CMD_LOG_DESCR_SHIFT       4ULL
#define EH_ERR_CMD_LOG_WRITE_SHIFT       3ULL
#define EH_ERR_CMD_LOG_READ_SHIFT        2ULL
#define EH_ERR_CMD_LOG_COMPRESS_SHIFT    1ULL
#define EH_ERR_CMD_LOG_DECOMPRESS_SHIFT  0ULL

// bits of error vec and error overflow
// in google implementation
#define EH_ERR_DBG_BIT       9ULL
#define EH_ERR_DSIZE_BIT     8ULL
#define EH_ERR_DENGINE_BIT   7ULL
#define EH_ERR_BAD_DESC_BIT  6ULL
#define EH_ERR_DEC_BIT       5ULL
#define EH_ERR_SLV_BIT       4ULL
#define EH_ERR_D_PIPE_TO_BIT 3ULL
#define EH_ERR_C_PIPE_TO_BIT 2ULL
#define EH_ERR_UR_BIT        1ULL
#define EH_ERR_BUS_TO_BIT    0ULL

#define EH_HWID_VENDOR(val)            SHIFT_AND_MASK(val, 16, 0xFFFF)
#define EH_HWID_DEVICE(val)            SHIFT_AND_MASK(val,  0, 0xFFFF)
#define EH_FEATURES2_RESULT_WR(val)    SHIFT_AND_MASK(val, 23, 0x1)
#define EH_FEATURES2_WIDE_REG(val)     SHIFT_AND_MASK(val, 22, 0x1)
#define EH_FEATURES2_DESC_FIXED(val)   SHIFT_AND_MASK(val, 21, 0x1)
#define EH_FEATURES2_WR_COHERENT(val)  SHIFT_AND_MASK(val, 20, 0x1)
#define EH_FEATURES2_RD_COHERENT(val)  SHIFT_AND_MASK(val, 19, 0x1)
#define EH_FEATURES2_BUF_MAX(val)      SHIFT_AND_MASK(val, 16, 0x7)
#define EH_FEATURES2_DECOMPR_CMDS(val) SHIFT_AND_MASK(val,  8, 0xFF)
#define EH_FEATURES2_DESC_TYPE(val)    SHIFT_AND_MASK(val,  5, 0x7)
#define EH_FEATURES2_FIFOS(val)        SHIFT_AND_MASK(val,  0, 0x1F)

#define EH_VENDOR_ID_GOOGLE 1

/* compression related */
#define EH_REG_CDESC_LOC     0x400
#define EH_REG_CDESC_WRIDX   0x408
#define EH_REG_CDESC_RDIDX   0x410
#define EH_REG_CDESC_CTRL    0x418
#define EH_REG_CINTERP_CTRL  0x420
#define EH_REG_CINTERP_TIMER 0x428

/* meanings of some of the bits for compression */
#define EH_CDESC_LOC_BASE_MASK              0xFFFFFFFFC0ULL
#define EH_CDESC_LOC_NUM_DESC_MASK          0xFULL
#define EH_CDESC_WRIDX_WRITE_IDX_MASK       0xFFFFULL
#define EH_CDESC_CTRL_FIFO_RESET            16ULL
#define EH_CDESC_CTRL_COMPRESS_BUSY_SHIFT   17ULL
#define EH_CDESC_CTRL_COMPRESS_ENABLE_SHIFT 18ULL
#define EH_CDESC_CTRL_COMPRESS_ERROR_SHIFT  19ULL
#define EH_CDESC_CTRL_COMPRESS_HALTED_SHIFT 20ULL
#define EH_CDESC_CTRL_COMPLETE_IDX_MASK     0xFFFFULL
#define EH_CINTERP_CTRL_IDX_MASK            0xFFFFULL
#define EH_CINTERP_CTRL_ENABLE_SHIFT        16ULL

/*
 * decompression related
 * we can have a number of register sets for decompression
 * each set fits within 0x40 bytes
 */

/* amount of space for each register set */
#define EH_DCMD_REGSET_SIZE 0x40

#define EH_DECOMPR_REGS 0x800

/* helpers to calculate offsets for decompression register set "n" */
#define EH_REG_DCMD_CSIZE(n) (0x800 + ((n) * EH_DCMD_REGSET_SIZE))
#define EH_REG_DCMD_DEST(n)  (0x808 + ((n) * EH_DCMD_REGSET_SIZE))
#define EH_REG_DCMD_RES(n)   (0x810 + ((n) * EH_DCMD_REGSET_SIZE))
#define EH_REG_DCMD_BUF0(n)  (0x818 + ((n) * EH_DCMD_REGSET_SIZE))
#define EH_REG_DCMD_BUF1(n)  (0x820 + ((n) * EH_DCMD_REGSET_SIZE))
#define EH_REG_DCMD_BUF2(n)  (0x828 + ((n) * EH_DCMD_REGSET_SIZE))
#define EH_REG_DCMD_BUF3(n)  (0x830 + ((n) * EH_DCMD_REGSET_SIZE))

/*
 * Decompression command status from the specification
 */
enum eh_dcmd_status {
	/* reset value */
	EH_DCMD_IDLE = 0x0,

	/* Decompressed bytes written to destination */
	EH_DCMD_DECOMPRESSED = 0x1,

	/* command completed with ERROR */
	EH_DCMD_ERROR = 0x4,

	/* buffer written buf hash didn't match decompressed bytes */
	EH_DCMD_HASH = 0x6,

	/* command waiting or being processed by hardware */
	EH_DCMD_PENDING = 0x7,
};

/* meanings of some of the bits for decompression */
#define EH_DCMD_DEST_BUF_MASK          ~((1ULL << 12ULL) - 1ULL)
#define EH_DCMD_DEST_STATUS_MASK       0xEULL
#define EH_DCMD_DEST_STATUS_SHIFT      1ULL
#define EH_DCMD_DEST_STATUS(val)       SHIFT_AND_MASK(val, 1, 0x7ULL)
#define EH_DCMD_DEST_INTR_MASK         0x1ULL
#define EH_DCMD_DEST_INTR_SHIFT        0x0ULL
#define EH_DCMD_CSIZE_HASH_SHIFT       32ULL
#define EH_DCMD_CSIZE_SIZE_SHIFT       16ULL
#define EH_DCMD_CSIZE_HASH_ENABLE_MASK 0x1ULL
#define EH_DCMD_BUF_SIZE_SHIFT         60ULL
#define EH_DCMD_RES_ADDR_MASK          0x000000FFFFFFFFF8ULL
#define EH_DCMD_RES_ENABLE_SHIFT       63ULL
#define EH_DCMD_RES_OVERWRITE_SHIFT    62ULL

#define EH_DCMD_DEST_TO_STATUS(d)                                              \
	(((d) & EH_DCMD_DEST_STATUS_MASK) >> EH_DCMD_DEST_STATUS_SHIFT)

/* a mask for the register set */
#define EH_DCMD_MASK 0x7C0

/* convert from a register offset to a register set */
#define EH_DCMD_REGSET(offset) (((offset) & EH_DCMD_MASK) >> 6)

/* (so far) everything above this offset is related to decompression */
#define EH_DECOMPRESSION_REG_BASE EH_DCMD_CSIZE(0)

#endif /* _EH_REGS_H_ */
