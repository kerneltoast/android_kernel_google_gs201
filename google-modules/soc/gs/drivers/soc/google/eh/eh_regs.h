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

/* a type 0 compression descriptor */
struct eh_compress_desc {
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
	unsigned short compr_len;

	/* 32 bits for the hash */
	unsigned int hash;

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
			unsigned long padding : 54;
		};
		unsigned long src_addr;
	};

	/* words 2 - 5, destination buffers */
	unsigned long dst_addr[EH_NUM_OF_FREE_BLOCKS];
};

#define EH_COMPRESS_DESC_SIZE sizeof(struct eh_compress_desc)

/* Address mask for compression descriptor */
#define EH_COMP_ADDR_LSB 0x0C
#define EH_COMP_SRC_ADDR_MASK ~(BIT(EH_COMP_ADDR_LSB)-1)

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

#define SHIFT_AND_MASK(val, shift, mask) \
	(((val) >> (unsigned long)(shift)) & (unsigned long)(mask))

/* meanings of some of the register bits for global */
#define EH_HWID_VENDOR_SHIFT             0UL
#define EH_HWID_DEVICE_SHIFT             16UL
#define EH_FEATURES2_RESULT_WRITE_SHIFT  23UL
#define EH_FEATURES2_WIDE_REG_SHIFT      22UL
#define EH_FEATURES2_DESC_FIXED_SHIFT    21UL
#define EH_FEATURES2_BUF_MAX_SHIFT       16UL
#define EH_FEATURES2_BUF_MAX_MASK        0x7UL
#define EH_FEATURES2_DECOMPR_CMDS_SHIFT  8UL
#define EH_FEATURES2_DECOMPR_CMDS_MASK   0xFFUL
#define EH_FEATURES2_DESC_TYPE_SHIFT     5UL
#define EH_FEATURES2_DESC_TYPE_MASK      0x7UL
#define EH_GCTRL_RESET_SHIFT             0UL

#define EH_ERR_COND_ERROR_OVERFLOW_SHIFT 36UL
#define EH_ERR_COND_ERROR_VEC_SHIFT      8UL
#define EH_ERR_COND_ERR_CNT_OV_SHIFT     7UL
#define EH_ERR_COND_Q_HALT_SHIFT         6UL
#define EH_ERR_COND_D_CMD_ERR_SHIFT      5UL
#define EH_ERR_COND_C_CMD_ERR_SHIFT      4UL
#define EH_ERR_COND_COR_SHIFT            3UL
#define EH_ERR_COND_UNC_SHIFT            2UL
#define EH_ERR_COND_DEV_FATAL_SHIFT      1UL
#define EH_ERR_COND_SYS_FATAL_SHIFT      0UL
#define EH_ERR_ADDR_LOG_ADDR_MASK        0xFFFFFFFFC0UL
#define EH_ERR_ADDR_LOG_ADDR_VLD_SHIFT   1UL
#define EH_ERR_ADDR_LOG_BUSY_SHIFT       0UL
#define EH_ERR_CMD_LOG_BUS_ID_MASK       0xFFUL
#define EH_ERR_CMD_LOG_BUS_ID_SHIFT      40UL
#define EH_ERR_CMD_LOG_CMD_MASK          0xFFUL
#define EH_ERR_CMD_LOG_CMD_SHIFT         32UL
#define EH_ERR_CMD_LOG_INDEX_MASK        0xFFFFUL
#define EH_ERR_CMD_LOG_INDEX_SHIFT       16UL
#define EH_ERR_CMD_LOG_BARRIER_SHIFT     9UL
#define EH_ERR_CMD_LOG_BUS_ID_VLD_SHIFT  8UL
#define EH_ERR_CMD_LOG_INDEX_VLD_SHIFT   7UL
#define EH_ERR_CMD_LOG_CMD_VLD_SHIFT     6UL
#define EH_ERR_CMD_LOG_PAYLOAD_SHIFT     5UL
#define EH_ERR_CMD_LOG_DESCR_SHIFT       4UL
#define EH_ERR_CMD_LOG_WRITE_SHIFT       3UL
#define EH_ERR_CMD_LOG_READ_SHIFT        2UL
#define EH_ERR_CMD_LOG_COMPRESS_SHIFT    1UL
#define EH_ERR_CMD_LOG_DECOMPRESS_SHIFT  0UL

// bits of error vec and error overflow
// in google implementation
#define EH_ERR_DBG_BIT       9UL
#define EH_ERR_DSIZE_BIT     8UL
#define EH_ERR_DENGINE_BIT   7UL
#define EH_ERR_BAD_DESC_BIT  6UL
#define EH_ERR_DEC_BIT       5UL
#define EH_ERR_SLV_BIT       4UL
#define EH_ERR_D_PIPE_TO_BIT 3UL
#define EH_ERR_C_PIPE_TO_BIT 2UL
#define EH_ERR_UR_BIT        1UL
#define EH_ERR_BUS_TO_BIT    0UL

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
#define EH_CDESC_LOC_BASE_MASK              0xFFFFFFFFC0UL
#define EH_CDESC_LOC_NUM_DESC_MASK          0xFUL
#define EH_CDESC_WRIDX_WRITE_IDX_MASK       0xFFFFUL
#define EH_CDESC_CTRL_COMPRESS_SIZE_LIMIT   21UL
#define EH_CDESC_CTRL_FIFO_RESET            16UL
#define EH_CDESC_CTRL_COMPRESS_BUSY_SHIFT   17UL
#define EH_CDESC_CTRL_COMPRESS_ENABLE_SHIFT 18UL
#define EH_CDESC_CTRL_COMPRESS_ERROR_SHIFT  19UL
#define EH_CDESC_CTRL_COMPRESS_HALTED_SHIFT 20UL
#define EH_CDESC_CTRL_COMPLETE_IDX_MASK     0xFFFFUL
#define EH_CINTERP_CTRL_IDX_MASK            0xFFFFUL
#define EH_CINTERP_CTRL_ENABLE_SHIFT        16UL

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
#define EH_DCMD_DEST_BUF_MASK          ~((1UL << 12UL) - 1UL)
#define EH_DCMD_DEST_STATUS_MASK       0xEUL
#define EH_DCMD_DEST_STATUS_SHIFT      1UL
#define EH_DCMD_DEST_STATUS(val)       SHIFT_AND_MASK(val, 1, 0x7UL)
#define EH_DCMD_DEST_INTR_MASK         0x1UL
#define EH_DCMD_DEST_INTR_SHIFT        0x0UL
#define EH_DCMD_CSIZE_HASH_SHIFT       32UL
#define EH_DCMD_CSIZE_SIZE_SHIFT       16UL
#define EH_DCMD_CSIZE_HASH_ENABLE_MASK 0x1UL
#define EH_DCMD_BUF_SIZE_SHIFT         60UL
#define EH_DCMD_RES_ADDR_MASK          0x000000FFFFFFFFF8UL
#define EH_DCMD_RES_ENABLE_SHIFT       63UL
#define EH_DCMD_RES_OVERWRITE_SHIFT    62UL

#define EH_DCMD_DEST_TO_STATUS(d)                                              \
	(((d) & EH_DCMD_DEST_STATUS_MASK) >> EH_DCMD_DEST_STATUS_SHIFT)

/* a mask for the register set */
#define EH_DCMD_MASK 0x7C0

/* convert from a register offset to a register set */
#define EH_DCMD_REGSET(offset) (((offset) & EH_DCMD_MASK) >> 6)

/* (so far) everything above this offset is related to decompression */
#define EH_DECOMPRESSION_REG_BASE EH_DCMD_CSIZE(0)

#endif /* _EH_REGS_H_ */
