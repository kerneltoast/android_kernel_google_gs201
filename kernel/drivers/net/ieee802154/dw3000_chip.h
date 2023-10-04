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
#ifndef __DW3000_CHIP_H
#define __DW3000_CHIP_H

/* Forward declaration */
struct dw3000;
struct dw3000_rssi;

/**
 * enum dw3000_chip_register_flags - flags for the register declaration
 * @DW3000_CHIPREG_NONE: no special flag defined
 * @DW3000_CHIPREG_DUMP: register is dump only, address is fileid number only
 * @DW3000_CHIPREG_RO: register is always read-only
 * @DW3000_CHIPREG_WP: register is write-protected. write refused if device is
 *  already active.
 * @DW3000_CHIPREG_PERM: register have a permanent access: chip off or on.
 *  This will be usefull for some virtual registers wired on callbacks
 * @DW3000_CHIPREG_OPENONCE: allow only one file instance at a time
 */
enum dw3000_chip_register_flags {
	DW3000_CHIPREG_NONE = 0,
	DW3000_CHIPREG_DUMP = 1,
	DW3000_CHIPREG_RO = 2,
	DW3000_CHIPREG_WP = 4,
	DW3000_CHIPREG_PERM = 8,
	DW3000_CHIPREG_OPENONCE = 16,
};

/**
 * typedef dw3000_chip_register_cb - virtual register callback function
 * @filp: the debugfs file structure pointer
 * @write: true when called for a write operation
 * @buffer: input or output buffer, depending on write parameter
 * @size: size of input buffer or available space of output buffer
 * @ppos: the callback handle the ppos to be blocking or not
 *
 * All functions of this type will parse input buffer and do the relevant
 * action according given parameters when write is true.
 * When write is false, relevant information is written to buffer.
 *
 * The filp parameter is used by the function to retrieve required private
 * data given when the file was created. See struct dw3000_chip_register_priv.
 *
 * Returns: length read from buffer or written to buffer or negative error
 */
typedef int (*dw3000_chip_register_cb)(struct file *filp, bool write,
				       void *buffer, size_t size, loff_t *ppos);

/**
 * struct dw3000_chip_register - version dependent register declaration
 * @name: register name
 * @address: register address
 * @size: register size (in bits if mask defined)
 * @mask: register mask (unused if 0)
 * @flags: or'ed value of enum dw3000_chip_register_flags
 * @callback: processing callback for a virtual register
 */
struct dw3000_chip_register {
	const char *const name;
	unsigned address;
	size_t size;
	unsigned mask;
	unsigned flags;
	dw3000_chip_register_cb callback;
};

/**
 * struct dw3000_chip_register_priv - private data for debugfs file
 * @dw: backpointer to DW3000 device instance
 * @reg: pointer to first struct dw3000_chip_register this file belong to
 * @count: number of struct dw3000_chip_register this file handle
 *
 * If a specific debugfs file need more data, it can derive this structure.
 */
struct dw3000_chip_register_priv {
	struct dw3000 *dw;
	const struct dw3000_chip_register *reg;
	size_t count;
};

/**
 * struct dw3000_chip_ops - version dependent chip operations
 * @softreset: soft-reset
 * @init: initialisation
 * @coex_init: initialise WiFi coexistence GPIO
 * @coex_gpio: change state of WiFi coexistence GPIO
 * @check_tx_ok: Check device has correctly entered tx state
 * @prog_ldo_and_bias_tune: programs the device's LDO and BIAS tuning
 * @get_config_mrxlut_chan: Lookup table default values for channel provided or NULL
 * @get_dgc_dec: Read DGC_DBG register
 * @pre_read_sys_time: Workaround before the SYS_TIME register reads
 * @adc_offset_calibration: Workaround to calibrate ADC offset
 * @pll_calibration_from_scratch: Workaround to calibrate the PLL from scratch
 * @pll_coarse_code: Workaround to set PLL coarse code
 * @prog_pll_coarse_code: Program PLL coarse code from OTP
 * @set_mrxlut: configure mrxlut
 * @kick_ops_table_on_wakeup: kick the desired operating parameter set table
 * @kick_dgc_on_wakeup: kick the DGC upon wakeup from sleep
 * @get_registers: Return known registers table and it's size
 * @compute_rssi: Uses the parameters to compute RSSI of current frame
 */
struct dw3000_chip_ops {
	int (*softreset)(struct dw3000 *dw);
	int (*init)(struct dw3000 *dw);
	int (*coex_init)(struct dw3000 *dw);
	int (*coex_gpio)(struct dw3000 *dw, bool state, int delay_us);
	int (*check_tx_ok)(struct dw3000 *dw);
	int (*prog_ldo_and_bias_tune)(struct dw3000 *dw);
	const u32 *(*get_config_mrxlut_chan)(struct dw3000 *dw, u8 channel);
	int (*get_dgc_dec)(struct dw3000 *dw, u8 *value);
	int (*pre_read_sys_time)(struct dw3000 *dw);
	int (*adc_offset_calibration)(struct dw3000 *dw);
	int (*pll_calibration_from_scratch)(struct dw3000 *dw);
	int (*pll_coarse_code)(struct dw3000 *dw);
	int (*prog_pll_coarse_code)(struct dw3000 *dw);
	int (*set_mrxlut)(struct dw3000 *dw, const u32 *lut);
	int (*kick_ops_table_on_wakeup)(struct dw3000 *dw);
	int (*kick_dgc_on_wakeup)(struct dw3000 *dw);
	const struct dw3000_chip_register *(*get_registers)(struct dw3000 *dw,
							    size_t *count);
	u32 (*compute_rssi)(struct dw3000 *dw, struct dw3000_rssi *rssi,
			    bool rx_tune, u8 sts);
};

/**
 * struct dw3000_chip_version - supported chip version definition
 * @id: device model ID
 * @ver: device registers version, saved to __dw3000_chip_version
 * @ops: associated version specific operations
 * @name: short version name of current device
 */
struct dw3000_chip_version {
	unsigned id;
	int ver;
	const struct dw3000_chip_ops *ops;
	const char *name;
};

/* DW3000 device model IDs (with or non PDOA) */
#define DW3000_C0_DEV_ID 0xdeca0302
#define DW3000_C0_PDOA_DEV_ID 0xdeca0312
#define DW3000_C0_VERSION 0
#define DW3000_D0_DEV_ID 0xdeca0303
#define DW3000_D0_PDOA_DEV_ID 0xdeca0313
#define DW3000_D0_VERSION 1
#define DW3000_E0_PDOA_DEV_ID 0xdeca0314
#define DW3000_E0_VERSION 2

/* Declaration of version specific chip operations */
extern const struct dw3000_chip_ops dw3000_chip_c0_ops;
extern const struct dw3000_chip_ops dw3000_chip_d0_ops;
extern const struct dw3000_chip_ops dw3000_chip_e0_ops;

#endif /* __DW3000_CHIP_H */
