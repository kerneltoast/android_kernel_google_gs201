/* SPDX-License-Identifier: GPL-2.0
 *
 * Synaptics TouchCom touchscreen driver
 *
 * Copyright (C) 2017-2020 Synaptics Incorporated. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

/**
 * @file: syna_tcm2_platform.h
 *
 * This file declares the platform-specific or hardware relevant data.
 *
 * The main structure, syna_hw_interface, abstracts the bus transferred,
 * ATTN signal, RST_N pin, and power control operations.
 */

#ifndef _SYNAPTICS_TCM2_PLATFORM_H_
#define _SYNAPTICS_TCM2_PLATFORM_H_

#include "syna_tcm2_runtime.h"

/**
 * @section: The capability of bus transferred
 *
 * Declare read/write capability in bytes (0 = unlimited)
 */
#define RD_CHUNK_SIZE (512)
#define WR_CHUNK_SIZE (512)


/**
 * @section: Defined Hardware-Specific Data
 *
 * @brief: syna_hw_bus_data
 *         Hardware Data for bus transferred
 *
 * @brief: syna_hw_attn_data
 *         Hardware Data for ATTN signal
 *
 * @brief: syna_hw_rst_data
 *         Hardware Data for RST_N pin
 *
 * @brief: syna_hw_pwr_data
 *         Hardware Data for power control
 *
 * @brief: syna_hw_interface
 *         Contain all above hardware data and abstract the
 *         hardware operations
 */

/* The hardware data especially for bus transferred */
struct syna_hw_bus_data {
	unsigned char type;
	/* capability of i/o chunk */
	unsigned int rd_chunk_size;
	unsigned int wr_chunk_size;
	/* clock frequency in hz */
	unsigned int frequency_hz;
	/* parameters for i2c */
	unsigned int i2c_addr;
	/* parameters for spi */
	unsigned int spi_mode;
	unsigned int spi_byte_delay_us;
	unsigned int spi_block_delay_us;
	/* mutex to protect the i/o, if needed */
	syna_pal_mutex_t io_mutex;
};

/* The hardware data especially for ATTN signal */
struct syna_hw_attn_data {
	/* parameters */
	int irq_gpio;
	int irq_on_state;
	unsigned long irq_flags;
	int irq_id;
	bool irq_enabled;
	/* mutex to protect the irq control, if needed */
	syna_pal_mutex_t irq_en_mutex;
};

/* The hardware data especially for RST_N pin */
struct syna_hw_rst_data {
	/* parameters */
	int reset_gpio;
	int reset_on_state;
	unsigned int reset_delay_ms;
	unsigned int reset_active_ms;
};

/* The hardware data especially for power control */
struct syna_hw_pwr_data {
	/* parameters */
	int vdd_gpio;
	int avdd_gpio;
	int power_on_state;
	unsigned int power_on_delay_ms;
	/* voltage */
	unsigned int vdd;
	unsigned int vled;
	unsigned int vio;
	unsigned int vddtx;
	/* regulators */
	const char *vdd_reg_name;
	void *vdd_reg_dev;
	const char *avdd_reg_name;
	void *avdd_reg_dev;
};

/**
 * @section: Hardware Interface Abstraction Layer
 *
 * The structure contains the hardware-specific implementations
 * on the target platform.
 */
struct syna_hw_interface {
	/* The handle of hardware device */
	void *pdev;

	/* Hardware specific data */
	struct syna_hw_bus_data bdata_io;
	struct syna_hw_attn_data bdata_attn;
	struct syna_hw_rst_data bdata_rst;
	struct syna_hw_pwr_data bdata_pwr;
	const char *fw_name;
	int pixels_per_mm;
	u16 compression_threhsold;
	u16 grip_delta_threshold;
	u16 grip_border_threshold;
#if IS_ENABLED(CONFIG_TOUCHSCREEN_OFFLOAD)
	u32 offload_id;
#endif
	int udfps_x;
	int udfps_y;
	bool dynamic_report_rate;

	/* Operation to do power on/off, if supported
	 *
	 * This is an optional operation.
	 *
	 * Implementation should request that the power device be
	 * enabled with the output at the proper voltage.
	 *
	 * Assign the pointer NULL if power supply module is not controllable.
	 *
	 * @param
	 *    [ in] hw_if: the handle of hw interface
	 *    [ in] en:    '1' for powering on, and '0' for powering off
	 *
	 * @return
	 *    0 on success; otherwise, on error.
	 */
	int (*ops_power_on)(struct syna_hw_interface *hw_if,
			bool en);

	/* Operation to perform the hardware reset, if supported
	 *
	 * This is an optional operation.
	 *
	 * The actual reset waveform should be reference to the device spec.
	 *
	 * Assign the pointer NULL if RST_N pin is not controllable.
	 *
	 * @param
	 *    [ in] hw_if: the handle of hw interface
	 *
	 * @return
	 *    0 on success; otherwise, on error.
	 */
	void (*ops_hw_reset)(struct syna_hw_interface *hw_if);

	/* Operation to set up the bus connection
	 *
	 * This is an optional operation to add in the extra configuration
	 * before doing connection.
	 *
	 * Assign the pointer NULL if this operation is not required.
	 *
	 * @param
	 *    [ in] hw_if:   the handle of hw interface
	 *    [ in] config:  parameters to change
	 *
	 * @return
	 *    0 or positive value on success; otherwise, on error.
	 */
	int (*ops_bus_setup)(struct syna_hw_interface *hw_if,
			struct syna_hw_bus_data *config);

	/* Operation to read the bare data from bus
	 *
	 * This is an essential operation; otherwise, the communication
	 * will not be created.
	 *
	 * @param
	 *    [ in] hw_if:   the handle of hw interface
	 *    [out] rd_data: buffer for storing data retrieved
	 *    [ in] rd_len:  length of reading data in bytes
	 *
	 * @return
	 *    0 or positive value on success; otherwise, on error.
	 */
	int (*ops_read_data)(struct syna_hw_interface *hw_if,
			unsigned char *rd_data, unsigned int rd_len);

	/* Operation to write the bare data to bus
	 *
	 * This is an essential operation; otherwise, the communication
	 * will not be created.
	 *
	 * @param
	 *    [ in] hw_if:   the handle of hw interface
	 *    [ in] wr_data: written data
	 *    [ in] wr_len:  length of written data in bytes
	 *
	 * @return
	 *    0 or positive value on success; otherwise, on error.
	 */
	int (*ops_write_data)(struct syna_hw_interface *hw_if,
			unsigned char *wr_data, unsigned int wr_len);

	/* Operation to enable/disable the irq, if supported
	 *
	 * This is an optional operation. Providing this operation could
	 * minimize the frequency of ISR being called.
	 *
	 * Once disabled, ISR will not be invoked even ATTN is asserted.
	 *
	 * Assign the pointer NULL if irq is not controllable.
	 *
	 * @param
	 *    [ in] hw_if: the handle of hw interface
	 *    [ in] en:    '1' for enabling, and '0' for disabling
	 *
	 * @return
	 *    0 on success; otherwise, on error.
	 */
	int (*ops_enable_irq)(struct syna_hw_interface *hw_if,
			bool en);

	/* Operation to wait for the signal of interrupt, if supported
	 *
	 * This is an optional operation to help for creating the
	 * custom interrupt handler. Besides, the recommendation is to
	 * implement in one-shot approach if possible.
	 *
	 * Implementation should return control if ATTN is asserted or
	 * specified timeout expire. If timeout is 0, should check the
	 * state of the ATTN signal and return control immediately.
	 *
	 * Assign the pointer NULL if customized ISR is not required.
	 *
	 * @param
	 *    [ in] hw_if:      the handle of hw interface
	 *    [ in] timeout_ms: time frame in milliseconds
	 *
	 * @return
	 *    0 on success; otherwise, on error.
	 */
	int (*ops_wait_irq)(struct syna_hw_interface *hw_if,
			unsigned int timeout_ms);

};
/* end of structure syna_hw_interface */


/**
 * syna_hw_interface_init()
 *
 * Initialize the lower-level hardware interface module.
 * After returning, the handle of hw interface should be ready.
 *
 * @param
 *    void
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_hw_interface_init(void);

/**
 * syna_hw_interface_exit()
 *
 * Delete the lower-level hardware interface module.
 *
 * @param
 *    void
 *
 * @return
 *    none.
 */
void syna_hw_interface_exit(void);


#endif /* end of _SYNAPTICS_TCM2_PLATFORM_H_ */
