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
 * @file synaptics_touchcom_func_base.h
 *
 * This file declares generic and foundational APIs being used to communicate
 * with Synaptics touch controller through TouchComm communication protocol.
 */

#ifndef _SYNAPTICS_TOUCHCOM_BASE_FUNCS_H_
#define _SYNAPTICS_TOUCHCOM_BASE_FUNCS_H_

#include "synaptics_touchcom_core_dev.h"

/**
 * syna_tcm_allocate_device()
 *
 * Create the TouchCom core device handle.
 * This function must be called in order to allocate the main device handle,
 * structure syna_tcm_dev, which will be passed to all other operations and
 * functions within the entire source code.
 *
 * Meanwhile, caller has to prepare specific syna_tcm_hw_interface structure,
 * so that all the implemented functions can access hardware components
 * through syna_tcm_hw_interface.
 *
 * @param
 *    [out] ptcm_dev_ptr: a pointer to the device handle returned
 *    [ in] hw_if:        hardware-specific data on target platform
 *    [ in] resp_reading: default resp reading method
 *                        set 'RESP_IN_ATTN' to apply ATTN-driven method;
 *                        set 'RESP_IN_POLLING' to read in resp by polling
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_allocate_device(struct tcm_dev **ptcm_dev_ptr,
		struct syna_hw_interface *hw_if, unsigned int resp_reading);

/**
 * syna_tcm_remove_device()
 *
 * Remove the TouchCom core device handle.
 * This function must be invoked when the device is no longer needed.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    none.
 */
void syna_tcm_remove_device(struct tcm_dev *tcm_dev);

/**
 * syna_tcm_detect_device()
 *
 * Determine the type of device being connected and distinguish which
 * version of TouchCom firmware running on the device.
 * This function should be called before using this TouchComm core library.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    on success, the current mode running on the device is returned;
 *    otherwise, negative value on error.
 */
int syna_tcm_detect_device(struct tcm_dev *tcm_dev);

/**
 * syna_tcm_get_event_data()
 *
 * Helper to read TouchComm messages when ATTN signal is asserted.
 * After returning, the ATTN signal should be no longer asserted.
 *
 * The 'code' returned will guide the caller on the next action.
 * For example, do touch reporting once returned code is equal to REPORT_TOUCH.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [out] code:    received report code
 *    [out] report:  report data returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_event_data(struct tcm_dev *tcm_dev,
		unsigned char *code,
		struct tcm_buffer *report);

/**
 * syna_tcm_change_resp_read()
 *
 * Helper to change the default resp reading method, which was previously set
 * when calling syna_tcm_allocate_device
 *
 * @param
 *    [in] tcm_dev: the device handle
 *    [in] request: resp reading method to change
 *                  set '0' or 'RESP_IN_ATTN' for ATTN-driven; otherwise,
 *                  assign a positive value standing for the polling time
 * @return
 *    none.
 */
void syna_tcm_change_resp_read(struct tcm_dev *tcm_dev, unsigned int request);

/**
 * syna_tcm_identify()
 *
 * Implement the standard command code, which is used to request
 * an IDENTIFY report packet
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [out] id_info: the identification info packet returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_identify(struct tcm_dev *tcm_dev,
		struct tcm_identification_info *id_info);

/**
 * syna_tcm_reset()
 *
 * Implement the standard command code, which is used to perform a sw reset
 * immediately. After a successful reset, an IDENTIFY report to indicate that
 * device is ready.
 *
 * Caller shall be aware that the firmware will be reloaded after reset.
 * Therefore, if expecting that a different firmware version is loaded, please
 * do app firmware setup after reset.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_reset(struct tcm_dev *tcm_dev);

/**
 * syna_tcm_enable_report()
 *
 * Implement the application fw command code to enable or disable a report.
 *
 * @param
 *    [ in] tcm_dev:     the device handle
 *    [ in] report_code: the requested report code being generated
 *    [ in] en:          '1' for enabling; '0' for disabling
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_enable_report(struct tcm_dev *tcm_dev,
		unsigned char report_code, bool en);

/**
 * syna_tcm_switch_fw_mode()
 *
 * Implement the command code to switch the firmware mode.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] mode:    target firmware mode
 *    [ in] fw_switch_delay: delay time for fw mode switching.
 *                           a positive value presents the time for polling;
 *                           or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_switch_fw_mode(struct tcm_dev *tcm_dev,
		unsigned char mode, unsigned int fw_switch_delay);

/**
 * syna_tcm_get_boot_info()
 *
 * Implement the bootloader command code, which is used to request a
 * boot information packet.
 *
 * @param
 *    [ in] tcm_dev:   the device handle
 *    [out] boot_info: the boot info packet returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_boot_info(struct tcm_dev *tcm_dev,
		struct tcm_boot_info *boot_info);

/**
 * syna_tcm_get_app_info()
 *
 * Implement the application fw command code to request an application
 * info packet from device
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [out] app_info: the application info packet returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_app_info(struct tcm_dev *tcm_dev,
		struct tcm_application_info *app_info);

/**
 * syna_tcm_get_static_config()
 *
 * Implement the application fw command code to retrieve the contents of
 * the static configuration.
 *
 * The size of static configuration is available in app info packet.
 *
 * @param
 *    [ in] tcm_dev:   the device handle
 *    [out] buf:       buffer stored the static configuration
 *    [ in] buf_size:  the size of given buffer
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_static_config(struct tcm_dev *tcm_dev,
		unsigned char *buf, unsigned int buf_size);

/**
 * syna_tcm_set_static_config()
 *
 * Implement the application fw command code to set the contents of
 * the static configuration. When the write is completed, the device will
 * restart touch sensing with the new settings
 *
 * The size of static configuration is available in app info packet.
 *
 * @param
 *    [ in] tcm_dev:          the device handle
 *    [ in] config_data:      the data of static configuration
 *    [ in] config_data_size: the size of given data
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_set_static_config(struct tcm_dev *tcm_dev,
		unsigned char *config_data, unsigned int config_data_size);

/**
 * syna_tcm_get_dynamic_config()
 *
 * Implement the application fw command code to get the value from the a single
 * field of the dynamic configuration
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [ in] id:       target field id
 *    [out] value:    the value returned
 *    [ in] delay_ms_resp: delay time for response reading.
 *                         a positive value presents the time for polling;
 *                         or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_dynamic_config(struct tcm_dev *tcm_dev,
		unsigned char id, unsigned short *value,
		unsigned int delay_ms_resp);

/**
 * syna_tcm_set_dynamic_config()
 *
 * Implement the application fw command code to set the specified value to
 * the selected field of the dynamic configuration
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [ in] id:       target field id
 *    [ in] value:    the value to the selected field
 *    [ in] delay_ms_resp: delay time for response reading.
 *                         a positive value presents the time for polling;
 *                         or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_set_dynamic_config(struct tcm_dev *tcm_dev,
		unsigned char id, unsigned short value,
		unsigned int delay_ms_resp);

/**
 * syna_tcm_rezero()
 *
 * Implement the application fw command code to force the device to rezero its
 * baseline estimate.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_rezero(struct tcm_dev *tcm_dev);

/**
 * syna_tcm_set_config_id()
 *
 * Implement the application fw command code to set the 16-byte config id,
 * which can be read in the app info packet.
 *
 * @param
 *    [ in] tcm_dev:   the device handle
 *    [ in] config_id: config id to be set
 *    [ in] size:      size of input data
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_set_config_id(struct tcm_dev *tcm_dev,
		unsigned char *config_id, unsigned int size);

/**
 * syna_tcm_sleep()
 *
 * Implement the application fw command code to put the device into low power
 * deep sleep mode or set to normal active mode
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] en:      '1' to low power deep sleep mode; '0' to active mode
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_sleep(struct tcm_dev *tcm_dev, bool en);

/**
 * syna_tcm_get_features()
 *
 * Implement the application fw command code to query the supported features.
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [out] info:    the features description packet returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_get_features(struct tcm_dev *tcm_dev,
		struct tcm_features_info *info);

/**
 * syna_tcm_run_production_test()
 *
 * Implement the appplication fw command code to request the device to run
 * the production test.
 *
 * Production tests are listed at enum test_code (PID$).
 *
 * @param
 *    [ in] tcm_dev:    the device handle
 *    [ in] test_item:  the requested testing item
 *    [out] tdata:      testing data returned
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_run_production_test(struct tcm_dev *tcm_dev,
	unsigned char test_item, struct tcm_buffer *tdata);

/**
 * syna_tcm_send_command()
 *
 * Helper to forward the custom commnd to the device
 *
 * @param
 *    [ in] tcm_dev:        the device handle
 *    [ in] command:        TouchComm command
 *    [ in] payload:        data payload, if any
 *    [ in] payload_length: length of data payload, if any
 *    [out] resp_code:      response code returned
 *    [out] resp:           buffer to store the response data
 *    [ in] delay_ms_resp:  delay time for response reading.
 *                          a positive value presents the time for polling;
 *                          or, set '0' or 'RESP_IN_ATTN' for ATTN driven
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_send_command(struct tcm_dev *tcm_dev,
			unsigned char command, unsigned char *payload,
			unsigned int payload_length, unsigned char *resp_code,
			struct tcm_buffer *resp, unsigned int delay_ms_resp);

/**
 * syna_tcm_enable_predict_reading()
 *
 * predict reading aims to retrieve all data in one transfer;
 * while, standard reads will read 4-byte header and payload data separately
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *    [ in] en:      '1' to low power deep sleep mode; '0' to active mode
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_enable_predict_reading(struct tcm_dev *tcm_dev, bool en);
/**
 * syna_tcm_get_message_crc()
 *
 * this function is used to return the crc of message retrieved previously
 *
 * @param
 *    [ in] tcm_dev: the device handle
 *
 * @return
 *    2 bytes crc value
 */
unsigned short syna_tcm_get_message_crc(struct tcm_dev *tcm_dev);
/**
 * syna_tcm_set_reset_occurrence_callback()
 *
 * Set up callback function once an unexpected reset received
 *
 * @param
 *    [ in] tcm_dev:  the device handle
 *    [ in] p_cb:     the pointer of callback function
 *    [ in] p_cbdata: pointer to caller data
 *
 * @return
 *    on success, 0 or positive value; otherwise, negative value on error.
 */
int syna_tcm_set_reset_occurrence_callback(struct tcm_dev *tcm_dev,
		tcm_reset_occurrence_callback_t p_cb, void *p_cbdata);


#endif /* end of _SYNAPTICS_TOUCHCOM_BASE_FUNCS_H_ */
