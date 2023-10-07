// SPDX-License-Identifier: GPL-2.0
/*
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
 * @file syna_tcm2_testing.c
 *
 * This file implements the sample code to perform chip testing.
 */
#include "syna_tcm2_testing.h"
#include "syna_tcm2_testing_limits.h"
#include "synaptics_touchcom_core_dev.h"
#include "synaptics_touchcom_func_base.h"

/* g_testing_dir represents the root folder of testing sysfs
 */
static struct kobject *g_testing_dir;
static struct syna_tcm *g_tcm_ptr;


/**
 * syna_testing_compare_byte_vector()
 *
 * Sample code to compare the test result with limits
 * by byte vector
 *
 * @param
 *    [ in] data: target test data
 *    [ in] data_size: size of test data
 *    [ in] limit: test limit value to be compared with
 *    [ in] limit_size: size of test limit
 *
 * @return
 *    on success, true; otherwise, return false
 */
static bool syna_testing_compare_byte_vector(unsigned char *data,
		unsigned int data_size, const unsigned char *limit,
		unsigned int limit_size)
{
	bool result = false;
	unsigned char tmp;
	unsigned char p, l;
	int i, j;

	if (!data || (data_size == 0)) {
		LOGE("Invalid test data\n");
		return false;
	}
	if (!limit || (limit_size == 0)) {
		LOGE("Invalid limits\n");
		return false;
	}

	if (limit_size < data_size) {
		LOGE("Limit size mismatched, data size: %d, limits: %d\n",
			data_size, limit_size);
		return false;
	}

	result = true;
	for (i = 0; i < data_size; i++) {
		tmp = data[i];

		for (j = 0; j < 8; j++) {
			p = GET_BIT(tmp, j);
			l = GET_BIT(limit[i], j);
			if (p != l) {
				LOGE("Fail on TRX-%03d (data:%X, limit:%X)\n",
					(i*8 + j), p, l);
				result = false;
			}
		}
	}

	return result;
}

/**
 * syna_testing_compare_frame()
 *
 * Sample code to compare the test result with limits
 * being formatted as a frame
 *
 * @param
 *    [ in] data: target test data
 *    [ in] data_size: size of test data
 *    [ in] rows: the number of rows
 *    [ in] cols: the number of column
 *    [ in] limits_hi: upper-bound test limit
 *    [ in] limits_lo: lower-bound test limit
 *
 * @return
 *    on success, true; otherwise, return false
 */
static bool syna_testing_compare_frame(unsigned char *data,
		unsigned int data_size, int rows, int cols,
		const short *limits_hi, const short *limits_lo)
{
	bool result = false;
	short *data_ptr = NULL;
	short limit;
	int i, j;

	if (!data || (data_size == 0)) {
		LOGE("Invalid test data\n");
		return false;
	}

	if (data_size < (2 * rows * cols)) {
		LOGE("Size mismatched, data:%d (exppected:%d)\n",
			data_size, (2 * rows * cols));
		result = false;
		return false;
	}

	if (rows > LIMIT_BOUNDARY) {
		LOGE("Rows mismatched, rows:%d (exppected:%d)\n",
			rows, LIMIT_BOUNDARY);
		result = false;
		return false;
	}

	if (cols > LIMIT_BOUNDARY) {
		LOGE("Columns mismatched, cols: %d (exppected:%d)\n",
			cols, LIMIT_BOUNDARY);
		result = false;
		return false;
	}

	result = true;

	if (!limits_hi)
		goto end_of_upper_bound_limit;

	data_ptr = (short *)&data[0];
	for (i = 0; i < rows; i++) {
		for (j = 0; j < cols; j++) {
			limit = limits_hi[i * LIMIT_BOUNDARY + j];
			if (*data_ptr > limit) {
				LOGE("Fail on (%2d,%2d)=%5d, limits_hi:%4d\n",
					i, j, *data_ptr, limit);
				result = false;
			}
			data_ptr++;
		}
	}

end_of_upper_bound_limit:

	if (!limits_lo)
		goto end_of_lower_bound_limit;

	data_ptr = (short *)&data[0];
	for (i = 0; i < rows; i++) {
		for (j = 0; j < cols; j++) {
			limit = limits_lo[i * LIMIT_BOUNDARY + j];
			if (*data_ptr < limit) {
				LOGE("Fail on (%2d,%2d)=%5d, limits_lo:%4d\n",
					i, j, *data_ptr, limit);
				result = false;
			}
			data_ptr++;
		}
	}

end_of_lower_bound_limit:
	return result;
}

/**
 * syna_testing_compare_list()
 *
 * Sample code to compare the test result with limits
 * being formatted as a list
 *
 * @param
 *    [ in] data: target test data
 *    [ in] data_size: size of test data
 *    [ in] rows: the number of rows
 *    [ in] cols: the number of column
 *    [ in] limits_hi: upper-bound test limit
 *    [ in] limits_lo: lower-bound test limit
 *
 * @return
 *    on success, true; otherwise, return false
 */
static bool syna_testing_compare_list(unsigned char *data,
		unsigned int data_size, int rows, int cols,
		const int *limits_hi, const int *limits_lo)
{
	bool result = false;
	int *data_ptr = NULL;
	int limit;
	int i;

	if (!data || (data_size == 0)) {
		LOGE("Invalid test data\n");
		return false;
	}

	if (data_size % (rows + cols) != 0) {
		LOGE("Size mismatched, data:%d (exppected:%d * N)\n",
			data_size, (rows + cols));
		result = false;
		return false;
	}

	if (rows > LIMIT_BOUNDARY) {
		LOGE("Rows mismatched, rows:%d (exppected:%d)\n",
			rows, LIMIT_BOUNDARY);
		result = false;
		return false;
	}

	if (cols > LIMIT_BOUNDARY) {
		LOGE("Columns mismatched, cols: %d (exppected:%d)\n",
			cols, LIMIT_BOUNDARY);
		result = false;
		return false;
	}

	result = true;

	if (!limits_hi)
		goto end_of_upper_bound_limit;

	data_ptr = (int *)&data[0];
	for (i = 0; i < cols; i++) {
		limit = limits_hi[i];
		if (*data_ptr > limit) {
			LOGE("Fail on cols-%2d=%5d, limits_hi:%4d\n",
				i, *data_ptr, limit);
			result = false;
		}
		data_ptr++;
	}
	for (i = 0; i < rows; i++) {
		limit = limits_hi[LIMIT_BOUNDARY + i];
		if (*data_ptr > limit) {
			LOGE("Fail on row-%2d=%5d, limits_hi:%4d\n",
				i, *data_ptr, limit);
			result = false;
		}
		data_ptr++;
	}

end_of_upper_bound_limit:

	if (!limits_lo)
		goto end_of_lower_bound_limit;

	data_ptr = (int *)&data[0];
	for (i = 0; i < cols; i++) {
		limit = limits_lo[i];
		if (*data_ptr < limit) {
			LOGE("Fail on cols-%2d=%5d, limits_lo:%4d\n",
				i, *data_ptr, limit);
			result = false;
		}
		data_ptr++;
	}
	for (i = 0; i < rows; i++) {
		limit = limits_lo[LIMIT_BOUNDARY + i];
		if (*data_ptr < limit) {
			LOGE("Fail on row-%2d=%5d, limits_lo:%4d\n",
				i, *data_ptr, limit);
			result = false;
		}
		data_ptr++;
	}

end_of_lower_bound_limit:
	return result;
}

/**
 * syna_testing_device_id()
 *
 * Sample code to ensure the device id is expected
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_device_id(struct syna_tcm *tcm)
{
	int retval;
	bool result;
	struct tcm_identification_info info;
	char *strptr = NULL;

	LOGI("Start testing\n");

	retval = syna_tcm_identify(tcm->tcm_dev, &info);
	if (retval < 0) {
		LOGE("Fail to get identification\n");
		result = false;
		goto exit;
	}

	strptr = strnstr(info.part_number,
					device_id_limit,
					strlen(info.part_number));
	if (strptr != NULL)
		result = true;
	else {
		LOGE("Device ID mismatched, FW: %s (limit: %s)\n",
			info.part_number, device_id_limit);
		result = false;
	}

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_config_id()
 *
 * Sample code to ensure the config id is expected
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_config_id(struct syna_tcm *tcm)
{
	int retval;
	bool result;
	struct tcm_application_info info;
	int idx;

	LOGI("Start testing\n");

	retval = syna_tcm_get_app_info(tcm->tcm_dev, &info);
	if (retval < 0) {
		LOGE("Fail to get app info\n");
		result = false;
		goto exit;
	}

	result = true;
	for (idx = 0; idx < sizeof(config_id_limit); idx++) {
		if (config_id_limit[idx] != info.customer_config_id[idx]) {
			LOGE("Fail on byte.%d (data: %02X, limit: %02X)\n",
				idx, info.customer_config_id[idx],
				config_id_limit[idx]);
			result = false;
		}
	}

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_check_id_show()
 *
 * Attribute to show the result of ID comparsion to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_check_id_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;

	if (!tcm->is_connected) {
		retval = scnprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	count = 0;

	retval = syna_testing_device_id(tcm);

	retval = scnprintf(buf, PAGE_SIZE - count,
			"Device ID check: %s\n",
			(retval < 0) ? "fail" : "pass");

	buf += retval;
	count += retval;

	retval = syna_testing_config_id(tcm);

	retval = scnprintf(buf, PAGE_SIZE - count,
			"Config ID check: %s\n",
			(retval < 0) ? "fail" : "pass");

	buf += retval;
	count += retval;

	retval = count;
exit:
	return retval;
}

static struct kobj_attribute kobj_attr_check_id =
	__ATTR(check_id, 0444, syna_testing_check_id_show, NULL);

/**
 * syna_testing_pt01()
 *
 * Sample code to perform PT01 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt01(struct syna_tcm *tcm, struct tcm_buffer *test_data)
{
	int retval;
	bool result = false;

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID01_TRX_TRX_SHORTS,
			test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID01_TRX_TRX_SHORTS);
		result = false;
		goto exit;
	}

	result = syna_testing_compare_byte_vector(test_data->buf,
			test_data->data_length,
			pt01_limits,
			ARRAY_SIZE(pt01_limits));

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt01_show()
 *
 * Attribute to show the result of PT01 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt01_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval, i;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;
	struct tcm_buffer test_data;

	if (!tcm->is_connected) {
		count = scnprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, true);

	syna_tcm_buf_init(&test_data);

	retval = syna_testing_pt01(tcm, &test_data);

	count += scnprintf(buf, PAGE_SIZE,
			"TEST PT$01: %s\n", (retval < 0) ? "fail" : "pass");

	for (i = 0; i < test_data.data_length; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count, "%d ",
				test_data.buf[i]);
	}
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	syna_tcm_buf_release(&test_data);

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, false);
exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt01 =
	__ATTR(pt01, 0444, syna_testing_pt01_show, NULL);

/**
 * syna_testing_pt05()
 *
 * Sample code to perform PT05 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt05(struct syna_tcm *tcm, struct tcm_buffer *test_data)
{
	int retval;
	bool result = false;

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID05_FULL_RAW_CAP,
			test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID05_FULL_RAW_CAP);
		result = false;
		goto exit;
	}

	result = syna_testing_compare_frame(test_data->buf,
			test_data->data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const short *)&pt05_hi_limits[0],
			(const short *)&pt05_lo_limits[0]);

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt05_show()
 *
 * Attribute to show the result of PT05 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt05_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval, i, j;
	short *data_ptr = NULL;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;
	struct tcm_buffer test_data;

	if (!tcm->is_connected) {
		count = scnprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, true);

	syna_tcm_buf_init(&test_data);

	retval = syna_testing_pt05(tcm, &test_data);

	count += scnprintf(buf, PAGE_SIZE,
			"TEST PT$05: %s\n", (retval < 0) ? "fail" : "pass");

	count += scnprintf(buf + count, PAGE_SIZE - count, "%d %d\n",
			tcm->tcm_dev->cols, tcm->tcm_dev->rows);

	data_ptr = (short *)&(test_data.buf[0]);
	for (i = 0; i < tcm->tcm_dev->rows; i++) {
		for (j = 0; j < tcm->tcm_dev->cols; j++) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "%d ",
					data_ptr[i * tcm->tcm_dev->cols + j]);
		}
		count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	}

	syna_tcm_buf_release(&test_data);

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, false);
exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt05 =
	__ATTR(pt05, 0444, syna_testing_pt05_show, NULL);

/**
 * syna_testing_pt0a()
 *
 * Sample code to perform PT0A testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt0a(struct syna_tcm *tcm, struct tcm_buffer *test_data)
{
	int retval;
	bool result = false;

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID10_DELTA_NOISE,
			test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID10_DELTA_NOISE);
		result = false;
		goto exit;
	}

	result = syna_testing_compare_frame(test_data->buf,
			test_data->data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const short *)&pt0a_hi_limits[0],
			(const short *)&pt0a_lo_limits[0]);

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt0a_show()
 *
 * Attribute to show the result of PT0A test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt0a_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval, i, j;
	short *data_ptr = NULL;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;
	struct tcm_buffer test_data;

	if (!tcm->is_connected) {
		count = scnprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, true);

	syna_tcm_buf_init(&test_data);

	retval = syna_testing_pt0a(tcm, &test_data);

	count += scnprintf(buf, PAGE_SIZE,
			"TEST PT$0A: %s\n", (retval < 0) ? "fail" : "pass");

	count += scnprintf(buf + count, PAGE_SIZE - count, "%d %d\n",
			tcm->tcm_dev->cols, tcm->tcm_dev->rows);

	data_ptr = (short *)&(test_data.buf[0]);
	for (i = 0; i < tcm->tcm_dev->rows; i++) {
		for (j = 0; j < tcm->tcm_dev->cols; j++) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "%d ",
					data_ptr[i * tcm->tcm_dev->cols + j]);
		}
		count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	}

	syna_tcm_buf_release(&test_data);

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, false);
exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt0a =
	__ATTR(pt0a, 0444, syna_testing_pt0a_show, NULL);

/**
 * syna_testing_pt10()
 *
 * Sample code to perform PT10 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt10(struct syna_tcm *tcm, struct tcm_buffer *test_data)
{
	int retval;
	bool result = false;

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID16_SENSOR_SPEED,
			test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID16_SENSOR_SPEED);
		result = false;
		goto exit;
	}

	result = syna_testing_compare_frame(test_data->buf,
			test_data->data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const short *)&pt10_hi_limits[0],
			(const short *)&pt10_lo_limits[0]);

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt10_show()
 *
 * Attribute to show the result of PT10 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt10_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval, i, j;
	short *data_ptr = NULL;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;
	struct tcm_buffer test_data;

	if (!tcm->is_connected) {
		count = scnprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, true);

	syna_tcm_buf_init(&test_data);

	retval = syna_testing_pt10(tcm, &test_data);

	count += scnprintf(buf, PAGE_SIZE,
			"TEST PT$10: %s\n", (retval < 0) ? "fail" : "pass");

	count += scnprintf(buf + count, PAGE_SIZE - count, "%d %d\n",
			tcm->tcm_dev->cols, tcm->tcm_dev->rows);

	data_ptr = (short *)&(test_data.buf[0]);
	for (i = 0; i < tcm->tcm_dev->rows; i++) {
		for (j = 0; j < tcm->tcm_dev->cols; j++) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "%d ",
					data_ptr[i * tcm->tcm_dev->cols + j]);
		}
		count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	}

	syna_tcm_buf_release(&test_data);

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, false);
exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt10 =
	__ATTR(pt10, 0444, syna_testing_pt10_show, NULL);

/**
 * syna_testing_pt11()
 *
 * Sample code to perform PT11 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt11(struct syna_tcm *tcm, struct tcm_buffer *test_data)
{
	int retval;
	bool result = false;

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID17_ADC_RANGE,
			test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID17_ADC_RANGE);
		result = false;
		goto exit;
	}

	result = syna_testing_compare_frame(test_data->buf,
			test_data->data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const short *)&pt11_hi_limits[0],
			(const short *)&pt11_lo_limits[0]);

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt11_show()
 *
 * Attribute to show the result of PT11 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt11_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval, i, j;
	short *data_ptr = NULL;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;
	struct tcm_buffer test_data;

	if (!tcm->is_connected) {
		count = scnprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, true);

	syna_tcm_buf_init(&test_data);

	retval = syna_testing_pt11(tcm, &test_data);

	count += scnprintf(buf, PAGE_SIZE,
			"TEST PT$11: %s\n", (retval < 0) ? "fail" : "pass");

	count += scnprintf(buf + count, PAGE_SIZE - count, "%d %d\n",
			tcm->tcm_dev->cols, tcm->tcm_dev->rows);

	data_ptr = (short *)&(test_data.buf[0]);
	for (i = 0; i < tcm->tcm_dev->rows; i++) {
		for (j = 0; j < tcm->tcm_dev->cols; j++) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "%d ",
					data_ptr[i * tcm->tcm_dev->cols + j]);
		}
		count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	}

	syna_tcm_buf_release(&test_data);

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, false);
exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt11 =
	__ATTR(pt11, 0444, syna_testing_pt11_show, NULL);

/**
 * syna_testing_pt12()
 *
 * Sample code to perform PT12 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt12(struct syna_tcm *tcm, struct tcm_buffer *test_data)
{
	int retval;
	bool result = false;

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID18_HYBRID_ABS_RAW,
			test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID18_HYBRID_ABS_RAW);
		result = false;
		goto exit;
	}

	result = syna_testing_compare_list(test_data->buf,
			test_data->data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const int *)&pt12_limits[0],
			NULL);

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt12_show()
 *
 * Attribute to show the result of PT12 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt12_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval, i;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;
	struct tcm_buffer test_data;
	int *data_ptr = NULL;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, true);

	syna_tcm_buf_init(&test_data);

	retval = syna_testing_pt12(tcm, &test_data);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$12: %s\n", (retval < 0) ? "fail" : "pass");

	count += scnprintf(buf + count, PAGE_SIZE - count, "%d %d\n",
			tcm->tcm_dev->cols, tcm->tcm_dev->rows);

	data_ptr = (int *)&(test_data.buf[0]);
	for (i = 0; i < tcm->tcm_dev->cols; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count, "%d ",
				data_ptr[i]);
	}
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	for (i = 0; i < tcm->tcm_dev->rows; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count, "%d ",
				data_ptr[tcm->tcm_dev->cols + i]);
	}
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	syna_tcm_buf_release(&test_data);

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, false);
exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt12 =
	__ATTR(pt12, 0444, syna_testing_pt12_show, NULL);

/**
 * syna_testing_pt16()
 *
 * Sample code to perform PT16 testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt16(struct syna_tcm *tcm, struct tcm_buffer *test_data)
{
	int retval;
	bool result = false;

	LOGI("Start testing\n");

	retval = syna_tcm_run_production_test(tcm->tcm_dev,
			TEST_PID22_TRANS_CAP_RAW,
			test_data);
	if (retval < 0) {
		LOGE("Fail to run test %d\n", TEST_PID22_TRANS_CAP_RAW);
		result = false;
		goto exit;
	}

	result = syna_testing_compare_frame(test_data->buf,
			test_data->data_length,
			tcm->tcm_dev->rows,
			tcm->tcm_dev->cols,
			(const short *)&pt16_hi_limits[0],
			(const short *)&pt16_lo_limits[0]);

exit:
	LOGI("Result = %s\n", (result)?"pass":"fail");

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt16_show()
 *
 * Attribute to show the result of PT11 test to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt16_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval, i, j;
	short *data_ptr = NULL;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;
	struct tcm_buffer test_data;

	if (!tcm->is_connected) {
		count = snprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, true);

	syna_tcm_buf_init(&test_data);

	retval = syna_testing_pt16(tcm, &test_data);

	count = snprintf(buf, PAGE_SIZE,
			"TEST PT$16: %s\n", (retval < 0) ? "fail" : "pass");

	count += scnprintf(buf + count, PAGE_SIZE - count, "%d %d\n",
			tcm->tcm_dev->cols, tcm->tcm_dev->rows);

	data_ptr = (short *)&(test_data.buf[0]);
	for (i = 0; i < tcm->tcm_dev->rows; i++) {
		for (j = 0; j < tcm->tcm_dev->cols; j++) {
			count += scnprintf(buf + count, PAGE_SIZE - count, "%d ",
					data_ptr[i * tcm->tcm_dev->cols + j]);
		}
		count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	}

	syna_tcm_buf_release(&test_data);

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, false);
exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt16 =
	__ATTR(pt16, 0444, syna_testing_pt16_show, NULL);


/**
 * syna_testing_pt_tag_moisture()
 *
 * Sample code to perform Tags Moisture (RID30) testing
 *
 * @param
 *    [ in] tcm: the driver handle
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
static int syna_testing_pt_tag_moisture(struct syna_tcm *tcm, struct tcm_buffer *test_data)
{
	int retval;
	bool result = false;
	unsigned char code;
	int attempt = 0;
	short *data_ptr = NULL;
	short limit;
	int i, j, rows, cols;

	rows = tcm->tcm_dev->rows;
	cols = tcm->tcm_dev->cols;

	LOGI("Start testing\n");

	/* do test in polling; disable irq */
	if (tcm->hw_if->ops_enable_irq)
		tcm->hw_if->ops_enable_irq(tcm->hw_if, false);

	syna_tcm_set_dynamic_config(tcm->tcm_dev, DC_DISABLE_DOZE, 1, RESP_IN_POLLING);

	retval = syna_tcm_enable_report(tcm->tcm_dev, 30, true);
	if (retval < 0) {
		LOGE("Fail to enable RID30\n");
		result = false;
		goto exit;
	}

	for (attempt = 0; attempt < 3; attempt++) {
		retval = syna_tcm_get_event_data(tcm->tcm_dev, &code,
			test_data);
		if ((retval >= 0) && (code == 30))
			break;
	}

	if ((retval < 0) || (code != 30)) {
		LOGE("Fail to get a frame of RID30 to test\n");
		result = false;
		goto exit;
	}

	if (test_data->data_length % (rows * cols) != 0) {
		LOGE("Invalid frame size of RID30, %d\n", test_data->data_length);
		result = false;
		goto exit;
	}

	retval = syna_tcm_enable_report(tcm->tcm_dev, 30, false);
	if (retval < 0) {
		LOGE("Fail to disable RID30\n");
		result = false;
		goto exit;
	}

	/* compare to the limits */
	result = true;
	data_ptr = (short *)&test_data->buf[0];
	for (i = 0; i < rows; i++) {
		for (j = 0; j < cols; j++) {
			if (*data_ptr < 0)
				continue;

			limit = pt_moisture_limits[i * LIMIT_BOUNDARY + j];
			if (*data_ptr > limit) {
				LOGE("Fail on (%2d,%2d)=%5d, limits_hi:%4d\n",
					i, j, *data_ptr, limit);
				result = false;
			}
			data_ptr++;
		}
	}

exit:
	syna_tcm_set_dynamic_config(tcm->tcm_dev, DC_DISABLE_DOZE, 0, RESP_IN_POLLING);

	LOGI("Result = %s\n", (result)?"pass":"fail");

	/* recover the irq */
	if (tcm->hw_if->ops_enable_irq)
		tcm->hw_if->ops_enable_irq(tcm->hw_if, true);

	return ((result) ? 0 : -1);
}

/**
 * syna_testing_pt_moisture_show()
 *
 * Attribute to show the result of Tags Moisture (RID30) to the console.
 *
 * @param
 *    [ in] kobj:  an instance of kobj
 *    [ in] attr:  an instance of kobj attribute structure
 *    [out] buf:  string buffer shown on console
 *
 * @return
 *    on success, number of characters being output;
 *    otherwise, negative value on error.
 */
static ssize_t syna_testing_pt_moisture_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int retval, i, j;
	short *data_ptr = NULL;
	unsigned int count = 0;
	struct syna_tcm *tcm = g_tcm_ptr;
	struct tcm_buffer test_data;

	if (!tcm->is_connected) {
		count = scnprintf(buf, PAGE_SIZE,
				"Device is NOT connected\n");
		goto exit;
	}

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, true);

	syna_tcm_buf_init(&test_data);

	retval = syna_testing_pt_tag_moisture(tcm, &test_data);

	count += scnprintf(buf, PAGE_SIZE,
			"TEST Tags Moisture: %s\n",
			(retval < 0) ? "fail" : "pass");

	count += scnprintf(buf + count, PAGE_SIZE - count, "%d %d\n",
			tcm->tcm_dev->cols, tcm->tcm_dev->rows);

	if (retval == 0) {
		data_ptr = (short *)&(test_data.buf[0]);
		for (i = 0; i < tcm->tcm_dev->rows; i++) {
			for (j = 0; j < tcm->tcm_dev->cols; j++) {
				count += scnprintf(buf + count, PAGE_SIZE - count, "%d ",
						data_ptr[i * tcm->tcm_dev->cols + j]);
			}
			count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
		}
	}

	syna_tcm_buf_release(&test_data);

	syna_set_bus_ref(tcm, SYNA_BUS_REF_SYSFS, false);

exit:
	return count;
}

static struct kobj_attribute kobj_attr_pt_tag_moisture =
	__ATTR(pt_moisture, 0444, syna_testing_pt_moisture_show, NULL);

/*
 * declaration of sysfs attributes
 */
static struct attribute *attrs[] = {
	&kobj_attr_check_id.attr,
	&kobj_attr_pt01.attr,
	&kobj_attr_pt05.attr,
	&kobj_attr_pt0a.attr,
	&kobj_attr_pt10.attr,
	&kobj_attr_pt11.attr,
	&kobj_attr_pt12.attr,
	&kobj_attr_pt16.attr,
	&kobj_attr_pt_tag_moisture.attr,
	NULL,
};

static struct attribute_group attr_testing_group = {
	.attrs = attrs,
};

/**
 * syna_testing_create_dir()
 *
 * Create a directory and register it with sysfs.
 * Then, create all defined sysfs files.
 *
 * @param
 *    [ in] tcm:  the driver handle
 *    [ in] sysfs_dir: root directory of sysfs nodes
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
int syna_testing_create_dir(struct syna_tcm *tcm,
		struct kobject *sysfs_dir)
{
	int retval = 0;

	g_testing_dir = kobject_create_and_add("testing",
			sysfs_dir);
	if (!g_testing_dir) {
		LOGE("Fail to create testing directory\n");
		return -EINVAL;
	}

	retval = sysfs_create_group(g_testing_dir, &attr_testing_group);
	if (retval < 0) {
		LOGE("Fail to create sysfs group\n");

		kobject_put(g_testing_dir);
		return retval;
	}

	g_tcm_ptr = tcm;

	return 0;
}
/**
 *syna_testing_remove_dir()
 *
 * Remove the allocate sysfs directory
 *
 * @param
 *    none
 *
 * @return
 *    on success, 0; otherwise, negative value on error.
 */
void syna_testing_remove_dir(void)
{
	if (g_testing_dir) {
		sysfs_remove_group(g_testing_dir, &attr_testing_group);

		kobject_put(g_testing_dir);
	}
}
