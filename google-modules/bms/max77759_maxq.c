/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020, Google Inc
 *
 * MAX77759 MAXQ opcode management.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>

#include <misc/logbuffer.h>
#include "max77759_maxq.h"
#include "max77759_regs.h"
#include "gbms_storage.h"

#define MAX_GPIO_TRIG_RISING			0
#define MAX_GPIO_TRIG_FALLING			1

#define MAX_GPIO5_TRIG_MASK			BIT(0)
#define MAX_GPIO5_TRIG(x)			((x) << 0)
#define MAX_GPIO6_TRIG_MASK			BIT(1)
#define MAX_GPIO6_TRIG(x)			((x) << 1)

#define PAYLOAD_REQUEST_LENGTH_BYTES		33
#define PAYLOAD_RESPONSE_LENGTH_BYTES		3
#define OPCODE_GPIO_TRIGGER_READ		0x21
#define OPCODE_GPIO_TRIGGER_R_REQ_LEN		1
#define OPCODE_GPIO_TRIGGER_R_RES_LEN		2
#define OPCODE_GPIO_TRIGGER_R_RES_OFFSET	1
#define OPCODE_GPIO_TRIGGER_WRITE		0x22
#define OPCODE_GPIO_TRIGGER_W_REQ_LEN           2
#define OPCODE_GPIO_TRIGGER_W_REQ_OFFSET        1
#define OPCODE_GPIO_TRIGGER_W_RES_LEN           1
#define OPCODE_GPIO_CONTROL_READ		0x23
#define OPCODE_GPIO_CONTROL_R_REQ_LEN		1
#define OPCODE_GPIO_CONTROL_R_RES_LEN		2
#define OPCODE_GPIO_CONTROL_R_RES_OFFSET	1
#define OPCODE_GPIO_CONTROL_WRITE		0x24
#define OPCODE_GPIO_CONTROL_W_REQ_LEN		2
#define OPCODE_GPIO_CONTROL_W_REQ_OFFSET	1
#define OPCODE_GPIO_CONTROL_W_RES_LEN		1
#define OPCODE_CHECK_CC_AND_SBU			0x85
#define OPCODE_CHECK_CC_AND_SBU_REQ_LEN		9
#define OPCODE_CHECK_CC_AND_SBU_RES_LEN		5
#define OPCODE_USER_SPACE_MAX_ADDR		31
#define OPCODE_USER_SPACE_MAX_LEN		30
#define OPCODE_USER_SPACE_READ			0x81
#define OPCODE_USER_SPACE_R_REQ_LEN		3
#define OPCODE_USER_SPACE_R_RES_LEN		32
#define OPCODE_USER_SPACE_WRITE			0x82
#define OPCODE_USER_SPACE_W_REQ_LEN		32
#define OPCODE_USER_SPACE_W_RES_LEN		32
#define MAXQ_AP_DATAOUT0			0x81
#define MAXQ_AP_DATAOUT32			0xA1
#define MAXQ_AP_DATAIN0				0xB1
#define MAXQ_REPLY_TIMEOUT_MS			200

#define RSBM_ADDR				0
#define RSBR_ADDR				4
#define SUFG_ADDR				8
#define RSOC_ADDR				10
#define RS_TAG_LENGTH				4
#define SU_TAG_LENGTH				1
#define RSOC_TAG_LENGTH				2
#define RS_TAG_OFFSET_ADDR			0
#define RS_TAG_OFFSET_LENGTH			1
#define RS_TAG_OFFSET_DATA			2

#define LOG_BUFFER_SIZE   256

struct max77759_maxq {
	struct completion reply_done;
	/* Denotes the current request in progress. */
	unsigned int req_no;
	/* Updated by the irq handler. */
	unsigned int req_done;
	/* Protects req_no and req_done variables. */
	struct mutex req_lock;
	struct logbuffer *log;
	bool init_done;
	struct regmap *regmap;

	/* To make sure that there is only one request in progress. */
	struct mutex maxq_lock;
	u8 request_opcode;
	bool poll;
};

enum write_userspace_offset {
	ADDR_OPCODE,
	START_ADDR,
	DATA_LEN,
	DATA_START
};

enum request_payload_offset {
	REQUEST_OPCODE,
	/* Following are specific to CHECK_CC_AND_SBU */
	REQUEST_TYPE,
	CC1RD,
	CC2RD,
	CCADCSKIPPED,
	CC1ADC,
	CC2ADC,
	SBU1ADC,
	SBU2ADC,
	WRITE_BYTES
};

enum response_payload_offset {
	RESPONSE_OPCODE,
	RESPONSE_TYPE,
	RESULT,
	CC_THRESH,
	SBU_THRESH
};

/* Caller holds req_lock */
static int maxq_read_response_locked(struct max77759_maxq *maxq,
				     u8 *response, u8 response_len)
{
	int ret = 0;

	ret = regmap_bulk_read(maxq->regmap, MAXQ_AP_DATAIN0, response,
			       response_len);
	if (ret) {
		logbuffer_log(maxq->log, "I2C request failed ret:%d", ret);
		return -EINVAL;
	}

	if (response[RESPONSE_OPCODE] != maxq->request_opcode) {
		logbuffer_log(maxq->log,
			      "OPCODE does not match request:%u response:%u",
			      maxq->request_opcode, response[RESPONSE_OPCODE]);
		return -EINVAL;
	}

	return ret;
}

/* Caller holds req_lock */
static int maxq_wait_for_response_locked(struct max77759_maxq *maxq,
					 unsigned int current_request,
					 u8 *response, u8 response_len)
{
	int ret = 0;
	unsigned long timeout = MAXQ_REPLY_TIMEOUT_MS;

	/* Wait for response from Maxq */
	if (maxq->poll) {
		msleep(timeout);
		ret = maxq_read_response_locked(maxq, response, response_len);
		goto exit;
	}

	/* IRQ from MAXQ */
	while (timeout) {
		mutex_unlock(&maxq->req_lock);
		timeout = wait_for_completion_timeout(&maxq->reply_done,
						      timeout);
		mutex_lock(&maxq->req_lock);
		if (current_request == maxq->req_done) {
			ret = maxq_read_response_locked(maxq, response,
							response_len);
			break;
		} else if (!timeout) {
			logbuffer_log(maxq->log,
				      "Timeout or Request number does not match current:req:%u req_done:%u",
				      current_request, maxq->req_done);
			ret = -EINVAL;
		}
	}

exit:
	logbuffer_log(maxq->log, "MAXQ DONE: current_req: %u ret:%d",
		      current_request, ret);
	return ret;
}

static int maxq_issue_opcode_command(struct max77759_maxq *maxq, u8 *request,
				     u8 request_length, u8 *response,
				     u8 response_length)
{
	unsigned int current_request;
	int ret = -ENODEV;

	if (!maxq->init_done)
		return ret;

	mutex_lock(&maxq->maxq_lock);
	maxq->request_opcode = request[REQUEST_OPCODE];
	mutex_lock(&maxq->req_lock);
	current_request = ++maxq->req_no;

	logbuffer_log(maxq->log, "MAXQ REQ:current_req: %u opcode:%u",
		      current_request, maxq->request_opcode);
	ret = regmap_bulk_write(maxq->regmap, MAXQ_AP_DATAOUT0, request,
				request_length);
	if (ret) {
		logbuffer_log(maxq->log, "I2C request write failed");
		goto req_unlock;
	}

	if (request_length < PAYLOAD_REQUEST_LENGTH_BYTES) {
		ret = regmap_write(maxq->regmap, MAXQ_AP_DATAOUT32, 0);
		if (ret) {
			logbuffer_log(maxq->log,
				      "I2C request write DATAOUT32 failed");
			goto req_unlock;
		}
	}

	ret = maxq_wait_for_response_locked(maxq, current_request,
					    response, response_length);
req_unlock:
	mutex_unlock(&maxq->req_lock);
	mutex_unlock(&maxq->maxq_lock);
	return ret;
}

static void maxq_log_array(struct max77759_maxq *maxq, char *title,
			   u8 *data, int length)
{
	char buf[LOG_BUFFER_SIZE];
	int i, len = 0;

	for (i = 0; i < length; i++) {
		len += scnprintf(&buf[len], LOG_BUFFER_SIZE - len,
				 "%#x ", data[i]);

		if (len >= LOG_BUFFER_SIZE)
			break;
        }

	logbuffer_log(maxq->log, "%s: %s", title, buf);

}

static int maxq_user_space_read(struct max77759_maxq *maxq, u8 *data)
{
	int ret, i;
	u8 request[OPCODE_USER_SPACE_R_REQ_LEN];
	u8 response[OPCODE_USER_SPACE_R_RES_LEN];

	request[ADDR_OPCODE] = OPCODE_USER_SPACE_READ;
	request[START_ADDR] = data[0];
	request[DATA_LEN] = data[1];

	if (request[START_ADDR] > OPCODE_USER_SPACE_MAX_ADDR)
		return -EINVAL;

	if (request[DATA_LEN] > OPCODE_USER_SPACE_MAX_LEN)
		return -EINVAL;

	logbuffer_log(maxq->log, "MAXQ user space read opcode:%#x, start:%#x, length:%#x",
				request[ADDR_OPCODE], request[START_ADDR], request[DATA_LEN]);
	ret = maxq_issue_opcode_command(maxq, request,
					OPCODE_USER_SPACE_R_REQ_LEN,
					response,
					OPCODE_USER_SPACE_R_RES_LEN);

	if (ret < 0)
		return ret;

	for (i = DATA_START; i < DATA_START + request[DATA_LEN]; i++)
		data[i - DATA_START] = response[i];

	maxq_log_array(maxq, "MAXQ user space read result",
					&response[DATA_START], request[DATA_LEN]);

	return ret;
}

static int maxq_user_space_write(struct max77759_maxq *maxq, u8 *data)
{
	int ret, i;
	u8 request[OPCODE_USER_SPACE_W_REQ_LEN];
	u8 response[OPCODE_USER_SPACE_W_RES_LEN];

	request[ADDR_OPCODE] = OPCODE_USER_SPACE_WRITE;
	request[START_ADDR] = data[0];
	request[DATA_LEN] = data[1];

	if (request[START_ADDR] > OPCODE_USER_SPACE_MAX_ADDR)
		return -EINVAL;

	if (request[DATA_LEN] > OPCODE_USER_SPACE_MAX_LEN)
		return -EINVAL;

	logbuffer_log(maxq->log, "MAXQ user space write opcode:%#x, start:%#x, length:%#x",
				request[ADDR_OPCODE], request[START_ADDR], request[DATA_LEN]);

	for (i = DATA_START; i < DATA_START + request[DATA_LEN]; i++)
		request[i] = data[i - 1];

	ret = maxq_issue_opcode_command(maxq, request,
					OPCODE_USER_SPACE_W_REQ_LEN,
					response,
					OPCODE_USER_SPACE_W_RES_LEN);

	if (ret < 0)
		return ret;

	maxq_log_array(maxq, "MAXQ user space write data",
					&response[DATA_START], request[DATA_LEN]);

	return ret;
}

static int maxq_rs_read(struct max77759_maxq *maxq, gbms_tag_t tag, u8 *data)
{
	int ret, len;
	u8 buff[OPCODE_USER_SPACE_R_RES_LEN];

	if (tag == GBMS_TAG_RSBM) {
		buff[RS_TAG_OFFSET_ADDR] = RSBM_ADDR;
		len = RS_TAG_LENGTH;
	} else if (tag == GBMS_TAG_RSBR) {
		buff[RS_TAG_OFFSET_ADDR] = RSBM_ADDR;
		len = RS_TAG_LENGTH;
	} else if (tag == GBMS_TAG_SUFG) {
		buff[RS_TAG_OFFSET_ADDR] = SUFG_ADDR;
		len = SU_TAG_LENGTH;
	} else if (tag == GBMS_TAG_RSOC) {
		buff[RS_TAG_OFFSET_ADDR] = RSOC_ADDR;
		len = RSOC_TAG_LENGTH;
	} else {
		return -EINVAL;
	}

	buff[RS_TAG_OFFSET_LENGTH] = len;

	ret = maxq_user_space_read(maxq, buff);
	if (ret < 0)
		return ret;

	memcpy(data, buff, len);

	return ret;
}

static int maxq_rs_write(struct max77759_maxq *maxq, gbms_tag_t tag, u8 *data)
{
	int ret, len;
	u8 buff[OPCODE_USER_SPACE_W_REQ_LEN];

	if (tag == GBMS_TAG_RSBM) {
		buff[RS_TAG_OFFSET_ADDR] = RSBM_ADDR;
		len = RS_TAG_LENGTH;
	} else if (tag == GBMS_TAG_RSBR) {
		buff[RS_TAG_OFFSET_ADDR] = RSBR_ADDR;
		len = RS_TAG_LENGTH;
	} else if (tag == GBMS_TAG_SUFG) {
		buff[RS_TAG_OFFSET_ADDR] = SUFG_ADDR;
		len = SU_TAG_LENGTH;
	} else if (tag == GBMS_TAG_RSOC) {
		buff[RS_TAG_OFFSET_ADDR] = RSOC_ADDR;
		len = RSOC_TAG_LENGTH;
	} else {
		return -EINVAL;
	}

	buff[RS_TAG_OFFSET_LENGTH] = len;

	memcpy(&buff[RS_TAG_OFFSET_DATA], data, len);

	ret = maxq_user_space_write(maxq, buff);

	return ret;
}

static int maxq_storage_read(gbms_tag_t tag, void *buff, size_t size,
				 void *ptr)
{
	int ret;
	struct max77759_maxq *maxq = ptr;

	switch (tag) {
	case GBMS_TAG_RS32:
		if (size && size > OPCODE_USER_SPACE_R_RES_LEN)
			return -EINVAL;
		ret = maxq_user_space_read(maxq, buff);
		break;
	case GBMS_TAG_RSBM:
	case GBMS_TAG_RSBR:
		if (size && size > RS_TAG_LENGTH)
			return -EINVAL;
		ret = maxq_rs_read(maxq, tag, buff);
		break;
	case GBMS_TAG_SUFG:
		if (size && size > SU_TAG_LENGTH)
			return -EINVAL;
		ret = maxq_rs_read(maxq, tag, buff);
		break;
	case GBMS_TAG_RSOC:
		if (size && size > RSOC_TAG_LENGTH)
			return -EINVAL;
		ret = maxq_rs_read(maxq, tag, buff);
		break;
	default:
		ret = -ENOENT;
		break;
	}

	return ret;
}

static int maxq_storage_write(gbms_tag_t tag, const void *buff, size_t size,
				  void *ptr)
{
	int ret;
	struct max77759_maxq *maxq = ptr;

	switch (tag) {
	case GBMS_TAG_RS32:
		if (size && size > OPCODE_USER_SPACE_W_RES_LEN)
			return -EINVAL;
		ret = maxq_user_space_write(maxq, (void *)buff);
		break;
	case GBMS_TAG_RSBM:
	case GBMS_TAG_RSBR:
		if (size && size > RS_TAG_LENGTH)
			return -EINVAL;
		ret = maxq_rs_write(maxq, tag, (void *)buff);
		break;
	case GBMS_TAG_SUFG:
		if (size && size > SU_TAG_LENGTH)
			return -EINVAL;
		ret = maxq_rs_write(maxq, tag, (void *)buff);
		break;
	case GBMS_TAG_RSOC:
		if (size && size > RSOC_TAG_LENGTH)
			return -EINVAL;
		ret = maxq_rs_write(maxq, tag, (void *)buff);
		break;
	default:
		ret = -ENOENT;
		break;
	}

	return ret;
}

static struct gbms_storage_desc maxq_storage_dsc = {
	.read = maxq_storage_read,
	.write = maxq_storage_write,
};

void maxq_irq(struct max77759_maxq *maxq)
{
	mutex_lock(&maxq->req_lock);
	maxq->req_done = maxq->req_no;
	logbuffer_log(maxq->log, "MAXQ IRQ: req_done: %u", maxq->req_done);
	complete(&maxq->reply_done);
	mutex_unlock(&maxq->req_lock);
}
EXPORT_SYMBOL_GPL(maxq_irq);

int maxq_query_contaminant(struct max77759_maxq *maxq, u8 cc1_raw,
			   u8 cc2_raw, u8 sbu1_raw, u8 sbu2_raw, u8 cc1_rd,
			   u8 cc2_rd, u8 type, u8 cc_adc_skipped,
			   u8 *response, u8 response_len)
{
	int ret;
	u8 payload[OPCODE_CHECK_CC_AND_SBU_REQ_LEN];

	payload[REQUEST_OPCODE] = OPCODE_CHECK_CC_AND_SBU;
	payload[REQUEST_TYPE] = type;
	payload[CC1RD] = cc1_rd;
	payload[CC2RD] = cc2_rd;
	payload[CCADCSKIPPED] = cc_adc_skipped;
	payload[CC1ADC] = cc1_raw;
	payload[CC2ADC] = cc2_raw;
	payload[SBU1ADC] = sbu1_raw;
	payload[SBU2ADC] = sbu2_raw;

	logbuffer_log(maxq->log,
		      "MAXQ opcode:%#x type:%#x cc1_rd:%#x cc2_rd:%#x ADCSKIPPED:%#x cc1adc:%#x cc2adc:%#x sbu1adc:%#x sbu2adc:%#x",
		      OPCODE_CHECK_CC_AND_SBU, type, cc1_rd, cc2_rd,
		      cc_adc_skipped, cc1_raw, cc2_raw, sbu1_raw, sbu2_raw);
	ret = maxq_issue_opcode_command(maxq, payload,
					OPCODE_CHECK_CC_AND_SBU_REQ_LEN,
					response, response_len);
	if (!ret)
		logbuffer_log(maxq->log, "MAXQ Contaminant response:%u",
			      response[RESULT]);

	return ret;
}
EXPORT_SYMBOL_GPL(maxq_query_contaminant);

int maxq_gpio_control_read(struct max77759_maxq *maxq, u8 *gpio)
{
	int ret;
	u8 request = OPCODE_GPIO_CONTROL_READ;
	u8 response[OPCODE_GPIO_CONTROL_R_RES_LEN];

	logbuffer_log(maxq->log, "MAXQ gpio read opcode:%#x", request);
	ret = maxq_issue_opcode_command(maxq, &request,
					OPCODE_GPIO_CONTROL_R_REQ_LEN,
					response,
					OPCODE_GPIO_CONTROL_R_RES_LEN);
	if (!ret)
		*gpio = response[OPCODE_GPIO_CONTROL_R_RES_OFFSET];
	logbuffer_log(maxq->log, "MAXQ GPIO:%#x", *gpio);

	return ret;
}
EXPORT_SYMBOL_GPL(maxq_gpio_control_read);

int maxq_gpio_control_write(struct max77759_maxq *maxq, u8 gpio)
{
	int ret;
	u8 request[OPCODE_GPIO_CONTROL_W_REQ_LEN];
	u8 response[OPCODE_GPIO_CONTROL_W_RES_LEN];

	request[REQUEST_OPCODE] = OPCODE_GPIO_CONTROL_WRITE;
	request[OPCODE_GPIO_CONTROL_W_REQ_OFFSET] = gpio;
	logbuffer_log(maxq->log, "MAXQ gpio write opcode:%#x val:%#x",
		      request[REQUEST_OPCODE],
		      request[OPCODE_GPIO_CONTROL_W_REQ_OFFSET]);
	ret = maxq_issue_opcode_command(maxq, request,
					OPCODE_GPIO_CONTROL_W_REQ_LEN,
					response,
					OPCODE_GPIO_CONTROL_W_RES_LEN);

	return ret;
}
EXPORT_SYMBOL_GPL(maxq_gpio_control_write);

int maxq_gpio_trigger_read(struct max77759_maxq *maxq, u8 gpio, bool *trigger_falling)
{
	int ret;
	u8 request = OPCODE_GPIO_TRIGGER_READ;
	u8 response[OPCODE_GPIO_TRIGGER_R_RES_LEN];

	/* Only GPIO5 and GPIO6 supported */
	if (gpio != 5 && gpio != 6) {
		logbuffer_log(maxq->log, "MAXQ gpio trigger read invalid gpio %d", gpio);
		return -EINVAL;
	}

	logbuffer_log(maxq->log, "MAXQ gpio trigger read opcode:%#x", request);
	ret = maxq_issue_opcode_command(maxq, &request,
					OPCODE_GPIO_TRIGGER_R_REQ_LEN,
					response,
					OPCODE_GPIO_TRIGGER_R_RES_LEN);
	if (!ret) {
		*trigger_falling = response[OPCODE_GPIO_TRIGGER_R_RES_OFFSET] & ((gpio == 5) ?
			MAX_GPIO5_TRIG_MASK : MAX_GPIO6_TRIG_MASK);
		logbuffer_log(maxq->log, "MAXQ GPIO%d TRIGGER:%s", gpio,
			      *trigger_falling ? "falling" : "rising");
	} else {
		logbuffer_log(maxq->log, "MAXQ GPIO%d TRIGGER read failed", gpio);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(maxq_gpio_trigger_read);

int maxq_gpio_trigger_write(struct max77759_maxq *maxq, u8 gpio, bool trigger_falling)
{
	int ret;
	u8 request[OPCODE_GPIO_TRIGGER_W_REQ_LEN], response[OPCODE_GPIO_TRIGGER_W_RES_LEN];
	u8 trigger_val;
	bool trigger_falling_other;

	/* Only GPIO5 and GPIO6 supported */
	if (gpio != 5 && gpio != 6) {
		logbuffer_log(maxq->log, "MAXQ gpio trigger read invalid gpio %d", gpio);
		return -EINVAL;
	}

	/* Read the other GPIO */
	ret = maxq_gpio_trigger_read(maxq, gpio == 5 ? 6 : 5, &trigger_falling_other);
	if (ret < 0)
		return ret;

	/* Compose trigger write val */
	if (gpio == 5) {
		trigger_val = MAX_GPIO6_TRIG(trigger_falling_other ? MAX_GPIO_TRIG_FALLING :
					     MAX_GPIO_TRIG_RISING);
		trigger_val |= MAX_GPIO5_TRIG(trigger_falling ? MAX_GPIO_TRIG_FALLING :
					      MAX_GPIO_TRIG_RISING);
	} else {
		trigger_val = MAX_GPIO5_TRIG(trigger_falling_other ? MAX_GPIO_TRIG_FALLING :
					     MAX_GPIO_TRIG_RISING);
		trigger_val |= MAX_GPIO6_TRIG(trigger_falling ? MAX_GPIO_TRIG_FALLING :
					      MAX_GPIO_TRIG_RISING);
	}

	request[REQUEST_OPCODE] = OPCODE_GPIO_TRIGGER_WRITE;
	request[OPCODE_GPIO_TRIGGER_W_REQ_OFFSET] = trigger_val;
	logbuffer_log(maxq->log, "MAXQ gpio trigger write opcode:%#x val:%#x",
		      request[REQUEST_OPCODE],
		      request[OPCODE_GPIO_TRIGGER_W_REQ_OFFSET]);
	ret = maxq_issue_opcode_command(maxq, request,
					OPCODE_GPIO_TRIGGER_W_REQ_LEN,
					response,
					OPCODE_GPIO_TRIGGER_W_RES_LEN);

	return ret;
}
EXPORT_SYMBOL_GPL(maxq_gpio_trigger_write);

struct max77759_maxq *maxq_init(struct device *dev, struct regmap *regmap,
				bool poll)
{
	struct max77759_maxq *maxq;
	int ret;

	maxq = devm_kzalloc(dev, sizeof(*maxq), GFP_KERNEL);
	if (IS_ERR_OR_NULL(maxq))
		return ERR_PTR(-ENOMEM);

	maxq->log = logbuffer_register("maxq");
	if (IS_ERR_OR_NULL(maxq->log)) {
		dev_err(dev, "MAXQ logbuffer register failed\n");
		maxq->log = NULL;
	}
	maxq->regmap = regmap;

	init_completion(&maxq->reply_done);
	mutex_init(&maxq->maxq_lock);
	mutex_init(&maxq->req_lock);
	maxq->poll = poll;

	ret = gbms_storage_register(&maxq_storage_dsc,
				    "max77759_maxq", maxq);
	if (ret < 0)
		dev_err(dev, "MAXQ gbms_storage_register failed, ret:%d\n", ret);

	maxq->init_done = true;

	logbuffer_log(maxq->log, "MAXQ: probe done");

	return maxq;
}
EXPORT_SYMBOL_GPL(maxq_init);

void maxq_remove(struct max77759_maxq *maxq)
{
	maxq->init_done = false;
	logbuffer_unregister(maxq->log);
}
EXPORT_SYMBOL_GPL(maxq_remove);
MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_DESCRIPTION("Maxim 77759 MAXQ OPCDOE management");
MODULE_LICENSE("GPL");
