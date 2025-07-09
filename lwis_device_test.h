/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Test Device Driver
 *
 * Copyright (c) 2022 Google, LLC
 */

#ifndef LWIS_DEVICE_TEST_H_
#define LWIS_DEVICE_TEST_H_

#include "lwis_commands.h"
#include "lwis_device.h"

#define SCRATCH_TEST_DEV_MEMORY_SIZE 32
#define TEST_DEVICE_IRQ_CNT 1
#define TEST_DEVICE_FAKE_INJECTION_IRQ 999

/*
 *  struct lwis_test_device
 *  The device majorly control/handle requests from test clients.
 */
struct lwis_test_device {
	struct lwis_device base_dev;
	/*
	 * For testing purposes, scratch memory is used as register space in
	 * test device.
	 */
	uint8_t scratch_mem[SCRATCH_TEST_DEV_MEMORY_SIZE];
};

int lwis_test_device_init(void);
int lwis_test_device_deinit(void);

#endif /* LWIS_DEVICE_TEST_H_ */
