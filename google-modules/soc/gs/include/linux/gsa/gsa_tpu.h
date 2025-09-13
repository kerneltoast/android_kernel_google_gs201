/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __LINUX_GSA_TPU_H
#define __LINUX_GSA_TPU_H

#include <linux/device.h>
#include <linux/types.h>

/*
 * GSA TPU Firmware Management interface
 */

/**
 * gsa_load_tpu_fw_image() - load specified TPU firmware image
 * @gsa: pointer to GSA &struct device
 * @img_hdr: dma address of TPU image meta information
 * @img_body: physical address of TPU image body
 *
 * This routine authenticates, locks and loads TPU firmware image specified by
 * @img_hdr/@img_body parameters.
 *
 * The TPU firmware image consists of two parts: a header (always 4K in size)
 * containing image meta information (including authentication parameters and
 * loading instructions) and image body which contains firmware itself. The
 * image header must be loaded into memory region allocated by calling
 * dma_alloc_coherent() for GSA device. This memory chunk can be discarded
 * after gsa_load_tpu_fw_image() call is complete. Firmware image body should
 * be loaded into physically contiguous memory region with base address matching
 * the TPU load address specified within TPU image header. This buffer becomes
 * inaccessible for duration of this call and remains inaccessible after if load
 * operation is successful.
 *
 * In general, the following sequence should happen:
 *   - GSA copies in the image header then authenticates and validates it
 *   - GSA locks down memory region containing firmware body making it
 *     inaccessible by outside world
 *   - GSA authenticates image firmware and if successful
 *   - GSA configure and prepare it for execution
 *
 * Return: 0 on success, negative error otherwise
 */
int gsa_load_tpu_fw_image(struct device *gsa,
			  dma_addr_t img_meta,
			  phys_addr_t img_body);

/**
 * gsa_unload_tpu_fw_image() - unlocks and unloads TPU firmware image
 * @gsa: pointer to GSA &struct device
 *
 * This routine unlocks memory regions locked by previously loaded TPU firmware
 * making them accessible for out side worlds,
 *
 * Return: 0 on success, negative error otherwise
 */
int gsa_unload_tpu_fw_image(struct device *gsa);

/**
 * enum gsa_tpu_state - TPU state
 * @GSA_TPU_STATE_INACTIVE:  TPU firmware is not loaded and TPU is in reset
 * @GSA_TPU_STATE_LOADED:    TPU firmware image is loaded but TPU is in reset
 * @GSA_TPU_STATE_RUNNING:   TPU is running
 * @GSA_TPU_STATE_SUSPENDED: TPU is suspended
 */
enum gsa_tpu_state {
	GSA_TPU_STATE_INACTIVE = 0,
	GSA_TPU_STATE_LOADED,
	GSA_TPU_STATE_RUNNING,
	GSA_TPU_STATE_SUSPENDED,
};

/**
 * enum gsa_tpu_cmd - TPU management commands
 * @GSA_TPU_GET_STATE: return current TPU state
 * @GSA_TPU_START:     take TPU out of reset and start executing loaded
 *                     firmware
 * @GSA_TPU_SUSPEND:   put TPU into suspended state
 * @GSA_TPU_RESUME:    take TPU out of suspended state and resume executing
 * @GSA_TPU_SHUTDOWN:  reset TPU
 */
enum gsa_tpu_cmd {
	GSA_TPU_GET_STATE = 0,
	GSA_TPU_START,
	GSA_TPU_SUSPEND,
	GSA_TPU_RESUME,
	GSA_TPU_SHUTDOWN,
};

/**
 * gsa_send_tpu_cmd() - execute specified TPU management command
 * @gsa: pointer to GSA &struct device
 * @cmd: &enum gsa_tpu_cmd to execute
 *
 * Return: new TPU state (&enum gsa_tpu_state) on success, negative error code
 *         otherwise.
 */
int gsa_send_tpu_cmd(struct device *gsa, enum gsa_tpu_cmd cmd);

#endif /* __LINUX_GSA_IMG_H */
