/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2023 Google LLC
 */
#ifndef __LINUX_GSA_DSP_H
#define __LINUX_GSA_DSP_H

#include <linux/device.h>
#include <linux/types.h>

/*
 * GSA DSP Firmware Management interface
 */

/**
 * gsa_load_dsp_fw_image() - load specified DSP firmware image
 * @gsa: pointer to GSA &struct device
 * @img_hdr: dma address of DSP image meta information
 * @img_body: physical address of DSP image body
 *
 * This routine authenticates, locks and loads DSP firmware image specified by
 * @img_hdr/@img_body parameters.
 *
 * The DSP firmware image consists of two parts: a header (always 4K in size)
 * containing image meta information (including authentication parameters and
 * loading instructions) and image body which contains firmware itself. The
 * image header must be loaded into memory region allocated by calling
 * dma_alloc_coherent() for GSA device. This memory chunk can be discarded
 * after gsa_load_dsp_fw_image() call is complete. Firmware image body should
 * be loaded into physically contiguous memory region with base address matching
 * the DSP load address specified within DSP image header. This buffer becomes
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
int gsa_load_dsp_fw_image(struct device *gsa, dma_addr_t img_meta, phys_addr_t img_body);

/**
 * gsa_unload_dsp_fw_image() - unlocks and unloads DSP firmware image
 * @gsa: pointer to GSA &struct device
 *
 * This routine unlocks memory regions locked by previously loaded DSP firmware
 * making them accessible for out side worlds,
 *
 * Return: 0 on success, negative error otherwise
 */
int gsa_unload_dsp_fw_image(struct device *gsa);

/**
 * enum gsa_dsp_state - DSP state
 * @GSA_DSP_STATE_INACTIVE:  All DSP firmware images are not loaded
 * @GSA_DSP_STATE_LOADING:   DSP firmware images are loading
 * @GSA_DSP_STATE_LOADED:    All DSP firmware images are loaded
 * @GSA_DSP_STATE_RUNNING:   DSP is running
 * @GSA_DSP_STATE_SUSPENDED: DSP is suspended
 */
enum gsa_dsp_state {
	GSA_DSP_STATE_INACTIVE = 0,
	GSA_DSP_STATE_LOADING,
	GSA_DSP_STATE_LOADED,
	GSA_DSP_STATE_RUNNING,
	GSA_DSP_STATE_SUSPENDED,
};

/**
 * enum gsa_dsp_cmd - DSP management commands
 * @GSA_DSP_GET_STATE: return current DSP state
 * @GSA_DSP_START:     take DSP out of reset and start executing loaded
 *                     firmware
 * @GSA_DSP_SUSPEND:   put DSP into suspended state
 * @GSA_DSP_RESUME:    take DSP out of suspended state and resume executing
 * @GSA_DSP_SHUTDOWN:  reset DSP
 */
enum gsa_dsp_cmd {
	GSA_DSP_GET_STATE = 0,
	GSA_DSP_START,
	GSA_DSP_SUSPEND,
	GSA_DSP_RESUME,
	GSA_DSP_SHUTDOWN,
};

/**
 * gsa_send_dsp_cmd() - execute specified DSP management command
 * @gsa: pointer to GSA &struct device
 * @cmd: &enum gsa_dsp_cmd to execute
 *
 * Return: new DSP state (&enum gsa_dsp_state) on success, negative error code
 *         otherwise.
 */
int gsa_send_dsp_cmd(struct device *gsa, enum gsa_dsp_cmd cmd);

#endif /* __LINUX_GSA_DSP_H */
