/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __LINUX_GSA_AOC_H
#define __LINUX_GSA_AOC_H

#include <linux/device.h>
#include <linux/types.h>

/*
 * GSA AOC Firmware Management interface
 */

/**
 * gsa_load_aoc_fw_image() - load specified AOC firmware image
 * @gsa: pointer to GSA &struct device
 * @img_hdr: dma address of AOC image meta information
 * @img_body: physical address of AOC image body
 *
 * This routine authenticates, locks and loads AOC firmware image specified by
 * @img_hdr/@img_body parameters.
 *
 * The AOC firmware image consists of two parts: a header (always 4K in size)
 * containing image meta information (including authentication parameters and
 * loading instructions) and image body which contains firmware itself. The
 * image header must be loaded into memory region allocated by calling
 * dma_alloc_coherent() for GSA device. This memory chunk can be discarded
 * after gsa_load_aoc_fw_image() call is complete. Firmware image body should
 * be loaded into physically contiguous memory region with base address matching
 * the AOC load address specified within AOC image header. This buffer becomes
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
int gsa_load_aoc_fw_image(struct device *gsa,
			  dma_addr_t img_meta,
			  phys_addr_t img_body);

/**
 * gsa_unload_aoc_fw_image() - unlocks and unloads AOC firmware image
 * @gsa: pointer to GSA &struct device
 *
 * This routine unlocks memory regions locked by previously loaded AOC firmware
 * making them accessible for out side worlds,
 *
 * Return: 0 on success, negative error otherwise
 */
int gsa_unload_aoc_fw_image(struct device *gsa);

/**
 * enum gsa_aoc_state - AOC state
 * @GSA_AOC_STATE_INACTIVE:  AOC firmware is not loaded and AOC is in reset
 * @GSA_AOC_STATE_LOADED:    AOC firmware image is loaded but AOC is in reset
 * @GSA_AOC_STATE_RUNNING:   AOC is running
 */
enum gsa_aoc_state {
	GSA_AOC_STATE_INACTIVE = 0,
	GSA_AOC_STATE_LOADED,
	GSA_AOC_STATE_RUNNING,
};

/**
 * enum gsa_aoc_cmd - AOC management commands
 * @GSA_AOC_GET_STATE:     return current AOC state
 * @GSA_AOC_START:         take AOC out of reset and start executing loaded
 *                         firmware
 * @GSA_AOC_SHUTDOWN:      put AOC into reset and return to loaded state
 * @GSA_AOC_RELEASE_RESET: take AOC A32 out of reset
 * @GSA_AOC_RESET:         put AOC into reset
 */
enum gsa_aoc_cmd {
	GSA_AOC_GET_STATE = 0,
	GSA_AOC_START = 1,
	GSA_AOC_SHUTDOWN = 4,
	GSA_AOC_RELEASE_RESET = 5,
	GSA_AOC_RESET = 6,
};

/**
 * gsa_send_aoc_cmd() - execute specified AOC management command
 * @gsa: pointer to GSA &struct device
 * @cmd: &enum gsa_aoc_cmd to execute
 *
 * Return: new AOC state (&enum gsa_aoc_state) on success, negative error code
 *         otherwise.
 */
int gsa_send_aoc_cmd(struct device *gsa, enum gsa_aoc_cmd cmd);

#endif /* __LINUX_GSA_IMG_H */
