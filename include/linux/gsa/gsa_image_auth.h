/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022 Google LLC
 */
#ifndef __LINUX_GSA_IMAGE_AUTH_H
#define __LINUX_GSA_IMAGE_AUTH_H

#include <linux/device.h>
#include <linux/types.h>

/*
 * GSA Image authentication interface
 */

/**
 * gsp_authenticate_image() - authenticate a generic binary image
 * @gsa: pointer to GSA &struct device
 * @img_hdr: dma address of image meta information
 * @img_body: physical address of image body
 *
 * This routine authenticates the binary image specified by
 * @img_hdr/@img_body parameters.
 *
 * The binary image consists of two parts: a header (always 4K in size)
 * containing image meta information (including authentication parameters and
 * loading instructions) and image body which contains the image itself. The
 * image header must be loaded into memory region allocated by calling
 * dma_alloc_coherent() for GSA device. This memory chunk can be discarded
 * after gsa_authenticate_image() call is complete. Image body should
 * be loaded into physically contiguous memory region.
 *
 * Return: 0 on success, negative error otherwise
 */
int gsa_authenticate_image(struct device *gsa, dma_addr_t img_meta, phys_addr_t img_body);

#endif /* __LINUX_GSA_IMAGE_AUTH_H */
