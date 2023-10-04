/*
 * Google LWIS Versioning File
 *
 * Copyright (c) 2022 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/string.h>

#include "lwis_version.h"

void lwis_get_feature_flags(char *buffer, size_t buffer_size)
{
	scnprintf(buffer, buffer_size,
		  "LWIS (Lightweight Imaging Subsystem) Copyright 2022, Google LLC\n");

	strlcat(buffer, "features: ", buffer_size);

	/*
	 * Feature flags start here:
	 */

	/* core:
	 * All features we have shipped with in our initial version of LWIS, the basic fundamental
	 * functions of LWIS.
	 */
	strlcat(buffer, "core", buffer_size);

	strlcat(buffer, "\n", buffer_size);
}