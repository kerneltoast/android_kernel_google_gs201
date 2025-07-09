// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google LWIS Versioning File
 *
 * Copyright (c) 2022 Google, LLC
 */

#include <linux/kernel.h>
#include <linux/string.h>

#include "lwis_version.h"

#include "lwis_commands.h"

void lwis_get_feature_flags(char *buffer, size_t buffer_size)
{
	static const char long_string_format[] =
		"LWIS (Lightweight Imaging Subsystem) Copyright 2022, Google LLC\n"
		"features:"
		/*
		 * Feature flags start here:
		 */
		/*
		 * core:
		 * All features we have shipped with in our initial version of LWIS, the basic
		 * fundamental functions of LWIS.
		 */
		" core"
		/*
		 * cmd-pkt:
		 * A forward and backward compatible interface for LWIS commands. It uses a single
		 * IOCTL interface with different command packets to replace the current multiple
		 * IOCTL interfaces for different commands. It resolves the kernel version mismatch
		 * error when interface(s) change.
		 */
		" cmd-pkt"
		/*
		 * fence:
		 * Support fence feature
		 */
		" fence"
		/*
		 * transaction-info-submit:
		 * Latest supported version of transaction-info-submit
		 */
		" transaction-info-submit=%d"
		/*
		 * pdma-io:
		 * Indicating that LWIS supports PDMA writing.
		 */
		" pdma-io"
		/* End of version string */
		"\n";
	scnprintf(buffer, buffer_size, long_string_format, LWIS_CMD_ID_TRANSACTION_SUBMIT);
}
