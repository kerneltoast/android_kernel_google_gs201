/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Google LWIS Feature Versioning File
 *
 * Copyright (c) 2022 Google, LLC
 */

#ifndef LWIS_VERSION_H_
#define LWIS_VERSION_H_

#include <linux/types.h>

/*
 * Get LWIS feature flags
 * This is where we specify the features this version of LWIS is supporting.
 *
 * Special characters:
 *   ' ' : A space is used to separate different features
 *   '=' : An equal sign is used to store key-value pairs
 *
 * Example:
 *   "features: foo bar=xyz"
 */
void lwis_get_feature_flags(char *buffer, size_t buffer_size);

#endif /* LWIS_VERSION_H_ */
