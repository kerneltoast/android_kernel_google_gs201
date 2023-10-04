/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * android_vendor.h - Android vendor data
 *
 * Copyright 2020 Google LLC
 *
 * These macros are to be used to reserve space in kernel data structures
 * for use by vendor modules.
 *
 * These macros should be used before the kernel abi is "frozen".
 * Fields can be added to various kernel structures that need space
 * for functionality implemented in vendor modules. The use of
 * these fields is vendor specific.
 */
#ifndef _ANDROID_VENDOR_H
#define _ANDROID_VENDOR_H

/*
 * ANDROID_VENDOR_DATA
 *   Reserve some "padding" in a structure for potential future use.
 *   This normally placed at the end of a structure.
 *   number: the "number" of the padding variable in the structure.  Start with
 *   1 and go up.
 *
 * ANDROID_VENDOR_DATA_ARRAY
 *   Same as ANDROID_VENDOR_DATA but allocates an array of u64 with
 *   the specified size
 */
#define ANDROID_VENDOR_DATA(n)
#define ANDROID_VENDOR_DATA_ARRAY(n, s)

#define ANDROID_OEM_DATA(n)
#define ANDROID_OEM_DATA_ARRAY(n, s)

#endif /* _ANDROID_VENDOR_H */
