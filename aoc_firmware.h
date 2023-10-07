// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google Whitechapel AoC Firmware loading support
 *
 * Copyright (c) 2019 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/firmware.h>

#define AOC_FIRMWARE_OFFSET_INVALID 0xffffffff

#define AOC_AUTH_HEADER_SIZE 4096

/* Dev builds bypass the UUID check on load */
bool _aoc_fw_is_release(const struct firmware *fw);

bool _aoc_fw_is_signed(const struct firmware *fw);

bool _aoc_fw_is_compatible(const struct firmware *fw);

bool _aoc_fw_is_valid(const struct firmware *fw);

u32 _aoc_fw_bootloader_offset(const struct firmware *fw);

u32 _aoc_fw_ipc_offset(const struct firmware *fw);

/* Returns firmware version, or NULL on error */
const char *_aoc_fw_version(const struct firmware *fw);

bool _aoc_fw_commit(const struct firmware *fw, void *dest);
