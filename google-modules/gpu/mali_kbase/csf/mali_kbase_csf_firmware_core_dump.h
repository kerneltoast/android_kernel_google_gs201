/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2021-2022 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#ifndef _KBASE_CSF_FIRMWARE_CORE_DUMP_H_
#define _KBASE_CSF_FIRMWARE_CORE_DUMP_H_

struct kbase_device;

/** Offset of the last field of core dump entry from the image header */
#define CORE_DUMP_ENTRY_START_ADDR_OFFSET (0x4)

/* Page size in bytes in use by MCU. */
#define FW_PAGE_SIZE 4096

/**
 * struct fw_core_dump_data - Context for seq_file operations used on 'fw_core_dump'
 * debugfs file.
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 */
struct fw_core_dump_data {
	struct kbase_device *kbdev;
};

/**
 * kbase_csf_firmware_core_dump_entry_parse() - Parse a "core dump" entry from
 *                                              the image header.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @entry: Pointer to section.
 *
 * Read a "core dump" entry from the image header, check the version for
 * compatibility and store the address pointer.
 *
 * Return: 0 if successfully parse entry, negative error code otherwise.
 */
int kbase_csf_firmware_core_dump_entry_parse(struct kbase_device *kbdev, const u32 *entry);

/**
 * kbase_csf_firmware_core_dump_init() - Initialize firmware core dump support
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 *         Must be zero-initialized.
 *
 * Creates the fw_core_dump debugfs file through which to request a firmware
 * core dump. The created debugfs file is cleaned up as part of kbdev debugfs
 * cleanup.
 *
 * The fw_core_dump debugs file that case be used in the following way:
 *
 * To explicitly request core dump:
 *     echo 1 >/sys/kernel/debug/mali0/fw_core_dump
 *
 * To output current core dump (after explicitly requesting a core dump, or
 * kernel driver reported an internal firmware error):
 *     cat /sys/kernel/debug/mali0/fw_core_dump
 */
void kbase_csf_firmware_core_dump_init(struct kbase_device *const kbdev);

/**
 * get_fw_core_dump_size - Get firmware core dump size
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 *
 * Return: size on success, -1 otherwise.
 */
size_t get_fw_core_dump_size(struct kbase_device *kbdev);

/**
 * fw_core_dump_create - Requests firmware to save state for a firmware core dump
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 *
 * Return: 0 on success, error code otherwise.
 */
int fw_core_dump_create(struct kbase_device *kbdev);

/**
 * fw_core_dump_write_elf_header - Writes ELF header for the FW core dump
 * @m: the seq_file handle
 *
 * Writes the ELF header of the core dump including program headers for
 * memory sections and a note containing the current MCU register
 * values.
 *
 * Excludes memory sections without read access permissions or
 * are for protected memory.
 *
 * The data written is as follows:
 * - ELF header
 * - ELF PHDRs for memory sections
 * - ELF PHDR for program header NOTE
 * - ELF PRSTATUS note
 * - 0-bytes padding to multiple of ELF_EXEC_PAGESIZE
 *
 * The actual memory section dumps should follow this (not written
 * by this function).
 *
 * Retrieves the necessary information via the struct
 * fw_core_dump_data stored in the private member of the seq_file
 * handle.
 *
 * Return:
 * * 0		- success
 * * -ENOMEM	- not enough memory for allocating ELF32 note
 */
int fw_core_dump_write_elf_header(struct seq_file *m);

#endif /* _KBASE_CSF_FIRMWARE_CORE_DUMP_H_ */
