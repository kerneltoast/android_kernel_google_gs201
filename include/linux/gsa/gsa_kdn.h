/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright 2020 Google LLC
 */
#ifndef __LINUX_GSA_KDN_H
#define __LINUX_GSA_KDN_H

#include <linux/device.h>
#include <linux/types.h>

/*
 * GSA KDN interface
 *
 * KDN is a hardware block in the UFS storage controller that can be used
 * by GSA to program storage encryption keys. There are 16 individually
 * programmable key slots. The keys programmed in these slots can be used
 * to perform inline data encryption. GSA exclusively controls the sender
 * side of KDN interface and is the only entity that can manage these keys.
 */

/*
 * The UFS CMVP module requires an uninitialized key in the KDN Master Key
 * table to perform self tests with zero key. Thus the total number of KDN
 * slots is decreased by one from 16 and is 15.
 */
#define KDN_SLOT_NUM 15

/**
 * enum kdn_op_mode - KDN operating mode
 * @KDN_FMP_MODE: legacy FMP mode
 * @KDN_HW_KDF_MODE: HW key derivation mode
 * @KDN_SW_KDF_MODE: SW key derivation mode
 */
enum kdn_op_mode {
	KDN_FMP_MODE = 0,
	KDN_HW_KDF_MODE = 1,
	KDN_SW_KDF_MODE = 2,
};

/**
 * enum kdn_ufs_descr_type - UFS descriptor type
 * @KDN_UFS_DESCR_TYPE_PRDT: use PRDT descriptor
 * @KDN_UFS_DESCR_TYPE_UTRD: use UTRD descriptor
 */
enum kdn_ufs_descr_type {
	KDN_UFS_DESCR_TYPE_PRDT = 0,
	KDN_UFS_DESCR_TYPE_UTRD = 1,
};

/**
 * gsa_kdn_set_operating_mode - configure KDN operating mode
 * @gsa: pointer to GSA device
 * @mode: &enum kdn_op_mode value to select crypto engine operating mode
 * @descr: one of &enum kdn_ufs_descr_type to select UFS descriptor format
 *
 * Return: 0 on success, negative error code otherwise
 */
int gsa_kdn_set_operating_mode(struct device *gsa,
			       enum kdn_op_mode mode,
			       enum kdn_ufs_descr_type descr);

/**
 * gsa_kdn_derive_raw_secret() - derive a raw secret
 * @gsa: pointer to GSA device
 * @buf: pointer to the buffer to store the derived secret
 * @buf_sz: size of the buffer specified by @buf parameter
 * @key_blob: pointer to the buffer containing ESK wrapped KDN key blob
 * @key_blob_len: number of bytes in @key_blob buffer
 *
 * This routine derives a 256-bit value from specified ESK wrapped GSA KDN key.
 *
 * Return: number of bytes placed into @buf buffer on success or a negative
 * error code otherwise.
 */
int gsa_kdn_derive_raw_secret(struct device *gsa, void *buf, size_t buf_sz,
			      const void *key_blob, size_t key_blob_len);

/**
 * gsa_kdn_program_key() - program specified ESK wrapped GSA KDN key
 * @gsa: pointer to GSA device
 * @slot: KDN slot to program specified key into
 * @key_blob: pointer to the buffer containing ESK wrapped KDN key blob
 * @key_blob_len: number of bytes in @key_blob buffer
 *
 * This routine modifies the key in the specified KDN slot (0-15 is a valid
 * key slot range). If a new key is specified (@key_blob and @key_blob_len
 * specify a buffer containing a valid ESK wrapped GSA KDN), the new key is
 * programmed. If a new key is not specified (@key_blob is NULL and
 * @key_blob_len is 0), the previously programmed key in the slot (if any)
 * is erased. If the caller specifies an invalid key, the previously
 * programmed key remains unchanged and an error is returned.
 *
 * Return: 0 on success or a negative error code otherwise
 */
int gsa_kdn_program_key(struct device *gsa, u32 slot, const void *key_blob,
			size_t key_blob_len);


/**
 * gsa_kdn_restore_keys() - reprogram all previously programmed KDN keys
 * @gsa: pointer to GSA device
 *
 * This routine can be called to restore the KDN controller state in
 * case the storage controller loses its state due to a power collapse.
 *
 * Return: 0 on success or a negative error code otherwise
 */
int gsa_kdn_restore_keys(struct device *gsa);


#endif /* __LINUX_GSA_KDN_H */
