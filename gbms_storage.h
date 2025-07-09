/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google Battery Management System
 *
 * Copyright (C) 2018 Google Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __GBMS_STORAGE_H__
#define __GBMS_STORAGE_H__

/*
 * GBMS Storage API
 * The API provides functions to access to data stored in the persistent and
 * semi-persistent storage of a device in a cross-platform and
 * location-independent fashion. Clients in kernel and userspace use this
 * directly and indirectly to retrieve battery serial number, cell chemistry
 * type, cycle bin count, battery lifetime history and other battery related
 * data.
 */

#define GBMS_STORAGE_ADDR_INVALID	-1
#define GBMS_STORAGE_INDEX_INVALID	-1

/* Battery Google Part Number */
#define GBMS_BGPN_LEN	10
/* Battery device info length */
#define GBMS_DINF_LEN	16
/* Battery manufacturer info length */
#define GBMS_MINF_LEN	30
/* Gauge Model State Restore */
#define GBMS_GMSR_LEN	23

/* TODO: link to the structure used to save this*/
#define BATT_ONE_HIST_LEN	12
#define GBMS_CCBIN_BUCKET_COUNT	10

/* Adds BPST and STRD */
#define GBMS_LOTR_DEFAULT 0xff
#define GBMS_LOTR_V1 1
#define GBMS_LOTR_V2 2

/* Date of manufacturing and first use */
#define BATT_EEPROM_TAG_XYMD_LEN 3

/* Allowing update FCR per every N cycles */
#define GBMS_FCRU_LEN	2

/*
 * Tags are u32 constants: hardcoding as hex since characters constants of more
 * than one byte such as 'BGCE' are frown upon.
 */
typedef uint32_t gbms_tag_t;

enum gbms_tags {
	GBMS_TAG_ACIM = 0x4143494d, /* Activation Impedance */
	GBMS_TAG_AYMD = 0x41594d44,
	GBMS_TAG_BCNT = 0x42434e54,
	GBMS_TAG_BGCE = 0x42474345,
	GBMS_TAG_BGPN = 0x4247504e,
	GBMS_TAG_BPST = 0x42505354, /* LOTRV1: health or spare */
	GBMS_TAG_BRES = 0x42524553,
	GBMS_TAG_BRID = 0x42524944,
	GBMS_TAG_CELC = 0x43454C43,
	GBMS_TAG_CLHI = 0x424C4849,
	GBMS_TAG_CMPC = 0x434d5043,
	GBMS_TAG_CNHS = 0x434E4853,
	GBMS_TAG_DINF = 0x44494e46,
	GBMS_TAG_DSNM = 0x44534e4d,
	GBMS_TAG_DXAC = 0x44584143,
	GBMS_TAG_FCRU = 0x46435255,
	GBMS_TAG_FGST = 0x46475354,
	GBMS_TAG_GCFE = 0x47434645,
	GBMS_TAG_GMSR = 0x474d5352,
	GBMS_TAG_HCNT = 0x48434e54,
	GBMS_TAG_HIST = 0x48495354,
	GBMS_TAG_LOTR = 0x4C4F5452,
	GBMS_TAG_MDLV = 0x4D444C56,
	GBMS_TAG_MINF = 0x4d494e46,
	GBMS_TAG_MXSN = 0x4d58534e,
	GBMS_TAG_MXCN = 0x4d58434e,
	GBMS_TAG_MYMD = 0x4d594d44,
	GBMS_TAG_THAS = 0x54484153,

	/* User Space Read/Write scratch */
	GBMS_TAG_FWHI = 0x46574849, /* Installed firmware history */
	GBMS_TAG_FWSF = 0x46575346, /* Firmware update Success Fail */
	GBMS_TAG_RS32 = 0x52533332,
	GBMS_TAG_RSBM = 0x5253424d,
	GBMS_TAG_RSBR = 0x52534252,
	GBMS_TAG_RSOC = 0x52534F43, /* save soc */
	GBMS_TAG_SUFG = 0x53554647, /* shutdown by user_request flag */

	/* Reboot scratch */
	GBMS_TAG_RRS0 = 0x52525330,
	GBMS_TAG_RRS1 = 0x52525331,
	GBMS_TAG_RRS2 = 0x52525332,
	GBMS_TAG_RRS3 = 0x52525333,
	GBMS_TAG_RRS4 = 0x52525334,
	GBMS_TAG_RRS5 = 0x52525335,
	GBMS_TAG_RRS6 = 0x52525336,
	GBMS_TAG_RRS7 = 0x52525337,

	GBMS_TAG_RAVG = 0x52415647,
	GBMS_TAG_RFCN = 0x5246434e,
	GBMS_TAG_SELC = 0x53454C43,
	GBMS_TAG_SNUM = 0x534e554d,

	GBMS_TAG_STRD = 0x53545244, /* LOTRV1: Swelling data */
};

/*
 * struct gbms_storage_desc - callbacks for a GBMS storage provider.
 *
 * Fields not used should be initialized with NULL. The provider name and the
 * iter callback are optional but strongly recommended. The write, fetch, store
 * and flush callbacks are optional, descriptors with a non NULL write/store
 * callback should have a non NULL read/fetch callback.
 *
 * The iterator callback (iter) is used to list the tags stored in the provider
 * and can be used to detect duplicates. The list of tags exported from iter
 * can be expected to be static (i.e. tags can be enumerated once on
 * registration).
 *
 * The read and write callbacks transfer the data associated with a tag. The
 * calls must return -ENOENT when called with a tag that is not known to the
 * provider, a negative number on every other error or the number of bytes
 * read or written to the device. The tag lookup for the read and write
 * callbacks must be very efficient (i.e. consider implementation that use hash
 * or switch statements).
 *
 * Fetch and store callbacks are used to grant non-mediated access to a range
 * of consecutive addresses in storage space. The implementation must return a
 * negative number on error or the number of bytes transferred with the
 * operation. Support caching of the tag data location requires non NULL fetch
 * and not NULL info callbacks.
 *
 * The read_data and write_data callbacks transfer the data associated with an
 * enumerator. The calls must return -ENOENT when called with a tag that is not
 * known to the provider, a negative number on every other error or the number
 * of bytes read or written to the device during data transfers.
 *
 * Clients can only access keys that are available on a device (i.e. clients
 * cannot create new tags) and the API returns -ENOENT when trying to access a
 * tag that is not available on a device, -EGAIN while the storage is not fully
 * initialized.
 *
 * @iter: callback, return the tags known from this provider
 * @info: callback, return address and size for tags (used for caching)
 * @read: callback, read data from a tag
 * @write: callback, write data to a tag
 * @fetch: callback, read up to count data bytes from an address
 * @store: callback, write up to count data bytes to an address
 * @flush: callback, request a fush of data to permanent storage
 * @read_data: callback, read the elements of an enumerations
 * @write_data: callback, write to the elements of an enumeration
 */
struct gbms_storage_desc {
	int (*iter)(int index, gbms_tag_t *tag, void *ptr);
	int (*info)(gbms_tag_t tag, size_t *addr, size_t *size, void *ptr);
	int (*read)(gbms_tag_t tag, void *data, size_t count, void *ptr);
	int (*write)(gbms_tag_t tag, const void *data, size_t count, void *ptr);
	int (*fetch)(void *data, size_t addr, size_t count, void *ptr);
	int (*store)(const void *data, size_t addr, size_t count, void *ptr);
	int (*flush)(bool force, void *ptr);

	int (*read_data)(gbms_tag_t tag, void *data, size_t count, int idx,
			 void *ptr);
	int (*write_data)(gbms_tag_t tag, const void *data, size_t count,
			  int idx, void *ptr);
};

struct nvmem_device;

#if IS_ENABLED(CONFIG_GOOGLE_BEE)

/* defaults */
extern int gbee_register_device(const char *name, int lotr, struct nvmem_device *nvram);
extern void gbee_destroy_device(void);

/* version 1 */
extern int gbee_storage01_info(gbms_tag_t tag, size_t *addr, size_t *count, void *ptr);
extern int gbee_storage01_iter(int index, gbms_tag_t *tag, void *ptr);

/* version 2 */
extern int gbee_storage02_info(gbms_tag_t tag, size_t *addr, size_t *count, void *ptr);
extern int gbee_storage_read_data_02(gbms_tag_t tag, void *data, size_t count, int idx, void *ptr);

/* defaults */
extern int gbee_storage_info(gbms_tag_t tag, size_t *addr, size_t *count, void *ptr);

#else

static inline int gbee_register_device(const char *name,
				       struct nvmem_device *nvram)
{ return -ENODEV; }

static inline void gbee_destroy_device(void) { }

static inline int gbee_storage02_info(gbms_tag_t tag, size_t *addr, size_t *count, void *ptr)
{ return -ENODEV; }
static inline int gbee_storage_read_data_02(gbms_tag_t tag, void *data, size_t count, int idx, void *ptr)
{ return -ENODEV; }

static inline int gbee_storage01_info(gbms_tag_t tag, size_t *addr, size_t *count, void *ptr)
{ return -ENODEV; }

static inline int gbee_storage01_iter(int index, gbms_tag_t *tag, void *ptr)
{ return -ENODEV; }

static inline int gbee_storage_info(gbms_tag_t tag, size_t *addr, size_t *count, void *ptr)
{ return -ENODEV; }

#endif

extern int gbms_storage_register(struct gbms_storage_desc *desc,
				 const char *name, void *ptr);
extern int gbms_storage_offline(const char *name, bool flush);

extern int gbms_storage_read(gbms_tag_t tag, void *data, size_t count);
extern int gbms_storage_write(gbms_tag_t tag, const void *data, size_t count);

extern int gbms_storage_read_data(gbms_tag_t tag, void *data, size_t count,
				  int idx);
extern int gbms_storage_write_data(gbms_tag_t tag, const void *data,
				   size_t count, int idx);
extern int gbms_storage_flush_all(void);

/* standard device implementation that read data from an enumeration */

struct gbms_storage_device;
extern struct gbms_storage_device *
gbms_storage_create_device(const char *name, gbms_tag_t tag);
extern void gbms_storage_cleanup_device(struct gbms_storage_device *gdev);


#endif /* __GBMS_STORAGE_H__ */
