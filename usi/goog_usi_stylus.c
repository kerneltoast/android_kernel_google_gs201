/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Google USI Stylus Support for Pixel devices.
 *
 * Copyright 2024 Google LLC.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hid.h>
#include <linux/hidraw.h>
#include <linux/input/mt.h>
#include <linux/miscdevice.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#include "goog_usi_stylus.h"

#define G_USI_MAGIC				0x49535547 /* GUSI */

#define G_USI_INPUT_NODE_NAME			"USI_Stylus"
#define G_USI_INPUT_NODE_PHYS			"USI_Stylus/1" /* only one stylus is supported */
#define G_USI_INPUT_NODE_VERSION		0x0100
#define G_USI_BATTERY_NODE_NAME			"USI_Stylus_Battery"

#define G_USI_SN_STR_LEN			17
/* the length from USI spec */
#define G_USI_GID_LEN				12
#define G_USI_CAP_LEN				12
#define G_USI_FW_VER_LEN			2
#define G_USI_VENDOR_EXT_LEN			2
#define G_USI_HASH_ID_LEN			2
#define G_USI_SESSION_ID_LEN			2

#undef pr_fmt
#define pr_fmt(fmt)				"gtd: GUSI: " fmt
#define G_USI_ERR(fmt, args...)			pr_err("%s: " fmt, __func__, ##args)
#define G_USI_DBG(fmt, args...)			pr_debug("%s: " fmt, __func__, ##args)
#define G_USI_LOG(fmt, args...)			pr_info("%s: " fmt, __func__, ##args)

enum g_usi_op_status {
	G_USI_OP_UNPAIRED	= 0,
	G_USI_OP_PAIRED		= 1,

	/*
	 * The stylus is identified.
	 *
	 * HID Feature Reports can be supported only when the stylus is
	 * identified
	 */
	G_USI_OP_IDENTIFIED	= 2
};

struct g_usi_header {
	u32 magic;	/* magic number */
	u32 version;	/* Google USI subsystem version */
};

struct g_usi_context {
	struct g_usi_header hdr;

	struct device *dev;			/* controller device */
	struct g_usi_callbacks *cbs;
	void *drvdata;

	/* battery node */
	char *battery_name;
	struct power_supply_desc psy_desc;
	struct power_supply_config psy_cfg;
	struct power_supply *bat_psy;

	/* input node */
	struct input_dev *input_dev;
	char *name;
	char *phys;
	struct input_id id;
	int abs_x_max;
	int abs_y_max;
	int distance_max;       /* 0 means distance is not supported */
	int tilt_min;
	int tilt_max;           /* 0 means tilt is not supported */

	/* USI Stylus Information */
	u32 stylus_info_flags; /* bitmap for information availability */

	/* Information collected from a paired stylus. */
	u8 stylus_capability[G_USI_CAP_LEN];		/* C.GetCapability() */
	u8 stylus_GID[G_USI_GID_LEN];			/* C.GetGID() */
	u8 stylus_fw_ver[G_USI_FW_VER_LEN];		/* C.GetFirmwareVersion() */
	u8 stylus_usi_ver;				/* C.GetUsiVersion() */
	u8 stylus_vendor_ext[G_USI_VENDOR_EXT_LEN];	/* C.GetVendorExtension() */
	u8 stylus_battery;				/* C.GetBattery() */

	/* Pairing information which can be used to restore the controller during resume process */
	struct g_usi_pairing_info pairing_info;

	/* 64-bit serial number */
	u32 stylus_sn_low;
	u32 stylus_sn_high;

	/*
	 * When a user asks a diagnose, there should be both HID SET DIAGNOSE and HID GET DIAGNOSE.
	 * During the HID SET DIAGNOSE, we don't actually send the beacon command. Instead,
	 * we keep the command. Later, when there's HID GET DIAGNOSE request, we send the beacon
	 * command and get the response
	 */
	bool is_diagnose_requested;	/* set true on SET request and set false on GET request */
	u8 diagnose_uplink[5];

	bool create_new_input_dev_flag; /*
					 * If the vid/pid of the current stylus is different from
					 * the previous one. We notify that by creating new input
					 * event node with the new vid/pid.
					 */
	bool recreate_evdev_enabled;	/* recreate input event node is enabled ? */

	bool is_flex_beacon;		/* true: flex beacon, false: standard beacon */
	enum g_usi_op_status status;	/* pen operation status */
	struct miscdevice g_usi_hidraw_dev; /* device for users to do HID SET/GET feature report */

	struct mutex lock;		/* usi context lock */
};

#define handle_to_context(_handle)		((struct g_usi_context *)_handle)
#define context_to_handle(_ctx)			((g_usi_handle_t)_ctx)

/* check if the usi_ctx is valid or not */
static inline bool g_usi_is_valid_context(struct g_usi_context *usi_ctx)
{
	if (!usi_ctx)
		return false;

	if (usi_ctx->hdr.magic != G_USI_MAGIC)
		return false;

	return true;
}

static inline void set_gid_available(struct g_usi_context *usi_ctx)
{
	usi_ctx->stylus_info_flags |= G_USI_GID_FLAG;
}

static inline bool is_gid_available(const struct g_usi_context *usi_ctx)
{
	return usi_ctx->stylus_info_flags & G_USI_GID_FLAG;
}

static inline void set_battery_available(struct g_usi_context *usi_ctx)
{
	usi_ctx->stylus_info_flags |= G_USI_BATTERY_FLAG;
}

static inline bool is_battery_available(const struct g_usi_context *usi_ctx)
{
	return usi_ctx->stylus_info_flags & G_USI_BATTERY_FLAG;
}

static inline void set_capability_available(struct g_usi_context *usi_ctx)
{
	usi_ctx->stylus_info_flags |= G_USI_CAPABILITY_FLAG;
}

static inline bool is_capability_available(const struct g_usi_context *usi_ctx)
{
	return usi_ctx->stylus_info_flags & G_USI_CAPABILITY_FLAG;
}

static inline bool is_preferred_color_read_only(const struct g_usi_context *usi_ctx)
{
	return !!(usi_ctx->stylus_capability[0] & 0x20); /* preferred color is RO ? */
}

static inline bool is_preferred_width_read_only(const struct g_usi_context *usi_ctx)
{
	return !!(usi_ctx->stylus_capability[0] & 0x80); /* preferred width is RO ? */
}

static inline bool is_preferred_style_read_only(const struct g_usi_context *usi_ctx)
{
	return !!(usi_ctx->stylus_capability[0] & 0x40); /* preferred style is RO ? */
}

static inline bool is_true_color_supported(const struct g_usi_context *usi_ctx)
{
	return !!(usi_ctx->stylus_capability[5] & 0x04); /* support true color ? */
}

static inline void set_fw_version_available(struct g_usi_context *usi_ctx)
{
	usi_ctx->stylus_info_flags |= G_USI_FW_VERSION_FLAG;
}

static inline bool is_fw_version_available(const struct g_usi_context *usi_ctx)
{
	return usi_ctx->stylus_info_flags & G_USI_FW_VERSION_FLAG;
}

static inline void set_usi_version_available(struct g_usi_context *usi_ctx)
{
	usi_ctx->stylus_info_flags |= G_USI_USI_VERSION_FLAG;
}

static inline bool is_usi_version_available(const struct g_usi_context *usi_ctx)
{
	return usi_ctx->stylus_info_flags & G_USI_USI_VERSION_FLAG;
}

static inline void set_vendor_ext_available(struct g_usi_context *usi_ctx)
{
	usi_ctx->stylus_info_flags |= G_USI_VENDOR_EXT_FLAG;
}

static inline bool is_vendor_ext_available(const struct g_usi_context *usi_ctx)
{
	return usi_ctx->stylus_info_flags & G_USI_VENDOR_EXT_FLAG;
}

static inline void set_pairing_info_available(struct g_usi_context *usi_ctx)
{
	usi_ctx->stylus_info_flags |= G_USI_PAIRING_INFO_FLAG;
}

static inline bool is_pairing_info_available(const struct g_usi_context *usi_ctx)
{
	return usi_ctx->stylus_info_flags & G_USI_PAIRING_INFO_FLAG;
}

static inline void clear_stylus_info_flags(struct g_usi_context *usi_ctx)
{
	usi_ctx->stylus_info_flags = 0; /* clear stylus info for the new stylus */
}

/**
 * goog_usi_report_gid() - reports GID to the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @gid: 12 bytes GID from the stylus via C.GetGID()
 *
 * The USI controller must collect the GID of the paired stylus and reports
 * it to the host. The host(vendor driver) should call this function to update
 * the GID maintined in the subsystem.
 *
 * Return: 0 on success, non-zero on failure
 */
int goog_usi_report_gid(g_usi_handle_t handle, const u8 *gid)
{
	u32 sn_high, sn_low;
	struct g_usi_context *usi_ctx = handle_to_context(handle);
	u16 vid, pid;

	if (!gid || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	memcpy(usi_ctx->stylus_GID, gid, sizeof(usi_ctx->stylus_GID));
	sn_low = gid[0] | gid[1] << 8 | gid[2] << 16 | gid[3] << 24;
	sn_high = gid[4] | gid[5] << 8 | gid[6] << 16 | gid[7] << 24;
	if (usi_ctx->stylus_sn_low != sn_low || usi_ctx->stylus_sn_high != sn_high) {
		usi_ctx->stylus_sn_low = sn_low;
		usi_ctx->stylus_sn_high = sn_high;

		/* notify serial number updated */
		power_supply_changed(usi_ctx->bat_psy);
	}

	vid = gid[8] | gid[9] << 8;
	pid = gid[10] | gid[11] << 8;
	if (usi_ctx->id.vendor != vid || usi_ctx->id.product != pid) { /* is new stylus ? */
		G_USI_LOG("Old(vid: 0x%04x, pid: 0x%04x) -> New(vid: 0x%04x, pid: 0x%04x)",
			  usi_ctx->id.vendor, usi_ctx->id.product, vid, pid);

		usi_ctx->id.vendor = vid;
		usi_ctx->id.product = pid;

		/* create new input node when the new stylus is unpaired */
		if (usi_ctx->recreate_evdev_enabled)
			usi_ctx->create_new_input_dev_flag = true;
	}

	usi_ctx->status = G_USI_OP_IDENTIFIED; /* Parsing GID is done. The stylus is identified */
	set_gid_available(usi_ctx);

	mutex_unlock(&usi_ctx->lock);

	return 0;
}
EXPORT_SYMBOL(goog_usi_report_gid);

/**
 * goog_usi_report_fw_version() - reports FW version to the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @fw_ver: 2 bytes FW version from the stylus via C.GetFirmwareVersion()
 *
 * The USI controller must collect the FW version of the paired stylus and reports
 * it to the host. The host(vendor driver) should call this function to update
 * the FW version maintined in the subsystem.
 *
 * Return: 0 on success, non-zero on failure
 */
int goog_usi_report_fw_version(g_usi_handle_t handle, const u8 *fw_ver)
{
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!fw_ver || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	memcpy(usi_ctx->stylus_fw_ver, fw_ver, sizeof(usi_ctx->stylus_fw_ver));
	set_fw_version_available(usi_ctx);

	mutex_unlock(&usi_ctx->lock);

	return 0;
}
EXPORT_SYMBOL(goog_usi_report_fw_version);

/**
 * goog_usi_get_fw_version() - Get the FW version stored in the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @fw_ver: 2 bytes FW version(Major, Minor)
 *
 * Return: 0 on success and @fw_ver has the major and minor version, non-zero on failure
 */
int goog_usi_get_fw_version(g_usi_handle_t handle, u8 *fw_ver)
{
	int ret = 0;
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!fw_ver || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	if (!is_fw_version_available(usi_ctx)) {
		ret = -ENODATA;
		goto error_ret;
	}

	memcpy(fw_ver, usi_ctx->stylus_fw_ver, sizeof(usi_ctx->stylus_fw_ver));
error_ret:
	mutex_unlock(&usi_ctx->lock);
	return ret;
}
EXPORT_SYMBOL(goog_usi_get_fw_version);

/**
 * goog_usi_report_usi_version() - report USI version to the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @usi_ver: 8 bits USI version(4bits Major, 4bits Minor) from the stylus via C.GetUsiVersion()
 *
 * The USI controller may collect the USI version of the paired stylus and reports
 * it to the host. The host(vendor driver) should call this function to update
 * the USI version maintined in the subsystem.
 *
 * Note: The USI controller may not collect the USI version from the stylus. In that case,
 * still the vendor driver should report the usi version which may be hard-coded.
 *
 * Return: 0 on success, non-zero on failure
 */
int goog_usi_report_usi_version(g_usi_handle_t handle, const u8 *usi_ver)
{
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!usi_ver || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameter");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	usi_ctx->stylus_usi_ver = *usi_ver;
	set_usi_version_available(usi_ctx);

	mutex_unlock(&usi_ctx->lock);

	return 0;
}
EXPORT_SYMBOL(goog_usi_report_usi_version);

/**
 * goog_usi_get_usi_version() - Get the USI version stored in the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @usi_ver: 8 bits USI version(4bits Major, 4bits Minor)
 *
 * Return: 0 on success and @usi_ver has the USI version, non-zero on failure
 */
int goog_usi_get_usi_version(g_usi_handle_t handle, u8 *usi_ver)
{
	int ret = 0;
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!usi_ver || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	if (!is_usi_version_available(usi_ctx)) {
		ret = -ENODATA;
		goto error_ret;
	}

	*usi_ver = usi_ctx->stylus_usi_ver;
error_ret:
	mutex_unlock(&usi_ctx->lock);
	return ret;
}
EXPORT_SYMBOL(goog_usi_get_usi_version);

/**
 * goog_usi_report_vendor_extension() - reports vendor_extension to the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @fw_version: 2 bytes vendor extension from the stylus via C.GetVendorExtension()
 *
 * The USI controller must retrieve the FW version of the paired stylus and reports
 * it to the host. The host(vendor driver) should call this function to update
 * the FW version maintined in the subsystem.
 *
 * Return: 0 on success, non-zero on failure
 */
int goog_usi_report_vendor_extension(g_usi_handle_t handle, const u8 *vendor_ext)
{
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!vendor_ext || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);
	memcpy(usi_ctx->stylus_vendor_ext, vendor_ext, sizeof(usi_ctx->stylus_vendor_ext));
	set_vendor_ext_available(usi_ctx);
	mutex_unlock(&usi_ctx->lock);

	return 0;
}
EXPORT_SYMBOL(goog_usi_report_vendor_extension);

/**
 * goog_usi_get_vendor_extension() - Get the vendor extension stored in the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @vendor_ext: 2 bytes vendor extension
 *
 *
 * Return: 0 on success and @vendor_Ext has the vendor extension, non-zero on failure
 */
int goog_usi_get_vendor_extension(g_usi_handle_t handle, u8 *vendor_ext)
{
	int ret = 0;
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!vendor_ext || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	if (!is_vendor_ext_available(usi_ctx)) {
		ret = -ENODATA;
		goto error_ret;
	}

	memcpy(vendor_ext, usi_ctx->stylus_vendor_ext, sizeof(usi_ctx->stylus_vendor_ext));
error_ret:
	mutex_unlock(&usi_ctx->lock);
	return ret;
}
EXPORT_SYMBOL(goog_usi_get_vendor_extension);

/**
 * goog_usi_report_capability() - reports capability to the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @gid: 12 bytes capability from the stylus via C.GetCapability()
 *
 * The USI controller must collect the capability of the paired stylus and reports
 * it to the host. The host(vendor driver) should call this function to update
 * the capability maintined in the subsystem.
 *
 * Return: 0 on success, non-zero on failure
 */
int goog_usi_report_capability(g_usi_handle_t handle, const u8 *cap)
{
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!cap || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	memcpy(usi_ctx->stylus_capability, cap, sizeof(usi_ctx->stylus_capability));
	set_capability_available(usi_ctx);

	mutex_unlock(&usi_ctx->lock);

	return 0;
}
EXPORT_SYMBOL(goog_usi_report_capability);

/**
 * goog_usi_set_fw_version() - reports the battery level to the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @bat: 1 bytes battery level from the stylus via C.GetBattery()
 *
 * The USI controller must retrieve the battery level of the paired stylus and reports
 * it to the host. The host(vendor driver) should call this function to update
 * the battery level maintined in the subsystem.
 *
 * Return: 0 on success, non-zero on failure
 */
int goog_usi_report_battery(g_usi_handle_t handle, const u8 *bat)
{
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!bat || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	if (usi_ctx->stylus_battery != bat[0]) {
		usi_ctx->stylus_battery = bat[0];

		/* notify new battery level received */
		power_supply_changed(usi_ctx->bat_psy);
	}
	set_battery_available(usi_ctx);

	mutex_unlock(&usi_ctx->lock);

	return 0;
}
EXPORT_SYMBOL(goog_usi_report_battery);

/**
 * goog_usi_get_battery() - Get the battery level stored in the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @bat: 1 byte battery level
 *
 *
 * Return: 0 on success and @bat has the battery level, non-zero on failure
 */
int goog_usi_get_battery(g_usi_handle_t handle, u8 *bat)
{
	int ret = 0;
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!bat || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	if (!is_battery_available(usi_ctx)) {
		ret = -ENODATA;
		goto error_ret;
	}

	*bat = usi_ctx->stylus_battery;

error_ret:
	mutex_unlock(&usi_ctx->lock);
	return ret;
}
EXPORT_SYMBOL(goog_usi_get_battery);

/**
 * goog_usi_get_serial_number() - Get the serial number stored in the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @sn_high: upper 16 bits of the serial number
 * @sn_low: lower 16 bits of the serial number
 *
 * Return: 0 on success and @sn_high and @sn_low has the 64 bits serial number, non-zero on failure
 */
int goog_usi_get_serial_number(g_usi_handle_t handle, u32 *sn_high, u32 *sn_low)
{
	int ret = 0;
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if ((!sn_high && !sn_low) || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	if (!is_gid_available(usi_ctx)) {
		ret = -ENODATA;
		goto error_ret;
	}

	if (sn_high)
		*sn_high = usi_ctx->stylus_sn_high;
	if (sn_low)
		*sn_low = usi_ctx->stylus_sn_low;

error_ret:
	mutex_unlock(&usi_ctx->lock);
	return ret;
}
EXPORT_SYMBOL(goog_usi_get_serial_number);

/**
 * goog_usi_get_serial_number_str() - Get the serial number in string stored in the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @sn_high: upper 16 bits of the serial number
 * @sn_low: lower 16 bits of the serial number
 *
 * Return: 0 on success and @sn_high and @sn_low has the 64 bits serial number, non-zero on failure
 */
int goog_usi_get_serial_number_str(g_usi_handle_t handle, char *buf, int buf_size)
{
	int ret = 0;
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!buf || buf_size < G_USI_SN_STR_LEN || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	if (!is_gid_available(usi_ctx)) {
		ret = -ENODATA;
		goto error_ret;
	}

	sprintf(buf, "%08X%08X", usi_ctx->stylus_sn_high, usi_ctx->stylus_sn_low);

error_ret:
	mutex_unlock(&usi_ctx->lock);
	return ret;
}
EXPORT_SYMBOL(goog_usi_get_serial_number_str);

/**
 * goog_usi_get_vid_pid() - Get the vendor ID and product ID stored in the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @vid: 16 bits vendor ID
 * @pid: 16 bits product ID
 *
 * Return: 0 on success and @vid has a vendor ID and @pid has a product ID, non-zero on failure
 */
int goog_usi_get_vid_pid(g_usi_handle_t handle, u16 *vid, u16 *pid)
{
	int ret = 0;
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if ((!vid && !pid) || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	if (!is_gid_available(usi_ctx)) {
		ret = -ENODATA;
		goto error_ret;
	}

	if (vid)
		*vid = usi_ctx->stylus_GID[8] | usi_ctx->stylus_GID[9] << 8;
	if (pid)
		*pid = usi_ctx->stylus_GID[10] | usi_ctx->stylus_GID[11] << 8;

error_ret:
	mutex_unlock(&usi_ctx->lock);
	return ret;
}
EXPORT_SYMBOL(goog_usi_get_vid_pid);

/**
 * goog_usi_update_status() - update the status of the stylus in use
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @type: &enum G_USI_STATUS
 * @data: additional data to support the specified @type
 *
 * Return: 0 on success, non-zero on failure
 */
int goog_usi_update_status(g_usi_handle_t handle, int type, void *data)
{
	struct g_usi_context *usi_ctx = handle_to_context(handle);
	struct g_usi_pairing_info *pairing_info = NULL;
	int ret = 0;

	if (!g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	switch (type) {
	case G_USI_UPDATE_NORMAL_PAIRING_DONE:
		if (!data) {
			ret = -EINVAL;
			break;
		}

		/*
		 * Sometimes, even for the stylus recently paired, there can be normal pairing
		 * instead of fast pairing. In this case, the driver needs to compare the HASH ID
		 * to check if it's the recently paired stylus. If it is then
		 * the driver re-use the existing information.
		 */
		pairing_info = (struct g_usi_pairing_info *)data;
		if (usi_ctx->pairing_info.hash_id[0] != pairing_info->hash_id[0] ||
		    usi_ctx->pairing_info.hash_id[1] != pairing_info->hash_id[1]) {
			clear_stylus_info_flags(usi_ctx);/* clear stylus info for the new stylus */

			/* save pairing information for the new stylus */
			memcpy(&usi_ctx->pairing_info, pairing_info, sizeof(*pairing_info));

			set_pairing_info_available(usi_ctx);
			G_USI_LOG("Normal Pairing: new pen");
		} else if (is_gid_available(usi_ctx)) {
			G_USI_LOG("Normal Pairing: same pen");
			usi_ctx->status = G_USI_OP_IDENTIFIED;
		}

		break;
	case G_USI_UPDATE_FAST_PAIRING_DONE:
		G_USI_LOG("Fast Pairing : 0x%08x", usi_ctx->stylus_info_flags);
		if (is_gid_available(usi_ctx))
			usi_ctx->status = G_USI_OP_IDENTIFIED;
		break;
	case G_USI_UPDATE_CRC_FAILURE:
		break;
	}

	mutex_unlock(&usi_ctx->lock);
	return 0;
}
EXPORT_SYMBOL(goog_usi_update_status);

/**
 * goog_usi_get_pairing_info() - get pairing information stored in the subsystem
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @pairing_info: &struct g_usi_pairing_info
 *
 * Return: 0 on success, non-zero on failure
 */
int goog_usi_get_pairing_info(g_usi_handle_t handle, struct g_usi_pairing_info *pairing_info)
{
	int ret = 0;
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!pairing_info || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	if (!is_pairing_info_available(usi_ctx)) {
		ret = -ENODATA;
		goto error_ret;
	}

	memcpy(pairing_info, &usi_ctx->pairing_info, sizeof(*pairing_info));

error_ret:
	mutex_unlock(&usi_ctx->lock);
	return ret;
}
EXPORT_SYMBOL(goog_usi_get_pairing_info);

/* USI Battery Information is exposed via Power Supply Subsystem */
static enum power_supply_property stylus_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_SCOPE,
};

static int stylus_get_battery_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	int low_bat_flag, capacity;
	struct g_usi_context *usi_ctx = (struct g_usi_context *)power_supply_get_drvdata(psy);
	u8 battery = 0;
	static u8 battery_sn_str[G_USI_SN_STR_LEN]; /* 64bit(16 nibbles) serial number */

	if (goog_usi_get_battery(context_to_handle(usi_ctx), &battery))
		return -EINVAL;

	low_bat_flag = battery & 0x80;
	capacity = battery & 0x7f;

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		/* stylus doesn't support fuel gauge */
		if (capacity == 127) {
			if (low_bat_flag) /* low battery flag */
				val->intval = 1;
			else /* no low battery flag */
				val->intval = 100;
		} else { /* 1 ~ 100 */
			val->intval = capacity;
		}
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		/* battery capacity is never updated */
		if (capacity == 0) {
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
			break;
		}

		if (low_bat_flag)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else if (capacity != 100) /* 127, 1 ~ 99 */
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		else /* 100 */
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;

		break;

	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		/* the latest serial number */
		if (goog_usi_get_serial_number_str(context_to_handle(usi_ctx),
						   battery_sn_str, sizeof(battery_sn_str))) {
			return -ENODATA;
		}
		val->strval = battery_sn_str;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;

	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static struct input_dev *g_usi_register_input(struct g_usi_context *usi_ctx)
{
	struct input_dev *in_dev;
	int ret = -1;

	in_dev = input_allocate_device();
	if (!in_dev)
		return NULL;

	in_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	in_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	in_dev->keybit[BIT_WORD(BTN_TOOL_PEN)] |= BIT_MASK(BTN_TOOL_PEN);
	in_dev->keybit[BIT_WORD(BTN_TOOL_RUBBER)] |= BIT_MASK(BTN_TOOL_RUBBER);
	in_dev->keybit[BIT_WORD(BTN_STYLUS)] |= BIT_MASK(BTN_STYLUS);
	in_dev->keybit[BIT_WORD(BTN_STYLUS2)] |= BIT_MASK(BTN_STYLUS2);
	in_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

	input_set_abs_params(in_dev, ABS_X, 0, usi_ctx->abs_x_max, 0, 0);
	input_set_abs_params(in_dev, ABS_Y, 0, usi_ctx->abs_y_max, 0, 0);
	input_set_abs_params(in_dev, ABS_PRESSURE, 0, 4095, 0, 0); /* USI spec */
	if (usi_ctx->distance_max)	/* if the controller supports distance */
		input_set_abs_params(in_dev, ABS_DISTANCE, 0, usi_ctx->distance_max, 0, 0);
	if (usi_ctx->tilt_max) {	/* if the controller supports tilt */
		input_set_abs_params(in_dev, ABS_TILT_X,
				     usi_ctx->tilt_min, usi_ctx->tilt_max, 0, 0);
		input_set_abs_params(in_dev, ABS_TILT_Y,
				     usi_ctx->tilt_min, usi_ctx->tilt_max, 0, 0);
	}

	__set_bit(EV_MSC, in_dev->evbit);
	__set_bit(MSC_SERIAL, in_dev->mscbit);

	in_dev->name = usi_ctx->name;
	in_dev->uniq = usi_ctx->name;
	in_dev->phys = usi_ctx->phys;
	in_dev->dev.parent = usi_ctx->dev;
	memcpy(&in_dev->id, &usi_ctx->id, sizeof(in_dev->id));

	ret = input_register_device(in_dev);
	if (ret) {
		goto input_register_device_error;
		return NULL;
	}

	return in_dev;

input_register_device_error:
	input_free_device(in_dev);
	return NULL;
}

static void g_usi_unregister_input(struct input_dev *in_dev)
{
	input_unregister_device(in_dev);
}

/**
 * goog_usi_get_stylus_info_flags() - Get the bitmap of the available stylus information
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @info_flags: bitmap of &enum G_USI_STYLUS_INFO. each bit represents if the subsysetm
 *            maintains the corresponding stylus information
 *
 * Return: 0 on success and @info_flags has the bitmap of available stylus information,
 *         non-zero on failure
 */
int goog_usi_get_stylus_info_flags(g_usi_handle_t handle, u32 *info_flags)
{
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!info_flags || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	*info_flags = usi_ctx->stylus_info_flags;

	return 0;
}
EXPORT_SYMBOL(goog_usi_get_stylus_info_flags);

/**
 * goog_usi_send_event() - send a USI motion event
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @usi_event: the &struct g_usi_event contains a motion data
 *
 * Resturn: 0 on success, non-zero on error
 */
int goog_usi_send_event(g_usi_handle_t handle, struct g_usi_event *usi_event)
{
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!usi_event || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	input_set_timestamp(usi_ctx->input_dev, usi_event->timestamp);
	input_report_abs(usi_ctx->input_dev, ABS_X, usi_event->x);
	input_report_abs(usi_ctx->input_dev, ABS_Y, usi_event->y);
	input_report_abs(usi_ctx->input_dev, ABS_PRESSURE, usi_event->pressure);
	input_report_key(usi_ctx->input_dev, BTN_TOUCH, !!usi_event->pressure);
	if (usi_ctx->tilt_max) {
		input_report_abs(usi_ctx->input_dev, ABS_TILT_X, usi_event->tilt_x);
		input_report_abs(usi_ctx->input_dev, ABS_TILT_Y, usi_event->tilt_y);
	}
	if (usi_ctx->distance_max)
		input_report_abs(usi_ctx->input_dev, ABS_DISTANCE, usi_event->distance);
	input_report_key(usi_ctx->input_dev, BTN_TOOL_PEN, 1);
	input_report_key(usi_ctx->input_dev, BTN_STYLUS, usi_event->buttons & 1);
	input_report_key(usi_ctx->input_dev, BTN_STYLUS2, (usi_event->buttons >> 1) & 1);
	input_report_key(usi_ctx->input_dev, BTN_TOOL_RUBBER, (usi_event->buttons >> 2) & 1);

	/*
	 * Input Subsystem doesn't support 64bits serial number.
	 * So we only reports the lower 32bit.
	 */
	if (is_gid_available(usi_ctx))
		input_event(usi_ctx->input_dev, EV_MSC, MSC_SERIAL, usi_ctx->stylus_sn_low);

	if (usi_ctx->status == G_USI_OP_UNPAIRED) {
		G_USI_LOG("USI Paired");
		usi_ctx->status = G_USI_OP_PAIRED;
	}

	input_sync(usi_ctx->input_dev);

	mutex_unlock(&usi_ctx->lock);

	return 0;
}
EXPORT_SYMBOL(goog_usi_send_event);

/**
 * goog_usi_send_event_leave() - send a USI motion leave event
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @usi_event_leave: the &struct g_usi_event_leave contains a motion data
 *
 * The vendor driver calls when the controller reports stylus is unpaired
 *
 * Resturn: 0 on success, non-zero on error
 */
int goog_usi_send_event_leave(g_usi_handle_t handle, struct g_usi_event_leave *usi_event_leave)
{
	struct g_usi_context *usi_ctx = handle_to_context(handle);
	struct input_dev *new_input_dev = NULL;

	if (!usi_event_leave || !g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	mutex_lock(&usi_ctx->lock);

	input_set_timestamp(usi_ctx->input_dev, usi_event_leave->timestamp);
	input_report_abs(usi_ctx->input_dev, ABS_PRESSURE, 0);
	input_report_key(usi_ctx->input_dev, BTN_TOUCH, 0);
	input_report_key(usi_ctx->input_dev, BTN_TOOL_PEN, 0);
	input_report_key(usi_ctx->input_dev, BTN_STYLUS, 0);
	input_report_key(usi_ctx->input_dev, BTN_STYLUS2, 0);
	input_report_key(usi_ctx->input_dev, BTN_TOOL_RUBBER, 0);

	/*
	 * Input Subsystem doesn't support 64bits serial number.
	 * So we only reports the lower 32bit.
	 */
	if (is_gid_available(usi_ctx))
		input_event(usi_ctx->input_dev, EV_MSC, MSC_SERIAL, usi_ctx->stylus_sn_low);

	usi_ctx->status = G_USI_OP_UNPAIRED;
	G_USI_LOG("USI Unpaired");

	input_sync(usi_ctx->input_dev);

	/* we got GIDs with new VID/PID pair */
	if (usi_ctx->create_new_input_dev_flag) {
		/* create new input device with the new VID/PID pair */
		new_input_dev = g_usi_register_input(usi_ctx);
		if (new_input_dev) {
			g_usi_unregister_input(usi_ctx->input_dev);
			usi_ctx->input_dev = new_input_dev;
		}

		usi_ctx->create_new_input_dev_flag = false;
	}

	mutex_unlock(&usi_ctx->lock);

	return 0;
}
EXPORT_SYMBOL(goog_usi_send_event_leave);

/**
 * goog_usi_set_drvdata() - set the vendor-specific data
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 * @data: pointer to vendor-specific data
 *
 * Return: 0 on success, non-zere on failure
 */
int goog_usi_set_drvdata(g_usi_handle_t handle, void *data)
{
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	usi_ctx->drvdata = data;

	return 0;
}
EXPORT_SYMBOL(goog_usi_set_drvdata);

/**
 * goog_usi_set_drvdata() - get the vendor-specific data
 *
 * @handle: handle(&typedef g_usi_handle_t) to the registered USI controller
 *
 * Return: Return 0 on success and @data contains the value, Non-zero on failure
 */
int goog_usi_get_drvdata(g_usi_handle_t handle, void **data)
{
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return -EINVAL;
	}

	*data = usi_ctx->drvdata;

	return 0;
}
EXPORT_SYMBOL(goog_usi_get_drvdata);

/*
 * Here, we only support HID GET/SET Feature Report via ioctl interface.
 *
 * We don't use HID core subsystem since the subsystem will automatically create
 * input devices based on HID Input Descriptor.
 */
/*
 * The following HID Report Descriptor is copied from
 * USIv2-HID-Report-Descriptor.h from universalstylus.org.
 *
 * Only GET/SET Feature Report are used. Input Report is NOT used.
 */
#define PHYSICAL_WIDTH				23585
#define LOGICAL_WIDTH				3200
#define PHYSICAL_HEIGHT				14740
#define LOGICAL_HEIGHT				5120

#define MAX_SUPPORTED_STYLI			1

/* Change these to your preferred values. */
#define	HID_REPORTID_TABLET			8
#define	HID_REPORTID_ERROR			10
#define	HID_REPORTID_GETSET_COLOR8		11
#define	HID_REPORTID_GETSET_WIDTH		12
#define	HID_REPORTID_GETSET_STYLE		13
#define	HID_REPORTID_DIAGNOSE			14
#define	HID_REPORTID_GETSET_BUTTONS		15
#define	HID_REPORTID_GET_FIRMWARE		16
#define	HID_REPORTID_GET_PROTOCOL		17
#define	HID_REPORTID_GETSET_VENDOR		18
#define	HID_REPORTID_SET_TRANSDUCER		19
#define	HID_REPORTID_GETSET_COLOR24		20

/* Convenience defines */
#define	TABLET_TIP				(1 << 0)
#define	TABLET_BARREL				(1 << 1)
#define	TABLET_SECONDARYBARREL			(1 << 2)
#define	TABLET_INVERT				(1 << 3)
#define	TABLET_ERASER				(1 << 4)
#define	TABLET_INRANGE				(1 << 5)

#define LOW_BYTE(x)				((x) & 0xFF)
#define HIGH_BYTE(x)				(((x) >> 8) & 0xFF)

/* 7.4 HID Descriptor for a Data Report */
static u8 USI_report_descriptor_v2_0[] = {
    0x05, 0x0d,                    // USAGE_PAGE (Digitizers)
    0x09, 0x02,                    // USAGE (Pen)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x20,                    //   USAGE (Stylus)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x85, HID_REPORTID_TABLET,     //     REPORT_ID (HID_REPORTID_TABLET)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0xa4,                          //     PUSH
    0x09, 0x30,                    //     USAGE (X)
    0x35, 0x00,                    //     PHYSICAL_MINIMUM (0)
    0x47, LOW_BYTE(PHYSICAL_WIDTH), HIGH_BYTE(PHYSICAL_WIDTH), 0x00, 0x00,  // PHYSICAL_MAXIMUM (PHYSICAL_WIDTH)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x27, LOW_BYTE(LOGICAL_WIDTH), HIGH_BYTE(LOGICAL_WIDTH), 0x00, 0x00,    // LOGICAL_MAXIMUM (LOGICAL_WIDTH)
    0x55, 0x0d,                    //     UNIT_EXPONENT (-3)
    0x65, 0x11,                    //     UNIT (Centimeter,SILinear)
    0x75, 0x10,                    //     REPORT_SIZE (16)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x09, 0x31,                    //     USAGE (Y)
    0x47, LOW_BYTE(PHYSICAL_HEIGHT), HIGH_BYTE(PHYSICAL_HEIGHT), 0x00, 0x00, // PHYSICAL_MAXIMUM (PHYSICAL_HEIGT)
    0x27, LOW_BYTE(LOGICAL_HEIGHT), HIGH_BYTE(LOGICAL_HEIGHT), 0x00, 0x00,   // LOGICAL_MAXIMUM (LOGICAL_HEIGT)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xb4,                          //     POP
    0x05, 0x0d,                    //     USAGE_PAGE (Digitizers)
    0x09, 0x38,                    //     USAGE (Transducer Index)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, MAX_SUPPORTED_STYLI,     //     LOGICAL_MAXIMUM (MAX_SUPPORTED_STYLI)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x09, 0x30,                    //     USAGE (Tip Pressure)
    0x75, 0x10,                    //     REPORT_SIZE (16)
    0x26, 0xff, 0x0f,              //     LOGICAL_MAXIMUM (4095)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x09, 0x31,                    //     USAGE (Barrel Pressure)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x09, 0x42,                    //     USAGE (Tip Switch)
    0x09, 0x44,                    //     USAGE (Barrel Switch)
    0x09, 0x5a,                    //     USAGE (Secondary Barrel Switch)
    0x09, 0x3c,                    //     USAGE (Invert)
    0x09, 0x45,                    //     USAGE (Eraser)
    0x09, 0x32,                    //     USAGE (In Range)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x95, 0x06,                    //     REPORT_COUNT (6)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x09, 0x3d,                    //     USAGE (X Tilt)
    0x55, 0x0e,                    //     UNIT_EXPONENT (-2)
    0x65, 0x14,                    //     UNIT (Eng Rot:Angular Pos)
    0x36, 0xd8, 0xdc,              //     PHYSICAL_MINIMUM (-9000)
    0x46, 0x28, 0x23,              //     PHYSICAL_MAXIMUM (9000)
    0x16, 0xd8, 0xdc,              //     LOGICAL_MINIMUM (-9000)
    0x26, 0x28, 0x23,              //     LOGICAL_MAXIMUM (9000)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x10,                    //     REPORT_SIZE (16)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x09, 0x3e,                    //     USAGE (Y Tilt)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x09, 0x41,                    //     USAGE (Twist)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x27, 0xa0, 0x8c, 0x00, 0x00,  //     LOGICAL_MAXIMUM (36000)
    0x35, 0x00,                    //     PHYSICAL_MINIMUM (0)
    0x47, 0xa0, 0x8c, 0x00, 0x00,  //     PHYSICAL_MAXIMUM (36000)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x20,                    //     USAGE_PAGE (Sensors)
    0x0a, 0x53, 0x04,              //     USAGE (Data Field: Acceleration Axis X)
    0x65, 0x00,                    //     UNIT (None)
    0x16, 0x01, 0xf8,              //     LOGICAL_MINIMUM (-2047)
    0x26, 0xff, 0x07,              //     LOGICAL_MAXIMUM (2047)
    0x75, 0x10,                    //     REPORT_SIZE (16)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x0a, 0x54, 0x04,              //     USAGE (Data Field: Acceleration Axis Y)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x0a, 0x55, 0x04,              //     USAGE (Data Field: Acceleration Axis Z)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x0a, 0x57, 0x04,              //     USAGE (Data Field: Angular Velocity Axis X)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x0a, 0x58, 0x04,              //     USAGE (Data Field: Angular Velocity Axis Y)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x0a, 0x59, 0x04,              //     USAGE (Data Field: Angular Velocity Axis Z)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x0a, 0x72, 0x04,              //     USAGE (Data Field: Heading X Axis)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x0a, 0x73, 0x04,              //     USAGE (Data Field: Heading Y Axis)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x0a, 0x74, 0x04,              //     USAGE (Data Field: Heading Z Axis)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x0d,                    //     USAGE_PAGE (Digitizers)
    0x09, 0x3b,                    //     USAGE (Battery Strength)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x64,                    //     LOGICAL_MAXIMUM (100)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x09, 0x5b,                    //     USAGE (Transducer Serial Number)
    0x17, 0x00, 0x00, 0x00, 0x80,  //     LOGICAL_MINIMUM(-2,147,483,648)
    0x27, 0xFF, 0xFF, 0xFF, 0x7F,  //     LOGICAL_MAXIMUM(2,147,483,647)
    0x75, 0x40,                    //     REPORT_SIZE (64)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x09, 0x6E,                    //     USAGE(Transducer Serial Number Part 2[110])
    0x75, 0x20,                    //     REPORT_SIZE (32)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x0d,                    //     USAGE_PAGE (Digitizers)
    0x09, 0x5c,                    //     USAGE (Preferred Color)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //     LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x09, 0x5c,                    //     USAGE (Preferred Color)
    0x27, 0xff, 0xff, 0xff, 0x00,  //     LOGICAL_MAXIMUM (0x00FFFFFF)
    0x75, 0x18,                    //     REPORT_SIZE (24)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x09, 0x6f,                    //     USAGE (No Preferred Color)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x07,                    //     REPORT_COUNT (7)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x09, 0x5e,                    //     USAGE (Preferred Line Width)
    0x26, 0xff, 0x00,              //     LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x09, 0x70,                    //     USAGE (Preferred Line Style)
    0xa1, 0x02,                    //     COLLECTION (Logical)
    0x15, 0x01,                    //       LOGICAL_MINIMUM (1)
    0x25, 0x06,                    //       LOGICAL_MAXIMUM (6)
    0x09, 0x72,                    //       USAGE (Ink)
    0x09, 0x73,                    //       USAGE (Pencil)
    0x09, 0x74,                    //       USAGE (Highlighter)
    0x09, 0x75,                    //       USAGE (Chisel Marker)
    0x09, 0x76,                    //       USAGE (Brush)
    0x09, 0x77,                    //       USAGE (No Preferred Line Style)
    0x81, 0x20,                    //       INPUT (Data,Ary,Abs,NPrf)
    0xc0,                          //     END_COLLECTION
    0x06, 0x00, 0xff,              //     USAGE_PAGE (Vendor Defined Page 1)
    0x09, 0x01,                    //     USAGE (Vendor Usage 1)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x27, 0xff, 0xff, 0x00, 0x00,  //     LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //     REPORT_SIZE (16)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x0d,                    //     USAGE_PAGE (Digitizers)
    0x55, 0x0c,                    //     UNIT_EXPONENT (-4)
    0x66, 0x01, 0x10,              //     UNIT (SI Lin:Time)
    0x47, 0xff, 0xff, 0x00, 0x00,  //     PHYSICAL_MAXIMUM (65535)
    0x27, 0xff, 0xff, 0x00, 0x00,  //     LOGICAL_MAXIMUM (65535)
    0x09, 0x56,                    //     USAGE (Scan Time)
    0x75, 0x10,                    //     REPORT_SIZE (16)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                          //   END_COLLECTION

// 7.5 HID Descriptor for Status Reports

// The following is the portion of the HID descriptor for the status report that A USI
// controller shall support for reporting status and error conditions.

    0x05, 0x0d,                    //   USAGE_PAGE (Digitizers)
    0x85, HID_REPORTID_ERROR,      //   REPORT_ID (HID_REPORTID_ERROR)
    0x09, 0x38,                    //   USAGE (Transducer Index)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, MAX_SUPPORTED_STYLI,     //   LOGICAL_MAXIMUM (MAX_SUPPORTED_STYLI)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x15, 0x01,                    //   LOGICAL_MINIMUM (1)
    0x25, 0x04,                    //   LOGICAL_MAXIMUM (4)
    0x09, 0x81,                    //   USAGE (Digitizer Error)
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x82,                    //     USAGE (Err Normal Status)
    0x09, 0x83,                    //     USAGE (Err Transducers Exceeded)
    0x09, 0x84,                    //     USAGE (Err Full Trans Features Unavail)
    0x09, 0x85,                    //     USAGE (Err Charge Low)
    0x81, 0x20,                    //     INPUT (Data,Ary,Abs,NPrf)
    0xc0,                          //   END_COLLECTION


// 7.6 HID Descriptor for Feature Reports

// Following is the portion of the HID descriptor for the Get/Set Feature Reports.

    // Feature Get/Set - 8-Bit Line Color
    0x85, HID_REPORTID_GETSET_COLOR8,    //   REPORT_ID (HID_REPORTID_GETSET_COLOR8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, MAX_SUPPORTED_STYLI,     //   LOGICAL_MAXIMUM (MAX_SUPPORTED_STYLI)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x09, 0x38,                    //   USAGE (Transducer Index)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0x09, 0x5c,                    //   USAGE (Preferred Color)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0x09, 0x5d,                    //   USAGE (Preferred Color is Locked)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0x95, 0x07,                    //   REPORT_COUNT (7)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Feature Get/Set - 24-Bit Line Color
    0x85, HID_REPORTID_GETSET_COLOR24,// REPORT_ID (HID_REPORTID_GETSET_COLOR24)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, MAX_SUPPORTED_STYLI,     //   LOGICAL_MAXIMUM (MAX_SUPPORTED_STYLI)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x09, 0x38,                    //   USAGE (Transducer Index)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0x09, 0x5c,                    //   USAGE (Preferred Color)
    0x27, 0xff, 0xff, 0xff, 0x00,  //   LOGICAL_MAXIMUM (0xFFFFFF)
    0x75, 0x18,                    //   REPORT_SIZE (24)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0x09, 0x6f,                    //   USAGE (No Preferred Color)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0x09, 0x5d,                    //   USAGE (Preferred Color is Locked)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Feature Get/Set - Line Width
    0x85, HID_REPORTID_GETSET_WIDTH,    //   REPORT_ID (HID_REPORTID_GETSET_WIDTH)
    0x09, 0x38,                    //   USAGE (Transducer Index)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, MAX_SUPPORTED_STYLI,     //   LOGICAL_MAXIMUM (MAX_SUPPORTED_STYLI)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0x09, 0x5e,                    //   USAGE (Preferred Line Width)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0x09, 0x5f,                    //   USAGE (Preferred Line Width is Locked)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0x75, 0x07,                    //   REPORT_SIZE (7)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Feature Get/Set - Line Style
    0x85, HID_REPORTID_GETSET_STYLE,    //   REPORT_ID (HID_REPORTID_GETSET_STYLE)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, MAX_SUPPORTED_STYLI,     //   LOGICAL_MAXIMUM (MAX_SUPPORTED_STYLI)
    0x09, 0x38,                    //   USAGE (Transducer Index)
    0xb1, 0x22,                    //   FEATURE (Data,Var,Abs,NPrf)
    0x09, 0x70,                    //   USAGE (Preferred Line Style)
    0x15, 0x01,                    //   LOGICAL_MINIMUM (1)
    0x25, 0x06,                    //   LOGICAL_MAXIMUM (6)
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x72,                    //     USAGE (Ink)
    0x09, 0x73,                    //     USAGE (Pencil)
    0x09, 0x74,                    //     USAGE (Highlighter)
    0x09, 0x75,                    //     USAGE (Chisel Marker)
    0x09, 0x76,                    //     USAGE (Brush)
    0x09, 0x77,                    //     USAGE (No Preferred Line Style)
    0xb1, 0x20,                    //     FEATURE (Data,Ary,Abs,NPrf)
    0xc0,                          //   END_COLLECTION
    0x09, 0x71,                    //   USAGE (Preferred Line Style is Locked)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0x75, 0x07,                    //   REPORT_SIZE (7)
    0xb1, 0x03,                    //   FEATURE (Cnst,Var,Abs)

    // Feature Get/Set - Diagnostic
    0x85, HID_REPORTID_DIAGNOSE,        //   REPORT_ID (HID_REPORTID_DIAGNOSE)
    0x09, 0x80,                    //   USAGE (Digitizer Diagnostic)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x75, 0x40,                    //   REPORT_SIZE (64)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)

    // Feature Get/Set - Buttons
    0x85, HID_REPORTID_GETSET_BUTTONS,  //   REPORT_ID (HID_REPORTID_GETSET_BUTTONS)
    0x09, 0xa5,                    //   USAGE (Transducer Switches)
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x38,                    //     USAGE (Transducer Index)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x25, MAX_SUPPORTED_STYLI,     //     LOGICAL_MAXIMUM (MAX_SUPPORTED_STYLI)
    0xb1, 0x02,                    //     FEATURE (Data,Var,Abs)
    0x15, 0x01,                    //     LOGICAL_MINIMUM (1)
    0x25, 0x05,                    //     LOGICAL_MAXIMUM (5)
    0x09, 0x44,                    //     USAGE (Barrel Switch)
    0xa1, 0x02,                    //     COLLECTION (Logical)
    0x09, 0xa4,                    //       USAGE (Switch Unimplemented)
    0x09, 0x44,                    //       USAGE (Barrel Switch)
    0x09, 0x5a,                    //       USAGE (Secondary Barrel Switch)
    0x09, 0x45,                    //       USAGE (Eraser)
    0x09, 0xa3,                    //       USAGE (Switch Disabled)
    0xb1, 0x20,                    //       FEATURE (Data,Ary,Abs,NPrf)
    0xc0,                          //     END_COLLECTION
    0x09, 0x5a,                    //     USAGE (Secondary Barrel Switch)
    0xa1, 0x02,                    //     COLLECTION (Logical)
    0x09, 0xa4,                    //       USAGE (Switch Unimplemented)
    0x09, 0x44,                    //       USAGE (Barrel Switch)
    0x09, 0x5a,                    //       USAGE (Secondary Barrel Switch)
    0x09, 0x45,                    //       USAGE (Eraser)
    0x09, 0xa3,                    //       USAGE (Switch Disabled)
    0xb1, 0x20,                    //       FEATURE (Data,Ary,Abs,NPrf)
    0xc0,                          //     END_COLLECTION
    0x09, 0x45,                    //     USAGE (Eraser)
    0xa1, 0x02,                    //     COLLECTION (Logical)
    0x09, 0xa4,                    //       USAGE (Switch Unimplemented)
    0x09, 0x44,                    //       USAGE (Barrel Switch)
    0x09, 0x5a,                    //       USAGE (Secondary Barrel Switch)
    0x09, 0x45,                    //       USAGE (Eraser)
    0x09, 0xa3,                    //       USAGE (Switch Disabled)
    0xb1, 0x20,                    //       FEATURE (Data,Ary,Abs,NPrf)
    0xc0,                          //     END_COLLECTION
    0xc0,                          //   END_COLLECTION

    // Feature Get - Firmware Version
    0x85, HID_REPORTID_GET_FIRMWARE,    //   REPORT_ID (HID_REPORTID_GET_FIRMWARE)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x05, 0x0d,                    //   USAGE_PAGE (Digitizers)
    0x09, 0x90,                    //   USAGE (Transducer Software Info.)
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x38,                    //     USAGE (Transducer Index)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, MAX_SUPPORTED_STYLI,     //     LOGICAL_MAXIMUM (MAX_SUPPORTED_STYLI)
    0xb1, 0x02,                    //     FEATURE (Data,Var,Abs)
    0x09, 0x5b,                    //     USAGE (Transducer Serial Number)
    0x17, 0x00, 0x00, 0x00, 0x80,  //     LOGICAL_MINIMUM(-2,147,483,648)
    0x27, 0xFF, 0xFF, 0xFF, 0x7F,  //     LOGICAL_MAXIMUM(2,147,483,647)
    0x75, 0x40,                    //     REPORT_SIZE (64)
    0xb1, 0x02,                    //     FEATURE (Data,Var,Abs)
    0x09, 0x6E,                    //     USAGE(Transducer Serial Number Part 2[110])
    0x75, 0x20,                    //     REPORT_SIZE (32)
    0xb1, 0x02,                    //     FEATURE (Data,Var,Abs)
    0x09, 0x91,                    //     USAGE (Transducer Vendor ID)
    0x75, 0x10,                    //     REPORT_SIZE (16)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x0f,              //     LOGICAL_MAXIMUM (4095)
    0xb1, 0x02,                    //     FEATURE (Data,Var,Abs)
    0x09, 0x92,                    //     USAGE (Transducer Product ID)
    0x27, 0xff, 0xff, 0x00, 0x00,  //     LOGICAL_MAXIMUM (65535)
    0xb1, 0x02,                    //     FEATURE (Data,Var,Abs)
    0x05, 0x06,                    //     USAGE_PAGE (Generic Device)
    0x09, 0x2a,                    //     USAGE (Software Version)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x26, 0xff, 0x00,              //     LOGICAL_MAXIMUM (255)
    0xa1, 0x02,                    //     COLLECTION (Logical)
    0x09, 0x2d,                    //       USAGE (Major)
    0xb1, 0x02,                    //       FEATURE (Data,Var,Abs)
    0x09, 0x2e,                    //       USAGE (Minor)
    0xb1, 0x02,                    //       FEATURE (Data,Var,Abs)
    0xc0,                          //     END_COLLECTION
    0xc0,                          //   END_COLLECTION

    // Feature Get - USI Version
    0x85, HID_REPORTID_GET_PROTOCOL,    //   REPORT_ID (HID_REPORTID_GET_PROTOCOL)
    0x05, 0x0d,                    //   USAGE_PAGE (Digitizers)
    0x25, MAX_SUPPORTED_STYLI,     //   LOGICAL_MAXIMUM (MAX_SUPPORTED_STYLI)
    0x09, 0x38,                    //   USAGE (Transducer Index)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0x05, 0x06,                    //   USAGE_PAGE (Generic Device)
    0x09, 0x2b,                    //   USAGE (Protocol Version)
    0xa1, 0x02,                    //   COLLECTION (Logical)
    0x09, 0x2d,                    //     USAGE (Major)
    0x26, 0xff, 0x00,              //     LOGICAL_MAXIMUM (255)
    0xb1, 0x02,                    //     FEATURE (Data,Var,Abs)
    0x09, 0x2e,                    //     USAGE (Minor)
    0xb1, 0x02,                    //     FEATURE (Data,Var,Abs)
    0xc0,                          //   END_COLLECTION

    // Feature Get/Set - Vendor Specific
    0x85, HID_REPORTID_GETSET_VENDOR,   //   REPORT_ID (HID_REPORTID_GETSET_VENDOR)
    0x05, 0x0d,                    //   USAGE_PAGE (Digitizers)
    0x09, 0x38,                    //   USAGE (Transducer Index)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x25, MAX_SUPPORTED_STYLI,     //   LOGICAL_MAXIMUM (MAX_SUPPORTED_STYLI)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)
    0x06, 0x00, 0xff,              //   USAGE_PAGE (Vendor Defined Page 1)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)


    // Feature Set - Select Transducer Index
    0x85, HID_REPORTID_SET_TRANSDUCER,  //   REPORT_ID (HID_REPORTID_SET_TRANSDUCER)
    0x05, 0x0d,                    //   USAGE_PAGE (Digitizers)
    0x09, 0xa6,                    //   USAGE (Transducer Index Selector)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, MAX_SUPPORTED_STYLI,     //   LOGICAL_MAXIMUM (MAX_SUPPORTED_STYLI)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)

    0xc0                           // END_COLLECTION
};

#define G_USI_DEFAULT_STYLUS_INDEX		1
#define	G_USI_HID_SET_REPORT			0x01
#define G_USI_HID_GET_REPORT			0x02
#define G_USI_HID_SETGET_REPORT			(G_USI_HID_SET_REPORT | G_USI_HID_GET_REPORT)

#define G_USI_GID_FW_VER_FLAG			(G_USI_GID_FLAG | G_USI_FW_VERSION_FLAG)
static struct hid_feature_report_info {
	u8 id;
	u8 type;		/* Bit Flag - supported report type. 1: SET, 2: GET, 3: SET&GET */
	u16 size;
	u32 required_stylus_info;
} hid_feature_report_infos[] = {
	{HID_REPORTID_GETSET_COLOR8,	G_USI_HID_SETGET_REPORT,	4,  G_USI_CAPABILITY_FLAG},
	{HID_REPORTID_GETSET_WIDTH,	G_USI_HID_SETGET_REPORT,	4,  G_USI_CAPABILITY_FLAG},
	{HID_REPORTID_GETSET_STYLE,	G_USI_HID_SETGET_REPORT,	4,  G_USI_CAPABILITY_FLAG},
	{HID_REPORTID_DIAGNOSE,		G_USI_HID_SETGET_REPORT,	9,  0},
	{HID_REPORTID_GETSET_BUTTONS,	G_USI_HID_SETGET_REPORT,	5,  G_USI_CAPABILITY_FLAG},
	{HID_REPORTID_GET_FIRMWARE,	G_USI_HID_GET_REPORT,		20, G_USI_GID_FW_VER_FLAG},
	{HID_REPORTID_GET_PROTOCOL,	G_USI_HID_GET_REPORT,		4,  G_USI_USI_VERSION_FLAG},
	{HID_REPORTID_GETSET_VENDOR,	G_USI_HID_SETGET_REPORT,	4,  G_USI_VENDOR_EXT_FLAG},
	{HID_REPORTID_SET_TRANSDUCER,	0,				2,  0},
	{HID_REPORTID_GETSET_COLOR24,	G_USI_HID_SETGET_REPORT,	6,  G_USI_CAPABILITY_FLAG},
};

enum {
	PREAMBLE,
	STYLUS_ID,
	COMMAND_ID,
	DATA,
	CRC,
	SUBCMD_ID,
};

/* main commands */
enum {
	G_USI_CONFIG_PHY,
	G_USI_CONFIG_DOWNLINK,
	G_USI_CONFIG_MENU_SELECT,
	G_USI_CONFIG_PACKET,
	G_USI_DISABLE_PACKET,
	G_USI_SET_COLOR,
	G_USI_SET_BUTTONS,
	G_USI_SET_TYPE,
	G_USI_VERIFY_HASH_LOW,
	G_USI_UPDATE_PHY,
	G_USI_CLEAR_INT_FLAGS,
	G_USI_TRANSMIT_POSITION_TONE,
	G_USI_SET_WIDTH,
	G_USI_SEND_MY_VENDOR_ID,
	G_USI_VERIFY_HASH_HIGH,
	G_USI_SET_VENDOR_EXTENSION,
	G_USI_UPDATE_PHY_TX2,
	G_USI_CONFIG_PACKET_TX2,
	G_USI_SEND_SESSION_ID,
	G_USI_SLOT_CADENCE,
	G_USI_CONFIG_PHY_FLEX,
	G_USI_UPDATE_PHY_FLEX,
	G_USI_SAVE_CONFIG_TO_MEM,
	G_USI_CONFIG_FROM_MEM,
	G_USI_WRITE_CMD_MAX,
	G_USI_READ_CMD = 63,
};

/* sub commands for read command(G_USI_READ_CMD) */
enum {
	G_USI_GET_BATTERY,
	G_USI_GET_CAPABILITY,
	G_USI_GET_USI_VERSION,
	G_USI_GET_GID,
	G_USI_GET_DATA,
	G_USI_GET_VENDOR_EXTENSION,
	G_USI_GET_PHY,
	G_USI_GET_TIMESLOT_SETUP0,
	G_USI_GET_TIMESLOT_SETUP1,
	G_USI_GET_INT_FLAGS,
	G_USI_GET_HASH_ID,
	G_USI_GET_FIRMWAR_EVERSION,
	G_USI_GET_LAST_ERROR,
	G_USI_GET_IMU,
	G_USI_GET_TILT_TWIST,
	G_USI_GET_TRANSMITTER_PARAMETERS,
	G_USI_GET_TIMESLOT_SETUP2,
	G_USI_GET_TIMESLOT_SETUP3,
	G_USI_GET_SESSION_ID,
	G_USI_RESERVED_FOR_USI = 19,
	G_USI_GET_RX_STATISTICS = 50,
	G_USI_GET_TX_STATISTICS = 51,
	G_USI_GET_READ_SUBCMD_MAX,
};

enum {
	G_USI_HID_STYLE_START			= 1,
	G_USI_HID_STYLE_INK			= G_USI_HID_STYLE_START,
	G_USI_HID_STYLE_PENCIL			= 2,
	G_USI_HID_STYLE_HIGHLIGHTER		= 3,
	G_USI_HID_STYLE_CHISEL_MARKER		= 4,
	G_USI_HID_STYLE_BRUSH			= 5,
	G_USI_HID_STYLE_NO_PREFERRED_STYLE	= 6,
	G_USI_HID_STYLE_END			= G_USI_HID_STYLE_NO_PREFERRED_STYLE
};

#define G_USI_STYLE_NO_PREFERRED		255

enum {
	G_USI_BUTTON_SWITCH_UNPMPLEMENTED	= 1,
	G_USI_BUTTON_BARREL_SWITCH		= 2,
	G_USI_BUTTON_SECONDARY_BARREL_SWITCH	= 3,
	G_USI_BUTTON_ERASER			= 4,
	G_USI_BUTTON_SWITCH_DISABLED		= 5,
};

static const struct true_color {
	u8 r;
	u8 g;
	u8 b;
} true_colors[] = {
	{0xF0, 0xF8, 0xFF},	// 0
	{0xFA, 0xEB, 0xD7},	// 1
	{0x00, 0xFF, 0xFF},	// 2
	{0x7F, 0xFF, 0xD4},	// 3
	{0xF0, 0xFF, 0xFF},	// 4
	{0xF5, 0xF5, 0xDC},
	{0xFF, 0xE4, 0xC4},
	{0x00, 0x00, 0x00},
	{0xFF, 0xEB, 0xCD},
	{0x00, 0x00, 0xFF},
	{0x8A, 0x2B, 0xE2},	// 10
	{0xA5, 0x2A, 0x2A},
	{0xDE, 0xB8, 0x87},
	{0x5F, 0x9E, 0xA0},
	{0x7F, 0xFF, 0x00},
	{0xD2, 0x69, 0x1E},
	{0xFF, 0x7F, 0x50},
	{0x64, 0x95, 0xED},
	{0xFF, 0xF8, 0xDC},
	{0xDC, 0x14, 0x3C},
	{0x00, 0xFF, 0xFF},	// 20
	{0x00, 0x00, 0x8B},
	{0x00, 0x8B, 0x8B},
	{0xB8, 0x86, 0x0B},
	{0xA9, 0xA9, 0xA9},
	{0x00, 0x64, 0x00},
	{0xBD, 0xB7, 0x6B},
	{0x8B, 0x00, 0x8B},
	{0x55, 0x6B, 0x2F},
	{0xFF, 0x8C, 0x00},
	{0x99, 0x32, 0xCC},	// 30
	{0x8B, 0x00, 0x00},
	{0xE9, 0x96, 0x7A},
	{0x8F, 0xBC, 0x8F},
	{0x48, 0x3D, 0x8B},
	{0x2F, 0x4F, 0x4F},
	{0x00, 0xCE, 0xD1},
	{0x94, 0x00, 0xD3},
	{0xFF, 0x14, 0x93},
	{0x00, 0xBF, 0xFF},
	{0x69, 0x69, 0x69},	// 40
	{0x1E, 0x90, 0xFF},
	{0xB2, 0x22, 0x22},
	{0xFF, 0xFA, 0xF0},
	{0x22, 0x8B, 0x22},
	{0xFF, 0x00, 0xFF},
	{0xDC, 0xDC, 0xDC},
	{0xF8, 0xF8, 0xFF},
	{0xFF, 0xD7, 0x00},
	{0xDA, 0xA5, 0x20},
	{0x80, 0x80, 0x80},	// 50
	{0x00, 0x80, 0x00},
	{0xAD, 0xFF, 0x2F},
	{0xF0, 0xFF, 0xF0},
	{0xFF, 0x69, 0xB4},
	{0xCD, 0x5C, 0x5C},
	{0x4B, 0x00, 0x82},
	{0xFF, 0xFF, 0xF0},
	{0xF0, 0xE6, 0x8C},
	{0xE6, 0xE6, 0xFA},
	{0xFF, 0xF0, 0xF5},	// 60
	{0x7C, 0xFC, 0x00},
	{0xFF, 0xFA, 0xCD},
	{0xAD, 0xD8, 0xE6},
	{0xF0, 0x80, 0x80},
	{0xE0, 0xFF, 0xFF},
	{0xFA, 0xFA, 0xD2},
	{0xD3, 0xD3, 0xD3},
	{0x90, 0xEE, 0x90},
	{0xFF, 0xB6, 0xC1},
	{0xFF, 0xA0, 0x7A},	// 70
	{0x20, 0xB2, 0xAA},
	{0x87, 0xCE, 0xFA},
	{0x77, 0x88, 0x99},
	{0xB0, 0xC4, 0xDE},
	{0xFF, 0xFF, 0xE0},
	{0x00, 0xFF, 0x00},
	{0x32, 0xCD, 0x32},
	{0xFA, 0xF0, 0xE6},
	{0xFF, 0x00, 0xFF},
	{0x80, 0x00, 0x00},	// 80
	{0x66, 0xCD, 0xAA},
	{0x00, 0x00, 0xCD},
	{0xBA, 0x55, 0xD3},
	{0x93, 0x70, 0xDB},
	{0x3C, 0xB3, 0x71},
	{0x7B, 0x68, 0xEE},
	{0x00, 0xFA, 0x9A},
	{0x48, 0xD1, 0xCC},
	{0xC7, 0x15, 0x85},
	{0x19, 0x19, 0x70},	// 90
	{0xF5, 0xFF, 0xFA},
	{0xFF, 0xE4, 0xE1},
	{0xFF, 0xE4, 0xB5},
	{0xFF, 0xDE, 0xAD},
	{0x00, 0x00, 0x80},
	{0xFD, 0xF5, 0xE6},
	{0x80, 0x80, 0x00},
	{0x6B, 0x8E, 0x23},
	{0xFF, 0xA5, 0x00},
	{0xFF, 0x45, 0x00},	// 100
	{0xDA, 0x70, 0xD6},
	{0xEE, 0xE8, 0xAA},
	{0x98, 0xFB, 0x98},
	{0xAF, 0xEE, 0xEE},
	{0xDB, 0x70, 0x93},
	{0xFF, 0xEF, 0xD5},
	{0xFF, 0xDA, 0xB9},
	{0xCD, 0x85, 0x3F},
	{0xFF, 0xC0, 0xCB},
	{0xDD, 0xA0, 0xDD},	// 110
	{0xB0, 0xE0, 0xE6},
	{0x80, 0x00, 0x80},
	{0x66, 0x33, 0x99},
	{0xFF, 0x00, 0x00},
	{0xBC, 0x8F, 0x8F},
	{0x41, 0x69, 0xE1},
	{0x8B, 0x45, 0x13},
	{0xFA, 0x80, 0x72},
	{0xF4, 0xA4, 0x60},
	{0x2E, 0x8B, 0x57},	// 120
	{0xFF, 0xF5, 0xEE},
	{0xA0, 0x52, 0x2D},
	{0xC0, 0xC0, 0xC0},
	{0x87, 0xCE, 0xEB},
	{0x6A, 0x5A, 0xCD},
	{0x70, 0x80, 0x90},
	{0xFF, 0xFA, 0xFA},
	{0x00, 0xFF, 0x7F},
	{0x46, 0x82, 0xB4},
	{0xD2, 0xB4, 0x8C},	// 130
	{0x00, 0x80, 0x80},
	{0xD8, 0xBF, 0xD8},
	{0xFF, 0x63, 0x47},
	{0x40, 0xE0, 0xD0},
	{0xEE, 0x82, 0xEE},
	{0xF5, 0xDE, 0xB3},
	{0xFF, 0xFF, 0xFF},
	{0xF5, 0xF5, 0xF5},
	{0xFF, 0xFF, 0x00},
	{0x9A, 0xCD, 0x32},	// 140
};

static const int beacon_bit_fields[] = {
	[PREAMBLE]   = 3,
	[STYLUS_ID]  = 3,
	[COMMAND_ID] = 6,
	[DATA]       = 16,
	[CRC]        = 5,
	[SUBCMD_ID]  = 7, /* subcommand + 'R'(reserved) */
};

#define DEFAULT_PREAMBLE_FLEX       0x1
#define DEFAULT_PREAMBLE_STD        0x4
#define CRC_POLYNOMIAL              0x39
#define CRC_POLYNOMIAL_LENGTH       6

/*
 * CRC calculation from USI 2.0 spec
 */
static u8 calculate_crc(u32 data)
{
	int i;
	int data_len = beacon_bit_fields[STYLUS_ID] +
			beacon_bit_fields[COMMAND_ID] + beacon_bit_fields[DATA];

	u32 crc_polynomial = CRC_POLYNOMIAL;
	u32 msb_mask = 1 << (data_len - 1);

	crc_polynomial = crc_polynomial << (data_len - CRC_POLYNOMIAL_LENGTH);

	for (i = 0; i < data_len; i++) {
		if (data & msb_mask)
			data = data ^ crc_polynomial;
		data = data << 1;
	}

	return data >> (data_len - (CRC_POLYNOMIAL_LENGTH - 1));
}

static u8 get_uplink_preamble(struct g_usi_context *usi_ctx)
{
	if (usi_ctx->is_flex_beacon)
		return DEFAULT_PREAMBLE_FLEX;

	return DEFAULT_PREAMBLE_STD;
}

static u16 create_uplink_data_to_read(u8 read_sub_cmd, int read_flag)
{
	u16 data;

	data = 0x01 << read_flag;
	data = (u16)read_sub_cmd | data << beacon_bit_fields[SUBCMD_ID];

	return data;
}

static void create_uplink(u8 *buf, u8 preamble, u8 stylus_id, u8 cmd_id, u16 data)
{
	u64 beacon = 0;
	u32 command = 0;
	u8 crc;

	command = (u32)data;
	command = (u32)cmd_id | command << beacon_bit_fields[COMMAND_ID];
	command = (u32)stylus_id | command << beacon_bit_fields[STYLUS_ID];

	crc = calculate_crc(command);

	beacon = (u64)crc;
	beacon = (u64)command | beacon << (beacon_bit_fields[DATA] +
				     beacon_bit_fields[COMMAND_ID] +
				     beacon_bit_fields[STYLUS_ID]);
	beacon = (u64)preamble | beacon << beacon_bit_fields[PREAMBLE];

	buf[0] = (u8)beacon;
	buf[1] = (u8)(beacon >> 8);
	buf[2] = (u8)(beacon >> 16);
	buf[3] = (u8)(beacon >> 24);
	buf[4] = (u8)(beacon >> 32);
}

static struct hid_feature_report_info *get_feature_report_info(int rpt_id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hid_feature_report_infos); i++) {
		if (hid_feature_report_infos[i].id == rpt_id)
			return &hid_feature_report_infos[i];
	}

	return NULL;
}

static u8 g_usi_color24_to_color8(u8 r, u8 g, u8 b)
{
	int color8_idx = 0, r_diff, g_diff, b_diff;
	u32 distance, closest = 0xFFFFFFFF;
	int i;

	for (i = 0; i < ARRAY_SIZE(true_colors); i++) {
		r_diff = r - true_colors[i].r;
		g_diff = g - true_colors[i].g;
		b_diff = b - true_colors[i].b;

		distance = r_diff * r_diff + g_diff * g_diff + b_diff * b_diff;
		if (distance < closest) {
			closest = distance;
			color8_idx = i;
		}
	}

	return color8_idx;
}

static bool g_usi_color8_to_color24(int color8_idx, u8 *r, u8 *g, u8 *b)
{
	if (color8_idx < 0 || color8_idx >= ARRAY_SIZE(true_colors))
		return false;

	*r = true_colors[color8_idx].r;
	*g = true_colors[color8_idx].g;
	*b = true_colors[color8_idx].b;

	return true;
}

/*
 * convert button info: USI button value to HID button value
 *
 * Refer USI 2.0 spec
 * 6.1.3.4 C.GetCapability()
 * 7.3.3.1.9 Get/Set Button and Active Tail
 */
static u8 get_button_hid_value(struct g_usi_context *usi_ctx, int btn_no)
{
	u8 usi_btn_func_value = 0xFF;

	/*
	 * Check with the capability of the stylus to see which buttons are implemented and
	 * get the designated function for the button if it's implemented.
	 */
	if (btn_no == 1 && usi_ctx->stylus_capability[1] & 0x08) /* Button1 implemented ? */
		usi_btn_func_value = usi_ctx->stylus_capability[6] & 0x03;
	else if (btn_no == 2 && (usi_ctx->stylus_capability[1] & 0x10)) /* Button2 implemented ? */
		usi_btn_func_value = (usi_ctx->stylus_capability[6] & 0x0C) >> 2;
	else if (btn_no == 3 && (usi_ctx->stylus_capability[1] & 0x20)) /* Eraser implemented ? */
		usi_btn_func_value = (usi_ctx->stylus_capability[6] & 0x30) >> 4;
	else
		return G_USI_BUTTON_SWITCH_UNPMPLEMENTED;

	if (usi_btn_func_value == 0) /* is the button disabled ? */
		return G_USI_BUTTON_SWITCH_DISABLED;

	/*
	 * USI Primary(1) -> HID Barrel(2)
	 * USI Secondary(2) -> HID Secondary Barrel(3)
	 * USI Eraser(3) -> HID Eraser(4)
	 */
	return usi_btn_func_value + 1;
}

static int g_usi_send_diag_cmd(struct g_usi_context *usi_ctx, u8 stylus_id,
			       u8 cmd_id, u16 cmd_data, u8 response[3])
{
	int ret;
	u8 uplink[5];
	u8 res[3];
	u8 preamble;

	memset(uplink, 0, sizeof(uplink));
	memset(res, 0, sizeof(res));

	preamble = get_uplink_preamble(usi_ctx);
	create_uplink(uplink, preamble, 1, cmd_id, cmd_data);
	mutex_unlock(&usi_ctx->lock);	/* This allow callback to call subsystem API functions */
	ret = usi_ctx->cbs->g_usi_send_uplink(context_to_handle(usi_ctx), uplink, res);
	mutex_lock(&usi_ctx->lock);
	if (ret != 0)
		return ret;

	if (response[2] & 0x40)	{ /* EIA(Error Information Available) is set ? */
		G_USI_ERR("EIA is set. Error Code : %d", response[2] & 0x3F);
		return -EIO;
	}

	memcpy(response, res, sizeof(res));

	return 0;
}

static int g_usi_send_write_cmd(struct g_usi_context *usi_ctx, u8 stylus_id,
				u8 cmd_id, u16 cmd_data)
{
	int ret;
	u8 response[3];

	ret = g_usi_send_diag_cmd(usi_ctx, stylus_id, cmd_id, cmd_data, response);
	if (ret != 0) {
		G_USI_ERR("diag for write failed : cmd %d", cmd_id);
		return ret;
	}

	/* response of a write command should be the same as the command data */
	if ((cmd_data & 0x00FF) != response[0] || ((cmd_data & 0xFF00) >> 8) != response[1])
		return false;

	return 0;
}

static int g_usi_send_read_cmd(struct g_usi_context *usi_ctx, u8 stylus_id,
			       u8 read_sub_cmd, int read_flag, u16 *read_data)
{
	int ret;
	u16 cmd_data;
	u8 response[3];

	cmd_data = create_uplink_data_to_read(read_sub_cmd, read_flag);
	ret = g_usi_send_diag_cmd(usi_ctx, stylus_id, G_USI_READ_CMD, cmd_data, response);
	if (ret != 0) {
		G_USI_ERR("diag for read failed: sub cmd %d", read_sub_cmd);
		return ret;
	}

	*read_data = response[1] << 8 | response[0];

	return 0;
}

static int g_usi_update_capability(struct g_usi_context *usi_ctx, int read_flag)
{
	int ret;
	u16 read_data;

	ret = g_usi_send_read_cmd(usi_ctx, 1, G_USI_GET_CAPABILITY, read_flag, &read_data);
	if (ret != 0) {
		G_USI_ERR("Read Capability failed: %d", ret);
		return ret;
	}

	usi_ctx->stylus_capability[read_flag * 2] = (u8)(read_data & 0x00FF);
	usi_ctx->stylus_capability[read_flag * 2 + 1] = (u8)((read_data >> 8) & 0x00FF);

	return 0;
}

static int g_usi_update_colors(struct g_usi_context *usi_ctx)
{
	int ret;

	/* refresh the part of capability for color information */
	ret = g_usi_update_capability(usi_ctx, 2);
	if (is_true_color_supported(usi_ctx)) {
		ret |= g_usi_update_capability(usi_ctx, 4);
		ret |= g_usi_update_capability(usi_ctx, 5);
	}

	return ret;
}

static int g_usi_hid_get_feature_report(struct g_usi_context *usi_ctx, u8 *buf, int buf_size,
					struct hid_feature_report_info *rpt_info)
{
	int ret = 0;
	u8 response[3];
	u8 color8 = 0;
	u8 usi_style = 0xFF;

	if (rpt_info->id != HID_REPORTID_DIAGNOSE)
		buf[1] = G_USI_DEFAULT_STYLUS_INDEX;

	switch (rpt_info->id) {
	case HID_REPORTID_GETSET_COLOR8:
		if (!is_capability_available(usi_ctx))
			return -ENODATA;

		buf[2] = usi_ctx->stylus_capability[4];		/* preferred color */
		buf[3] = is_preferred_color_read_only(usi_ctx); /* preferred color is RO ? */
		break;
	case HID_REPORTID_GETSET_WIDTH:
		if (!is_capability_available(usi_ctx))
			return -ENODATA;

		buf[2] = usi_ctx->stylus_capability[3];		/* preferred width */
		buf[3] = is_preferred_width_read_only(usi_ctx); /* preferred width is RO ? */
		break;
	case HID_REPORTID_GETSET_STYLE:
		if (!is_capability_available(usi_ctx))
			return -ENODATA;

		usi_style = usi_ctx->stylus_capability[2];	/* preferred style */
		/* USI to HID conversion for No Preference */
		if (usi_style == G_USI_STYLE_NO_PREFERRED)
			buf[2] = G_USI_HID_STYLE_NO_PREFERRED_STYLE;
		else
			buf[2] = usi_style;
		buf[3] = is_preferred_style_read_only(usi_ctx); /* preferred style is RO ? */
		break;
	case HID_REPORTID_GETSET_BUTTONS:
		if (!is_capability_available(usi_ctx))
			return -ENODATA;

		buf[2] = get_button_hid_value(usi_ctx, 1); /* get button 1 */
		buf[3] = get_button_hid_value(usi_ctx, 2); /* get button 2 */
		buf[4] = get_button_hid_value(usi_ctx, 3); /* get button 3 */
		break;
	case HID_REPORTID_GET_FIRMWARE:
		if (!is_gid_available(usi_ctx) || !is_fw_version_available(usi_ctx))
			return -ENODATA;

		/* USI 2.0 spec. 7.3.3.1.3 Get Stylus Firmware Info */
		/* 64bits Transducer Serial Number(GID0 ~ GID3) */
		memcpy(buf + 2, usi_ctx->stylus_GID, 8);

		/* 32bits Transducer Serial Number Part 2(GID2 ~ GID3) & VID/PID(GID4 ~ GID5) */
		memcpy(buf + 10, usi_ctx->stylus_GID + 4, 8);

		/* FW version major/minor */
		buf[18] = usi_ctx->stylus_fw_ver[1];
		buf[19] = usi_ctx->stylus_fw_ver[0];
		break;
	case HID_REPORTID_GETSET_COLOR24:
		if (!is_capability_available(usi_ctx))
			return -ENODATA;

		buf[5] = 0;
		if (is_preferred_color_read_only(usi_ctx))
			buf[5] |= 0x02;				/* Preferred Color is RO */

		if (usi_ctx->stylus_capability[4] == 0xFF) {	/* No Preferred Color */
			buf[2] = 0; /* Blue Color Value */
			buf[3] = 0; /* Green Color Value */
			buf[4] = 0;  /* Red Color Value */
			buf[5] |= 0x01; /* True for no Preferred Color */

			break;
		}

		/* There's the preferred color */

		if (is_true_color_supported(usi_ctx)) {
			buf[2] = usi_ctx->stylus_capability[10]; /* Blue Value */
			buf[3] = usi_ctx->stylus_capability[11]; /* Green Value */
			buf[4] = usi_ctx->stylus_capability[8];  /* Red Value */
		} else {
			color8 = usi_ctx->stylus_capability[4];

			/* convert color8 to color24 - buf[2]:blue, buf[3]:green, buf[4]:red */
			if (g_usi_color8_to_color24(color8, buf + 4, buf + 3, buf + 2) == false)
				return -ENODATA;
		}

		break;
	case HID_REPORTID_GET_PROTOCOL:
		if (!is_usi_version_available(usi_ctx))
			return -ENODATA;

		buf[2] = (usi_ctx->stylus_usi_ver & 0xF0) >> 4;
		buf[3] = usi_ctx->stylus_usi_ver & 0x0F;
		break;
	case HID_REPORTID_GETSET_VENDOR:
		if (!is_vendor_ext_available(usi_ctx))
			return -ENODATA;

		buf[2] = usi_ctx->stylus_vendor_ext[0];
		buf[3] = usi_ctx->stylus_vendor_ext[1];
		break;
	case HID_REPORTID_DIAGNOSE:
		if (!usi_ctx->cbs->g_usi_send_uplink) {
			G_USI_ERR("Get Diagnostic Feature Request is not supported");
			return -EINVAL;
		}

		if (!usi_ctx->is_diagnose_requested) {
			G_USI_ERR("No HID SET DIAGNOSE is requested");
			return -ENODATA;
		}

		mutex_unlock(&usi_ctx->lock);
		ret = usi_ctx->cbs->g_usi_send_uplink(context_to_handle(usi_ctx),
						      usi_ctx->diagnose_uplink, response);
		mutex_lock(&usi_ctx->lock);
		if (ret != 0) {
			G_USI_ERR("Get Last Error failed : %d", ret);
			break;
		}

		memcpy(&buf[1], response, sizeof(response));

		usi_ctx->is_diagnose_requested = false;
		break;
	}

	return ret;
}

static int g_usi_hid_set_feature_report(struct g_usi_context *usi_ctx, u8 *buf, int buf_size,
					struct hid_feature_report_info *rpt_info)
{
	int ret = 0;
	int i;
	u16 cmd_data = 0;
	u8 stylus_has_button = 0x20;		/* refer to C.GetCapability() in USI 2.0 spec */
	u8 color8 = 0, r, g, b;

	if (!usi_ctx->cbs->g_usi_send_uplink) {
		G_USI_ERR("Set Feature Request is not supported");
		return -EINVAL;
	}

	switch (rpt_info->id) {
	case HID_REPORTID_GETSET_COLOR8:
		if (!is_capability_available(usi_ctx))
			return -EAGAIN;

		if (is_preferred_color_read_only(usi_ctx))
			return -EINVAL;

		color8 = buf[2];
		if (is_true_color_supported(usi_ctx) &&
		    g_usi_color8_to_color24(color8, &r, &g, &b)) {
			cmd_data = 0x0100 | b;
			g_usi_send_write_cmd(usi_ctx, 1, G_USI_SET_COLOR, cmd_data);

			cmd_data = 0x0200 | g;
			g_usi_send_write_cmd(usi_ctx, 1, G_USI_SET_COLOR, cmd_data);

			cmd_data = 0x0400 | r;
			g_usi_send_write_cmd(usi_ctx, 1, G_USI_SET_COLOR, cmd_data);
		}

		cmd_data = color8;
		ret = g_usi_send_write_cmd(usi_ctx, 1, G_USI_SET_COLOR, cmd_data);
		if (ret != 0)
			G_USI_ERR("Set Color8 failed : %d", ret);

		ret = g_usi_update_colors(usi_ctx);
		if (ret != 0)
			G_USI_ERR("Color Sync failed : %d", ret);

		break;
	case HID_REPORTID_GETSET_COLOR24:
		if (!is_capability_available(usi_ctx))
			return -EAGAIN;

		if (is_preferred_color_read_only(usi_ctx))
			return -EINVAL;

		color8 = g_usi_color24_to_color8(buf[4], buf[3], buf[2]);

		if (is_true_color_supported(usi_ctx)) {
			for (i = 0; i < 3; i++) {
				cmd_data = (0x0100 << i) | buf[i + 2];
				ret = g_usi_send_write_cmd(usi_ctx, 1, G_USI_SET_COLOR, cmd_data);
				if (ret != 0) {
					G_USI_ERR("Set Color24 - %04X failed : %d", cmd_data, ret);
					break;
				}
			}
		}

		cmd_data = color8;
		ret = g_usi_send_write_cmd(usi_ctx, 1, G_USI_SET_COLOR, cmd_data);
		if (ret != 0)
			G_USI_ERR("Set Color8 failed : %d", ret);

		ret = g_usi_update_colors(usi_ctx);
		if (ret != 0)
			G_USI_ERR("Color Sync failed : %d", ret);

		break;
	case HID_REPORTID_GETSET_WIDTH:
		if (!is_capability_available(usi_ctx))
			return -EAGAIN;

		if (is_preferred_width_read_only(usi_ctx))
			return -EINVAL;

		cmd_data = buf[2];
		ret = g_usi_send_write_cmd(usi_ctx, 1, G_USI_SET_WIDTH, cmd_data);
		if (ret != 0)
			G_USI_ERR("Set Width failed : %d", ret);

		ret = g_usi_update_capability(usi_ctx, 1);
		if (ret != 0)
			G_USI_ERR("Width Sync failed : %d", ret);

		break;
	case HID_REPORTID_GETSET_STYLE:
		if (!is_capability_available(usi_ctx))
			return -EAGAIN;

		if (is_preferred_style_read_only(usi_ctx))
			return -EINVAL;

		if (buf[2] < G_USI_HID_STYLE_START || buf[2] > G_USI_HID_STYLE_END) {
			G_USI_ERR("Set Style - invalid style %d", buf[2]);
			return -EINVAL;
		}

		if (buf[2] == G_USI_HID_STYLE_NO_PREFERRED_STYLE)
			cmd_data = G_USI_STYLE_NO_PREFERRED;
		else
			cmd_data = buf[2];

		ret = g_usi_send_write_cmd(usi_ctx, 1, G_USI_SET_TYPE, cmd_data);
		if (ret != 0)
			G_USI_ERR("Set Type failed : %d", ret);

		ret = g_usi_update_capability(usi_ctx, 1);
		if (ret != 0)
			G_USI_ERR("Type Sync failed : %d", ret);

		break;
	case HID_REPORTID_GETSET_BUTTONS:
		if (!is_capability_available(usi_ctx))
			return -EAGAIN;

		cmd_data = 0;
		for (i = 4; i >= 2; i--) {
			cmd_data <<= 2;

			if (buf[i] < G_USI_BUTTON_SWITCH_UNPMPLEMENTED ||
			    buf[i] > G_USI_BUTTON_SWITCH_DISABLED) {
				G_USI_ERR("Set Button[%d] - invalid function %d", i - 1, buf[i]);
				return -EINVAL;
			}

			if (usi_ctx->stylus_capability[1] & stylus_has_button)
				cmd_data |= buf[i] - 1;

			stylus_has_button >>= 1;
		}

		ret = g_usi_send_write_cmd(usi_ctx, 1, G_USI_SET_BUTTONS, cmd_data);
		if (ret != 0)
			G_USI_ERR("Set Buttons failed : %d", ret);

		ret = g_usi_update_capability(usi_ctx, 3);
		if (ret != 0)
			G_USI_ERR("Type Sync failed : %d", ret);

		break;
	case HID_REPORTID_GETSET_VENDOR:
		cmd_data = buf[3] << 8 | buf[2];
		ret = g_usi_send_write_cmd(usi_ctx, 1, G_USI_SET_VENDOR_EXTENSION, cmd_data);
		if (ret != 0)
			G_USI_ERR("Set VendorExtension failed : %d", ret);

		/* TODO: Get Vendor extension via diag */
		break;
	case HID_REPORTID_DIAGNOSE:
		usi_ctx->is_diagnose_requested = false;

		for (i = 0; i < 5; i++)
			usi_ctx->diagnose_uplink[i] = buf[i + 1];

		usi_ctx->is_diagnose_requested = true;
		break;

	default:
		G_USI_ERR("Set Feature Report with report ID %d is not supported", rpt_info->id);
		ret = -EINVAL;
	}

	return ret;
}

static int device_open(struct inode *inode, struct file *file)
{
	struct g_usi_context __maybe_unused *usi_ctx = container_of(file->private_data,
								    struct g_usi_context,
								    g_usi_hidraw_dev);
	return 0;
}

static int device_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long device_ioctl(struct file *file, u32 cmd, unsigned long arg)
{
	struct g_usi_context *usi_ctx = container_of(file->private_data,
						     struct g_usi_context,
						     g_usi_hidraw_dev);
	struct hid_feature_report_info *rpt_info;
	u8 *hid_buf;
	void __user *user_arg = (void __user *)arg;
	u32 hid_len;
	u32 len_arg;
	long ret = 0;
	struct hidraw_devinfo dinfo;
	u8 rpt_id;

	if (!g_usi_is_valid_context(usi_ctx))
		return -EFAULT;

	mutex_lock(&usi_ctx->lock);

	switch (cmd) {
	case HIDIOCGRDESCSIZE:
		if (put_user(sizeof(USI_report_descriptor_v2_0), (int __user *)arg))
			ret = -EFAULT;
		break;
	case HIDIOCGRDESC:
		if (get_user(len_arg, (int __user *)arg))
			ret = -EFAULT;
		else if (len_arg > HID_MAX_DESCRIPTOR_SIZE - 1)
			ret = -EINVAL;
		else if (copy_to_user(user_arg + offsetof(struct hidraw_report_descriptor,
							  value[0]),
				      USI_report_descriptor_v2_0,
				      min((u32)sizeof(USI_report_descriptor_v2_0), len_arg)))
			ret = -EFAULT;
		break;
	case HIDIOCGRAWINFO:
		dinfo.bustype = usi_ctx->id.bustype;
		dinfo.vendor = usi_ctx->id.vendor;
		dinfo.product = usi_ctx->id.product;
		if (copy_to_user(user_arg, &dinfo, sizeof(dinfo)))
			ret = -EFAULT;
		break;
	default:
		if (usi_ctx->status != G_USI_OP_IDENTIFIED) {
			G_USI_LOG("pen is not in range or identified yet\n");
			ret = 0;
			break;
		}

		hid_len = _IOC_SIZE(cmd);
		hid_buf = memdup_user(user_arg, hid_len);
		if (IS_ERR(hid_buf)) {
			ret = PTR_ERR(hid_buf);
			break;
		}

		rpt_id = hid_buf[0];

		rpt_info = get_feature_report_info(rpt_id);
		if (!rpt_info) {
			G_USI_ERR("Invalid Report ID : %d\n", rpt_id);
			ret = -EINVAL;
			goto setget_error;
		}

		if (rpt_info->size > hid_len) {
			G_USI_ERR("too small input buffer for the report(ID:%d) %d > %d\n",
				  rpt_id, rpt_info->size, hid_len);
			ret = -EINVAL;
			goto setget_error;
		}

		switch (_IOC_NR(cmd)) {
		case _IOC_NR(HIDIOCSFEATURE(0)):
			if (rpt_id == HID_REPORTID_SET_TRANSDUCER) {
				/* we only support one stylus */
				if (hid_buf[1] == 1) {
					ret = hid_len;
				} else {
					G_USI_ERR("Invalid Transducer Index : %d\n", hid_buf[1]);
					ret = -EINVAL;
				}

				break;
			}

			if (!(rpt_info->type & G_USI_HID_SET_REPORT)) {
				G_USI_ERR("Invalid Report ID for SET FEATURE %d\n", rpt_id);
				ret = -EINVAL;
				break;
			}

			ret = g_usi_hid_set_feature_report(usi_ctx, hid_buf, hid_len, rpt_info);
			if (ret == 0)
				ret = hid_len;
			else
				G_USI_ERR("cannot handle set feaure report id %d", rpt_info->id);

			break;
		case _IOC_NR(HIDIOCGFEATURE(0)):
			if (!(rpt_info->type & G_USI_HID_GET_REPORT)) {
				G_USI_ERR("Invalid Report ID for GET FEATURE %d\n", rpt_id);
				ret = -EINVAL;
				break;
			}

			ret = g_usi_hid_get_feature_report(usi_ctx, hid_buf, hid_len, rpt_info);
			if (ret == 0) {
				ret = hid_len;
			} else if (ret == -ENODATA) {
				G_USI_LOG("data for report id %d is not ready\n", rpt_info->id);
				ret = 0; /* ask retry */
			} else {
				G_USI_ERR("Error Get HID Feature Report %ld\n", ret);
				break;
			}

			if (copy_to_user(user_arg, hid_buf, ret))
				ret = -EFAULT;

			break;
		default:
			G_USI_ERR("cmd %d is not supported\n", cmd);
			ret = -EINVAL;
			break;
		}
setget_error:
		kfree(hid_buf);
	}

	mutex_unlock(&usi_ctx->lock);

	return ret;
}

static ssize_t recreate_evdev_enabled_store(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct g_usi_context *usi_ctx = container_of(dev_get_drvdata(dev), struct g_usi_context,
						     g_usi_hidraw_dev);

	if (kstrtobool(buf, &usi_ctx->recreate_evdev_enabled))
		G_USI_ERR("error: invalid input!\n");

	return count;
}

static ssize_t recreate_evdev_enabled_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	struct g_usi_context *usi_ctx = container_of(dev_get_drvdata(dev), struct g_usi_context,
						     g_usi_hidraw_dev);

	return sysfs_emit(buf, "recreate_evdev_enabled : %d\n", usi_ctx->recreate_evdev_enabled);
}

static DEVICE_ATTR_RW(recreate_evdev_enabled);

static struct attribute *g_usi_attrs[] = {
	&dev_attr_recreate_evdev_enabled.attr,
	NULL,
};

static const struct attribute_group g_usi_attrs_group = {
	.attrs = g_usi_attrs,
};

static const struct file_operations g_usi_fops = {
	.owner = THIS_MODULE,
	.open = device_open,
	.release = device_release,
	.unlocked_ioctl = device_ioctl
};

static int g_usi_create_hidraw(struct g_usi_context *usi_ctx)
{
	int ret;

	usi_ctx->g_usi_hidraw_dev.name = "g_usi_hidraw";
	usi_ctx->g_usi_hidraw_dev.mode = 0600;
	usi_ctx->g_usi_hidraw_dev.fops = &g_usi_fops;

	ret = misc_register(&usi_ctx->g_usi_hidraw_dev);
	if (ret < 0) {
		G_USI_ERR("misc_register failed %s", usi_ctx->g_usi_hidraw_dev.name);
		return ret;
	}

	ret = sysfs_create_group(&usi_ctx->g_usi_hidraw_dev.this_device->kobj, &g_usi_attrs_group);
	if (ret) {
		G_USI_ERR("failed to create attributes : %d\n", ret);
		goto error_sysfs_create_group;
	}

	return 0;

error_sysfs_create_group:
	misc_deregister(&usi_ctx->g_usi_hidraw_dev);

	return ret;
}

static void g_usi_remove_hidraw(struct g_usi_context *usi_ctx)
{
	sysfs_remove_group(&usi_ctx->g_usi_hidraw_dev.this_device->kobj, &g_usi_attrs_group);
	misc_deregister(&usi_ctx->g_usi_hidraw_dev);
}

/**
 * goog_usi_register() - register a USI device(controller)
 *
 * @usi - &struct g_usi has the information about the USI device
 *
 * Return: &typedef g_usi_handle_t if successful, G_USI_INVALID_HANDLE if not.
 */
g_usi_handle_t goog_usi_register(struct g_usi *usi)
{
	struct g_usi_context *usi_ctx;

	if (!usi || !usi->dev || !usi->cbs) {
		G_USI_ERR("Invalid arguments");
		return G_USI_INVALID_HANDLE;
	}

	usi_ctx = kzalloc(sizeof(*usi_ctx), GFP_KERNEL);
	if (!usi_ctx)
		return G_USI_INVALID_HANDLE;

	/* register power supply for stylus battery */
	usi_ctx->psy_desc.name = G_USI_BATTERY_NODE_NAME;
	usi_ctx->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	usi_ctx->psy_desc.properties = stylus_battery_props;
	usi_ctx->psy_desc.num_properties = ARRAY_SIZE(stylus_battery_props);
	usi_ctx->psy_desc.get_property = stylus_get_battery_property;
	usi_ctx->psy_cfg.drv_data = usi_ctx;

	usi_ctx->bat_psy = power_supply_register(usi->dev, &usi_ctx->psy_desc, &usi_ctx->psy_cfg);
	if (IS_ERR(usi_ctx->bat_psy)) {
		G_USI_ERR("Can't register power supply, err = %ld\n", PTR_ERR(usi_ctx->bat_psy));
		goto register_battery_error;
	}

	/* register input device for stylus */
	usi_ctx->dev = usi->dev;
	usi_ctx->name = G_USI_INPUT_NODE_NAME;
	usi_ctx->phys = G_USI_INPUT_NODE_PHYS;
	usi_ctx->abs_x_max = usi->abs_x_max;
	usi_ctx->abs_y_max = usi->abs_y_max;
	usi_ctx->distance_max = usi->distance_max;
	usi_ctx->tilt_min = usi->tilt_min;
	usi_ctx->tilt_max = usi->tilt_max;

	usi_ctx->id.bustype = usi->bustype;
	usi_ctx->id.version = G_USI_INPUT_NODE_VERSION;
	/*
	 * The default IDs are 0xFFFF/0xFFFF and later when a USI stylus is
	 * paired. We will expose the vendor/product IDs from the USI stylus.
	 */
	usi_ctx->id.vendor = 0xFFFF;
	usi_ctx->id.product = 0xFFFF;
	usi_ctx->recreate_evdev_enabled = 1;
	usi_ctx->input_dev = g_usi_register_input(usi_ctx);
	if (!usi_ctx->input_dev) {
		G_USI_ERR("Cannot create input device");
		goto register_input_error;
	}

	usi_ctx->is_flex_beacon = usi->is_flex_beacon;
	usi_ctx->cbs = usi->cbs;

	if (g_usi_create_hidraw(usi_ctx)) {
		G_USI_ERR("Cannot create usi_hidraw");
		goto create_hidraw_error;
	}

	usi_ctx->hdr.version = 1;
	usi_ctx->hdr.magic = G_USI_MAGIC;

	mutex_init(&usi_ctx->lock);

	return (g_usi_handle_t)usi_ctx;

create_hidraw_error:
	g_usi_unregister_input(usi_ctx->input_dev);

register_input_error:
	power_supply_unregister(usi_ctx->bat_psy);

register_battery_error:
	kfree(usi_ctx);

	return G_USI_INVALID_HANDLE;
}
EXPORT_SYMBOL(goog_usi_register);

/**
 * goog_usi_unregister() - unregister the USI controller
 *
 * @handle: &typedef g_usi_handle_t of the registered USI controller
 */
void goog_usi_unregister(g_usi_handle_t handle)
{
	struct g_usi_context *usi_ctx = handle_to_context(handle);

	if (!g_usi_is_valid_context(usi_ctx)) {
		G_USI_ERR("Invalid Parameters");
		return;
	}

	mutex_lock(&usi_ctx->lock);

	usi_ctx->hdr.magic = 0;
	usi_ctx->hdr.version = 0;
	g_usi_remove_hidraw(usi_ctx);
	g_usi_unregister_input(usi_ctx->input_dev);
	power_supply_unregister(usi_ctx->bat_psy);
	mutex_unlock(&usi_ctx->lock);

	kfree(usi_ctx);
}
EXPORT_SYMBOL(goog_usi_unregister);

MODULE_DESCRIPTION("Google USI Subsystem");
MODULE_AUTHOR("Hyungwoo Yang <hyungwooyang@google.com>");
MODULE_LICENSE("GPL v2");
