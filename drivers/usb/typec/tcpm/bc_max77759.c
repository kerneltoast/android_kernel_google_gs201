// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019, Google LLC
 *
 * MAX77759 BC1.2 management.
 */

#include <linux/device.h>
#include <linux/irqreturn.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec.h>
#include <misc/logbuffer.h>

#include "bc_max77759.h"
#include "max77759_helper.h"
#include "tcpci.h"
#include "tcpci_max77759.h"

struct bc12_status {
	struct workqueue_struct *wq;
	struct max77759_plat *chip;
	/*
	 * TODO: set to unknown upon disconnect and roleswap
	 * Set after ChgTypRun
	 */
	enum power_supply_usb_type usb_type;
	/* Protects changes to this structure. */
	struct mutex lock;
	struct power_supply *usb_psy;
	bool retry_done;
	/* Tracks whether BC12 is enabled */
	bool enable;
	/* Status callback */
	bc12_status_callback bc12_callback;
};

struct bc12_update {
	struct work_struct bc12_work;
	u8 vendor_alert1_status;
	u8 vendor_bc_status1;
	u8 vendor_bc_status2;
	struct bc12_status *bc12;
};

enum power_supply_usb_type get_usb_type(struct bc12_status *bc12)
{
	return bc12->usb_type;
}

/*
 * Scheduled as a work item
 */
static void vendor_bc12_alert(struct work_struct *work)
{
	struct bc12_update *update = container_of(work, struct bc12_update,
						  bc12_work);
	struct bc12_status *bc12 = update->bc12;
	struct max77759_plat *plat = bc12->chip;
	struct regmap *regmap = plat->data.regmap;
	union power_supply_propval val;
	int ret = 0;
	void *usb_psy_data = power_supply_get_drvdata(bc12->usb_psy);

	mutex_lock(&bc12->lock);
	logbuffer_log(plat->log,
		      "VENDOR_ALERT1: %#x bc_stat1: %#x bc_stat2: %#x bc12_enable:%c"
		      , update->vendor_alert1_status, update->vendor_bc_status1,
		      update->vendor_bc_status2, bc12->enable ? 'y' : 'n');

	if (!bc12->enable)
		goto exit;
	/*
	 * If Data contact Detect timeout interrupt is detected,
	 * detection could report erroneous charger type,
	 * because DP/DN have failed to make contact during detection.
	 */
	if (update->vendor_alert1_status & DCDTMOINT) {
		/*
		 * TODO: DCDtimeout at 2s. Can be 900ms as well.
		 * TODO: On failure, Retry running BC1.2 manually ONCE for every
		 * attach
		 */
		if (update->vendor_bc_status1 & DCDTMO) {
			logbuffer_log(plat->log, "BC12: DCD timeout occurred.");
			if (!bc12->retry_done && bc12->enable) {
				ret = max77759_update_bits8(regmap, VENDOR_BC_CTRL1, CHGDETMAN,
							    CHGDETMAN);
				logbuffer_log(plat->log, "BC12: Manual detection triggered: %d",
					      ret);
				bc12->retry_done = true;
			}
		}
	} else if (update->vendor_alert1_status & CHGTYPINT) {
		switch (update->vendor_bc_status1 & CHGTYP) {
		case CHGTYP_NOT_ATTACHED:
			bc12->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			logbuffer_log(plat->log, "BC12: nothing attached");
			break;
		case CHGTYP_SDP:
			bc12->usb_type = POWER_SUPPLY_USB_TYPE_SDP;
			logbuffer_log(plat->log, "BC12: SDP detected");
			break;
		case CHGTYP_CDP:
			bc12->usb_type = POWER_SUPPLY_USB_TYPE_CDP;
			logbuffer_log(plat->log, "BC12: CDP detected");
			break;
		case CHGTYP_DCP:
			bc12->usb_type = POWER_SUPPLY_USB_TYPE_DCP;
			logbuffer_log(plat->log, "BC12: DCP detected");
			break;
		default:
			bc12->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			logbuffer_log(plat->log, "BC12: unknown detected");
			break;
		}
		val.intval = bc12->usb_type;
		if (power_supply_set_property(bc12->usb_psy,
					      POWER_SUPPLY_PROP_USB_TYPE,
					      &val))
			logbuffer_log(plat->log, "BC12: usb_psy update failed");
	}

	if (update->vendor_alert1_status & CHGTYPRUNRINT) {
		logbuffer_log(plat->log, "BC12: running");
		if (bc12->bc12_callback)
			bc12->bc12_callback(bc12->chip, true);
	}
	if (update->vendor_alert1_status & CHGTYPRUNFINT) {
		logbuffer_log(plat->log, "BC12: not running");
		if (bc12->bc12_callback)
			bc12->bc12_callback(bc12->chip, false);
	}
	if (update->vendor_alert1_status & PRCHGTYPINT) {
		logbuffer_log(plat->log, "BC12: Proprietary port. Starting sdp timeout");
		usb_psy_start_sdp_timeout(usb_psy_data);
	}

exit:
	mutex_unlock(&bc12->lock);
	devm_kfree(plat->dev, update);
}

void process_bc12_alert(struct bc12_status *bc12, u8 vendor_alert1_status)
{
	struct bc12_update *update;
	struct max77759_plat *plat = bc12->chip;
	struct regmap *regmap = plat->data.regmap;

	logbuffer_log(plat->log, "Process BC12");
	update = devm_kzalloc(plat->dev, sizeof(*update), GFP_ATOMIC);
	if (!update) {
		logbuffer_log(plat->log, "BC12: No memory");
		return;
	}
	INIT_WORK(&update->bc12_work, vendor_bc12_alert);

	update->vendor_alert1_status = vendor_alert1_status;
	max77759_read8(regmap, VENDOR_BC_STATUS1, &update->vendor_bc_status1);
	max77759_read8(regmap, VENDOR_BC_STATUS2, &update->vendor_bc_status2);
	update->bc12 = bc12;

	queue_work(bc12->wq, &update->bc12_work);
}
EXPORT_SYMBOL_GPL(process_bc12_alert);

void bc12_reset_retry(struct bc12_status *bc12)
{
	bc12->retry_done = false;
}
EXPORT_SYMBOL_GPL(bc12_reset_retry);

void bc12_enable(struct bc12_status *bc12, bool enable)
{
	int ret;
	struct max77759_plat *plat = bc12->chip;
	struct regmap *regmap = plat->data.regmap;

	ret = max77759_update_bits8(regmap, VENDOR_BC_CTRL1, CHGDETEN, enable ? CHGDETEN : 0);
	logbuffer_log(plat->log, "BC12: %s ret: %d", enable ? "enabled" : "disabled", ret);
	if (!ret)
		bc12->enable = enable;
}
EXPORT_SYMBOL_GPL(bc12_enable);

bool bc12_get_status(struct bc12_status *bc12)
{
	return bc12->enable;
}
EXPORT_SYMBOL_GPL(bc12_get_status);

/*
 * Call during disconnect to clear the chg_typ
 */
void clear_chg_typ(struct bc12_status *bc12)
{
	mutex_lock(&bc12->lock);
	bc12->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	mutex_unlock(&bc12->lock);
}
EXPORT_SYMBOL_GPL(clear_chg_typ);

void bc12_teardown(struct bc12_status *bc12)
{
	power_supply_put(bc12->usb_psy);
}
EXPORT_SYMBOL_GPL(bc12_teardown);

struct bc12_status *bc12_init(struct max77759_plat *plat, bc12_status_callback callback)
{
	struct bc12_status *bc12;
	struct device *dev = plat->dev;
	struct device_node *dn;
	const char *usb_psy_name;
	int ret;
	struct regmap *regmap = plat->data.regmap;
	struct power_supply *usb_psy;

	dn = dev_of_node(plat->dev);
	if (!dn) {
		dev_err(plat->dev, "bc12: of node not found\n");
		return ERR_PTR(-EINVAL);
	}

	usb_psy_name = of_get_property(dn, "usb-psy-name", NULL);
	if (!usb_psy_name) {
		dev_err(plat->dev, "bc12: usb-psy-name not set\n");
		return ERR_PTR(-EINVAL);
	}

	usb_psy = power_supply_get_by_name(usb_psy_name);
	if (IS_ERR_OR_NULL(usb_psy)) {
		dev_err(plat->dev, "bc12: Power supply get failed");
		return ERR_PTR(-EPROBE_DEFER);
	}

	bc12 = devm_kzalloc(dev, sizeof(*bc12), GFP_KERNEL);
	if (!bc12) {
		ret = -ENOMEM;
		goto power_supply_put;
	}

	/* Enabled by default in hardware */
	bc12->enable = true;
	bc12->bc12_callback = callback;
	bc12->chip = plat;
	mutex_init(&bc12->lock);
	bc12->usb_psy = usb_psy;

	bc12->wq = create_singlethread_workqueue("max77759-bc12");
	if (!bc12->wq) {
		dev_err(plat->dev, "bc12: workqueue creation failed\n");
		ret = -ENOMEM;
		goto power_supply_put;
	}

	/* Unmask Vendor alert */
	max77759_write8(regmap, VENDOR_ALERT_MASK1, 0xff);

	return bc12;

power_supply_put:
	power_supply_put(usb_psy);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(bc12_init);

MODULE_DESCRIPTION("MAX77759_BC12 Module");
MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_LICENSE("GPL");
