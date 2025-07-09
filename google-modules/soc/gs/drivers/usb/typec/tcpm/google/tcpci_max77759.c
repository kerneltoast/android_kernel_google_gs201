// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2019, Google LLC
 *
 * MAX77759 TCPCI driver
 */

#include <linux/debugfs.h>
#include <linux/extcon.h>
#include <linux/extcon-provider.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/usb/pd.h>
#include <linux/usb/pd_vdo.h>
#include <linux/usb/tcpm.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <misc/logbuffer.h>
#include <trace/hooks/typec.h>

#include "bc_max77759.h"
#include <linux/usb/max77759_export.h>
#include "max77759_helper.h"
#include "max777x9_contaminant.h"
#include "tcpci_max77759.h"
#include "tcpci_max77759_vendor_reg.h"
#include "google_tcpci_shim.h"
#include "usb_icl_voter.h"
#include "usb_psy.h"
#include "usb_thermal_voter.h"

#define TCPCI_MODE_VOTER	"TCPCI"
#define LIMIT_SINK_VOTER	"LIMIT_SINK_CURRENT_VOTER"
#define LIMIT_ACCESSORY_VOTER	"LIMIT_ACCESSORY_CURRENT_VOTER"

#define AICL_ACTIVE_EL "AICL_ACTIVE_EL"

#define TCPC_RECEIVE_BUFFER_COUNT_OFFSET                0
#define TCPC_RECEIVE_BUFFER_FRAME_TYPE_OFFSET           1
#define TCPC_RECEIVE_BUFFER_RX_BYTE_BUF_OFFSET          2

#define TCPCI_HI_Z_CC		0xf
/*
 * LongMessage not supported, hence 32 bytes for buf to be read from RECEIVE_BUFFER.
 * DEVICE_CAPABILITIES_2.LongMessage = 0, the value in READABLE_BYTE_COUNT reg shall be
 * less than or equal to 31. Since, RECEIVE_BUFFER len = 31 + 1(READABLE_BYTE_COUNT).
 */
#define TCPC_RECEIVE_BUFFER_LEN                         32

#define PD_ACTIVITY_TIMEOUT_MS				10000
#define IO_ERROR_RETRY_MS				3000
#define VSAFE0V_DEBOUNCE_MS				15
#define VBUS_RAMPUP_TIMEOUT_MS				250
#define VBUS_RAMPUP_MAX_RETRY				8

#define GBMS_MODE_VOTABLE	"CHARGER_MODE"

/*
 * BCL_USB needs to be voted for both source and sink. bcl_usb_votable's callback can take more
 * than a msec to execute so this is invoke from its own workqueue to not block the rest of the
 * state machine.
 */
#define BCL_USB_VOTABLE		"BCL_USB"
#define BCL_USB_VOTER		"BCL_USB_VOTER"
#define BCL_USB_VOTE		0

#define MAX77759_DEVICE_ID_A1				0x2
#define MAX77759_PRODUCT_ID				0x59
#define MAX77779_PRODUCT_ID				0x79

#define MAX77759_DISABLE_TOGGLE				1
#define MAX77759_ENABLE_TOGGLE				0
/* Vote value doesn't matter. Only status matters. */
#define MAX77759_DISABLE_TOGGLE_VOTE			1

#define MAX77759_RP_MISSING_TIMEOUT_MS                 2000

#define AICL_CHECK_MS				       10000

#define EXT_BST_OVP_CLEAR_DELAY_MS		       1000

/* system use cases */
enum gbms_charger_modes {
	GBMS_USB_BUCK_ON	= 0x30,
	GBMS_USB_OTG_ON		= 0x31,
	GBMS_USB_OTG_FRS_ON	= 0x32,
};

enum bcl_usb_mode {
	USB_PLUGGED,
	USB_UNPLUGGED,
};

#define CONTAMINANT_DETECT_DISABLE	0
#define CONTAMINANT_DETECT_AP		1
#define CONTAMINANT_DETECT_MAXQ		2

#define TCPM_RESTART_TOGGLING		0
#define CONTAMINANT_HANDLES_TOGGLING	1

#define VOLTAGE_ALARM_HI_EN_MV		3000
#define VOLTAGE_ALARM_HI_DIS_MV		21000
#define VOLTAGE_ALARM_LOW_EN_MV		1500
#define VOLTAGE_ALARM_LOW_DIS_MV	0
#define VBUS_PRESENT_THRESHOLD_MV	4000

#define TCPC_ALERT_VENDOR		BIT(15)

#define FLOATING_CABLE_OR_SINK_INSTANCE_THRESHOLD	10
#define AUTO_ULTRA_LOW_POWER_MODE_REENABLE_MS		600000

#define REGMAP_REG_MAX_ADDR			0x95
#define REGMAP_REG_COUNT			(REGMAP_REG_MAX_ADDR + 1)

#define cc_open_or_toggling(cc1, cc2) \
	(((cc1) == TYPEC_CC_OPEN) && ((cc2) == TYPEC_CC_OPEN))

#define rp_3a_detected(cc1, cc2) \
	((((cc1) == TYPEC_CC_RP_3_0) && ((cc2) == TYPEC_CC_OPEN)) || \
	 (((cc1) == TYPEC_CC_OPEN) && ((cc2) == TYPEC_CC_RP_3_0)))

#define rp_1a5_detected(cc1, cc2) \
	((((cc1) == TYPEC_CC_RP_1_5) && ((cc2) == TYPEC_CC_OPEN)) || \
	 (((cc1) == TYPEC_CC_OPEN) && ((cc2) == TYPEC_CC_RP_1_5)))

#define rp_def_detected(cc1, cc2) \
	((((cc1) == TYPEC_CC_RP_DEF) && ((cc2) == TYPEC_CC_OPEN)) || \
	 (((cc1) == TYPEC_CC_OPEN) && ((cc2) == TYPEC_CC_RP_DEF)))

#define port_is_sink(cc1, cc2) \
	(rp_def_detected(cc1, cc2) || rp_1a5_detected(cc1, cc2) || rp_3a_detected(cc1, cc2))

#define is_rd_open(cc1, cc2) \
	((((cc1) == TYPEC_CC_RD) && ((cc2) == TYPEC_CC_OPEN)) || \
	 (((cc1) == TYPEC_CC_OPEN) && ((cc2) == TYPEC_CC_RD)))

#define is_rd_ra(cc1, cc2) \
	((((cc1) == TYPEC_CC_RD) && ((cc2) == TYPEC_CC_RA)) || \
	 (((cc1) == TYPEC_CC_RA) && ((cc2) == TYPEC_CC_RD)))

#define port_is_source(cc1, cc2) \
	(is_rd_open(cc1, cc2) || is_rd_ra(cc1, cc2))

#define is_debug_accessory_detected(cc1, cc2) \
	((((cc1) == TYPEC_CC_RP_DEF) || ((cc1) == TYPEC_CC_RP_1_5) || ((cc1) == TYPEC_CC_RP_3_0)) && \
	 (((cc1) == TYPEC_CC_RP_DEF) || ((cc1) == TYPEC_CC_RP_1_5) || ((cc1) == TYPEC_CC_RP_3_0)))

#define FLOATING_CABLE_INSTANCE_THRESHOLD	5
#define AUTO_ULTRA_LOW_POWER_MODE_REENABLE_MS	600000

#define VOLTAGE_DP_AUX_DEFAULT_UV	3300000

#define SRC_CURRENT_LIMIT_MA		0

#define DISCONNECT_DEBOUNCE_MS		1200

#define LOG_LVL_DEBUG				1
#define LOG_LVL_INFO				2

/*
 * Set CURRENT_LOG_LEVEL to 0 in order to disable all logging activity, else set it to desired
 * value to increase or decrease verbosity.
 */
#define CURRENT_LOG_LEVEL			LOG_LVL_DEBUG

#define LOG(LOG_LEVEL, LOG, FMT, ...)		\
do {						\
	if (LOG_LEVEL <= CURRENT_LOG_LEVEL)	\
		logbuffer_log(LOG, FMT __VA_OPT__(,) __VA_ARGS__); \
} while (0)

#define OVP_OP_RETRY	3

enum ovp_operation {
	OVP_RESET,
	OVP_ON,
	OVP_OFF
};

static struct logbuffer *tcpm_log;

static bool modparam_conf_sbu;
module_param_named(conf_sbu, modparam_conf_sbu, bool, 0644);
MODULE_PARM_DESC(conf_sbu, "Configure sbu pins");

static char boot_mode_string[64];
module_param_string(mode, boot_mode_string, sizeof(boot_mode_string), 0440);
MODULE_PARM_DESC(mode, "Android bootmode");

static u32 partner_src_caps[PDO_MAX_OBJECTS];
static unsigned int nr_partner_src_caps;
static bool port_src_pdo_updated;
static bool limit_src_cap_enable;
static u32 orig_src_current;
static unsigned int nr_orig_src_pdo;
spinlock_t g_caps_lock;

static unsigned int sink_discovery_delay_ms;

/* Callback for data_active changes */
void (*data_active_callback)(void *data_active_payload, enum typec_data_role role, bool active);
void *data_active_payload;
/* Callback for orientation changes */
void (*orientation_callback)(void *orientation_payload);
void *orientation_payload;

static void max77759_get_cc(struct max77759_plat *chip, enum typec_cc_status *cc1,
			    enum typec_cc_status *cc2);

static bool hooks_installed;

struct dp_notification_event {
	struct max77759_plat *chip;
	unsigned long mode;
	struct kthread_work dp_notification_work;
};

static const struct regmap_range max77759_tcpci_range[] = {
	regmap_reg_range(0x00, REGMAP_REG_MAX_ADDR)
};

const struct regmap_access_table max77759_tcpci_write_table = {
	.yes_ranges = max77759_tcpci_range,
	.n_yes_ranges = ARRAY_SIZE(max77759_tcpci_range),
};

static const struct regmap_config max77759_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REGMAP_REG_MAX_ADDR,
	.wr_table = &max77759_tcpci_write_table,
};

static int max77759_get_vbus_voltage_mv(struct i2c_client *tcpc_client);

static ssize_t frs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->frs);
};
static DEVICE_ATTR_RO(frs);

static ssize_t auto_discharge_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->data.auto_discharge_disconnect ? 1 : 0);
};
static DEVICE_ATTR_RO(auto_discharge);

static ssize_t bc12_enabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%d\n", bc12_get_status(chip->bc12) ? 1 : 0);
};
static DEVICE_ATTR_RO(bc12_enabled);

/* Debugfs disabled in user builds. */
static ssize_t registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	struct regmap *regmap = chip->data.regmap;
	u8 dump[REGMAP_REG_COUNT];
	int ret, offset = 0, addr;

	ret = regmap_bulk_read(regmap, 0, dump, REGMAP_REG_COUNT);
	if (ret < 0) {
		dev_err(chip->dev, "[%s]: Failed to dump ret:%d\n", __func__, ret);
		return 0;
	}

	for (addr = 0; addr < REGMAP_REG_COUNT; addr++) {
		ret = sysfs_emit_at(buf, offset, "%x: %x\n", addr, dump[addr]);
		if (!ret) {
			dev_err(chip->dev, "[%s]: Not all registers printed. last:%x\n", __func__,
				addr - 1);
			break;
		}
		offset += ret;
	}

	return offset;
};
static DEVICE_ATTR_RO(registers);

static ssize_t contaminant_detection_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->contaminant_detection_userspace);
};

static int update_contaminant_detection_locked(struct max77759_plat *chip, int val)
{

	if (!chip->contaminant)
		return -ENODEV;
	chip->contaminant_detection = val;

	if (chip->contaminant_detection)
		max777x9_enable_contaminant_detection(chip, chip->contaminant_detection ==
						      CONTAMINANT_DETECT_MAXQ);
	else
		max777x9_disable_contaminant_detection(chip);

	LOG(LOG_LVL_DEBUG, chip->log, "[%s]: %d", __func__, chip->contaminant_detection);
	return 0;
}

static ssize_t contaminant_detection_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	int val, ret;

	if (kstrtoint(buf, 10, &val) < 0)
		return -EINVAL;

	mutex_lock(&chip->rc_lock);
	ret = update_contaminant_detection_locked(chip, val);
	if (!ret)
		chip->contaminant_detection_userspace = val;
	mutex_unlock(&chip->rc_lock);
	return (ret < 0) ? ret : count;
}
static DEVICE_ATTR_RW(contaminant_detection);

static ssize_t cc_toggle_enable_show(struct device *dev, struct device_attribute *attr,
				     char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->toggle_disable_status ? 0 : 1);
};

static ssize_t cc_toggle_enable_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	int val, ret;

	if (kstrtoint(buf, 10, &val) < 0)
		return -EINVAL;

	logbuffer_logk(chip->log, LOGLEVEL_INFO, "Requesting CC toggle, cc state: curr=%s next=%s",
		       !chip->toggle_disable_status ? "on" : "off", val ? "on" : "off");

	ret = gvotable_cast_vote(chip->toggle_disable_votable, "USER_VOTE",
				 (void *)MAX77759_DISABLE_TOGGLE_VOTE, val ?
				 MAX77759_ENABLE_TOGGLE : MAX77759_DISABLE_TOGGLE);
	if (ret < 0)
		dev_err(chip->dev, "Cannot set TOGGLE DISABLE=%d (%d)\n", val, ret);

	return count;
}
static DEVICE_ATTR_RW(cc_toggle_enable);

static ssize_t non_compliant_reasons_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return compliance_warnings_to_buffer(chip->compliance_warnings, buf);
};

static DEVICE_ATTR_RO(non_compliant_reasons);

static ssize_t contaminant_detection_status_show(struct device *dev, struct device_attribute *attr,
						 char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	struct max777x9_contaminant *contaminant;

	if (!chip)
		return -EAGAIN;

	contaminant = chip->contaminant;

	if (!contaminant)
		return -EAGAIN;

	return scnprintf(buf, PAGE_SIZE, "%d\n", max777x9_is_contaminant_detected(chip));
}
static DEVICE_ATTR_RO(contaminant_detection_status);

static ssize_t usb_limit_sink_enable_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "%u\n", chip->limit_sink_enable);
};

/* usb_limit_sink_current has to be set before usb_limit_sink_enable is invoked */
static ssize_t usb_limit_sink_enable_store(struct device *dev, struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	bool enable;
	int ret;

	if (kstrtobool(buf, &enable) < 0)
		return -EINVAL;

	if (enable) {
		ret = gvotable_cast_vote(chip->usb_icl_el, LIMIT_SINK_VOTER,
					 (void *)(long)chip->limit_sink_current, true);
		if (ret < 0) {
			dev_err(chip->dev, "Cannot set sink current %d uA (%d)\n",
				chip->limit_sink_current, ret);
			goto exit;
		}
	} else {
		ret = gvotable_cast_vote(chip->usb_icl_el, LIMIT_SINK_VOTER, 0, false);
		if (ret < 0) {
			dev_err(chip->dev, "Cannot unvote for sink current (%d)\n", ret);
			goto exit;
		}
	}

	chip->limit_sink_enable = enable;

exit:
	return count;
}
static DEVICE_ATTR_RW(usb_limit_sink_enable);

static ssize_t usb_limit_sink_current_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "%u\n", chip->limit_sink_current);
};

/* limit_sink_current will not be updated if limit_sink_enable is already enabled */
static ssize_t usb_limit_sink_current_store(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int val;

	if (kstrtouint(buf, 0, &val) < 0)
		return -EINVAL;

	/* Never accept current over 3A */
	if (val > 3000000)
		return -EINVAL;

	chip->limit_sink_current = val;

	return count;
}
static DEVICE_ATTR_RW(usb_limit_sink_current);

static ssize_t usb_limit_accessory_enable_show(struct device *dev, struct device_attribute *attr,
					  char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "%u\n", chip->limit_accessory_enable);
};

/* usb_limit_accessory_current has to be set before usb_limit_accessory_enable is invoked */
static ssize_t usb_limit_accessory_enable_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	bool enable;
	int ret;

	if (kstrtobool(buf, &enable) < 0)
		return -EINVAL;

	if (enable) {
		ret = gvotable_cast_vote(chip->usb_icl_el, LIMIT_ACCESSORY_VOTER,
					 (void *)(long)chip->limit_accessory_current, true);
		if (ret < 0) {
			dev_err(chip->dev, "Cannot set accessory current %d uA (%d)\n",
				chip->limit_accessory_current, ret);
			goto exit;
		}
	} else {
		ret = gvotable_cast_vote(chip->usb_icl_el, LIMIT_ACCESSORY_VOTER, 0, false);
		if (ret < 0) {
			dev_err(chip->dev, "Cannot unvote for accessory current (%d)\n", ret);
			goto exit;
		}
	}

	chip->limit_accessory_enable = enable;

exit:
	return count;
}
static DEVICE_ATTR_RW(usb_limit_accessory_enable);

static ssize_t usb_limit_accessory_current_show(struct device *dev, struct device_attribute *attr,
						char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "%u\n", chip->limit_accessory_current);
};

static ssize_t usb_limit_accessory_current_store(struct device *dev, struct device_attribute *attr,
						 const char *buf, size_t count)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	unsigned int val;

	if (kstrtouint(buf, 0, &val) < 0)
		return -EINVAL;

	/* Never accept current over 3A */
	if (val > 3000000)
		return -EINVAL;

	chip->limit_accessory_current = val;

	return count;
}
static DEVICE_ATTR_RW(usb_limit_accessory_current);

static ssize_t sbu_pullup_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "%u\n", chip->current_sbu_state);
}

static ssize_t sbu_pullup_store(struct device *dev, struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	int val, ret = 0;
	bool enable = false;
	bool crossbar_reverse = false;

	if (kstrtoint(buf, 0, &val) < 0)
		return -EINVAL;

	switch (val) {
	case 0:
		if (chip->sbu_mux_en_gpio >= 0)
			gpio_set_value_cansleep(chip->sbu_mux_en_gpio, 0);
		gpio_set_value_cansleep(chip->sbu_mux_sel_gpio, 0);
		enable = false;
		break;
	case 1:
		if (chip->sbu_mux_en_gpio >= 0)
			gpio_set_value_cansleep(chip->sbu_mux_en_gpio, 0);
		gpio_set_value_cansleep(chip->sbu_mux_sel_gpio, 1);
		enable = false;
		break;
	case 2:
		if (chip->sbu_mux_en_gpio >= 0)
			gpio_set_value_cansleep(chip->sbu_mux_en_gpio, 1);
		gpio_set_value_cansleep(chip->sbu_mux_sel_gpio, 0);
		enable = true;
		break;
	case 3:
		if (chip->sbu_mux_en_gpio >= 0)
			gpio_set_value_cansleep(chip->sbu_mux_en_gpio, 1);
		gpio_set_value_cansleep(chip->sbu_mux_sel_gpio, 1);
		enable = true;
		crossbar_reverse = true;
		break;
	default:
		goto set_sbu_state;
	}

	if ((enable && !chip->dp_regulator_enabled) || (!enable && chip->dp_regulator_enabled)) {
		ret = enable ? regulator_enable(chip->dp_regulator) : \
			regulator_disable(chip->dp_regulator);
		if (ret >= 0)
			chip->dp_regulator_enabled = enable;
		dev_info(chip->dev, "dp regulator_%s %s ret:%d", enable ? "enable" : "disable",
				ret < 0 ? "fail" : "success", ret);
		ret = enable ? regulator_set_voltage(chip->dp_regulator, VOLTAGE_DP_AUX_DEFAULT_UV,
							VOLTAGE_DP_AUX_DEFAULT_UV) : \
				regulator_set_voltage(chip->dp_regulator, chip->dp_regulator_min_uv,
							chip->dp_regulator_max_uv);
		dev_info(chip->dev, "dp regulator_set_voltage %s ret:%d",
				ret < 0 ? "fail" : "success", ret);
	}

	if (chip->product_id == MAX77779_PRODUCT_ID) {
		ret = max77759_write8(chip->data.regmap, TCPC_VENDOR_SBUSW_CTRL, enable ?
				      (crossbar_reverse ? SBUSW_XBAR_POL_REVERSE :
				      SBUSW_XBAR_POL_NORMAL) : (modparam_conf_sbu ?
				      SBUSW_SERIAL_UART : 0));
		LOG(LOG_LVL_DEBUG, chip->log, "SBU Cross Bar SW %s %s, ret:%d",
		    enable ? "Enable" : "Disable", ret < 0 ? "fail" : "success", ret);
	} else {
		ret = max77759_write8(chip->data.regmap, TCPC_VENDOR_SBUSW_CTRL, enable ?
				      SBUSW_PATH_1 : (modparam_conf_sbu ? SBUSW_SERIAL_UART : 0));
	}
	logbuffer_logk(chip->log, LOGLEVEL_INFO, "SBU dp switch %s %s ret:%d",
		       enable ? "enable" : "disable", ret < 0 ? "fail" : "success", ret);

set_sbu_state:
	dev_info(chip->dev, "dp_debug: sbu_pullup_store: val:%d \n", val);
	if (!ret)
		chip->current_sbu_state = val;

	return count;
}
static DEVICE_ATTR_RW(sbu_pullup);

static ssize_t irq_hpd_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "%u\n", chip->irq_hpd_count);
};
static DEVICE_ATTR_RO(irq_hpd_count);

static ssize_t usb_limit_source_enable_show(struct device *dev, struct device_attribute *attr,
					    char *buf)
{
	return sysfs_emit(buf, "%u\n", limit_src_cap_enable);
}

static ssize_t usb_limit_source_enable_store(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));
	bool enable;

	if (kstrtobool(buf, &enable) < 0)
		return -EINVAL;

	spin_lock(&g_caps_lock);
	port_src_pdo_updated = false;
	limit_src_cap_enable = enable;
	spin_unlock(&g_caps_lock);

	tcpm_cc_change(chip->tcpci->port);

	return count;
}
static DEVICE_ATTR_RW(usb_limit_source_enable);

static ssize_t manual_disable_vbus_show(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct max77759_plat *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "%u\n", chip->manual_disable_vbus);
};
static DEVICE_ATTR_RO(manual_disable_vbus);

static struct device_attribute *max77759_device_attrs[] = {
	&dev_attr_frs,
	&dev_attr_bc12_enabled,
	&dev_attr_registers,
	&dev_attr_auto_discharge,
	&dev_attr_contaminant_detection,
	&dev_attr_contaminant_detection_status,
	&dev_attr_cc_toggle_enable,
	&dev_attr_non_compliant_reasons,
	&dev_attr_usb_limit_sink_enable,
	&dev_attr_usb_limit_sink_current,
	&dev_attr_usb_limit_accessory_enable,
	&dev_attr_usb_limit_accessory_current,
	&dev_attr_sbu_pullup,
	&dev_attr_usb_limit_source_enable,
	&dev_attr_irq_hpd_count,
	&dev_attr_manual_disable_vbus,
	NULL
};

void register_data_active_callback(void (*callback)(void *data_active_payload,
						    enum typec_data_role role, bool active),
				   void *data)
{
	data_active_callback = callback;
	data_active_payload = data;
}
EXPORT_SYMBOL_GPL(register_data_active_callback);

void register_orientation_callback(void (*callback)(void *orientation_payload), void *data)
{
	orientation_callback = callback;
	orientation_payload = data;
}
EXPORT_SYMBOL_GPL(register_orientation_callback);

static void ovp_operation(struct max77759_plat *chip, int operation);
#ifdef CONFIG_GPIOLIB
static int ext_bst_en_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	return GPIOF_DIR_OUT;
}

static int ext_bst_en_gpio_get(struct gpio_chip *gpio, unsigned int offset)
{
	int ret;
	u8 val;
	struct max77759_plat *chip = gpiochip_get_data(gpio);
	struct regmap *regmap = chip->data.regmap;

	ret = max77759_read8(regmap, TCPC_VENDOR_EXTBST_CTRL, &val);
	LOG(LOG_LVL_DEBUG, chip->log, "%s: ret:%d", __func__, ret);

	return val & EXT_BST_EN;
}

static void ext_bst_en_gpio_set(struct gpio_chip *gpio, unsigned int offset, int value)
{
	struct max77759_plat *chip = gpiochip_get_data(gpio);
	struct regmap *regmap = chip->data.regmap;
	bool vsafe0v, toggle_ovp = false;
	int ret;
	u8 raw;

	ret = max77759_read8(regmap, TCPC_EXTENDED_STATUS, &raw);
	if (ret < 0)
		vsafe0v = chip->vsafe0v;
	else
		vsafe0v = !!(raw & TCPC_EXTENDED_STATUS_VSAFE0V);

	/* b/309900468 toggle ovp to make sure that Vbus is vSafe0V when setting EXT_BST_EN. */
	if (chip->in_switch_gpio >= 0 && value && !vsafe0v)
		toggle_ovp = true;

	if (toggle_ovp)
		ovp_operation(chip, OVP_OFF);

	ret = max77759_write8(regmap, TCPC_VENDOR_EXTBST_CTRL, value ? EXT_BST_EN : 0);
	LOG(LOG_LVL_DEBUG, chip->log, "%s: TCPC_VENDOR_EXTBST_CTRL value:%d ret:%d", __func__,
	    value, ret);

	if (toggle_ovp) {
		mdelay(10);

		ovp_operation(chip, OVP_ON);
	}
}

static int ext_bst_en_gpio_init(struct max77759_plat *chip)
{
	int ret;

	/* Setup GPIO controller */
	chip->gpio.owner = THIS_MODULE;
	chip->gpio.parent = chip->dev;
	chip->gpio.label = "max77759_tcpc_gpio";
	chip->gpio.get_direction = ext_bst_en_gpio_get_direction;
	chip->gpio.get = ext_bst_en_gpio_get;
	chip->gpio.set = ext_bst_en_gpio_set;
	chip->gpio.base = -1;
	chip->gpio.ngpio = 1;
	chip->gpio.can_sleep = true;
	chip->gpio.of_node = of_find_node_by_name(chip->dev->of_node, chip->gpio.label);

	if (!chip->gpio.of_node)
		dev_err(chip->dev, "Failed to find %s DT node\n", chip->gpio.label);

	ret = devm_gpiochip_add_data(chip->dev, &chip->gpio, chip);
	if (ret)
		dev_err(chip->dev, "Failed to initialize gpio chip\n");

	return ret;
}
#endif

static struct max77759_plat *tdata_to_max77759(struct google_shim_tcpci_data *tdata)
{
	return container_of(tdata, struct max77759_plat, data);
}

static void max77759_init_regs(struct regmap *regmap, struct logbuffer *log)
{
	u16 alert_mask = 0;
	int ret;

	ret = max77759_write16(regmap, TCPC_ALERT, 0xffff);
	if (ret < 0)
		return;

	ret = max77759_write16(regmap, TCPC_VENDOR_ALERT, 0xffff);
	if (ret < 0)
		return;

	ret = regmap_write(regmap, TCPC_EXTENDED_STATUS_MASK,
			   TCPC_EXTENDED_STATUS_VSAFE0V);
	if (ret < 0) {
		LOG(LOG_LVL_DEBUG, log, "Error writing TCPC_EXTENDED_STATUS_MASK ret:%d", ret);
		return;
	}

	LOG(LOG_LVL_DEBUG, log, "[%s] Init EXTENDED_STATUS_MASK: VSAFE0V", __func__);

	ret = max77759_write8(regmap, TCPC_ALERT_EXTENDED, 0xff);
	if (ret < 0) {
		LOG(LOG_LVL_DEBUG, log, "Unable to clear TCPC_ALERT_EXTENDED ret:%d\n", ret);
		return;
	}

	alert_mask = TCPC_ALERT_TX_SUCCESS | TCPC_ALERT_TX_DISCARDED |
		TCPC_ALERT_TX_FAILED | TCPC_ALERT_RX_HARD_RST |
		TCPC_ALERT_RX_STATUS | TCPC_ALERT_VENDOR | TCPC_ALERT_CC_STATUS |
		TCPC_ALERT_VBUS_DISCNCT | TCPC_ALERT_RX_BUF_OVF |
		TCPC_ALERT_EXTENDED_STATUS | TCPC_ALERT_POWER_STATUS |
		/* Enable Extended alert for detecting Fast Role Swap Signal */
		TCPC_ALERT_EXTND;

	ret = max77759_write16(regmap, TCPC_ALERT_MASK, alert_mask);
	if (ret < 0)
		return;
	LOG(LOG_LVL_DEBUG, log, "[%s] Init ALERT_MASK: %u", __func__, alert_mask);

	max77759_read16(regmap, TCPC_ALERT_MASK, &alert_mask);
	LOG(LOG_LVL_DEBUG, log, "[%s] Init ALERT_MASK read : %u", __func__, alert_mask);

	/* Enable vbus voltage monitoring, voltage alerts, bleed discharge */
	ret = max77759_update_bits8(regmap, TCPC_POWER_CTRL, TCPC_POWER_CTRL_VBUS_VOLT_MON |
				    TCPC_DIS_VOLT_ALRM | TCPC_POWER_CTRL_BLEED_DISCHARGE,
				    TCPC_POWER_CTRL_BLEED_DISCHARGE);
	if (ret < 0)
		return;
	LOG(LOG_LVL_DEBUG, log,
	    "TCPC_POWER_CTRL: Enable voltage monitoring, alarm, bleed discharge");

	ret = max77759_write8(regmap, TCPC_ALERT_EXTENDED_MASK, TCPC_SINK_FAST_ROLE_SWAP);
	if (ret < 0) {
		LOG(LOG_LVL_DEBUG, log, "Unable to unmask FAST_ROLE_SWAP interrupt");
		return;
	}

	ret = max77759_update_bits8(regmap, TCPC_VENDOR_VCON_CTRL, VCNILIM_MASK, VCNILIM_300_MA);
	if (ret < 0)
		LOG(LOG_LVL_DEBUG, log, "TCPC_VENDOR_VCON_CTRL: update vcnilim to 300mA failed");
}

static int post_process_pd_message(struct max77759_plat *chip, struct pd_message msg)
{
	enum pd_ctrl_msg_type pd_type = pd_header_type_le(msg.header);

	if (pd_type == PD_DATA_VENDOR_DEF) {
		u32 payload[2];
		int i;

		for (i = 0; i < 2; i++) {
			payload[i] = le32_to_cpu(msg.payload[i]);
			if ((PD_VDO_VID(payload[0]) == USB_TYPEC_DP_SID))
				LOG(LOG_LVL_DEBUG, chip->log, "DP VDO[%d] 0x%x", i, payload[i]);
		}

		if (PD_VDO_SVDM(payload[0]) && (PD_VDO_VID(payload[0]) == USB_TYPEC_DP_SID) &&
		    ((PD_VDO_CMD(payload[0]) == CMD_ATTENTION) ||
		    (PD_VDO_CMD(payload[0]) == DP_CMD_STATUS_UPDATE)) &&
		    (payload[1] & DP_STATUS_IRQ_HPD)) {
			chip->irq_hpd_count++;
			LOG(LOG_LVL_DEBUG, chip->log, "DP IRQ_HPD:%d count:%u",
			    !!(payload[1] & DP_STATUS_IRQ_HPD), chip->irq_hpd_count);
			// sysfs_notify(&chip->dev->kobj, NULL, "irq_hpd_count");
			kobject_uevent(&chip->dev->kobj, KOBJ_CHANGE);
		}
	}

	return 0;
}

static int process_rx(struct max77759_plat *chip, u16 status)
{
	struct pd_message msg;
	u8 count, frame_type, rx_buf[TCPC_RECEIVE_BUFFER_LEN];
	int ret, payload_index;
	u8 *rx_buf_ptr;
	enum pd_ctrl_msg_type pd_type;

	/*
	 * READABLE_BYTE_COUNT: Indicates the number of bytes in the RX_BUF_BYTE_x registers
	 * plus one (for the RX_BUF_FRAME_TYPE) Table 4-36.
	 * Read the count and frame type.
	 */
	LOG(LOG_LVL_INFO, chip->log, "%d", __LINE__);
	ret = regmap_raw_read(chip->data.regmap, TCPC_RX_BYTE_CNT, rx_buf, 2);
	LOG(LOG_LVL_INFO, chip->log, "%d", __LINE__);
	if (ret < 0) {
		dev_err(chip->dev, "TCPC_RX_BYTE_CNT read failed ret:%d", ret);
		return -EIO;
	}

	count = rx_buf[TCPC_RECEIVE_BUFFER_COUNT_OFFSET];
	frame_type = rx_buf[TCPC_RECEIVE_BUFFER_FRAME_TYPE_OFFSET];

	if (count == 0 || frame_type != TCPC_RX_BUF_FRAME_TYPE_SOP) {
		ret = max77759_write16(chip->data.regmap, TCPC_ALERT, TCPC_ALERT_RX_STATUS);
		dev_err(chip->dev, "%s", count ==  0 ? "error: count is 0" :
			"error frame_type is not SOP");
		if (ret < 0)
			return -EIO;
	}

	/*
	 * 1. struct pd_message does not have RX_BUF_FRAME_TYPE.
	 * 2. READABLE_BYTE_COUNT is exclusive of itself.
	 */
	if (count > sizeof(struct pd_message) + 1 || count + 1 > TCPC_RECEIVE_BUFFER_LEN) {
		dev_err(chip->dev, "Invalid TCPC_RX_BYTE_CNT %d", count);
		return 0;
	}

	/*
	 * Read count + 1 as RX_BUF_BYTE_x is hidden and can only be read through
	 * TCPC_RX_BYTE_CNT
	 */
	count += 1;
	ret = regmap_raw_read(chip->data.regmap, TCPC_RX_BYTE_CNT, rx_buf, count);
	LOG(LOG_LVL_INFO, chip->log, "%d", __LINE__);
	if (ret < 0) {
		dev_err(chip->dev, "Error: TCPC_RX_BYTE_CNT read failed: %d", ret);
		return -EIO;
	}

	rx_buf_ptr = rx_buf + TCPC_RECEIVE_BUFFER_RX_BYTE_BUF_OFFSET;
	msg.header = cpu_to_le16(*(u16 *)rx_buf_ptr);
	rx_buf_ptr = rx_buf_ptr + sizeof(msg.header);
	for (payload_index = 0; payload_index < pd_header_cnt_le(msg.header); payload_index++,
	     rx_buf_ptr += sizeof(msg.payload[0]))
		msg.payload[payload_index] = cpu_to_le32(*(u32 *)rx_buf_ptr);

	LOG(LOG_LVL_INFO, chip->log, "%d", __LINE__);

	/*
	 * Read complete, clear RX status alert bit.
	 * Clear overflow as well if set.
	 */
	ret = max77759_write16(chip->data.regmap, TCPC_ALERT, status & TCPC_ALERT_RX_BUF_OVF ?
			       TCPC_ALERT_RX_STATUS | TCPC_ALERT_RX_BUF_OVF :
			       TCPC_ALERT_RX_STATUS);
	if (ret < 0)
		return -EIO;

	LOG(LOG_LVL_DEBUG, chip->log, "rx clear");
	pd_type = pd_header_type_le(msg.header);
	if (pd_type == PD_CTRL_PR_SWAP) {
		LOG(LOG_LVL_DEBUG, chip->log, "PD_CTRL_PR_SWAP");
		/* To prevent disconnect during PR_SWAP. */
		ret = max77759_write16(chip->data.regmap, TCPC_VBUS_SINK_DISCONNECT_THRESH, 0);
		/* TODO: tcpci->pr_swap = true; */
		if (ret < 0)
			return -EIO;
	}

	tcpm_pd_receive(chip->port, &msg);

	ret = post_process_pd_message(chip, msg);
	if (ret < 0)
		return ret;

	return 0;
}

struct max77759_compliance_warnings *init_compliance_warnings(struct max77759_plat *chip)
{
	struct max77759_compliance_warnings *compliance_warnings;

	compliance_warnings = devm_kzalloc(chip->dev, sizeof(*compliance_warnings), GFP_KERNEL);
	if (!compliance_warnings)
		return ERR_PTR(-ENOMEM);

	compliance_warnings->chip = chip;

	return compliance_warnings;
}

ssize_t compliance_warnings_to_buffer(struct max77759_compliance_warnings *compliance_warnings,
				      char *buf)
{
	memset(buf, 0, PAGE_SIZE);
	strncat(buf, "[", strlen("["));
	if (compliance_warnings->other)
		strncat(buf, "other, ", strlen("other, "));
	if (compliance_warnings->debug_accessory)
		strncat(buf, "debug-accessory, ", strlen("debug-accessory, "));
	if (compliance_warnings->bc12)
		strncat(buf, "bc12, ", strlen("bc12, "));
	if (compliance_warnings->missing_rp)
		strncat(buf, "missing_rp, ", strlen("missing_rp, "));
	if (compliance_warnings->input_power_limited)
		strncat(buf, "input_power_limited, ", strlen("input_power_limited, "));
	strncat(buf, "]", strlen("]"));
	return strnlen(buf, PAGE_SIZE);
}

void update_compliance_warnings(struct max77759_plat *chip, int warning, bool value)
{
	bool compliance_warnings_changed = false;

	switch (warning) {
	case COMPLIANCE_WARNING_OTHER:
		compliance_warnings_changed = (chip->compliance_warnings->other != value);
		chip->compliance_warnings->other = value;
		break;
	case COMPLIANCE_WARNING_DEBUG_ACCESSORY:
		compliance_warnings_changed = (chip->compliance_warnings->debug_accessory != value);
		chip->compliance_warnings->debug_accessory = value;
		break;
	case COMPLIANCE_WARNING_BC12:
		compliance_warnings_changed = (chip->compliance_warnings->bc12 != value);
		chip->compliance_warnings->bc12 = value;
		break;
	case COMPLIANCE_WARNING_MISSING_RP:
		compliance_warnings_changed = (chip->compliance_warnings->missing_rp != value);
		chip->compliance_warnings->missing_rp = value;
		break;
	case COMPLIANCE_WARNING_INPUT_POWER_LIMITED:
		compliance_warnings_changed =
				(chip->compliance_warnings->input_power_limited != value);
		chip->compliance_warnings->input_power_limited = value;
		break;
	}

	if (compliance_warnings_changed) {
		kobject_uevent(&chip->dev->kobj, KOBJ_CHANGE);
		LOG(LOG_LVL_DEBUG, chip->log,
		    "compliance warning %d changed, new value: %d", warning, value);
	}
}

static void max77759_non_compliant_bc12_callback(void *data, bool status)
{
	struct max77759_plat *chip = data;

	/* Exclude Rp-1.5 or higher power sources */
	if ((status && !(chip->cc1 == TYPEC_CC_RP_3_0 || chip->cc1 == TYPEC_CC_RP_1_5 ||
			chip->cc2 == TYPEC_CC_RP_3_0 || chip->cc2 == TYPEC_CC_RP_1_5)) || !status)
		update_compliance_warnings(chip, COMPLIANCE_WARNING_BC12, status);
}

static void enable_dp_pulse(struct max77759_plat *chip)
{
	struct regmap *regmap = chip->data.regmap;
	int ret;

	ret = max77759_update_bits8(regmap, VENDOR_BC_CTRL2, DPDNMAN | DPDRV,
				    DPDNMAN | DPDRV_3V0 << DPDRV_SHIFT);
	if (ret < 0)
		LOG(LOG_LVL_DEBUG, chip->log, "%s failed to set dpDnMan and dpDrv", __func__);

	mdelay(100);

	ret = max77759_update_bits8(regmap, VENDOR_BC_CTRL2, DPDNMAN | DPDRV,
				    DPDRV_OPEN << DPDRV_SHIFT);
	if (ret < 0)
		LOG(LOG_LVL_DEBUG, chip->log, "%s failed to disable dpDnMan and dpDrv", __func__);
}

void enable_data_path_locked(struct max77759_plat *chip)
{
	int ret;
	bool enable_data = false;
	struct regmap *regmap = chip->data.regmap;

	if (chip->force_device_mode_on) {
		LOG(LOG_LVL_DEBUG, chip->log,
		    "%s skipping as force_device_mode_on is set", __func__);
		return;
	}

	enable_data = ((chip->pd_data_capable || chip->no_bc_12 || chip->bc12_data_capable ||
		       chip->debug_acc_connected) && !chip->bc12_running) ||
		       chip->data_role == TYPEC_HOST;

	logbuffer_logk(chip->log, LOGLEVEL_INFO,
		       "pd_data_capable:%u no_bc_12:%u bc12_data_capable:%u attached:%u debug_acc:%u bc12_running:%u data_active:%u",
		       chip->pd_data_capable ? 1 : 0, chip->no_bc_12 ? 1 : 0,
		       chip->bc12_data_capable ? 1 : 0, chip->attached ? 1 : 0,
		       chip->debug_acc_connected, chip->bc12_running ? 1 : 0,
		       chip->data_active ? 1 : 0);

	if (chip->attached && enable_data && !chip->data_active) {
		/* Disable BC1.2 to prevent BC1.2 detection during PR_SWAP */
		bc12_enable(chip->bc12, false);
		/*
		 * Clear running flag here as PD might have configured data
		 * before BC12 started to run.
		 */
		chip->bc12_running = false;

		if (chip->alt_path_active) {
			LOG(LOG_LVL_DEBUG, chip->log, "%s skipping enabling as alt path is active",
			    __func__);
			/* Enable switch for Host mode because alt_path works for Host Mode only */
			if (chip->data_role == TYPEC_HOST) {
				ret = max77759_write8(regmap, TCPC_VENDOR_USBSW_CTRL,
						      USBSW_CONNECT);
				LOG(LOG_LVL_DEBUG, chip->log, "Turning on dp switches %s", ret < 0 ?
				    "fail" : "success");
			}

			chip->active_data_role = chip->data_role;
			if (data_active_callback)
				(*data_active_callback)(data_active_payload, chip->data_role, true);
			return;
		}

		/*
		 * b/188614064: While swapping from host to device switches will not be configured
		 * by HW. So always enable the switches here.
		 */
		ret = max77759_write8(regmap, TCPC_VENDOR_USBSW_CTRL, USBSW_CONNECT);
		LOG(LOG_LVL_DEBUG, chip->log,
		    "Turning on dp switches %s", ret < 0 ? "fail" : "success");

		if (get_usb_type(chip->bc12) == POWER_SUPPLY_USB_TYPE_CDP &&
		    !chip->pd_data_capable) {
			LOG(LOG_LVL_DEBUG, chip->log, "CDP detected, gen dp pulse");
				enable_dp_pulse(chip);
		}

		ret = extcon_set_state_sync(chip->extcon, chip->data_role == TYPEC_HOST ?
					    EXTCON_USB_HOST : EXTCON_USB, 1);
		logbuffer_logk(chip->log, LOGLEVEL_INFO, "%s turning on %s",
			       ret < 0 ? "Failed" : "Succeeded",
			       chip->data_role == TYPEC_HOST ? "Host" : "Device");
		chip->data_active = true;
		chip->active_data_role = chip->data_role;
		if (data_active_callback)
			(*data_active_callback)(data_active_payload, chip->active_data_role, true);
	} else if (chip->data_active && (!chip->attached || !enable_data)) {
		if (chip->alt_path_active) {
			LOG(LOG_LVL_DEBUG, chip->log, "%s skipping turning off as alt path is active",
			    __func__);
			if (data_active_callback)
				(*data_active_callback)(data_active_payload,
							chip->active_data_role, false);
			return;
		}

		ret = extcon_set_state_sync(chip->extcon, chip->active_data_role == TYPEC_HOST ?
					    EXTCON_USB_HOST : EXTCON_USB, 0);
		logbuffer_logk(chip->log, LOGLEVEL_INFO, "%s turning off %s",
			       ret < 0 ? "Failed" : "Succeeded",
			       chip->active_data_role == TYPEC_HOST ? "Host" : "Device");
		chip->data_active = false;
		if (data_active_callback)
			(*data_active_callback)(data_active_payload, chip->active_data_role, false);
		if  (chip->active_data_role == TYPEC_HOST) {
			ret = max77759_write8(regmap, TCPC_VENDOR_USBSW_CTRL, USBSW_DISCONNECT);
			LOG(LOG_LVL_DEBUG, chip->log,
			    "Turning off dp switches %s", ret < 0 ? "fail" : "success");
		}
	}
}
EXPORT_SYMBOL_GPL(enable_data_path_locked);

void data_alt_path_active(struct max77759_plat *chip, bool active)
{
	chip->alt_path_active = active;
}
EXPORT_SYMBOL_GPL(data_alt_path_active);

static void max777x9_bcl_usb_update(struct max77759_plat *chip, enum bcl_usb_mode mode)
{
	if (!IS_ERR_OR_NULL(chip->bcl_usb_wq)) {
		chip->bcl_usb_vote = mode;
		kthread_mod_delayed_work(chip->bcl_usb_wq, &chip->bcl_usb_votable_work,
					 msecs_to_jiffies(0));
	}
}

static void max77759_force_discharge(struct max77759_plat *chip, bool enable)
{
	struct google_shim_tcpci *tcpci = chip->tcpci;
	u8 pwr_ctrl;
	int ret;

	ret = max77759_read8(tcpci->regmap, TCPC_POWER_CTRL, &pwr_ctrl);
	LOG(LOG_LVL_DEBUG, chip->log, "%s: FORCE_DISCHARGE %u -> %u, ret %d", __func__,
	    !!(pwr_ctrl & TCPC_POWER_CTRL_FORCE_DISCHARGE), enable, ret);
	ret = max77759_update_bits8(chip->data.regmap, TCPC_POWER_CTRL,
				    TCPC_POWER_CTRL_FORCE_DISCHARGE,
				    enable ? TCPC_POWER_CTRL_FORCE_DISCHARGE : 0);
	if (ret < 0)
		LOG(LOG_LVL_DEBUG, chip->log, "%s force discharge failed",
		    enable ? "enabling" : "disabling");
}

static void enable_vbus_work(struct kthread_work *work)
{
	struct max77759_plat *chip  =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct max77759_plat, enable_vbus_work);
	int ret;

	LOG(LOG_LVL_DEBUG, chip->log, "%s", __func__);
	if (IS_ERR_OR_NULL(chip->charger_mode_votable)) {
		chip->charger_mode_votable = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
		if (IS_ERR_OR_NULL(chip->charger_mode_votable)) {
			logbuffer_logk(chip->log, LOGLEVEL_ERR,
			    "ERR: GBMS_MODE_VOTABLE lazy get failed with error %ld",
			    PTR_ERR(chip->charger_mode_votable));
			return;
		}
	}

	ret = gvotable_cast_vote(chip->charger_mode_votable, TCPCI_MODE_VOTER,
				 chip->no_external_boost ? (void *)GBMS_USB_OTG_FRS_ON :
				 (void *)GBMS_USB_OTG_ON, true);

	logbuffer_logk(chip->log, LOGLEVEL_INFO, "%s: GBMS_MODE_VOTABLE voting source ret:%d",
	    ret < 0 ? "Error" : "Success", ret);

	if (ret < 0)
		return;

	max777x9_bcl_usb_update(chip, USB_PLUGGED);

	if (!chip->sourcing_vbus)
		chip->sourcing_vbus = 1;
}

static int max77759_set_vbus(struct google_shim_tcpci *tcpci, struct google_shim_tcpci_data *tdata,
			     bool source, bool sink)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);
	int ret;

	if (source && sink) {
		LOG(LOG_LVL_DEBUG, chip->log, "ERR: both source and sink set. Not voting");
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(chip->charger_mode_votable)) {
		chip->charger_mode_votable = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
		if (IS_ERR_OR_NULL(chip->charger_mode_votable)) {
			LOG(LOG_LVL_DEBUG, chip->log,
			    "ERR: GBMS_MODE_VOTABLE lazy get failed with error %ld",
			    PTR_ERR(chip->charger_mode_votable));
			return 0;
		}
	}
	kthread_flush_work(&chip->enable_vbus_work.work);

	if (source && !sink) {
		if (chip->manual_disable_vbus)
			/* ensure force_discharge cleared before enabling vbus */
			max77759_force_discharge(chip, false);
		kthread_mod_delayed_work(chip->wq, &chip->enable_vbus_work, 0);
		return 0;
	} else if (sink && !source) {
		if (chip->manual_disable_vbus)
			/* ensure force_discharge cleared before buck on */
			max77759_force_discharge(chip, false);
		ret = gvotable_cast_vote(chip->charger_mode_votable, TCPCI_MODE_VOTER,
					 (void *)GBMS_USB_BUCK_ON, true);
		max777x9_bcl_usb_update(chip, USB_PLUGGED);
	} else {
		/* just one will do */
		ret = gvotable_cast_vote(chip->charger_mode_votable, TCPCI_MODE_VOTER,
					 (void *)GBMS_USB_BUCK_ON, false);
		max777x9_bcl_usb_update(chip, USB_UNPLUGGED);
	}

	LOG(LOG_LVL_DEBUG, chip->log, "%s: GBMS_MODE_VOTABLE voting source:%c sink:%c ret:%d",
	    ret < 0 ? "Error" : "Success", source ? 'y' : 'n', sink ? 'y' : 'n', ret);

	if (ret < 0)
		return ret;

	if (!source && chip->sourcing_vbus) {
		chip->sourcing_vbus = 0;
		chip->vbus_present = 0;
		LOG(LOG_LVL_DEBUG, chip->log,
		    "[%s]: vbus_present %d", __func__, chip->vbus_present);
		tcpm_vbus_change(tcpci->port);
	}

	return 0;
}

static void max77759_frs_sourcing_vbus(struct google_shim_tcpci *tcpci,
				       struct google_shim_tcpci_data *tdata)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);
	int ret;

	kthread_flush_work(&chip->enable_vbus_work.work);

	if (IS_ERR_OR_NULL(chip->charger_mode_votable)) {
		chip->charger_mode_votable = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
		if (IS_ERR_OR_NULL(chip->charger_mode_votable)) {
			LOG(LOG_LVL_DEBUG, chip->log,
			    "ERR: GBMS_MODE_VOTABLE lazy get failed with error %ld",
			    PTR_ERR(chip->charger_mode_votable));
			return;
		}
	}

	ret = gvotable_cast_vote(chip->charger_mode_votable, TCPCI_MODE_VOTER,
				 (void *)GBMS_USB_OTG_FRS_ON, true);
	LOG(LOG_LVL_DEBUG, chip->log, "%s: GBMS_MODE_VOTABLE ret:%d", __func__, ret);

	if (!ret)
		chip->sourcing_vbus = 1;

	/*
	 * TODO: move this line to max77759_set_vbus after the change in TCPM gets upstreamed and
	 * cherry-picked to Pixel codebase.
	 * Be sure to ensure that this will only be called during FR_SWAP.
	 */
	usb_psy_set_sink_state(chip->usb_psy_data, false);
}

static void vsafe0v_debounce_work(struct kthread_work *work)
{
	struct max77759_plat *chip  =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct max77759_plat, vsafe0v_work);
	struct google_shim_tcpci *tcpci = chip->tcpci;

	/* update to TCPM only if it is still Vsafe0V */
	if (!chip->vsafe0v)
		return;

	chip->vbus_present = 0;
	LOG(LOG_LVL_DEBUG, chip->log, "[%s]: vsafe0v debounced, vbus_present 0", __func__);
	tcpm_vbus_change(tcpci->port);
}

void disconnect_missing_rp_partner(struct max77759_plat *chip)
{
	union power_supply_propval val;
	int ret;

	LOG(LOG_LVL_DEBUG, chip->log, "Disconnect missing Rp partner");
	val.intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	max77759_set_vbus(chip->tcpci, chip->tcpci->data, false, false);
	update_compliance_warnings(chip, COMPLIANCE_WARNING_MISSING_RP, false);
	/*
	 * clear AICL warning for missing rp as detach will not be signalled for
	 * MISSING_RP + INPUT_POWER_LIMITED(AICL)
	 */
	update_compliance_warnings(chip, COMPLIANCE_WARNING_INPUT_POWER_LIMITED, false);
	chip->vbus_mv = 0;
	val.intval = 0;
	ret = power_supply_set_property(chip->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &val);
	if (ret < 0)
		LOG(LOG_LVL_DEBUG, chip->log,
		    "unable to set max voltage to %d, ret=%d", chip->vbus_mv, ret);
	if (power_supply_set_property(chip->usb_psy, POWER_SUPPLY_PROP_USB_TYPE, &val))
		LOG(LOG_LVL_DEBUG, chip->log, "missing_rp: usb_psy set unknown failed");
	usb_psy_set_sink_state(chip->usb_psy_data, false);
}

static void bcl_usb_vote_work(struct kthread_work *work)
{
	int ret;
	struct max77759_plat *chip  =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct max77759_plat, bcl_usb_votable_work);

	if (IS_ERR_OR_NULL(chip->bcl_usb_votable))
		chip->bcl_usb_votable = gvotable_election_get_handle(BCL_USB_VOTABLE);

	if (chip->bcl_usb_votable) {
		ret = gvotable_cast_vote(chip->bcl_usb_votable, BCL_USB_VOTER,
		                         (void *)BCL_USB_VOTE, chip->bcl_usb_vote);
		LOG(LOG_LVL_DEBUG, chip->log, "bcl_usb_vote: %d : %d", ret, chip->bcl_usb_vote);
	}
}

static void check_missing_rp_work(struct kthread_work *work)
{
	struct max77759_plat *chip  =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct max77759_plat, check_missing_rp_work);
	union power_supply_propval val;
	unsigned int pwr_status;
	enum typec_cc_status cc1, cc2;
	ktime_t now = ktime_get_boottime();
	int ret;
	bool first_toggle_debounce = now - chip->first_toggle_time_since_boot >=
				     ms_to_ktime(MAX77759_RP_MISSING_TIMEOUT_MS);

	if (chip->first_toggle || !first_toggle_debounce) {
		kthread_mod_delayed_work(chip->wq, &chip->check_missing_rp_work,
					 msecs_to_jiffies(MAX77759_RP_MISSING_TIMEOUT_MS));
		LOG(LOG_LVL_DEBUG, chip->log, "Delaying Missing Rp Work. Initial port reset is not"
		    " complete yet and port hasn't started to toggle");
		return;
	}

	ret = regmap_read(chip->data.regmap, TCPC_POWER_STATUS, &pwr_status);
	if (ret < 0) {
		LOG(LOG_LVL_DEBUG, chip->log, "Abort %s; TCPC_POWER_STATUS read error", __func__);
		return;
	}

	max77759_get_cc(chip, &cc1, &cc2);

	if (!!(pwr_status & TCPC_POWER_STATUS_VBUS_PRES) &&
	    (cc_open_or_toggling(cc1, cc2) ||
	    (cc1 == TYPEC_CC_RP_DEF && cc2 == TYPEC_CC_RP_DEF)) &&
	    !chip->compliance_warnings->missing_rp) {
		LOG(LOG_LVL_DEBUG, chip->log,
		    "%s: Missing or incorrect Rp partner detected. Enable WAR", __func__);
		/* Assume DCP for missing Rp non-compliant power source */
		val.intval = POWER_SUPPLY_USB_TYPE_DCP;
		max77759_set_vbus(chip->tcpci, chip->tcpci->data, false, true);
		if (power_supply_set_property(chip->usb_psy, POWER_SUPPLY_PROP_USB_TYPE, &val))
			LOG(LOG_LVL_DEBUG, chip->log, "%s: usb_psy set dcp failed", __func__);
		chip->vbus_mv = 5000;
		val.intval = chip->vbus_mv * 1000;
		ret = power_supply_set_property(chip->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &val);
		if (ret < 0)
			LOG(LOG_LVL_DEBUG, chip->log, "%s: unable to set max voltage to %d, ret=%d",
			    __func__, chip->vbus_mv * 1000, ret);
		update_compliance_warnings(chip, COMPLIANCE_WARNING_MISSING_RP, true);
		usb_psy_set_sink_state(chip->usb_psy_data, true);
	} else if (chip->compliance_warnings->missing_rp) {
		if (!(pwr_status & TCPC_POWER_STATUS_VBUS_PRES))
			disconnect_missing_rp_partner(chip);
	}
}

static void check_missing_rp(struct max77759_plat *chip, bool vbus_present,
			     enum typec_cc_status cc1, enum typec_cc_status cc2)
{
	unsigned int pwr_status;
	int ret;

	ret = regmap_read(chip->data.regmap, TCPC_POWER_STATUS, &pwr_status);
	if (ret < 0) {
		LOG(LOG_LVL_DEBUG, chip->log, "Abort %s; TCPC_POWER_STATUS read error", __func__);
		return;
	}

	if (!!(pwr_status & TCPC_POWER_STATUS_VBUS_PRES) && cc_open_or_toggling(cc1, cc2)) {
		kthread_mod_delayed_work(chip->wq, &chip->check_missing_rp_work,
					 msecs_to_jiffies(MAX77759_RP_MISSING_TIMEOUT_MS));
	} else if (chip->compliance_warnings->missing_rp) {
		kthread_cancel_delayed_work_sync(&chip->check_missing_rp_work);
		if (!(pwr_status & TCPC_POWER_STATUS_VBUS_PRES))
			disconnect_missing_rp_partner(chip);
	}
}

/* Clears EXTBST_CTRL when ovp condition is detected while sourcing vbus */
static bool check_and_clear_ext_bst(struct max77759_plat *chip)
{
	unsigned int pwr_status;
	u16 vbus_mv;
	bool ret = false;

	mutex_lock(&chip->ext_bst_ovp_clear_lock);
	regmap_read(chip->data.regmap, TCPC_POWER_STATUS, &pwr_status);
	vbus_mv = max77759_get_vbus_voltage_mv(chip->client);
	LOG(LOG_LVL_DEBUG, chip->log, "sourcing_vbus_high:%d vbus_mv:%u",
	    !!(pwr_status & TCPC_POWER_STATUS_SRC_HI_VOLT), vbus_mv);

	if (chip->sourcing_vbus_high) {
		ret = true;
		goto ext_bst_ovp_clear_unlock;
	}

	if ((pwr_status & TCPC_POWER_STATUS_SRC_HI_VOLT) && chip->sourcing_vbus &&
	    vbus_mv > chip->ext_bst_ovp_clear_mv) {
		LOG(LOG_LVL_DEBUG, chip->log, "%s: clear TCPC_VENDOR_EXTBST_CTRL", __func__);
		ret = max77759_write8(chip->tcpci->regmap, TCPC_VENDOR_EXTBST_CTRL, 0);
		chip->sourcing_vbus_high = 1;
		tcpm_vbus_change(chip->tcpci->port);
		ret = true;
		goto ext_bst_ovp_clear_unlock;
	}

ext_bst_ovp_clear_unlock:
	mutex_unlock(&chip->ext_bst_ovp_clear_lock);
	return ret;
}

/*
 * Rechecks vbus ovp condition after a delay as POWER_STATUS_SRC_HI_VOLT is set whenever vbus
 * voltage exceeds VSAFE5V(MAX). To avoid false positives when acting as source, vbus voltage
 * is checked to see whether it exceeds ext-bst-ovp-clear-mv. The check is re-run after a
 * delay as external voltage applied does not get reflected in the vbus voltage readings
 * right away when POWER_STATUS_SRC_HI_VOLT is set.
 */
static void ext_bst_ovp_clear_work(struct kthread_work *work)
{
	struct max77759_plat *chip  =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct max77759_plat, ext_bst_ovp_clear_work);

	if (chip->ext_bst_ovp_clear_mv)
		check_and_clear_ext_bst(chip);
}

static void process_power_status(struct max77759_plat *chip)
{
	struct google_shim_tcpci *tcpci = chip->tcpci;
	struct logbuffer *log = chip->log;
	unsigned int pwr_status;
	int ret;

	ret = regmap_read(tcpci->regmap, TCPC_POWER_STATUS, &pwr_status);
	LOG(LOG_LVL_DEBUG, log, "TCPC_ALERT_POWER_STATUS status:0x%x", pwr_status);
	if (ret < 0)
		return;

	if (pwr_status == 0xff) {
		max77759_init_regs(tcpci->regmap, log);
		return;
	}

	if (pwr_status & TCPC_POWER_STATUS_SOURCING_VBUS) {
		if (!(pwr_status & TCPC_POWER_STATUS_VBUS_PRES)) {
			/*
			 * Sourcing vbus might be set before vbus present is
			 * set. This implies vbus has not reached VSAFE5V yet
			 * (or) TCPC_POWER_STATUS_VBUS_PRES is arriving late.
			 * Hold back signalling sourcing vbus here.
			 */
			LOG(LOG_LVL_DEBUG, log, "Discard sourcing vbus. Vbus present not set");
		} else {
			chip->sourcing_vbus = 1;
			tcpm_sourcing_vbus(tcpci->port);
			chip->in_frs = false;
		}
	}

	if ((pwr_status & TCPC_POWER_STATUS_SRC_HI_VOLT) && chip->sourcing_vbus &&
	    chip->ext_bst_ovp_clear_mv)
		if (!check_and_clear_ext_bst(chip))
			kthread_mod_delayed_work(chip->wq, &chip->ext_bst_ovp_clear_work,
						 msecs_to_jiffies(EXT_BST_OVP_CLEAR_DELAY_MS));

	if (chip->in_frs) {
		chip->in_frs = false;
		/*
		 * While in FRS transition consider vbus present as a signal for
		 * sourcing vbus as controller would have reversed the direction
		 * here. This signal could arrive before or after
		 * TCPC_POWER_STATUS_SOURCING_VBUS
		 */
		if (pwr_status & TCPC_POWER_STATUS_VBUS_PRES) {
			chip->sourcing_vbus = 1;
			tcpm_sourcing_vbus(tcpci->port);
		}
	}

	if (pwr_status & TCPC_POWER_STATUS_VBUS_PRES)
		chip->vbus_present = 1;
	else if (!chip->data.auto_discharge_disconnect && !(pwr_status &
							    TCPC_POWER_STATUS_VBUS_PRES))
		chip->vbus_present = 0;
	LOG(LOG_LVL_DEBUG, chip->log, "[%s]: vbus_present %d", __func__, chip->vbus_present);
	tcpm_vbus_change(tcpci->port);
	/*
	 * Check for missing-rp non compliant power source.
	 * Skip when usb is throttled due to overheat.
	 */
	if (!chip->usb_throttled && !chip->toggle_disable_status)
		check_missing_rp(chip, !!(pwr_status & TCPC_POWER_STATUS_VBUS_PRES), chip->cc1,
				 chip->cc2);

	if (chip->quick_ramp_vbus_ovp && chip->vbus_present) {
		kthread_cancel_delayed_work_sync(&chip->reset_ovp_work);
		chip->reset_ovp_retry = 0;
	}

	/* TODO: remove this cc event b/211341677 */
	if (!strncmp(boot_mode_string, "charger", strlen("charger")) && chip->vbus_present) {
		dev_info(chip->dev, "WA: trigger cc event in charger mode");
		tcpm_cc_change(tcpci->port);
	}

	/*
	 * Enable data path when TCPC signals sink debug accesssory connected
	 * and disable when disconnected.
	 */
	if ((!chip->debug_acc_connected && (pwr_status & TCPC_POWER_STATUS_DBG_ACC_CON)) ||
	    (chip->debug_acc_connected && !(pwr_status & TCPC_POWER_STATUS_DBG_ACC_CON))) {
		mutex_lock(&chip->data_path_lock);
		chip->debug_acc_connected = pwr_status & TCPC_POWER_STATUS_DBG_ACC_CON ? 1 : 0;
		chip->data_role = TYPEC_DEVICE;
		/*
		 * Renable BC1.2 upon disconnect if disabled. Needed for
		 * sink-only mode such as fastbootd/Recovery.
		 */
		if (chip->attached && !chip->debug_acc_connected && !bc12_get_status(chip->bc12))
			bc12_enable(chip->bc12, true);
		chip->attached = chip->debug_acc_connected;
		enable_data_path_locked(chip);
		mutex_unlock(&chip->data_path_lock);

		/* Log Debug Accessory to Device Compliance Warnings, or Remove from List. */
		update_compliance_warnings(chip, COMPLIANCE_WARNING_DEBUG_ACCESSORY,
					   chip->debug_acc_connected);

		LOG(LOG_LVL_DEBUG, log,
		    "Debug accessory %s", chip->debug_acc_connected ? "connected" : "disconnected");
		if (!chip->debug_acc_connected && modparam_conf_sbu) {
			ret = max77759_write8(tcpci->regmap, TCPC_VENDOR_SBUSW_CTRL,
					      SBUSW_SERIAL_UART);
			LOG(LOG_LVL_DEBUG, log,
			    "SBU switch enable %s", ret < 0 ? "fail" : "success");
		}
		usb_psy_set_attached_state(chip->usb_psy_data, chip->attached);
	}
}

static void process_tx(struct google_shim_tcpci *tcpci, u16 status, struct logbuffer *log)
{
	if (status & TCPC_ALERT_TX_SUCCESS) {
		LOG(LOG_LVL_DEBUG, log, "TCPC_ALERT_TX_SUCCESS");
		tcpm_pd_transmit_complete(tcpci->port, TCPC_TX_SUCCESS);
	} else if (status & TCPC_ALERT_TX_DISCARDED) {
		LOG(LOG_LVL_DEBUG, log, "TCPC_ALERT_TX_DISCARDED");
		tcpm_pd_transmit_complete(tcpci->port, TCPC_TX_DISCARDED);
	} else if (status & TCPC_ALERT_TX_FAILED) {
		LOG(LOG_LVL_DEBUG, log, "TCPC_ALERT_TX_FAILED");
		tcpm_pd_transmit_complete(tcpci->port, TCPC_TX_FAILED);
	}

	/* Reinit regs as Hard reset sets them to default value */
	if ((status & TCPC_ALERT_TX_SUCCESS) && (status &
						 TCPC_ALERT_TX_FAILED))
		max77759_init_regs(tcpci->regmap, log);
}

static int max77759_enable_voltage_alarm(struct max77759_plat *chip, bool enable, bool high)
{
	int ret;

	if (!enable) {
		ret = max77759_update_bits8(chip->tcpci->regmap, TCPC_POWER_CTRL,
					    TCPC_DIS_VOLT_ALRM, TCPC_DIS_VOLT_ALRM);
		if (ret < 0)
			LOG(LOG_LVL_DEBUG, chip->log,
			    "Unable to disable voltage alarm, ret = %d", ret);
		return ret;
	}

	/* Set voltage alarm */
	ret = max77759_update_bits16(chip->tcpci->regmap, TCPC_VBUS_VOLTAGE_ALARM_HI_CFG,
				     TCPC_VBUS_VOLTAGE_MASK,
				     (high ? VOLTAGE_ALARM_HI_EN_MV : VOLTAGE_ALARM_HI_DIS_MV) /
				     TCPC_VBUS_VOLTAGE_LSB_MV);
	if (ret < 0) {
		LOG(LOG_LVL_DEBUG, chip->log,
		    "Unable to config VOLTAGE_ALARM_HI_CFG, ret = %d", ret);
		return ret;
	}

	ret = max77759_update_bits16(chip->tcpci->regmap, TCPC_VBUS_VOLTAGE_ALARM_LO_CFG,
				     TCPC_VBUS_VOLTAGE_MASK,
				     (!high ? VOLTAGE_ALARM_LOW_EN_MV : VOLTAGE_ALARM_LOW_DIS_MV) /
				     TCPC_VBUS_VOLTAGE_LSB_MV);
	if (ret < 0) {
		LOG(LOG_LVL_DEBUG, chip->log,
		    "Unable to config VOLTAGE_ALARM_LO_CFG, ret = %d", ret);
		return ret;
	}

	ret = max77759_update_bits8(chip->tcpci->regmap, TCPC_POWER_CTRL, TCPC_DIS_VOLT_ALRM, 0);
	if (ret < 0) {
		LOG(LOG_LVL_DEBUG, chip->log, "Unable to enable voltage alarm, ret = %d", ret);
		return ret;
	}

	ret = max77759_update_bits16(chip->tcpci->regmap, TCPC_ALERT_MASK,
				     TCPC_ALERT_V_ALARM_LO | TCPC_ALERT_V_ALARM_HI,
				     high ? TCPC_ALERT_V_ALARM_HI : TCPC_ALERT_V_ALARM_LO);
	if (ret < 0)
		LOG(LOG_LVL_DEBUG, chip->log,
		    "Unable to unmask voltage alarm interrupt, ret = %d", ret);

	return ret;
}

static int max77759_get_vbus_voltage_mv(struct i2c_client *tcpc_client)
{
	u16 raw;
	int ret;
	struct max77759_plat *chip = i2c_get_clientdata(tcpc_client);

	if (!chip || !chip->tcpci || !chip->tcpci->regmap)
		return -EAGAIN;

	/* TCPC_POWER_CTRL_VBUS_VOLT_MON enabled in init_regs */
	ret = max77759_read16(chip->tcpci->regmap, TCPC_VBUS_VOLTAGE, &raw);

	return ret ? 0 : ((raw & TCPC_VBUS_VOLTAGE_MASK) * TCPC_VBUS_VOLTAGE_LSB_MV);
}

/* Acquire rc lock before calling */
static void floating_cable_sink_detected_handler_locked(struct max77759_plat *chip)
{
	chip->floating_cable_or_sink_detected++;
	LOG(LOG_LVL_DEBUG, chip->log,
	    "floating_cable_or_sink_detected count: %d", chip->floating_cable_or_sink_detected);
	if (chip->floating_cable_or_sink_detected >= FLOATING_CABLE_OR_SINK_INSTANCE_THRESHOLD) {
		max777x9_disable_auto_ultra_low_power_mode(chip, true);
		alarm_start_relative(&chip->reenable_auto_ultra_low_power_mode_alarm,
				     ms_to_ktime(AUTO_ULTRA_LOW_POWER_MODE_REENABLE_MS));
	}
}

static void ovp_operation(struct max77759_plat *chip, int operation)
{
	int gpio_val, retry = 0;

	mutex_lock(&chip->ovp_lock);
	if (operation == OVP_RESET || operation == OVP_OFF) {
		do {
			gpio_set_value_cansleep(chip->in_switch_gpio,
						!chip->in_switch_gpio_active_high);
			gpio_val = gpio_get_value_cansleep(chip->in_switch_gpio);
			LOG(LOG_LVL_DEBUG, chip->log,
			    "%s: OVP disable gpio_val:%d in_switch_gpio_active_high:%d retry:%d",
			    __func__, gpio_val, chip->in_switch_gpio_active_high, retry++);
		} while ((gpio_val != !chip->in_switch_gpio_active_high) && (retry < OVP_OP_RETRY));
	}

	if (operation == OVP_RESET)
		mdelay(10);

	if (operation == OVP_RESET || operation == OVP_ON) {
		retry = 0;
		do {
			gpio_set_value_cansleep(chip->in_switch_gpio,
						chip->in_switch_gpio_active_high);
			gpio_val = gpio_get_value_cansleep(chip->in_switch_gpio);
			LOG(LOG_LVL_DEBUG, chip->log,
			    "%s: OVP enable gpio_val:%d in_switch_gpio_active_high:%d retry:%d",
			    __func__, gpio_val, chip->in_switch_gpio_active_high, retry++);
		} while ((gpio_val != chip->in_switch_gpio_active_high) && (retry < OVP_OP_RETRY));
	}
	mutex_unlock(&chip->ovp_lock);
}

static void reset_ovp_work(struct kthread_work *work)
{
	struct max77759_plat *chip  =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct max77759_plat, reset_ovp_work);
	u16 vbus_mv = max77759_get_vbus_voltage_mv(chip->client);

	LOG(LOG_LVL_DEBUG, chip->log, "%s: vbus %u mv", __func__, vbus_mv);

	if (vbus_mv > VBUS_PRESENT_THRESHOLD_MV)
		return;

	ovp_operation(chip, OVP_RESET);
	chip->reset_ovp_retry++;

	LOG(LOG_LVL_DEBUG, chip->log, "ovp reset done [%d]", chip->reset_ovp_retry);

	if (chip->reset_ovp_retry < VBUS_RAMPUP_MAX_RETRY)
		kthread_mod_delayed_work(chip->wq, &chip->reset_ovp_work,
					 msecs_to_jiffies(VBUS_RAMPUP_TIMEOUT_MS));
	else
		chip->reset_ovp_retry = 0;

}

static void max77759_get_cc(struct max77759_plat *chip, enum typec_cc_status *cc1,
			    enum typec_cc_status *cc2)
{
	struct google_shim_tcpci *tcpci = chip->tcpci;
	u8 reg, role_control;
	int ret;

	ret = max77759_read8(tcpci->regmap, TCPC_ROLE_CTRL, &role_control);
	if (ret < 0)
		return;

	ret = max77759_read8(tcpci->regmap, TCPC_CC_STATUS, &reg);
	if (ret < 0)
		return;

	*cc1 = tcpci_to_typec_cc((reg >> TCPC_CC_STATUS_CC1_SHIFT) &
				 TCPC_CC_STATUS_CC1_MASK,
				 reg & TCPC_CC_STATUS_TERM ||
				 tcpc_presenting_rd(role_control, CC1));
	*cc2 = tcpci_to_typec_cc((reg >> TCPC_CC_STATUS_CC2_SHIFT) &
				 TCPC_CC_STATUS_CC2_MASK,
				 reg & TCPC_CC_STATUS_TERM ||
				 tcpc_presenting_rd(role_control, CC2));
}

/*
 * WAR (b/335368150): Use the flag manual_disable_vbus to check whether OTG_SW_EN
 * (EXT_BST_EN in max77779) is used. If true (not used), notify BMS to turn off Vbus as soon as
 * disconnect is detected by the driver so that VBUS can discharge when entering
 * Disconnected_As_Src state. Also enable force discharge as auto discharge would automatically turn
 * off after tSafe0V if software is slow to disable vbus.
 *
 * Check the status of TCPC_POWER_CTRL_AUTO_DISCHARGE for some usecases that this WAR is not needed,
 * such as Power Role Swap (Apply_RC state).
 */
static int max77759_manual_vbus_handling_on_cc_change(struct max77759_plat *chip,
						      enum typec_cc_status new_cc1,
						      enum typec_cc_status new_cc2)
{
	struct google_shim_tcpci *tcpci = chip->tcpci;
	bool auto_discharge_enabled, disconnect_as_source;
	u8 pwr_ctrl;
	int ret;

	if (!chip->manual_disable_vbus)
		return 0;

	ret = max77759_read8(tcpci->regmap, TCPC_POWER_CTRL, &pwr_ctrl);
	if (ret < 0) {
		LOG(LOG_LVL_DEBUG, chip->log, "%s: failed to read TCPC_POWER_CTRL ret:%d", __func__,
		    ret);
		return ret;
	}

	auto_discharge_enabled = !!(pwr_ctrl & TCPC_POWER_CTRL_AUTO_DISCHARGE);
	disconnect_as_source = chip->sourcing_vbus && auto_discharge_enabled &&
			       port_is_source(chip->cc1, chip->cc2) &&
			       cc_open_or_toggling(new_cc1, new_cc2);
	if (disconnect_as_source) {
		max77759_force_discharge(chip, true);
		ret = gvotable_cast_vote(chip->charger_mode_votable, TCPCI_MODE_VOTER,
					 (void *)GBMS_USB_BUCK_ON, false);
		max777x9_bcl_usb_update(chip, USB_UNPLUGGED);

		LOG(LOG_LVL_DEBUG, chip->log, "%s: GBMS_MODE_VOTABLE voting 0 for BUCK_ON ret:%d",
		    ret < 0 ? "Error" : "Success", ret);

		chip->sourcing_vbus = 0;
		chip->vbus_present = 0;
		tcpm_vbus_change(tcpci->port);
	}

	return ret;
}

static void max77759_cache_cc(struct max77759_plat *chip, enum typec_cc_status new_cc1,
			      enum typec_cc_status new_cc2)
{
	/*
	 * If the Vbus OVP is restricted to quick ramp-up time for incoming Vbus to work properly,
	 * queue a delayed work to check the Vbus status later. Cancel the delayed work once the CC
	 * is back to Open as we won't expect that Vbus is coming.
	 */
	if (chip->quick_ramp_vbus_ovp) {
		if (cc_open_or_toggling(chip->cc1, chip->cc2) && port_is_sink(new_cc1, new_cc2)) {
			kthread_mod_delayed_work(chip->wq, &chip->reset_ovp_work,
						 msecs_to_jiffies(VBUS_RAMPUP_TIMEOUT_MS));
		} else if (cc_open_or_toggling(new_cc1, new_cc2)) {
			kthread_cancel_delayed_work_sync(&chip->reset_ovp_work);
			chip->reset_ovp_retry = 0;
		}
	}

	LOG(LOG_LVL_DEBUG, chip->log,
	    "cc1: %u -> %u cc2: %u -> %u", chip->cc1, new_cc1, chip->cc2, new_cc2);
	chip->cc1 = new_cc1;
	chip->cc2 = new_cc2;
}

/* hold irq_status_lock before calling */
static irqreturn_t _max77759_irq_locked(struct max77759_plat *chip, u16 status,
					struct logbuffer *log)
{
	u16 vendor_status = 0, vendor_status2 = 0, raw;
	struct google_shim_tcpci *tcpci = chip->tcpci;
	int ret;
	const u16 mask = status & TCPC_ALERT_RX_BUF_OVF ? status &
		~(TCPC_ALERT_RX_STATUS | TCPC_ALERT_RX_BUF_OVF) :
		status & ~TCPC_ALERT_RX_STATUS;
	u8 reg_status;
	bool contaminant_cc_update_handled = false, invoke_tcpm_for_cc_update = false,
		port_clean = false;
	unsigned int pwr_status;

	pm_wakeup_event(chip->dev, PD_ACTIVITY_TIMEOUT_MS);
	LOG(LOG_LVL_DEBUG, log, "TCPC_ALERT status: %#x", status);
	/**
	 * Clear alert status for everything except RX_STATUS, which shouldn't
	 * be cleared until we have successfully retrieved message.
	 **/
	if (status & ~TCPC_ALERT_RX_STATUS) {
		ret = max77759_write16(tcpci->regmap, TCPC_ALERT, mask);
		if (ret < 0)
			goto reschedule;
	}

	if (status & TCPC_ALERT_RX_BUF_OVF && !(status &
						TCPC_ALERT_RX_STATUS)) {
		LOG(LOG_LVL_DEBUG, log, "TCPC_ALERT_RX_BUF_OVF");
		ret = max77759_write16(tcpci->regmap, TCPC_ALERT,
				       (TCPC_ALERT_RX_STATUS |
					TCPC_ALERT_RX_BUF_OVF));
		if (ret < 0)
			goto reschedule;
	}

	if (status & TCPC_ALERT_EXTND) {
		ret = max77759_read8(tcpci->regmap, TCPC_ALERT_EXTENDED, &reg_status);
		if (ret < 0)
			goto reschedule;

		ret = max77759_write8(tcpci->regmap, TCPC_ALERT_EXTENDED, reg_status);
		if (ret < 0)
			goto reschedule;

		if (reg_status & TCPC_SINK_FAST_ROLE_SWAP) {
			LOG(LOG_LVL_DEBUG, log, "FRS Signal");
			chip->in_frs = true;
			tcpm_sink_frs(tcpci->port);
		}
	}

	if (status & TCPC_ALERT_RX_STATUS) {
		LOG(LOG_LVL_DEBUG, log, "Enter process rx");
		ret = process_rx(chip, status);
		if (ret == -EIO)
			goto reschedule;
	}

	if (status & TCPC_ALERT_TX_DISCARDED)
		LOG(LOG_LVL_DEBUG, log, "TX_DISCARDED");

	if (status & TCPC_ALERT_VENDOR) {
		LOG(LOG_LVL_DEBUG, log, "TCPC_VENDOR_ALERT Mask");
		ret = max77759_write8(tcpci->regmap, TCPC_VENDOR_ALERT_MASK
				      , 0x0);
		if (ret < 0)
			goto reschedule;

		ret = max77759_write8(tcpci->regmap,
				      TCPC_VENDOR_ALERT_MASK2, 0x0);
		if (ret < 0)
			goto reschedule;

		/* Clear VENDOR_ALERT*/
		ret = max77759_read16(tcpci->regmap, TCPC_VENDOR_ALERT,
				      &vendor_status);
		if (ret < 0)
			goto reschedule;
		LOG(LOG_LVL_DEBUG, log, "TCPC_VENDOR_ALERT 0x%x", vendor_status);

		process_bc12_alert(chip->bc12, vendor_status);
		ret = max77759_write16(tcpci->regmap, TCPC_VENDOR_ALERT,
				       vendor_status);

		ret = max77759_read16(tcpci->regmap, TCPC_VENDOR_ALERT2, &vendor_status2);
		if (ret < 0)
			goto reschedule;
		LOG(LOG_LVL_DEBUG, log, "TCPC_VENDOR_ALERT2 0x%x", vendor_status2);

		ret = max77759_write16(tcpci->regmap, TCPC_VENDOR_ALERT2, vendor_status2);
		if (ret < 0)
			goto reschedule;
	}

	if (status & TCPC_ALERT_VBUS_DISCNCT) {
		LOG(LOG_LVL_DEBUG, log, "TCPC_ALERT_VBUS_DISCNCT, %umv",
		    max77759_get_vbus_voltage_mv(chip->client));
		chip->vbus_present = 0;
		LOG(LOG_LVL_DEBUG, chip->log,
		    "[%s]: vbus_present %d", __func__, chip->vbus_present);
		tcpm_vbus_change(tcpci->port);
		if (chip->force_device_mode_on) {
			ret = max77759_write8(tcpci->regmap, TCPC_VENDOR_USBSW_CTRL, USBSW_CONNECT);
			LOG(LOG_LVL_DEBUG, chip->log,
			    "Forcing on dp switches %s", ret < 0 ? "fail" : "success");
			if (ret < 0)
				goto reschedule;
		}
	}

	if (status & TCPC_ALERT_CC_STATUS) {
		/**
		 * Process generic CC updates if it doesn't belong to
		 * contaminant detection.
		 */
		mutex_lock(&chip->rc_lock);
		LOG(LOG_LVL_DEBUG, chip->log, "Servicing TCPC_ALERT_CC_STATUS");
		if (!chip->usb_throttled && chip->contaminant_detection &&
		    (tcpm_port_is_toggling(tcpci->port) ||
		     max777x9_is_contaminant_detected(chip))) {
			LOG(LOG_LVL_DEBUG, chip->log, "Invoking process_contaminant_alert");
			ret = max777x9_process_contaminant_alert(chip->contaminant, false, true,
								 &contaminant_cc_update_handled,
								 &port_clean);
			if (ret < 0) {
				mutex_unlock(&chip->rc_lock);
				goto reschedule;
			} else if (chip->check_contaminant ||
				   max777x9_is_contaminant_detected(chip)) {
				/*
				 * Taken in debounce path when the port is dry.
				 * Move TCPM back to TOGGLING.
				 */
				if (port_clean) {
					chip->check_contaminant = false;
					tcpm_port_clean(chip->port);
				}
				/* tcpm_cc_change does not have to be invoked. */
				invoke_tcpm_for_cc_update = false;
			} else {
				/*
				 * Invoke TCPM when CC update not related to contaminant detection.
				 */
				invoke_tcpm_for_cc_update = !contaminant_cc_update_handled;
				/*
				 * CC status change handled by contaminant algorithm.
				 * Handle floating cable if detected.
				 */
				if (contaminant_cc_update_handled) {
					LOG(LOG_LVL_DEBUG, log,
					    "CC update: Contaminant algorithm responded");
					if (max777x9_is_floating_cable_or_sink_detected(chip)) {
						floating_cable_sink_detected_handler_locked(chip);
						LOG(LOG_LVL_DEBUG, chip->log,
						    "Floating cable detected");
					} else {
						chip->floating_cable_or_sink_detected = 0;
						LOG(LOG_LVL_DEBUG, chip->log,
						    "Floating cable counter cleared");
					}
				}
			}
		} else {
			invoke_tcpm_for_cc_update = true;
		}

		if (invoke_tcpm_for_cc_update) {
			enum typec_cc_status new_cc1, new_cc2;

			LOG(LOG_LVL_DEBUG, chip->log, "invoke_tcpm_for_cc_update");
			tcpm_cc_change(tcpci->port);
			max77759_get_cc(chip, &new_cc1, &new_cc2);
			/*
			 * To preserve the tcpm event ordering, do this optional special vbus
			 * handling after tcpm_cc_change because tcpm_vbus_change will be called
			 * here. Note that this function may spend several milliseconds for gvotable
			 * function calls.
			 */
			max77759_manual_vbus_handling_on_cc_change(chip, new_cc1, new_cc2);
			max77759_cache_cc(chip, new_cc1, new_cc2);
			/* Check for missing-rp non compliant power source */
			if (!regmap_read(tcpci->regmap, TCPC_POWER_STATUS, &pwr_status) &&
			    !chip->usb_throttled && !chip->toggle_disable_status)
				check_missing_rp(chip, !!(pwr_status & TCPC_POWER_STATUS_VBUS_PRES),
						 chip->cc1, chip->cc2);
			/* TCPM has detected valid CC terminations */
			if (!tcpm_port_is_toggling(tcpci->port)) {
				chip->floating_cable_or_sink_detected = 0;
				/*
				 * Only re-enable auto ultra low power mode only
				 * when contaminant detection is enabled.
				 */
				if (chip->contaminant_detection_userspace !=
					CONTAMINANT_DETECT_DISABLE)
					max777x9_disable_auto_ultra_low_power_mode(chip, false);
			} else if (!chip->usb_throttled && chip->contaminant_detection) {
				/*
				 * TCPM has not detected valid CC terminations
				 * and neither the comparators nor ADC
				 * readings indicate sink or floating cable.
				 * Mitigate AP wakeups here.
				 *
				 * The counter will also incremented when
				 * transitioning from *_READY states to
				 * TOGGLING state. This shouldn't have adverse
				 * effect as the FLOATING_CABLE_OR_SINK_INSTANCE_THRESHOLD
				 * is now doubled.
				 */
				LOG(LOG_LVL_DEBUG, chip->log, "Treating as floating cable");
				floating_cable_sink_detected_handler_locked(chip);
			}
		}
		mutex_unlock(&chip->rc_lock);
	}

	if (status & TCPC_ALERT_POWER_STATUS)
		process_power_status(chip);

	if (status & TCPC_ALERT_V_ALARM_LO) {
		ret = max77759_read16(tcpci->regmap, TCPC_VBUS_VOLTAGE_ALARM_LO_CFG, &raw);
		if (ret < 0)
			goto reschedule;

		LOG(LOG_LVL_DEBUG, log, "VBUS LOW ALARM triggered: thresh:%umv vbus:%umv",
		    (raw & TCPC_VBUS_VOLTAGE_MASK) * TCPC_VBUS_VOLTAGE_LSB_MV,
		    max77759_get_vbus_voltage_mv(chip->client));
		max77759_enable_voltage_alarm(chip, true, true);

		ret = extcon_set_state_sync(chip->extcon, EXTCON_MECHANICAL, 0);
		LOG(LOG_LVL_DEBUG, chip->log, "%s: %s turning off connected, ret=%d",
		    __func__, ret < 0 ? "Failed" : "Succeeded", ret);
	}

	if (status & TCPC_ALERT_V_ALARM_HI) {
		ret = max77759_read16(tcpci->regmap, TCPC_VBUS_VOLTAGE_ALARM_HI_CFG, &raw);
		if (ret < 0)
			goto reschedule;

		LOG(LOG_LVL_DEBUG, log, "VBUS HIGH ALARM triggered: thresh:%umv vbus:%umv",
		    (raw & TCPC_VBUS_VOLTAGE_MASK) * TCPC_VBUS_VOLTAGE_LSB_MV,
		    max77759_get_vbus_voltage_mv(chip->client));
		max77759_enable_voltage_alarm(chip, true, false);

		ret = extcon_set_state_sync(chip->extcon, EXTCON_MECHANICAL, 1);
		LOG(LOG_LVL_DEBUG, chip->log, "%s: %s turning on connected, ret=%d",
		    __func__, ret < 0 ? "Failed" : "Succeeded", ret);
	}

	if (status & TCPC_ALERT_RX_HARD_RST) {
		LOG(LOG_LVL_DEBUG, log, "TCPC_ALERT_RX_HARD_RST");
		/* To prevent disconnect during hardreset. */
		ret = max77759_write16(tcpci->regmap,
				       TCPC_VBUS_SINK_DISCONNECT_THRESH,
				       0);
		if (ret < 0)
			goto reschedule;

		tcpm_pd_hard_reset(tcpci->port);
		max77759_init_regs(tcpci->regmap, log);
	}

	if (status & TCPC_ALERT_TX_SUCCESS || status &
	    TCPC_ALERT_TX_DISCARDED || status & TCPC_ALERT_TX_FAILED)
		process_tx(tcpci, status, log);

	if (status & TCPC_ALERT_VENDOR) {
		LOG(LOG_LVL_DEBUG, log, "Exit TCPC_VENDOR_ALERT Unmask");
		ret = max77759_write8(tcpci->regmap, TCPC_VENDOR_ALERT_MASK
				      , 0xff);
		if (ret < 0)
			goto reschedule;
		ret = max77759_write8(tcpci->regmap,
				      TCPC_VENDOR_ALERT_MASK2, 0xff);
		if (ret < 0)
			goto reschedule;
	}

	if (status & TCPC_ALERT_EXTENDED_STATUS) {
		bool vsafe0v;
		ret = max77759_read8(tcpci->regmap, TCPC_EXTENDED_STATUS,
				     (u8 *)&raw);
		if (ret < 0)
			goto reschedule;

		vsafe0v = raw & TCPC_EXTENDED_STATUS_VSAFE0V;
		LOG(LOG_LVL_DEBUG, log, "VSAFE0V (runtime): %c -> %c",
		    chip->vsafe0v ? 'Y' : 'N', vsafe0v ? 'Y' : 'N');

		if (vsafe0v && chip->manual_disable_vbus)
			max77759_force_discharge(chip, false);

		/*
		 * b/199991513 For some OVP chips, when the incoming Vbus ramps up from 0, there is
		 * a chance that an induced voltage (over Vsafe0V) behind the OVP would appear for a
		 * short time and then drop to 0 (Vsafe0V), and ramp up to some HIGH voltage
		 * (e.g Vsafe5V). To ignore the unwanted Vsafe0V event, queue a delayed work and
		 * re-check the voltage after VSAFE0V_DEBOUNCE_MS.
		 *
		 * The OVP which is restricted to quick ramp-up Vbus is the same as the one
		 * mentioned above. Thus re-use the same flag chip->quick_ramp_vbus_ovp.
		 */
		if (chip->quick_ramp_vbus_ovp) {
			if (!chip->vsafe0v && vsafe0v)
				kthread_mod_delayed_work(chip->wq, &chip->vsafe0v_work,
							 msecs_to_jiffies(VSAFE0V_DEBOUNCE_MS));
		} else if (vsafe0v) {
			chip->vbus_present = 0;
			LOG(LOG_LVL_DEBUG, chip->log,
			    "[%s]: vbus_present %d", __func__, chip->vbus_present);
			tcpm_vbus_change(tcpci->port);
		}

		if (vsafe0v)
			chip->sourcing_vbus_high = 0;

		chip->vsafe0v = vsafe0v;
	}

	LOG(LOG_LVL_DEBUG, log, "TCPC_ALERT status done: %#x", status);

	return IRQ_HANDLED;
reschedule:
	chip->irq_status = status;
	LOG(LOG_LVL_DEBUG, log, "TCPC_ALERT IO error occurred. status: %#x", status);
	kthread_mod_delayed_work(chip->wq, &chip->max77759_io_error_work,
				 msecs_to_jiffies(IO_ERROR_RETRY_MS));
	pm_wakeup_event(chip->dev, PD_ACTIVITY_TIMEOUT_MS + IO_ERROR_RETRY_MS);
	return IRQ_HANDLED;
}

static irqreturn_t max77759_irq(int irq, void *dev_id)
{
	struct max77759_plat *chip = dev_id;
	u16 status;
	irqreturn_t irq_return;
	int ret;

	LOG(LOG_LVL_DEBUG, chip->log, "TCPC_ALERT threaded irq running ");
	if (!chip->tcpci)
		return IRQ_HANDLED;

	ret = max77759_read16(chip->tcpci->regmap, TCPC_ALERT, &status);
	if (ret < 0)
		return ret;
	mutex_lock(&chip->irq_status_lock);
	while (status) {
		irq_return = _max77759_irq_locked(chip, status, chip->log);
		/* Do not return if the ALERT is already set. */
		LOG(LOG_LVL_DEBUG, chip->log, "TCPC_ALERT read alert status");
		ret = max77759_read16(chip->tcpci->regmap, TCPC_ALERT, &status);
		if (ret < 0)
			break;
		LOG(LOG_LVL_DEBUG, chip->log, "TCPC_ALERT status pending: %#x", status);
	}
	mutex_unlock(&chip->irq_status_lock);

	return irq_return;
}

static irqreturn_t max77759_isr(int irq, void *dev_id)
{
	struct max77759_plat *chip = dev_id;

	LOG(LOG_LVL_DEBUG, chip->log, "TCPC_ALERT triggered ");
	pm_wakeup_event(chip->dev, PD_ACTIVITY_TIMEOUT_MS);

	if (!chip->tcpci)
		return IRQ_HANDLED;

	return IRQ_WAKE_THREAD;
}

static void max77759_io_error_work(struct kthread_work *work)
{
	struct max77759_plat *chip =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct max77759_plat, max77759_io_error_work);
	pm_wakeup_event(chip->dev, PD_ACTIVITY_TIMEOUT_MS);
	mutex_lock(&chip->irq_status_lock);
	LOG(LOG_LVL_DEBUG, chip->log, "IO error retry. status: %#x", chip->irq_status);
	_max77759_irq_locked(chip, chip->irq_status, chip->log);
	mutex_unlock(&chip->irq_status_lock);
}

static int max77759_init_alert(struct max77759_plat *chip,
			       struct i2c_client *client)
{
	int ret, irq_gpio;

	irq_gpio = of_get_named_gpio(client->dev.of_node, "usbpd,usbpd_int", 0);
	client->irq = gpio_to_irq(irq_gpio);
	if (!client->irq)
		return -ENODEV;

	ret = devm_request_threaded_irq(chip->dev, client->irq, max77759_isr,
					max77759_irq,
					(IRQF_TRIGGER_LOW | IRQF_ONESHOT),
					dev_name(chip->dev), chip);

	if (ret < 0)
		return ret;

	enable_irq_wake(client->irq);
	return 0;
}

/* Called while holding rc_lock */
static void max77759_enable_toggling_locked(struct max77759_plat *chip, bool enable)
{
	int ret;

	if (!enable) {
		ret = max77759_write8(chip->data.regmap, TCPC_ROLE_CTRL, TCPCI_HI_Z_CC);
		LOG(LOG_LVL_DEBUG, chip->log, "%s: HI-Z ret:%d", __func__, ret);
		return;
	}

	ret = max77759_write8(chip->data.regmap, TCPC_ROLE_CTRL, chip->role_ctrl_cache);
	if (ret < 0) {
		LOG(LOG_LVL_DEBUG, chip->log, "%s: update ROLE_CTRL failed ret:%d", __func__, ret);
		return;
	}

	ret = max77759_update_bits8(chip->data.regmap, TCPC_TCPC_CTRL,
				    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT,
				    TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);
	if (ret < 0) {
		LOG(LOG_LVL_DEBUG, chip->log,
		    "%s: Enable LK4CONN alert failed ret:%d", __func__, ret);
		return;
	}

	ret = regmap_write(chip->data.regmap, TCPC_COMMAND, TCPC_CMD_LOOK4CONNECTION);
	if (ret < 0)
		LOG(LOG_LVL_DEBUG, chip->log, "%s: Enable LK4CONN failed ret:%d", __func__, ret);
}

static int max77759_start_toggling(struct google_shim_tcpci *tcpci,
				   struct google_shim_tcpci_data *tdata,
				   enum typec_cc_status cc)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);
	u8 reg = TCPC_ROLE_CTRL_DRP, pwr_ctrl;
	int ret;
	enum typec_cc_status cc1, cc2;

	/* Wait for tcpci_register_port to finish. */
	while (READ_ONCE(chip->tcpci) == NULL)
		cpu_relax();

	max77759_get_cc(chip, &cc1, &cc2);

	switch (cc) {
	case TYPEC_CC_RP_DEF:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_DEF <<
			TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_1_5:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_1_5 <<
			TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	case TYPEC_CC_RP_3_0:
		reg |= (TCPC_ROLE_CTRL_RP_VAL_3_0 <<
			TCPC_ROLE_CTRL_RP_VAL_SHIFT);
		break;
	default:
		break;
	}

	if (cc == TYPEC_CC_RD)
		reg |= (TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RD << TCPC_ROLE_CTRL_CC2_SHIFT);
	else
		reg |= (TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC1_SHIFT) |
			(TCPC_ROLE_CTRL_CC_RP << TCPC_ROLE_CTRL_CC2_SHIFT);

	max77759_init_regs(chip->tcpci->regmap, chip->log);

	chip->role_ctrl_cache = reg;
	mutex_lock(&chip->rc_lock);
	if (chip->toggle_disable_status)
		goto unlock;

	/* Kick debug accessory state machine when enabling toggling for the first time */
	if (chip->first_toggle) {
		if ((chip->in_switch_gpio >= 0) && is_debug_accessory_detected(cc1, cc2)) {
			LOG(LOG_LVL_DEBUG, chip->log, "[%s]: Kick Debug accessory FSM", __func__);
			ovp_operation(chip, OVP_RESET);
		}
		chip->first_toggle_time_since_boot = ktime_get_boottime();
		chip->first_toggle = false;
	}

	/* Renable BC1.2*/
	if (!bc12_get_status(chip->bc12))
		bc12_enable(chip->bc12, true);

	/* Re-enable retry */
	bc12_reset_retry(chip->bc12);

	/* Disable Auto disacharge before enabling toggling */
	ret = max77759_read8(tcpci->regmap, TCPC_POWER_CTRL, &pwr_ctrl);
	LOG(LOG_LVL_DEBUG, chip->log, "TCPC_POWER_CTRL:0x%x ret:%d", pwr_ctrl, ret);
	if (pwr_ctrl & TCPC_POWER_CTRL_AUTO_DISCHARGE) {
		LOG(LOG_LVL_DEBUG, chip->log, "TCPC_POWER_CTRL_AUTO_DISCHARGE not cleared");
		ret = regmap_update_bits(tcpci->regmap, TCPC_POWER_CTRL,
					 TCPC_POWER_CTRL_AUTO_DISCHARGE, 0);
		if (ret < 0)
			LOG(LOG_LVL_DEBUG, chip->log,
			    "[%s]: Disabling auto discharge failed", __func__);
	}

	/* b/223078393: Disable ext bst upon toggling */
	ret = max77759_write8(tcpci->regmap, TCPC_VENDOR_EXTBST_CTRL, 0);
	LOG(LOG_LVL_DEBUG, chip->log, "%s: clear TCPC_VENDOR_EXTBST_CTRL ret:%d", __func__, ret);

	if (chip->contaminant_detection)
		update_contaminant_detection_locked(chip, chip->contaminant_detection);
	else
		max77759_enable_toggling_locked(chip, true);

unlock:
	mutex_unlock(&chip->rc_lock);

	return 0;
}

static void max77759_set_partner_usb_comm_capable(struct google_shim_tcpci *tcpci,
						  struct google_shim_tcpci_data *data, bool capable)
{
	struct max77759_plat *chip = tdata_to_max77759(data);

	mutex_lock(&chip->data_path_lock);
	chip->pd_data_capable = capable;
	enable_data_path_locked(chip);
	mutex_unlock(&chip->data_path_lock);
}

static int max77759_usb_set_orientation(struct typec_switch_dev *sw,
					enum typec_orientation orientation)
{
	struct max77759_plat *chip = typec_switch_get_drvdata(sw);
	enum typec_cc_polarity polarity = orientation == TYPEC_ORIENTATION_REVERSE ?
		TYPEC_POLARITY_CC2 : TYPEC_POLARITY_CC1;
	int ret;

	chip->orientation = orientation;
	ret = extcon_set_property(chip->extcon, EXTCON_USB, EXTCON_PROP_USB_TYPEC_POLARITY,
				  (union extcon_property_value)(int)polarity);
	logbuffer_logk(chip->log, LOGLEVEL_INFO,
		       "%s setting polarity USB %d", ret < 0 ? "Failed" : "Succeeded", polarity);

	ret = extcon_set_property(chip->extcon, EXTCON_USB_HOST, EXTCON_PROP_USB_TYPEC_POLARITY,
				  (union extcon_property_value)(int)polarity);
	logbuffer_logk(chip->log, LOGLEVEL_INFO, "%s setting polarity USB_HOST %d",
		       ret < 0 ? "Failed" : "Succeeded", polarity);

	chip->polarity = polarity;

	if (orientation_callback)
		(*orientation_callback)(orientation_payload);

	return ret;
}

static int max77759_vote_icl(struct max77759_plat *chip, u32 max_ua)
{
	int ret = 0;
	struct usb_vote vote;

	/*
	 * TCPM sets max_ua to zero for Rp-default which needs to be
	 * ignored. PPS values reflect the requested ones not the max.
	 */
	mutex_lock(&chip->icl_proto_el_lock);
	if ((chip->usb_type != POWER_SUPPLY_USB_TYPE_PD && max_ua == 0 && chip->online) ||
	    chip->online == TCPM_PSY_PROG_ONLINE)
		goto exit;

	init_vote(&vote, proto_voter_reason[USB_ICL_PD], USB_ICL_PD, max_ua);
	ret = gvotable_cast_vote(chip->usb_icl_proto_el,
				 proto_voter_reason[USB_ICL_PD], &vote,
				 chip->online);

	LOG(LOG_LVL_DEBUG, chip->log, "%s: %s:%d voting enabled:%s usb proto_el: %d by %s",
	    __func__, ret < 0 ? "error" : "success", ret, chip->online ? "enabled" : "disabled",
	    vote.val, proto_voter_reason[USB_ICL_PD]);

exit:
	mutex_unlock(&chip->icl_proto_el_lock);
	return ret;
}

static void icl_work_item(struct kthread_work *work)
{
	struct max77759_plat *chip  =
		container_of(container_of(work, struct kthread_delayed_work, work),
			     struct max77759_plat, icl_work);
	union power_supply_propval current_max = {0}, voltage_max = {0}, online = {0},
	      usb_type = {0}, val = {0};
	int ret;

	power_supply_get_property(chip->tcpm_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &current_max);
	power_supply_get_property(chip->tcpm_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &voltage_max);
	power_supply_get_property(chip->tcpm_psy, POWER_SUPPLY_PROP_ONLINE, &online);
	power_supply_get_property(chip->tcpm_psy, POWER_SUPPLY_PROP_USB_TYPE, &usb_type);
	logbuffer_logk(chip->log, LOGLEVEL_INFO,
		      "%s: ONLINE:%d USB_TYPE:%d CURRENT_MAX:%d VOLTAGE_MAX:%d",
		      __func__, online.intval, usb_type.intval, current_max.intval, voltage_max.intval);

	/* Debounce disconnect for power adapters that can source at least 1.5A */
	if (chip->debounce_adapter_disconnect && chip->online && !online.intval &&
	    chip->typec_current_max >= 1500000) {
		logbuffer_log(chip->log, "Debouncing disconnect\n");
		/* Reduce current limit 500mA during debounce */
		max77759_vote_icl(chip, 500000);
		chip->debounce_adapter_disconnect = false;
		kthread_mod_delayed_work(chip->wq, &chip->icl_work,
					 msecs_to_jiffies(DISCONNECT_DEBOUNCE_MS));
		return;
	}

	chip->vbus_mv = voltage_max.intval / 1000;
	val.intval = voltage_max.intval;
	ret = power_supply_set_property(chip->usb_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX, &val);
	if (ret < 0)
		LOG(LOG_LVL_DEBUG, chip->log,
		    "unable to set max voltage to %d, ret=%d", voltage_max.intval, ret);

	chip->online = online.intval;
	chip->usb_type = usb_type.intval;
	chip->typec_current_max = current_max.intval;
	usb_psy_set_sink_state(chip->usb_psy_data, chip->online);
	max77759_vote_icl(chip, chip->typec_current_max);
}

static int psy_changed(struct notifier_block *nb, unsigned long evt, void *ptr)
{
	struct max77759_plat *chip = container_of(nb, struct max77759_plat, psy_notifier);
	struct power_supply *psy = ptr;
	union power_supply_propval online = {0}, usb_type = {0};

	if (!strstr(psy->desc->name, "tcpm-source") || evt != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &online);
	power_supply_get_property(psy, POWER_SUPPLY_PROP_USB_TYPE, &usb_type);
	logbuffer_logk(chip->log, LOGLEVEL_INFO, "ONLINE:%d USB_TYPE:%d", online.intval,
		       usb_type.intval);
	chip->tcpm_psy = psy;

	if (chip->online && !online.intval && chip->typec_current_max >= 1500000)
		chip->debounce_adapter_disconnect = true;
	else
		chip->debounce_adapter_disconnect = false;

	/* Notifier is atomic, hence offloading */
	kthread_mod_delayed_work(chip->wq, &chip->icl_work, 0);
	return NOTIFY_OK;
}

static int max77759_get_vbus_voltage_max_mv(struct i2c_client *tcpc_client)
{
	struct max77759_plat *chip = i2c_get_clientdata(tcpc_client);

	return chip ? chip->vbus_mv : 0;
}

static int max77759_set_vbus_voltage_max_mv(struct i2c_client *tcpc_client,
					    unsigned int mv)
{
	struct max77759_plat *chip = i2c_get_clientdata(tcpc_client);

	if (chip)
		chip->vbus_mv = mv;

	return 0;
}

static int max77759_get_vbus(struct google_shim_tcpci *tcpci, struct google_shim_tcpci_data *data)
{
	struct max77759_plat *chip = tdata_to_max77759(data);
	u8 pwr_status;
	int ret;

	ret = max77759_read8(tcpci->regmap, TCPC_POWER_STATUS, &pwr_status);
	if (ret < 0) {
		LOG(LOG_LVL_DEBUG, chip->log, "[%s]: Unable to fetch power status, ret=%d\n",
		    __func__, ret);
		return ret;
	}

	if (!ret && !chip->vbus_present && (pwr_status & TCPC_POWER_STATUS_VBUS_PRES)) {
		LOG(LOG_LVL_DEBUG, chip->log, "[%s]: syncing vbus_present", __func__);
		chip->vbus_present = 1;
	}

	LOG(LOG_LVL_DEBUG, chip->log, "[%s]: chip vbus_present %d, live vbus_present %d, %umv",
	    __func__, chip->vbus_present, !!(pwr_status & TCPC_POWER_STATUS_VBUS_PRES),
	    max77759_get_vbus_voltage_mv(chip->client));

	if (chip->toggle_disable_status) {
		LOG(LOG_LVL_DEBUG, chip->log, "%s: toggle disabled, return Vbus off", __func__);
		return 0;
	}

	if (chip->sourcing_vbus_high) {
		LOG(LOG_LVL_DEBUG, chip->log, "%s: sourcing vbus high, return Vbus off", __func__);
		return 0;
	}

	return chip->vbus_present;
}

static int max77759_usb_set_role(struct usb_role_switch *sw, enum usb_role role)
{
	struct max77759_plat *chip = usb_role_switch_get_drvdata(sw);
	enum typec_data_role typec_data_role = TYPEC_DEVICE;
	bool attached = role != USB_ROLE_NONE, enable_data;
	int ret;

	if (role == USB_ROLE_HOST)
		typec_data_role = TYPEC_HOST;

	mutex_lock(&chip->data_path_lock);

	enable_data = chip->pd_data_capable || chip->no_bc_12 || chip->bc12_data_capable ||
		chip->data_role == TYPEC_HOST || chip->debug_acc_connected;

	if (!chip->force_device_mode_on && chip->data_active && !chip->alt_path_active &&
	    (chip->active_data_role != typec_data_role || !attached || !enable_data)) {
		ret = extcon_set_state_sync(chip->extcon,
					    chip->active_data_role ==
					    TYPEC_HOST ? EXTCON_USB_HOST :
					    EXTCON_USB, 0);

		LOG(LOG_LVL_DEBUG, chip->log, "%s turning off %s", ret < 0 ? "Failed" : "Succeeded",
		    chip->active_data_role == TYPEC_HOST ? "Host" : "Device");
		chip->data_active = false;
		if (data_active_callback)
			(*data_active_callback)(data_active_payload, chip->active_data_role, false);

		if  (chip->active_data_role == TYPEC_HOST) {
			ret = max77759_write8(chip->data.regmap, TCPC_VENDOR_USBSW_CTRL,
					      USBSW_DISCONNECT);
			LOG(LOG_LVL_DEBUG, chip->log,
			    "Turning off dp switches %s", ret < 0 ? "fail" : "success");
		}
	}

	/* Renable BC1.2 */
	if (chip->attached && !attached && !bc12_get_status(chip->bc12))
		bc12_enable(chip->bc12, true);
	/*
	 * To prevent data stack enumeration failure, previously there
	 * was a 300msec delay here
	 */

	chip->attached = attached;
	chip->data_role = typec_data_role;
	enable_data_path_locked(chip);
	mutex_unlock(&chip->data_path_lock);
	usb_psy_set_attached_state(chip->usb_psy_data, chip->attached);

	/*
	 * Renable BC1.2 upon disconnect if disabled. Needed for sink-only mode such as
	 * fastbootd/Recovery.
	 */
	if (chip->attached && !attached && !bc12_get_status(chip->bc12))
		bc12_enable(chip->bc12, true);

	/*
	 * Clear COMPLIANCE_WARNING_INPUT_POWER_LIMITED which tracks AICL_ACTIVE only upon
	 * disconnect. This prevents the incommpatible charging notification to not change status
	 * during the charging session. AICL active is system/battery load dependent and
	 * hence can change status during a charge session.
	 */
	if (!attached) {
		update_compliance_warnings(chip, COMPLIANCE_WARNING_INPUT_POWER_LIMITED, false);
		/* Clear BC12 as fallback when hardware does not clear it on disconnect. */
		update_compliance_warnings(chip, COMPLIANCE_WARNING_BC12, false);

		/*
		 * b/335901921
		 * If someone calls tcpm_get_partner_src_caps before the charger sends the new Src
		 * Caps, the caller will get the old Src Caps which might be from the previous PD
		 * connection. To avoid this bug, clear nr_partner_src_caps if the attach session is
		 * ended (from the Type-C's perspective).
		 * The best solution is to call max77759_store_partner_src_caps vendor_hook from
		 * TCPM to clear partner_src_caps and nr_partner_src_caps when the cable is
		 * detached.
		 */
		spin_lock(&g_caps_lock);
		nr_partner_src_caps = 0;
		spin_unlock(&g_caps_lock);
	}

	return 0;
}

static void max77759_store_partner_src_caps(void *unused,
					    unsigned int *nr_source_caps,
					    u32 (*source_caps)[])
{
	int i;

	spin_lock(&g_caps_lock);

	nr_partner_src_caps = *nr_source_caps > PDO_MAX_OBJECTS ?
			      PDO_MAX_OBJECTS : *nr_source_caps;

	for (i = 0; i < nr_partner_src_caps; i++)
		partner_src_caps[i] = (*source_caps)[i];

	spin_unlock(&g_caps_lock);
}

/*
 * Don't call this function in interrupt context. Caller needs to free the
 * memory by calling tcpm_put_partner_src_caps.
 */
int tcpm_get_partner_src_caps(struct tcpm_port *port, u32 **src_pdo)
{
	int i, ret;

	*src_pdo = kcalloc(PDO_MAX_OBJECTS, sizeof(u32), GFP_KERNEL);
	if (!src_pdo)
		return -ENOMEM;

	spin_lock(&g_caps_lock);

	if (!nr_partner_src_caps) {
		ret = -ENODATA;
		goto cleanup;
	}

	for (i = 0, ret = nr_partner_src_caps; i < nr_partner_src_caps; i++)
		(*src_pdo)[i] = partner_src_caps[i];

	goto unlock;

cleanup:
	kfree(*src_pdo);
	*src_pdo = NULL;
unlock:
	spin_unlock(&g_caps_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tcpm_get_partner_src_caps);

void tcpm_put_partner_src_caps(u32 **src_pdo)
{
	kfree(*src_pdo);
	*src_pdo = NULL;
}
EXPORT_SYMBOL_GPL(tcpm_put_partner_src_caps);

void max77759_bc12_is_running(struct max77759_plat *chip, bool running)
{
	if (chip) {
		mutex_lock(&chip->data_path_lock);
		chip->bc12_running = running;
		if (!running)
			enable_data_path_locked(chip);
		mutex_unlock(&chip->data_path_lock);
	}
}

static void max77759_set_port_data_capable(struct i2c_client *tcpc_client,
					   enum power_supply_usb_type
					   usb_type)
{
	struct max77759_plat *chip = i2c_get_clientdata(tcpc_client);

	switch (usb_type) {
	case POWER_SUPPLY_USB_TYPE_SDP:
	case POWER_SUPPLY_USB_TYPE_CDP:
		mutex_lock(&chip->data_path_lock);
		chip->bc12_data_capable = true;
		enable_data_path_locked(chip);
		mutex_unlock(&chip->data_path_lock);
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
	case POWER_SUPPLY_USB_TYPE_UNKNOWN:
		mutex_lock(&chip->data_path_lock);
		chip->bc12_data_capable = false;
		enable_data_path_locked(chip);
		mutex_unlock(&chip->data_path_lock);
		break;
	default:
		chip->bc12_data_capable = false;
		break;
	}
}

static const unsigned int usbpd_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_MECHANICAL,
	EXTCON_NONE,
};

static int tcpci_init(struct google_shim_tcpci *tcpci, struct google_shim_tcpci_data *data)
{
	/*
	 * Generic TCPCI overwrites the regs once this driver initializes
	 * them. Prevent this by returning -1.
	 */
	return -1;
}

static int usb_throttle_votable_callback(struct gvotable_election *el,
					 const char *reason, void *value)
{
	struct max77759_plat *chip = gvotable_get_data(el);
	int throttled = (long)value ? USB_SUSPENDED : USB_RESUMED;

	mutex_lock(&chip->rc_lock);
	chip->usb_throttled =  throttled;
	LOG(LOG_LVL_DEBUG, chip->log, "%s: reason %s value %ld\n", __func__, reason, (long)value);
	mutex_unlock(&chip->rc_lock);

	return 0;
}

static int max77759_toggle_disable_votable_callback(struct gvotable_election *el,
						    const char *reason, void *value)
{
	struct max77759_plat *chip = gvotable_get_data(el);
	int disable = (long)value ? MAX77759_DISABLE_TOGGLE : MAX77759_ENABLE_TOGGLE;

	mutex_lock(&chip->rc_lock);
	if (chip->toggle_disable_status == disable) {
		mutex_unlock(&chip->rc_lock);
		return 0;
	}

	chip->toggle_disable_status = disable;
	if (chip->toggle_disable_status) {
		update_contaminant_detection_locked(chip, CONTAMINANT_DETECT_DISABLE);
		max777x9_disable_contaminant_detection(chip);
		max77759_enable_toggling_locked(chip, false);
		/*
		 * If external Vbus OVP is present, disable it to block Vbus.
		 * If there is no external Vbus OVP, inform TCPM of the change on Vbus.
		 * The mock Vbus absence will be reported in max77759_get_vbus callback.
		 */
		if (chip->in_switch_gpio >= 0) {
			ovp_operation(chip, OVP_OFF);
			LOG(LOG_LVL_DEBUG, chip->log, "[%s]: Disable in-switch set %s / active %s",
			    __func__, !chip->in_switch_gpio_active_high ? "high" : "low",
			    chip->in_switch_gpio_active_high ? "high" : "low");
		} else {
			tcpm_vbus_change(chip->tcpci->port);
		}
	} else {
		if (chip->contaminant_detection_userspace)
			update_contaminant_detection_locked(chip,
							    chip->contaminant_detection_userspace);
		else
			max77759_enable_toggling_locked(chip, true);
		/*
		 * If external Vbus OVP is present, enable it to reflect the real Vbus status.
		 * If there is no external Vbus OVP, inform TCPM of the change on Vbus.
		 * The real Vbus status will be queried in max77759_get_vbus callback.
		 */
		if (chip->in_switch_gpio >= 0) {
			ovp_operation(chip, OVP_ON);
			LOG(LOG_LVL_DEBUG, chip->log, "[%s]: Enable in-switch set %s / active %s",
			    __func__, chip->in_switch_gpio_active_high ? "high" : "low",
			    chip->in_switch_gpio_active_high ? "high" : "low");
		} else {
			tcpm_vbus_change(chip->tcpci->port);
		}
	}
	mutex_unlock(&chip->rc_lock);
	LOG(LOG_LVL_DEBUG, chip->log, "%s: reason %s value %ld\n", __func__, reason, (long)value);
	return 0;
}

#ifdef CONFIG_DEBUG_FS
static ssize_t force_device_mode_on_write(struct file *file, const char __user *ubuf, size_t count,
					  loff_t *ppos)
{
	struct max77759_plat *chip = file->private_data;
	long result, ret;

	ret = kstrtol_from_user(ubuf, count, 10, &result);
	if (ret)
		return ret;

	if (result == chip->force_device_mode_on)
		return count;

	mutex_lock(&chip->data_path_lock);
	chip->force_device_mode_on = result;
	/* Tear down previous data role if needed */
	if (((result && chip->active_data_role != TYPEC_DEVICE) ||
	    (!result && chip->active_data_role != chip->data_role)) && chip->data_active) {
		ret = extcon_set_state_sync(chip->extcon,
					    chip->active_data_role == TYPEC_HOST ?
					    EXTCON_USB_HOST : EXTCON_USB, 0);

		LOG(LOG_LVL_DEBUG, chip->log, "%s: %s turning off %s",
		    __func__, ret < 0 ? "Failed" : "Succeeded",
		    chip->active_data_role == TYPEC_HOST ? "Host" : "Device");
		chip->data_active = false;
		if (data_active_callback)
			(*data_active_callback)(data_active_payload, chip->active_data_role, false);
	}

	if (result && !chip->data_active) {
		ret = extcon_set_state_sync(chip->extcon, EXTCON_USB, 1);
		LOG(LOG_LVL_DEBUG, chip->log,
		    "%s: %s turning on device", __func__, ret < 0 ? "Failed" : "Succeeded");
		chip->data_active = !ret;
		chip->active_data_role = TYPEC_DEVICE;
		if (data_active_callback)
			(*data_active_callback)(data_active_payload, chip->active_data_role, true);
	} else if (!result) {
		enable_data_path_locked(chip);
	}

	mutex_unlock(&chip->data_path_lock);
	return count;
}

static ssize_t force_device_mode_on_read(struct file *file, char __user *userbuf, size_t count,
					 loff_t *ppos)
{
	struct max77759_plat *chip = file->private_data;
	char buf[16];
	int ret;

	ret = snprintf(buf, sizeof(buf) - 1, "%d\n", chip->force_device_mode_on);

	return simple_read_from_buffer(userbuf, count, ppos, buf, ret);
}

static const struct file_operations force_device_mode_on_fops = {
	.read	= force_device_mode_on_read,
	.write	= force_device_mode_on_write,
	.open	= simple_open,
	.llseek = default_llseek,
};
#endif

static void max77759_get_timer_value(void *unused, const char *state, enum typec_timer timer,
				     unsigned int *val)
{
	switch (timer) {
	case SINK_DISCOVERY_BC12:
		*val = sink_discovery_delay_ms;
		break;
	case SINK_WAIT_CAP:
		*val = 450;
		break;
	case SOURCE_OFF:
		*val = 870;
		break;
	case CC_DEBOUNCE:
		*val = 170;
		break;
	default:
		break;
	}
}

static void max77759_tcpm_log(void *unused, const char *log, bool *bypass)
{
	if (tcpm_log)
		LOG(LOG_LVL_DEBUG, tcpm_log, "%s", log);

	*bypass = true;
}

static void max77759_modify_src_caps(void *unused, unsigned int *nr_src_pdo,
				     u32 (*src_pdo)[], bool *modified)
{
	spin_lock(&g_caps_lock);

	if (port_src_pdo_updated) {
		spin_unlock(&g_caps_lock);
		return;
	}

	if (limit_src_cap_enable) {
		(*src_pdo)[0] &= ~(PDO_CURR_MASK << PDO_FIXED_CURR_SHIFT);
		(*src_pdo)[0] |= PDO_FIXED_CURR(SRC_CURRENT_LIMIT_MA);
		*nr_src_pdo = 1;
	} else {
		(*src_pdo)[0] |= PDO_FIXED_CURR(orig_src_current);
		*nr_src_pdo = nr_orig_src_pdo;
	}

	port_src_pdo_updated = true;
	*modified = true;

	spin_unlock(&g_caps_lock);
}

static int max77759_register_vendor_hooks(struct i2c_client *client)
{
	int ret;

	if (hooks_installed)
		return 0;

	ret = register_trace_android_vh_typec_store_partner_src_caps(
			max77759_store_partner_src_caps, NULL);
	if (ret) {
		dev_err(&client->dev,
			"register_trace_android_vh_typec_store_partner_src_caps failed ret:%d\n",
			ret);
		return ret;
	}

	ret = register_trace_android_vh_typec_tcpm_get_timer(max77759_get_timer_value, NULL);
	if (ret) {
		dev_err(&client->dev,
			"register_trace_android_vh_typec_tcpm_get_timer failed ret:%d\n", ret);
		return ret;
	}

	ret = register_trace_android_vh_typec_tcpm_log(max77759_tcpm_log, NULL);
	if (ret) {
		dev_err(&client->dev,
			"register_trace_android_vh_typec_tcpm_log failed ret:%d\n", ret);
		return ret;
	}

	port_src_pdo_updated = true;
	ret = register_trace_android_vh_typec_tcpm_modify_src_caps(max77759_modify_src_caps, NULL);
	if (ret) {
		dev_err(&client->dev,
			"register_trace_android_vh_typec_tcpm_modify_src_caps failed ret:%d\n",
			ret);
		return ret;
	}

	hooks_installed = true;

	return ret;
}

static void reenable_auto_ultra_low_power_mode_work_item(struct kthread_work *work)
{
	struct max77759_plat *chip = container_of(work, struct max77759_plat,
						  reenable_auto_ultra_low_power_mode_work);

	chip->floating_cable_or_sink_detected = 0;
	max777x9_disable_auto_ultra_low_power_mode(chip, false);
}

static enum alarmtimer_restart reenable_auto_ultra_low_power_mode_alarm_handler(struct alarm *alarm,
										ktime_t time)
{
	struct max77759_plat *chip = container_of(alarm, struct max77759_plat,
						  reenable_auto_ultra_low_power_mode_alarm);

	logbuffer_log(chip->log, "timer fired: enable_auto_ultra_low_power_mode");
	if (max777x9_is_contaminant_detected(chip)) {
		LOG(LOG_LVL_DEBUG, chip->log,
			      "Skipping enable_auto_ultra_low_power_mode. Dry detection in progress");
		goto exit;
	}
	kthread_queue_work(chip->wq, &chip->reenable_auto_ultra_low_power_mode_work);
	pm_wakeup_event(chip->dev, PD_ACTIVITY_TIMEOUT_MS);

exit:
	return ALARMTIMER_NORESTART;
}

static void max_tcpci_check_contaminant(struct google_shim_tcpci *tcpci,
					struct google_shim_tcpci_data *tdata)
{
	struct max77759_plat *chip = tdata_to_max77759(tdata);
	bool contaminant_cc_status_handled = false, port_clean = false;
	int ret = 0;

	mutex_lock(&chip->rc_lock);
	LOG(LOG_LVL_DEBUG, chip->log, "max_tcpci_check_contaminant");
	if (chip->usb_throttled) {
		LOG(LOG_LVL_DEBUG, chip->log, "usb throttled; port clean");
		tcpm_port_clean(chip->port);
		mutex_unlock(&chip->rc_lock);
		return;
	}
	if (chip->contaminant_detection) {
		ret = max777x9_process_contaminant_alert(chip->contaminant, true, false,
							 &contaminant_cc_status_handled,
							 &port_clean);
		if (ret < 0) {
			logbuffer_logk(chip->log, LOGLEVEL_ERR, "I/O error in %s", __func__);
			/* Assume clean port */
			tcpm_port_clean(chip->port);
		} else if (port_clean) {
			LOG(LOG_LVL_DEBUG, chip->log, "port clean");
			tcpm_port_clean(chip->port);
		} else {
			LOG(LOG_LVL_DEBUG, chip->log, "port dirty");
			chip->check_contaminant = true;
		}
	} else {
		LOG(LOG_LVL_DEBUG, chip->log, "port clean; Contaminant detection not enabled");
		tcpm_port_clean(chip->port);
	}
	mutex_unlock(&chip->rc_lock);
}

static void dp_notification_work_item(struct kthread_work *work)
{
	struct dp_notification_event *evt = container_of(work, struct dp_notification_event,
							 dp_notification_work);
	struct max77759_plat *chip = evt->chip;
	int dp, ret;

	logbuffer_logk(chip->log, LOGLEVEL_INFO, "dp wq %s: %lu", __func__, evt->mode);

	switch (evt->mode) {
	case TYPEC_DP_STATE_A:
	case TYPEC_DP_STATE_C:
	case TYPEC_DP_STATE_E:
		dp = 1;
		chip->lanes = 4;
		if (chip->sbu_mux_en_gpio >= 0)
			gpio_set_value_cansleep(chip->sbu_mux_en_gpio, 1);
		gpio_set_value_cansleep(chip->sbu_mux_sel_gpio,
					chip->orientation == TYPEC_ORIENTATION_NORMAL ?
					0 : 1);
		break;
	case TYPEC_DP_STATE_B:
	case TYPEC_DP_STATE_D:
	case TYPEC_DP_STATE_F:
		dp = 1;
		chip->lanes = 2;
		if (chip->sbu_mux_en_gpio >= 0)
			gpio_set_value_cansleep(chip->sbu_mux_en_gpio, 1);
		gpio_set_value_cansleep(chip->sbu_mux_sel_gpio,
					chip->orientation == TYPEC_ORIENTATION_NORMAL ?
					0 : 1);
		break;
	default:
		dp = 0;
	}

	if ((dp && !chip->dp_regulator_enabled) || (!dp && chip->dp_regulator_enabled)) {
		ret = dp ? regulator_enable(chip->dp_regulator) : \
			   regulator_disable(chip->dp_regulator);
		if (ret >= 0)
			chip->dp_regulator_enabled = dp;
		LOG(LOG_LVL_DEBUG, chip->log, "dp regulator_%s %s ret:%d",
		    dp ? "enable" : "disable", ret < 0 ? "fail" : "success", ret);
		ret = dp ? regulator_set_voltage(chip->dp_regulator, VOLTAGE_DP_AUX_DEFAULT_UV,
						 VOLTAGE_DP_AUX_DEFAULT_UV) : \
			   regulator_set_voltage(chip->dp_regulator, chip->dp_regulator_min_uv,
						 chip->dp_regulator_max_uv);
		LOG(LOG_LVL_DEBUG, chip->log,
		    "dp regulator_set_voltage %s ret:%d", ret < 0 ? "fail" : "success", ret);
	}

	if (chip->product_id == MAX77779_PRODUCT_ID) {
		ret = max77759_write8(chip->data.regmap, TCPC_VENDOR_SBUSW_CTRL, dp ?
				      (chip->orientation == TYPEC_ORIENTATION_REVERSE ?
				      SBUSW_XBAR_POL_REVERSE : SBUSW_XBAR_POL_NORMAL) :
				      (modparam_conf_sbu ? SBUSW_SERIAL_UART : 0));
		LOG(LOG_LVL_DEBUG, chip->log, "SBU Cross Bar SW %s %s, orientation:%d ret:%d",
		    dp ? "Enable" : "Disable", ret < 0 ? "fail" : "success", chip->orientation,
		    ret);
	} else {
		ret = max77759_write8(chip->data.regmap, TCPC_VENDOR_SBUSW_CTRL, dp ? SBUSW_PATH_1 :
				      (modparam_conf_sbu ? SBUSW_SERIAL_UART : 0));
	}

	LOG(LOG_LVL_DEBUG, chip->log, "%s Signaling dp altmode: %s ret:%d",
	    ret < 0 ? "Failed" : "Succeeded", dp ? "on" : "off", ret);
	logbuffer_logk(chip->log, LOGLEVEL_INFO, "dp altmode orientation:%d lanes:%d dp:%d",
		      (int)chip->orientation, chip->lanes, dp);

	devm_kfree(chip->dev, evt);
}

static int max77759_usb_set_mode(struct typec_mux_dev *mux, struct typec_mux_state *state)
{
	struct max77759_plat *chip = typec_mux_get_drvdata(mux);
	struct dp_notification_event *evt;

	if (!state || !state->alt) {
		LOG(LOG_LVL_DEBUG, chip->log, "%s: dropping event", __func__);
		return 0;
	}

	evt = devm_kzalloc(chip->dev, sizeof(*evt), GFP_KERNEL);
	if (!evt) {
		LOG(LOG_LVL_DEBUG, chip->log, "dp notification: Dropping event");
		return 0;
	}
	kthread_init_work(&evt->dp_notification_work, dp_notification_work_item);
	evt->chip = chip;
	evt->mode = state->mode;
	kthread_queue_work(chip->dp_notification_wq, &evt->dp_notification_work);
	pm_wakeup_event(chip->dev, PD_ACTIVITY_TIMEOUT_MS);
	return 0;
}

static int max77759_setup_data_notifier(struct max77759_plat *chip)
{
	struct usb_role_switch_desc desc = { };
	struct typec_switch_desc sw_desc = { };
	struct typec_mux_desc mux_desc = {};
	u32 conn_handle;
	int ret;

	chip->extcon = devm_extcon_dev_allocate(chip->dev, usbpd_extcon_cable);
	if (IS_ERR(chip->extcon)) {
		dev_err(chip->dev, "Error allocating extcon: %ld\n",
			PTR_ERR(chip->extcon));
		return PTR_ERR(chip->extcon);
	}

	ret = devm_extcon_dev_register(chip->dev, chip->extcon);
	if (ret < 0) {
		dev_err(chip->dev, "failed to register extcon device:%d\n", ret);
		return ret;
	}

	extcon_set_property_capability(chip->extcon, EXTCON_USB,
				       EXTCON_PROP_USB_TYPEC_POLARITY);
	extcon_set_property_capability(chip->extcon, EXTCON_USB_HOST,
				       EXTCON_PROP_USB_TYPEC_POLARITY);

	of_property_read_u32(dev_of_node(chip->dev), "conn", &conn_handle);
	desc.fwnode = &of_find_node_by_phandle(conn_handle)->fwnode;
	desc.driver_data = chip;
	desc.name = fwnode_get_name(dev_fwnode(chip->dev));
	desc.set = max77759_usb_set_role;

	chip->usb_sw = usb_role_switch_register(chip->dev, &desc);
	if (IS_ERR(chip->usb_sw)) {
		ret = PTR_ERR(chip->usb_sw);
		dev_err(chip->dev, "Error while registering role switch:%d\n", ret);
		return ret;
	}

	sw_desc.fwnode = dev_fwnode(chip->dev);
	sw_desc.drvdata = chip;
	sw_desc.name = fwnode_get_name(dev_fwnode(chip->dev));
	sw_desc.set = max77759_usb_set_orientation;

	chip->typec_sw = typec_switch_register(chip->dev, &sw_desc);
	if (IS_ERR(chip->typec_sw)) {
		ret = PTR_ERR(chip->typec_sw);
		dev_err(chip->dev, "Error while registering orientation switch:%d\n", ret);
		goto usb_sw_free;
	}

	mux_desc.fwnode = dev_fwnode(chip->dev);
	mux_desc.drvdata = chip;
	mux_desc.name = fwnode_get_name(dev_fwnode(chip->dev));
	mux_desc.set = max77759_usb_set_mode;

	chip->mode_mux = typec_mux_register(chip->dev, &mux_desc);
	if (IS_ERR(chip->mode_mux)) {
		ret = PTR_ERR(chip->mode_mux);
		dev_err(chip->dev, "Error while registering mode mux:%d\n", ret);
		goto usb_sw_free;
	}

	return 0;

usb_sw_free:
	usb_role_switch_unregister(chip->usb_sw);
	return ret;
}

static void max77759_teardown_data_notifier(struct max77759_plat *chip)
{
	if (!IS_ERR_OR_NULL(chip->typec_sw))
		typec_switch_unregister(chip->typec_sw);
	if (!IS_ERR_OR_NULL(chip->usb_sw))
		usb_role_switch_unregister(chip->usb_sw);
}

static bool is_aicl_limited(struct max77759_plat *chip)
{
	unsigned int vbus_present, snk_vbus, pwr_status;
	union power_supply_propval current_now = {0};
	int ret;
	bool default_power, is_dcp;

	ret = regmap_read(chip->data.regmap, TCPC_POWER_STATUS, &pwr_status);
	if (ret < 0) {
		LOG(LOG_LVL_DEBUG, chip->log, "Abort %s; TCPC_POWER_STATUS read error", __func__);
		return false;
	}

	vbus_present = pwr_status & TCPC_POWER_STATUS_VBUS_PRES;
	snk_vbus = pwr_status & TCPC_POWER_STATUS_SINKING_VBUS;
	power_supply_get_property(chip->usb_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &current_now);
	default_power = !(chip->cc1 == TYPEC_CC_RP_3_0 || chip->cc1 == TYPEC_CC_RP_1_5 ||
			  chip->cc2 == TYPEC_CC_RP_3_0 || chip->cc2 == TYPEC_CC_RP_1_5);
	is_dcp = (get_usb_type(chip->bc12) == POWER_SUPPLY_USB_TYPE_DCP);

	LOG(LOG_LVL_DEBUG, chip->log,
	    "AICL %s active vbus_present:%c snk_vbus:%c current_now:%d default_power:%c DCP:%c",
	    chip->aicl_active ? "" : "not", vbus_present ? 'y' : 'n', snk_vbus ? 'y' : 'n',
	    current_now.intval, default_power ? 'y' : 'n', is_dcp ? 'y' : 'n');
	/*
	 * AICL_ACTIVE + Charging over USB + USB input current less than 500mA and charging from
	 * default power sources.
	 *
	 * USB input current could be reported as 0 in scenarios such as charge full.
	 * Exclude these cases as input current should not be 0 esp. when input current is limited.
	 */
	if (!current_now.intval)
		return false;
	else if (chip->aicl_active && vbus_present && snk_vbus && current_now.intval < 500000 &&
		 default_power && is_dcp)
		return true;

	return false;
}

static void aicl_check_alarm_work_item(struct kthread_work *work)
{
	struct max77759_plat *chip = container_of(work, struct max77759_plat,
						  aicl_check_alarm_work);

	/*
	 * Set here and clear COMPLIANCE_WARNING_INPUT_POWER_LIMITED which tracks AICL_ACTIVE only
	 * upon disconnect. This prevents the incommpatible charging notification to not change
	 * status during the charging session. AICL active is system/battery load dependent and
	 * hence can change status during a charge session.
	 */
	if (is_aicl_limited(chip))
		update_compliance_warnings(chip, COMPLIANCE_WARNING_INPUT_POWER_LIMITED, true);
}

static enum alarmtimer_restart aicl_check_alarm_handler(struct alarm *alarm, ktime_t time)
{
	struct max77759_plat *chip = container_of(alarm, struct max77759_plat, aicl_check_alarm);

	LOG(LOG_LVL_DEBUG, chip->log, "timer fired: %s", __func__);
	kthread_queue_work(chip->wq, &chip->aicl_check_alarm_work);
	pm_wakeup_event(chip->dev, AICL_CHECK_MS);

	return ALARMTIMER_NORESTART;
}

static int max77759_aicl_active_cb(struct gvotable_election *el, const char *reason, void *value)
{
	struct max77759_plat *chip = gvotable_get_data(el);
	bool aicl_active = !!(long)value;

	chip->aicl_active = aicl_active;

	if (is_aicl_limited(chip)) {
		/* Recheck after AICL_CHECK_MS */
		alarm_start_relative(&chip->aicl_check_alarm, ms_to_ktime(AICL_CHECK_MS));
	} else {
		alarm_cancel(&chip->aicl_check_alarm);
		kthread_cancel_work_sync(&chip->aicl_check_alarm_work);
	}

	return 0;
}

static int max77759_probe(struct i2c_client *client,
			  const struct i2c_device_id *i2c_id)
{
	int ret, i;
	struct max77759_plat *chip;
	char *usb_psy_name;
	struct device_node *dn, *ovp_dn, *regulator_dn, *conn;
	u8 power_status, pid;
	u16 device_id;
	u32 ovp_handle, regulator_handle;
	const char *ovp_status;
	enum of_gpio_flags flags;
	u32 first_src_pdo = 0;

	ret = max77759_register_vendor_hooks(client);
	if (ret)
		return ret;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->client->dev.init_name = "i2c-max77759tcpc";
	chip->data.regmap = devm_regmap_init_i2c(client,
						 &max77759_regmap_config);
	if (IS_ERR(chip->data.regmap)) {
		dev_err(&client->dev, "Regmap init failed\n");
		return PTR_ERR(chip->data.regmap);
	}

	dn = dev_of_node(&client->dev);
	if (!dn) {
		dev_err(&client->dev, "of node not found\n");
		return -EINVAL;
	}

	chip->charger_mode_votable = gvotable_election_get_handle(GBMS_MODE_VOTABLE);
	if (IS_ERR_OR_NULL(chip->charger_mode_votable)) {
		dev_err(&client->dev, "TCPCI: GBMS_MODE_VOTABLE get failed: %ld",
			PTR_ERR(chip->charger_mode_votable));
		if (!of_property_read_bool(dn, "gvotable-lazy-probe"))
			return -EPROBE_DEFER;
	}
	kthread_init_work(&chip->reenable_auto_ultra_low_power_mode_work,
			  reenable_auto_ultra_low_power_mode_work_item);
	alarm_init(&chip->reenable_auto_ultra_low_power_mode_alarm, ALARM_BOOTTIME,
		   reenable_auto_ultra_low_power_mode_alarm_handler);
	kthread_init_work(&chip->aicl_check_alarm_work, aicl_check_alarm_work_item);
	alarm_init(&chip->aicl_check_alarm, ALARM_BOOTTIME, aicl_check_alarm_handler);

	chip->in_switch_gpio = -EINVAL;
	if (of_property_read_bool(dn, "ovp-present")) {
		chip->in_switch_gpio = of_get_named_gpio_flags(dn, "in-switch-gpio", 0, &flags);
		if (chip->in_switch_gpio < 0) {
			dev_err(&client->dev, "in-switch-gpio not found\n");
			return -EPROBE_DEFER;
		}
		chip->in_switch_gpio_active_high = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	} else if (!of_property_read_u32(dn, "max20339,ovp", &ovp_handle)) {
		ovp_dn = of_find_node_by_phandle(ovp_handle);
		if (!IS_ERR_OR_NULL(ovp_dn) &&
		    !of_property_read_string(ovp_dn, "status", &ovp_status) &&
		    strncmp(ovp_status, "disabled", strlen("disabled"))) {
			chip->in_switch_gpio = of_get_named_gpio_flags(dn, "in-switch-gpio", 0,
								       &flags);
			if (chip->in_switch_gpio < 0) {
				dev_err(&client->dev, "in-switch-gpio not found\n");
				return -EPROBE_DEFER;
			}
			chip->in_switch_gpio_active_high = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		}
	}

	chip->sbu_mux_en_gpio = of_get_named_gpio_flags(dn, "sbu-mux-en-gpio", 0, &flags);
	if (chip->sbu_mux_en_gpio < 0) {
		dev_err(&client->dev, "sbu-mux-en-gpio not found\n");
	}
	chip->sbu_mux_sel_gpio = of_get_named_gpio_flags(dn, "sbu-mux-sel-gpio", 0, &flags);
	if (chip->sbu_mux_sel_gpio < 0) {
		dev_err(&client->dev, "sbu-mux-sel-gpio not found\n");
	}
	if (of_property_read_bool(dn, "bcl-usb-voting")) {
		chip->bcl_usb_votable = gvotable_election_get_handle(BCL_USB_VOTABLE);
		if (IS_ERR_OR_NULL(chip->bcl_usb_votable))
			dev_err(&client->dev, "TCPCI: BCL_USB_VOTABLE get failed: %ld",
				PTR_ERR(chip->bcl_usb_votable));
	}
	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->icl_proto_el_lock);
	mutex_init(&chip->data_path_lock);
	mutex_init(&chip->rc_lock);
	mutex_init(&chip->irq_status_lock);
	mutex_init(&chip->ovp_lock);
	mutex_init(&chip->ext_bst_ovp_clear_lock);
	spin_lock_init(&g_caps_lock);
	chip->first_toggle = true;

	ret = max77759_read8(chip->data.regmap, TCPC_POWER_STATUS,
			     &power_status);
	if (ret < 0)
		return ret;

	if (power_status & TCPC_POWER_STATUS_UNINIT) {
		dev_err(&client->dev, "TCPC not ready!");
		return -EPROBE_DEFER;
	}

	chip->toggle_disable_votable =
		gvotable_create_bool_election(NULL, max77759_toggle_disable_votable_callback, chip);
	if (IS_ERR_OR_NULL(chip->toggle_disable_votable)) {
		ret = PTR_ERR(chip->toggle_disable_votable);
		dev_err(chip->dev, "no toggle_disable votable (%d)\n", ret);
		return ret;
	}
	gvotable_set_vote2str(chip->toggle_disable_votable, gvotable_v2s_int);
	gvotable_election_set_name(chip->toggle_disable_votable, "TOGGLE_DISABLE");

	chip->usb_throttle_votable =
		gvotable_create_bool_election(NULL, usb_throttle_votable_callback, chip);
	if (IS_ERR_OR_NULL(chip->usb_throttle_votable)) {
		ret = PTR_ERR(chip->usb_throttle_votable);
		dev_err(chip->dev, "USB throttle votable (%d) failed to create\n", ret);
		return ret;
	}
	gvotable_set_vote2str(chip->usb_throttle_votable, gvotable_v2s_int);
	gvotable_election_set_name(chip->usb_throttle_votable, USB_THROTTLE_VOTABLE);

	/* Chip level tcpci callbacks */
	chip->data.set_vbus = max77759_set_vbus;
	chip->data.start_drp_toggling = max77759_start_toggling;
	chip->data.TX_BUF_BYTE_x_hidden = 1;
	chip->data.vbus_vsafe0v = true;
	chip->data.set_partner_usb_comm_capable = max77759_set_partner_usb_comm_capable;
	chip->data.init = tcpci_init;
	chip->data.frs_sourcing_vbus = max77759_frs_sourcing_vbus;
	chip->data.check_contaminant = max_tcpci_check_contaminant;
	chip->data.get_vbus = max77759_get_vbus;

	chip->compliance_warnings = init_compliance_warnings(chip);
	if (IS_ERR_OR_NULL(chip->compliance_warnings)) {
		ret = PTR_ERR(chip->compliance_warnings);
		dev_err(&client->dev, "init_compliance_warnings failed, ptr: %d", ret);
		return ret;
	}

	chip->log = logbuffer_register("usbpd");
	if (IS_ERR_OR_NULL(chip->log)) {
		dev_err(&client->dev, "logbuffer get failed");
		chip->log = NULL;
	}

	chip->psy_ops.tcpc_get_vbus_voltage_mv =
		max77759_get_vbus_voltage_mv;
	chip->psy_ops.tcpc_get_vbus_voltage_max_mv =
		max77759_get_vbus_voltage_max_mv;
	chip->psy_ops.tcpc_set_vbus_voltage_max_mv =
		max77759_set_vbus_voltage_max_mv;
	chip->psy_ops.tcpc_set_port_data_capable =
		max77759_set_port_data_capable;
	chip->usb_psy_data = usb_psy_setup(client, chip->log, &chip->psy_ops, chip,
					   &max77759_non_compliant_bc12_callback);
	if (IS_ERR_OR_NULL(chip->usb_psy_data)) {
		dev_err(&client->dev, "USB psy failed to initialize");
		ret = PTR_ERR(chip->usb_psy_data);
		goto logbuffer_unreg;
	}

	/* Defered probe returned until usb power supply showup.*/
	chip->bc12 = bc12_init(chip, max77759_bc12_is_running);
	if (IS_ERR_OR_NULL(chip->bc12)) {
		ret = PTR_ERR(chip->bc12);
		goto unreg_psy;
	}

	usb_psy_name = (char *)of_get_property(dn, "usb-psy-name", NULL);
	if (!usb_psy_name) {
		dev_err(&client->dev, "usb-psy-name not set\n");
		ret = -EINVAL;
		goto teardown_bc12;
	}

	chip->no_bc_12 = of_property_read_bool(dn, "no-bc-12");
	chip->no_external_boost = of_property_read_bool(dn, "no-external-boost");
	of_property_read_u32(dn, "sink-discovery-delay-ms", &sink_discovery_delay_ms);

	conn = of_get_child_by_name(dn, "connector");
	if (!conn) {
		dev_err(&client->dev, "connector node not present\n");
		ret = -ENODEV;
		goto teardown_bc12;
	}

	/* DRP is expected and "source-pdos" should be present in device tree */
	nr_orig_src_pdo = of_property_count_u32_elems(conn, "source-pdos");
	if (nr_orig_src_pdo < 0) {
		dev_err(&client->dev, "failed to count elems in source-pdos\n");
		of_node_put(conn);
		ret = nr_orig_src_pdo;
		goto teardown_bc12;
	}

	ret = of_property_read_u32_index(conn, "source-pdos", 0, &first_src_pdo);
	of_node_put(conn);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read the first source-pdo\n");
		goto teardown_bc12;
	}
	orig_src_current = ((first_src_pdo >> PDO_FIXED_CURR_SHIFT) & PDO_CURR_MASK) * 10;

	chip->usb_psy = power_supply_get_by_name(usb_psy_name);
	if (IS_ERR_OR_NULL(chip->usb_psy) || !chip->usb_psy) {
		dev_err(&client->dev, "usb psy not up\n");
		ret = -EPROBE_DEFER;
		goto teardown_bc12;
	}

	chip->dp_regulator = devm_regulator_get(chip->dev, "pullup");
	if (IS_ERR_OR_NULL(chip->dp_regulator) ) {
		dev_err(&client->dev, "pullup regulator get failed %ld\n",
			PTR_ERR(chip->dp_regulator));
		ret = -EPROBE_DEFER;
		goto psy_put;
	}
	if (!of_property_read_u32(dn, "pullup-supply", &regulator_handle)) {
		regulator_dn = of_find_node_by_phandle(regulator_handle);
		if (!IS_ERR_OR_NULL(regulator_dn)) {
			if (of_property_read_u32(regulator_dn, "regulator-min-microvolt",
						 &chip->dp_regulator_min_uv)) {
				dev_err(&client->dev, "failed to read regulator-min-microvolt\n");
				goto psy_put;
			}
			if (of_property_read_u32(regulator_dn, "regulator-max-microvolt",
						 &chip->dp_regulator_max_uv)) {
				dev_err(&client->dev, "failed to read regulator-max-microvolt\n");
				goto psy_put;
			}
		}
	}

	ret = max77759_read16(chip->data.regmap, TCPC_BCD_DEV, &device_id);
	if (ret < 0)
		goto dp_regulator_put;

	LOG(LOG_LVL_DEBUG, chip->log, "TCPC DEVICE id:%d", device_id);

	ret = max77759_read8(chip->data.regmap, TCPC_PRODUCT_ID, &pid);
	if (ret < 0)
		goto dp_regulator_put;
	LOG(LOG_LVL_DEBUG, chip->log, "TCPC PID:%d", pid);

	/* Default enable on A1 or higher on MAX77759 */
	chip->contaminant_detection =
		((pid == MAX77759_PRODUCT_ID) && (device_id >= MAX77759_DEVICE_ID_A1)) ||
		(pid == MAX77779_PRODUCT_ID);
	chip->contaminant_detection_userspace = chip->contaminant_detection;
	if (chip->contaminant_detection) {
		LOG(LOG_LVL_DEBUG, chip->log, "Contaminant detection enabled");
		chip->data.check_contaminant = max_tcpci_check_contaminant;
		chip->contaminant = max777x9_contaminant_init(chip, chip->contaminant_detection,
							      pid == MAX77779_PRODUCT_ID);
	}

	/* Cache product_id to determine dp_regulator handling */
	chip->product_id = pid;

	ret = max77759_setup_data_notifier(chip);
	if (ret < 0)
		goto dp_regulator_put;
	max77759_init_regs(chip->data.regmap, chip->log);

	/* Default enable on MAX77759 A1 or higher. Default enable on MAX77779 */
	if (pid == MAX77779_PRODUCT_ID || device_id >= MAX77759_DEVICE_ID_A1) {
		chip->manual_disable_vbus = of_property_read_bool(dn, "manual-disable-vbus");
		dev_info(&client->dev, "manual disable_vbus %u", chip->manual_disable_vbus);
		chip->data.auto_discharge_disconnect = true;
		chip->frs = true;
	}

	chip->wq = kthread_create_worker(0, "wq-tcpm-tcpc");
	if (IS_ERR_OR_NULL(chip->wq)) {
		ret = PTR_ERR(chip->wq);
		goto teardown_data;
	}

	chip->dp_notification_wq = kthread_create_worker(0, "wq-tcpc-dp-notification");
	if (IS_ERR_OR_NULL(chip->dp_notification_wq)) {
		ret = PTR_ERR(chip->dp_notification_wq);
		goto destroy_worker;
	}
	if (of_property_read_bool(dn, "bcl-usb-voting")) {
		chip->bcl_usb_wq = kthread_create_worker(0, "wq-bcl-usb");
		if (IS_ERR_OR_NULL(chip->bcl_usb_wq)) {
			ret = PTR_ERR(chip->bcl_usb_wq);
			goto destroy_dp_worker;
		}
		kthread_init_delayed_work(&chip->bcl_usb_votable_work, bcl_usb_vote_work);
	}

	kthread_init_delayed_work(&chip->icl_work, icl_work_item);
	kthread_init_delayed_work(&chip->enable_vbus_work, enable_vbus_work);
	kthread_init_delayed_work(&chip->vsafe0v_work, vsafe0v_debounce_work);
	kthread_init_delayed_work(&chip->max77759_io_error_work, max77759_io_error_work);
	kthread_init_delayed_work(&chip->check_missing_rp_work, check_missing_rp_work);
	kthread_init_delayed_work(&chip->ext_bst_ovp_clear_work, ext_bst_ovp_clear_work);

	/*
	 * b/218797880 Some OVP chips are restricted to quick Vin ramp-up time which means that if
	 * the ramp-up time is longer than a certain value, the OVP will keep being disabled if the
	 * status of the ON pin has been already set to active.
	 */
	chip->quick_ramp_vbus_ovp = of_property_read_bool(dn, "quick-ramp-vbus-ovp");
	if (chip->quick_ramp_vbus_ovp)
		kthread_init_delayed_work(&chip->reset_ovp_work, reset_ovp_work);

	chip->psy_notifier.notifier_call = psy_changed;
	ret = power_supply_reg_notifier(&chip->psy_notifier);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register power supply callback\n");
		goto destroy_usb_bcl_worker;
	}

	chip->usb_icl_proto_el = gvotable_election_get_handle(USB_ICL_PROTO_EL);
	if (IS_ERR_OR_NULL(chip->usb_icl_proto_el)) {
		dev_err(&client->dev, "TCPCI: USB ICL PROTO EL get failed:%ld",
			PTR_ERR(chip->usb_icl_proto_el));
		ret = -ENODEV;
		goto unreg_notifier;
	}

	chip->usb_icl_el = gvotable_election_get_handle(USB_ICL_EL);
	if (IS_ERR_OR_NULL(chip->usb_icl_el)) {
		dev_err(&client->dev, "TCPCI: USB ICL EL get failed:%ld",
			PTR_ERR(chip->usb_icl_el));
		ret = -ENODEV;
		goto unreg_notifier;
	}

	chip->aicl_active_el =
		gvotable_create_bool_election(AICL_ACTIVE_EL, max77759_aicl_active_cb, chip);
	if (IS_ERR_OR_NULL(chip->aicl_active_el)) {
		ret = PTR_ERR(chip->aicl_active_el);
		dev_err(chip->dev, "Unable to create aicl_active_el(%d)\n", ret);
		goto unreg_notifier;
	}
	gvotable_set_vote2str(chip->aicl_active_el, gvotable_v2s_int);

	chip->tcpci = google_tcpci_shim_register_port(chip->dev, &chip->data);
	if (IS_ERR_OR_NULL(chip->tcpci)) {
		dev_err(&client->dev, "TCPCI port registration failed");
		ret = PTR_ERR(chip->tcpci);
		goto unreg_aicl_el;
	}
	chip->port = google_tcpci_shim_get_tcpm_port(chip->tcpci);

	max77759_enable_voltage_alarm(chip, true, true);

	if (!of_property_read_u32(dn, "ext-bst-ovp-clear-mv", &chip->ext_bst_ovp_clear_mv))
		LOG(LOG_LVL_DEBUG, chip->log, "ext_bst_ovp_clear_mv set to %u",
		    chip->ext_bst_ovp_clear_mv);

	ret = max77759_init_alert(chip, client);
	if (ret < 0)
		goto unreg_port;

	device_init_wakeup(chip->dev, true);

	for (i = 0; max77759_device_attrs[i]; i++) {
		ret = device_create_file(&client->dev, max77759_device_attrs[i]);
		if (ret < 0)
			dev_err(&client->dev, "TCPCI: Unable to create device attr[%d] ret:%d:", i,
				ret);
	}

	if (!modparam_conf_sbu) {
		ret = max77759_write8(chip->data.regmap, TCPC_VENDOR_SBUSW_CTRL, 0);
		LOG(LOG_LVL_DEBUG, chip->log,
		    "SBU switch disable %s", ret < 0 ? "fail" : "success");
	}

#ifdef CONFIG_DEBUG_FS
	chip->dentry = debugfs_create_dir("tcpci_max77759", NULL);
	if (IS_ERR(chip->dentry)) {
		dev_err(&client->dev, "TCPCI: debugfs dentry failed: %ld", PTR_ERR(chip->dentry));
	} else {
		debugfs_create_file("force_device_mode_on", 0644, chip->dentry, chip,
				    &force_device_mode_on_fops);
	}
#endif

#ifdef CONFIG_GPIOLIB
	ret = ext_bst_en_gpio_init(chip);
	if (ret)
		goto remove_files;
#endif
	return 0;

remove_files:
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(chip->dentry);
#endif
	for (i = 0; max77759_device_attrs[i]; i++)
		device_remove_file(&client->dev, max77759_device_attrs[i]);
unreg_port:
	google_tcpci_shim_unregister_port(chip->tcpci);
unreg_aicl_el:
	gvotable_destroy_election(chip->aicl_active_el);
unreg_notifier:
	power_supply_unreg_notifier(&chip->psy_notifier);
destroy_usb_bcl_worker:
        if (!IS_ERR_OR_NULL(chip->bcl_usb_wq))
		kthread_destroy_worker(chip->bcl_usb_wq);
destroy_dp_worker:
	kthread_destroy_worker(chip->dp_notification_wq);
destroy_worker:
	kthread_destroy_worker(chip->wq);
teardown_data:
	max77759_teardown_data_notifier(chip);
dp_regulator_put:
	devm_regulator_put(chip->dp_regulator);
psy_put:
	power_supply_put(chip->usb_psy);
teardown_bc12:
	bc12_teardown(chip->bc12);
unreg_psy:
	usb_psy_teardown(chip->usb_psy_data);
logbuffer_unreg:
	logbuffer_unregister(chip->log);

	return ret;
}

static void max77759_remove(struct i2c_client *client)
{
	struct max77759_plat *chip = i2c_get_clientdata(client);
	int i;

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(chip->dentry);
#endif
	for (i = 0; max77759_device_attrs[i]; i++)
		device_remove_file(&client->dev, max77759_device_attrs[i]);
	if (!IS_ERR_OR_NULL(chip->tcpci))
		google_tcpci_shim_unregister_port(chip->tcpci);
	if (!IS_ERR_OR_NULL(chip->dp_regulator))
		devm_regulator_put(chip->dp_regulator);
	if (!IS_ERR_OR_NULL(chip->aicl_active_el))
		gvotable_destroy_election(chip->aicl_active_el);
	if (!IS_ERR_OR_NULL(chip->usb_psy))
		power_supply_put(chip->usb_psy);
	if (!IS_ERR_OR_NULL(chip->usb_psy_data))
		usb_psy_teardown(chip->usb_psy_data);
	if (!IS_ERR_OR_NULL(chip->bc12))
		bc12_teardown(chip->bc12);
	if (!IS_ERR_OR_NULL(chip->log))
		logbuffer_unregister(chip->log);
	if (!IS_ERR_OR_NULL(chip->dp_notification_wq))
		kthread_destroy_worker(chip->dp_notification_wq);
	if (!IS_ERR_OR_NULL(chip->wq))
		kthread_destroy_worker(chip->wq);
	if (!IS_ERR_OR_NULL(chip->bcl_usb_wq))
		kthread_destroy_worker(chip->bcl_usb_wq);
	power_supply_unreg_notifier(&chip->psy_notifier);
	max77759_teardown_data_notifier(chip);
}

static void max77759_shutdown(struct i2c_client *client)
{
	struct max77759_plat *chip = i2c_get_clientdata(client);
	int ret;

	dev_info(&client->dev, "disabling Type-C upon shutdown\n");
	kthread_cancel_delayed_work_sync(&chip->check_missing_rp_work);
	kthread_cancel_delayed_work_sync(&chip->icl_work);
	if (!IS_ERR_OR_NULL(chip->bcl_usb_wq))
		kthread_cancel_delayed_work_sync(&chip->bcl_usb_votable_work);
	/* Set current limit to 0. Will eventually happen after hi-Z as well */
	max77759_vote_icl(chip, 0);
	power_supply_unreg_notifier(&chip->psy_notifier);
	/* Prevent re-enabling toggling */
	/* Hi-z CC pins to trigger disconnection */
	ret = gvotable_cast_vote(chip->toggle_disable_votable, "SHUTDOWN_VOTE",
				 (void *)MAX77759_DISABLE_TOGGLE_VOTE, MAX77759_DISABLE_TOGGLE);
	if (ret < 0)
		dev_err(chip->dev, "Cannot set TOGGLE DISABLE (%d)\n", ret);
}

static const struct i2c_device_id max77759_id[] = {
	{ "max77759tcpc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77759_id);

#ifdef CONFIG_OF
static const struct of_device_id max77759_of_match[] = {
	{ .compatible = "max77759tcpc", },
	{},
};
MODULE_DEVICE_TABLE(of, max77759_of_match);
#endif

static struct i2c_driver max77759_i2c_driver = {
	.driver = {
		.name = "max77759tcpc",
		.of_match_table = of_match_ptr(max77759_of_match),
	},
	.probe = max77759_probe,
	.remove = max77759_remove,
	.id_table = max77759_id,
	.shutdown = max77759_shutdown,
};

static int __init max77759_i2c_driver_init(void)
{
	tcpm_log = logbuffer_register("tcpm");
	if (IS_ERR_OR_NULL(tcpm_log))
		return -EAGAIN;

	return i2c_add_driver(&max77759_i2c_driver);
}
module_init(max77759_i2c_driver_init);

static void __exit max77759_i2c_driver_exit(void)
{
	i2c_del_driver(&max77759_i2c_driver);
}
module_exit(max77759_i2c_driver_exit);

MODULE_AUTHOR("Badhri Jagan Sridharan <badhri@google.com>");
MODULE_DESCRIPTION("MAX77759 USB Type-C Port Controller Interface Driver");
MODULE_LICENSE("GPL");
