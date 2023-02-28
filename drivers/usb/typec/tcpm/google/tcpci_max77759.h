// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 Google LLC
 *
 */

#ifndef __TCPCI_MAX77759_H
#define __TCPCI_MAX77759_H

#include <linux/alarmtimer.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/usb/tcpm.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/usb/role.h>
#include <linux/usb/typec_mux.h>

#include "usb_psy.h"

struct gvotable_election;
struct logbuffer;
struct max77759_contaminant;
struct max77759_compliance_warnings;
struct tcpci_data;
struct max77759_io_error;

struct max77759_plat {
	struct tcpci_data data;
	struct tcpci *tcpci;
	struct device *dev;
	struct bc12_status *bc12;
	struct i2c_client *client;
	struct power_supply *usb_psy;
	struct max77759_contaminant *contaminant;
	struct gvotable_election *usb_icl_proto_el;
	struct gvotable_election *usb_icl_el;
	struct gvotable_election *charger_mode_votable;
	bool vbus_enabled;
	/* Data role notified to the data stack */
	enum typec_data_role active_data_role;
	/* Data role from the TCPM stack */
	enum typec_data_role data_role;
	/* CC polarity from the TCPM stack */
	enum typec_cc_polarity polarity;
	/* protects tcpc_enable_data_path */
	struct mutex data_path_lock;
	/* Vote for data from BC1.2 */
	bool bc12_data_capable;
	/* Infered from pd caps */
	bool pd_data_capable;
	/* Vote from TCPC for attached */
	bool attached;
	/* Reflects the signal sent out to the data stack */
	bool data_active;
	/* Alernate data path */
	bool alt_path_active;
	/* Reflects whether the current partner can do PD */
	bool pd_capable;
	void *usb_psy_data;
	struct mutex icl_proto_el_lock;
	/* Set vbus voltage alarms */
	bool set_voltage_alarm;
	unsigned int vbus_mv;
	/* USB Data notification */
	struct extcon_dev *extcon;
	bool no_bc_12;
	/* Platform does not support external boost */
	bool no_external_boost;
	struct tcpm_port *port;
	struct usb_psy_ops psy_ops;
	/* toggle in_switch to kick debug accessory statemachine when already connected */
	int in_switch_gpio;
	/* 0:active_low 1:active_high */
	bool in_switch_gpio_active_high;
	bool first_toggle;
	/* Set true to vote "limit_sink_current" on USB ICL */
	bool limit_sink_enable;
	/* uA */
	unsigned int limit_sink_current;
	/* Indicate that the Vbus OVP is restricted to quick ramp-up time for incoming voltage. */
	bool quick_ramp_vbus_ovp;
	int reset_ovp_retry;
	/* Set true to vote "limit_accessory_current" on USB ICL */
	bool limit_accessory_enable;
	/* uA */
	unsigned int limit_accessory_current;

	/* True when TCPC is in SINK DEBUG ACCESSORY CONNECTED state */
	u8 debug_acc_connected:1;
	/* Cache status when sourcing vbus. Used to modify vbus_present status */
	u8 sourcing_vbus:1;
	/* Cache vbus_present as MAX77759 reports vbus present = 0 when vbus < 4V */
	u8 vbus_present:1;
	u8 cc1;
	u8 cc2;

	/* Runtime flags */
	int frs;
	bool in_frs;
	bool vsafe0v;

	/*
	 * Current status of contaminant detection.
	 * 0 - Disabled
	 * 1 - AP
	 * 2 - MAXQ
	 */
	int contaminant_detection;
	/* Userspace status */
	bool contaminant_detection_userspace;
	/* Consecutive floating cable instances */
	unsigned int floating_cable_or_sink_detected;
	/* Timer to re-enable auto ultra lower mode for contaminant detection */
	struct alarm reenable_auto_ultra_low_power_mode_alarm;
	/* Bottom half for alarm */
	struct kthread_work reenable_auto_ultra_low_power_mode_work;

	/* Protects contaminant_detection variable and role_control */
	struct mutex rc_lock;

	/* Accumulate votes to disable toggling as needed */
	struct gvotable_election *toggle_disable_votable;
	/* Current status of toggle */
	bool toggle_disable_status;
	/* Cached role ctrl setting */
	u8 role_ctrl_cache;

	struct notifier_block psy_notifier;
	int online;
	int usb_type;
	int typec_current_max;
	struct kthread_worker *wq;
	struct kthread_delayed_work icl_work;
	struct kthread_delayed_work enable_vbus_work;
	struct kthread_delayed_work vsafe0v_work;
	struct kthread_delayed_work reset_ovp_work;
	struct kthread_delayed_work check_missing_rp_work;

	/* Notifier for data role */
	struct usb_role_switch *usb_sw;
	/* Notifier for orientation */
	struct typec_switch *typec_sw;

	/* Reflects whether BC1.2 is still running */
	bool bc12_running;

	/* To handle io error - Last cached IRQ status*/
	u16 irq_status;
	struct kthread_delayed_work max77759_io_error_work;
	/* Hold before calling _max77759_irq */
	struct mutex irq_status_lock;

	/* non compliant reasons */
	struct max77759_compliance_warnings *compliance_warnings;

	/*
	 * When set missing Rp detection has a longer delay to overcome
	 * additional delay during boot.
	 */
	bool first_rp_missing_timeout;

	/* EXT_BST_EN exposed as GPIO */
#ifdef CONFIG_GPIOLIB
	struct gpio_chip gpio;
#endif

	struct logbuffer *log;

	u8 force_device_mode_on:1;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
#endif
};

/*
 * bc1.2 registration
 */

struct max77759_usb;

void register_tcpc(struct max77759_usb *usb, struct max77759_plat *chip);

#define MAXQ_DETECT_TYPE_CC_AND_SBU	0x10
#define MAXQ_DETECT_TYPE_SBU_ONLY	0x30

int maxq_query_contaminant(u8 cc1_raw, u8 cc2_raw, u8 sbu1_raw, u8 sbu2_raw,
			   u8 cc1_rd, u8 cc2_rd, u8 type, u8 cc_adc_skipped,
			   u8 *response, u8 length);
int __attribute__((weak)) maxq_query_contaminant(u8 cc1_raw, u8 cc2_raw, u8 sbu1_raw, u8 sbu2_raw,
						 u8 cc1_rd, u8 cc2_rd, u8 type, u8 cc_adc_skipped,
						 u8 *response, u8 length)
{
	return -EINVAL;
}

struct max77759_contaminant *max77759_contaminant_init(struct max77759_plat *plat, bool enable);
int process_contaminant_alert(struct max77759_contaminant *contaminant, bool debounce_path,
			      bool tcpm_toggling, bool *cc_status_handled);
int enable_contaminant_detection(struct max77759_plat *chip, bool maxq);
int disable_contaminant_detection(struct max77759_plat *chip);
bool is_contaminant_detected(struct max77759_plat *chip);
bool is_floating_cable_or_sink_detected(struct max77759_plat *chip);
void disable_auto_ultra_low_power_mode(struct max77759_plat *chip, bool disable);

#define VBUS_VOLTAGE_MASK		0x3ff
#define VBUS_VOLTAGE_LSB_MV		25
#define VBUS_HI_HEADROOM_MV		500
#define VBUS_LO_MV			4500

enum tcpm_psy_online_states {
	TCPM_PSY_OFFLINE = 0,
	TCPM_PSY_FIXED_ONLINE,
	TCPM_PSY_PROG_ONLINE,
};

void enable_data_path_locked(struct max77759_plat *chip);
void data_alt_path_active(struct max77759_plat *chip, bool active);
void register_data_active_callback(void (*callback)(void *data_active_payload), void *data);
void register_orientation_callback(void (*callback)(void *orientation_payload), void *data);

#define COMPLIANCE_WARNING_OTHER 0
#define COMPLIANCE_WARNING_DEBUG_ACCESSORY 1
#define COMPLIANCE_WARNING_BC12 2
#define COMPLIANCE_WARNING_MISSING_RP 3

struct max77759_compliance_warnings {
	struct max77759_plat *chip;
	bool other;
	bool debug_accessory;
	bool bc12;
	bool missing_rp;
};

ssize_t compliance_warnings_to_buffer(struct max77759_compliance_warnings *compliance_warnings,
				      char *buf);
void update_compliance_warnings(struct max77759_plat *chip, int warning, bool value);

#endif /* __TCPCI_MAX77759_H */
