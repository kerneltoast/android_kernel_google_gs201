/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Google Corp.
 *
 * Author:
 *  Howard.Yen <howardyen@google.com>
 */

#ifndef __LINUX_AOC_USB_H
#define __LINUX_AOC_USB_H

#include <linux/notifier.h>

#include "xhci.h"

/*
 * This variable used to present if aoc_usb module was probed done. If offload
 * is enabled, the controller needs to wait for the aoc_usb probe done and then
 * continue the controller's probe.
 */
extern bool aoc_usb_probe_done;

enum aoc_usb_msg {
	SYNC_DEVICE_CONTEXT,
	GET_DCBAA_PTR,
	SET_DCBAA_PTR,
	GET_TR_DEQUEUE_PTR,
	SETUP_DONE,
	GET_ISOC_TR_INFO,
	SET_ISOC_TR_INFO,
	SYNC_CONN_STAT,
	SET_OFFLOAD_STATE,
	SEND_FB_EP_INFO
};

enum aoc_usb_state {
	USB_DISCONNECTED,
	USB_CONNECTED
};

enum usb_offload_op_mode {
	USB_OFFLOAD_STOP,
	USB_OFFLOAD_SIMPLE_AUDIO_ACCESSORY,
	USB_OFFLOAD_DRAM
};

enum usb_recover_state {
	NONE,
	RECOVER_HOST_OFF,
	RECOVER_HOST_ON,
	RECOVERED
};

enum usb_dev_hub {
	UNDEFINED,
	USB1,
	USB1_1,
	USB2,
	USB2_1,
	USB3,
	USB3_1
};

struct xhci_vendor_data {
	struct xhci_hcd *xhci;

	bool usb_accessory_enabled;
	bool usb_audio_offload;
	bool dt_direct_usb_access;
	bool offload_state;

	/* count how many usb audio devices are connected */
	int usb_audio_count;

	enum usb_offload_op_mode op_mode;

	struct workqueue_struct *irq_wq;
	struct work_struct xhci_vendor_irq_work;
};

struct aoc_usb_drvdata {
	struct aoc_service_dev *adev;

	struct mutex lock;
	struct wakeup_source *ws;

	struct notifier_block nb;

	long service_timeout;
	unsigned int usb_conn_state;
};

struct get_dev_ctx_args {
	unsigned int slot_id;
	size_t length;
	u8 *dev_ctx;
};

struct conn_stat_args {
	u16 bus_id;
	u16 dev_num;
	u16 slot_id;
	u32 conn_stat;
};

struct get_isoc_tr_info_args {
	u16 ep_id;
	u16 dir;
	u32 type;
	u32 num_segs;
	u32 seg_ptr;
	u32 max_packet;
	u32 deq_ptr;
	u32 enq_ptr;
	u32 cycle_state;
	u32 num_trbs_free;
};

struct feedback_ep_info_args {
	bool enabled;
	u16 bus_id;
	u16 dev_num;
	u16 slot_id;
	u16 ep_num;
	u32 max_packet;
	u16 binterval;
	u16 brefresh;
};

int xhci_vendor_helper_init(void);
int usb_vendor_helper_init(void);
int snd_usb_audio_vendor_helper_init(void);

extern int xhci_handle_event(struct xhci_hcd *xhci);
extern void xhci_update_erst_dequeue(struct xhci_hcd *xhci,
				     union xhci_trb *event_ring_deq);
extern int xhci_exynos_register_vendor_ops(struct xhci_vendor_ops *vendor_ops);
int xhci_send_feedback_ep_info(struct xhci_hcd *xhci, struct feedback_ep_info_args *cmd_args);
int xhci_get_usb_audio_count(struct xhci_hcd *xhci);
int xhci_set_offload_state(struct xhci_hcd *xhci, bool enabled);
struct xhci_hcd *get_xhci_hcd_by_udev(struct usb_device *udev);

int usb_host_mode_state_notify(enum aoc_usb_state usb_state);

int register_aoc_usb_notifier(struct notifier_block *nb);
int unregister_aoc_usb_notifier(struct notifier_block *nb);

extern int dwc3_otg_host_enable(bool enabled);

extern bool aoc_alsa_usb_capture_enabled(void);
extern bool aoc_alsa_usb_playback_enabled(void);

#endif /* __LINUX_AOC_USB_H */
