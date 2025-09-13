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

#include "usbaudio.h"
#include "xhci.h"

enum aoc_usb_msg {
	SET_DCBAA_PTR,
	GET_TR_DEQUEUE_PTR,
	SETUP_DONE,
	SET_ISOC_TR_INFO,
	SYNC_CONN_STAT,
	SET_OFFLOAD_STATE
};

enum aoc_usb_state {
	USB_DISCONNECTED,
	USB_CONNECTED
};


struct aoc_usb_drvdata {
	struct aoc_service_dev *adev;

	struct mutex lock;
	struct wakeup_source *ws;

	struct notifier_block nb;

	long service_timeout;
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

int register_aoc_usb_notifier(struct notifier_block *nb);
int unregister_aoc_usb_notifier(struct notifier_block *nb);

extern bool aoc_alsa_usb_callback_register(void (*callback)(struct usb_device*,
							    struct usb_host_endpoint*));
extern bool aoc_alsa_usb_callback_unregister(void);

int notify_offload_state(bool enabled);
int xhci_set_dcbaa_ptr(u64 aoc_dcbaa_ptr);
int xhci_setup_done(void);
int xhci_sync_conn_stat(unsigned int bus_id, unsigned int dev_num, unsigned int slot_id,
			       unsigned int conn_stat);
int usb_host_mode_state_notify(enum aoc_usb_state usb_state);
int xhci_set_isoc_tr_info(u16 ep_id, u16 dir, struct xhci_ring *ep_ring);
int xhci_get_usb_audio_count(void);

int xhci_offload_helper_init(void);
int usb_vendor_helper_init(void);

extern int dwc3_otg_host_ready(bool ready);
extern bool aoc_alsa_usb_capture_enabled(void);
extern bool aoc_alsa_usb_playback_enabled(void);

#endif /* __LINUX_AOC_USB_H */
