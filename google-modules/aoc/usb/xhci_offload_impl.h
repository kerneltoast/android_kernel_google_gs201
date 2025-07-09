/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020 Google Corp.
 *
 * Author:
 *  Howard.Yen <howardyen@google.com>
 */

#ifndef __XHCI_OFFLOAD_IMPL_H
#define __XHCI_OFFLOAD_IMPL_H

struct xhci_offload_data {
	struct xhci_hcd *xhci;

	bool usb_audio_offload;
	bool dt_direct_usb_access;
	bool offload_state;

	/* count how many usb audio devices are connected */
	int usb_audio_count;

	struct work_struct offload_connect_ws;
};

struct xhci_offload_data *xhci_get_offload_data(void);
int xhci_offload_helper_init(void);

extern int xhci_exynos_register_offload_ops(struct xhci_exynos_ops *offload_ops);

#endif /* __XHCI_OFFLOAD_IMPL_H */
