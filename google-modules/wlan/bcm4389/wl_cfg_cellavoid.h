/*
 * Cellular channel avoidance implementation
 *
 * Copyright (C) 2025, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Dual:>>
 */

#ifndef _wl_cfg_cellavoid_h_
#define _wl_cfg_cellavoid_h_

#include <linux/netdevice.h>
#include <linux/nl80211.h>

#define MAX_AP_INTERFACE	2
#define MAX_STA_INTERFACE	2

extern int wl_cellavoid_init(struct bcm_cfg80211 *cfg);
extern void wl_cellavoid_deinit(struct bcm_cfg80211 *cfg);
extern int wl_cellavoid_reinit(struct bcm_cfg80211 *cfg);
extern void wl_cellavoid_sync_lock(struct bcm_cfg80211 *cfg);
extern void wl_cellavoid_sync_unlock(struct bcm_cfg80211 *cfg);
extern int wl_cfgvendor_cellavoid_set_cell_channels(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void  *data, int len);
extern int wl_cellavoid_set_requested_freq_bands(struct net_device *ndev,
	void *cai, u32 *pElem_freq, u32 freq_list_len);
extern void wl_cellavoid_clear_requested_freq_bands(struct net_device *ndev,
	void *cai);
extern bool wl_cellavoid_operation_allowed(void *cai,
	chanspec_t chanspec, enum nl80211_iftype type);
extern void wl_cellavoid_free_csa_info(void *cai,
	struct net_device *ndev);
extern chanspec_t wl_cellavoid_find_chspec_fromband(void *cai, int band);
extern chanspec_t wl_cellavoid_find_widechspec_fromchspec(void *cai, chanspec_t chanspec);
extern void wl_cellavoid_set_csa_done(void *cai);
extern bool wl_cellavoid_mandatory_isset(void *cai, enum nl80211_iftype type);
extern bool wl_cellavoid_is_safe(void *cai, chanspec_t chanspec);
extern bool wl_cellavoid_is_safe_overlap(void *cai, chanspec_t chanspec);
extern wifi_interface_mode wl_cellavoid_mandatory_to_usable_channel_filter(void *cai);
#ifdef WL_CELLULAR_CHAN_AVOID_DUMP
extern void wl_cellavoid_sanity_check_chan_info_list(void *cai);
#endif /* WL_CELLULAR_CHAN_AVOID_DUMP */
#endif /* _wl_cfg_cellavoid_h_ */
