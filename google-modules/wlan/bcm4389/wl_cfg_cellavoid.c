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

#include <typedefs.h>
#include <linuxver.h>
#include <linux/kernel.h>

#include <bcmutils.h>
#include <bcmstdlib_s.h>
#include <bcmwifi_channels.h>
#include <bcmendian.h>
#include <ethernet.h>
#ifdef WL_WPS_SYNC
#include <eapol.h>
#endif /* WL_WPS_SYNC */
#include <802.11.h>
#include <bcmiov.h>
#include <linux/if_arp.h>
#include <asm/uaccess.h>

#include <ethernet.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <linux/ieee80211.h>
#include <linux/wait.h>
#include <net/cfg80211.h>
#include <net/rtnetlink.h>

#include <wlioctl.h>
#include <bcmevent.h>
#include <wldev_common.h>
#include <wl_cfg80211.h>
#include <wl_cfgp2p.h>
#include <wl_cfgscan.h>
#include <wl_cfgvif.h>
#include <bcmdevs.h>
#include <bcmdevs_legacy.h>
#include <linux/list_sort.h>
#include <wl_cfgvendor.h>
#include <wl_cfg_cellavoid.h>

#define INVALID_CHSPEC_BW	(0xFFFF)

#define CELLAVOID_NO_POWER_CAP (0x7FFFFFFF)
#define CELLAVOID_TXCAP_MIN_VAL -32	/* dBm */
#define CELLAVOID_TXCAP_MAX_VAL 31	/* dBm */
#define CELLAVOID_MAX_CH 128u
#define WL_CELLAVOID_INFORM(args) WL_ERR(args)

#define CELLAVOID_CSA_CNT		50u

#define CSA_DELAYWORK_CSA_INTERVAL	(CELLAVOID_CSA_CNT * 100u)
#define CSA_DELAYWORK_FAIL_INTERVAL	1000u
#define CSA_DELAYWORK_BUSY_INTERVAL	200u
#define CSA_DELAYWORK_FIRST_INTERVAL	0u

#define CSA_MAX_RETRY_CNT		20u

typedef enum cellavoid_ch_state {
	CELLAVOID_STATE_CH_UNSAFE = 0,
	CELLAVOID_STATE_CH_SAFE = 1
} cellavoid_ch_state_t;

typedef struct wl_cellavoid_chan_param {
	int band;
	int center_channel;
	int8 pwr_cap;
	chanspec_bw_t chspec_bw;
	chanspec_band_t chspec_band;
} wl_cellavoid_chan_param_t;

/* struct for channel info list
 * allocated on the country code change and
 * moved to the safe list (availale channel list)
 * If framework gives unsafe channel list,
 * then corresponding channel infos go to the unsafe list(cellular chan list)
 */
typedef struct wl_cellavoid_chan_info {
	struct list_head list;
	chanspec_t chanspec;
	int8 pwr_cap;
} wl_cellavoid_chan_info_t;

typedef struct wl_cellavoid_csa_info {
	struct list_head list;
	struct net_device *ndev;
	chanspec_t chanspec;
} wl_cellavoid_csa_info_t;

typedef struct wl_cellavoid_req_band {
	struct net_device *ndev;
	uint32 req_band;
} wl_cellavoid_req_band_t;

/* This struct is used in cfg vendor function setting function */
typedef struct wl_cellavoid_param {
	wl_cellavoid_chan_param_t *chan_param;
	u8 chan_cnt;
	u32 mandatory;
} wl_cellavoid_param_t;

typedef struct wl_cellavoid_info {
	osl_t *osh;
	struct mutex sync;
	wl_cellavoid_param_t params;
	u32 mandatory_flag;
	u16 cell_chan_info_cnt;
	struct list_head cell_chan_info_list;
	struct list_head avail_chan_info_list;
	wl_cellavoid_req_band_t req_band[MAX_AP_INTERFACE];
	bool csa_progress;
	u32 csa_info_cnt;
	struct list_head csa_info_list;
	u32 csa_reschedule_cnt;
} wl_cellavoid_info_t;

static int wl_cellavoid_verify_avail_chan_list(struct bcm_cfg80211 *cfg,
	wl_cellavoid_info_t *cellavoid_info);
static void wl_cellavoid_clear_cell_chan_list(wl_cellavoid_info_t *cellavoid_info);
static void wl_cellavoid_free_avail_chan_list(wl_cellavoid_info_t *cellavoid_info);
static int wl_cellavoid_alloc_avail_chan_list(struct wiphy *wiphy,
	wl_cellavoid_info_t *cellavoid_info);
static int wl_cellavoid_restore_txpwrcap(struct bcm_cfg80211 *cfg,
	wl_cellavoid_info_t *cellavoid_info);
static void wl_cellavoid_do_csa_work(struct work_struct *work);
static void wl_cellavoid_free_csa_info_list(wl_cellavoid_info_t *cellavoid_info);
static int wl_cellavoid_set_cell_channels(struct bcm_cfg80211 *cfg, wl_cellavoid_param_t *param);

/* Initialize context */
int
wl_cellavoid_init(struct bcm_cfg80211 *cfg)
{
	wl_cellavoid_info_t *cellavoid_info;
	int ret = BCME_OK;

	WL_MEM(("%s: Enter\n", __FUNCTION__));
	cellavoid_info = (wl_cellavoid_info_t *)
		MALLOCZ(cfg->osh, sizeof(*cellavoid_info));
	if (cellavoid_info == NULL) {
		WL_ERR(("failed to create cellavoid_info\n"));
		return BCME_NOMEM;
	}

	cellavoid_info->params.chan_param =
		(wl_cellavoid_chan_param_t *)MALLOCZ(cfg->osh,
		CELLAVOID_MAX_CH * sizeof(*(cellavoid_info->params.chan_param)));
	if (cellavoid_info->params.chan_param == NULL) {
		MFREE(cfg->osh, cellavoid_info, sizeof(*cellavoid_info));
		WL_ERR(("failed to create cellavoid_info params\n"));
		return BCME_NOMEM;
	}

	INIT_LIST_HEAD(&cellavoid_info->cell_chan_info_list);
	INIT_LIST_HEAD(&cellavoid_info->avail_chan_info_list);
	INIT_LIST_HEAD(&cellavoid_info->csa_info_list);

	INIT_DELAYED_WORK(&cfg->csa_delayed_work, wl_cellavoid_do_csa_work);
	mutex_init(&cellavoid_info->sync);

	cellavoid_info->osh = cfg->osh;
	cfg->cellavoid_info = cellavoid_info;

	return ret;
}

/* deinitialize context */
void
wl_cellavoid_deinit(struct bcm_cfg80211 *cfg)
{
	wl_cellavoid_info_t *cellavoid_info = cfg->cellavoid_info;

	WL_MEM(("%s: Enter\n", __FUNCTION__));
	if (!cellavoid_info) {
		return;
	}

	cancel_delayed_work(&cfg->csa_delayed_work);

	mutex_lock(&cellavoid_info->sync);
	wl_cellavoid_free_csa_info_list(cellavoid_info);
	wl_cellavoid_clear_cell_chan_list(cellavoid_info);
	wl_cellavoid_free_avail_chan_list(cellavoid_info);
	MFREE(cfg->osh, cellavoid_info->params.chan_param,
		CELLAVOID_MAX_CH * sizeof(*(cellavoid_info->params.chan_param)));
	mutex_unlock(&cellavoid_info->sync);
	MFREE(cfg->osh, cellavoid_info, sizeof(*cellavoid_info));

	cellavoid_info = NULL;
}

void
wl_cellavoid_sync_lock(struct bcm_cfg80211 *cfg)
{

	wl_cellavoid_info_t *cellavoid_info = cfg->cellavoid_info;

	if (!cellavoid_info) {
		return;
	}

	mutex_lock(&cellavoid_info->sync);
}

void
wl_cellavoid_sync_unlock(struct bcm_cfg80211 *cfg)
{
	wl_cellavoid_info_t *cellavoid_info = cfg->cellavoid_info;

	if (!cellavoid_info) {
		return;
	}

	mutex_unlock(&cellavoid_info->sync);
}

/* Called in wl_update_wiphybands function
 * 1) If there's item in unsafe channel list (cellular channel list), then
 * move it to safe channel list(available channel list)
 * 2) Then, free all the channel info items
 * 3) Trigger IOVAR to remove txpwrcap in FW (wl_cellavoid_restore_txpwrcap)
 * 4) Allocate new channel items for the new country code
 * 5) Recreate cellular channel list from saved params
 */
int
wl_cellavoid_reinit(struct bcm_cfg80211 *cfg)
{
	wl_cellavoid_info_t *cellavoid_info = cfg->cellavoid_info;
	int ret = BCME_ERROR;

	WL_MEM(("%s: Enter\n", __FUNCTION__));
	if (!cellavoid_info) {
		return ret;
	}

	cancel_delayed_work_sync(&cfg->csa_delayed_work);

	mutex_lock(&cellavoid_info->sync);

	wl_cellavoid_free_csa_info_list(cellavoid_info);
	wl_cellavoid_clear_cell_chan_list(cellavoid_info);
	wl_cellavoid_free_avail_chan_list(cellavoid_info);

	ret = wl_cellavoid_restore_txpwrcap(cfg, cellavoid_info);
	if (ret != BCME_OK) {
		goto fail;
	}
	ret = wl_cellavoid_alloc_avail_chan_list(bcmcfg_to_wiphy(cfg), cellavoid_info);
	if (ret != BCME_OK) {
		goto fail;
	}

	/* Recreate cellular channel list from saved params */
	ret = wl_cellavoid_set_cell_channels(cfg, &cellavoid_info->params);
	if (ret != BCME_OK) {
		goto fail;
	}

	mutex_unlock(&cellavoid_info->sync);
	return ret;

fail:
	wl_cellavoid_free_avail_chan_list(cellavoid_info);

	mutex_unlock(&cellavoid_info->sync);
	return ret;
}

static void
wl_cellavoid_free_csa_info_list(wl_cellavoid_info_t *cellavoid_info)
{
	wl_cellavoid_csa_info_t *csa_info, *next;

	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(csa_info, next, &cellavoid_info->csa_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		list_del(&csa_info->list);
		MFREE(cellavoid_info->osh, csa_info, sizeof(*csa_info));
	}

	cellavoid_info->csa_info_cnt = 0;
	cellavoid_info->csa_progress = FALSE;

}

void
wl_cellavoid_free_csa_info(void *cai, struct net_device *ndev)
{
	wl_cellavoid_csa_info_t *csa_info, *next;
	wl_cellavoid_info_t *cellavoid_info = cai;

	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(csa_info, next, &cellavoid_info->csa_info_list, list) {
		GCC_DIAGNOSTIC_POP();
			if (csa_info->ndev == ndev) {
				list_del(&csa_info->list);
				cellavoid_info->csa_info_cnt--;
				if (cellavoid_info->csa_info_cnt == 0) {
					cellavoid_info->csa_progress = FALSE;
					cellavoid_info->csa_reschedule_cnt = 0;
				}
				MFREE(cellavoid_info->osh, csa_info, sizeof(*csa_info));
				break;
		}
	}
}

static int
wl_cellavoid_add_csa_info(wl_cellavoid_info_t *cellavoid_info,
	struct net_device *ndev, chanspec_t chanspec)
{
	wl_cellavoid_csa_info_t *csa_info = NULL;

	/* Allocate csa info */
	csa_info = (wl_cellavoid_csa_info_t *)
		MALLOCZ(cellavoid_info->osh, sizeof(wl_cellavoid_csa_info_t));
	if (!csa_info) {
		WL_ERR(("failed to allocate chan info\n"));
		return -ENOMEM;
	}

	csa_info->ndev = ndev;
	csa_info->chanspec = chanspec;

	list_add_tail(&csa_info->list, &cellavoid_info->csa_info_list);
	cellavoid_info->csa_info_cnt++;

	return BCME_OK;
}

void
wl_cellavoid_set_csa_done(void *cai)
{
	wl_cellavoid_info_t *cellavoid_info = cai;

	if (cellavoid_info == NULL) {
		return;
	}

	mutex_lock(&cellavoid_info->sync);

	if (list_empty(&cellavoid_info->csa_info_list)) {
		cellavoid_info->csa_progress = FALSE;
	}

	mutex_unlock(&cellavoid_info->sync);
}

static void
wl_cellavoid_do_csa_work(struct work_struct *work)
{
	struct bcm_cfg80211 *cfg;
	wl_cellavoid_info_t *cellavoid_info;
	wl_cellavoid_csa_info_t *csa_info = NULL;
	struct delayed_work *dw = to_delayed_work(work);
	wl_chan_switch_t csa_arg;
	uint delay = CSA_DELAYWORK_BUSY_INTERVAL;
	char smbuf[WLC_IOCTL_SMLEN];
	struct net_info *iter, *next;
	bool found = FALSE;
	int err;

	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	cfg = container_of(dw, struct bcm_cfg80211, csa_delayed_work);
	GCC_DIAGNOSTIC_POP();

	cellavoid_info = cfg->cellavoid_info;

	mutex_lock(&cellavoid_info->sync);

	if (!list_empty(&cellavoid_info->csa_info_list)) {
		csa_info = list_entry(cellavoid_info->csa_info_list.next,
			wl_cellavoid_csa_info_t, list);

		/* Need to check ndev is still valid */
		GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
		for_each_ndev(cfg, iter, next) {
			GCC_DIAGNOSTIC_POP();
			if ((iter->ndev) &&
				(iter->ndev->ieee80211_ptr->iftype == NL80211_IFTYPE_AP) &&
				wl_get_drv_status(cfg, CONNECTED, iter->ndev) &&
				iter->ndev == csa_info->ndev) {
				found = TRUE;
			}
		}

		if (found == FALSE) {
			wl_cellavoid_free_csa_info(cellavoid_info, csa_info->ndev);
			delay = CSA_DELAYWORK_BUSY_INTERVAL;
			goto exit;
		}

		if (wl_get_drv_status_all(cfg, SCANNING) ||
			wl_get_drv_status_all(cfg, CONNECTING)) {
			WL_INFORM_MEM(("scanning/connecting, "
				"so delay for a while, target chspec %x\n",
				csa_info->chanspec));
			goto reschedule;
		}

		bzero(&csa_arg, sizeof(csa_arg));
		csa_arg.mode = DOT11_CSA_MODE_ADVISORY;
		csa_arg.count = CELLAVOID_CSA_CNT;
		csa_arg.reg = 0;
		csa_arg.chspec = wl_chspec_host_to_driver(csa_info->chanspec);

		/* TBD, ndev valid check, limit on reschedule count */
		WL_INFORM_MEM(("CSA, target chspec %x\n", csa_info->chanspec));

		err = wldev_iovar_setbuf(csa_info->ndev, "csa", &csa_arg, sizeof(csa_arg),
			smbuf, sizeof(smbuf), NULL);
		if (err == BCME_BUSY) {
			WL_INFORM_MEM(("device is busy, so delay for a while, target chspec %x\n",
				csa_info->chanspec));
			goto reschedule;
		} else {
			wl_cellavoid_free_csa_info(cellavoid_info, csa_info->ndev);
			if (err < 0) {
				WL_ERR(("csa failed, err %d\n", err));
				delay = CSA_DELAYWORK_FAIL_INTERVAL;
			} else {
				delay = CSA_DELAYWORK_CSA_INTERVAL;
			}

		}
	}

exit:
	if (list_empty(&cellavoid_info->csa_info_list)) {
		mutex_unlock(&cellavoid_info->sync);
		return;
	}

reschedule:
	if (cellavoid_info->csa_reschedule_cnt < CSA_MAX_RETRY_CNT) {
		cellavoid_info->csa_reschedule_cnt++;
		schedule_delayed_work(&cfg->csa_delayed_work,
			msecs_to_jiffies((const unsigned int)delay));
	} else {
		WL_ERR(("Hit CSA retry limit\n"));
		wl_cellavoid_free_csa_info_list(cellavoid_info);
	}
	mutex_unlock(&cellavoid_info->sync);
}

/* Move channel items from unsafe channel list (cellular channel list) to
 * safe channel list(available channel list)
 */
static void
wl_cellavoid_clear_cell_chan_list(wl_cellavoid_info_t *cellavoid_info)
{
	wl_cellavoid_chan_info_t *chan_info, *next;

	WL_MEM(("%s: Enter\n", __FUNCTION__));
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->cell_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		list_del(&chan_info->list);
		/* Restore channel info to the value of safe channel */
		chan_info->pwr_cap = CELLAVOID_TXCAP_MAX_VAL;
		list_add(&chan_info->list, &cellavoid_info->avail_chan_info_list);
	}
	cellavoid_info->cell_chan_info_cnt = 0;
	cellavoid_info->mandatory_flag = 0;
}

/* Free all the channel items in the safe channel list(available channel list)
 * Called on country code change
 */
static void
wl_cellavoid_free_avail_chan_list(wl_cellavoid_info_t *cellavoid_info)
{
	wl_cellavoid_chan_info_t *chan_info, *next;

	WL_MEM(("%s: Enter\n", __FUNCTION__));
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->avail_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		list_del(&chan_info->list);
		MFREE(cellavoid_info->osh, chan_info, sizeof(*chan_info));
	}
}

/* Used when framework sets unsafe channel
 * If the chanspec from the framework matches with items
 * in the safe channel list(available channel list)
 * Then, detach the item from the list, and returns the pointer for the channel item
 */
static wl_cellavoid_chan_info_t *
wl_cellavoid_get_chan_info_from_avail_chan_list(wl_cellavoid_info_t *cellavoid_info,
	chanspec_t chanspec)
{
	wl_cellavoid_chan_info_t *chan_info, *next;
	wl_cellavoid_chan_info_t *ret = NULL;
	char chanspec_str[CHANSPEC_STR_LEN];

	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->avail_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (chan_info->chanspec == chanspec) {
			list_del(&chan_info->list);
			wf_chspec_ntoa(chanspec, chanspec_str);
			WL_INFORM(("removed %s (0x%x) in avail list\n", chanspec_str, chanspec));
			ret = chan_info;
			break;
		}
	}

	return ret;
}

/* Check the chanspec is whether safe or unsafe */
static cellavoid_ch_state_t
wl_cellavoid_get_chan_info(wl_cellavoid_info_t *cellavoid_info, chanspec_t chanspec)
{
	wl_cellavoid_chan_info_t *chan_info, *next;

	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->cell_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (chan_info->chanspec == chanspec) {
			return CELLAVOID_STATE_CH_UNSAFE;
		}
	}

	return CELLAVOID_STATE_CH_SAFE;
}

static cellavoid_ch_state_t
wl_cellavoid_get_chan_info_overlap(wl_cellavoid_info_t *cellavoid_info, chanspec_t chanspec)
{
	wl_cellavoid_chan_info_t *chan_info, *next;

	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->cell_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (wf_chspec_overlap(chan_info->chanspec, chanspec)) {
			return CELLAVOID_STATE_CH_UNSAFE;
		}
	}

	return CELLAVOID_STATE_CH_SAFE;
}

bool
wl_cellavoid_is_safe(void *cai, chanspec_t chanspec)
{
	wl_cellavoid_info_t *cellavoid_info = cai;

	if (wl_cellavoid_get_chan_info(cellavoid_info, chanspec) == CELLAVOID_STATE_CH_UNSAFE) {
		return FALSE;
	} else {
		return TRUE;
	}
}

bool
wl_cellavoid_is_safe_overlap(void *cai, chanspec_t chanspec)
{
	wl_cellavoid_info_t *cellavoid_info = cai;

	if (wl_cellavoid_get_chan_info_overlap(cellavoid_info, chanspec)
			== CELLAVOID_STATE_CH_UNSAFE) {
		return FALSE;
	} else {
		return TRUE;
	}
}

bool
wl_cellavoid_mandatory_isset(void *cai, enum nl80211_iftype type)
{
	bool mandatory = FALSE;
	wl_cellavoid_info_t *cellavoid_info = cai;

	switch (type) {
		case NL80211_IFTYPE_P2P_GO:
		case NL80211_IFTYPE_P2P_DEVICE:
			if (cellavoid_info->mandatory_flag & WL_CELL_AVOID_WIFI_DIRECT) {
				mandatory = TRUE;
			}
			break;
		case NL80211_IFTYPE_AP:
			if (cellavoid_info->mandatory_flag & WL_CELL_AVOID_SOFTAP) {
				mandatory = TRUE;
			}
			break;

		default:
			break;
	}
	return mandatory;
}

wifi_interface_mode
wl_cellavoid_mandatory_to_usable_channel_filter(void *cai)
{
	wifi_interface_mode mode = 0;
	wl_cellavoid_info_t *cellavoid_info = cai;

	if (cellavoid_info->mandatory_flag & WL_CELL_AVOID_WIFI_DIRECT) {
		mode |= ((1U << WIFI_INTERFACE_P2P_GO) | (1U << WIFI_INTERFACE_P2P_CLIENT));
	}
	if (cellavoid_info->mandatory_flag & WL_CELL_AVOID_SOFTAP) {
		mode |= (1U << WIFI_INTERFACE_SOFTAP);
	}
	if (cellavoid_info->mandatory_flag & WL_CELL_AVOID_NAN) {
		mode |= (1U << WIFI_INTERFACE_NAN);
	}

	return mode;
}

/* Check the chanspec is whether safe or unsafe */
bool
wl_cellavoid_operation_allowed(void *cai, chanspec_t chanspec,
	enum nl80211_iftype type)
{
	bool allowed = TRUE;
	wl_cellavoid_info_t *cellavoid_info = cai;

	if (wl_cellavoid_get_chan_info(cellavoid_info, chanspec) == CELLAVOID_STATE_CH_UNSAFE &&
		wl_cellavoid_mandatory_isset(cellavoid_info, type)) {
		allowed = FALSE;
	}

	return allowed;
}

/* Used when moving channel item to the unsafe channel list (cellular channel list) */
static void
wl_cellavoid_move_chan_info_to_cell_chan_list(wl_cellavoid_info_t *cellavoid_info,
	wl_cellavoid_chan_info_t * chan_info)
{
	list_add(&chan_info->list, &cellavoid_info->cell_chan_info_list);
	cellavoid_info->cell_chan_info_cnt++;
}

/* Used when moving channel item to the safe channel list (avail channel list) */
static void
wl_cellavoid_move_chan_info_to_avail_chan_list(wl_cellavoid_info_t *cellavoid_info,
	wl_cellavoid_chan_info_t * chan_info)
{
	list_add(&chan_info->list, &cellavoid_info->avail_chan_info_list);
}

/* Used when sorting the channel list
 * wider bw, large channel number comes first and
 * narrower bw, small channel number comes later after sorting
 */
static int
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 70))
wl_cellavoid_chan_info_compare(void *priv, const struct list_head *a, const struct list_head *b)
#else
wl_cellavoid_chan_info_compare(void *priv, struct list_head *a, struct list_head *b)
#endif
{
	uint8 i1_chan, i2_chan;
	uint16 i1_bw, i2_bw;

	wl_cellavoid_chan_info_t *info1 = CONTAINEROF(a, wl_cellavoid_chan_info_t, list);
	wl_cellavoid_chan_info_t *info2 = CONTAINEROF(b, wl_cellavoid_chan_info_t, list);

	i1_chan = wf_chspec_ctlchan(info1->chanspec);
	i2_chan = wf_chspec_ctlchan(info2->chanspec);
	i1_bw = CHSPEC_BW(info1->chanspec);
	i2_bw = CHSPEC_BW(info2->chanspec);

	/* Wider BW comes first */
	if (i1_bw > i2_bw) {
		return -1;
	} else if (i1_bw < i2_bw) {
		return 1;
	} else {
		if (i1_chan > i2_chan) {
			return -1;
		} else if (i1_chan < i2_chan) {
			return 1;
		} else {
			return 0;
		}
	}
}

/* Used when sorting the channel list, sort both unsafe channel list (cellular channel list) and
 * safe channel list (avail channel list)
 */
static void
wl_cellavoid_sort_chan_info_list(wl_cellavoid_info_t *cellavoid_info)
{
	/* Sorting ascending */
	list_sort(NULL, &cellavoid_info->cell_chan_info_list, wl_cellavoid_chan_info_compare);
	list_sort(NULL, &cellavoid_info->avail_chan_info_list, wl_cellavoid_chan_info_compare);
}

#ifdef WL_CELLULAR_CHAN_AVOID_DUMP
/* Dump function, shows chanspec/pwrcap item both in the unsafe channel list (cellular channel list)
 * and safe channel list (avail channel list)
 */
static void
wl_cellavoid_dump_chan_info_list(wl_cellavoid_info_t *cellavoid_info)
{
	wl_cellavoid_chan_info_t *chan_info, *next;
	char chanspec_str[CHANSPEC_STR_LEN];
	int cell_chan_info_cnt = 0, avail_chan_info_cnt = 0;

	WL_MEM(("%s: Enter\n", __FUNCTION__));
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->cell_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		wf_chspec_ntoa(chan_info->chanspec, chanspec_str);
		WL_MEM(("Cellular : chanspec %s(%x), pwrcap %d\n",
			chanspec_str, chan_info->chanspec, chan_info->pwr_cap));
		cell_chan_info_cnt++;
	}

	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->avail_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		wf_chspec_ntoa(chan_info->chanspec, chanspec_str);
		WL_MEM(("Avail : chanspec %s(%x), pwrcap %d\n",
			chanspec_str, chan_info->chanspec, chan_info->pwr_cap));
		avail_chan_info_cnt++;
	}

	WL_INFORM_MEM(("%s: cellavoid_info->cell_chan_info_cnt(%d), mandatory_flag(%d), "
		"cell_chan_info_cnt(%d), avail_chan_info_cnt(%d)\n",
		__FUNCTION__, cellavoid_info->cell_chan_info_cnt,
		cellavoid_info->mandatory_flag, cell_chan_info_cnt, avail_chan_info_cnt));
}

void wl_cellavoid_sanity_check_chan_info_list(void *cai)
{
	wl_cellavoid_info_t *cellavoid_info = cai;
	wl_cellavoid_chan_info_t *chan_info, *next;
	char chanspec_str[CHANSPEC_STR_LEN];
	int cell_chan_info_cnt = 0, avail_chan_info_cnt = 0;

	WL_MEM(("%s: Enter\n", __FUNCTION__));
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->cell_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		wf_chspec_ntoa(chan_info->chanspec, chanspec_str);
		WL_MEM(("Cellular : chanspec %s(%x), pwrcap %d\n",
			chanspec_str, chan_info->chanspec, chan_info->pwr_cap));
		cell_chan_info_cnt++;
	}

	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->avail_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		wf_chspec_ntoa(chan_info->chanspec, chanspec_str);
		WL_MEM(("Avail : chanspec %s(%x), pwrcap %d\n",
			chanspec_str, chan_info->chanspec, chan_info->pwr_cap));
		avail_chan_info_cnt++;
	}

	WL_INFORM_MEM(("%s: cellavoid_info->cell_chan_info_cnt(%d), mandatory_flag(%d), "
		"cell_chan_info_cnt(%d), avail_chan_info_cnt(%d)\n",
		__FUNCTION__, cellavoid_info->cell_chan_info_cnt,
		cellavoid_info->mandatory_flag, cell_chan_info_cnt, avail_chan_info_cnt));

	if ((cell_chan_info_cnt == 0) && (avail_chan_info_cnt == 0)) {
		WL_ERR(("### both cell&avail channel list are zero!! ###\n"));
	}
}
#endif /* WL_CELLULAR_CHAN_AVOID_DUMP */

/* Allocate channel item(chanspec + pwrcap), called upon country code change */
static wl_cellavoid_chan_info_t *
wl_cellavoid_alloc_chan_info(wl_cellavoid_info_t *cellavoid_info, chanspec_t chanspec)
{
	wl_cellavoid_chan_info_t *chan_info = NULL;

	/* Allocate availabe channel info */
	chan_info = (wl_cellavoid_chan_info_t *)
		MALLOCZ(cellavoid_info->osh, sizeof(wl_cellavoid_chan_info_t));
	if (!chan_info) {
		WL_ERR(("failed to allocate chan info\n"));
		return NULL;
	}

	chan_info->chanspec = chanspec;
	chan_info->pwr_cap = CELLAVOID_TXCAP_MAX_VAL;

	return chan_info;
}

/* Allocate channel item(chanspec + pwrcap) per band (2g or 5g) */
static int
wl_cellavoid_alloc_avail_chan_list_band(wl_cellavoid_info_t *cellavoid_info,
	struct ieee80211_supported_band *sband)
{
	struct ieee80211_channel *channel;
	wl_cellavoid_chan_info_t *chan_info = NULL;
	int i, j;
	uint16 bandwidth[] = {WL_CHANSPEC_BW_40, WL_CHANSPEC_BW_80};
	uint8 ctlchan;
	chanspec_band_t band;
	chanspec_t chanspec = INVCHANSPEC;

	for (i = 0; i < sband->n_channels; i++) {
		channel = &sband->channels[i];
		/* If channel from Kernel wiphy is disabled state or DFS channel, drop */
		if (channel->flags & IEEE80211_CHAN_DISABLED ||
			IS_RADAR_CHAN(channel->flags)) {
			WL_MEM(("chanspec %x is not allowed\n", channel->hw_value));
			continue;
		}

		/* Allocate channel item (chanspec + pwrcap) */
		chan_info = wl_cellavoid_alloc_chan_info(cellavoid_info, channel->hw_value);
		if (chan_info == NULL) {
			goto free_list;
		}
		WL_MEM(("chanspec %x is added to avail_chan_list\n", channel->hw_value));

		/* Move allocated channel item(20Mhz)
		 * to the safe channel list (avail channel list)
		 */
		wl_cellavoid_move_chan_info_to_avail_chan_list(cellavoid_info, chan_info);

		if (sband->band == NL80211_BAND_5GHZ) {
			/* If 5ghz, also create channel item for 40/80 chanspec */
			ctlchan = wf_chspec_ctlchan(chan_info->chanspec);
			band = CHSPEC_BAND(chan_info->chanspec);
			ASSERT(band == WL_CHANSPEC_BAND_5G);
			for (j = 0; j < (sizeof(bandwidth) / sizeof(uint16)); j++) {
#ifdef WL_6G_320_SUPPORT
				chanspec = wf_create_chspec_from_primary(ctlchan,
					bandwidth[j], band, 0);
#else
				chanspec = wf_create_chspec_from_primary(ctlchan,
					bandwidth[j], band);
#endif /* WL_6G_320_SUPPORT */
				if (chanspec == INVCHANSPEC) {
					WL_ERR(("invalid chanspec ctlchan %d, band %d "
						"bandwidth %d\n", ctlchan, band, bandwidth[j]));
					goto free_list;
				}
				/* Allocate channel item (chanspec + pwrcap) for 40/80 chanspec */
				chan_info = wl_cellavoid_alloc_chan_info(cellavoid_info, chanspec);
				if (chan_info == NULL) {
					goto free_list;
				}
				WL_MEM(("chanspec %x is added to avail_chan_list\n", chanspec));

				/* Add 40/80 chanspec item to the available channel list */
				wl_cellavoid_move_chan_info_to_avail_chan_list(cellavoid_info,
					chan_info);
			}
		}
	}
	return BCME_OK;

free_list:
	/* If there's an error, free all the items in the safe channel list (avail channel list) */
	wl_cellavoid_free_avail_chan_list(cellavoid_info);
	return BCME_NOMEM;
}

#define MAX_20MHZ_CHANNELS   16u

/* This function is used verifying channel items
 * created by wl_cellavoid_alloc_avail_chan_list are valid
 * by comparing the channel item to chan_info_list from FW
 * If it does not match with chan_info_list, drop
 * CH165 only support 20MHz BW, so 165/40, 165/80 chanspecs created by
 * wl_cellavoid_alloc_avail_chan_list_band are dropped by this function
 */
static int
wl_cellavoid_verify_avail_chan_list(struct bcm_cfg80211 *cfg, wl_cellavoid_info_t *cellavoid_info)
{
	wl_cellavoid_chan_info_t *chan_info, *next;
	u16 list_count;
	void *dngl_chan_list;
	bool legacy_chan_info = FALSE;
	bool found;
	int i, j, k, err;
	chanspec_t chanspec = 0;
	char chanspec_str[CHANSPEC_STR_LEN];
	uint32 restrict_chan, chaninfo;
	u32 arr_idx = 0, band;
	u8 chan_array[MAX_20MHZ_CHANNELS] = {0};
	wl_chanspec_attr_v1_t overlap[MAX_20MHZ_CHANNELS];

	/* Get chan_info_list or chanspec from FW */
#define LOCAL_BUF_LEN 4096
	dngl_chan_list = MALLOCZ(cfg->osh, LOCAL_BUF_LEN);
	if (dngl_chan_list == NULL) {
		WL_ERR(("failed to allocate local buf\n"));
		return BCME_NOMEM;
	}

	err = wldev_iovar_getbuf_bsscfg(bcmcfg_to_prmry_ndev(cfg), "chan_info_list", NULL,
		0, dngl_chan_list, LOCAL_BUF_LEN, 0, &cfg->ioctl_buf_sync);
	if (err == BCME_UNSUPPORTED) {
		WL_INFORM(("get chan_info_list, UNSUPPORTED\n"));
		err = wldev_iovar_getbuf_bsscfg(bcmcfg_to_prmry_ndev(cfg), "chanspecs", NULL,
			0, dngl_chan_list, LOCAL_BUF_LEN, 0, &cfg->ioctl_buf_sync);
		if (err != BCME_OK) {
			WL_ERR(("get chanspecs err(%d)\n", err));
			MFREE(cfg->osh, dngl_chan_list, LOCAL_BUF_LEN);
			return err;
		}
		/* Update indicating legacy chan info usage */
		legacy_chan_info = TRUE;
	} else if (err != BCME_OK) {
		WL_ERR(("get chan_info_list err(%d)\n", err));
		MFREE(cfg->osh, dngl_chan_list, LOCAL_BUF_LEN);
		return err;
	}

	list_count = legacy_chan_info ? ((wl_uint32_list_t *)dngl_chan_list)->count :
		((wl_chanspec_list_v1_t *)dngl_chan_list)->count;

	/* Comparing the channel item to chan_info_list from FW
	 * If the chanspec in channel item is not supported by FW,
	 * delete it from the safe channel list (avail channel list)
	 */
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->avail_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		wf_get_all_ext(chan_info->chanspec, chan_array);
		bzero(overlap, sizeof(overlap));
		band = CHSPEC_BAND(chan_info->chanspec);
		arr_idx = 0;
		found = FALSE;
		for (i = 0; i < dtoh32(list_count); i++) {
			if (legacy_chan_info) {
				chanspec = (chanspec_t)
					dtoh32(((wl_uint32_list_t *)dngl_chan_list)->element[i]);
				restrict_chan = 0x0;
			} else {
				chanspec = (chanspec_t)dtoh32
				(((wl_chanspec_list_v1_t *)dngl_chan_list)->chspecs[i].chanspec);

				chaninfo = dtoh32
				(((wl_chanspec_list_v1_t *)dngl_chan_list)->chspecs[i].chaninfo);

				/* Store chanspec attribute of subbands channel. */
				if ((CHSPEC_BAND(chanspec) == band) &&
					(CHSPEC_BW(chanspec) == WL_CHANSPEC_BW_20)) {
					for (j = 0; j < MAX_20MHZ_CHANNELS; j++) {
						if (!chan_array[j]) {
							/* if entry is empty, break */
							break;
						}
						if (chan_array[j] == CHSPEC_CHANNEL(chanspec)) {
							overlap[arr_idx].chanspec = chanspec;
							overlap[arr_idx].chaninfo = chaninfo;
							WL_DBG(("sel_chspec:%x overlap_chspec:%x\n",
									chan_info->chanspec,
									overlap[arr_idx].chanspec));
							arr_idx++;
							break;
						}
					}
				}

				restrict_chan = ((chaninfo & WL_CHAN_RADAR) ||
					(chaninfo & WL_CHAN_PASSIVE) ||
					(chaninfo & WL_CHAN_CLM_RESTRICTED));

				if (chan_info->chanspec == chanspec) {
					for (k = 0; k < arr_idx; k++) {
						restrict_chan |=
							((overlap[k].chaninfo & WL_CHAN_RADAR) ||
							(overlap[k].chaninfo & WL_CHAN_PASSIVE) ||
							(overlap[k].chaninfo &
								WL_CHAN_CLM_RESTRICTED));
					}
				}
			}

			if ((!restrict_chan) && (chan_info->chanspec == chanspec)) {
				found = TRUE;
				break;
			}
		}

		if (found == FALSE) {
			list_del(&chan_info->list);
			wf_chspec_ntoa(chan_info->chanspec, chanspec_str);
			WL_INFORM_MEM(("chanspec %s(%x) is removed from avail list\n",
				chanspec_str, chan_info->chanspec));
			MFREE(cfg->osh, chan_info, sizeof(*chan_info));
		}
	}
	MFREE(cfg->osh, dngl_chan_list, LOCAL_BUF_LEN);
#undef LOCAL_BUF_LEN

	return BCME_OK;
}

/* Allocate channel item(chanspec + pwrcap) from both band (2g and 5g)
 * Channel information comes from wiphy in Kernel
 */
static int
wl_cellavoid_alloc_avail_chan_list(struct wiphy *wiphy, wl_cellavoid_info_t *cellavoid_info)
{
	struct ieee80211_supported_band *sband;
	int ret;

	WL_MEM(("%s: Enter\n", __FUNCTION__));
	sband = wiphy->bands[IEEE80211_BAND_2GHZ];
	if (!sband || !sband->n_channels) {
		WL_ERR(("No 2ghz channel exists\n"));
		return BCME_ERROR;
	}

	ret = wl_cellavoid_alloc_avail_chan_list_band(cellavoid_info, sband);
	if (ret) {
		return ret;
	}

	sband = wiphy->bands[IEEE80211_BAND_5GHZ];
	if (!sband || !sband->n_channels) {
		WL_ERR(("No 5ghz channel exists\n"));
		return BCME_OK;
	}

	ret = wl_cellavoid_alloc_avail_chan_list_band(cellavoid_info, sband);
	if (ret) {
		return ret;
	}

	/* Verify channel list from the IOVAR chanspecs */
	return wl_cellavoid_verify_avail_chan_list(wiphy_priv(wiphy), cellavoid_info);
}

/* Used to remove existing pwrcap in the FW */
static int
wl_cellavoid_restore_txpwrcap(struct bcm_cfg80211 *cfg, wl_cellavoid_info_t *cellavoid_info)
{
	int ret;
	int hdr_size, payload_size, total_size;
	bcm_iov_buf_t *iov_buf = NULL;
	wl_cell_avoid_ch_info_v1_t *subcmd;

	hdr_size = OFFSETOF(bcm_iov_buf_t, data);
	payload_size = sizeof(*subcmd);
	total_size = hdr_size + payload_size;

	iov_buf = (bcm_iov_buf_t *)MALLOCZ(cfg->osh, total_size);
	if (iov_buf == NULL) {
		WL_ERR(("memory alloc failure size %d\n", total_size));
		return BCME_NOMEM;
	}

	iov_buf->version = htod16(WL_CELL_AVOID_IOV_VERSION_1);
	iov_buf->id = htod16(WL_CELL_AVOID_SUBCMD_CH_INFO);
	iov_buf->len = htod16(payload_size);

	subcmd = (wl_cell_avoid_ch_info_v1_t *)iov_buf->data;
	subcmd->version = htod16(WL_CELL_AVOID_SUB_IOV_VERSION_1);
	subcmd->length = htod16(payload_size);
	/* This will remove cellular channel info in FW and FW will reset txpwrcap */
	subcmd->flags = htod16(WL_CELL_AVOID_REMOVE_CH_INFO);

	ret = wldev_iovar_setbuf(bcmcfg_to_prmry_ndev(cfg), "cellavoid", (char *)iov_buf,
		total_size, cfg->ioctl_buf, WLC_IOCTL_SMLEN, &cfg->ioctl_buf_sync);
	if (ret != BCME_OK) {
		WL_ERR(("fail to restore txpwrcap ret : %d\n", ret));
	}

	MFREE(cfg->osh, iov_buf, total_size);

	return ret;
}

/* Used to deliver chanspec + pwrcap to the FW */
static int
wl_cellavoid_apply_txpwrcap(struct bcm_cfg80211 *cfg, wl_cellavoid_info_t *cellavoid_info)
{
	int ret;
	int hdr_size, payload_size, total_size;
	bcm_iov_buf_t *iov_buf = NULL;
	wl_cell_avoid_ch_info_v1_t *subcmd;
	wl_cellavoid_chan_info_t *chan_info, *next;
	int i = 0;

	/* If there isn't any unsafe channel(cellular channel),
	 * then remove cellular channel + pwrcap info in FW
	 */
	if (cellavoid_info->cell_chan_info_cnt == 0) {
		return wl_cellavoid_restore_txpwrcap(cfg, cellavoid_info);
	}

	hdr_size = OFFSETOF(bcm_iov_buf_t, data);
	payload_size = sizeof(*subcmd) +
		cellavoid_info->cell_chan_info_cnt * sizeof(wl_cell_pwrcap_chanspec_v1_t);
	total_size = hdr_size + payload_size;

	iov_buf = (bcm_iov_buf_t *)MALLOCZ(cfg->osh, total_size);
	if (iov_buf == NULL) {
		WL_ERR(("memory alloc failure size %d\n", total_size));
		return -ENOMEM;
	}

	iov_buf->version = htod16(WL_CELL_AVOID_IOV_VERSION_1);
	iov_buf->id = htod16(WL_CELL_AVOID_SUBCMD_CH_INFO);
	iov_buf->len = htod16(payload_size);

	subcmd = (wl_cell_avoid_ch_info_v1_t *)iov_buf->data;
	subcmd->version = htod16(WL_CELL_AVOID_SUB_IOV_VERSION_1);
	subcmd->length = htod16(payload_size);
	subcmd->flags = htod16(cellavoid_info->mandatory_flag);
	subcmd->cnt =  htod16(cellavoid_info->cell_chan_info_cnt);

	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->cell_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		subcmd->list[i].chanspec = htod16(chan_info->chanspec);
		subcmd->list[i].pwrcap = chan_info->pwr_cap;
		i++;
	}
	ASSERT(cellavoid_info->cell_chan_info_cnt == i);

	ret = wldev_iovar_setbuf(bcmcfg_to_prmry_ndev(cfg), "cellavoid", (char *)iov_buf,
		total_size, cfg->ioctl_buf, WLC_IOCTL_MEDLEN, &cfg->ioctl_buf_sync);
	if (ret != BCME_OK) {
		WL_ERR(("fail to set txpwrcap ret : %d\n", ret));
	}

	return ret;
}

int
wl_cellavoid_set_requested_freq_bands(struct net_device *ndev,
	void *cai, u32 *pElem_freq, u32 freq_list_len)
{
	int i, j;
	chanspec_t chanspec;
	wl_cellavoid_info_t *cellavoid_info = cai;

	for (i = 0; i < MAX_AP_INTERFACE; i++) {
		if (cellavoid_info->req_band[i].ndev == NULL) {
			break;
		}
	}

	if (i == MAX_AP_INTERFACE) {
		WL_ERR(("No empty slot, reset all slots.\n"));
		for (i = 0; i < MAX_AP_INTERFACE; i++) {
			cellavoid_info->req_band[i].ndev = NULL;
			cellavoid_info->req_band[i].req_band = WLC_BAND_INVALID;
		}

		/* Reset i = 0 */
		i = 0;
	}

	cellavoid_info->req_band[i].ndev = ndev;
	cellavoid_info->req_band[i].req_band = 0;

	for (j = 0; j < freq_list_len; j++) {
		chanspec = wl_freq_to_chanspec(pElem_freq[j]);
		/* mark all the bands found */
		cellavoid_info->req_band[i].req_band |=
			CHSPEC_TO_WLC_BAND(CHSPEC_BAND(chanspec));
	}

	WL_INFORM_MEM(("name %s, req_band %x is in slot %d\n",
		cellavoid_info->req_band[i].ndev->name, cellavoid_info->req_band[i].req_band, i));

#ifndef WL_SOFTAP_6G
	if (cellavoid_info->req_band[i].req_band == WLC_BAND_6G) {
		WL_INFORM_MEM(("6G softap is not supported\n"));
		return BCME_UNSUPPORTED;
	}
#endif /* !WL_SOFTAP_6G */

	return BCME_OK;
}

void
wl_cellavoid_clear_requested_freq_bands(struct net_device *ndev, void *cai)
{
	int i;
	wl_cellavoid_info_t *cellavoid_info = cai;

	for (i = 0; i < MAX_AP_INTERFACE; i++) {
		if (cellavoid_info->req_band[i].ndev == ndev) {
			break;
		}
	}

	if (i == MAX_AP_INTERFACE) {
		WL_ERR(("No matched slot, ignore.\n"));
		return;
	}

	WL_INFORM_MEM(("name %s, req_band %x in slot %d cleared\n",
		cellavoid_info->req_band[i].ndev->name, cellavoid_info->req_band[i].req_band, i));

	cellavoid_info->req_band[i].ndev = NULL;
	cellavoid_info->req_band[i].req_band = WLC_BAND_INVALID;
}

static int
wl_cellavoid_find_requested_freq_bands(struct net_device *ndev, wl_cellavoid_info_t *cellavoid_info)
{
	int i;

	for (i = 0; i < MAX_AP_INTERFACE; i++) {
		if (cellavoid_info->req_band[i].ndev == ndev) {
			break;
		}
	}

	if (i == MAX_AP_INTERFACE) {
		for (i = 0; i < MAX_AP_INTERFACE; i++) {
			WL_ERR(("can not find valid slot, id %d, name %s, req_band %x\n",
				i, cellavoid_info->req_band[i].ndev->name,
				cellavoid_info->req_band[i].req_band));

		}
		return WLC_BAND_INVALID;
	}

	return cellavoid_info->req_band[i].req_band;
}

static wl_cellavoid_chan_info_t *
wl_cellavoid_find_chinfo_sameband(wl_cellavoid_info_t *cellavoid_info,
	chanspec_t chanspec)
{
	wl_cellavoid_chan_info_t *chan_info, *next;
	wl_cellavoid_chan_info_t *ret = NULL;

	/* Find channel info with same band in available channel list(safe channel) first */
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->avail_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (CHSPEC_TO_WLC_BAND(chan_info->chanspec) ==
			CHSPEC_TO_WLC_BAND(chanspec)) {
			ret = chan_info;
			WL_INFORM_MEM(("chanspec %x found in avail list\n", chan_info->chanspec));
			goto exit;
		}
	}

	/* If it's not found and mandatory flag is set return null */
	if (cellavoid_info->mandatory_flag & WL_CELL_AVOID_SOFTAP) {
		WL_INFORM_MEM(("No chanspec in avail list and mandatory flag set\n"));
		goto exit;
	}

	/* If it's not found and mandatory flag is zeo,
	 * Find the current chanspec from cellular channel list(unsafe list)
	 * This is to reduce the number of channel switch trial
	 */
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->cell_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (chan_info->chanspec == chanspec) {
			ret = chan_info;
			WL_INFORM_MEM(("chanspec %x found in cellular list\n",
				chan_info->chanspec));
			goto exit;
		}
	}

exit:
	if (ret == NULL) {
		WL_INFORM_MEM(("No chanspec in avail list/cellular list\n"));
	}

	return ret;
}

static wl_cellavoid_chan_info_t *
wl_cellavoid_find_chinfo_fromchspec(wl_cellavoid_info_t *cellavoid_info,
	chanspec_t chanspec)
{
	wl_cellavoid_chan_info_t *chan_info, *next;
	wl_cellavoid_chan_info_t *ret = NULL;

	/* Find in available channel list(safe channel) first */
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->avail_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		/* List is always sorted to come wide bw comes first,
		 * so the first one is the widest one
		 */
		if (wf_chspec_ctlchan(chan_info->chanspec) == wf_chspec_ctlchan(chanspec)) {
			/* check the overlap for 5G band only */
			if (CHSPEC_IS2G(chan_info->chanspec) ||
					wl_cellavoid_is_safe_overlap(cellavoid_info,
					chan_info->chanspec)) {
				ret = chan_info;
				WL_INFORM_MEM(("ctrl channel %d (0x%x) found in avail list\n",
					wf_chspec_ctlchan(chan_info->chanspec),
					chan_info->chanspec));
				goto exit;
			}
		}
	}

	/* If it's not found and mandatory flag is set return null */
	if (cellavoid_info->mandatory_flag & WL_CELL_AVOID_SOFTAP) {
		WL_INFORM_MEM(("No chanspec in avail list and mandatory flag set\n"));
		goto exit;
	}

	/* If it's not found and mandatory flag is zeo,
	 * pick up the chanspec from cellular channel list(unsafe list)
	 */
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->cell_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		/* List is always sorted to come wide bw comes first,
		 * so the first one is the widest one
		 */
		if (wf_chspec_ctlchan(chan_info->chanspec) == wf_chspec_ctlchan(chanspec)) {
			ret = chan_info;
			WL_INFORM_MEM(("ctrl channel %d (0x%x) found in cellular list\n",
				wf_chspec_ctlchan(chan_info->chanspec), chan_info->chanspec));
			goto exit;
		}
	}

exit:
	if (ret == NULL) {
		wl_cellavoid_chan_info_t *chan_info, *next;
		char chanspec_str[CHANSPEC_STR_LEN];

		GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
		list_for_each_entry_safe(chan_info, next, &cellavoid_info->cell_chan_info_list,
				list) {
			GCC_DIAGNOSTIC_POP();
			wf_chspec_ntoa(chan_info->chanspec, chanspec_str);
			WL_INFORM_MEM(("Cellular : chanspec %s(%x), pwrcap %d\n",
				chanspec_str, chan_info->chanspec, chan_info->pwr_cap));
		}

		GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
		list_for_each_entry_safe(chan_info, next, &cellavoid_info->avail_chan_info_list,
				list) {
			GCC_DIAGNOSTIC_POP();
			wf_chspec_ntoa(chan_info->chanspec, chanspec_str);
			WL_INFORM_MEM(("Avail : chanspec %s(%x), pwrcap %d\n",
				chanspec_str, chan_info->chanspec, chan_info->pwr_cap));
		}
		WL_INFORM_MEM(("No chanspec in avail list/cellular list\n"));
	}

	return ret;
}

static wl_cellavoid_chan_info_t *
wl_cellavoid_find_chinfo_fromband(wl_cellavoid_info_t *cellavoid_info, int band)
{
	wl_cellavoid_chan_info_t *chan_info, *next;
	wl_cellavoid_chan_info_t *ret = NULL;

	/* Find in available channel list(safe channel) first */
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->avail_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (CHSPEC_TO_WLC_BAND(chan_info->chanspec) == band) {
			ret = chan_info;
			WL_INFORM_MEM(("chanspec %x found in avail list\n", chan_info->chanspec));
			goto exit;
		}
	}

	/* If it's not found and mandatory flag is set return null */
	if (cellavoid_info->mandatory_flag & WL_CELL_AVOID_SOFTAP) {
		WL_INFORM_MEM(("No chanspec in avail list and mandatory flag set\n"));
		goto exit;
	}

	/* If it's not found and mandatory flag is zeo,
	 * pick up the chanspec from cellular channel list(unsafe list)
	 */
	GCC_DIAGNOSTIC_PUSH_SUPPRESS_CAST();
	list_for_each_entry_safe(chan_info, next, &cellavoid_info->cell_chan_info_list, list) {
		GCC_DIAGNOSTIC_POP();
		if (CHSPEC_TO_WLC_BAND(chan_info->chanspec) == band) {
			ret = chan_info;
			WL_INFORM_MEM(("chanspec %x found in cellular list\n",
				chan_info->chanspec));
			goto exit;
		}
	}

exit:
	if (ret == NULL) {
		WL_INFORM_MEM(("No chanspec in avail list/cellular list\n"));
	}

	return ret;
}

chanspec_t
wl_cellavoid_find_chspec_fromband(void *cai, int band)
{
	wl_cellavoid_chan_info_t* chan_info;
	chanspec_t chanspec;
	wl_cellavoid_info_t *cellavoid_info = cai;

	chan_info = wl_cellavoid_find_chinfo_fromband(cellavoid_info, band);
	if (chan_info == NULL) {
		chanspec = INVCHANSPEC;
	} else {
		chanspec = chan_info->chanspec;
	}

	return chanspec;
}

chanspec_t
wl_cellavoid_find_widechspec_fromchspec(void *cai, chanspec_t chanspec)
{
	wl_cellavoid_chan_info_t* chan_info;
	chanspec_t wide_chanspec;
	wl_cellavoid_info_t *cellavoid_info = cai;

	chan_info = wl_cellavoid_find_chinfo_fromchspec(cellavoid_info, chanspec);
	if (chan_info == NULL) {
		wide_chanspec = INVCHANSPEC;
	} else {
		wide_chanspec = chan_info->chanspec;
	}

	return wide_chanspec;
}

static wl_cellavoid_chan_info_t *
wl_cellavoid_find_ap_chan_info(struct bcm_cfg80211 *cfg, chanspec_t ap_chspec,
	chanspec_t sta_chspec, int csa_target_band)
{
	int ap_band, sta_band = WLC_BAND_INVALID;
	wl_cellavoid_chan_info_t *chan_info = NULL;

	WL_INFORM_MEM(("AP chspec %x, STA chspec %x, CSA target %x\n",
		ap_chspec, sta_chspec, csa_target_band));

	if (csa_target_band == WLC_BAND_INVALID) {
		return NULL;
	}

	/* This will be checked later */
	if (csa_target_band & WLC_BAND_6G) {
		csa_target_band &= ~WLC_BAND_6G;
	}

	ap_band = CHSPEC_TO_WLC_BAND(ap_chspec);
	if (sta_chspec) {
		sta_band = CHSPEC_TO_WLC_BAND(sta_chspec);
	}

	/* Same band CSA first */
	if (csa_target_band & ap_band) {
		if (sta_band == ap_band) {
			/* SCC in this core */
			WL_INFORM_MEM(("STA in the same core, band %d\n", sta_band));
			chan_info = wl_cellavoid_find_chinfo_fromchspec(cfg->cellavoid_info,
					sta_chspec);
		} else {
			/* No STA in this core */
			WL_INFORM_MEM(("No STA in the same core, band %d\n", ap_band));
			chan_info = wl_cellavoid_find_chinfo_sameband(cfg->cellavoid_info,
				ap_chspec);
		}
		csa_target_band &= ~ap_band;
	}

	/* If there's no target to CSA, try in different core */
	if (chan_info == NULL && csa_target_band != 0) {
		if (csa_target_band == sta_band) {
			/* STA in the another core, so check STA chanspec is available to use
			 * Skip DFS case
			 */
			WL_INFORM_MEM(("STA in the another core. band %d\n", csa_target_band));
			if (!wl_is_chanspec_restricted(cfg, sta_chspec)) {
				chan_info = wl_cellavoid_find_chinfo_fromchspec(cfg->cellavoid_info,
					sta_chspec);
			}
		} else {
			/* No STA in another core */
			WL_INFORM_MEM(("No STA in the another core, band %d\n", csa_target_band));
			chan_info = wl_cellavoid_find_chinfo_fromband(cfg->cellavoid_info,
				csa_target_band);
		}
	}

	if (chan_info) {
		WL_INFORM_MEM(("Found chan info %x\n", chan_info->chanspec));
	}

	return chan_info;
}

/* After making the safe channel/unsafe channel list,
 * perform actions needs to be done (AP->CSA and others)
 * Then, deliver chanspec + pwrcap information to the FW
 * by calling wl_cellavoid_apply_txpwrcap
 */
static int
wl_cellavoid_handle_apsta_concurrency(struct bcm_cfg80211 *cfg)
{
	wl_cellavoid_info_t *cellavoid_info = cfg->cellavoid_info;
	wl_ap_oper_data_t ap_oper_data = {0};
	cellavoid_ch_state_t ch_state;
	int i, ap_band = WLC_BAND_INVALID;
	int req_band, csa_target_band;
	uint32 sta_cnt;
	chanspec_t sta_chanspec;
	wl_cellavoid_chan_info_t *csa_chan_info = NULL;
	char chanspec_str1[CHANSPEC_STR_LEN], chanspec_str2[CHANSPEC_STR_LEN];
	int ret = BCME_OK;

	sta_cnt = wl_cfgvif_get_iftype_count(cfg, WL_IF_TYPE_STA);
	if (sta_cnt == MAX_STA_INTERFACE) {
		/* This is STA+STA case */
		return BCME_OK;
	}

	/* Check whether AP and STA is already operational */
	wl_get_ap_chanspecs(cfg, &ap_oper_data);
	sta_chanspec = wl_cfg80211_get_sta_chanspec(cfg);

	/* If there's any AP interface */
	if (ap_oper_data.count > 0) {
		for (i = 0; i < ap_oper_data.count; i++) {
			ch_state = wl_cellavoid_get_chan_info(cellavoid_info,
				ap_oper_data.iface[i].chspec);

			/* If AP is on the safe channel, skip this AP */
			if (ch_state == CELLAVOID_STATE_CH_SAFE) {
				continue;
			}

			/* AP channel is unsafe channel(cellular channel) */
			/* Get AP band */
			ap_band = CHSPEC_TO_WLC_BAND(ap_oper_data.iface[i].chspec);
			if (ap_oper_data.count == MAX_AP_INTERFACE) {
				/* 2AP case, CSA in the same wlc */
				WL_INFORM_MEM(("AP/AP, CSA only in the same band, AP chanspec %x\n",
					ap_oper_data.iface[i].chspec));
				csa_target_band = ap_band;
				if (csa_target_band == WLC_BAND_5G) {
					csa_target_band |= WLC_BAND_6G;
				}
			} else {
				/* 1 AP case, AP bsscfg can move to any wlc */
				WL_INFORM_MEM(("AP, CSA to any band, AP chanspec %x\n",
					ap_oper_data.iface[i].chspec));
				csa_target_band = WLC_BAND_2G | WLC_BAND_5G | WLC_BAND_6G;
			}

			/* Take ACS band info and
			 * filtering csa_target_band with requested ACS band
			 */
			req_band =
				wl_cellavoid_find_requested_freq_bands(ap_oper_data.iface[i].ndev,
				cellavoid_info);

			if (req_band != WLC_BAND_INVALID) {
				/* If ACS band info is valid, apply it as the filter */
				WL_INFORM_MEM(("csa_target_band %x, reqband %x\n",
					csa_target_band, req_band));
				csa_target_band &= req_band;
			}

			/* Handle STA concurrency scenario and get chan into to switch channel */
			csa_chan_info = wl_cellavoid_find_ap_chan_info(cfg,
				ap_oper_data.iface[i].chspec, sta_chanspec, csa_target_band);

			/* If channel exists, schedule channel swith for this AP */
			if (csa_chan_info) {
				/* Schedule CSA only when the target chanspec is different
				 * from cur chanspec
				 */
				if (ap_oper_data.iface[i].chspec != csa_chan_info->chanspec) {
					wf_chspec_ntoa(ap_oper_data.iface[i].chspec, chanspec_str1);
					wf_chspec_ntoa(csa_chan_info->chanspec, chanspec_str2);
					WL_INFORM_MEM(("add csa item, chanspec org %s(%x) -> "
						"target %s(%x)\n", chanspec_str1,
						ap_oper_data.iface[i].chspec, chanspec_str2,
						csa_chan_info->chanspec));
					ret = wl_cellavoid_add_csa_info(cellavoid_info,
						ap_oper_data.iface[i].ndev,
						csa_chan_info->chanspec);
					if (ret != BCME_OK) {
						WL_ERR(("add csa info failed\n"));
						break;
					}
				}
			} else {
				WL_INFORM_MEM(("AP %s is not allowed to work, cur chanspec %x\n",
					ap_oper_data.iface[i].ndev->name,
					ap_oper_data.iface[i].chspec));
			}
		}
	}

	if (ret != BCME_OK) {
		wl_cellavoid_free_csa_info_list(cellavoid_info);
	} else {
		if (!list_empty(&cellavoid_info->csa_info_list)) {
			WL_INFORM_MEM(("scheduling csa work, item count %d\n",
				cellavoid_info->csa_info_cnt));
			cellavoid_info->csa_progress = TRUE;
			cellavoid_info->csa_reschedule_cnt = 0;
			schedule_delayed_work(&cfg->csa_delayed_work,
				msecs_to_jiffies((const unsigned int)CSA_DELAYWORK_FIRST_INTERVAL));
		}
	}

	return ret;
}

/* Used by cfg vendor interface to verify the param is correct and update bw/chanspec band */
static int
wl_cellavoid_validate_param(struct bcm_cfg80211 *cfg, wl_cellavoid_param_t *param)
{
	int ret = BCME_OK;
	int i, param_ch;
	chanspec_band_t param_band;
	chanspec_bw_t bw;

	for (i = 0; i < param->chan_cnt; i++) {
		bw = INVALID_CHSPEC_BW;

		/* Not supported DFS band */
		if (wl_cfgscan_is_dfs_set(param->chan_param[i].band)) {
			WL_ERR(("Not supported DFS band\n"));
			ret = -EINVAL;
			goto exit;
		}

		param_ch = param->chan_param[i].center_channel;
		param_band = (param->chan_param[i].band == WIFI_BAND_BG) ?
			WL_CHANSPEC_BAND_2G : WL_CHANSPEC_BAND_5G;

		if (param_band == WL_CHANSPEC_BAND_2G) {
			if (wf_valid_20MHz_chan(param_ch, param_band)) {
				bw = WL_CHANSPEC_BW_20;
			}
		} else {
			if (wf_valid_40MHz_center_chan(param_ch, param_band)) {
				bw = WL_CHANSPEC_BW_40;
			} else if (wf_valid_80MHz_center_chan(param_ch, param_band)) {
				bw = WL_CHANSPEC_BW_80;
			} else if (wf_valid_160MHz_center_chan(param_ch, param_band)) {
				bw = WL_CHANSPEC_BW_160;
			} else if (wf_valid_20MHz_chan(param_ch, param_band)) {
				bw = WL_CHANSPEC_BW_20;
			}
		}

		if (bw == INVALID_CHSPEC_BW) {
			WL_ERR(("Not supported band %d, channel %d, pwrcap %d\n",
				param->chan_param[i].band, param->chan_param[i].center_channel,
				param->chan_param[i].pwr_cap));
		}

		param->chan_param[i].chspec_bw = bw;
		param->chan_param[i].chspec_band = param_band;
	}

exit:
	return ret;
}

static int
wl_cellavoid_save_input_params(struct bcm_cfg80211 *cfg, wl_cellavoid_param_t *param)
{
	wl_cellavoid_info_t *cellavoid_info = cfg->cellavoid_info;

	if (!cellavoid_info || !param) {
		return -EPERM;
	}

	if (cellavoid_info->csa_progress) {
		return -EBUSY;
	}

	/* Backup the input parameter,
	 * this will be used to make safe/unsafe channel list upon the country code change
	 */
	cellavoid_info->params.chan_cnt = param->chan_cnt;
	cellavoid_info->params.mandatory = param->mandatory;

	(void)memcpy_s(cellavoid_info->params.chan_param,
		CELLAVOID_MAX_CH * sizeof(*(cellavoid_info->params.chan_param)),
		param->chan_param, param->chan_cnt * sizeof(*(param->chan_param)));

	return BCME_OK;
}

static int
wl_cellavoid_set_cell_channels(struct bcm_cfg80211 *cfg, wl_cellavoid_param_t *param)
{
	wl_cellavoid_info_t *cellavoid_info = cfg->cellavoid_info;
	int ret = BCME_OK, i, j, cnt;
	int param_ch;
	chanspec_band_t param_band;
	chanspec_bw_t param_bw;
	wl_cellavoid_chan_info_t *chan_info;
	chanspec_t chspecs[WF_NUM_SIDEBANDS_160MHZ];

	WL_MEM(("%s: Enter\n", __FUNCTION__));
	if (!cellavoid_info || !param) {
		return -EPERM;
	}

	/* CSA triggered by cellular channel is on-going */
	if (cellavoid_info->csa_progress) {
		return -EBUSY;
	}

	/* Clear unsafe channel list, move back unsafe channel list to safe channel list */
	wl_cellavoid_clear_cell_chan_list(cellavoid_info);

	/* Set mandatory flag */
	cellavoid_info->mandatory_flag = param->mandatory;

	/* Check channel params and set unsafe channel list */
	for (i = 0; i < param->chan_cnt; i++) {
		param_ch = param->chan_param[i].center_channel;
		param_band = param->chan_param[i].chspec_band;
		param_bw = param->chan_param[i].chspec_bw;

		if (wf_valid_160MHz_center_chan(param_ch, param_band)) {
			WL_INFORM_MEM(("160MHz channel is not supported ch : %d band %x\n",
				param_ch, param_band));
			continue;
		}

		bzero(chspecs, sizeof(chspecs));

		ret = wl_get_all_sideband_chanspecs(param_ch, param_band, param_bw,
			chspecs, &cnt);
		if (ret != BCME_OK || cnt == 0) {
			WL_ERR(("channel is not supported ch : %d band %d\n",
				param_ch, param_band));
			continue;
		}

		for (j = 0; j < cnt; j++) {
			/* Find chanspecs in the safe channel list(avail channel list)
			 * If the chanspec exists, detach the channel item
			 * from the safe channel list(avail channel list)
			 */
			chan_info = wl_cellavoid_get_chan_info_from_avail_chan_list(cellavoid_info,
				chspecs[j]);
			if (chan_info == NULL) {
				WL_MEM(("no chan info for chanspec %x\n", chspecs[j]));
				continue;
			}

			/* If the chanspec exists, set txpwr cap for this chanspec */
			chan_info->pwr_cap = param->chan_param[i].pwr_cap;

			/* Move this chan info to the unsafe channel list(cellular channel list */
			wl_cellavoid_move_chan_info_to_cell_chan_list(cellavoid_info, chan_info);
		}
	}

	/* Sort the safe/unsafe channel list for dump */
	wl_cellavoid_sort_chan_info_list(cellavoid_info);

#ifdef WL_CELLULAR_CHAN_AVOID_DUMP
	wl_cellavoid_dump_chan_info_list(cellavoid_info);
#endif /* WL_CELLULAR_CHAN_AVOID_DUMP */

	/* Perform actions needs to be done (AP->CSA)
	 */
	ret = wl_cellavoid_handle_apsta_concurrency(cfg);
	if (ret != BCME_OK) {
		WL_ERR(("returned fail %d\n", ret));
		goto fail;
	}

	/* Deliver chanspec + pwrcap information to the FW */
	ret = wl_cellavoid_apply_txpwrcap(cfg, cellavoid_info);

	return ret;

fail:
	wl_cellavoid_clear_cell_chan_list(cellavoid_info);

	return ret;
}

static int
wl_cellavoid_update_param(struct bcm_cfg80211 *cfg, wl_cellavoid_param_t *param)
{
	int err = BCME_OK;

	wl_cellavoid_sync_lock(cfg);

	err = wl_cellavoid_save_input_params(cfg, param);
	if (err == BCME_OK) {
		err = wl_cellavoid_set_cell_channels(cfg, param);
		if (err) {
			WL_INFORM_MEM(("Failed to set cellular channel list, err:%d\n", err));
		}
	} else {
		WL_INFORM_MEM(("Failed to save input params, err:%d\n", err));
	}

	wl_cellavoid_sync_unlock(cfg);

	return err;
}

static int
wl_cellavoid_is_pwrcap_valid(struct bcm_cfg80211 *cfg,
	int32 fwk_val, int8 *pwrcap)
{
	int ret = BCME_OK;

	WL_DBG_MEM(("fwk_val:0x%x\n", fwk_val));

	if (fwk_val == CELLAVOID_NO_POWER_CAP) {
		*pwrcap = CELLAVOID_TXCAP_MAX_VAL;
	} else if ((fwk_val < CELLAVOID_TXCAP_MAX_VAL) &&
		(fwk_val >= CELLAVOID_TXCAP_MIN_VAL)) {
		*pwrcap = fwk_val;
	} else {
		WL_ERR(("unsupported value for pwr_cap:0x%x\n",
			fwk_val));
		ret = BCME_BADARG;
	}

	return ret;
}

/* cfg vendor interface function */
int
wl_cfgvendor_cellavoid_set_cell_channels(struct wiphy *wiphy,
		struct wireless_dev *wdev, const void  *data, int len)
{
	int err = BCME_OK, rem, rem1, rem2, type;
	int8 pwrcap;
	wl_cellavoid_param_t param;
	wl_cellavoid_chan_param_t* cur_chan_param = NULL;
	const struct nlattr *iter, *iter1, *iter2;
	struct bcm_cfg80211 *cfg = wiphy_priv(wiphy);

	BCM_REFERENCE(wdev);

	WL_INFORM(("%s: Enter\n", __FUNCTION__));
	bzero(&param, sizeof(param));
	if (len <= 0) {
		WL_ERR(("Length of the nlattr is not valid len : %d\n", len));
		err = -EINVAL;
		goto exit;
	}
	nla_for_each_attr(iter, data, len, rem) {
		type = nla_type(iter);
		switch (type) {
		case CELLAVOID_ATTRIBUTE_CNT:
			param.chan_cnt = nla_get_u8(iter);
			if (param.chan_cnt > CELLAVOID_MAX_CH) {
				err = -EINVAL;
				goto exit;
			}
			param.chan_param = (wl_cellavoid_chan_param_t *)MALLOCZ(cfg->osh,
					sizeof(wl_cellavoid_chan_param_t) * param.chan_cnt);
			if (param.chan_param == NULL) {
				WL_ERR(("failed to allocate target param for (%d)\n",
					param.chan_cnt));
				err = -ENOMEM;
				goto exit;
			}

			break;
		case CELLAVOID_ATTRIBUTE_MANDATORY:
			param.mandatory = nla_get_u32(iter);
			break;
		case CELLAVOID_ATTRIBUTE_CONFIG:
			if (param.chan_param == NULL) {
				WL_ERR(("chan_param is NULL (%d)\n", param.chan_cnt));
				err = -ENOMEM;
				goto exit;
			}
			cur_chan_param = param.chan_param;
			nla_for_each_nested(iter1, iter, rem1) {
				if ((uint8 *)cur_chan_param >= ((uint8 *)param.chan_param +
					sizeof(wl_cellavoid_chan_param_t) * param.chan_cnt)) {
					WL_ERR(("increased addr is over its max size\n"));
					err = -EINVAL;
					goto exit;
				}
				nla_for_each_nested(iter2, iter1, rem2) {
					type = nla_type(iter2);
					switch (type) {
						case CELLAVOID_ATTRIBUTE_BAND:
							cur_chan_param->band = nla_get_u32(iter2);
							break;
						case CELLAVOID_ATTRIBUTE_CHANNEL:
							cur_chan_param->center_channel =
								nla_get_u32(iter2);
							break;
						case CELLAVOID_ATTRIBUTE_PWRCAP:
							err = wl_cellavoid_is_pwrcap_valid(cfg,
								nla_get_u32(iter2), &pwrcap);
							if (err == BCME_OK) {
								cur_chan_param->pwr_cap = pwrcap;
							} else {
								WL_ERR(("pwr_cap is not valid\n"));
								goto exit;
							}
							break;
					}
				}
				WL_DBG_MEM(("CELLAVOID PARAM - BAND:%d CHAN:%d PWR_CAP:%d\n",
					cur_chan_param->band, cur_chan_param->center_channel,
					cur_chan_param->pwr_cap));
				cur_chan_param++;
			}
			break;
		}
	}

	WL_INFORM_MEM(("CELLAVOID PARAM - CNT:%d MANDATORY:%d\n",
		param.chan_cnt, param.mandatory));

	err = wl_cellavoid_validate_param(cfg, &param);
	if (err) {
		err = -EINVAL;
		goto exit;
	}

	err = wl_cellavoid_update_param(cfg, &param);

exit:
	/* free the config param table */
	if (param.chan_param) {
		MFREE(cfg->osh, param.chan_param,
			sizeof(wl_cellavoid_chan_param_t) * param.chan_cnt);
	}
	return err;
}
