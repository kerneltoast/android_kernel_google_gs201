/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.

 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC : contains son hdd API implementation
 */

#include <qdf_types.h>
#include <wlan_hdd_main.h>
#include <wlan_hdd_sta_info.h>
#include <wlan_hdd_regulatory.h>
#include <os_if_son.h>
#include <sap_internal.h>
#include <wma_api.h>
#include <wlan_hdd_hostapd.h>
#include <wlan_reg_services_api.h>
#include <son_ucfg_api.h>
#include <wlan_hdd_son.h>

/**
 * hdd_son_is_acs_in_progress() - whether acs is in progress or not
 * @vdev: vdev
 *
 * Return: true if acs is in progress
 */
static uint32_t hdd_son_is_acs_in_progress(struct wlan_objmgr_vdev *vdev)
{
	struct hdd_adapter *adapter;
	bool in_progress = false;

	if (!vdev) {
		hdd_err("null vdev");
		return in_progress;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return in_progress;
	}

	if (!hdd_adapter_is_ap(adapter)) {
		hdd_err("vdev id %d is not AP", adapter->vdev_id);
		return in_progress;
	}

	in_progress = qdf_atomic_read(&adapter->session.ap.acs_in_progress);

	return in_progress;
}

/**
 * hdd_son_chan_width_to_chan_width() - translate son chan width
 *                                      to mac chan width
 * @son_chwidth: son chan width
 *
 * Return: mac chan width
 */
static enum eSirMacHTChannelWidth hdd_son_chan_width_to_chan_width(
				enum ieee80211_cwm_width son_chwidth)
{
	enum eSirMacHTChannelWidth chwidth;

	switch (son_chwidth) {
	case IEEE80211_CWM_WIDTH20:
		chwidth = eHT_CHANNEL_WIDTH_20MHZ;
		break;
	case IEEE80211_CWM_WIDTH40:
		chwidth = eHT_CHANNEL_WIDTH_40MHZ;
		break;
	case IEEE80211_CWM_WIDTH80:
		chwidth = eHT_CHANNEL_WIDTH_80MHZ;
		break;
	case IEEE80211_CWM_WIDTH160:
		chwidth = eHT_CHANNEL_WIDTH_160MHZ;
		break;
	case IEEE80211_CWM_WIDTH80_80:
		chwidth = eHT_CHANNEL_WIDTH_80P80MHZ;
		break;
	default:
		chwidth = eHT_MAX_CHANNEL_WIDTH;
	}

	return chwidth;
}

/**
 * hdd_son_set_chwidth() - set son chan width
 * @vdev: vdev
 * @son_chwidth: son chan width
 *
 * Return: 0 on success, negative errno on failure
 */
static int hdd_son_set_chwidth(struct wlan_objmgr_vdev *vdev,
			       enum ieee80211_cwm_width son_chwidth)
{
	enum eSirMacHTChannelWidth chwidth;
	struct hdd_adapter *adapter;

	if (!vdev) {
		hdd_err("null vdev");
		return -EINVAL;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return -EINVAL;
	}

	chwidth = hdd_son_chan_width_to_chan_width(son_chwidth);

	return hdd_set_mac_chan_width(adapter, chwidth);
}

/**
 * hdd_chan_width_to_son_chwidth() - translate mac chan width
 *                                   to son chan width
 * @chwidth: mac chan width
 *
 * Return: son chan width
 */
static enum ieee80211_cwm_width hdd_chan_width_to_son_chwidth(
				enum eSirMacHTChannelWidth chwidth)
{
	enum ieee80211_cwm_width son_chwidth;

	switch (chwidth) {
	case eHT_CHANNEL_WIDTH_20MHZ:
		son_chwidth = IEEE80211_CWM_WIDTH20;
		break;
	case eHT_CHANNEL_WIDTH_40MHZ:
		son_chwidth = IEEE80211_CWM_WIDTH40;
		break;
	case eHT_CHANNEL_WIDTH_80MHZ:
		son_chwidth = IEEE80211_CWM_WIDTH80;
		break;
	case eHT_CHANNEL_WIDTH_160MHZ:
		son_chwidth = IEEE80211_CWM_WIDTH160;
		break;
	case eHT_CHANNEL_WIDTH_80P80MHZ:
		son_chwidth = IEEE80211_CWM_WIDTH80_80;
		break;
	default:
		son_chwidth = IEEE80211_CWM_WIDTHINVALID;
	}

	return son_chwidth;
}

/**
 * hdd_son_get_chwidth() - get chan width
 * @vdev: vdev
 *
 * Return: son chan width
 */
static enum ieee80211_cwm_width hdd_son_get_chwidth(
						struct wlan_objmgr_vdev *vdev)
{
	enum eSirMacHTChannelWidth chwidth;
	struct hdd_adapter *adapter;
	enum ieee80211_cwm_width son_chwidth = IEEE80211_CWM_WIDTHINVALID;

	if (!vdev) {
		hdd_err("null vdev");
		return son_chwidth;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return son_chwidth;
	}

	chwidth = wma_cli_get_command(adapter->vdev_id, WMI_VDEV_PARAM_CHWIDTH,
				      VDEV_CMD);

	if (chwidth < 0) {
		hdd_err("Failed to get chwidth");
		return son_chwidth;
	}

	return hdd_chan_width_to_son_chwidth(chwidth);
}

/**
 * hdd_son_chan_ext_offset_to_chan_type() - translate son chan extend offset
 *                                          to chan type
 * @son_chan_ext_offset: son chan ext offset
 *
 * Return: tSirMacHTChannelType
 */
static tSirMacHTChannelType hdd_son_chan_ext_offset_to_chan_type(
				enum sec20_chan_offset son_chan_ext_offset)
{
	tSirMacHTChannelType chan_type;

	switch (son_chan_ext_offset) {
	case EXT_CHAN_OFFSET_ABOVE:
		chan_type = eHT_CHAN_HT40PLUS;
		break;
	case EXT_CHAN_OFFSET_BELOW:
		chan_type = eHT_CHAN_HT40MINUS;
		break;
	default:
		chan_type = eHT_CHAN_HT20;
		break;
	}

	return chan_type;
}

/**
 * hdd_son_set_chan_ext_offset() - set son chan extend offset
 * @vdev: vdev
 * @son_chan_ext_offset: son chan extend offset
 *
 * Return: 0 on success, negative errno on failure
 */
static int hdd_son_set_chan_ext_offset(
				struct wlan_objmgr_vdev *vdev,
				enum sec20_chan_offset son_chan_ext_offset)
{
	enum eSirMacHTChannelType chan_type;
	QDF_STATUS status;
	int retval = -EINVAL;
	struct hdd_adapter *adapter;

	if (!vdev) {
		hdd_err("null vdev");
		return retval;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return retval;
	}
	if (!hdd_adapter_is_ap(adapter)) {
		hdd_err("vdev id %d is not AP", adapter->vdev_id);
		return retval;
	}

	retval = 0;
	chan_type = hdd_son_chan_ext_offset_to_chan_type(son_chan_ext_offset);
	status = hdd_set_sap_ht2040_mode(adapter, chan_type);
	if (status != QDF_STATUS_SUCCESS) {
		hdd_err("Cannot set SAP HT20/40 mode!");
		retval = -EINVAL;
	}

	return retval;
}

/**
 * hdd_chan_type_to_son_chan_ext_offset() - translate tSirMacHTChannelType
 *                                          to son chan extend offset
 * @chan_type: tSirMacHTChannelType
 *
 * Return: son chan extend offset
 */
static enum sec20_chan_offset hdd_chan_type_to_son_chan_ext_offset(
				tSirMacHTChannelType chan_type)
{
	enum sec20_chan_offset son_chan_ext_offset;

	switch (chan_type) {
	case eHT_CHAN_HT40PLUS:
		son_chan_ext_offset = EXT_CHAN_OFFSET_ABOVE;
		break;
	case eHT_CHAN_HT40MINUS:
		son_chan_ext_offset = EXT_CHAN_OFFSET_BELOW;
		break;
	default:
		son_chan_ext_offset = EXT_CHAN_OFFSET_NA;
		break;
	}

	return son_chan_ext_offset;
}

/**
 * hdd_son_get_chan_ext_offset() - get chan extend offset
 * @vdev: vdev
 *
 * Return: enum sec20_chan_offset
 */
static enum sec20_chan_offset hdd_son_get_chan_ext_offset(
						struct wlan_objmgr_vdev *vdev)
{
	enum eSirMacHTChannelType chan_type;
	QDF_STATUS status;
	struct hdd_adapter *adapter;

	if (!vdev) {
		hdd_err("null vdev");
		return 0;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return 0;
	}
	if (!hdd_adapter_is_ap(adapter)) {
		hdd_err("vdev id %d is not AP", adapter->vdev_id);
		return 0;
	}

	status = hdd_get_sap_ht2040_mode(adapter, &chan_type);
	if (status != QDF_STATUS_SUCCESS) {
		hdd_err("Cannot set SAP HT20/40 mode!");
		return 0;
	}

	return hdd_chan_type_to_son_chan_ext_offset(chan_type);
}

/**
 * hdd_son_bandwidth_to_phymode() - get new eCsrPhyMode according
 *                                  to son band width
 * @son_bandwidth: son band width
 * @old_phymode: old eCsrPhyMode
 * @phymode: new eCsrPhyMode to get
 *
 * Return: void
 */
static void hdd_son_bandwidth_to_phymode(uint32_t son_bandwidth,
					 eCsrPhyMode old_phymode,
					 eCsrPhyMode *phymode)
{
	*phymode = old_phymode;

	switch (son_bandwidth) {
	case HT20:
	case HT40:
		*phymode = eCSR_DOT11_MODE_11n;
		break;
	case VHT20:
	case VHT40:
	case VHT80:
	case VHT160:
	case VHT80_80:
		*phymode = eCSR_DOT11_MODE_11ac;
		break;
	case HE20:
	case HE40:
	case HE80:
	case HE160:
	case HE80_80:
		*phymode = eCSR_DOT11_MODE_11ax;
		break;
	default:
		break;
	}
}

/**
 * hdd_son_bandwidth_to_bonding_mode() - son band with to bonding mode
 * @son_bandwidth: son band width
 * @bonding_mode: bonding mode to get
 *
 * Return: void
 */
static void hdd_son_bandwidth_to_bonding_mode(uint32_t son_bandwidth,
					      uint32_t *bonding_mode)
{
	switch (son_bandwidth) {
	case HT40:
	case VHT40:
	case VHT80:
	case VHT160:
	case VHT80_80:
	case HE40:
	case HE80:
	case HE160:
	case HE80_80:
		*bonding_mode = WNI_CFG_CHANNEL_BONDING_MODE_ENABLE;
		break;
	default:
		*bonding_mode = WNI_CFG_CHANNEL_BONDING_MODE_DISABLE;
	}
}

/**
 * hdd_son_set_bandwidth() - set band width
 * @vdev: vdev
 * @son_bandwidth: son band width
 *
 * Return: 0 on success, negative errno on failure
 */
static int hdd_son_set_bandwidth(struct wlan_objmgr_vdev *vdev,
				 uint32_t son_bandwidth)
{
	eCsrPhyMode phymode;
	eCsrPhyMode old_phymode;
	uint8_t supported_band;
	uint32_t bonding_mode;
	struct hdd_adapter *adapter;
	struct hdd_context *hdd_ctx;

	if (!vdev) {
		hdd_err("null vdev");
		return -EINVAL;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return -EINVAL;
	}

	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	if (!hdd_ctx) {
		hdd_err("null hdd ctx");
		return -EINVAL;
	}
	old_phymode = sme_get_phy_mode(hdd_ctx->mac_handle);

	hdd_son_bandwidth_to_phymode(son_bandwidth, old_phymode, &phymode);

	if (wlan_reg_is_6ghz_supported(hdd_ctx->psoc))
		supported_band = REG_BAND_MASK_ALL;
	else
		supported_band = BIT(REG_BAND_2G) | BIT(REG_BAND_5G);

	hdd_son_bandwidth_to_bonding_mode(son_bandwidth, &bonding_mode);

	return hdd_update_phymode(adapter, phymode, supported_band,
				  bonding_mode);
}

/**
 * hdd_phymode_chwidth_to_son_bandwidth() - get son bandwidth from
 *                                          phymode and chwidth
 * @phymode: eCsrPhyMode
 * @chwidth: eSirMacHTChannelWidth
 *
 * Return: son bandwidth
 */
static uint32_t hdd_phymode_chwidth_to_son_bandwidth(
					eCsrPhyMode phymode,
					enum eSirMacHTChannelWidth chwidth)
{
	uint32_t son_bandwidth = NONHT;

	switch (phymode) {
	case eCSR_DOT11_MODE_abg:
	case eCSR_DOT11_MODE_11a:
	case eCSR_DOT11_MODE_11b:
	case eCSR_DOT11_MODE_11g:
	case eCSR_DOT11_MODE_11g_ONLY:
	case eCSR_DOT11_MODE_11b_ONLY:
		son_bandwidth = NONHT;
		break;
	case eCSR_DOT11_MODE_11n:
	case eCSR_DOT11_MODE_11n_ONLY:
		son_bandwidth = HT20;
		if (chwidth == eHT_CHANNEL_WIDTH_40MHZ)
			son_bandwidth = HT40;
		break;
	case eCSR_DOT11_MODE_11ac:
	case eCSR_DOT11_MODE_11ac_ONLY:
		son_bandwidth = VHT20;
		if (chwidth == eHT_CHANNEL_WIDTH_40MHZ)
			son_bandwidth = VHT40;
		else if (chwidth == eHT_CHANNEL_WIDTH_80MHZ)
			son_bandwidth = VHT80;
		else if (chwidth == eHT_CHANNEL_WIDTH_160MHZ)
			son_bandwidth = VHT160;
		else if (chwidth == eHT_CHANNEL_WIDTH_80P80MHZ)
			son_bandwidth = VHT80_80;
		break;
	case eCSR_DOT11_MODE_11ax:
	case eCSR_DOT11_MODE_11ax_ONLY:
	case eCSR_DOT11_MODE_AUTO:
		son_bandwidth = HE20;
		if (chwidth == eHT_CHANNEL_WIDTH_40MHZ)
			son_bandwidth = HE40;
		else if (chwidth == eHT_CHANNEL_WIDTH_80MHZ)
			son_bandwidth = HE80;
		else if (chwidth == eHT_CHANNEL_WIDTH_160MHZ)
			son_bandwidth = HE160;
		else if (chwidth == eHT_CHANNEL_WIDTH_80P80MHZ)
			son_bandwidth = HE80_80;
		break;
	default:
		break;
	}

	return son_bandwidth;
}

/**
 * hdd_son_get_bandwidth() - get band width
 * @vdev: vdev
 *
 * Return: band width
 */
static uint32_t hdd_son_get_bandwidth(struct wlan_objmgr_vdev *vdev)
{
	enum eSirMacHTChannelWidth chwidth;
	eCsrPhyMode phymode;
	struct hdd_adapter *adapter;
	struct hdd_context *hdd_ctx;

	if (!vdev) {
		hdd_err("null vdev");
		return NONHT;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return NONHT;
	}

	chwidth = wma_cli_get_command(adapter->vdev_id, WMI_VDEV_PARAM_CHWIDTH,
				      VDEV_CMD);

	if (chwidth < 0) {
		hdd_err("Failed to get chwidth");
		return NONHT;
	}
	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	if (!hdd_ctx) {
		hdd_err("null hdd ctx");
		return -NONHT;
	}
	phymode = sme_get_phy_mode(hdd_ctx->mac_handle);

	return hdd_phymode_chwidth_to_son_bandwidth(phymode, chwidth);
}

/**
 * hdd_son_band_to_band() - translate SON band mode to reg_wifi_band
 * @band: given enum wlan_band_id
 *
 * Return: reg_wifi_band
 */
static enum reg_wifi_band hdd_son_band_to_band(enum wlan_band_id band)
{
	enum reg_wifi_band reg_band = REG_BAND_UNKNOWN;

	switch (band) {
	case WLAN_BAND_2GHZ:
		reg_band = REG_BAND_2G;
		break;
	case WLAN_BAND_5GHZ:
		reg_band = REG_BAND_5G;
		break;
	case WLAN_BAND_6GHZ:
		reg_band = REG_BAND_6G;
		break;
	default:
		break;
	}

	return reg_band;
}

/**
 * hdd_son_set_chan() - set chan
 * @vdev: vdev
 * @chan: given chan
 * @son_band: given enum wlan_band_id
 *
 * Return: 0 on success, negative errno on failure
 */
static int hdd_son_set_chan(struct wlan_objmgr_vdev *vdev, int chan,
			    enum wlan_band_id son_band)
{
	struct hdd_adapter *adapter;
	enum reg_wifi_band band = hdd_son_band_to_band(son_band);
	bool status;
	qdf_freq_t freq;
	struct wlan_objmgr_pdev *pdev;
	struct wlan_objmgr_psoc *psoc;

	if (!vdev) {
		hdd_err("null vdev");
		return -EINVAL;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return -EINVAL;
	}
	if (!hdd_adapter_is_ap(adapter)) {
		hdd_err("vdev id %d is not AP", adapter->vdev_id);
		return -ENOTSUPP;
	}

	pdev = wlan_vdev_get_pdev(vdev);
	if (!pdev) {
		hdd_err("null pdev");
		return -EINVAL;
	}

	psoc = wlan_vdev_get_psoc(vdev);
	if (!psoc) {
		hdd_err("null psoc");
		return -EINVAL;
	}

	freq = wlan_reg_chan_band_to_freq(pdev, chan, BIT(band));
	status = policy_mgr_is_sap_allowed_on_dfs_freq(pdev, adapter->vdev_id,
						       freq);
	if (!status) {
		hdd_err("sap_allowed_on_dfs_freq check fails");
		return -EINVAL;
	}
	wlan_hdd_set_sap_csa_reason(psoc, adapter->vdev_id,
				    CSA_REASON_USER_INITIATED);

	return hdd_softap_set_channel_change(adapter->dev, freq, CH_WIDTH_MAX,
					     false);
}

/**
 * hdd_son_set_country() - set country code
 * @vdev: vdev
 * @country_code:pointer to country code
 *
 * Return: 0 if country code is set successfully
 */
static int hdd_son_set_country(struct wlan_objmgr_vdev *vdev,
			       char *country_code)
{
	struct hdd_adapter *adapter;
	struct hdd_context *hdd_ctx;

	if (!vdev) {
		hdd_err("null vdev");
		return -EINVAL;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return -EINVAL;
	}
	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	if (!hdd_ctx) {
		hdd_err("null hdd ctx");
		return -EINVAL;
	}

	return hdd_reg_set_country(hdd_ctx, country_code);
}

/**
 * hdd_son_set_candidate_freq() - set candidate freq. Switch to this freq
 *                                after radar is detected
 * @vdev: vdev
 * @freq: candidate frequency
 *
 * Return: 0 if candidate freq is set successfully.
 */
static int hdd_son_set_candidate_freq(struct wlan_objmgr_vdev *vdev,
				      qdf_freq_t freq)
{
	struct hdd_adapter *adapter;
	struct sap_context *sap_ctx;

	if (!vdev) {
		hdd_err("null vdev");
		return -EINVAL;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return -EINVAL;
	}
	if (!hdd_adapter_is_ap(adapter)) {
		hdd_err("vdev id %d is not AP", adapter->vdev_id);
		return -EINVAL;
	}

	sap_ctx = WLAN_HDD_GET_SAP_CTX_PTR(adapter);
	if (!sap_ctx) {
		hdd_err("null sap_ctx");
		return -EINVAL;
	}

	sap_ctx->candidate_freq = freq;

	return 0;
}

/**
 * hdd_son_get_candidate_freq() - get candidate freq
 * @vdev: vdev
 *
 * Return: candidate freq
 */
static qdf_freq_t hdd_son_get_candidate_freq(struct wlan_objmgr_vdev *vdev)
{
	struct hdd_adapter *adapter;
	struct sap_context *sap_ctx;
	qdf_freq_t freq = 0;

	if (!vdev) {
		hdd_err("null vdev");
		return freq;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return freq;
	}
	if (!hdd_adapter_is_ap(adapter)) {
		hdd_err("vdev id %d is not AP", adapter->vdev_id);
		return freq;
	}

	sap_ctx = WLAN_HDD_GET_SAP_CTX_PTR(adapter);
	if (!sap_ctx) {
		hdd_err("null sap_ctx");
		return freq;
	}
	freq = sap_ctx->candidate_freq;

	return freq;
}

/**
 * hdd_son_phy_mode_to_vendor_phy_mode() - translate son phy mode to
 *                                         vendor_phy_mode
 * @mode: son phy mode
 *
 * Return: qca_wlan_vendor_phy_mode
 */
static enum qca_wlan_vendor_phy_mode hdd_son_phy_mode_to_vendor_phy_mode(
						enum ieee80211_phymode mode)
{
	enum qca_wlan_vendor_phy_mode vendor_mode;

	switch (mode) {
	case IEEE80211_MODE_AUTO:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_AUTO;
		break;
	case IEEE80211_MODE_11A:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11A;
		break;
	case IEEE80211_MODE_11B:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11B;
		break;
	case IEEE80211_MODE_11G:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11G;
		break;
	case IEEE80211_MODE_11NA_HT20:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11NA_HT20;
		break;
	case IEEE80211_MODE_11NG_HT20:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11NG_HT20;
		break;
	case IEEE80211_MODE_11NA_HT40PLUS:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11NA_HT40PLUS;
		break;
	case IEEE80211_MODE_11NA_HT40MINUS:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11NA_HT40MINUS;
		break;
	case IEEE80211_MODE_11NG_HT40PLUS:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11NG_HT40PLUS;
		break;
	case IEEE80211_MODE_11NG_HT40MINUS:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11NG_HT40MINUS;
		break;
	case IEEE80211_MODE_11NG_HT40:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11NG_HT40;
		break;
	case IEEE80211_MODE_11NA_HT40:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11NA_HT40;
		break;
	case IEEE80211_MODE_11AC_VHT20:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT20;
		break;
	case IEEE80211_MODE_11AC_VHT40PLUS:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT40PLUS;
		break;
	case IEEE80211_MODE_11AC_VHT40MINUS:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT40MINUS;
		break;
	case IEEE80211_MODE_11AC_VHT40:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT40;
		break;
	case IEEE80211_MODE_11AC_VHT80:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT80;
		break;
	case IEEE80211_MODE_11AC_VHT160:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT160;
		break;
	case IEEE80211_MODE_11AC_VHT80_80:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AC_VHT80P80;
		break;
	case IEEE80211_MODE_11AXA_HE20:
	case IEEE80211_MODE_11AXG_HE20:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AX_HE20;
		break;
	case IEEE80211_MODE_11AXA_HE40PLUS:
	case IEEE80211_MODE_11AXG_HE40PLUS:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AX_HE40PLUS;
		break;
	case IEEE80211_MODE_11AXA_HE40MINUS:
	case IEEE80211_MODE_11AXG_HE40MINUS:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AX_HE40MINUS;
		break;
	case IEEE80211_MODE_11AXA_HE40:
	case IEEE80211_MODE_11AXG_HE40:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AX_HE40;
		break;
	case IEEE80211_MODE_11AXA_HE80:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AX_HE80;
		break;
	case IEEE80211_MODE_11AXA_HE160:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AX_HE160;
		break;
	case IEEE80211_MODE_11AXA_HE80_80:
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_11AX_HE80P80;
		break;
	default:
		hdd_err("Invalid config phy mode %d, set it as auto", mode);
		vendor_mode = QCA_WLAN_VENDOR_PHY_MODE_AUTO;
		break;
	}

	return vendor_mode;
}

/**
 * hdd_son_set_phymode() - set son phy mode
 * @vdev: vdev
 * @mode: son phy mode to set
 *
 * Return: 0 on success, negative errno on failure
 */
static int hdd_son_set_phymode(struct wlan_objmgr_vdev *vdev,
			       enum ieee80211_phymode mode)
{
	struct hdd_adapter *adapter;
	enum qca_wlan_vendor_phy_mode vendor_phy_mode;

	if (!vdev) {
		hdd_err("null vdev");
		return -EINVAL;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return -EINVAL;
	}

	vendor_phy_mode = hdd_son_phy_mode_to_vendor_phy_mode(mode);

	return hdd_set_phy_mode(adapter, vendor_phy_mode);
}

/**
 * hdd_phymode_chwidth_freq_to_son_phymode() - get son phymode from csr phymode
 *                                             chan width and freq
 * @phymode: csr phymode
 * @chwidth: chan width
 * @freq: chan frequence
 *
 * Return: ieee80211_phymode
 */
static enum ieee80211_phymode hdd_phymode_chwidth_freq_to_son_phymode(
					eCsrPhyMode phymode,
					enum eSirMacHTChannelWidth chwidth,
					qdf_freq_t freq)
{
	uint32_t band_2g = WLAN_REG_IS_24GHZ_CH_FREQ(freq);
	enum ieee80211_phymode son_phymode;

	switch (phymode) {
	case eCSR_DOT11_MODE_abg:
	case eCSR_DOT11_MODE_AUTO:
		son_phymode = IEEE80211_MODE_AUTO;
		break;
	case eCSR_DOT11_MODE_11a:
		son_phymode = IEEE80211_MODE_11A;
		break;
	case eCSR_DOT11_MODE_11b:
	case eCSR_DOT11_MODE_11b_ONLY:
		son_phymode = IEEE80211_MODE_11B;
		break;
	case eCSR_DOT11_MODE_11g:
	case eCSR_DOT11_MODE_11g_ONLY:
		son_phymode = IEEE80211_MODE_11G;
		break;
	case eCSR_DOT11_MODE_11n:
	case eCSR_DOT11_MODE_11n_ONLY:
		if (chwidth == eHT_CHANNEL_WIDTH_40MHZ) {
			if (band_2g)
				son_phymode = IEEE80211_MODE_11NG_HT40;
			else
				son_phymode = IEEE80211_MODE_11NA_HT40;
		} else {
			if (band_2g)
				son_phymode = IEEE80211_MODE_11NG_HT20;
			else
				son_phymode = IEEE80211_MODE_11NA_HT20;
		}
		break;
	case eCSR_DOT11_MODE_11ac:
	case eCSR_DOT11_MODE_11ac_ONLY:
		if (chwidth == eHT_CHANNEL_WIDTH_160MHZ)
			son_phymode = IEEE80211_MODE_11AC_VHT160;
		else if (chwidth == eHT_CHANNEL_WIDTH_80P80MHZ)
			son_phymode = IEEE80211_MODE_11AC_VHT80_80;
		else if (chwidth == eHT_CHANNEL_WIDTH_80MHZ)
			son_phymode = IEEE80211_MODE_11AC_VHT80;
		else if (chwidth == eHT_CHANNEL_WIDTH_40MHZ)
			son_phymode = IEEE80211_MODE_11AC_VHT40;
		else
			son_phymode = IEEE80211_MODE_11AC_VHT20;
		break;
	case eCSR_DOT11_MODE_11ax:
	case eCSR_DOT11_MODE_11ax_ONLY:
		if (chwidth == eHT_CHANNEL_WIDTH_160MHZ) {
			son_phymode = IEEE80211_MODE_11AXA_HE160;
		} else if (chwidth == eHT_CHANNEL_WIDTH_80P80MHZ) {
			son_phymode = IEEE80211_MODE_11AXA_HE80_80;
		} else if (chwidth == eHT_CHANNEL_WIDTH_80MHZ) {
			son_phymode = IEEE80211_MODE_11AXA_HE80;
		} else if (chwidth == eHT_CHANNEL_WIDTH_40MHZ) {
			if (band_2g)
				son_phymode = IEEE80211_MODE_11AXG_HE40;
			else
				son_phymode = IEEE80211_MODE_11AXA_HE40;
		} else {
			if (band_2g)
				son_phymode = IEEE80211_MODE_11AXG_HE20;
			else
				son_phymode = IEEE80211_MODE_11AXA_HE20;
		}
		break;
	default:
		son_phymode = IEEE80211_MODE_AUTO;
		break;
	}

	return son_phymode;
}

/**
 * hdd_son_get_phymode() - get son phy mode
 * @vdev: vdev
 *
 * Return: enum ieee80211_phymode
 */
static enum ieee80211_phymode hdd_son_get_phymode(struct wlan_objmgr_vdev *vdev)
{
	enum eSirMacHTChannelWidth chwidth;
	eCsrPhyMode phymode;
	struct hdd_adapter *adapter;
	qdf_freq_t freq;
	struct wlan_objmgr_pdev *pdev;
	struct hdd_context *hdd_ctx;

	if (!vdev) {
		hdd_err("null vdev");
		return IEEE80211_MODE_AUTO;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return IEEE80211_MODE_AUTO;
	}

	pdev = wlan_vdev_get_pdev(vdev);
	if (!pdev) {
		hdd_err("null pdev");
		return IEEE80211_MODE_AUTO;
	}

	freq = ucfg_son_get_operation_chan_freq_vdev_id(pdev, adapter->vdev_id);

	chwidth = wma_cli_get_command(adapter->vdev_id, WMI_VDEV_PARAM_CHWIDTH,
				      VDEV_CMD);

	if (chwidth < 0) {
		hdd_err("Failed to get chwidth");
		return IEEE80211_MODE_AUTO;
	}
	hdd_ctx = WLAN_HDD_GET_CTX(adapter);
	if (!hdd_ctx) {
		hdd_err("null hdd ctx");
		return IEEE80211_MODE_AUTO;
	}

	phymode = sme_get_phy_mode(hdd_ctx->mac_handle);

	return hdd_phymode_chwidth_freq_to_son_phymode(phymode, chwidth, freq);
}

/**
 * hdd_son_set_acl_policy() - set son acl policy
 * @vdev: vdev
 * @son_acl_policy: enum ieee80211_acl_cmd
 *
 * Return: QDF_STATUS
 */
static QDF_STATUS hdd_son_set_acl_policy(struct wlan_objmgr_vdev *vdev,
					 ieee80211_acl_cmd son_acl_policy)
{
	struct hdd_adapter *adapter;
	QDF_STATUS status = QDF_STATUS_E_INVAL;

	if (!vdev) {
		hdd_err("null vdev");
		return status;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return status;
	}

	switch (son_acl_policy) {
	case IEEE80211_MACCMD_POLICY_OPEN:
		status = wlansap_set_acl_mode(WLAN_HDD_GET_SAP_CTX_PTR(adapter),
					      eSAP_ALLOW_ALL);
		break;
	case IEEE80211_MACCMD_POLICY_ALLOW:
		status = wlansap_set_acl_mode(WLAN_HDD_GET_SAP_CTX_PTR(adapter),
					      eSAP_DENY_UNLESS_ACCEPTED);
		break;
	case IEEE80211_MACCMD_POLICY_DENY:
		status = wlansap_set_acl_mode(WLAN_HDD_GET_SAP_CTX_PTR(adapter),
					      eSAP_ACCEPT_UNLESS_DENIED);
		break;
	case IEEE80211_MACCMD_FLUSH:
	case IEEE80211_MACCMD_DETACH:
		status = wlansap_clear_acl(WLAN_HDD_GET_SAP_CTX_PTR(adapter));
		break;
	default:
		hdd_err("invalid son acl policy %d", son_acl_policy);
		break;
	}

	return status;
}

/**
 * hdd_acl_policy_to_son_acl_policy() - convert acl policy to son acl policy
 * @acl_policy: acl policy
 *
 * Return: son acl policy. enum ieee80211_acl_cmd
 */
static ieee80211_acl_cmd hdd_acl_policy_to_son_acl_policy(
						eSapMacAddrACL acl_policy)
{
	ieee80211_acl_cmd son_acl_policy = IEEE80211_MACCMD_DETACH;

	switch (acl_policy) {
	case eSAP_ACCEPT_UNLESS_DENIED:
		son_acl_policy = IEEE80211_MACCMD_POLICY_DENY;
		break;
	case eSAP_DENY_UNLESS_ACCEPTED:
		son_acl_policy = IEEE80211_MACCMD_POLICY_ALLOW;
		break;
	case eSAP_ALLOW_ALL:
		son_acl_policy = IEEE80211_MACCMD_POLICY_OPEN;
		break;
	default:
		hdd_err("invalid acl policy %d", acl_policy);
		break;
	}

	return son_acl_policy;
}

/**
 * hdd_son_get_acl_policy() - get son acl policy
 * @vdev: vdev
 *
 * Return: son acl policy. enum ieee80211_acl_cmd
 */
static ieee80211_acl_cmd hdd_son_get_acl_policy(struct wlan_objmgr_vdev *vdev)
{
	eSapMacAddrACL acl_policy;
	struct hdd_adapter *adapter;
	ieee80211_acl_cmd son_acl_policy = IEEE80211_MACCMD_DETACH;

	if (!vdev) {
		hdd_err("null vdev");
		return son_acl_policy;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return son_acl_policy;
	}

	wlansap_get_acl_mode(WLAN_HDD_GET_SAP_CTX_PTR(adapter), &acl_policy);

	son_acl_policy = hdd_acl_policy_to_son_acl_policy(acl_policy);

	return son_acl_policy;
}

/**
 * hdd_son_add_acl_mac() - add mac to access control list(ACL)
 * @vdev: vdev
 * @acl_mac: mac address to add
 *
 * Return: 0 on success, negative errno on failure
 */
static int hdd_son_add_acl_mac(struct wlan_objmgr_vdev *vdev,
			       struct qdf_mac_addr *acl_mac)
{
	eSapACLType list_type;
	QDF_STATUS qdf_status;
	eSapMacAddrACL acl_policy;
	struct hdd_adapter *adapter;

	if (!vdev) {
		hdd_err("null vdev");
		return -EINVAL;
	}
	if (!acl_mac) {
		hdd_err("null acl_mac");
		return -EINVAL;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return -EINVAL;
	}

	wlansap_get_acl_mode(WLAN_HDD_GET_SAP_CTX_PTR(adapter), &acl_policy);

	if (acl_policy == eSAP_ACCEPT_UNLESS_DENIED) {
		list_type = eSAP_BLACK_LIST;
	} else if (acl_policy == eSAP_DENY_UNLESS_ACCEPTED) {
		list_type = eSAP_WHITE_LIST;
	} else {
		hdd_err("Invalid ACL policy %d.", acl_policy);
		return -EINVAL;
	}
	qdf_status = wlansap_modify_acl(WLAN_HDD_GET_SAP_CTX_PTR(adapter),
					acl_mac->bytes, list_type,
					ADD_STA_TO_ACL);
	if (QDF_IS_STATUS_ERROR(qdf_status)) {
		hdd_err("Modify ACL failed");
		return -EIO;
	}

	return 0;
}

/**
 * hdd_son_del_acl_mac() - delete mac from acl
 * @vdev: vdev
 * @acl_mac: mac to remove
 *
 * Return: 0 on success, negative errno on failure
 */
static int hdd_son_del_acl_mac(struct wlan_objmgr_vdev *vdev,
			       struct qdf_mac_addr *acl_mac)
{
	eSapACLType list_type;
	QDF_STATUS qdf_status;
	eSapMacAddrACL acl_policy;
	struct hdd_adapter *adapter;
	struct sap_context *sap_ctx;

	if (!vdev) {
		hdd_err("null vdev");
		return -EINVAL;
	}
	if (!acl_mac) {
		hdd_err("null acl_mac");
		return -EINVAL;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return -EINVAL;
	}

	sap_ctx = WLAN_HDD_GET_SAP_CTX_PTR(adapter);
	if (!sap_ctx) {
		hdd_err("null sap ctx");
		return -EINVAL;
	}

	wlansap_get_acl_mode(sap_ctx, &acl_policy);

	if (acl_policy == eSAP_ACCEPT_UNLESS_DENIED) {
		list_type = eSAP_BLACK_LIST;
	} else if (acl_policy == eSAP_DENY_UNLESS_ACCEPTED) {
		list_type = eSAP_WHITE_LIST;
	} else {
		hdd_err("Invalid ACL policy %d.", acl_policy);
		return -EINVAL;
	}
	qdf_status = wlansap_modify_acl(sap_ctx, acl_mac->bytes, list_type,
					DELETE_STA_FROM_ACL);
	if (QDF_IS_STATUS_ERROR(qdf_status)) {
		hdd_err("Modify ACL failed");
		return -EIO;
	}

	return 0;
}

/**
 * hdd_son_kickout_mac() - kickout sta with given mac
 * @vdev: vdev
 * @acl_mac: sta mac to kickout
 *
 * Return: 0 on success, negative errno on failure
 */
static int hdd_son_kickout_mac(struct wlan_objmgr_vdev *vdev,
			       struct qdf_mac_addr *mac)
{
	struct hdd_adapter *adapter;

	if (!vdev) {
		hdd_err("null vdev");
		return -EINVAL;
	}
	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return -EINVAL;
	}

	if (mac)
		return wlan_hdd_del_station(adapter, mac->bytes);
	else
		return wlan_hdd_del_station(adapter, NULL);
}

static uint8_t hdd_son_get_rx_nss(struct wlan_objmgr_vdev *vdev)
{
	struct hdd_adapter *adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	uint8_t rx_nss = 0;

	if (!adapter) {
		hdd_err("null adapter");
		return 0;
	}
	hdd_get_rx_nss(adapter, &rx_nss);

	return rx_nss;
}

static void hdd_son_deauth_sta(struct wlan_objmgr_vdev *vdev,
			       uint8_t *peer_mac,
			       bool ignore_frame)
{
	struct hdd_adapter *adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	struct csr_del_sta_params param;

	if (!adapter) {
		hdd_err("null adapter");
		return;
	}

	qdf_mem_copy(param.peerMacAddr.bytes, peer_mac, QDF_MAC_ADDR_SIZE);
	param.subtype = SIR_MAC_MGMT_DEAUTH;
	param.reason_code = ignore_frame ? REASON_HOST_TRIGGERED_SILENT_DEAUTH
					 : REASON_UNSPEC_FAILURE;
	hdd_debug("Peer - "QDF_MAC_ADDR_FMT" Ignore Frame - %u",
		  QDF_FULL_MAC_REF(peer_mac), ignore_frame);

	if (hdd_softap_sta_deauth(adapter, &param) != QDF_STATUS_SUCCESS)
		hdd_err("Error in deauthenticating peer");
}

static void hdd_son_modify_acl(struct wlan_objmgr_vdev *vdev,
			       uint8_t *peer_mac,
			       bool allow_auth)
{
	QDF_STATUS status;
	struct hdd_adapter *adapter = wlan_hdd_get_adapter_from_objmgr(vdev);

	if (!adapter) {
		hdd_err("null adapter");
		return;
	}
	hdd_debug("Peer - " QDF_MAC_ADDR_FMT " Allow Auth - %u",
		  QDF_MAC_ADDR_REF(peer_mac), allow_auth);
	if (allow_auth) {
		status = wlansap_modify_acl(WLAN_HDD_GET_SAP_CTX_PTR(adapter),
					    peer_mac,
					    eSAP_BLACK_LIST,
					    DELETE_STA_FROM_ACL);
		status = wlansap_modify_acl(WLAN_HDD_GET_SAP_CTX_PTR(adapter),
					    peer_mac,
					    eSAP_WHITE_LIST,
					    ADD_STA_TO_ACL);
	} else {
		status = wlansap_modify_acl(WLAN_HDD_GET_SAP_CTX_PTR(adapter),
					    peer_mac,
					    eSAP_WHITE_LIST,
					    DELETE_STA_FROM_ACL);
		status = wlansap_modify_acl(WLAN_HDD_GET_SAP_CTX_PTR(adapter),
					    peer_mac,
					    eSAP_BLACK_LIST,
					    ADD_STA_TO_ACL);
	}
}

static int hdd_son_send_cfg_event(struct wlan_objmgr_vdev *vdev,
				  uint32_t event_id,
				  uint32_t event_len,
				  const uint8_t *event_buf)
{
	struct hdd_adapter *adapter;
	uint32_t len;
	uint32_t idx;
	struct sk_buff *skb;

	if (!event_buf) {
		hdd_err("invalid event buf");
		return -EINVAL;
	}

	adapter = wlan_hdd_get_adapter_from_objmgr(vdev);
	if (!adapter) {
		hdd_err("null adapter");
		return -EINVAL;
	}

	len = nla_total_size(sizeof(event_id)) +
			nla_total_size(event_len) +
			NLMSG_HDRLEN;
	idx = QCA_NL80211_VENDOR_SUBCMD_GET_WIFI_CONFIGURATION_INDEX;
	skb = cfg80211_vendor_event_alloc(adapter->hdd_ctx->wiphy,
					  &adapter->wdev,
					  len,
					  idx,
					  GFP_KERNEL);
	if (!skb) {
		hdd_err("failed to alloc cfg80211 vendor event");
		return -EINVAL;
	}

	if (nla_put_u32(skb,
			QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_COMMAND,
			event_id)) {
		hdd_err("failed to put attr config generic command");
		kfree_skb(skb);
		return -EINVAL;
	}

	if (nla_put(skb,
		    QCA_WLAN_VENDOR_ATTR_CONFIG_GENERIC_DATA,
		    event_len,
		    event_buf)) {
		hdd_err("failed to put attr config generic data");
		kfree_skb(skb);
		return -EINVAL;
	}

	cfg80211_vendor_event(skb, GFP_KERNEL);

	return 0;
}

static int hdd_son_deliver_opmode(struct wlan_objmgr_vdev *vdev,
				  uint32_t event_len,
				  const uint8_t *event_buf)
{
	return hdd_son_send_cfg_event(vdev,
				      QCA_NL80211_VENDOR_SUBCMD_OPMODE_UPDATE,
				      event_len,
				      event_buf);
}

static int hdd_son_deliver_smps(struct wlan_objmgr_vdev *vdev,
				uint32_t event_len,
				const uint8_t *event_buf)
{
	return hdd_son_send_cfg_event(vdev,
				      QCA_NL80211_VENDOR_SUBCMD_SMPS_UPDATE,
				      event_len,
				      event_buf);
}

void hdd_son_register_callbacks(struct hdd_context *hdd_ctx)
{
	struct son_callbacks cb_obj = {0};

	cb_obj.os_if_is_acs_in_progress = hdd_son_is_acs_in_progress;
	cb_obj.os_if_set_chan_ext_offset = hdd_son_set_chan_ext_offset;
	cb_obj.os_if_get_chan_ext_offset = hdd_son_get_chan_ext_offset;
	cb_obj.os_if_set_bandwidth = hdd_son_set_bandwidth;
	cb_obj.os_if_get_bandwidth = hdd_son_get_bandwidth;
	cb_obj.os_if_set_chan = hdd_son_set_chan;
	cb_obj.os_if_set_acl_policy = hdd_son_set_acl_policy;
	cb_obj.os_if_get_acl_policy = hdd_son_get_acl_policy;
	cb_obj.os_if_add_acl_mac = hdd_son_add_acl_mac;
	cb_obj.os_if_del_acl_mac = hdd_son_del_acl_mac;
	cb_obj.os_if_kickout_mac = hdd_son_kickout_mac;
	cb_obj.os_if_set_country_code = hdd_son_set_country;
	cb_obj.os_if_set_candidate_freq = hdd_son_set_candidate_freq;
	cb_obj.os_if_get_candidate_freq = hdd_son_get_candidate_freq;
	cb_obj.os_if_set_phymode = hdd_son_set_phymode;
	cb_obj.os_if_get_phymode = hdd_son_get_phymode;
	cb_obj.os_if_get_rx_nss = hdd_son_get_rx_nss;
	cb_obj.os_if_set_chwidth = hdd_son_set_chwidth;
	cb_obj.os_if_get_chwidth = hdd_son_get_chwidth;
	cb_obj.os_if_deauth_sta = hdd_son_deauth_sta;
	cb_obj.os_if_modify_acl = hdd_son_modify_acl;

	os_if_son_register_hdd_callbacks(hdd_ctx->psoc, &cb_obj);

	ucfg_son_register_deliver_opmode_cb(hdd_ctx->psoc,
					    hdd_son_deliver_opmode);
	ucfg_son_register_deliver_smps_cb(hdd_ctx->psoc,
					  hdd_son_deliver_smps);
}
