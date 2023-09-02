/*
 * This file is part of the UWB stack for linux.
 *
 * Copyright (c) 2020-2021 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo. Please contact Qorvo to inquire about licensing terms.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <net/mcps802154.h>

MODULE_AUTHOR("Saad Zouiten <saad.zouiten@qorvo.com>");
MODULE_DESCRIPTION("stubbed 802.15.4 mcps driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");

static const int g_time_interval = 1000;
static struct timer_list g_timer;
static struct mcps802154_llhw *driver_llhw;
static bool rx_enabled;
static bool tx_queued;
static bool started;
static bool stop_timer;
static bool scanning;
static u8 seq;
static u8 curchan;

const char *const calib_strings[] = { "calib.param1", NULL };
static u8 calib;

static void periodic_task(struct timer_list *unused)
{
	if (stop_timer)
		return;
	pr_info("fake_mcps: %s called\n", __func__);
	/*Restarting the timer...*/
	mod_timer(&g_timer, jiffies + msecs_to_jiffies(g_time_interval));
	if (rx_enabled) {
		rx_enabled = false;
		mcps802154_rx_frame(driver_llhw);
	}
	if (tx_queued) {
		tx_queued = false;
		mcps802154_tx_done(driver_llhw);
	}
}

static int start(struct mcps802154_llhw *llhw)
{
	pr_info("fake_mcps: %s called\n", __func__);
	started = true;
	return 0;
}

static void stop(struct mcps802154_llhw *llhw)
{
	pr_info("fake_mcps: %s called\n", __func__);
	started = false;
}

static int tx_frame(struct mcps802154_llhw *llhw, struct sk_buff *skb,
		    const struct mcps802154_tx_frame_config *config,
		    int frame_idx, int next_delay_dtu)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: tx_frame called skb len=%d\n", skb->len);
	print_hex_dump(KERN_CONT, " ", DUMP_PREFIX_OFFSET, 16, 1, skb->data,
		       skb->len, false);
	tx_queued = true;
	return 0;
}

static int rx_enable(struct mcps802154_llhw *llhw,
		     const struct mcps802154_rx_frame_config *info,
		     int frame_idx, int next_delay_dtu)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	rx_enabled = true;
	return 0;
}

static int rx_disable(struct mcps802154_llhw *llhw)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	rx_enabled = false;
	return 0;
}

static struct sk_buff *create_fake_data_frame(void)
{
	unsigned char *data;
	struct sk_buff *skb = dev_alloc_skb(20 + 2);

	if (!skb)
		return NULL;
	data = skb_put(skb, 20);
	data[0] = 0x21; /* Frame Control Field */
	data[1] = 0xc8; /* Frame Control Field */
	data[2] = 0x8b; /* Sequence number */
	data[3] = 0xff; /* Destination PAN ID 0xffff */
	data[4] = 0xff; /* Destination PAN ID */
	data[5] = 0x02; /* Destination short address 0x0002 */
	data[6] = 0x00; /* Destination short address */
	data[7] = 0x23; /* Source PAN ID 0x0023 */
	data[8] = 0x00; /* */
	data[9] = 0x60; /* Source extended address ae:c2:4a:1c:21:16:e2:60 */
	data[10] = 0xe2; /* */
	data[11] = 0x16; /* */
	data[12] = 0x21; /* */
	data[13] = 0x1c; /* */
	data[14] = 0x4a; /* */
	data[15] = 0xc2; /* */
	data[16] = 0xae; /* */
	data[17] = 0xaa; /* Payload */
	data[18] = 0xbb; /* */
	data[19] = 0xcc; /* */
	return skb;
}

static struct sk_buff *create_fake_beacon_frame(void)
{
	unsigned char *data;
	struct sk_buff *skb = dev_alloc_skb(17 + 2);

	if (!skb)
		return NULL;
	data = skb_put(skb, 17);
	data[0] = 0x00; /* Frame Control Field: 0b00000000 */
	data[1] = 0xd0; /* Frame Control Field: 0b11010000 */
	data[2] = seq; /* Sequence number */
	data[3] = curchan; /* Source PAN ID 0x0100 + channel */
	data[4] = 0x01; /* */
	data[5] = curchan; /* Source extended address ae:c2:4a:1c:21:16:e2:xx */
	data[6] = 0xe2; /* */
	data[7] = 0x16; /* */
	data[8] = 0x21; /* */
	data[9] = 0x1c; /* */
	data[10] = 0x4a; /* */
	data[11] = 0xc2; /* */
	data[12] = 0xae; /* */
	data[13] = 0x77; /* Superframe spec */
	data[14] = 0xcf; /* */
	data[15] = 0x00; /* GTS spec */
	data[16] = 0x00; /* Pending address spec */
	/* TODO: CFP Alloc */
	/* TODO: CFP Usage */
	/* Update seq for next beacon */
	seq = (seq + 1) & 0x3f;
	return skb;
}

static int rx_get_frame(struct mcps802154_llhw *llhw, struct sk_buff **skb,
			struct mcps802154_rx_frame_info *info)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	if (scanning)
		*skb = create_fake_beacon_frame();
	else
		*skb = create_fake_data_frame();
	if (!*skb) {
		pr_err("RX buffer allocation failed\n");
		return -ENOMEM;
	}
	return 0;
}

static int rx_get_error_frame(struct mcps802154_llhw *llhw,
			      struct mcps802154_rx_frame_info *info)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	return 0;
}

static int idle(struct mcps802154_llhw *llhw, bool timestamp, u32 timestamp_dtu)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	return 0;
}

static int reset(struct mcps802154_llhw *llhw)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	return 0;
}

static int get_current_timestamp_dtu(struct mcps802154_llhw *llhw,
				     u32 *timestamp_dtu)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	*timestamp_dtu = 0;
	return 0;
}

static u64 tx_timestamp_dtu_to_rmarker_rctu(
	struct mcps802154_llhw *llhw, u32 tx_timestamp_dtu,
	const struct mcps802154_hrp_uwb_params *hrp_uwb_params,
	const struct mcps802154_channel *channel_params, int ant_set_id)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	return 0;
}

static s64 difference_timestamp_rctu(struct mcps802154_llhw *llhw,
				     u64 timestamp_a_rctu, u64 timestamp_b_rctu)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	return 0;
}

static int compute_frame_duration_dtu(struct mcps802154_llhw *llhw,
				      int payload_bytes)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	return 0;
}

static int set_channel(struct mcps802154_llhw *llhw, u8 page, u8 channel,
		       u8 preamble_code)
{
	pr_info("fake_mcps: %s called\n", __func__);
	seq = 0; /* reset beacon sequence number */
	curchan = channel; /* save current channel */
	return 0;
}

static int set_hrp_uwb_params(struct mcps802154_llhw *llhw,
			      const struct mcps802154_hrp_uwb_params *params)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	return 0;
}

static int set_hw_addr_filt(struct mcps802154_llhw *llhw,
			    struct ieee802154_hw_addr_filt *filt,
			    unsigned long changed)
{
	if (changed & IEEE802154_AFILT_SADDR_CHANGED) {
		pr_info("fake_mcps: new short addr=%x", filt->short_addr);
	}
	if (changed & IEEE802154_AFILT_IEEEADDR_CHANGED) {
		pr_info("fake_mcps: new extended addr=%llx", filt->ieee_addr);
	}
	if (changed & IEEE802154_AFILT_PANID_CHANGED) {
		pr_info("fake_mcps: new pan id=%x", filt->pan_id);
	}
	if (changed & IEEE802154_AFILT_PANC_CHANGED) {
		pr_info("fake_mcps: new pan coordinator=%x", filt->pan_coord);
	}
	return 0;
}

static int set_txpower(struct mcps802154_llhw *llhw, s32 mbm)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	return 0;
}

static int set_cca_mode(struct mcps802154_llhw *llhw,
			const struct wpan_phy_cca *cca)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	return 0;
}

static int set_cca_ed_level(struct mcps802154_llhw *llhw, s32 mbm)
{
	if (!started) {
		pr_err("fake_mcps: %s called and not started\n", __func__);
		return -EIO;
	}
	pr_info("fake_mcps: %s called\n", __func__);
	return 0;
}

static int set_promiscuous_mode(struct mcps802154_llhw *llhw, bool on)
{
	pr_info("fake_mcps: %s called on=%d\n", __func__, on);
	return 0;
}

static int set_scanning_mode(struct mcps802154_llhw *llhw, bool mode)
{
	pr_info("fake_mcps: %s called\n", __func__);
	scanning = mode;
	return 0;
}

static int set_calibration(struct mcps802154_llhw *llhw, const char *key,
			   void *value, size_t length)
{
	pr_info("fake_mcps: %s called\n", __func__);
	if (!key || !value || length != 1)
		return -EINVAL;
	if (strcmp(key, calib_strings[0]) == 0)
		calib = *(u8 *)value;
	else
		return -ENOENT;
	return 0;
}

static int get_calibration(struct mcps802154_llhw *llhw, const char *key,
			   void *value, size_t length)
{
	pr_info("fake_mcps: %s called\n", __func__);
	if (!key || !value || length < 1)
		return -EINVAL;
	if (strcmp(key, calib_strings[0]) == 0)
		*(u8 *)value = calib;
	else
		return -ENOENT;
	return 1;
}

static const char *const *list_calibration(struct mcps802154_llhw *llhw)
{
	pr_info("fake_mcps: %s called\n", __func__);
	return calib_strings;
}

static int vendor_cmd(struct mcps802154_llhw *llhw, u32 vendor_id, u32 subcmd,
		      void *data, size_t data_len)
{
	pr_info("fake_mcps: %s called\n", __func__);
	return 0;
}

static const struct mcps802154_ops fake_ops = {
	.start = start,
	.stop = stop,
	.tx_frame = tx_frame,
	.rx_enable = rx_enable,
	.rx_disable = rx_disable,
	.rx_get_frame = rx_get_frame,
	.rx_get_error_frame = rx_get_error_frame,
	.idle = idle,
	.reset = reset,
	.get_current_timestamp_dtu = get_current_timestamp_dtu,
	.tx_timestamp_dtu_to_rmarker_rctu = tx_timestamp_dtu_to_rmarker_rctu,
	.difference_timestamp_rctu = difference_timestamp_rctu,
	.compute_frame_duration_dtu = compute_frame_duration_dtu,
	.set_channel = set_channel,
	.set_hrp_uwb_params = set_hrp_uwb_params,
	.set_hw_addr_filt = set_hw_addr_filt,
	.set_txpower = set_txpower,
	.set_cca_mode = set_cca_mode,
	.set_cca_ed_level = set_cca_ed_level,
	.set_promiscuous_mode = set_promiscuous_mode,
	.set_scanning_mode = set_scanning_mode,
	.set_calibration = set_calibration,
	.get_calibration = get_calibration,
	.list_calibration = list_calibration,
	.vendor_cmd = vendor_cmd,
};

static int __init fake_init(void)
{
	int r;

	pr_info("fake_mcps: init\n");
	driver_llhw = mcps802154_alloc_llhw(0, &fake_ops);
	if (driver_llhw == NULL) {
		return -ENOMEM;
	}
	driver_llhw->hw->flags =
		(IEEE802154_HW_TX_OMIT_CKSUM | IEEE802154_HW_AFILT |
		 IEEE802154_HW_PROMISCUOUS | IEEE802154_HW_RX_OMIT_CKSUM);
	driver_llhw->flags =
		(MCPS802154_LLHW_BPRF | MCPS802154_LLHW_DATA_RATE_6M81 |
		 MCPS802154_LLHW_PHR_DATA_RATE_850K |
		 MCPS802154_LLHW_PHR_DATA_RATE_6M81 | MCPS802154_LLHW_PRF_16 |
		 MCPS802154_LLHW_PRF_64 | MCPS802154_LLHW_PSR_32 |
		 MCPS802154_LLHW_PSR_64 | MCPS802154_LLHW_PSR_128 |
		 MCPS802154_LLHW_PSR_256 | MCPS802154_LLHW_PSR_1024 |
		 MCPS802154_LLHW_PSR_4096 | MCPS802154_LLHW_SFD_4A |
		 MCPS802154_LLHW_SFD_4Z_8 | MCPS802154_LLHW_STS_SEGMENT_1 |
		 MCPS802154_LLHW_AOA_AZIMUTH | MCPS802154_LLHW_AOA_ELEVATION |
		 MCPS802154_LLHW_AOA_FOM);
	/* UWB High band 802.15.4a-2007. */
	driver_llhw->hw->phy->supported.channels[4] |= 0xffe0;

	/* UWB symbol duration at PRF16 & PRF64 is ~1us */
	driver_llhw->hw->phy->symbol_duration = 1;

	/* Set extended address. */
	driver_llhw->hw->phy->perm_extended_addr = 0xd6552cd6e41ceb57;

	/* fake driver phy channel 5 as default */
	driver_llhw->hw->phy->current_page = 4;
	driver_llhw->hw->phy->current_channel = 5;
	driver_llhw->current_preamble_code = 9;

	r = mcps802154_register_llhw(driver_llhw);
	if (r) {
		mcps802154_free_llhw(driver_llhw);
		driver_llhw = NULL;
		return r;
	}

	/*Starting the periodic task.*/
	timer_setup(&g_timer, periodic_task, 0);
	mod_timer(&g_timer, jiffies + msecs_to_jiffies(g_time_interval));
	return 0;
}

static void __exit fake_exit(void)
{
	pr_info("fake_mcps: Exit\n");
	mcps802154_unregister_llhw(driver_llhw);
	mcps802154_free_llhw(driver_llhw);
	driver_llhw = NULL;
	/*Deleting the timer aka the periodic task.*/
	stop_timer = true;
	del_timer(&g_timer);
}

module_init(fake_init);
module_exit(fake_exit);
