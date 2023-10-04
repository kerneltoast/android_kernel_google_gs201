#include <kunit/test.h>

/* Forward declaration of replacement functions. */
static u64 kunit_get_boottime_ns(void);

/* Replace ktime_get_boottime_ns calls to kunit_get_boottime_ns calls. */
#define ktime_get_boottime_ns kunit_get_boottime_ns
#define dw3000_get_dtu_time kunit_dw3000_get_dtu_time
#define trace_dw3000_power_stats(dw, state, boot_time_ns, len_or_date) \
	do {                                                           \
	} while (0)

/* Include tested functions. */
#include "dw3000_core.h"
#include "dw3000_power_stats.h"

/* Keep sync with the same tables in dw3000_core.c */
const struct dw3000_plen_info _plen_info[] = {
	{
		.symb = 64,
		.pac_symb = 8,
		.dw_reg = DW3000_PLEN_64,
		.dw_pac_reg = DW3000_PAC8,
	},
	{
		.symb = 1024,
		.pac_symb = 32,
		.dw_reg = DW3000_PLEN_1024,
		.dw_pac_reg = DW3000_PAC32,
	},
	{
		.symb = 4096,
		.pac_symb = 64,
		.dw_reg = DW3000_PLEN_4096,
		.dw_pac_reg = DW3000_PAC32,
	},
	{
		.symb = 32,
		.pac_symb = 8,
		.dw_reg = DW3000_PLEN_32,
		.dw_pac_reg = DW3000_PAC8,
	},
	{
		.symb = 128,
		.pac_symb = 8,
		.dw_reg = DW3000_PLEN_128,
		.dw_pac_reg = DW3000_PAC8,
	},
	{
		.symb = 1536,
		.pac_symb = 64,
		.dw_reg = DW3000_PLEN_1536,
		.dw_pac_reg = DW3000_PAC32,
	},
	{
		.symb = 72,
		.pac_symb = 8,
		.dw_reg = DW3000_PLEN_72,
		.dw_pac_reg = DW3000_PAC8,
	},
	{
		/* Invalid */
		.symb = 0,
		.pac_symb = 0,
		.dw_reg = 0,
		.dw_pac_reg = 0,
	},
	{
		.symb = 256,
		.pac_symb = 16,
		.dw_reg = DW3000_PLEN_256,
		.dw_pac_reg = DW3000_PAC16,
	},
	{
		.symb = 2048,
		.pac_symb = 64,
		.dw_reg = DW3000_PLEN_2048,
		.dw_pac_reg = DW3000_PAC32,
	},
	{
		/* Invalid */
		.symb = 0,
		.pac_symb = 0,
		.dw_reg = 0,
		.dw_pac_reg = 0,
	},
	{
		/* Invalid */
		.symb = 0,
		.pac_symb = 0,
		.dw_reg = 0,
		.dw_pac_reg = 0,
	},
	{
		.symb = 512,
		.pac_symb = 16,
		.dw_reg = DW3000_PLEN_512,
		.dw_pac_reg = DW3000_PAC16,
	},
};

/* Chip per symbol for 850kbps (512) and 6.8Mbps (64) */
const int _chip_per_symbol_info[2] = { 512, 64 };

const struct dw3000_prf_info _prf_info[] = {
	{
		/* Invalid PRF */
		0,
	},
	{
		/* 16 MHz */
		.chip_per_symb = 496,
	},
	{
		/* 64 MHz */
		.chip_per_symb = 508,
	},
};

/* Static variable declarations */
static u64 kunit_boot_time_ns;
static u32 kunit_dtu_time;

/**
 * kunit_get_boottime_ns() - ktime_get_boottime_ns replacement for tests
 *
 * Returns: boot time value in ns which increment by 1e6 for each call.
 */
static u64 kunit_get_boottime_ns(void)
{
	kunit_boot_time_ns += 1000000;
	return kunit_boot_time_ns;
}

/**
 * kunit_dw3000_get_dtu_time() - dw3000_get_dtu_time kunit wrapper
 * @dw: the DW device
 *
 * Return: The current simulated DTU time.
 */
u32 kunit_dw3000_get_dtu_time(struct dw3000 *dw)
{
	return kunit_dtu_time;
}

/* Define the test cases. */

static void dw3000_ktime_to_dtu_test_basic(struct kunit *test)
{
	struct dw3000 *dw = kunit_kzalloc(test, sizeof(*dw), GFP_KERNEL);
	/* Ensure allocation succeeded. */
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dw);
	/* Tests with time_zero_ns == 0 */
	KUNIT_EXPECT_EQ(test, 0u, dw3000_ktime_to_dtu(dw, 0));
	KUNIT_EXPECT_EQ(test, 15u, dw3000_ktime_to_dtu(dw, 1000));
	KUNIT_EXPECT_EQ(test, 156u, dw3000_ktime_to_dtu(dw, 10000));
	KUNIT_EXPECT_EQ(test, 312u, dw3000_ktime_to_dtu(dw, 20000));
	/* Tests with time_zero_ns == 1000000 */
	dw->time_zero_ns = 1000000;
	KUNIT_EXPECT_EQ(test, 0u, dw3000_ktime_to_dtu(dw, 1000000));
	KUNIT_EXPECT_EQ(test, 15u, dw3000_ktime_to_dtu(dw, 1001000));
	KUNIT_EXPECT_EQ(test, (u32)-15600, dw3000_ktime_to_dtu(dw, 0));
	KUNIT_EXPECT_EQ(test, (u32)-15584, dw3000_ktime_to_dtu(dw, 1000));
}

static void dw3000_dtu_to_ktime_test_basic(struct kunit *test)
{
	struct dw3000 *dw = kunit_kzalloc(test, sizeof(*dw), GFP_KERNEL);
	/* Ensure allocation succeeded. */
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dw);
	/* Tests with time_zero_ns == 0 */
	KUNIT_EXPECT_EQ(test, 0ll, dw3000_dtu_to_ktime(dw, 0));
	KUNIT_EXPECT_EQ(test, 64102ll, dw3000_dtu_to_ktime(dw, 1000));
	/* Tests with time_zero_ns == 1000000 */
	dw->time_zero_ns = 1000000;
	KUNIT_EXPECT_EQ(test, 1000000ll, dw3000_dtu_to_ktime(dw, 0));
	KUNIT_EXPECT_EQ(test, 1064102ll, dw3000_dtu_to_ktime(dw, 1000));
}

static void dw3000_dtu_to_sys_time_test_basic(struct kunit *test)
{
	struct dw3000 *dw = kunit_kzalloc(test, sizeof(*dw), GFP_KERNEL);
	/* Ensure allocation succeeded. */
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dw);
	/* Tests with dtu_sync == 0 & sys_time_sync == 0 */
	KUNIT_EXPECT_EQ(test, 0u, dw3000_dtu_to_sys_time(dw, 0));
	KUNIT_EXPECT_EQ(test, 1u << 4, dw3000_dtu_to_sys_time(dw, 1));
	/* Tests with dtu_sync == 10000 & sys_time_sync == 0 */
	dw->dtu_sync = 10000;
	KUNIT_EXPECT_EQ(test, 0u, dw3000_dtu_to_sys_time(dw, 10000));
	KUNIT_EXPECT_EQ(test, 1u << 4, dw3000_dtu_to_sys_time(dw, 10001));
	/* Tests with dtu_sync == 0 & sys_time_sync == 0 */
	dw->sys_time_sync = 1000000;
	KUNIT_EXPECT_EQ(test, 1000000u, dw3000_dtu_to_sys_time(dw, 10000));
	KUNIT_EXPECT_EQ(test, 1000016u, dw3000_dtu_to_sys_time(dw, 10001));
}

static void dw3000_sys_time_to_dtu_test_basic(struct kunit *test)
{
	u32 dtu_near = 0;
	struct dw3000 *dw = kunit_kzalloc(test, sizeof(*dw), GFP_KERNEL);
	/* Ensure allocation succeeded. */
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dw);
	/* Tests with dtu_sync == 0 & sys_time_sync == 0 */
	KUNIT_EXPECT_EQ(test, 0u, dw3000_sys_time_to_dtu(dw, 0, dtu_near));
	KUNIT_EXPECT_EQ(test, 1u, dw3000_sys_time_to_dtu(dw, 1 << 4, dtu_near));
	/* Tests with dtu_near == 10000 */
	dtu_near = 10000;
	KUNIT_EXPECT_EQ(test, 1005u,
			dw3000_sys_time_to_dtu(dw, 1005 << 4, dtu_near));
	/* Tests with dtu_sync == 1000000/16 & sys_time_sync == 1000000 */
	dw->sys_time_sync = 1000000;
	dw->dtu_sync = 1000000 >> 4;
	KUNIT_EXPECT_EQ(test, 10000u,
			dw3000_sys_time_to_dtu(dw, 10000 << 4, dtu_near));
	/* Tests with real values from traces
	   wakeup: dtu_near: 0x773a2f1, dtu_sync: 0x852a9ff, sys_time_sync: 0x293a6
	   rx_enable: timestamp_dtu=0x085776e8
	   read_rx_timestamp: value 1336512593 -> 0x4fa99051 -> sys_time 0x13ea64
	   get_rx_frame: timestamp_dtu=0x185776e6 timestamp_rctu=0xffffffffc3fee418
	*/
	dtu_near = 0x773a2f1u;
	dw->dtu_sync = 0x852a9ffu;
	dw->sys_time_sync = 0x293a6u;
	KUNIT_EXPECT_EQ(test, 0x853bf6au,
			dw3000_sys_time_to_dtu(dw, 0x13ea64u, dtu_near));
}

static void dw3000_sys_time_rctu_to_dtu_test_basic(struct kunit *test)
{
	struct dw3000 *dw = kunit_kzalloc(test, sizeof(*dw), GFP_KERNEL);

	/* Ensure allocation succeeded. */
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dw);

	/* Tests with dtu_sync == 0, sys_time_sync == 0 and dtu_near == 0.
	 * Set kunit_dtu_time to DW3000_DTU_FREQ to get dtu_near == 0 in
	 * dw3000_sys_time_rctu_to_dtu(). */
	kunit_dtu_time = DW3000_DTU_FREQ;
	KUNIT_EXPECT_EQ(test, 0u, dw3000_sys_time_rctu_to_dtu(dw, 0));
	KUNIT_EXPECT_EQ(test, 1u,
			dw3000_sys_time_rctu_to_dtu(dw, DW3000_RCTU_PER_DTU));

	/* Tests with dtu_near == 10000.
	 * Set kunit_dtu_time to DW3000_DTU_FREQ + 10000 to get dtu_near == 10000
	 * in dw3000_sys_time_rctu_to_dtu(). */
	kunit_dtu_time = DW3000_DTU_FREQ + 10000;
	KUNIT_EXPECT_EQ(
		test, 1005u,
		dw3000_sys_time_rctu_to_dtu(dw, 1005 * DW3000_RCTU_PER_DTU));

	/* Tests with dtu_sync == 1000000/16 & sys_time_sync == 1000000 */
	dw->sys_time_sync = 1000000;
	dw->dtu_sync = 1000000 >> 4;
	KUNIT_EXPECT_EQ(
		test, 10000u,
		dw3000_sys_time_rctu_to_dtu(dw, 10000 * DW3000_RCTU_PER_DTU));

	/* Tests with real values from traces:
	 * timestamp_rctu: 63852263355
	 * dtu_sync: 13025
	 * sys_time_sync: 5414
	 * dtu_near: 4349
	 * dw3000_sys_time_rctu_to_dtu() return: 15601618
	 *
	 * Set kunit_dtu_time to DW3000_DTU_FREQ + 4349 to get dtu_near == 4349
	 * in dw3000_sys_time_rctu_to_dtu(). */
	kunit_dtu_time = DW3000_DTU_FREQ + 4349;
	dw->dtu_sync = 13025;
	dw->sys_time_sync = 5414;
	KUNIT_EXPECT_EQ(test, 15601618u,
			dw3000_sys_time_rctu_to_dtu(dw, 63852263355));
}

static void power_stats_test_setup(struct dw3000 *dw)
{
	struct dw3000_power *pwr = &dw->power;

	kunit_boot_time_ns = 0;
	pwr->stats[DW3000_PWR_OFF].count = 1;
	pwr->start_time = ktime_get_boottime_ns();
	dw->config.txPreambLength = DW3000_PLEN_64;
	dw->config.txCode = 9;
	dw->config.sfdType = DW3000_SFD_TYPE_4Z;
	dw3000_update_timings(dw);
}

static void dw3000_power_stats_test_basic(struct kunit *test)
{
	struct mcps802154_llhw *llhw =
		kunit_kzalloc(test, sizeof(*llhw), GFP_KERNEL);
	struct dw3000 *dw = kunit_kzalloc(test, sizeof(*dw), GFP_KERNEL);
	struct dw3000_power *pwr = &dw->power;
	u64 incr = 1000000;
	/* Ensure allocation succeeded and good state. */
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, llhw);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dw);
	if (!dw || !llhw)
		return;
	dw->llhw = llhw;
	power_stats_test_setup(dw);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_OFF, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, incr, pwr->start_time);

	/* Simple changes
	 * - RUN - DEEPSLEEP - RUN - DEEPSLEEP - RUN - OFF - RUN - OFF
	 * - cur_state change to new state immediately
	 * - stats[X].count change immediately
	 * - stats[X].dur change when another new state Y apply
	 * - each call update time by 1e6 increment thank to ktime_get_boottime_ns()
	 *   replacement function.
	 */
	dw3000_power_stats(dw, DW3000_PWR_RUN, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_RUN, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, incr * 1, pwr->stats[DW3000_PWR_OFF].dur);
	dw3000_power_stats(dw, DW3000_PWR_DEEPSLEEP, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_DEEPSLEEP, pwr->cur_state);
	dw3000_power_stats(dw, DW3000_PWR_RUN, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_RUN, pwr->cur_state);
	dw3000_power_stats(dw, DW3000_PWR_DEEPSLEEP, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_DEEPSLEEP, pwr->cur_state);
	dw3000_power_stats(dw, DW3000_PWR_RUN, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_RUN, pwr->cur_state);
	dw3000_power_stats(dw, DW3000_PWR_OFF, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_OFF, pwr->cur_state);
	dw3000_power_stats(dw, DW3000_PWR_RUN, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_RUN, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, incr * 2, pwr->stats[DW3000_PWR_OFF].dur);
	dw3000_power_stats(dw, DW3000_PWR_OFF, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_OFF, pwr->cur_state);
	dw3000_power_stats(dw, DW3000_PWR_OFF, 0);
	KUNIT_EXPECT_EQ(test, incr * 3, pwr->stats[DW3000_PWR_OFF].dur);
	dw3000_power_stats(dw, DW3000_PWR_OFF, 0);
	/* Final state checks */
	KUNIT_EXPECT_EQ(test, 3ull, pwr->stats[DW3000_PWR_OFF].count);
	KUNIT_EXPECT_EQ(test, 2ull, pwr->stats[DW3000_PWR_DEEPSLEEP].count);
	KUNIT_EXPECT_EQ(test, 4ull, pwr->stats[DW3000_PWR_RUN].count);
	KUNIT_EXPECT_EQ(test, incr * 4, pwr->stats[DW3000_PWR_OFF].dur);
	KUNIT_EXPECT_EQ(test, incr * 2, pwr->stats[DW3000_PWR_DEEPSLEEP].dur);
	KUNIT_EXPECT_EQ(test, incr * 4, pwr->stats[DW3000_PWR_RUN].dur);
}

static void dw3000_power_stats_test_tx(struct kunit *test)
{
	struct mcps802154_llhw *llhw =
		kunit_kzalloc(test, sizeof(*llhw), GFP_KERNEL);
	struct dw3000 *dw = kunit_kzalloc(test, sizeof(*dw), GFP_KERNEL);
	struct dw3000_power *pwr = &dw->power;
	u64 incr = 1000000;
	u64 txdur = 0;

	/* Ensure allocation succeeded and good state. */
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, llhw);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dw);
	if (!dw || !llhw)
		return;
	dw->llhw = llhw;
	power_stats_test_setup(dw);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_OFF, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, incr, pwr->start_time);

	/* TX case
	 * - OFF - RUN - TX - IDLE - TX - IDLE - TX - IDLE - TX - IDLE - OFF
	 * - stats[TX].dur is in DTU, which is a 15.6MHz clock
	 * - since purpose is to test d3000_power_stats() and not
	 *   dw3000_frame_duration_dtu(), tx_adjust is summed to test
	 *   resulting stats[TX].dur.
	 */
	KUNIT_EXPECT_EQ(test, 0ull, pwr->stats[DW3000_PWR_TX].count);
	KUNIT_EXPECT_EQ(test, 0ull, pwr->stats[DW3000_PWR_TX].dur);
	dw3000_power_stats(dw, DW3000_PWR_RUN, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_RUN, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, 1ull, pwr->stats[DW3000_PWR_RUN].count);

	dw3000_power_stats(dw, DW3000_PWR_TX, 0);
	txdur += pwr->tx_adjust; /* sum calculated frame duration */
	KUNIT_EXPECT_EQ(test, DW3000_PWR_TX, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, 1ull, pwr->stats[DW3000_PWR_TX].count);
	dw3000_power_stats(dw, DW3000_PWR_IDLE, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_IDLE, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, txdur, pwr->stats[DW3000_PWR_TX].dur);

	dw3000_power_stats(dw, DW3000_PWR_TX, 16);
	txdur += pwr->tx_adjust; /* sum calculated frame duration */
	KUNIT_EXPECT_EQ(test, DW3000_PWR_TX, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, 2ull, pwr->stats[DW3000_PWR_TX].count);
	dw3000_power_stats(dw, DW3000_PWR_IDLE, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_IDLE, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, txdur, pwr->stats[DW3000_PWR_TX].dur);

	dw3000_power_stats(dw, DW3000_PWR_TX, 32);
	txdur += pwr->tx_adjust; /* sum calculated frame duration */
	KUNIT_EXPECT_EQ(test, DW3000_PWR_TX, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, 3ull, pwr->stats[DW3000_PWR_TX].count);
	dw3000_power_stats(dw, DW3000_PWR_IDLE, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_IDLE, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, txdur, pwr->stats[DW3000_PWR_TX].dur);

	dw3000_power_stats(dw, DW3000_PWR_TX, 127);
	txdur += pwr->tx_adjust; /* sum calculated frame duration */
	KUNIT_EXPECT_EQ(test, DW3000_PWR_TX, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, 4ull, pwr->stats[DW3000_PWR_TX].count);
	dw3000_power_stats(dw, DW3000_PWR_IDLE, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_IDLE, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, txdur, pwr->stats[DW3000_PWR_TX].dur);

	dw3000_power_stats(dw, DW3000_PWR_OFF, 0);
	dw3000_power_stats(dw, DW3000_PWR_OFF, 0);
	/* Final state checks */
	KUNIT_EXPECT_EQ(test, DW3000_PWR_OFF, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, 1ull, pwr->stats[DW3000_PWR_RUN].count);
	KUNIT_EXPECT_EQ(test, 5ull, pwr->stats[DW3000_PWR_IDLE].count);
	KUNIT_EXPECT_EQ(test, 0ull, pwr->stats[DW3000_PWR_RX].count);
	KUNIT_EXPECT_EQ(test, 2ull, pwr->stats[DW3000_PWR_OFF].count);
	KUNIT_EXPECT_EQ(test, 2 * incr, pwr->stats[DW3000_PWR_OFF].dur);
	KUNIT_EXPECT_EQ(test, 9 * incr, pwr->stats[DW3000_PWR_RUN].dur);
	KUNIT_EXPECT_EQ(test, 0ull, pwr->stats[DW3000_PWR_IDLE].dur);
	KUNIT_EXPECT_EQ(test, 0ull, pwr->stats[DW3000_PWR_RX].dur);
}

static void dw3000_power_stats_test_rx(struct kunit *test)
{
	struct mcps802154_llhw *llhw =
		kunit_kzalloc(test, sizeof(*llhw), GFP_KERNEL);
	struct dw3000 *dw = kunit_kzalloc(test, sizeof(*dw), GFP_KERNEL);
	struct dw3000_power *pwr = &dw->power;
	u64 incr = 1000000; /* in ns first */
	u64 rxdur = 0;

	/* Ensure allocation succeeded and good state. */
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, llhw);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dw);
	if (!dw || !llhw)
		return;
	dw->llhw = llhw;
	power_stats_test_setup(dw);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_OFF, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, incr, pwr->start_time);

	/* RX case
	 * - OFF - RUN - RX - IDLE - RX - IDLE - RX - IDLE - RX - IDLE - OFF
	 * - stats[RX].dur is in DTU, which is a 15.6MHz clock
	 * - we check here stats[RX].dur is well updated according give DTU dates
	 */
	incr = 15600; /* those checks are in DTU */
	KUNIT_EXPECT_EQ(test, 0ull, pwr->stats[DW3000_PWR_RX].count);
	KUNIT_EXPECT_EQ(test, 0ull, pwr->stats[DW3000_PWR_RX].dur);
	dw3000_power_stats(dw, DW3000_PWR_RUN, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_RUN, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, 1ull, pwr->stats[DW3000_PWR_RUN].count);

	dw3000_power_stats(dw, DW3000_PWR_RX, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_RX, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, 1ull, pwr->stats[DW3000_PWR_RX].count);
	dw3000_power_stats(dw, DW3000_PWR_IDLE, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_IDLE, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, incr, pwr->stats[DW3000_PWR_RX].dur);

	dw3000_power_stats(dw, DW3000_PWR_RX, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_RX, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, 2ull, pwr->stats[DW3000_PWR_RX].count);
	dw3000_power_stats(dw, DW3000_PWR_IDLE, 0);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_IDLE, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, 2 * incr, pwr->stats[DW3000_PWR_RX].dur);

	rxdur = pwr->stats[DW3000_PWR_RX].dur;
	dw3000_power_stats(dw, DW3000_PWR_RX, (int)0x10000);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_RX, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, 3ull, pwr->stats[DW3000_PWR_RX].count);
	dw3000_power_stats(dw, DW3000_PWR_IDLE, (int)0x20000u);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_IDLE, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, rxdur + 0x10000ull,
			pwr->stats[DW3000_PWR_RX].dur);

	rxdur = pwr->stats[DW3000_PWR_RX].dur;
	dw3000_power_stats(dw, DW3000_PWR_RX, (int)-128);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_RX, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, 4ull, pwr->stats[DW3000_PWR_RX].count);
	dw3000_power_stats(dw, DW3000_PWR_IDLE, (int)128);
	KUNIT_EXPECT_EQ(test, DW3000_PWR_IDLE, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, rxdur + 256ull, pwr->stats[DW3000_PWR_RX].dur);

	dw3000_power_stats(dw, DW3000_PWR_OFF, 0);
	dw3000_power_stats(dw, DW3000_PWR_OFF, 0);
	/* Final state checks */
	incr = 1000000; /* those checks are in ns */
	KUNIT_EXPECT_EQ(test, DW3000_PWR_OFF, pwr->cur_state);
	KUNIT_EXPECT_EQ(test, 1ull, pwr->stats[DW3000_PWR_RUN].count);
	KUNIT_EXPECT_EQ(test, 5ull, pwr->stats[DW3000_PWR_IDLE].count);
	KUNIT_EXPECT_EQ(test, 0ull, pwr->stats[DW3000_PWR_TX].count);
	KUNIT_EXPECT_EQ(test, 2ull, pwr->stats[DW3000_PWR_OFF].count);
	KUNIT_EXPECT_EQ(test, 2 * incr, pwr->stats[DW3000_PWR_OFF].dur);
	KUNIT_EXPECT_EQ(test, 9 * incr, pwr->stats[DW3000_PWR_RUN].dur);
	KUNIT_EXPECT_EQ(test, 0ull, pwr->stats[DW3000_PWR_IDLE].dur);
	KUNIT_EXPECT_EQ(test, 0ull, pwr->stats[DW3000_PWR_TX].dur);
}

static struct kunit_case dw3000_core_test_cases[] = {
	KUNIT_CASE(dw3000_ktime_to_dtu_test_basic),
	KUNIT_CASE(dw3000_dtu_to_ktime_test_basic),
	KUNIT_CASE(dw3000_dtu_to_sys_time_test_basic),
	KUNIT_CASE(dw3000_sys_time_to_dtu_test_basic),
	KUNIT_CASE(dw3000_sys_time_rctu_to_dtu_test_basic),
	KUNIT_CASE(dw3000_power_stats_test_basic),
	KUNIT_CASE(dw3000_power_stats_test_tx),
	KUNIT_CASE(dw3000_power_stats_test_rx),
	{}
};

static struct kunit_suite dw3000_core_test_suite = {
	.name = "dw3000-core",
	.test_cases = dw3000_core_test_cases,
};
kunit_test_suite(dw3000_core_test_suite);

MODULE_LICENSE("GPL v2");
