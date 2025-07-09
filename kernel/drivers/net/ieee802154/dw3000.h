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
#ifndef __DW3000_H
#define __DW3000_H

#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/skbuff.h>
#include <net/mcps802154.h>
#include <linux/pm_qos.h>
#include <linux/regulator/consumer.h>
#include "dw3000_chip.h"
#include "dw3000_stm.h"
#include "dw3000_calib.h"
#include "dw3000_testmode_nl.h"
#include "dw3000_nfcc_coex.h"
#include "dw3000_pctt.h"
#include "dw3000_debugfs.h"

#undef BIT_MASK
#ifndef DEBUG
#define DEBUG 0
#endif

#define LOG(format, arg...)                                             \
	do {                                                            \
		pr_info("dw3000: %s(): " format "\n", __func__, ##arg); \
	} while (0)

extern unsigned dw3000_regulator_delay_us;

/* Defined constants when SPI CRC mode is used */
enum dw3000_spi_crc_mode {
	DW3000_SPI_CRC_MODE_NO = 0, /* No CRC */
	DW3000_SPI_CRC_MODE_WR, /* This is used to enable SPI CRC check (the SPI
				   CRC check will be enabled on DW3000 and CRC-8
				   added for SPI write transactions) */
	DW3000_SPI_CRC_MODE_WRRD /* This is used to optionally enable additional
				    CRC check on the SPI read operations, while
				    the CRC check on the SPI write operations is
				    also enabled */
};

/* ISR data */
struct dw3000_isr_data {
	u64 status; /* initial value of register as ISR is entered */
	u16 datalength; /* length of frame */
	u64 ts_rctu; /* frame timestamp in RCTU unit */
	u8 dss_stat; /* value of the dual-SPI semaphore events */
	u8 rx_flags; /* RX frame flags, see dw3000_rx_flags */
};

/* Time units and conversion factor */
#define DW3000_DTU_PER_SYS_POWER 4

#define DW3000_CHIP_FREQ 499200000
#define DW3000_CHIP_PER_SYS 2
#define DW3000_CHIP_PER_DTU \
	(DW3000_CHIP_PER_SYS * (1 << DW3000_DTU_PER_SYS_POWER))
#define DW3000_CHIP_PER_DLY 512
#define DW3000_RCTU_PER_CHIP 128
#define DW3000_RCTU_PER_DTU (DW3000_RCTU_PER_CHIP * DW3000_CHIP_PER_DTU)
#define DW3000_RCTU_PER_SYS (DW3000_RCTU_PER_CHIP * DW3000_CHIP_PER_SYS)
#define DW3000_RCTU_PER_DLY (DW3000_CHIP_PER_DLY / DW3000_RCTU_PER_CHIP)

#define DW3000_DTU_FREQ (DW3000_CHIP_FREQ / DW3000_CHIP_PER_DTU)

/* 6.9.1.5 in 4z, for HRP UWB PHY:
   416 chips = 416 / (499.2 * 10^6) ~= 833.33 ns */
#define DW3000_DTU_PER_RSTU (416 / DW3000_CHIP_PER_DTU)
#define DW3000_DTU_PER_DLY (DW3000_CHIP_PER_DLY / DW3000_CHIP_PER_DTU)
#define DW3000_SYS_PER_DLY (DW3000_CHIP_PER_DLY / DW3000_CHIP_PER_SYS)

#define DW3000_ANTICIP_DTU (16 * (DW3000_DTU_FREQ / 1000))

#define DTU_TO_US(x) (int)((s64)(x)*1000000 / DW3000_DTU_FREQ)
#define US_TO_DTU(x) (u32)((s64)(x)*DW3000_DTU_FREQ / 1000000)
#define NS_TO_DTU(x) (u32)((s64)(x) * (DW3000_DTU_FREQ / 100000) / 10000)

#define DW3000_RX_ENABLE_STARTUP_DLY 16
#define DW3000_RX_ENABLE_STARTUP_DTU                          \
	(DW3000_RX_ENABLE_STARTUP_DLY * DW3000_CHIP_PER_DLY / \
	 DW3000_CHIP_PER_DTU)

/* UWB High band 802.15.4a-2007. Only channels 5 & 9 for DW3000. */
#define DW3000_SUPPORTED_CHANNELS ((1 << 5) | (1 << 9))

/* Number of symbols of accumulation to wait before checking
   for an SFD pattern, when Ipatov len > 64 */
#define DW3000_RX_SFD_HLDOFF 0x20
/* Default number of symbols of accumulation to wait before checking
   for an SFD pattern */
#define DW3000_RX_SFD_HLDOFF_DEF 0x14

/**
 * typedef dw3000_wakeup_done_cb - Wake up done handler.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
typedef int (*dw3000_wakeup_done_cb)(struct dw3000 *dw);

/**
 * typedef dw3000_idle_timeout_cb - Idle timeout handler.
 * @dw: Driver context.
 *
 * Return: 0 on success, else an error.
 */
typedef int (*dw3000_idle_timeout_cb)(struct dw3000 *dw);

/**
 * struct dw3000_otp_data - data read from OTP memory of DW3000 device
 * @partID: device part ID
 * @lotID: device lot ID
 * @ldo_tune_lo: tuned value used by chip specific prog_ldo_and_bias_tune
 * @ldo_tune_hi: tuned value used by chip specific prog_ldo_and_bias_tune
 * @bias_tune: tuned value used by chip specific prog_ldo_and_bias_tune
 * @dgc_addr: dgc_addr value used by chip specific prog_ldo_and_bias_tune
 * @xtal_trim: tuned value used by dw3000_prog_xtrim
 * @vBatP: vBatP
 * @tempP: tempP
 * @rev: OTP revision
 * @mode: Saved last OTP read mode to avoid multiple read
 * @pll_coarse_code: PLL coarse code use by chip->prog_pll_coarse_code
 */
struct dw3000_otp_data {
	u32 partID;
	u32 lotID;
	u32 ldo_tune_lo;
	u32 ldo_tune_hi;
	u32 bias_tune;
	u32 dgc_addr;
	u8 xtal_trim;
	u8 vBatP;
	u8 tempP;
	u8 rev;
	int mode;
	u32 pll_coarse_code;
};

/**
 * enum dw3000_ciagdiag_reg_select - CIA diagnostic register selector config.
 * @DW3000_CIA_DIAG_REG_SELECT_WITHOUT_STS: without STS
 * @DW3000_CIA_DIAG_REG_SELECT_WITH_STS: with STS
 * @DW3000_CIA_DIAG_REG_SELECT_WITH_PDAO_M3: PDOA mode 3
 *
 * According to DW3000's configuration, we must read some values
 * (e.g: channel impulse response power, preamble accumulation count)
 * in different registers in the CIA interface.
 */
enum dw3000_ciagdiag_reg_select {
	DW3000_CIA_DIAG_REG_SELECT_WITHOUT_STS = 0,
	DW3000_CIA_DIAG_REG_SELECT_WITH_STS = 1,
	DW3000_CIA_DIAG_REG_SELECT_WITH_PDAO_M3 = 2,
};

/**
 * struct dw3000_local_data - Local data and register cache
 * @spicrc: current SPI crc mode
 * @ciadiag_reg_select: CIA diagnostic register selector according to DW3000's
 *  config
 * @ciadiag_enabled: CIA diagnostic on/off
 * @dgc_otp_set: true if DGC info is programmed into OTP
 * @check_cfo: true when CFO checking is required for next RX frame
 * @ack_time: Auto ack turnaroud time
 * @dblbuffon: double buffering mode
 * @xtal_bias: XTAL trimming adjustment value
 * @max_frames_len: current configured max frame length
 * @sleep_mode: bitfield for work to be done on wakeup
 * @ststhreshold: current computed STS threshold
 * @tx_fctrl: Transmit frame control
 * @rx_timeout_pac: Preamble detection timeout period in units of PAC size
 *  symbols
 * @rx_frame_timeout_dly: reception frame timeout period in units of dly.
 * @w4r_time: Wait-for-response time (RX after TX delay)
 * @sts_key: STS Key
 * @sts_iv: STS IV
 */
struct dw3000_local_data {
	enum dw3000_spi_crc_mode spicrc;
	enum dw3000_ciagdiag_reg_select ciadiag_reg_select;
	bool ciadiag_enabled : 1;
	bool dgc_otp_set : 1;
	bool check_cfo : 1;
	u8 dblbuffon;
	u8 ack_time;
	s8 xtal_bias;
	u16 sleep_mode;
	u16 max_frames_len;
	s16 ststhreshold;
	u16 rx_timeout_pac;
	u32 rx_frame_timeout_dly;
	u32 tx_fctrl;
	u32 w4r_time;
	u8 sts_key[AES_KEYSIZE_128];
	u8 sts_iv[AES_BLOCK_SIZE];
};

/* Statistics items */
enum dw3000_stats_items {
	DW3000_STATS_RX_GOOD,
	DW3000_STATS_RX_ERROR,
	DW3000_STATS_RX_TO,
	__DW3000_STATS_COUNT
};

/**
 * struct dw3000_stats - DW3000 statistics
 * @count: Count per-items
 * @rssi: Last RSSI data per type
 * @enabled: Stats enabled flag
 * @indexes: Free-running index per-items
 */
struct dw3000_stats {
	/* Total stats */
	u16 count[__DW3000_STATS_COUNT];
	/* Required data array for calculation of the RSSI average */
	struct dw3000_rssi rssi[DW3000_RSSI_REPORTS_MAX];
	/* Stats on/off */
	bool enabled;
	u16 indexes[__DW3000_STATS_COUNT];
};

/* Maximum skb length
 *
 * Maximum supported frame size minus the checksum.
 */
#define DW3000_MAX_SKB_LEN (IEEE802154_MAX_SIFS_FRAME_SIZE - IEEE802154_FCS_LEN)

/* Additional information on rx. */
enum dw3000_rx_flags {
	/* Set if an automatic ack is send. */
	DW3000_RX_FLAG_AACK = BIT(0),
	/* Set if no data. */
	DW3000_RX_FLAG_ND = BIT(1),
	/* Set if timestamp known. */
	DW3000_RX_FLAG_TS = BIT(2),
	/* Ranging bit */
	DW3000_RX_FLAG_RNG = BIT(3),
	/* CIA done */
	DW3000_RX_FLAG_CIA = BIT(4),
	/* CIA error */
	DW3000_RX_FLAG_CER = BIT(5),
	/* STS error */
	DW3000_RX_FLAG_CPER = BIT(6)
};

/* Receive descriptor */
struct dw3000_rx {
	/* Receive lock */
	spinlock_t lock;
	/* Socket buffer */
	struct sk_buff *skb;
	/* Frame timestamp */
	u64 ts_rctu;
	/* Additional information on rx. See dw3000_rx_flags. */
	u8 flags;
	/* Busy flag */
	u8 busy;
};

/* DW3000 STS length field of the CP_CFG register (unit of 8 symbols bloc) */
enum dw3000_sts_lengths {
	DW3000_STS_LEN_8 = 0,
	DW3000_STS_LEN_16 = 1,
	DW3000_STS_LEN_32 = 2,
	DW3000_STS_LEN_64 = 3,
	DW3000_STS_LEN_128 = 4,
	DW3000_STS_LEN_256 = 5,
	DW3000_STS_LEN_512 = 6,
	DW3000_STS_LEN_1024 = 7,
	DW3000_STS_LEN_2048 = 8
};

/* DW3000 power supply structure */
struct dw3000_power_control {
	/* Power supply  generic */
	struct regulator *regulator_vdd;
	/* Power supply  1.8V */
	struct regulator *regulator_1p8;
	/* Power supply  2.5V */
	struct regulator *regulator_2p5;
};

/**
 * struct dw3000_config - Structure holding current device configuration
 * @chan: Channel number (5 or 9)
 * @txPreambLength: DW3000_PLEN_64..DW3000_PLEN_4096
 * @txCode: TX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
 * @rxCode: RX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
 * @sfdType: SFD type (0 for short IEEE 8b standard, 1 for DW 8b, 2 for DW 16b, 2 for 4z BPRF)
 * @dataRate: Data rate {DW3000_BR_850K or DW3000_BR_6M8}
 * @phrMode: PHR mode {0x0 - standard DW3000_PHRMODE_STD, 0x3 - extended frames DW3000_PHRMODE_EXT}
 * @phrRate: PHR rate {0x0 - standard DW3000_PHRRATE_STD, 0x1 - at datarate DW3000_PHRRATE_DTA}
 * @sfdTO: SFD timeout value (in symbols)
 * @stsMode: STS mode (no STS, STS before PHR or STS after data)
 * @stsLength: STS length (see enum dw3000_sts_lengths)
 * @pdoaMode: PDOA mode
 * @ant: Antennas currently connected to RF1 & RF2 ports respectively
 * @pdoaOffset: Calibrated PDOA offset
 * @pdoaLut: Pointer to calibrated PDOA to AoA look-up table
 * @rmarkerOffset: Calibrated rmarker offset
 * @promisc: Promiscuous mode enabled?
 * @alternate_pulse_shape: set alternate pulse shape
 * @hw_addr_filt: HW filter configuration
 */
struct dw3000_config {
	u8 chan;
	u8 txPreambLength;
	u8 txCode;
	u8 rxCode;
	u8 sfdType;
	u8 dataRate;
	u8 phrMode;
	u8 phrRate;
	u16 sfdTO;
	u8 stsMode;
	enum dw3000_sts_lengths stsLength;
	u8 pdoaMode;
	s8 ant[2];
	s16 pdoaOffset;
	const dw3000_pdoa_lut_t *pdoaLut;
	u32 rmarkerOffset;
	bool promisc;
	bool alternate_pulse_shape;
	struct ieee802154_hw_addr_filt hw_addr_filt;
};

/**
 * struct dw3000_txconfig - Structure holding current TX configuration
 * @PGdly: PG delay
 * @PGcount: PG count
 * @power: TX power for 1ms frame
 *  31:24     TX_CP_PWR
 *  23:16     TX_SHR_PWR
 *  15:8      TX_PHR_PWR
 *  7:0       TX_DATA_PWR
 * @smart: TX smart power enabled flag
 * @testmode_enabled: Normal or test mode
 */
struct dw3000_txconfig {
	u8 PGdly;
	u8 PGcount;
	u32 power;
	bool smart;
	bool testmode_enabled;
};

enum operational_state {
	DW3000_OP_STATE_OFF = 0,
	DW3000_OP_STATE_DEEP_SLEEP,
	DW3000_OP_STATE_SLEEP,
	DW3000_OP_STATE_WAKE_UP,
	DW3000_OP_STATE_INIT_RC,
	DW3000_OP_STATE_IDLE_RC,
	DW3000_OP_STATE_IDLE_PLL,
	DW3000_OP_STATE_TX_WAIT,
	DW3000_OP_STATE_TX,
	DW3000_OP_STATE_RX_WAIT,
	DW3000_OP_STATE_RX,
	DW3000_OP_STATE_MAX,
};

/**
 * struct sysfs_power_stats - DW3000 device power related data
 * @dur: accumulated duration in selected state in ns except for RX/TX where
 *  duration is in DTU
 * @count: number of time this state was active
 */
struct sysfs_power_stats {
	u64 dur;
	u64 count;
};

/**
 * enum power_state - DW3000 device current power state
 * @DW3000_PWR_OFF: DW3000 is OFF (unpowered or in reset)
 * @DW3000_PWR_DEEPSLEEP: DW3000 is ACTIVE (started) but in DEEP SLEEP
 * @DW3000_PWR_RUN: DW3000 is ACTIVE (include RX/TX state below)
 * @DW3000_PWR_IDLE: DW3000 is ACTIVE but IDLE (only count is used for it)
 * @DW3000_PWR_RX: DW3000 is currently RECEIVING
 * @DW3000_PWR_TX: DW3000 is currently TRANSMITTING
 * @DW3000_PWR_MAX: Number of power states stored in struct dw3000_power
 */
enum power_state {
	DW3000_PWR_OFF = 0,
	DW3000_PWR_DEEPSLEEP,
	DW3000_PWR_RUN,
	DW3000_PWR_IDLE,
	DW3000_PWR_RX,
	DW3000_PWR_TX,
	DW3000_PWR_MAX,
};

/**
 * enum config_changed_flags - values for configuration changed bitfield
 * @DW3000_AFILT_SADDR_CHANGED: same as IEEE802154_AFILT_SADDR_CHANGED
 * @DW3000_AFILT_IEEEADDR_CHANGED: same as IEEE802154_AFILT_IEEEADDR_CHANGED
 * @DW3000_AFILT_PANID_CHANGED: same as IEEE802154_AFILT_PANID_CHANGED
 * @DW3000_AFILT_PANC_CHANGED: same as IEEE802154_AFILT_PANC_CHANGED
 * @DW3000_CHANNEL_CHANGED: channel configuration changed
 * @DW3000_PCODE_CHANGED: preamble code configuration changed
 * @DW3000_PREAMBLE_LENGTH_CHANGED: length of preamble changed
 * @DW3000_SFD_CHANGED: Start Frame Delimiter changed
 * @DW3000_PHR_RATE_CHANGED: Physical HeadeR rate changed
 * @DW3000_DATA_RATE_CHANGED: Data rate changed
 */
enum config_changed_flags {
	DW3000_AFILT_SADDR_CHANGED = IEEE802154_AFILT_SADDR_CHANGED,
	DW3000_AFILT_IEEEADDR_CHANGED = IEEE802154_AFILT_IEEEADDR_CHANGED,
	DW3000_AFILT_PANID_CHANGED = IEEE802154_AFILT_PANID_CHANGED,
	DW3000_AFILT_PANC_CHANGED = IEEE802154_AFILT_PANC_CHANGED,
	DW3000_CHANNEL_CHANGED = BIT(4),
	DW3000_PCODE_CHANGED = BIT(5),
	DW3000_PREAMBLE_LENGTH_CHANGED = BIT(6),
	DW3000_SFD_CHANGED = BIT(7),
	DW3000_PHR_RATE_CHANGED = BIT(8),
	DW3000_DATA_RATE_CHANGED = BIT(9),
};

/**
 * struct dw3000_power - DW3000 device power related data
 * @stats: accumulated statistics defined by struct sysfs_power_stats
 * @start_time: timestamp of current state start
 * @cur_state: current state defined by enum power_state
 * @tx_adjust: TX time adjustment based on frame length
 * @rx_start: RX start date in DTU for RX time adjustment
 * @interrupts: Hardware interrupts count on the device.
 */
struct dw3000_power {
	struct sysfs_power_stats stats[DW3000_PWR_MAX];
	u64 start_time;
	int cur_state;
	int tx_adjust;
	u32 rx_start;
	atomic64_t interrupts;
};

/**
 * struct dw3000_deep_sleep_state - Useful data to restore on wake up
 * @next_operational_state: operational state to enter after DEEP SLEEP mode
 * @config_changed: bitfield of configuration changed during DEEP-SLEEP
 * @frame_idx: saved frame index to use for deferred TX/RX
 * @tx_skb: saved frame to transmit for deferred TX
 * @tx_config: saved config to use for deferred TX
 * @rx_config: saved parameter for deferred RX
 * @regbackup: registers backup to detect diff
 * @compare_work: deferred registers backup compare work
 */
struct dw3000_deep_sleep_state {
	enum operational_state next_operational_state;
	unsigned long config_changed;
	int frame_idx;
	struct sk_buff *tx_skb;
	union {
		struct mcps802154_tx_frame_config tx_config;
		struct mcps802154_rx_frame_config rx_config;
	};
#ifdef CONFIG_DW3000_DEBUG
	void *regbackup;
	struct work_struct compare_work;
#endif
};

/** enum dw3000_rctu_conv_state - DTU to RCTU conversion state
 * @UNALIGNED: need to redo all
 * @ALIGNED: aligned to DTU but not synced yet with RCTU
 * @ALIGNED_SYNCED: all done
 *
 */
enum dw3000_rctu_conv_state { UNALIGNED = 0, ALIGNED, ALIGNED_SYNCED };

/**
 * struct dw3000_rctu_conv - DTU to RCTU conversion data
 * @state: current state of converter
 * @alignment_rmarker_dtu:  alignment DTU value
 * @synced_rmarker_rctu: rmarker RCTU value
 */
struct dw3000_rctu_conv {
	enum dw3000_rctu_conv_state state;
	u32 alignment_rmarker_dtu;
	u64 synced_rmarker_rctu;
};

struct dw3000_cir_data;

#define DW3000_MAX_QUEUED_SPI_XFER 32
#define DW3000_QUEUED_SPI_BUFFER_SZ 2048

/* Number of samples to average. */
#define DW3000_NB_AVERAGE 1

/**
 * struct dw3000_rx_ctx - Custom rx context use with rx_init/rx_get_measurement.
 * @pdoa_rad_q11: Array of PDoA measurements.
 * @aoa_rad_q11: Array of AoA measurements.
 * @index: Index for pdoa_rad_q11 and aoa_rad_q11 arrays.
 * @sample_valid_nb: Number of valid measurements in array, used for average.
 */
struct dw3000_rx_ctx {
	int pdoa_rad_q11[DW3000_NB_AVERAGE];
	int aoa_rad_q11[DW3000_NB_AVERAGE];
	int index;
	int sample_valid_nb;
};

/**
 * struct dw3000 - main DW3000 device structure
 * @spi: pointer to corresponding spi device
 * @dev: pointer to generic device holding sysfs attributes
 * @pm_qos_req: CPU latency request object
 * @sysfs_power_dir: kobject holding sysfs power directory
 * @chip_ops: version specific chip operations
 * @llhw: pointer to associated struct mcps802154_llhw
 * @config: current running chip configuration
 * @txconfig: current running TX configuration
 * @data: local data and register cache
 * @otp_data: OTP data cache
 * @calib_data: calibration data
 * @stats: statistics
 * @power: power related statistics and states
 * @rctu_conv: RCTU converter
 * @time_zero_ns: initial time in ns to convert ktime to/from DTU
 * @dtu_sync: synchro DTU immediately after wakeup
 * @sys_time_sync: device SYS_TIME immediately after wakeup
 * @sleep_enter_dtu: DTU when entered sleep
 * @deep_sleep_state: state related to the deep sleep
 * @idle_timeout: true when idle_timeout_dtu is a valid date.
 * @idle_timeout_dtu: timestamp requested to leave idle mode.
 * @idle_timer: timer to exiting after an idle call.
 * @timer_expired_work: call mcps802154_timer_expired outside driver kthread.
 * @wakeup_done_cb: callback called on wakeup done.
 * @idle_timeout_cb: callback when idle timer expired
 * @auto_sleep_margin_us: configurable automatic deep sleep margin
 * @need_ranging_clock: true if next operation need ranging clock
 *			and deep sleep cannot be used
 * @nfcc_coex: NFCC coexistence specific context
 * @pctt: PCTT specific context
 * @chip_dev_id: identified chip device ID
 * @chip_idx: index of current chip in supported devices array
 * @of_max_speed_hz: saved SPI max speed from device tree
 * @iface_is_started: interface status
 * @has_lock_pm: power management locked status
 * @reset_gpio: GPIO to use for hard reset
 * @regulators: structure that holds regulators to toggle
 * @is_powered: save current regulators state
 * @chips_per_pac: chips per PAC unit
 * @pre_timeout_pac: preamble timeout in PAC unit
 * @coex_delay_us: WiFi coexistence GPIO delay in us
 * @coex_margin_us: WiFi coexistence GPIO margin in us
 * @coex_interval_us: Minimum interval between two operations in us
 *		      under which WiFi coexistence GPIO is kept active
 * @coex_gpio: WiFi coexistence GPIO, >= 0 if activated
 * @coex_enabled: WiFi coexistence activation
 * @coex_status: WiFi coexistence GPIO status, 1 if activated
 * @lna_pa_mode: LNA/PA configuration to use
 * @autoack: auto-ack status, true if activated
 * @pgf_cal_running: true if pgf calibration is running
 * @stm: High-priority thread state machine
 * @rx: received skbuff and associated spinlock
 * @current_operational_state: internal operational state of the chip
 * @operational_state_wq: Wait queue for operational state
 * @debugfs: debugfs informations
 * @spi_pid: PID of the SPI controller pump messages
 * @dw3000_pid: PID the dw3000 state machine thread
 * @restricted_channels: bit field of restricted channels
 * @tx_rf2: parameter to enable the tx on rf2 port
 * @rx_rf2: parameter to enable the rx on rf2 port
 * @cir_data_changed: true if buffer data have been reallocated
 * @full_cia_read: CIA registers fully loaded into cir_data struct
 * @cir_data: allocated CIR exploitation data
 * @msg_queue: SPI message holding transfer queue
 * @msg_queue_xfer: next transfer available
 * @msg_queue_xfer_count: number of queued transfers
 * @msg_queue_buf: buffer for queued transfers
 * @msg_queue_buf_pos: current position in buffer
 * @msg_mutex: mutex protecting @msg_readwrite_fdx
 * @msg_readwrite_fdx: pre-computed generic register read/write SPI message
 * @msg_fast_command: pre-computed fast command SPI message
 * @msg_read_cir_pwr: pre-computed SPI message
 * @msg_read_pacc_cnt: pre-computed SPI message
 * @msg_read_rdb_status: pre-computed SPI message
 * @msg_read_rx_timestamp: pre-computed SPI message
 * @msg_read_rx_timestamp_a: pre-computed SPI message
 * @msg_read_rx_timestamp_b: pre-computed SPI message
 * @msg_read_sys_status: pre-computed SPI message
 * @msg_read_all_sys_status: pre-computed SPI message
 * @msg_read_sys_time: pre-computed SPI message
 * @msg_write_sys_status: pre-computed SPI message
 * @msg_write_all_sys_status: pre-computed SPI message
 * @msg_read_dss_status: pre-computed SPI message
 * @msg_write_dss_status: pre-computed SPI message
 * @msg_write_spi_collision_status: pre-computed SPI message
 */
struct dw3000 {
	/* SPI device */
	struct spi_device *spi;
	/* Generic device */
	struct device *dev;
	/* PM QoS object */
	struct pm_qos_request pm_qos_req;
	/* Kernel object holding sysfs power sub-directory */
	struct kobject sysfs_power_dir;
	/* Chip version specific operations */
	const struct dw3000_chip_ops *chip_ops;
	/* MCPS 802.15.4 device */
	struct mcps802154_llhw *llhw;
	/* Configuration */
	struct dw3000_config config;
	struct dw3000_txconfig txconfig;
	/* Data */
	struct dw3000_local_data data;
	struct dw3000_otp_data otp_data;
	struct dw3000_calibration_data calib_data;
	/* Statistics */
	struct dw3000_stats stats;
	struct dw3000_power power;
	/* Time conversion */
	struct dw3000_rctu_conv rctu_conv;
	s64 time_zero_ns;
	u32 dtu_sync;
	u32 sys_time_sync;
	u32 sleep_enter_dtu;
	/* Deep Sleep & MCPS Idle management */
	struct dw3000_deep_sleep_state deep_sleep_state;
	bool idle_timeout;
	u32 idle_timeout_dtu;
	struct hrtimer idle_timer;
	struct work_struct timer_expired_work;
	dw3000_wakeup_done_cb wakeup_done_cb;
	dw3000_idle_timeout_cb idle_timeout_cb;
	bool need_ranging_clock;
	int auto_sleep_margin_us;
	/* NFCC coexistence specific context. */
	struct dw3000_nfcc_coex nfcc_coex;
	/* PCTT specific context. */
	struct dw3000_pctt pctt;
	/* Chip type management */
	u32 chip_dev_id;
	u32 chip_idx;
	/* Saved SPI max speed from device tree */
	u32 of_max_speed_hz;
	/* True when MCPS start() operation had been called */
	atomic_t iface_is_started;
	/* SPI controller power-management */
	bool has_lock_pm;
	/* Control GPIOs */
	struct gpio_desc *reset_gpio;
	/* Regulators handler  */
	struct dw3000_power_control regulators;
	bool is_powered;
	/* Chips per PAC unit. */
	int chips_per_pac;
	/* Preamble timeout in PAC unit. */
	int pre_timeout_pac;
	/* WiFi coexistence GPIO and delay */
	unsigned coex_delay_us;
	unsigned coex_margin_us;
	unsigned coex_interval_us;
	s8 coex_gpio;
	bool coex_enabled;
	int coex_status;
	/* LNA/PA mode */
	s8 lna_pa_mode;
	/* Is auto-ack activated? */
	bool autoack;
	/* pgf calibration running */
	bool pgf_cal_running;
	/* State machine */
	struct dw3000_state stm;
	/* Receive descriptor */
	struct dw3000_rx rx;
	/* Internal operational state of the chip */
	enum operational_state current_operational_state;
	/* Wait queue for operational state */
	wait_queue_head_t operational_state_wq;
	/* Debugfs settings */
	struct dw3000_debugfs debugfs;
	/* PID of the SPI controller pump messages */
	pid_t spi_pid;
	/* PID the dw3000 state machine thread */
	pid_t dw3000_pid;
	/* Restricted channels */
	u16 restricted_channels;
	/* enable tx on RF2 port  */
	u8 tx_rf2;
	/* enable rx on RF2 port  */
	u8 rx_rf2;
	/* Channel impulse response data */
	bool cir_data_changed;
	bool full_cia_read;
	struct dw3000_cir_data *cir_data;
	/* SPI message holding transfers queue */
	struct spi_message *msg_queue;
	struct spi_transfer *msg_queue_xfer;
	unsigned msg_queue_xfer_count;
	/* Buffer for queued transfers */
	char *msg_queue_buf;
	char *msg_queue_buf_pos;
	/* dw3000 thread clamp value  */
	int min_clamp_value;
	/* Insert new fields before this line */

	/* Shared message protected by a mutex */
	struct mutex msg_mutex;
	/* Precomputed spi_messages */
	struct spi_message *msg_readwrite_fdx;
	struct spi_message *msg_fast_command;
	struct spi_message *msg_read_rdb_status;
	struct spi_message *msg_read_rx_timestamp;
	struct spi_message *msg_read_rx_timestamp_a;
	struct spi_message *msg_read_rx_timestamp_b;
	struct spi_message *msg_read_sys_status;
	struct spi_message *msg_read_all_sys_status;
	struct spi_message *msg_read_sys_time;
	struct spi_message *msg_write_sys_status;
	struct spi_message *msg_write_all_sys_status;
	struct spi_message *msg_read_dss_status;
	struct spi_message *msg_write_dss_status;
	struct spi_message *msg_write_spi_collision_status;
};

/**
 * dw3000_is_active() - Return if DW is in active state (UP and running)
 * @dw: The DW device.
 *
 * Allow to know if DW is in active state (dw3000_enable() called successfully)
 * Used to avoid modification of registers while DW is in use.
 * The chip can be in DEEP_SLEEP state and interface still up & running
 *
 * Return: true is DW is in active state
 */
static inline bool dw3000_is_active(struct dw3000 *dw)
{
	return atomic_read(&dw->iface_is_started);
}

/**
 * nfcc_coex_to_dw() - Help function to retrieve dw3000 context from nfcc_coex.
 * @nfcc_coex: NFCC coexistence context.
 *
 * Returns: DW3000 device context.
 */
static inline struct dw3000 *nfcc_coex_to_dw(struct dw3000_nfcc_coex *nfcc_coex)
{
	return container_of(nfcc_coex, struct dw3000, nfcc_coex);
}

#endif /* __DW3000_H */
