/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __BROWNOUT_STATS_H
#define __BROWNOUT_STATS_H

#if IS_ENABLED(CONFIG_REGULATOR_S2MPG14) || IS_ENABLED(CONFIG_REGULATOR_S2MPG12)
#define METER_CHANNEL_MAX	12
#else
#define METER_CHANNEL_MAX	8
#endif

/* Using a constant size for all the platforms.
 */
#define COMPATIBLE_METER_CHANNEL_MAX	16
static_assert(METER_CHANNEL_MAX <= COMPATIBLE_METER_CHANNEL_MAX);

/* Brownout triggered source need to be sync with the dt-bindings.
 * google-modules/soc/gs/include/dt-bindings/soc/google/zumapro-bcl.h
 */
#define SMPL_WARN	0
#define OCP_WARN_CPUCL1	1
#define OCP_WARN_CPUCL2	2
#define SOFT_OCP_WARN_CPUCL1	3
#define SOFT_OCP_WARN_CPUCL2	4
#define OCP_WARN_TPU	5
#define SOFT_OCP_WARN_TPU	6
#define OCP_WARN_GPU	7
#define SOFT_OCP_WARN_GPU	8
#define PMIC_SOC	9
#define UVLO1	10
#define UVLO2	11
#define BATOILO1	12
#define BATOILO2	13
#define PMIC_120C	14
#define PMIC_140C	15
#define PMIC_OVERHEAT	16
#define BATOILO	BATOILO1
#define TRIGGERED_SOURCE_MAX	17
#define VIMON_BUF_SIZE		256
#define VIMON_BYTES_PER_ENTRY	2
#define MAX77779_VIMON_DATA_SIZE	(VIMON_BUF_SIZE / VIMON_BYTES_PER_ENTRY)
#define MAX77779_VIMON_CH_DATA_SIZE	(MAX77779_VIMON_DATA_SIZE / 2)

/* Mitigation Module ID need to be sync with the dt-bindings.
 * google-modules/soc/gs/include/dt-bindings/soc/google/zumapro-bcl.h
 */
#define AUDIO_MITIGATION_ID		0 /* ODPM non monitored */
#define CELLULAR_MITIGATION_ID		1
#define DISPLAY_MITIGATION_ID		2
#define HAPTICS_MITIGATION_ID		3 /* ODPM non monitored */
#define MODEM_MITIGATION_ID		4
#define WLAN_MITIGATION_ID		5
#define CPU_LITTLE_MITIGATION_ID	6
#define CPU_MID_MITIGATION_ID		7
#define CPU_BIG_MITIGATION_ID		8
#define GPU_MITIGATION_ID		9
#define TPU_MITIGATION_ID		10
#define DDR_MITIGATION_ID		11
#define CAMERA_MITIGATION_ID		12
#define MIF_MITIGATION_ID		13
#define INT_MITIGATION_ID		14
#define LDO_MITIGATION_ID		15
#define GNSS_MITIGATION_ID		16
#define AOC_MITIGATION_ID		17
#define UFS_MITIGATION_ID		18
#define MAX_MITIGATION_MODULE		19

struct odpm_lpf {
	struct timespec64 time;
	u32 value[COMPATIBLE_METER_CHANNEL_MAX];
};

struct vimon_data {
	s32 data[MAX77779_VIMON_DATA_SIZE];
	s32 v_data[MAX77779_VIMON_CH_DATA_SIZE];
	s32 i_data[MAX77779_VIMON_CH_DATA_SIZE];
	size_t count;
};

/* Notice: sysfs only allocates a buffer of PAGE_SIZE
 * so the sizeof brownout_stats should be smaller than that
 */
struct brownout_stats {
	struct timespec64 triggered_time;
	u32 triggered_idx;

	struct odpm_lpf main_odpm_lpf;
	struct odpm_lpf sub_odpm_lpf;
	struct vimon_data vimon_intf;
	u32 triggered_state;
};
static_assert(sizeof(struct brownout_stats) <= PAGE_SIZE);

#endif /* __BROWNOUT_STATS_H */
