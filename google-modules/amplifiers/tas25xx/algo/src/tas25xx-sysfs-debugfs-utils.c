#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include "algo/inc/tas_smart_amp_v2.h"
#include "tas25xx-sysfs-debugfs-utils.h"
#include "tas25xx-algo-intf.h"

#define CALIB_FAILED	0xCACACACA
#define NO_CALIB	0xAA		/*170*/
#define RDC_MIN_L	(2621440)	/*3145728/2^19 = 6 ohm*/
#define RDC_MAX_L	(5242880)	/*5242880/2^19 = 10 ohm*/
#define RDC_MIN_R	(2621440)	/*3145728/2^19 = 6 ohm */
#define RDC_MAX_R	(5242880)	/*5242880/2^19 = 10 ohm*/

/* user impedance: (detect_val/2^19)*/
#define TRANSF_IMPED_TO_USER_I(X) \
	(((X * 100) >> 19) / 100)
#define TRANSF_IMPED_TO_USER_M(X) \
	(((X * 100) >> 19) % 100)

#define MAX_CONTROL_NAME 48
#define STR_SZ_TAS       512

struct smartpa_algo_data{
	int ch_count;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dbg_dir;
#endif
	int mi2s_rx_port_id;
	int mi2s_tx_port_id;
	uint32_t calibRe[MAX_CHANNELS];
	int imped_min[MAX_CHANNELS];
	int imped_max[MAX_CHANNELS];
	int fres_min[MAX_CHANNELS];
	int fres_max[MAX_CHANNELS];
	int Qt[MAX_CHANNELS];
	struct i2c_client *i2c_client;
	int debugfs_init_done;
};

static struct smartpa_algo_data	*s_smartpa_algo;
static uint32_t s_re_calib[MAX_CHANNELS];

static int oc_counter[MAX_CHANNELS];

/* Interfaces required for customer*/
int smartpa_init_dbg(char *calib_result_str, char *calib_result_int, int size);
int smartpa_read_freq_dbg(char *buffer, int size);
void smartpa_read_prars_dbg(int temp[5], unsigned char addr);
void smartpa_get_client(struct i2c_client **client, unsigned char addr);
int smartpa_check_calib_dbg(void);
/* Interfaces required for customer -- end*/

/* Prototypes */
static int smartpa_calib_get(uint32_t *calib_value);
static int alloc_memory_for_smartpa_algo_client(void);

void tas25xx_algo_bump_oc_count(int channel, int reset)
{
	if (reset)
		oc_counter[channel] = 0;
	else
		oc_counter[channel]++;
}

static void smartpa_init_re_value(uint32_t *re_value)
{
	int i, channels;

	if (!s_smartpa_algo) {
		pr_err("[TI-SmartPA]: %s SmartPA_priv is NULL\n", __func__);
	} else {
		channels = s_smartpa_algo->ch_count;
		memcpy(s_re_calib, re_value, sizeof(uint32_t) * channels);
		memcpy(s_smartpa_algo->calibRe, s_re_calib, sizeof(s_re_calib));
		for (i = 0; i < channels; i++, re_value++) {
			pr_debug("[TI-SmartPA]%s: re_value[%d]=%x\n", __func__, i, *re_value);
			pr_debug("[TI-SmartPA]%s: s_re_calib[%d]=%x\n", __func__, i, s_re_calib[i]);
		}
	}
}

static void smartpa_set_re(uint32_t *calibRe)
{
	int nSize = sizeof(uint32_t);
	int ret;
	uint8_t iter = 0;
	uint32_t paramid = 0;

	if (!s_smartpa_algo || !calibRe) {
		pr_err("[TI-SmartPA] %s: s_smartpa_algo or calibRe is NULL\n", __func__);
		return;
	}

	for (iter = 0; iter < s_smartpa_algo->ch_count; iter++) {
		if (calibRe[iter] != 0) {
			pr_info("[TI-SmartPA]%s: Payload : %d", __func__, calibRe[iter]);
			paramid = TAS_CALC_PARAM_IDX(TAS_SA_SET_RE, 1, iter+1);
			ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&calibRe[iter],
					paramid, TAS_SET_PARAM, nSize, TISA_MOD_RX);
			pr_info("[TI-SmartPA]%s: set Re[%d]: %d, ret=%d", __func__, iter,
					calibRe[iter], ret);
		} else {
			pr_err("[TI-SmartPA]%s: Cannot set Re calib for ch=%d", __func__, iter);
		}
	}
}

static bool rdc_check_valid(uint32_t rdc, uint8_t iter)
{
	if (!s_smartpa_algo) {
		pr_err("[TI-SmartPA]: %s s_smartpa_algo is NULL\n", __func__);
		return false;
	}

	if (rdc > s_smartpa_algo->imped_min[iter] &&
			rdc < s_smartpa_algo->imped_max[iter])
		return true;


	pr_info("[TI-SmartPA]%s: rdc=%x invalid, [%d, %d]\n", __func__,
			rdc, s_smartpa_algo->imped_min[iter], s_smartpa_algo->imped_max[iter]);

	return false;
}

static bool smartpa_check_re(void)
{
	int rc = 0;
	uint32_t impedance[MAX_CHANNELS] = {0};
	uint8_t iter = 0, channels = 0;

	pr_debug("[TI-SmartPA]%s:smartpa_check_re enter.\n", __func__);
	if (!s_smartpa_algo) {
		pr_err("[TI-SmartPA]%s: s_smartpa_algo is NULL\n", __func__);
		return false;
	}

	channels = s_smartpa_algo->ch_count;

	for (iter = 0; iter < channels; iter++) {
		if (rdc_check_valid(s_smartpa_algo->calibRe[iter], iter) ||
				(s_smartpa_algo->calibRe[iter] == 0xCACACACA)) {
			pr_info("[TI-SmartPA]%s:smartpa_check_re[%d]:%d ok\n", __func__, iter,
					s_smartpa_algo->calibRe[iter]);
			rc += 1;
		}
	}

	if (rc == channels)
		return true;

	rc = smartpa_calib_get(impedance);
	if (rc == 0) {
		pr_info("[TI-SmartPA]%s: get re failed.\n", __func__);
		return false;
	}

	rc = 0;
	for (iter = 0; iter < channels; iter++) {
		s_smartpa_algo->calibRe[iter] = impedance[iter];
		if (rdc_check_valid(s_smartpa_algo->calibRe[iter], iter) ||
				(s_smartpa_algo->calibRe[iter] == 0xCACACACA)) {
			pr_info("[TI-SmartPA]%s:smartpa_check_re[%d]:%d success.\n",
					__func__, iter, impedance[iter]);
			rc += 1;
		} else {
			pr_info("[TI-SmartPA]%s:smartpa_check_re[%d]:%d failed.\n",
					__func__, iter, impedance[iter]);
		}
	}

	if (rc == channels)
		return true;

	return false;
}

static int smartpa_calib_get(uint32_t *calib_value)
{
	int found = 0;
	int channels = 1;

	if (!s_smartpa_algo || !calib_value) {
		pr_err("[TI-SmartPA]%s: s_smartpa_algo or calib_value is NULL\n", __func__);
		return false;
	}

	channels =  s_smartpa_algo->ch_count;
	memcpy(calib_value, s_re_calib, sizeof(s_re_calib));

	if (calib_value != NULL) {
		pr_info("[TI-SmartPA]%s:calibrate:get calib_value[0] %d\n",
				__func__, calib_value[0]);
		found = 1;
		if (channels == 2)
			pr_info("[TI-SmartPA]%s:calibrate:get calib_value[1] %d\n",
					__func__, calib_value[1]);
	} else {
		pr_info("[TI-SmartPA]%s:calib_value is NULL\n", __func__);
	}
	return found;
}

/* update the cached Re values in kernel once the calibration is complete*/
static int smartpa_calib_save(uint32_t *calib_value)
{
	int ret = 0;
	int i = 0;
	uint8_t channels  = 1;

	if (!s_smartpa_algo || !calib_value) {
		pr_err("[TI-SmartPA]%s: private data or calib_value is NULL\n", __func__);
		ret = -1;
	} else {
		channels = s_smartpa_algo->ch_count;
		smartpa_init_re_value(calib_value);

		for (i = 0; i < sizeof(uint32_t) * channels; i++)
			pr_info("[[TI-SmartPA]%s: calib_val=0x%x ", __func__, calib_value[i]);

		pr_info("\n");
	}

	return ret;
}

#ifdef CONFIG_DEBUG_FS
static ssize_t smartpa_dbgfs_calibrate_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	int ret = 0;
	uint32_t data = 0;
	uint8_t nSize = sizeof(uint32_t);
	uint32_t paramid = 0;
	uint32_t calib_re[MAX_CHANNELS];
	char *str;
	int ret_count = 0;
	uint8_t iter = 0, channels = 1;

	pr_info("%s\n", __func__);
	if (*ppos)
		return 0;

	if (!s_smartpa_algo) {
		pr_err("[TI-SmartPA]%s: SmartPA_priv is NULL\n", __func__);
		ret = -1;
		return ret;
	}
	channels = s_smartpa_algo->ch_count;

	/* calib init */
	for (iter = 0; iter < channels; iter++)	{
		oc_counter[iter] = 0;
		data = 1;
		paramid = TAS_CALC_PARAM_IDX(TAS_SA_CALIB_INIT, 1, iter+1);
		ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data, paramid,
				TAS_SET_PARAM,
				nSize, TISA_MOD_RX);
		if (ret < 0)
			goto end;
	}
	pr_info("[TI-SmartPA]%s:dbgfs_calibrate_read: calib init\n", __func__);

	msleep(2*1000);

	/*get Re*/
	for (iter = 0; iter < channels; iter++)	{
		paramid = TAS_CALC_PARAM_IDX(TAS_SA_GET_RE, 1, iter+1);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_GET_PARAM, sizeof(uint32_t),
				TISA_MOD_RX);
		if (ret < 0)
			goto deinit;

		calib_re[iter] = data;
		if (oc_counter[iter] > 0) {
			calib_re[iter] = 0;
			pr_err("[TI-SmartPA]%s:init_dbg: short-ckt detected for channel-%d sc_count %d\n",
					__func__, iter, oc_counter[iter]);
		}
		pr_info("[TI-SmartPA]%s: calib_re-%d 0x%x\n", __func__, (int)iter, (int)calib_re[iter]);

		if ((calib_re[iter] < s_smartpa_algo->imped_min[iter])
				|| (calib_re[iter] > s_smartpa_algo->imped_max[iter]))
			calib_re[iter] = CALIB_FAILED;

		s_smartpa_algo->calibRe[iter] = calib_re[iter];
		pr_info("[TI-SmartPA]%s: calib_re[%d] is %d\n", __func__, iter, calib_re[iter]);
	}

	pr_info("%s ALLOC: %d", __func__, STR_SZ_TAS*channels);

	str = kmalloc(STR_SZ_TAS*channels, GFP_KERNEL);
	if (!str) {
		ret = -ENOMEM;
		goto deinit;
	}

	ret = 0;
	if (channels == 2) {
		if (calib_re[0] == CALIB_FAILED)
			ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = 0x%x; ",
					calib_re[0]);
		else
			ret = scnprintf(str, STR_SZ_TAS,
					"Channel[0] = %02d.%02d; ",
					TRANSF_IMPED_TO_USER_I(calib_re[0]),
					TRANSF_IMPED_TO_USER_M(calib_re[0]));
		if (calib_re[1] == CALIB_FAILED)
			ret += scnprintf(str+ret, STR_SZ_TAS-ret,
					"Channel[1] = 0x%x;\n",
					calib_re[1]);
		else
			ret += scnprintf(str+ret, STR_SZ_TAS-ret,
					"Channel[1] = %02d.%02d;\n",
					TRANSF_IMPED_TO_USER_I(calib_re[1]),
					TRANSF_IMPED_TO_USER_M(calib_re[1]));
	} else {
		if (calib_re[0] == CALIB_FAILED)
			ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = 0x%x;\n",
					calib_re[0]);
		else
			ret = scnprintf(str, STR_SZ_TAS,
					"Channel[0] = %02d.%02d;\n",
					TRANSF_IMPED_TO_USER_I(calib_re[0]),
					TRANSF_IMPED_TO_USER_M(calib_re[0]));
	}
	ret_count = simple_read_from_buffer(user_buf, count, ppos, str, ret);

	smartpa_calib_save(calib_re);

	pr_info("[TI-SmartPA]%s:debugfs test count %d, ret_count %d, ret %d\n",
			__func__, (int)count, (int)ret_count, (int)ret);
	kfree(str);

deinit:
	for (iter = 0; iter < channels; iter++) {
		data = 0;
		paramid = TAS_CALC_PARAM_IDX(TAS_SA_CALIB_DEINIT, 1, iter+1);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_SET_PARAM,
				nSize, TISA_MOD_RX);
	}
	pr_info("[TI-SmartPA]%s:dbgfs_calibrate_read: decalib init\n", __func__);

end:
	pr_debug("[TI-SmartPA]%s:dbgfs_calibrate_read: end\n", __func__);

	if (ret < 0)
		return ret;
	else
		return ret_count;
}

static const struct file_operations smartpa_dbgfs_calibrate_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_calibrate_read,
	.llseek = default_llseek,
};

static ssize_t smartpa_dbgfs_impedance_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	return 0;
}

static const struct file_operations smartpa_dbgfs_impedance_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_impedance_read,
	.llseek = default_llseek,
};

static ssize_t smartpa_dbgfs_f0_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	uint32_t calibRe[MAX_CHANNELS];
	uint32_t F0[MAX_CHANNELS], Q[MAX_CHANNELS];
	int ret = 0, ret_count = 0;
	uint32_t data;
	uint32_t paramid = 0;
	int nSize = sizeof(uint32_t);
	char *str;
	uint8_t iter = 0, channels = 1;

	pr_info("[TI-SmartPA]%s: enter\n", __func__);
	if (*ppos)
		return 0;

	if (!s_smartpa_algo) {
		pr_err("[TI-SmartPA]%s: SmartPA_priv is NULL\n", __func__);
		ret = -1;
		return ret;
	}
	channels = s_smartpa_algo->ch_count;

	/* Load Calib */
	if (smartpa_check_re()) {
		calibRe[0] = s_smartpa_algo->calibRe[0];
		if (channels == 2)
			calibRe[1] = s_smartpa_algo->calibRe[1];
		smartpa_set_re(calibRe);
	}

	for (iter = 0; iter < channels; iter++) {
		oc_counter[iter] = 0;
		data = 1;
		paramid = TAS_CALC_PARAM_IDX(TAS_SA_F0_TEST_INIT, 1, iter+1);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_SET_PARAM,
				nSize, TISA_MOD_RX);
		if (ret < 0)
			goto end;
	}

	/* wait for 5 s*/
	msleep(5000);

	/* read f0 */
	for (iter = 0; iter < channels; iter++)	{
		data = 0;//Resets data to 0
		paramid = TAS_CALC_PARAM_IDX(TAS_SA_GET_F0, 1, iter+1);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_GET_PARAM,
				sizeof(data),
				TISA_MOD_RX);
		if (ret < 0)
			goto end;
		F0[iter] = data;
		if (oc_counter[iter] > 0) {
			F0[iter] = 0;
			pr_err("[TI-SmartPA]%s:debugfs: short-ckt detected for channel-%d sc_count %d\n",
					__func__, iter, oc_counter[iter]);
		}
		pr_info("[TI-SmartPA]%s: F0[%d] is %d\n",
				__func__, iter, F0[iter]);
	}

	pr_info("%s ALLOC: %d", __func__, STR_SZ_TAS*channels);

	str = kmalloc(STR_SZ_TAS*channels, GFP_KERNEL);
	if (!str) {
		ret = -ENOMEM;
		goto deinit;
	}
	ret = 0;
	if (channels == 2) {
		ret = scnprintf(str, STR_SZ_TAS,
				"Channel[0] = %d; Channel[1] = %d;\n",
				(F0[0] >> 19), (F0[1] >> 19));
	} else
		ret = scnprintf(str, STR_SZ_TAS, "Channel[0] = %d;\n",
				(F0[0] >> 19));

	ret_count = simple_read_from_buffer(user_buf, count, ppos, str, ret);
	kfree(str);

	/* read Q */
	for (iter = 0; iter < channels; iter++) {
		data = 0;
		paramid = TAS_CALC_PARAM_IDX(TAS_SA_GET_Q, 1, iter+1);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_GET_PARAM,
				sizeof(data),
				TISA_MOD_RX);
		if (ret < 0)
			goto end;
		Q[iter] = data;
		if (oc_counter[iter] > 0) {
			Q[iter] = 0;
			pr_err("[TI-SmartPA]%s: short-ckt detected for channel-%d sc_count %d\n",
					__func__, iter, oc_counter[iter]);
		}
		pr_info("[TI-SmartPA]%s: Q[%d] is %d\n", __func__, iter, Q[iter]);
	}

deinit:
	for (iter = 0; iter < channels; iter++) {
		data = 0;
		paramid = TAS_CALC_PARAM_IDX(TAS_SA_F0_TEST_DEINIT, 1, iter+1);
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_SET_PARAM, nSize, TISA_MOD_RX);
		if (ret < 0)
			goto end;
	}

end:
	if (ret < 0)
		return ret;
	else
		return ret_count;
}

static const struct file_operations smartpa_dbgfs_f0_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_f0_read,
	.llseek = default_llseek,
};
static ssize_t smartpa_dbgfs_temperature_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	return 0;
}

static const struct file_operations smartpa_dbgfs_temperature_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_temperature_read,
	.llseek = default_llseek,
};
static ssize_t smartpa_dbgfs_QFactor_read(struct file *file,
		char __user *user_buf, size_t count,
		loff_t *ppos)
{
	return 0;
}

static const struct file_operations smartpa_dbgfs_QFactor_fops = {
	.open = simple_open,
	.read = smartpa_dbgfs_QFactor_read,
	.llseek = default_llseek,
};

static void smartpa_debug_init(struct smartpa_algo_data *algo)
{
	char name[60];
	struct i2c_client *i2c = algo->i2c_client;

	if (!i2c)
		scnprintf(name, MAX_CONTROL_NAME, "audio-tismartpa");
	else
		scnprintf(name, MAX_CONTROL_NAME, "audio-%s", i2c->name);

	algo->dbg_dir = debugfs_create_dir(name, NULL);
	debugfs_create_file("calibrate", S_IRUGO|S_IWUGO, algo->dbg_dir,
			i2c, &smartpa_dbgfs_calibrate_fops);
	debugfs_create_file("impedance", S_IRUGO|S_IWUGO, algo->dbg_dir,
			i2c, &smartpa_dbgfs_impedance_fops);
	debugfs_create_file("f0detect", S_IRUGO|S_IWUGO, algo->dbg_dir,
			i2c, &smartpa_dbgfs_f0_fops);
	debugfs_create_file("QFactor", S_IRUGO|S_IWUGO, algo->dbg_dir,
			i2c, &smartpa_dbgfs_QFactor_fops);
	debugfs_create_file("temperature", S_IRUGO|S_IWUGO, algo->dbg_dir,
			i2c, &smartpa_dbgfs_temperature_fops);
}

static void smartpa_debug_remove(struct smartpa_algo_data *algo)
{
	if (algo->dbg_dir)
		debugfs_remove_recursive(algo->dbg_dir);
}

#endif /*CONFIG_DEBUG_FS*/

int smartpa_init_dbg(char *calib_result_str, char *calib_re_int_ref, int size)
{
	uint32_t calib_re[MAX_CHANNELS] = {0};
	uint32_t paramid = 0;
	int ret = 0, n = 0;
	uint32_t data = 0;
	bool done[MAX_CHANNELS] = {false};
	int nSize = sizeof(uint32_t);
	uint8_t iter = 0, channels = 1;

	pr_debug("[TI-SmartPA]: enter %s\n", __func__);

	if (!s_smartpa_algo) {
		pr_err("[TI-SmartPA]%s: s_smartpa_algo is NULL\n", __func__);
		ret = -1;
		return ret;
	}

	channels = s_smartpa_algo->ch_count;
	if (channels == 1)
		done[1] = true;

	msleep(1000);

	if (1) {
		/*calib init*/
		for (iter = 0; iter < channels; iter++) {
			oc_counter[iter] = 0;
			data = 1;//Value is ignored
			paramid = TAS_CALC_PARAM_IDX(TAS_SA_CALIB_INIT, 1, iter+1);
			ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data,
					paramid,
					TAS_SET_PARAM, nSize,
					TISA_MOD_RX);
			if (ret < 0) {
				done[iter] = false;
				pr_info("[TI-SmartPA]%s:init_dbg:set calib_data error.\n",
						__func__);
				ret = -ENOMEM;
			}
		}
		pr_info("[TI-SmartPA]%s:init_dbg: calib init\n", __func__);

		msleep(3 * 1000);

		/*get Re*/
		for (iter = 0; iter < channels; iter++) {
			data = 0;
			paramid = TAS_CALC_PARAM_IDX(TAS_SA_GET_RE, 1, iter+1);
			ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
					TAS_GET_PARAM, /*length */ 4, TISA_MOD_RX);
			if (ret < 0) {
				done[iter] = false;
				pr_info("[TI-SmartPA]%s:init_dbg: decalib init\n",
						__func__);
			} else {
				calib_re[iter] = data;
				if (oc_counter[iter] > 0) {
					calib_re[iter] = 0;
					pr_err("[TI-SmartPA]%s: short-ckt detected for channel-%d sc_count %d\n",
							__func__, iter, oc_counter[iter]);
				}

				if ((calib_re[iter] < s_smartpa_algo->imped_min[iter]) ||
						(calib_re[iter] > s_smartpa_algo->imped_max[iter]))
					done[iter] = false;
				else
					done[iter] = true;
				pr_info("[TI-SmartPA]%s: calib_re is %d, valid range (%d %d)\n",
						__func__, calib_re[iter],
						s_smartpa_algo->imped_min[iter],
						s_smartpa_algo->imped_max[iter]);
			}
		}
	} else {
		done[0] = false;
		if (channels == 2)
			done[1] = false;
		ret = -EINVAL;
		pr_info("[TI-SmartPA]%s: failed to calibrate %d\n",
				__func__, ret);
	}

	n += scnprintf(calib_result_str + n, size - n, "current status:[TI-SmartPA]: %s\n",
			(channels == 1) ? "Mono" : "Stereo");

	for (iter = 0; iter < channels; iter++) {
		n += scnprintf(calib_result_str + n, size - n,
				"Channel[%d]: impedance %02d.%02d ohm, valid range(%02d.%02d ~ %02d.%02d ohm).\n",
				iter,
				TRANSF_IMPED_TO_USER_I(calib_re[iter]),
				TRANSF_IMPED_TO_USER_M(calib_re[iter]),
				TRANSF_IMPED_TO_USER_I(s_smartpa_algo->imped_min[iter]),
				TRANSF_IMPED_TO_USER_M(s_smartpa_algo->imped_min[iter]),
				TRANSF_IMPED_TO_USER_I(s_smartpa_algo->imped_max[iter]),
				TRANSF_IMPED_TO_USER_M(s_smartpa_algo->imped_max[iter]));
		pr_info("[TI-SmartPA]%s: calibRe[%d] %d\n",
				__func__, iter, calib_re[iter]);
		if (!done[iter])
			calib_re[iter] = CALIB_FAILED;
	}

	/* calibration result in human readable format*/
	n += scnprintf(calib_result_str + n, size - n, "\n Calibrate result: %s\n",
			(done[0] && done[1]) ? "OKAY(impedance ok)." : "ERROR!");
	calib_result_str[n] = 0;
	/* calibration result - re values only*/
	memcpy(calib_re_int_ref, calib_re, sizeof(uint32_t)*channels);

	pr_info("[TI-SmartPA]%s: write to file\n", __func__);

	s_smartpa_algo->calibRe[0] = calib_re[0];
	s_smartpa_algo->calibRe[1] = calib_re[1];
	pr_info("[TI-SmartPA] %s: update Re value\n", __func__);

	smartpa_calib_save(calib_re);

	/*deinit:*/
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Value is ignored
		paramid = TAS_CALC_PARAM_IDX(TAS_SA_CALIB_DEINIT, 1, iter+1);
		ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data, paramid,
				TAS_SET_PARAM, nSize, TISA_MOD_RX);
		pr_info("[TI-SmartPA]%s: decalib init\n", __func__);
	}

	/*end:*/
	pr_debug("[TI-SmartPA]%s: end\n", __func__);

	if (done[0] && done[1])
		ret = 1;
	else
		ret = 0;
	return ret;
}

int smartpa_read_freq_dbg(char *buffer, int size)
{
	uint32_t calibRe[MAX_CHANNELS] = {0};
	uint32_t F0[MAX_CHANNELS] = {0}, Q[MAX_CHANNELS] = {0};
	int ret = 0, n = 0, i[MAX_CHANNELS] = {0}, j[MAX_CHANNELS] = {0};
	uint32_t data = 0;
	uint32_t paramid = 0;
	int nSize = sizeof(uint32_t);
	uint8_t iter = 0, channels = 1;

	pr_debug("[TI-SmartPA]%s: enter\n", __func__);

	if (!s_smartpa_algo) {
		pr_err("[TI-SmartPA]%s: s_smartpa_algo is NULL\n", __func__);
		ret = -1;
		return ret;
	}
	channels = s_smartpa_algo->ch_count;

	/*Load Calib*/
	if (smartpa_check_re()) {

		for (iter = 0; iter < channels; iter++)
			calibRe[iter] = s_smartpa_algo->calibRe[iter];
		smartpa_set_re(calibRe);
	}

	for (iter = 0; iter < channels; iter++) {
		oc_counter[iter] = 0;
		data = 1;//Value is ignored
		paramid = TAS_CALC_PARAM_IDX(TAS_SA_F0_TEST_INIT, 1, iter+1);
		ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data, paramid,
				TAS_SET_PARAM, nSize, TISA_MOD_RX);
	}

	msleep(5000);

	for (iter = 0; iter < channels; iter++) {
		n += scnprintf(buffer+n, size-n, "Ch[%d] ", iter);
		if (calibRe[iter] == CALIB_FAILED)
			n += scnprintf(buffer+n, size-n, "Rdc = %x\n",
					calibRe[iter]);
		else
			n += scnprintf(buffer+n, size-n, "Rdc = %02d.%02d\n",
					TRANSF_IMPED_TO_USER_I(calibRe[iter]),
					TRANSF_IMPED_TO_USER_M(calibRe[iter]));
		while ((i[iter]++ < 5) && (j[iter] < 3)) {
			data = 0;
			paramid = TAS_CALC_PARAM_IDX(TAS_SA_GET_F0, 1, iter+1);
			ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
					TAS_GET_PARAM, sizeof(data), TISA_MOD_RX);
			F0[iter] = data;

			data = 0;
			paramid = TAS_CALC_PARAM_IDX(TAS_SA_GET_Q, 1, iter+1);
			ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
					TAS_GET_PARAM, sizeof(data), TISA_MOD_RX);
			Q[iter] = data;

			if (oc_counter[iter] > 0) {
				F0[iter] = 0;
				Q[iter] = 0;
				pr_err("[TI-SmartPA]%s:debugfs: short-ckt detected for channel-%d sc_count %d\n",
						__func__, iter, oc_counter[iter]);
			}

			if (((F0[iter] >> 19) < s_smartpa_algo->fres_min[iter])
					|| ((F0[iter] >> 19) > s_smartpa_algo->fres_max[iter])
					|| (((Q[iter] * 100) >> 19) < s_smartpa_algo->Qt[iter]))
				j[iter] = 0;
			else
				j[iter]++;

			pr_info("[TI-SmartPA]%s:read freq dbg channel[%d]: f0 = %d Q = %d i = %d j = %d\n",
					__func__, iter, F0[iter],
					Q[iter], i[iter], j[iter]);
			n += scnprintf(buffer+n, size-n, "f0: %d Q: %d.%d\n",
					(F0[iter] >> 19),
					TRANSF_IMPED_TO_USER_I(Q[iter]),
					TRANSF_IMPED_TO_USER_M(Q[iter]));
			msleep(500);
		}
		n += scnprintf(buffer+n, size-n, "f0 (%d~%d)\nQ_Min: %d.%d\n",
				s_smartpa_algo->fres_min[iter],
				s_smartpa_algo->fres_max[iter],
				s_smartpa_algo->Qt[iter] / 100,
				s_smartpa_algo->Qt[iter] % 100);
		if (j[iter] == 3)
			n += scnprintf(buffer+n, size-n, "PASS\n");
		else
			n += scnprintf(buffer+n, size-n, "FAIL\n");
	}

	buffer[n] = 0;

	/*deinit :*/
	for (iter = 0; iter < channels; iter++) {
		data = 0;
		paramid = TAS_CALC_PARAM_IDX(TAS_SA_F0_TEST_DEINIT, 1, iter+1);
		ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data, paramid,
				TAS_SET_PARAM, nSize, TISA_MOD_RX);
	}
	/*end:*/
	return 0;
}

void smartpa_read_prars_dbg(int temp[5], unsigned char addr)
{
	pr_info("[TI-SmartPA]%s: enter.\n", __func__);
}

/* This should be called first */
int smaramp_set_i2c_client(struct i2c_client *i2c)
{
	int ret = alloc_memory_for_smartpa_algo_client();

	if (ret)
		return ret;

	s_smartpa_algo->i2c_client = i2c;
	return 0;
}

void smartpa_get_client(struct i2c_client **client, unsigned char addr)

{
	pr_debug("[TI-SmartPA]%s: enter.\n", __func__);

	if (s_smartpa_algo)
		*client = s_smartpa_algo->i2c_client;
}

int smartpa_check_calib_dbg(void)
{
	uint32_t impedance[MAX_CHANNELS] = {0};
	uint8_t iter = 0, channels = 0;
	int ret = 1;

	if (!s_smartpa_algo)
		return 0;

	pr_debug("[TI-SmartPA]%s: enter.\n", __func__);

	smartpa_calib_get(impedance);
	channels = s_smartpa_algo->ch_count;
	for (iter = 0; iter < channels; iter++)
		ret &= rdc_check_valid(impedance[iter], iter);

	return ret;
}

int tas25xx_parse_algo_bin_sysfs(int ch_count, u8 *buf)
{
	uint32_t *ptr = (uint32_t *)buf;
	int ret = 0, i;

	if (!s_smartpa_algo)
		ret = alloc_memory_for_smartpa_algo_client();

	if (ret || !s_smartpa_algo) {
		pr_err("[TI-SmartPA]%s: s_smartpa_algo is NULL\n", __func__);
		return ret;
	}

	for (i = 0; i < ch_count; i++) {
		s_smartpa_algo->imped_min[i] = *ptr;
		ptr++;
		s_smartpa_algo->imped_max[i] = *ptr;
		ptr++;
		pr_info("[SmartPA-%s]: Re(min,max) = %d, %d\n", __func__,
			s_smartpa_algo->imped_min[i], s_smartpa_algo->imped_max[i]);

		s_smartpa_algo->fres_min[i] = *ptr;
		ptr++;
		s_smartpa_algo->fres_max[i] = *ptr;
		ptr++;

		pr_info("[SmartPA-%s]: F(min,max) = %d, %d\n", __func__,
			s_smartpa_algo->fres_min[i], s_smartpa_algo->fres_max[i]);

		s_smartpa_algo->Qt[i] = *ptr;
		ptr++;
		pr_info("[SmartPA-%s]: Qt = %d\n", __func__, s_smartpa_algo->Qt[i]);

		/* skip 5 dummy params */
		ptr += 5;
	}

	return ret;
}

void tas25xx_send_algo_calibration(void)
{
	if (smartpa_check_re()) {
		smartpa_set_re(s_smartpa_algo->calibRe);
		pr_info("[TI-SmartPA]%s:SetRe[0] called %d(0x%x)", __func__,
				s_smartpa_algo->calibRe[0], s_smartpa_algo->calibRe[0]);
		if (s_smartpa_algo->ch_count == 2)
			pr_info("[TI-SmartPA]%s:SetRe[1] called %d(0x%x)",
					__func__, s_smartpa_algo->calibRe[1],
					s_smartpa_algo->calibRe[1]);
	} else {
		pr_info("[TI-SmartPA]%s:SetRe is not called", __func__);
	}
}

static int alloc_memory_for_smartpa_algo_client(void)
{
	if (!s_smartpa_algo) {
		int size = sizeof(struct smartpa_algo_data);

		pr_info("%s ALLOC: %d", __func__, size);

		s_smartpa_algo = kmalloc(size, GFP_KERNEL);
		memset(s_smartpa_algo, 0, sizeof(struct smartpa_algo_data));
	}

	if (!s_smartpa_algo)
		return -ENOMEM;

	return 0;
}

int tas_smartamp_add_algo_controls_debugfs(struct snd_soc_component *c,
		int number_of_channels)
{
	int ret = alloc_memory_for_smartpa_algo_client();

	if (ret)
		return ret;

	s_smartpa_algo->ch_count = number_of_channels;

#ifdef CONFIG_DEBUG_FS
	smartpa_debug_init(s_smartpa_algo);
#endif

	return 0;
}

void tas_smartamp_remove_algo_controls_debugfs(struct snd_soc_component *c)
{
	if (s_smartpa_algo) {
#ifdef CONFIG_DEBUG_FS
		smartpa_debug_remove(s_smartpa_algo);
#endif
		kfree(s_smartpa_algo);
		s_smartpa_algo = NULL;
	}
}

