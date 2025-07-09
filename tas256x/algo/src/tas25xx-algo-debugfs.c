#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <sound/soc.h>
#include "algo/inc/tas_smart_amp_v2.h"

#define CALIB_FAILED	0xCACACACA
#define NO_CALIB		0xAA		//170
#define RDC_MIN_L		(2621440)	//3145728/2^19 = 6 ohm
#define RDC_MAX_L		(5242880)	//5242880/2^19 = 10 ohm
#define RDC_MIN_R		(2621440)	//3145728/2^19 = 6 ohm
#define RDC_MAX_R		(5242880)	//5242880/2^19 = 10 ohm

/* user impedance: (detect_val/2^19)*/
#define TRANSF_IMPED_TO_USER_I(X) \
		(((X * 100) >> 19) / 100)
#define TRANSF_IMPED_TO_USER_M(X) \
		(((X * 100) >> 19) % 100)

#define CALIBRATE_FILE   "/mnt/vendor/persist/audio/smartamp.bin"
#define FREQ_FILE   "/data/engineermode/speakerleak"
#define MAX_CONTROL_NAME        48
#define STR_SZ_TAS 512

typedef struct {
	int mn_channels;
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
} smartpa_algo_data_t;

static smartpa_algo_data_t	*s_smartpa_algo;

/* Interfaces required for customer*/
int smartpa_init_dbg(char *buffer, int size);
int smartpa_read_freq_dbg(char *buffer, int size);
void smartpa_read_prars_dbg(int temp[5], unsigned char addr);
void smartpa_get_client(struct i2c_client **client, unsigned char addr);
int smartpa_check_calib_dbg(void);
/* Interfaces required for customer -- end*/

/* Prototypes */
static int smartpa_calib_get(uint32_t *calib_value);
static int alloc_memory_for_smartpa_algo_client(void);

static void smartpa_set_re(uint32_t *calibRe)
{
	int nSize = sizeof(uint32_t);
	int ret;
	uint8_t iter = 0;
	uint32_t paramid = 0;

	if (!s_smartpa_algo || !calibRe) {
		pr_err("[SmartPA-%d]: s_smartpa_algo or calibRe is NULL\n",
			__LINE__);
		return;
	}
	//if((calibRe != NO_CALIB) && (calibRe != CALIB_FAILED)) {
	for (iter = 0; iter < s_smartpa_algo->mn_channels; iter++) {
		if (calibRe[iter] != 0) {
			pr_info("[SmartPA-%d]: smartamp : Payload : %d",
				__LINE__, calibRe[iter]);
			paramid =
				((iter+1) << 24) |
				(nSize << 16) |
				TAS_SA_SET_RE;
			ret = tas25xx_smartamp_algo_ctrl(
					(uint8_t *)&calibRe[iter],
					paramid,
					TAS_SET_PARAM, nSize,
					TISA_MOD_RX);
			pr_err("[SmartPA-%d]: set Re[%d]: %d, ret=%d", __LINE__,
				iter, calibRe[iter], ret);
		} else
			pr_err("[SmartPA-%d]: Cannot set Re for calib status wrong",
				__LINE__);
	}
}

static bool rdc_check_valid(uint32_t rdc, uint8_t iter)
{
	if (!s_smartpa_algo) {
		pr_err("[SmartPA-%d]: s_smartpa_algo is NULL\n", __LINE__);
		return false;
	}
	if (rdc > s_smartpa_algo->imped_min[iter] &&
		rdc < s_smartpa_algo->imped_max[iter]) {
		return true;
	}

	pr_info("[SmartPA-%d]rdc check: rdc=%d invalid, [%d, %d]\n", __LINE__,
			rdc, s_smartpa_algo->imped_min[iter],
			s_smartpa_algo->imped_max[iter]);
	return false;
}

static bool smartpa_check_re(void)
{
	int rc = 0;
	uint32_t impedance[MAX_CHANNELS] = {0};
	uint8_t iter = 0, channels = 0;

	pr_info("[SmartPA-%d] smartpa_check_re enter.\n", __LINE__);
	if (!s_smartpa_algo) {
		pr_err("[SmartPA-%d]: s_smartpa_algo is NULL\n", __LINE__);
		return false;
	}
	channels = s_smartpa_algo->mn_channels;

	for (iter = 0; iter < channels; iter++) {
		if (rdc_check_valid(s_smartpa_algo->calibRe[iter], iter) ||
			(s_smartpa_algo->calibRe[iter] == 0xCACACACA)) {
			pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d ok.\n",
					__LINE__, iter,
					s_smartpa_algo->calibRe[iter]);
			rc += 1;
		}
	}
	if (rc == channels)
		return true;

	rc = smartpa_calib_get(impedance);
	if (rc == 0) {
		pr_info("[SmartPA-%d] smartpa_check_re get re failed.\n",
			__LINE__);
		return false;
	}

	rc = 0;
	for (iter = 0; iter < channels; iter++) {
		s_smartpa_algo->calibRe[iter] = impedance[iter];
		if (rdc_check_valid(s_smartpa_algo->calibRe[iter], iter) ||
			(s_smartpa_algo->calibRe[iter] == 0xCACACACA)) {
			pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d success.\n",
				__LINE__, iter, impedance[iter]);
			rc += 1;
		} else {
			pr_info("[SmartPA-%d] smartpa_check_re[%d]:%d failed.\n",
				__LINE__, iter, impedance[iter]);
		}
	}

	if (rc == channels)
		return true;

	return false;
}

static int smartpa_calib_get(uint32_t *calib_value)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int found = 0;
	loff_t pos = 0;
	int channels = 1;

	if (!s_smartpa_algo || !calib_value) {
		pr_err("[SmartPA-%d]: s_smartpa_algo or calib_value is NULL\n",
			__LINE__);
		return false;
	}
	channels =  s_smartpa_algo->mn_channels;

	*calib_value = 0;
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(CALIBRATE_FILE, O_RDONLY, 0);
	if (!IS_ERR_OR_NULL(pfile)) {
		found = 1;
		vfs_read(pfile, (char *)calib_value, sizeof(uint32_t)*channels,
			&pos);
		pr_info("[SmartPA-%d]calibrate:get calib_value[0] %d\n",
			__LINE__, calib_value[0]);
		if (channels == 2)
			pr_info("[SmartPA-%d]calibrate:get calib_value[1] %d\n",
				__LINE__, calib_value[1]);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]calibrate: No found\n", __LINE__);
		found = 0;
	}

	set_fs(old_fs);

	return found;
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

	if (!s_smartpa_algo) {
		pr_err("[SmartPA-%d]: SmartPA_priv is NULL\n", __LINE__);
		ret = -1;
		return ret;
	}
	channels = s_smartpa_algo->mn_channels;

	//calib init
	for (iter = 0; iter < channels; iter++)	{
		data = 1;//Value is ignored
		paramid = ((TAS_SA_CALIB_INIT)|((iter+1)<<24)|(1<<16));
		ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data, paramid,
			TAS_SET_PARAM,
			nSize, TISA_MOD_RX);
		if (ret < 0)
			goto end;
	}
	pr_info("[SmartPA-%d]dbgfs_calibrate_read: calib init\n", __LINE__);

	msleep(2*1000);

	//get Re
	for (iter = 0; iter < channels; iter++)	{
		paramid = ((TAS_SA_GET_RE)|((iter+1)<<24)|(1<<16));
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_GET_PARAM, sizeof(uint32_t),
				TISA_MOD_RX);
		if (ret < 0)
			goto deinit;

		calib_re[iter] = data;
		pr_info("[SmartPa-%d]debugfs: calib_re-%d 0x%x\n", __LINE__,
			(int)iter, (int)calib_re[iter]);

		if ((calib_re[iter] < s_smartpa_algo->imped_min[iter])
			|| (calib_re[iter] > s_smartpa_algo->imped_max[iter]))
			calib_re[iter] = CALIB_FAILED;

		s_smartpa_algo->calibRe[iter] = calib_re[iter];
		pr_info("[SmartPA-%d]debugfs calib_re[%d] is %d\n", __LINE__,
			iter, calib_re[iter]);
	}

	str = kmalloc(STR_SZ_TAS*channels, GFP_KERNEL);
	if (!str) {
		pr_info("[SmartPA-%d]debugfs calibre: failed to kmalloc\n",
			__LINE__);
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
	pr_info("[SmartPA-%d]debugfs count %d, ret_count %d, ret %d\n",
			__LINE__, (int)count, (int)ret_count, (int)ret);
	kfree(str);

deinit:
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Value is ignored
		paramid = ((TAS_SA_CALIB_DEINIT)|((iter+1)<<24)|(1<<16));
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_SET_PARAM,
				nSize, TISA_MOD_RX);
	}
	pr_info("[SmartPA-%d]dbgfs_calibrate_read: decalib init\n", __LINE__);

end:
	pr_info("[SmartPA-%d]dbgfs_calibrate_read: end\n", __LINE__);

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
	u32 length = 1;
	uint32_t calibRe[MAX_CHANNELS];
	uint32_t F0[MAX_CHANNELS], Q[MAX_CHANNELS];
	int ret = 0, ret_count = 0;
	uint32_t data;
	mm_segment_t fs;
	uint32_t paramid = 0;
	int nSize = sizeof(uint32_t);
	char *str;
	struct file *fp = NULL;
	loff_t pos;
	uint8_t iter = 0, channels = 1;

	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);

	if (!s_smartpa_algo) {
		pr_err("[SmartPA-%d]: SmartPA_priv is NULL\n", __LINE__);
		ret = -1;
		return ret;
	}
	channels = s_smartpa_algo->mn_channels;

	//Load Calib
	if (smartpa_check_re()) {
		calibRe[0] = s_smartpa_algo->calibRe[0];
		if (channels == 2)
			calibRe[1] = s_smartpa_algo->calibRe[1];
		smartpa_set_re(calibRe);
	}

	for (iter = 0; iter < channels; iter++) {
		data = 1;//Value is ignored
		paramid = ((TAS_SA_F0_TEST_INIT) |
				(length << 16) |
				((iter+1) << 24));
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
					TAS_SET_PARAM,
					nSize, TISA_MOD_RX);
		if (ret < 0)
			goto end;
	}
	//wait 5s
	msleep(5000);
	//read F0
	for (iter = 0; iter < channels; iter++)	{
		data = 0;//Resets data to 0
		paramid = (TAS_SA_GET_F0 | (length << 16) | ((iter+1) << 24));
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_GET_PARAM,
				4,/*length **/
				TISA_MOD_RX);
		if (ret < 0)
			goto end;
		F0[iter] = data;
		pr_err("[SmartPA-%d]: F0[%d] is %d\n",
			__LINE__, iter, F0[iter]);
	}

	str = kmalloc(STR_SZ_TAS*channels, GFP_KERNEL);
	if (!str) {
		pr_info("[SmartPA-%d]debugfs calibre: failed to kmalloc\n",
			__LINE__);
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

	//read Q
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Reset data to 0
		paramid = (TAS_SA_GET_Q | (length << 16) | ((iter+1) << 24));
		ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_GET_PARAM,
				/*length **/ 4, TISA_MOD_RX);
		if (ret < 0)
			goto end;
		Q[iter] = data;
		pr_err("[SmartPA-%d]: Q[%d] is %d\n", __LINE__, iter, Q[iter]);
	}

	//write to file
	fp = filp_open(CALIBRATE_FILE, O_RDWR | O_CREAT, 0644);
	if (fp > 0) {
		fs = get_fs();
		set_fs(KERNEL_DS);
		pos = 0;
		nSize = vfs_write(fp, (char *)&F0[0], sizeof(uint32_t), &pos);
		nSize = vfs_write(fp, (char *)&Q[0], sizeof(uint32_t), &pos);
		pr_info("[SmartPA-%d] write to file channel[0], F0 = %d, Q = %d\n",
			__LINE__, F0[0], Q[0]);
		if (channels == 2) {
			nSize = vfs_write(fp, (char *)&F0[1], sizeof(uint32_t),
				&pos);
			nSize = vfs_write(fp, (char *)&Q[1], sizeof(uint32_t),
				&pos);
			pr_info("[SmartPA-%d] write to file channel[1], F0 = %d, Q = %d\n",
				__LINE__, F0[1], Q[1]);
		}
		filp_close(fp, NULL);
		set_fs(fs);
	}

deinit:
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Value is ignored
		paramid = ((TAS_SA_F0_TEST_DEINIT) | (length << 16) |
			((iter+1) << 24));
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

static void smartpa_debug_init(smartpa_algo_data_t *algo)
{
	char name[60];
	struct i2c_client *i2c = algo->i2c_client;

	if (!i2c)
		scnprintf(name, MAX_CONTROL_NAME, "audio-tismartpa");
	else
		scnprintf(name, MAX_CONTROL_NAME, "audio-%s", i2c->name);

	algo->dbg_dir = debugfs_create_dir(name, NULL);
	debugfs_create_file("calibrate", 0666, algo->dbg_dir,
			i2c, &smartpa_dbgfs_calibrate_fops);
	debugfs_create_file("impedance", 0666, algo->dbg_dir,
			i2c, &smartpa_dbgfs_impedance_fops);
	debugfs_create_file("f0detect", 0666, algo->dbg_dir,
			i2c, &smartpa_dbgfs_f0_fops);
	debugfs_create_file("QFactor", 0666, algo->dbg_dir,
			i2c, &smartpa_dbgfs_QFactor_fops);
	debugfs_create_file("temperature", 0666, algo->dbg_dir,
			i2c, &smartpa_dbgfs_temperature_fops);
}

static void smartpa_debug_remove(smartpa_algo_data_t *algo)
{
	debugfs_remove_recursive(algo->dbg_dir);
}

#endif //CONFIG_DEBUG_FS

static int smartpa_calib_save(uint32_t *calib_value)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;
	uint8_t channels  = 1;

	if (!s_smartpa_algo || !calib_value) {
		pr_err("[SmartPA-%d]: SmartPA_priv or calib_value is NULL\n",
			__LINE__);
			ret = -1;
			return ret;
	}
	channels = s_smartpa_algo->mn_channels;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(CALIBRATE_FILE, O_RDWR | O_CREAT, 0666);
	if (!IS_ERR(pfile)) {
		pr_info("[SmartPA-%d]smartpa_calib_save: save calib_value[0]=%d\n",
			__LINE__, calib_value[0]);
		if (channels == 2)
			pr_info("[SmartPA-%d]smartpa_calib_save: save calib_value[1]=%d\n",
				__LINE__, calib_value[1]);
		vfs_write(pfile, (char *)calib_value, sizeof(uint32_t)*channels,
			&pos);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]smartpa_calib_save: %s open failed!\n",
			__LINE__, CALIBRATE_FILE);
		ret = -1;
	}

	set_fs(old_fs);

	return ret;
}

int smartpa_init_dbg(char *buffer, int size)
{
	uint32_t calib_re[MAX_CHANNELS] = {0};
	uint32_t paramid = 0;
	int ret = 0, n = 0;
	uint32_t data = 0;
	bool done[MAX_CHANNELS] = {false};
	int nSize = sizeof(uint32_t);
	uint8_t iter = 0, channels = 1;

	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);

	if (!s_smartpa_algo) {
		pr_err("[SmartPA-%d]: s_smartpa_algo is NULL\n", __LINE__);
		ret = -1;
		return ret;
	}
	channels = s_smartpa_algo->mn_channels;
	if (channels == 1)
		done[1] = true;

	if (1) { //s_smartpa_algo->mb_power_up) {
		//calib init
		for (iter = 0; iter < channels; iter++) {
			data = 1;//Value is ignored
			paramid = ((TAS_SA_CALIB_INIT)|((iter+1)<<24)|(1<<16));
			ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data,
							paramid,
							TAS_SET_PARAM, nSize,
							TISA_MOD_RX);
			if (ret < 0) {
				done[iter] = false;
				pr_info("[SmartPA-%d]init_dbg:set calib_data error.\n",
					__LINE__);
				ret = -ENOMEM;
			}
		}
		pr_info("[SmartPA-%d]init_dbg: calib init\n", __LINE__);

		msleep(3 * 1000);

		//get Re
		for (iter = 0; iter < channels; iter++) {
			data = 0;//Reset data to 0
			paramid = ((TAS_SA_CALIB_DEINIT)|
					((iter+1)<<24)|(1<<16));
			ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
				TAS_GET_PARAM, /*length */ 4,
				TISA_MOD_RX);
			if (ret < 0) {
				done[iter] = false;
				pr_info("[SmartPA-%d]init_dbg: decalib init\n",
					__LINE__);
			} else {
				calib_re[iter] = data;

				if ((calib_re[iter] < s_smartpa_algo->imped_min[iter]) ||
					(calib_re[iter] > s_smartpa_algo->imped_max[iter]))
					done[iter] = false;
				else
					done[iter] = true;
				pr_info("[SmartPA-%d]init_dbg: calib_re is %d, valid range (%d %d)\n",
					__LINE__, calib_re[iter],
					s_smartpa_algo->imped_min[iter],
					s_smartpa_algo->imped_max[iter]);
			}
		}
	} else {
		done[0] = false;
		if (channels == 2)
			done[1] = false;
		ret = -EINVAL;
		pr_info("[SmartPA-%d]dbg init: failed to calibrate %d\n",
			__LINE__, ret);
	}

	n += scnprintf(buffer + n, size - n, "current status:[SmartPA] %s\n",
		(channels == 1) ? "Mono" : "Stereo");
	for (iter = 0; iter < channels; iter++) {
		n += scnprintf(buffer + n, size - n,
			"Channel[%d]: impedance %02d.%02d ohm, valid range(%02d.%02d ~ %02d.%02d ohm).\n",
			iter,
			TRANSF_IMPED_TO_USER_I(calib_re[iter]),
			TRANSF_IMPED_TO_USER_M(calib_re[iter]),
			TRANSF_IMPED_TO_USER_I(s_smartpa_algo->imped_min[iter]),
			TRANSF_IMPED_TO_USER_M(s_smartpa_algo->imped_min[iter]),
			TRANSF_IMPED_TO_USER_I(s_smartpa_algo->imped_max[iter]),
			TRANSF_IMPED_TO_USER_M(s_smartpa_algo->imped_max[iter]));
		pr_info("[SmartPA-%d]init_dbg: calibRe[%d] %d\n",
			__LINE__, iter, calib_re[iter]);
		if (!done[iter])
			calib_re[iter] = CALIB_FAILED;
	}
	n += scnprintf(buffer + n, size - n, "\n Calibrate result: %s\n",
		(done[0] && done[1]) ? "OKAY(impedance ok)." : "ERROR!");
	buffer[n] = 0;

	pr_info("[SmartPA-%d]init_dbg: write to file\n", __LINE__);

	s_smartpa_algo->calibRe[0] = calib_re[0];
	s_smartpa_algo->calibRe[1] = calib_re[1];
	pr_info("[SmartPA-%d]init_dbg: update Re value\n", __LINE__);
	smartpa_calib_save(calib_re);

//deinit:
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Value is ignored
		paramid  = ((TAS_SA_CALIB_DEINIT)|((iter+1)<<24)|(1<<16));
		ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data, paramid,
			TAS_SET_PARAM, nSize, TISA_MOD_RX);
		pr_info("[SmartPA-%d]init_dbg: decalib init\n", __LINE__);
	}
//end:
	pr_info("[SmartPA-%d]init_dbg: end\n", __LINE__);

	if (done[0] && done[1])
		ret = 0;
	else
		ret = -1;
	return ret;
}

static int smartpa_freq_save(char *buffer, int count)
{
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	loff_t pos = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	pfile = filp_open(FREQ_FILE, O_RDWR | O_CREAT, 0666);
	if (!IS_ERR(pfile)) {
		pr_info("[SmartPA-%d]freq: save count=%d\n", __LINE__, count);
		vfs_write(pfile, buffer, count, &pos);
		filp_close(pfile, NULL);
	} else {
		pr_info("[SmartPA-%d]freq: %s open failed!\n", __LINE__,
			FREQ_FILE);
		ret = -1;
		ret = ret;
	}

	set_fs(old_fs);

	return ret;
}

int smartpa_read_freq_dbg(char *buffer, int size)
{
	u32 length = 1;
	uint32_t calibRe[MAX_CHANNELS] = {0};
	uint32_t F0[MAX_CHANNELS] = {0}, Q[MAX_CHANNELS] = {0};
	int ret = 0, n = 0, i[MAX_CHANNELS] = {0}, j[MAX_CHANNELS] = {0};
	uint32_t data = 0;
	uint32_t paramid = 0;
	int nSize = sizeof(uint32_t);
	uint8_t iter = 0, channels = 1;

	pr_info("[SmartPA-%d]: enter %s\n", __LINE__, __func__);

	if (!s_smartpa_algo) {
		pr_err("[SmartPA-%d]: s_smartpa_algo is NULL\n", __LINE__);
		ret = -1;
		return ret;
	}
	channels = s_smartpa_algo->mn_channels;

	//Load Calib
	if (smartpa_check_re()) {

		for (iter = 0; iter < channels; iter++)
			calibRe[iter] = s_smartpa_algo->calibRe[iter];
		smartpa_set_re(calibRe);
	}

	for (iter = 0; iter < channels; iter++) {
		data = 1;//Value is ignored
		paramid = ((TAS_SA_F0_TEST_INIT) | (length << 16) |
			((iter+1) << 24));
		ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data, paramid,
				TAS_SET_PARAM, nSize, TISA_MOD_RX);
	}
	//wait 5s
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
			//read F0
			data = 0;//Reset data to 0
			paramid = (TAS_SA_GET_F0 | (length << 16) |
				((iter+1) << 24));
			ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
					TAS_GET_PARAM, /*length **/ 4,
					TISA_MOD_RX);
			F0[iter] = data;

			//read Q
			data = 0;//Reset data to 0
			paramid = (TAS_SA_GET_Q | (length << 16) |
				((iter+1) << 24));
			ret = tas25xx_smartamp_algo_ctrl((u8 *)&data, paramid,
					TAS_GET_PARAM, /*length **/ 4,
					TISA_MOD_RX);
			Q[iter] = data;

			if (((F0[iter] >> 19) < s_smartpa_algo->fres_min[iter])
				|| ((F0[iter] >> 19) > s_smartpa_algo->fres_max[iter])
				|| (((Q[iter] * 100) >> 19) < s_smartpa_algo->Qt[iter]))
				j[iter] = 0;
			else
				j[iter]++;

			pr_info("[SmartPA-%d]read freq dbg channel[%d]: f0 = %d Q = %d i = %d j = %d\n",
					__LINE__, iter, F0[iter],
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

	ret = smartpa_freq_save(buffer, n);
	buffer[n] = 0;

//deinit :
	for (iter = 0; iter < channels; iter++) {
		data = 0;//Value is ignored
		paramid = ((TAS_SA_F0_TEST_DEINIT) |
			(length << 16) |
			((iter+1) << 24));
		ret = tas25xx_smartamp_algo_ctrl((uint8_t *)&data, paramid,
			TAS_SET_PARAM, nSize, TISA_MOD_RX);
	}
//end:
	return 0;
}

void smartpa_read_prars_dbg(int temp[5], unsigned char addr)
{
	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);
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
	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);

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

	pr_info("[SmartPA-%d]: %s enter.\n", __LINE__, __func__);

	smartpa_calib_get(impedance);
	channels = s_smartpa_algo->mn_channels;
	for (iter = 0; iter < channels; iter++)
		ret &= rdc_check_valid(impedance[iter], iter);

	return ret;
}

int tas25xx_parse_algo_dt_debugfs(struct device_node *np)
{
	int temp, ret = 0;

	if (!s_smartpa_algo) {
		pr_err("[SmartPA-%d]: s_smartpa_algo is NULL\n", __LINE__);
		return -ENOMEM;
	}

	ret = of_property_read_u32(np, "vivo,impedance-min", &temp);
	s_smartpa_algo->imped_min[0] = (!ret) ? (int)temp : RDC_MIN_L;

	ret = of_property_read_u32(np, "vivo,impedance-max", &temp);
	s_smartpa_algo->imped_max[0] = (!ret) ? (int)temp : RDC_MAX_L;

	ret = of_property_read_u32(np, "vivo,frequency-min", &temp);
	s_smartpa_algo->fres_min[0] = (!ret) ? (int)temp : 500;

	ret = of_property_read_u32(np, "vivo,frequency-max", &temp);
	s_smartpa_algo->fres_max[0] = (!ret) ? (int)temp : 1100;

	ret = of_property_read_u32(np, "vivo,Qt-min", &temp);
	s_smartpa_algo->Qt[0] = (!ret) ? (int)temp : 100;

	if (s_smartpa_algo->mn_channels == 2) {
		ret = of_property_read_u32(np, "vivo,impedance-min", &temp);
		s_smartpa_algo->imped_min[1] = (!ret) ? (int)temp : RDC_MIN_R;

		ret = of_property_read_u32(np, "vivo,impedance-max", &temp);
		s_smartpa_algo->imped_max[1] = (!ret) ? (int)temp : RDC_MAX_R;

		ret = of_property_read_u32(np, "vivo,frequency-min", &temp);
		s_smartpa_algo->fres_min[1] = (!ret) ? (int)temp : 500;

		ret = of_property_read_u32(np, "vivo,frequency-max", &temp);
		s_smartpa_algo->fres_max[1] = (!ret) ? (int)temp : 1100;

		ret = of_property_read_u32(np, "vivo,Qt-min", &temp);
		s_smartpa_algo->Qt[1] = (!ret) ? (int)temp : 100;
	}

	return 0;
}

void tas25xx_send_algo_calibration(void)
{
	if (smartpa_check_re()) {
		smartpa_set_re(s_smartpa_algo->calibRe);
		pr_info("[SmartPA-%d] SetRe[0] called %d(0x%x)", __LINE__,
			s_smartpa_algo->calibRe[0], s_smartpa_algo->calibRe[0]);
		if (s_smartpa_algo->mn_channels == 2)
			pr_info("[SmartPA-%d] SetRe[1] called %d(0x%x)",
				__LINE__, s_smartpa_algo->calibRe[1],
				s_smartpa_algo->calibRe[1]);
	} else {
		pr_err("[SmartPA-%d] SetRe is not called", __LINE__);
	}
}

static int alloc_memory_for_smartpa_algo_client(void)
{
	if (!s_smartpa_algo) {
		int size = sizeof(smartpa_algo_data_t);

		s_smartpa_algo = kmalloc(size, GFP_KERNEL);
		memset(s_smartpa_algo, 0, sizeof(smartpa_algo_data_t));
	}

	if (!s_smartpa_algo)
		return -ENOMEM;

	return 0;
}
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
int tas_smartamp_add_algo_controls_debugfs(struct snd_soc_component *c,
	int number_of_channels)
#else
int tas_smartamp_add_algo_controls_debugfs(struct snd_soc_codec *c,
	int number_of_channels)
#endif
{
	int ret = alloc_memory_for_smartpa_algo_client();

	if (ret)
		return ret;

	s_smartpa_algo->mn_channels = number_of_channels;

#ifdef CONFIG_DEBUG_FS
	smartpa_debug_init(s_smartpa_algo);
#endif

	return 0;
}
#if KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE
void tas_smartamp_remove_algo_controls_debugfs(struct snd_soc_component *c)
#else
void tas_smartamp_remove_algo_controls_debugfs(struct snd_soc_codec *c)
#endif
{
	if (s_smartpa_algo) {
		smartpa_debug_remove(s_smartpa_algo);

		kfree(s_smartpa_algo);
		s_smartpa_algo = NULL;
	}
}
