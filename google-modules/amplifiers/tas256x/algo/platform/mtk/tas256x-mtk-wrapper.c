#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include "algo/inc/tas_smart_amp_v2.h"

#define TAS_PAYLOAD_SIZE	14

struct mtk_apr {
	uint32_t param_id;
	uint32_t data[TAS_PAYLOAD_SIZE];
};

int tas25xx_smartamp_get_set(uint8_t *data_buff, uint32_t param_id,
	uint8_t get_set, uint16_t length, module_id_t module_id)
{
	int32_t ret = 0;
	int32_t rd_length = 0;
	struct mtk_apr apr_buff = {0};
	int32_t i = 0;
	(void)module_id;

	pr_err("[TI-SmartPA:%s] get_set %d param_id %d length %d\n",
		__func__, get_set, param_id, length);
	if (length > TAS_PAYLOAD_SIZE*4) {
		pr_err("[TI-SmartPA:%s] Out of bound length %d\n", length);
		ret = -1;
		return ret;
	}

	switch (get_set) {
	case TAS_SET_PARAM:
		{
			apr_buff.param_id = param_id;
			pr_err("[TI-SmartPA:%s] TAS_SET_PARAM param_id %d\n",
				__func__, param_id);
			memcpy(apr_buff.data, data_buff, length);
			ret = mtk_spk_send_ipi_buf_to_dsp((void *)&apr_buff,
				length+sizeof(param_id)); //give whole buff size
		}
		break;
	case TAS_GET_PARAM:
		{
			apr_buff.param_id = param_id;
			pr_err("[TI-SmartPA:%s] TAS_GET_PARAM param_id %d\n",
				__func__, param_id);
			memset(apr_buff.data, 0, length);
			/* update param_id firstly, since param_id
			 * can not be sent by get_buf
			 */
			ret = mtk_spk_send_ipi_buf_to_dsp((void *)&apr_buff,
				sizeof(param_id));
			if (ret == 0) {
				ret = mtk_spk_recv_ipi_buf_from_dsp(
					(void *)&apr_buff,
					length+sizeof(param_id), &rd_length);
				pr_err("[TI-SmartPA:%s] TAS_GET rd_length %d\n",
					__func__, rd_length);
				if ((ret == 0) &&
					(rd_length <= TAS_PAYLOAD_SIZE*4+sizeof(param_id))) {
					memcpy(data_buff, apr_buff.data, rd_length-sizeof(param_id));
				}
			}
			//For Debug
			for (i = 0; i < length/4; i++)
				pr_err("[TI-SmartPA:%s] apr_buff.data[%d] = 0x%0x\n",
					__func__, i, apr_buff.data[i]);
		}
		break;
	}

	return ret;
}

