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
#include "algo/src/tas25xx-algo-intf.h"
#include "mtk-sp-spk-amp.h"

#define TAS_PAYLOAD_SIZE	14


struct mtk_apr {
	uint32_t param_id;
	uint32_t data[TAS_PAYLOAD_SIZE];
};

int tas25xx_smartamp_get_set(uint8_t *data_buff, uint32_t param_id,
	uint8_t get_set, uint32_t length, enum module_id_t module_id)
{
	int32_t ret = 0;
	int32_t actual_rd_sz = 0;
	struct mtk_apr apr_buff = {0};
	int32_t i = 0;
	uint32_t max_payload_sz = TAS_PAYLOAD_SIZE*sizeof(uint32_t);
	uint32_t max_buf_sz = max_payload_sz + sizeof(param_id);
	uint32_t rdwr_buf_sz = length + sizeof(param_id);
	(void)module_id;

	pr_info("[TI-SmartPA:%s] get_set %d param_id %d length %d\n",
		__func__, get_set, param_id, length);

	if (length > max_payload_sz) {
		pr_err("[TI-SmartPA:%s] Out of bound length %d\n", length);
		ret = -EINVAL;
		return ret;
	}

	switch (get_set) {
	case TAS_SET_PARAM:
		{
			apr_buff.param_id = param_id;
			pr_info("[TI-SmartPA:%s] TAS_SET_PARAM param_id %d\n",
				__func__, param_id);
			memcpy(apr_buff.data, data_buff, length);
			ret = mtk_spk_send_ipi_buf_to_dsp((void *)&apr_buff,
				rdwr_buf_sz);
		}
		break;

	case TAS_GET_PARAM:
		{
			apr_buff.param_id = param_id;
			pr_debug("[TI-SmartPA:%s] TAS_GET_PARAM param_id %d\n",
				__func__, param_id);
			memset(apr_buff.data, 0, length);
			/* update param_id firstly, since param_id
			 * can not be sent by get_buf
			 */
			ret = mtk_spk_send_ipi_buf_to_dsp((void *)&apr_buff, sizeof(param_id));
			if (ret == 0) {
				ret = mtk_spk_recv_ipi_buf_from_dsp((void *)&apr_buff,
					rdwr_buf_sz, &actual_rd_sz);
				pr_info("[TI-SmartPA:%s] TAS_GET actual_rd_sz %d\n",
					__func__, actual_rd_sz);
				if ((ret == 0) && (actual_rd_sz <= max_buf_sz)) {
					memcpy(data_buff, apr_buff.data,
						actual_rd_sz - sizeof(param_id));
				} else {
					/* invalid length */
					ret = -EINVAL;
				}
			}
			for (i = 0; i < length/4; i++)
				pr_info("[TI-SmartPA:%s] apr_buff.data[%d] = 0x%0x\n",
					__func__, i, apr_buff.data[i]);
		}
		break;
	}

	return ret;
}
