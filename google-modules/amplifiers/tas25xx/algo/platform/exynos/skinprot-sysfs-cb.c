#include <sound/soc.h>
#include <sound/samsung/sec_audio_sysfs.h>
#include "../../src/tas25xx-calib-validation.h"

struct snd_soc_component *s_tas25xx_component;

static int tas_get_coil_temp(enum amp_id id)
{
	int32_t ret = -EINVAL;
	struct snd_soc_component *component = s_tas25xx_component;

	if (!component) {
		pr_err("TI-SmartPA %s: invalid component registered\n", __func__);
		return -EPERM;
	}

	if (id >=  AMP_ID_MAX) {
		dev_err(component->dev, "%s: invalid id(%d)\n", __func__, id);
		return ret;
	}
	return tas_get_coil_temp_nocheck((int)id);
}

int tas_set_surface_temp(enum amp_id id, int temperature)
{
	struct snd_soc_component *component = s_tas25xx_component;

	if (!component) {
		pr_err("TI-SmartPA %s: component NULL\n", __func__);
		return -EPERM;
	}

	if (id >=  AMP_ID_MAX) {
		pr_err("TI-SmartPA: %s invalid id(%d)\n", __func__, id);
		return -EINVAL;
	}
	return tas_set_surface_temp_nocheck((int)id, temperature);
}

void register_tas25xx_cb_component(struct snd_soc_component *component)
{
	s_tas25xx_component = component;
	pr_info("TI-SmartPA: %s registering skin prot interfaces.. \n", __func__);

	audio_register_curr_temperature_cb(tas_get_coil_temp);
	audio_register_surface_temperature_cb(tas_set_surface_temp);
}
EXPORT_SYMBOL_GPL(register_tas25xx_cb_component);
