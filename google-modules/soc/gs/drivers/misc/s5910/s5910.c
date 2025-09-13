// SPDX-License-Identifier: GPL-2.0-only

#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spmi.h>
#include <linux/s5910.h>

#define S5910_RETRY_COUNT 3

struct s5910_seq_step {
	u32 delay;
	u32 reg;
	u32 val;
};

struct s5910_info {
	struct spmi_device	*sdev;
	struct regmap		*regmap;
	struct s5910_seq_step   *off_sequence;
	struct s5910_seq_step   *on_sequence;
	size_t			off_step_cnt;
	size_t			on_step_cnt;
};

static int of_dev_node_match(struct device *dev, const void *node)
{
	return dev->of_node == node;
}

static const struct regmap_range s5910_wr_range[] = {
	regmap_reg_range(0x0000, 0x0008),
	regmap_reg_range(0x0190, 0x0194),
	regmap_reg_range(0x0190, 0x0194),
	regmap_reg_range(0x0201, 0x0201),
	regmap_reg_range(0x0207, 0x0207),
	regmap_reg_range(0x0209, 0x0209),
	regmap_reg_range(0x020a, 0x020e),
	regmap_reg_range(0x0211, 0x0244),
	regmap_reg_range(0x0275, 0x027e),
	regmap_reg_range(0x027f, 0x029a),
};

static const struct regmap_access_table s5910_wr_table = {
	.yes_ranges = s5910_wr_range,
	.n_yes_ranges = ARRAY_SIZE(s5910_wr_range),
};

static const struct regmap_range s5910_rd_range[] = {
	regmap_reg_range(0x0000, 0x0008),
	regmap_reg_range(0x0190, 0x0194),
	regmap_reg_range(0x0190, 0x0194),
	regmap_reg_range(0x0200, 0x0201),
	regmap_reg_range(0x0207, 0x0207),
	regmap_reg_range(0x0209, 0x0209),
	regmap_reg_range(0x020a, 0x020e),
	regmap_reg_range(0x0211, 0x0244),
	regmap_reg_range(0x0275, 0x027e),
	regmap_reg_range(0x027f, 0x029a),
};

static const struct regmap_access_table s5910_rd_table = {
	.yes_ranges = s5910_rd_range,
	.n_yes_ranges = ARRAY_SIZE(s5910_rd_range),
};

static struct regmap_config s5910_regmap_cfg = {
	.name = "s5910_clock_buffer",
	.reg_bits = 16,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.max_register = 0x29a,
	.wr_table = &s5910_wr_table,
	.rd_table = &s5910_rd_table,
};

static int s5910_dt_init(struct device *dev, struct s5910_info *info)
{
	struct property *prop;
	size_t num_bytes;
	size_t num_cells;
	size_t num_steps;
	struct s5910_seq_step *lseq, *oseq;
	size_t step_ndx;
	size_t cell_ndx;
	u32 *val;

	if (!dev->of_node) {
		dev_err(dev, "No of_node\n");
		return -ENOENT;
	}

	prop = of_find_property(dev->of_node, "s5910,seq", NULL);
	if (!prop) {
		dev_err(dev, "Unable to read s5910,seq\n");
		return -ENODATA;
	}

	num_bytes = prop->length;
	num_cells = num_bytes / sizeof(u32);
	num_steps = num_cells / 3;

	lseq = devm_kzalloc(dev,
			num_steps*sizeof(struct s5910_seq_step),
			GFP_KERNEL);
	if (!lseq)
		return -ENOMEM;

	val = prop->value;
	if (!val) {
		dev_err(dev, "Invalid s5910,seq value\n");
		return -EINVAL;
	}

	for (step_ndx = 0; step_ndx < num_steps; step_ndx++) {
		cell_ndx = step_ndx * 3;

		lseq[step_ndx].delay = be32_to_cpu(val[cell_ndx + 0]);
		lseq[step_ndx].reg = be32_to_cpu(val[cell_ndx + 1]);
		lseq[step_ndx].val = be32_to_cpu(val[cell_ndx + 2]);
	}
	info->off_sequence = lseq;
	info->off_step_cnt = num_steps;

	prop = of_find_property(dev->of_node, "s5910,on_seq", NULL);
	if (!prop) {
		dev_err(dev, "Unable to read s5910,on_seq\n");
		return -ENODATA;
	}

	num_bytes = prop->length;
	num_cells = num_bytes / sizeof(u32);
	num_steps = num_cells / 3;

	oseq = devm_kzalloc(dev,
			num_steps*sizeof(struct s5910_seq_step),
			GFP_KERNEL);
	if (!oseq)
		return -ENOMEM;

	val = prop->value;
	if (!val) {
		dev_err(dev, "Invalid s5910,on_seq value\n");
		return -EINVAL;
	}

	for (step_ndx = 0; step_ndx < num_steps; step_ndx++) {
		cell_ndx = step_ndx * 3;

		oseq[step_ndx].delay = be32_to_cpu(val[cell_ndx + 0]);
		oseq[step_ndx].reg = be32_to_cpu(val[cell_ndx + 1]);
		oseq[step_ndx].val = be32_to_cpu(val[cell_ndx + 2]);
	}
	info->on_sequence = oseq;
	info->on_step_cnt = num_steps;

	return 0;
}

static int s5910_probe(struct spmi_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct s5910_info *info;
	size_t step_ndx;
	int rc;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	rc = s5910_dt_init(dev, info);
	if (!rc) {
		for (step_ndx = 0; step_ndx < info->off_step_cnt; step_ndx++) {
			dev_info(dev, "off: %zd %d %#03x %#02x\n", step_ndx,
					info->off_sequence[step_ndx].delay,
					info->off_sequence[step_ndx].reg,
					info->off_sequence[step_ndx].val);
		}

		for (step_ndx = 0; step_ndx < info->on_step_cnt; step_ndx++) {
			dev_info(dev, "on: %zd %d %#03x %#02x\n", step_ndx,
					info->on_sequence[step_ndx].delay,
					info->on_sequence[step_ndx].reg,
					info->on_sequence[step_ndx].val);
		}
	}

	spmi_device_set_drvdata(sdev, info);
	info->sdev = sdev;

	info->regmap = devm_regmap_init_spmi_ext(sdev, &s5910_regmap_cfg);
	if (IS_ERR(&info->regmap)) {
		dev_err(&sdev->dev, "Failed to initialize regmap\n");
		return -EINVAL;
	}

	return 0;
}

static const struct of_device_id s5910_match_table[] = {
	{
		.compatible = "google,s5910-spmi",
	},
	{}
};
MODULE_DEVICE_TABLE(of, s5910_match_table);

static struct spmi_driver s5910_driver = {
	.probe		= s5910_probe,
	.driver		= {
		.name	= "s5910_spmi_driver",
		.of_match_table = s5910_match_table,
	},
};

static int s5910_write(struct device *dev, u32 reg, u32 val)
{
	struct s5910_info *info = dev_get_drvdata(dev);
	int rc = -EIO;
	int i;
	u32 chk_val;

	for (i = 0; i < S5910_RETRY_COUNT; i++) {
		rc = regmap_write(info->regmap, reg, val);
		if (rc < 0) {
			dev_err(dev, "ERROR: write to %#03x failed (%d)\n",
					reg, rc);
			continue;
		}
		rc = regmap_read(info->regmap, reg, &chk_val);
		if (rc < 0) {
			dev_err(dev, "ERROR: read from %#03x failed (%d)\n",
					reg, rc);
			continue;
		}

		if (val != chk_val) {
			dev_err(dev, "ERROR: read value doesn't match written %#03x: wrote %#02x, expected %#02x\n",
					reg, val, chk_val);
			rc = -EIO;
			continue;
		}
		return 0;
	}

	return rc;
}

int s5910_shutdown_sequence(struct device *dev)
{
	struct s5910_info *info = dev_get_drvdata(dev);
	size_t ndx;
	u32 delay_ms; /* delay before sending cmd */
	u32 reg; /* target register */
	u32 val; /* value to write */

	for (ndx = 0; ndx < info->off_step_cnt; ndx++) {
		delay_ms = info->off_sequence[ndx].delay;
		reg = info->off_sequence[ndx].reg;
		val = info->off_sequence[ndx].val;

		dev_info(dev, "%zu %d %#03x %#02x\n", ndx, delay_ms, reg, val);
		if (delay_ms > 0)
			msleep(delay_ms);
		s5910_write(dev, reg, val);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(s5910_shutdown_sequence);

int s5910_turn_on_sequence(struct device *dev)
{
	struct s5910_info *info = dev_get_drvdata(dev);
	size_t ndx;
	u32 delay_ms; /* delay before sending cmd */
	u32 reg; /* target register */
	u32 val; /* value to write */

	for (ndx = 0; ndx < info->on_step_cnt; ndx++) {
		delay_ms = info->on_sequence[ndx].delay;
		reg = info->on_sequence[ndx].reg;
		val = info->on_sequence[ndx].val;

		dev_info(dev, "%zu %d %#03x %#02x\n", ndx, delay_ms, reg, val);
		if (delay_ms > 0)
			msleep(delay_ms);
		s5910_write(dev, reg, val);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(s5910_turn_on_sequence);

int s5910_check_lpm_mode(struct device *dev)
{
	int rc;
	u32 val;

	struct s5910_info *info = dev_get_drvdata(dev);
	rc = regmap_read(info->regmap, 0x020e, &val);
	if (rc < 0) {
		dev_err(dev, "ERROR: read LPM mode failed (%d)\n", rc);
		return rc;
	}

	dev_info(dev, "s5910 DCXO LPM mode: %#02x\n", val);
	return 0;
}
EXPORT_SYMBOL_GPL(s5910_check_lpm_mode);

struct device *s5910_get_device(struct device_node *node)
{
	struct device *dev = NULL;
	struct bus_type *sbt = s5910_driver.driver.bus;

	if (sbt)
		dev = bus_find_device(sbt, NULL, node, of_dev_node_match);

	return dev;
}
EXPORT_SYMBOL_GPL(s5910_get_device);

struct spmi_device *s5910_get_spmi_device(struct device_node *node)
{
	struct device *dev;
	struct spmi_device *sdev = NULL;

	dev = s5910_get_device(node);
	if (dev)
		sdev = to_spmi_device(dev);
	return sdev;
}
EXPORT_SYMBOL_GPL(s5910_get_spmi_device);

static int __init s5910_init(void)
{
	int rv = 0;

	spmi_driver_register(&s5910_driver);
	return rv;
}
module_init(s5910_init);

static void __exit s5910_exit(void)
{
	spmi_driver_unregister(&s5910_driver);
}
module_exit(s5910_exit);

MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Jim Wylder<jwylder@google.com>");
