// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/spmi.h>

#define SPMI_CMD_ADDRESS_MASK  0x0f
#define SPMI_CMD_ADDRESS_SHIFT    8
#define SPMI_CMD_PAYLOAD_MASK  0xff
#define SPMI_CMD_PAYLOAD_SHIFT    0
#define SPMI_CMD_LEN             12  /* num bits excluding parity */

#define SPMI_DATA_LEN             8  /* num bits excluding parity */

#define DEFAULT_CLK_DELAY_US    100

struct spmi_bb_info {
	struct spmi_controller	*ctrl;
	struct device		*dev;
	struct gpio_desc        *gpio_clk;
	struct gpio_desc        *gpio_dat;
	unsigned long		delay_us;
};

static inline void gpio_set_clk_out(struct spmi_bb_info *info, int value)
{
	gpiod_direction_output(info->gpio_clk, value);
}

static inline void gpio_set_clk_in(struct spmi_bb_info *info)
{
	gpiod_direction_input(info->gpio_clk);
}

static inline void gpio_set_dat_out(struct spmi_bb_info *info, int value)
{
	gpiod_direction_output(info->gpio_dat, value);
}

static inline void gpio_set_dat_in(struct spmi_bb_info *info)
{
	gpiod_direction_input(info->gpio_dat);
}

static inline void spmi_enable(struct spmi_bb_info *info)
{
	gpio_set_clk_out(info, 0);
	gpio_set_dat_out(info, 0);
}

static inline void spmi_disable(struct spmi_bb_info *info)
{
	gpio_set_clk_in(info);
	gpio_set_dat_in(info);
}

static inline void spmi_clk_delay(struct spmi_bb_info *info)
{
	udelay(info->delay_us);
}

static inline void spmi_set_clk(struct spmi_bb_info *info, bool high)
{
	gpiod_set_value_cansleep(info->gpio_clk, (int)high);
}

static inline void spmi_set_dat(struct spmi_bb_info *info, bool high)
{
	gpiod_set_value_cansleep(info->gpio_dat, (int)high);
}

static inline bool spmi_get_dat(struct spmi_bb_info *info)
{
	return !!gpiod_get_value(info->gpio_dat);
}

static void spmi_send(struct spmi_bb_info *info, u32 data, u32 num_bits)
{
	u32 bit = 0;
	int mask = BIT(num_bits); /* start one too high */

	spmi_set_clk(info, 0);
	while (mask >>= 1) {
		bit = data & mask;

		spmi_set_clk(info, 1);
		spmi_set_dat(info, bit);
		spmi_clk_delay(info);
		spmi_set_clk(info, 0);
		spmi_clk_delay(info);
	}

}

static void spmi_recv(struct spmi_bb_info *info, u32 num_bits, u32 *data)
{
	*data = 0;

	spmi_set_clk(info, 0);
	while (num_bits--) {
		spmi_set_clk(info, 1);
		*data <<= 1;
		*data |= spmi_get_dat(info);
		spmi_clk_delay(info);
		spmi_set_clk(info, 0);
		spmi_clk_delay(info);
	}
}

static bool spmi_calc_parity(u32 data, u32 num_bits)
{
	bool parity = 1;

	while (num_bits--) {
		parity ^= data & 0x1;
		data = data >> 1;
	}
	return parity;
}

static void spmi_send_frame(struct spmi_bb_info *info, u32 data, u8 num_bits)
{
	u32 parity = spmi_calc_parity(data, num_bits);
	u32 data_p = data << 1 | (parity & 0x01);

	spmi_send(info, data_p, num_bits + 1);
}

static bool spmi_recv_frame(struct spmi_bb_info *info, u32 *data, u32 num_bits)
{
	bool parity;
	bool calc_parity;

	spmi_recv(info, num_bits + 1, data);
	parity = *data & 0x1;
	*data = *data >> 1;
	calc_parity = spmi_calc_parity(*data, num_bits);
	return parity == calc_parity;
}

static void spmi_send_ssc(struct spmi_bb_info *info)
{
	gpio_set_dat_out(info, 0);
	spmi_set_clk(info, 0);
	spmi_set_dat(info, 1);
	spmi_clk_delay(info);
	spmi_set_dat(info, 0);
	spmi_clk_delay(info);
}

static void spmi_send_command_frame(struct spmi_bb_info *info,
		u8 addr, u8 payload)
{
	u16 data = (u16)
		((addr & SPMI_CMD_ADDRESS_MASK) << SPMI_CMD_ADDRESS_SHIFT |
		 (payload & SPMI_CMD_PAYLOAD_MASK) << SPMI_CMD_PAYLOAD_SHIFT);

	spmi_send_frame(info, data, SPMI_CMD_LEN);
}

static inline void spmi_send_data_frame(struct spmi_bb_info *info, u8 payload)
{
	spmi_send_frame(info, payload, SPMI_DATA_LEN);
}

/* return 1 (true) if parity matched */
static inline bool spmi_recv_data_frame(struct spmi_bb_info *info, u8 *payload)
{
	u32 data;

	bool ret = spmi_recv_frame(info, &data, SPMI_DATA_LEN);
	*payload = (u8) data;
	return ret;
}

static void spmi_bus_park_cycle(struct spmi_bb_info *info)
{
	spmi_set_clk(info, 1);
	spmi_set_dat(info, 0);
	spmi_clk_delay(info);
	gpio_set_dat_in(info);
	spmi_set_clk(info, 0);
	spmi_clk_delay(info);
}

static bool spmi_recv_ack_nak_one_device(struct spmi_bb_info *info)
{
	bool bit;

	spmi_bus_park_cycle(info);
	spmi_set_clk(info, 1);
	spmi_clk_delay(info);
	spmi_set_clk(info, 0);
	bit = spmi_get_dat(info);
	spmi_clk_delay(info);
	spmi_bus_park_cycle(info);

	return bit;
}

static bool spmi_cmd_seq_extended_register_write(struct spmi_bb_info *info,
	u8 sid, u8 reg, const u8 *val, u8 bytes)
{
	u32 i;
	u8 command = SPMI_CMD_EXT_WRITE | (bytes - 1 & 0x0f);

	spmi_send_ssc(info);
	spmi_send_command_frame(info, sid, command);
	spmi_send_data_frame(info, reg);
	for (i = 0; i < bytes; i++)
		spmi_send_data_frame(info, val[i]);

	return spmi_recv_ack_nak_one_device(info);
}

static bool spmi_cmd_seq_extended_register_read(struct spmi_bb_info *info,
	u8 sid, u8 reg, u8 *val, u8 bytes)
{
	bool ret = true;
	u32 i;
	u8 command = SPMI_CMD_EXT_READ | (bytes - 1 & 0x0f);

	spmi_send_ssc(info);
	spmi_send_command_frame(info, sid, command);
	spmi_send_data_frame(info, reg);
	spmi_bus_park_cycle(info);
	for (i = 0; i < bytes; i++)
		ret &= spmi_recv_data_frame(info, (val + i));

	spmi_bus_park_cycle(info);
	return ret;
}

static bool spmi_cmd_seq_extended_register_write_long(struct spmi_bb_info *info, u8 sid,
	u16 reg, const u8 *val, u8 bytes)
{
	u32 i;
	u8 command = SPMI_CMD_EXT_WRITEL | ((bytes - 1) & 0x05);
	u8 upper_addr = (reg & 0xff00) >> 8;
	u8 lower_addr = (reg & 0x00ff);

	spmi_send_ssc(info);
	spmi_send_command_frame(info, sid, command);
	spmi_send_data_frame(info, upper_addr);
	spmi_send_data_frame(info, lower_addr);
	for (i = 0; i < bytes; i++)
		spmi_send_data_frame(info, val[i]);

	return spmi_recv_ack_nak_one_device(info);
}

static bool spmi_cmd_seq_extended_register_read_long(struct spmi_bb_info *info,
		u8 sid, u16 reg, u8 *val, u8 bytes)
{
	bool ret = true;
	u32 i;
	u8 command = SPMI_CMD_EXT_READL | ((bytes - 1) & 0x05);
	u8 upper_addr = (reg & 0xff00) >> 8;
	u8 lower_addr = (reg & 0x00ff);
	u8 val8 = 0;

	spmi_send_ssc(info);
	spmi_send_command_frame(info, sid, command);
	spmi_send_data_frame(info, upper_addr);
	spmi_send_data_frame(info, lower_addr);
	spmi_bus_park_cycle(info);
	for (i = 0; i < bytes; i++) {
		ret &= spmi_recv_data_frame(info, &val8);
		val[i] = val8;
	}
	spmi_bus_park_cycle(info);
	return ret;

}

static bool spmi_cmd_seq_register_write(struct spmi_bb_info *info,
		u8 sid, u8 reg, u8 val)
{
	u8 command = SPMI_CMD_WRITE | (reg & 0x1f);

	spmi_send_ssc(info);
	spmi_send_command_frame(info, sid, command);
	spmi_send_data_frame(info, val);
	return spmi_recv_ack_nak_one_device(info);
}

static bool spmi_cmd_seq_register_read(struct spmi_bb_info *info,
		u8 sid, u8 reg, u8 *val)
{
	u8 command = SPMI_CMD_READ | (reg & 0x1f);
	bool ret;

	spmi_send_ssc(info);
	spmi_send_command_frame(info, sid, command);
	spmi_bus_park_cycle(info);
	ret = spmi_recv_data_frame(info, val);
	if (ret)
		pr_err("%s %d", __func__, ret);
	spmi_bus_park_cycle(info);

	return val;
}

static void spmi_bus_arbitrate(struct spmi_bb_info *info)
{
	gpio_set_dat_out(info, 0);
	spmi_set_clk(info, 0);
	spmi_clk_delay(info);
	spmi_clk_delay(info);

	spmi_set_dat(info, 1);
	spmi_clk_delay(info);
	spmi_clk_delay(info);
	spmi_clk_delay(info);
	spmi_clk_delay(info);
	spmi_clk_delay(info);

	/* first clock */
	spmi_set_clk(info, 1);
	spmi_clk_delay(info);
	spmi_set_clk(info, 0);
	spmi_clk_delay(info);

	/* bus park cycle */
	spmi_set_clk(info, 1);
	spmi_set_dat(info, 0);
	spmi_clk_delay(info);
	spmi_set_clk(info, 0);
	spmi_clk_delay(info);

	/* c */
	spmi_set_clk(info, 1);
	spmi_set_dat(info, 1);
	spmi_clk_delay(info);
	spmi_set_clk(info, 0);
	spmi_clk_delay(info);

	/* a */
	spmi_set_clk(info, 1);
	spmi_set_dat(info, 0);
	gpio_set_dat_in(info);
	spmi_clk_delay(info);
	spmi_set_clk(info, 0);
	spmi_clk_delay(info);

	/* xtra clock */
	spmi_set_clk(info, 1);
	spmi_clk_delay(info);
	spmi_set_clk(info, 0);
	spmi_clk_delay(info);

	/* mpl0 */
	spmi_set_clk(info, 1);
	spmi_clk_delay(info);
	spmi_set_clk(info, 0);
	spmi_clk_delay(info);

	/* mpl1 */
	spmi_set_clk(info, 1);
	spmi_clk_delay(info);
	spmi_set_clk(info, 0);
	spmi_clk_delay(info);

	/* mpl2 */
	spmi_set_clk(info, 1);
	spmi_clk_delay(info);
	spmi_set_clk(info, 0);
	spmi_clk_delay(info);

	/* mpl3 */
	spmi_set_clk(info, 1);
	gpio_set_dat_out(info, 1);
	spmi_clk_delay(info);
	spmi_set_clk(info, 0);
	spmi_clk_delay(info);

	spmi_set_dat(info, 0);
}

static int spmi_bb_write_cmd(struct spmi_controller *ctrl,
		u8 opc, u8 sid, u16 addr, const u8 *buf, size_t len)
{
	int ret = 0;
	bool ack_nak_bit = true;
	struct spmi_bb_info *info = spmi_controller_get_drvdata(ctrl);

	spmi_enable(info);
	spmi_bus_arbitrate(info);
	switch (opc) {
	case SPMI_CMD_WRITE:
		ack_nak_bit = spmi_cmd_seq_register_write(info,
				sid, addr, buf[0]);
		break;
	case SPMI_CMD_EXT_WRITE:
		ack_nak_bit = spmi_cmd_seq_extended_register_write(info,
				sid, addr, buf, len);
		break;
	case SPMI_CMD_EXT_WRITEL:
		ack_nak_bit = spmi_cmd_seq_extended_register_write_long(info,
				sid, addr, buf, len);
		break;
	case SPMI_CMD_RESET:
	case SPMI_CMD_SLEEP:
	case SPMI_CMD_SHUTDOWN:
	case SPMI_CMD_WAKEUP:
	case SPMI_CMD_AUTHENTICATE:
	case SPMI_CMD_MSTR_READ:
	case SPMI_CMD_MSTR_WRITE:
	case SPMI_CMD_TRANSFER_BUS_OWNERSHIP:
	case SPMI_CMD_DDB_MASTER_READ:
	case SPMI_CMD_DDB_SLAVE_READ:
	case SPMI_CMD_EXT_READ:
	case SPMI_CMD_EXT_READL:
	case SPMI_CMD_ZERO_WRITE:
	default:
		dev_err(&ctrl->dev, "invalid opcode = %#02x\n", opc);
		ret = -EINVAL;
	}

	if (!ack_nak_bit) {
		dev_err(&ctrl->dev, "register write not acked\n");
		ret = -EIO;
	}

	spmi_disable(info);
	return ret;
}

static int spmi_bb_read_cmd(struct spmi_controller *ctrl,
			 u8 opc, u8 sid, u16 addr, u8 *buf, size_t len)
{
	int ret = 0;
	bool ack_nak_bit = true;
	struct spmi_bb_info *info = spmi_controller_get_drvdata(ctrl);

	spmi_enable(info);
	spmi_bus_arbitrate(info);
	switch (opc) {
	case SPMI_CMD_READ:
		ack_nak_bit = spmi_cmd_seq_register_read(info,
				sid, addr, buf);
		break;
	case SPMI_CMD_EXT_READ:
		ack_nak_bit = spmi_cmd_seq_extended_register_read(info,
				sid, addr, buf, len);
		break;
	case SPMI_CMD_EXT_READL:
		ack_nak_bit = spmi_cmd_seq_extended_register_read_long(info,
				sid, addr, buf, len);
		break;
	case SPMI_CMD_WRITE:
	case SPMI_CMD_EXT_WRITE:
	case SPMI_CMD_EXT_WRITEL:
	case SPMI_CMD_RESET:
	case SPMI_CMD_SLEEP:
	case SPMI_CMD_SHUTDOWN:
	case SPMI_CMD_WAKEUP:
	case SPMI_CMD_AUTHENTICATE:
	case SPMI_CMD_MSTR_READ:
	case SPMI_CMD_MSTR_WRITE:
	case SPMI_CMD_TRANSFER_BUS_OWNERSHIP:
	case SPMI_CMD_DDB_MASTER_READ:
	case SPMI_CMD_DDB_SLAVE_READ:
	case SPMI_CMD_ZERO_WRITE:
	default:
		dev_err(&ctrl->dev, "invalid opcode = %#02x\n", opc);
		ret = -EINVAL;
	}

	if (!ack_nak_bit) {
		dev_err(&ctrl->dev, "register read not acked\n");
		ret = -EIO;
	}
	spmi_disable(info);
	return 0;
}

static int spmi_bb_dt_init(struct spmi_bb_info *info)
{
	struct device *dev = &info->ctrl->dev;
	u32 delay_us;
	int rc;

	if (!dev->of_node) {
		dev_err(dev, "No of_node\n");
		return -ENOENT;
	}

	rc = of_property_read_u32(dev->of_node, "delay_us", &delay_us);
	if (rc < 0) {
		dev_err(dev, "Unable to read delay_us\n");
		return -ENODATA;
	}

	info->delay_us = delay_us;

	return 0;
}

static int spmi_bb_probe(struct platform_device *pdev)
{
	struct spmi_bb_info *spmi_bb_info;
	struct spmi_controller *ctrl;
	int ret;

	ctrl = spmi_controller_alloc(&pdev->dev, sizeof(*spmi_bb_info));
	if (!ctrl) {
		dev_err(&pdev->dev, "cannot allocate spmi_bb_info\n");
		return -ENOMEM;
	}
	spmi_bb_info = spmi_controller_get_drvdata(ctrl);
	spmi_bb_info->ctrl = ctrl;

	platform_set_drvdata(pdev, spmi_bb_info);

	/* Get the clock gpio */
	spmi_bb_info->gpio_clk = devm_gpiod_get(&pdev->dev, "clk", GPIOD_IN);
	if (IS_ERR(spmi_bb_info->gpio_clk)) {
		ret = PTR_ERR(spmi_bb_info->gpio_clk);
		dev_err(&pdev->dev, "spmi_clk ERROR %d\n", ret);
		goto err_put_controller;
	}

	/* Get the data gpio */
	spmi_bb_info->gpio_dat = devm_gpiod_get(&pdev->dev, "dat", GPIOD_IN);
	if (IS_ERR(spmi_bb_info->gpio_dat)) {
		ret = PTR_ERR(spmi_bb_info->gpio_dat);
		dev_err(&pdev->dev, "spmi_dat ERROR %d\n", ret);
		goto err_put_controller;
	}

	spmi_bb_info->delay_us = DEFAULT_CLK_DELAY_US; /* dt_init may overwrite */
	spmi_bb_dt_init(spmi_bb_info);

	/* Callbacks */
	ctrl->read_cmd = spmi_bb_read_cmd;
	ctrl->write_cmd = spmi_bb_write_cmd;

	ret = spmi_controller_add(ctrl);
	if (ret) {
		dev_err(&pdev->dev, "Unable to add controller (%d)!\n", ret);
		goto err_put_controller;
	}

	return 0;

err_put_controller:
	spmi_controller_put(ctrl);
	return ret;
}

static int spmi_bb_remove(struct platform_device *pdev)
{
	struct spmi_bb_info *spmi_bb_info = platform_get_drvdata(pdev);
	struct spmi_controller *ctrl = spmi_bb_info->ctrl;

	spmi_controller_remove(ctrl);
	spmi_controller_put(ctrl);
	return 0;
}

static const struct of_device_id spmi_bb_match_table[] = {
	{
		.compatible = "google,bitbang-spmi-controller",
	},
	{}
};
MODULE_DEVICE_TABLE(of, spmi_bb_match_table);

static struct platform_driver spmi_bb_driver = {
	.probe		= spmi_bb_probe,
	.remove		= spmi_bb_remove,
	.driver		= {
		.name	= "spmi_bitbang_controller",
		.of_match_table = spmi_bb_match_table,
	},
};

static int __init spmi_bb_init(void)
{
	int rv;

	rv = platform_driver_register(&spmi_bb_driver);
	return rv;
}
module_init(spmi_bb_init);

static void __exit spmi_bb_exit(void)
{
	platform_driver_unregister(&spmi_bb_driver);
}
module_exit(spmi_bb_exit);

MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Jim Wylder<jwylder@google.com>");
