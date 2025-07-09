/* drivers/input/keydebug-core.c
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/of.h>
#include <linux/time.h>
#include <linux/keycombo.h>
#include <linux/keydebug.h>
#include <linux/kernel-top.h>
#include <linux/sysrq.h>

/*
 * On the kernel command line specify
 * keydebug.kernel_top_enable=1 to enable the kernel_top
 * By default kernel_top is turned on
 */
static int kernel_top_enable = 1;
module_param(kernel_top_enable, int, 0644);

/*
 * On the kernel command line specify
 * keydebug.show_dstate_enable=1 to enable the show_dstate
 * By default show_dstate is turned on
 */
static int show_dstate_enable = 1;
module_param(show_dstate_enable, int, 0644);

/*
 * On the kernel command line specify
 * keydebug.showallcpus_enable=1 to enable the showallcpus
 * By default showallcpus is turned on
 */
static int showallcpus_enable = 1;
module_param(showallcpus_enable, int, 0644);

/*
 * On the kernel command line specify
 * keydebug.bind_s2d=0 to avoid config S2D.
 * By default keydebug disables S2D when receiving down event, and restores when key released.
 */
static int bind_s2d = 1;
module_param(bind_s2d, int, 0644);

#define DEFAULT_DBG_DELAY 3000 /* millisecond */
static int probe_cnt;
static struct workqueue_struct *kdbg_wq;

static int s2d_get_enable_noop(void)
{
	return -EPERM;
}

static int s2d_set_enable_noop(int en)
{
	return -EPERM;
}

static int (*s2d_get_enable)(void) = s2d_get_enable_noop;
static int (*s2d_set_enable)(int en) = s2d_set_enable_noop;

void keydebug_register_s2d_ops(void *get, void *set)
{
	s2d_get_enable = get;
	s2d_set_enable = set;
}
EXPORT_SYMBOL_GPL(keydebug_register_s2d_ops);

void do_keydebug(struct work_struct *this)
{
	struct delayed_work *dwork = container_of(this, struct delayed_work,
									work);
	struct keydebug_platform_data *pdata = container_of(dwork,
				struct keydebug_platform_data, delayed_work);

	if (pdata->ktop) {
		kernel_top_print(pdata->ktop);
		kernel_top_destroy(pdata->ktop);
		pdata->ktop = NULL;
	}

	if (show_dstate_enable) {
		pr_info("=======     Show D state tasks++   =======\n");
		handle_sysrq('w');
		pr_info("=======     Show D state tasks--   =======\n");
	}

	if (showallcpus_enable)
		handle_sysrq('l');

	pdata->keydebug_requested = false;
}

static int s2d_state_xchg(int new, int *pold)
{
	int old;

	if (new < 0)
		return -EINVAL;

	old = s2d_get_enable();
	if (old < 0)
		return old;

	pr_info(KEYDEBUG_NAME ": turning s2d %d->%d\n", old, new);

	if (pold)
		*pold = old;
	return s2d_set_enable(new);
}

static void keydebug_event_down(void *priv)
{
	struct keydebug_platform_data *pdata = priv;
	uint32_t msecs = pdata->dbg_fn_delay ?: DEFAULT_DBG_DELAY;

	if (bind_s2d)
		s2d_state_xchg(false, &pdata->s2d_state_backup);

	if (pdata->keydebug_requested) {
		pr_info("%s: request is running\n", __func__);
		return;
	} else {
		WARN_ON(pdata->ktop);
		pdata->ktop = NULL;
		if (kernel_top_enable)
			WARN_ON(kernel_top_init(pdata->pdev_child->dev.parent,
					&pdata->ktop));

		queue_delayed_work(kdbg_wq,
			&pdata->delayed_work, msecs_to_jiffies(msecs));

		pdata->keydebug_requested = true;
	}
}

static void keydebug_event_up(void *priv)
{
	struct keydebug_platform_data *pdata = priv;

	if (bind_s2d)
		s2d_state_xchg(pdata->s2d_state_backup, NULL);
}

static int keydebug_parse_dt(struct device *dev,
	struct keydebug_platform_data *pdata)
{
	int ret = 0, cnt = 0, num_keys;
	struct device_node *dt = dev_of_node(dev);
	struct property *prop;

	/* Parse key_down_delay */
	if (of_property_read_u32(dt, "key_down_delay", &pdata->key_down_delay))
		pr_info("%s: DT:key_down_delay property not found\n", __func__);

	/* Parse dbg_delay */
	if (of_property_read_u32(dt, "dbg_fn_delay", &pdata->dbg_fn_delay))
		pr_info("%s: DT:dbg_fn_delay property not found\n", __func__);

	/* Must have keys_down property */
	prop = of_find_property(dt, "keys_down", NULL);
	if (!prop) {
		pr_err("%s: DT:keys_down property not found\n", __func__);
		ret = -EINVAL;
		goto err_parse_keys_down_failed;
	} else {
		num_keys = prop->length / sizeof(uint32_t);
	}

	/* Allocate down_size + 1 to assure 0 terminated */
	pdata->keys_down = devm_kzalloc(dev,
		(num_keys + 1) * sizeof(uint32_t), GFP_KERNEL);
	if (!pdata->keys_down) {
		pr_err("%s: DT:keys_down fail to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_parse_keys_down_failed;
	}

	if (of_property_read_u32_array(dt, "keys_down",
			pdata->keys_down, num_keys)) {
		pr_err("%s: DT:keys_down parse err\n", __func__);
		ret = -EINVAL;
		goto err_keys_down_failed;
	}

	pr_info("%s: DT:key_down_delay=%d dbg_fn_delay=%d"
		" keys_down num_keys=%d\n", __func__, pdata->key_down_delay,
		pdata->dbg_fn_delay, num_keys);

	for (cnt = 0; cnt < num_keys; cnt++)
		pr_info("%s: DT:keys_down=%d\n", __func__,
			pdata->keys_down[cnt]);

	return 0;

err_keys_down_failed:
	devm_kfree(dev, pdata->keys_down);
err_parse_keys_down_failed:
	return ret;
}

static int keydebug_probe(struct platform_device *pdev)
{
	int ret = -ENOMEM;
	struct device *dev = &pdev->dev;
	struct keycombo_platform_data *pdata_child;
	struct keydebug_platform_data *pdata = dev_get_platdata(dev);
	int down_size = 0, size;
	uint32_t key, *keyp;
	int *ckeyp;

	/* only allow one instance at a time*/
	if (probe_cnt)
		return 0;
	else
		probe_cnt++;

	if (dev_of_node(dev) && !pdata) {
		pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: fail to allocate "
				"keydebug_platform_data\n", __func__);
			ret = -ENOMEM;
			goto err_get_pdata_fail;
		}

		ret = keydebug_parse_dt(dev, pdata);
		if (ret < 0) {
			pr_err("%s: keydebug_parse_dt fail\n", __func__);
			goto err_parse_fail;
		}
	} else if (!pdata)
		return -EINVAL;

	kdbg_wq = alloc_workqueue("kdbgd", WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);

	INIT_DELAYED_WORK(&pdata->delayed_work, do_keydebug);

	pdata->pdev_child = platform_device_alloc(KEYCOMBO_NAME,
		PLATFORM_DEVID_AUTO);
	if (!pdata->pdev_child)
		return -ENOMEM;

	pdata->pdev_child->dev.parent = dev;

	/* Calculate valid keys down size*/
	keyp = pdata->keys_down;
	while ((key = *keyp++)) {
		if (key >= KEY_MAX)
			continue;
		down_size++;
	}

	/* Allocate down_size + 1 to assure 0 terminated */
	size = sizeof(struct keycombo_platform_data)
			+ sizeof(int) * (down_size + 1);
	pdata_child = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!pdata_child)
		goto error;

	/* Copy valid key code*/
	keyp = pdata->keys_down;
	ckeyp = pdata_child->keys_down;
	while ((*ckeyp = *keyp++)) {
		if (*ckeyp >= KEY_MAX)
			continue;
		ckeyp++;
	}

	pdata_child->priv = pdata;
	pdata_child->key_down_fn = keydebug_event_down;
	pdata_child->key_up_fn = keydebug_event_up;
	pdata_child->key_down_delay = pdata->key_down_delay;
	ret = platform_device_add_data(pdata->pdev_child, pdata_child, size);
	if (ret)
		goto error;

	return platform_device_add(pdata->pdev_child);
error:
	platform_device_put(pdata->pdev_child);
err_parse_fail:
	devm_kfree(dev, pdata);
err_get_pdata_fail:
	if (kdbg_wq)
		destroy_workqueue(kdbg_wq);
	probe_cnt--;
	return ret;
}

static int keydebug_remove(struct platform_device *pdev)
{
	struct keydebug_platform_data *pdata = dev_get_platdata(&pdev->dev);

	flush_delayed_work(&pdata->delayed_work);

	platform_device_put(pdata->pdev_child);
	if (kdbg_wq)
		destroy_workqueue(kdbg_wq);
	probe_cnt = 0;
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id keydebug_match_table[] = {
	{ .compatible = KEYDEBUG_NAME},
	{},
};
MODULE_DEVICE_TABLE(of, keydebug_match_table);
#else
#define keydebug_match_table NULL
#endif

struct platform_driver keydebug_driver = {
	.probe = keydebug_probe,
	.remove = keydebug_remove,
	.driver = {
		.name = KEYDEBUG_NAME,
		.owner = THIS_MODULE,
		.of_match_table = keydebug_match_table,
	},
};

static int __init keydebug_init(void)
{
	return platform_driver_register(&keydebug_driver);
}

static void __exit keydebug_exit(void)
{
	return platform_driver_unregister(&keydebug_driver);
}

module_init(keydebug_init);
module_exit(keydebug_exit);
MODULE_DESCRIPTION("keydebug Driver");
MODULE_LICENSE("GPL v2");
