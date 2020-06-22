// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Authors:
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *	Seung-Woo Kim <sw0312.kim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/pm_runtime.h>
#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_probe_helper.h>

#include <linux/component.h>

#include <drm/exynos_drm.h>

#include <exynos_drm_drv.h>
#include <exynos_drm_fbdev.h>
#include <exynos_drm_fb.h>
#include <exynos_drm_plane.h>
#include <exynos_drm_gem.h>
#include <exynos_drm_writeback.h>

#define DRIVER_NAME	"exynos"
#define DRIVER_DESC	"Samsung SoC DRM"
#define DRIVER_DATE	"20110530"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

int exynos_atomic_check(struct drm_device *dev,
			struct drm_atomic_state *state)
{
	int ret;

	ret = drm_atomic_helper_check_modeset(dev, state);
	if (ret)
		return ret;

	ret = drm_atomic_normalize_zpos(dev, state);
	if (ret)
		return ret;

	ret = drm_atomic_helper_check_planes(dev, state);
	if (ret)
		return ret;

	return ret;
}

static const struct vm_operations_struct exynos_drm_gem_vm_ops = {
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static int exynos_drm_open(struct drm_device *dev, struct drm_file *file)
{
	struct drm_exynos_file_private *file_priv;

	file_priv = kzalloc(sizeof(*file_priv), GFP_KERNEL);
	if (!file_priv)
		return -ENOMEM;

	file->driver_priv = file_priv;

	return 0;
}

static void exynos_drm_postclose(struct drm_device *dev, struct drm_file *file)
{
	kfree(file->driver_priv);
	file->driver_priv = NULL;
}

static void exynos_drm_lastclose(struct drm_device *dev)
{
	exynos_drm_fbdev_restore_mode(dev);
}

static const struct file_operations exynos_drm_driver_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.mmap		= exynos_drm_gem_mmap,
	.poll		= drm_poll,
	.read		= drm_read,
	.unlocked_ioctl	= drm_ioctl,
	.compat_ioctl	= drm_compat_ioctl,
	.release	= drm_release,
};

static struct drm_driver exynos_drm_driver = {
	.driver_features	   = DRIVER_MODESET | DRIVER_ATOMIC |
				     DRIVER_RENDER | DRIVER_GEM,
	.open			   = exynos_drm_open,
	.lastclose		   = exynos_drm_lastclose,
	.postclose		   = exynos_drm_postclose,
	.gem_free_object_unlocked  = exynos_drm_gem_free_object,
	.gem_vm_ops		   = &exynos_drm_gem_vm_ops,
	.dumb_create		   = exynos_drm_gem_dumb_create,
	.dumb_map_offset	   = exynos_drm_gem_dumb_map_offset,
	.prime_handle_to_fd	   = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	   = drm_gem_prime_fd_to_handle,
	.gem_prime_import	   = exynos_drm_gem_prime_import,
	.gem_prime_import_sg_table = exynos_drm_gem_prime_import_sg_table,
	.gem_print_info		   = exynos_drm_gem_print_info,
	.fops			   = &exynos_drm_driver_fops,
	.name			   = DRIVER_NAME,
	.desc			   = DRIVER_DESC,
	.date			   = DRIVER_DATE,
	.major			   = DRIVER_MAJOR,
	.minor			   = DRIVER_MINOR,
};

/* forward declaration */
static struct platform_driver exynos_drm_platform_driver;

struct exynos_drm_driver_info {
	struct platform_driver *driver;
	unsigned int flags;
};

#define DRM_COMPONENT_DRIVER	BIT(0)	/* supports component framework */
#define DRM_VIRTUAL_DEVICE	BIT(1)	/* create virtual platform device */
#define DRM_DMA_DEVICE		BIT(2)	/* can be used for dma allocations */

#define DRV_PTR(drv, cond) (IS_ENABLED(cond) ? &drv : NULL)

/*
 * Connector drivers should not be placed before associated crtc drivers,
 * because connector requires pipe number of its crtc during initialization.
 */
static struct exynos_drm_driver_info exynos_drm_drivers[] = {
	{
		DRV_PTR(dpp_driver, CONFIG_DRM_SAMSUNG_DPP),
		DRM_COMPONENT_DRIVER
	}, {
		DRV_PTR(decon_driver, CONFIG_DRM_SAMSUNG_DECON),
		DRM_COMPONENT_DRIVER | DRM_DMA_DEVICE
	}, {
		DRV_PTR(dsim_driver, CONFIG_DRM_SAMSUNG_DSI),
		DRM_COMPONENT_DRIVER
	}, {
		DRV_PTR(writeback_driver, CONFIG_DRM_SAMSUNG_WB),
		DRM_COMPONENT_DRIVER
	}, {
		&exynos_drm_platform_driver,
		DRM_VIRTUAL_DEVICE
	}
};

static int compare_dev(struct device *dev, void *data)
{
	return dev == (struct device *)data;
}

static struct component_match *exynos_drm_match_add(struct device *dev)
{
	struct component_match *match = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(exynos_drm_drivers); ++i) {
		struct exynos_drm_driver_info *info = &exynos_drm_drivers[i];
		struct device *p = NULL, *d;

		if (!info->driver || !(info->flags & DRM_COMPONENT_DRIVER))
			continue;

		while ((d = bus_find_device(&platform_bus_type, p,
					    &info->driver->driver,
					    (void *)platform_bus_type.match))) {
			put_device(p);
			component_match_add(dev, &match, compare_dev, d);
			p = d;
		}
		put_device(p);
	}

	return match ?: ERR_PTR(-ENODEV);
}

static int exynos_drm_bind(struct device *dev)
{
	struct exynos_drm_private *private;
	struct drm_encoder *encoder;
	struct drm_device *drm;
	u32 wb_mask = 0;
	u32 encoder_mask = 0;
	int ret;

	drm = drm_dev_alloc(&exynos_drm_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	private = kzalloc(sizeof(struct exynos_drm_private), GFP_KERNEL);
	if (!private) {
		ret = -ENOMEM;
		goto err_free_drm;
	}

	init_waitqueue_head(&private->wait);
	spin_lock_init(&private->lock);

	dev_set_drvdata(dev, drm);
	drm->dev_private = (void *)private;

	drm_mode_config_init(drm);

	exynos_drm_mode_config_init(drm);

	/* Try to bind all sub drivers. */
	ret = component_bind_all(drm->dev, drm);
	if (ret)
		goto err_mode_config_cleanup;

	drm_for_each_encoder(encoder, drm) {
		if (encoder->encoder_type == DRM_MODE_ENCODER_VIRTUAL)
			wb_mask |= drm_encoder_mask(encoder);

		encoder_mask |= drm_encoder_mask(encoder);
	}

	drm_for_each_encoder(encoder, drm) {
		if (encoder->encoder_type == DRM_MODE_ENCODER_VIRTUAL)
			encoder->possible_clones = encoder_mask;
		else
			encoder->possible_clones = drm_encoder_mask(encoder) |
				wb_mask;

		pr_info("encoder[%d] type(0x%x) possible_clones(0x%x)\n",
				encoder->index, encoder->encoder_type,
				encoder->possible_clones);
	}

	ret = drm_vblank_init(drm, drm->mode_config.num_crtc);
	if (ret)
		goto err_unbind_all;

	drm_mode_config_reset(drm);

	/*
	 * enable drm irq mode.
	 * - with irq_enabled = true, we can use the vblank feature.
	 *
	 * P.S. note that we wouldn't use drm irq handler but
	 *	just specific driver own one instead because
	 *	drm framework supports only one irq handler.
	 */
	drm->irq_enabled = true;

	/* init kms poll for handling hpd */
	drm_kms_helper_poll_init(drm);

	ret = exynos_drm_fbdev_init(drm);
	if (ret)
		goto err_cleanup_poll;

	/* register the DRM device */
	ret = drm_dev_register(drm, 0);
	if (ret < 0)
		goto err_cleanup_fbdev;

	return 0;

err_cleanup_fbdev:
	exynos_drm_fbdev_fini(drm);
err_cleanup_poll:
	drm_kms_helper_poll_fini(drm);
err_unbind_all:
	component_unbind_all(drm->dev, drm);
err_mode_config_cleanup:
	drm_mode_config_cleanup(drm);
	kfree(private);
err_free_drm:
	drm_dev_put(drm);
	drm_mode_config_cleanup(drm);
	kfree(private);

	return ret;
}

static void exynos_drm_unbind(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	drm_dev_unregister(drm);

	exynos_drm_fbdev_fini(drm);
	drm_kms_helper_poll_fini(drm);

	component_unbind_all(drm->dev, drm);
	drm_mode_config_cleanup(drm);

	kfree(drm->dev_private);
	drm->dev_private = NULL;
	dev_set_drvdata(dev, NULL);

	drm_dev_put(drm);
}

static const struct component_master_ops exynos_drm_ops = {
	.bind		= exynos_drm_bind,
	.unbind		= exynos_drm_unbind,
};

static int exynos_drm_platform_probe(struct platform_device *pdev)
{
	struct component_match *match;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	match = exynos_drm_match_add(&pdev->dev);
	if (IS_ERR(match))
		return PTR_ERR(match);

	return component_master_add_with_match(&pdev->dev, &exynos_drm_ops,
					       match);
}

static int exynos_drm_platform_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &exynos_drm_ops);
	return 0;
}

static struct platform_driver exynos_drm_platform_driver = {
	.probe	= exynos_drm_platform_probe,
	.remove	= exynos_drm_platform_remove,
	.driver	= {
		.name	= "exynos-drm",
	},
};

static void exynos_drm_unregister_devices(void)
{
	int i;

	for (i = ARRAY_SIZE(exynos_drm_drivers) - 1; i >= 0; --i) {
		struct exynos_drm_driver_info *info = &exynos_drm_drivers[i];
		struct device *dev;

		if (!info->driver || !(info->flags & DRM_VIRTUAL_DEVICE))
			continue;

		while ((dev = bus_find_device(&platform_bus_type, NULL,
					    &info->driver->driver,
					    (void *)platform_bus_type.match))) {
			put_device(dev);
			platform_device_unregister(to_platform_device(dev));
		}
	}
}

static int exynos_drm_register_devices(void)
{
	struct platform_device *pdev;
	int i;

	for (i = 0; i < ARRAY_SIZE(exynos_drm_drivers); ++i) {
		struct exynos_drm_driver_info *info = &exynos_drm_drivers[i];

		if (!info->driver || !(info->flags & DRM_VIRTUAL_DEVICE))
			continue;

		pdev = platform_device_register_simple(
					info->driver->driver.name, -1, NULL, 0);
		if (IS_ERR(pdev))
			goto fail;
	}

	return 0;
fail:
	exynos_drm_unregister_devices();
	return PTR_ERR(pdev);
}

static void exynos_drm_unregister_drivers(void)
{
	int i;

	for (i = ARRAY_SIZE(exynos_drm_drivers) - 1; i >= 0; --i) {
		struct exynos_drm_driver_info *info = &exynos_drm_drivers[i];

		if (!info->driver)
			continue;

		platform_driver_unregister(info->driver);
	}
}

static int exynos_drm_register_drivers(void)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(exynos_drm_drivers); ++i) {
		struct exynos_drm_driver_info *info = &exynos_drm_drivers[i];

		if (!info->driver)
			continue;

		ret = platform_driver_register(info->driver);
		if (ret)
			goto fail;
	}
	return 0;
fail:
	exynos_drm_unregister_drivers();
	return ret;
}

static int exynos_drm_init(void)
{
	int ret;

	if (enable_fbdev_display) {
		pr_info("DRM/KMS not selected\n");
		return -EINVAL;
	}

	ret = exynos_drm_register_devices();
	if (ret)
		return ret;

	ret = exynos_drm_register_drivers();
	if (ret)
		goto err_unregister_pdevs;

	return 0;

err_unregister_pdevs:
	exynos_drm_unregister_devices();

	return ret;
}

static void exynos_drm_exit(void)
{
	exynos_drm_unregister_drivers();
	exynos_drm_unregister_devices();
}

module_init(exynos_drm_init);
module_exit(exynos_drm_exit);

MODULE_AUTHOR("Inki Dae <inki.dae@samsung.com>");
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_AUTHOR("Seung-Woo Kim <sw0312.kim@samsung.com>");
MODULE_DESCRIPTION("Samsung SoC DRM Driver");
MODULE_LICENSE("GPL");
