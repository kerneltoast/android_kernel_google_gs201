/* SPDX-License-Identifier: MIT */

#include "gs_panel/gs_panel_test.h"

static const struct gs_panel_test_desc google_common_test = {
	.test_funcs = NULL,
	.regs_desc = NULL,
	.query_desc = NULL,
};

static int common_panel_test_probe(struct platform_device *pdev)
{
	struct gs_panel_test *test;

	test = devm_kzalloc(&pdev->dev, sizeof(*test), GFP_KERNEL);
	if (!test)
		return -ENOMEM;

	return gs_panel_test_common_init(pdev, test);
}

static const struct of_device_id gs_panel_test_of_match[] = {
	{ .compatible = "google,gs-panel-common-test", .data = &google_common_test },
	{}
};
MODULE_DEVICE_TABLE(of, gs_panel_test_of_match);

static struct platform_driver gs_panel_test_driver = {
	.probe = common_panel_test_probe,
	.remove = gs_panel_test_common_remove,
	.driver = {
		.name = "gs-panel-common-test",
		.of_match_table = gs_panel_test_of_match,
	},
};
module_platform_driver(gs_panel_test_driver);

MODULE_AUTHOR("Safayat Ullah <safayat@google.com>");
MODULE_DESCRIPTION("MIPI-DSI based Google panel test common driver");
MODULE_LICENSE("Dual MIT/GPL");
