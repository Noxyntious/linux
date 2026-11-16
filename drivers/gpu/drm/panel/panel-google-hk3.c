// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) Google LLC
 *
 * Boot-handoff panel driver for the Pixel 8 Pro (husky) display, "hk3"
 * (Samsung DDIC, 1344x2992 LTPO OLED, MIPI DSI command mode, 4 lanes,
 * dual-slice DSC 1.1 at 8 bpp).
 *
 * The bootloader powers the panel, trains the DSI link and runs the panel's
 * power-on sequence, then leaves it lit. This driver does not power or reset
 * it: prepare() is a no-op, and enable() only (re)programs what the DECON
 * needs the panel to agree on - the DSC decoder (0x9D + PPS), the image-size
 * and bit-depth selectors (0xC3 / 0xF2), TE, and brightness control - before
 * display-on. If the panel was put to sleep by disable(), enable() wakes it.
 *
 * Panel data (DSC PPS, timings, register values) is from Google's
 * panel-google-hk3.c (kernel/devices/google/shusky, android-gs-shusky-6.1-
 * android16): wqhd_pps_config, hk3_modes, and the DSC/C3/F2 block of
 * hk3_enable(). Structure follows panel-google-komodo.c in this tree, whose
 * DDIC uses the same selector registers.
 *
 * Only native WQHD at the bootloader's refresh rate (60 Hz) is offered for
 * now; 120 Hz and the 1008x2244 mode need hk3's frequency-change commands
 * and a per-mode DSC config on the host side.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define HK3_BRIGHTNESS_MAX	4095
#define HK3_BRIGHTNESS_DEFAULT	1353	/* vendor dft_brightness, ~140 nits */

struct hk3_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset_gpio;
	bool asleep;	/* set by disable(): the next enable() must sleep-out */
};

static inline struct hk3_panel *to_hk3_panel(struct drm_panel *panel)
{
	return container_of(panel, struct hk3_panel, panel);
}

/* vendor wqhd_pps_config (1344x2992); slice_count from HK3_WQHD_DSC */
static const struct drm_dsc_config hk3_wqhd_dsc = {
	.line_buf_depth = 9,
	.bits_per_component = 8,
	.convert_rgb = true,
	.slice_width = 672,
	.slice_height = 187,
	.slice_count = 2,
	.simple_422 = false,
	.pic_width = 1344,
	.pic_height = 2992,
	.rc_tgt_offset_high = 3,
	.rc_tgt_offset_low = 3,
	.bits_per_pixel = 128,
	.rc_edge_factor = 6,
	.rc_quant_incr_limit1 = 11,
	.rc_quant_incr_limit0 = 11,
	.initial_xmit_delay = 512,
	.initial_dec_delay = 592,
	.block_pred_enable = true,
	.first_line_bpg_offset = 12,
	.initial_offset = 6144,
	.rc_buf_thresh = {
		14, 28, 42, 56, 70, 84, 98, 105,
		112, 119, 121, 123, 125, 126
	},
	.rc_range_params = {
		{ 0, 4, 2 }, { 0, 4, 0 }, { 1, 5, 0 }, { 1, 6, 62 },
		{ 3, 7, 60 }, { 3, 7, 58 }, { 3, 7, 56 }, { 3, 8, 56 },
		{ 3, 9, 56 }, { 3, 10, 54 }, { 5, 11, 54 }, { 5, 12, 52 },
		{ 5, 13, 52 }, { 7, 13, 52 }, { 13, 15, 52 }
	},
	.rc_model_size = 8192,
	.flatness_min_qp = 3,
	.flatness_max_qp = 12,
	.initial_scale_value = 32,
	.scale_decrement_interval = 9,
	.scale_increment_interval = 5177,
	.nfl_bpg_offset = 133,
	.slice_bpg_offset = 112,
	.final_offset = 4336,
	.vbr_enable = false,
	.slice_chunk_size = 672,
	.dsc_version_minor = 1,
	.dsc_version_major = 1,
};

/* vendor "1344x2992x60": hfp 80, hsa 24, hbp 42; vfp 12, vsa 4, vbp 22 */
static const struct drm_display_mode hk3_wqhd_60 = {
	.clock = (1344 + 80 + 24 + 42) * (2992 + 12 + 4 + 22) * 60 / 1000,
	.hdisplay = 1344,
	.hsync_start = 1344 + 80,
	.hsync_end = 1344 + 80 + 24,
	.htotal = 1344 + 80 + 24 + 42,
	.vdisplay = 2992,
	.vsync_start = 2992 + 12,
	.vsync_end = 2992 + 12 + 4,
	.vtotal = 2992 + 12 + 4 + 22,
	.width_mm = 70,
	.height_mm = 155,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

#define hk3_dcs(dsi, seq...)						\
	mipi_dsi_dcs_write_buffer((dsi), (const u8[]){ seq },		\
				  sizeof((const u8[]){ seq }))

static int hk3_panel_noop(struct drm_panel *panel)
{
	return 0;
}

static int hk3_panel_enable(struct drm_panel *panel)
{
	struct hk3_panel *ctx = to_hk3_panel(panel);
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	dev_info(&dsi->dev, "enable (asleep=%d)\n", ctx->asleep);

	if (ctx->asleep) {
		mipi_dsi_dcs_exit_sleep_mode(dsi);
		msleep(120);
		ctx->asleep = false;
	}

	/* DSC: enable + PPS (vendor hk3_enable "pps" block) */
	hk3_dcs(dsi, 0x9D, 0x01);
	drm_dsc_pps_payload_pack(&pps, &hk3_wqhd_dsc);
	ret = mipi_dsi_picture_parameter_set(dsi, &pps);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to send DSC PPS: %d\n", ret);

	/* image size (0x0C = WQHD) and 8-bit depth (0x01) selectors */
	hk3_dcs(dsi, 0xF0, 0x5A, 0x5A);		/* unlock */
	hk3_dcs(dsi, 0xC3, 0x0C);
	hk3_dcs(dsi, 0xB0, 0x00, 0x01, 0xF2);
	hk3_dcs(dsi, 0xF2, 0x01);
	hk3_dcs(dsi, 0xF0, 0xA5, 0xA5);		/* lock */

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0)
		dev_err(&dsi->dev, "tear on failed: %d\n", ret);
	/* WRCTRLD: brightness control on (vendor HK3_WRCTRLD_BCTRL_BIT) */
	hk3_dcs(dsi, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x20);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	dev_info(&dsi->dev, "enable done, display on: %d\n", ret);
	return 0;
}

static int hk3_panel_disable(struct drm_panel *panel)
{
	struct hk3_panel *ctx = to_hk3_panel(panel);

	dev_info(&ctx->dsi->dev, "disable\n");
	mipi_dsi_dcs_set_display_off(ctx->dsi);
	mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	ctx->asleep = true;
	return 0;
}

static int hk3_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &hk3_wqhd_60);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = hk3_wqhd_60.width_mm;
	connector->display_info.height_mm = hk3_wqhd_60.height_mm;
	connector->display_info.bpc = 8;

	return 1;
}

static const struct drm_panel_funcs hk3_panel_funcs = {
	.prepare = hk3_panel_noop,
	.unprepare = hk3_panel_noop,
	.enable = hk3_panel_enable,
	.disable = hk3_panel_disable,
	.get_modes = hk3_panel_get_modes,
};

/* 12-bit DBV over DCS 0x51, MSB first; above ~3271 is the HBM band */
static int hk3_bl_update_status(struct backlight_device *bl)
{
	struct hk3_panel *ctx = bl_get_data(bl);

	return mipi_dsi_dcs_set_display_brightness_large(ctx->dsi,
					backlight_get_brightness(bl));
}

static const struct backlight_ops hk3_bl_ops = {
	.update_status = hk3_bl_update_status,
};

static int hk3_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = HK3_BRIGHTNESS_DEFAULT,
		.max_brightness = HK3_BRIGHTNESS_MAX,
	};
	struct backlight_device *bl;
	struct hk3_panel *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct hk3_panel, panel,
				   &hk3_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	/*
	 * Claimed high (the run state) so the lit panel is not disturbed; the
	 * driver never pulses it, as nothing here can re-power the panel yet.
	 */
	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset gpio\n");

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->hs_rate = 1368000000;	/* vendor MIPI_DSI_FREQ_MBPS_DEFAULT */
	dsi->dsc = (struct drm_dsc_config *)&hk3_wqhd_dsc;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;

	bl = devm_backlight_device_register(dev, dev_name(dev), dev, ctx,
					    &hk3_bl_ops, &props);
	if (IS_ERR(bl))
		return dev_err_probe(dev, PTR_ERR(bl),
				     "failed to register backlight\n");
	ctx->panel.backlight = bl;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	return 0;
}

static void hk3_panel_remove(struct mipi_dsi_device *dsi)
{
	struct hk3_panel *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id hk3_panel_of_match[] = {
	{ .compatible = "google,hk3-panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, hk3_panel_of_match);

static struct mipi_dsi_driver hk3_panel_driver = {
	.probe = hk3_panel_probe,
	.remove = hk3_panel_remove,
	.driver = {
		.name = "panel-google-hk3",
		.of_match_table = hk3_panel_of_match,
	},
};
module_mipi_dsi_driver(hk3_panel_driver);

MODULE_DESCRIPTION("Google hk3 (Pixel 8 Pro) boot-handoff panel driver");
MODULE_LICENSE("GPL");
