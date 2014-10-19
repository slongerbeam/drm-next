/*
 * i.MX IPUv3 Graphics driver
 *
 * Copyright (C) 2011 Sascha Hauer, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */
#include <linux/component.h>
#include <linux/module.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/imx_drm.h>

#include "imx-drm.h"
#include "ipuv3-plane.h"

#define DRIVER_DESC		"i.MX IPUv3 Graphics"

struct ipu_channels {
	int dma[2]; /* BG, FG */
	int dp[2];  /* BG, FG */
	int dc;
};

#define NO_DP -1

static const struct ipu_channels sync_dual_plane = {
	.dma = { IPUV3_CHANNEL_MEM_BG_SYNC, IPUV3_CHANNEL_MEM_FG_SYNC },
	.dp  = { IPU_DP_FLOW_SYNC_BG, IPU_DP_FLOW_SYNC_FG },
	.dc  = IPU_DC_CHANNEL_DP_SYNC,
};
static const struct ipu_channels sync_single_plane = {
	.dma = { IPUV3_CHANNEL_MEM_DC_SYNC, },
	.dp  = { NO_DP, NO_DP },
	.dc  = IPU_DC_CHANNEL_SYNC,
};

/*
 * This driver does not yet support async flows for "smart" displays,
 * but keep this around for future reference. The crtc nodes could in
 * future add an "async" property.
 */
#if 0
static const struct ipu_channels async_dual_plane = {
	.dma = { IPUV3_CHANNEL_MEM_BG_ASYNC, IPUV3_CHANNEL_MEM_FG_ASYNC },
	.dp  = { IPU_DP_FLOW_ASYNC0_BG, IPU_DP_FLOW_ASYNC0_FG },
	.dc  = IPU_DC_CHANNEL_DP_ASYNC,
};
static const struct ipu_channels async_single_plane = {
	.dma = { IPUV3_CHANNEL_MEM_DC_ASYNC, },
	.dp  = { NO_DP, NO_DP },
	.dc    = IPU_DC_CHANNEL_ASYNC,
};
#endif

struct ipu_crtc {
	struct device		*dev;
	struct drm_crtc		base;
	struct imx_drm_crtc	*imx_crtc;
	struct device           *ipu_dev; /* our ipu */
	struct ipu_soc          *ipu;
	struct device_node      *port;    /* our port */
	int			id;       /* this crtc's id */

	const struct ipu_channels *ch;

	/* plane[0] is the full plane, plane[1] is the partial plane */
	struct ipu_plane	plane[2];
	bool			have_overlay; /* we have a partial plane */

	struct ipu_dc		*dc;
	struct ipu_di		*di;
	int			enabled;

	u32			interface_pix_fmt;
	struct ipu_dc_if_map    *interface_pix_map;

	unsigned long		di_clkflags;
	int			di_hsync_pin;
	int			di_vsync_pin;
};

#define to_ipu_crtc(x) container_of(x, struct ipu_crtc, base)

static struct ipu_plane *pipe_to_plane(struct ipu_crtc *ipu_crtc,
				       int pipe)
{
	struct ipu_plane *plane;

	plane = &ipu_crtc->plane[0];
	if (pipe == plane->pipe)
		return plane;

	plane = &ipu_crtc->plane[1];
	if (ipu_crtc->have_overlay && pipe == plane->pipe)
		return plane;

	return NULL;
}

static void ipu_fb_enable(struct ipu_crtc *ipu_crtc)
{
	if (ipu_crtc->enabled)
		return;

	ipu_dc_enable(ipu_crtc->dc);
	ipu_plane_enable(&ipu_crtc->plane[0]);
	/* Start DC channel and DI after IDMAC */
	ipu_dc_enable_channel(ipu_crtc->dc);
	ipu_di_enable(ipu_crtc->di);
	ipu_di_enable_clock(ipu_crtc->di);

	ipu_crtc->enabled = 1;
}

static void ipu_fb_disable(struct ipu_crtc *ipu_crtc)
{
	if (!ipu_crtc->enabled)
		return;

	/* Stop DC channel and DI before IDMAC */
	ipu_dc_disable_channel(ipu_crtc->dc);
	ipu_di_disable(ipu_crtc->di);
	ipu_plane_disable(&ipu_crtc->plane[0]);
	ipu_dc_disable(ipu_crtc->dc);
	ipu_di_disable_clock(ipu_crtc->di);

	ipu_crtc->enabled = 0;
}

static void ipu_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);

	dev_dbg(ipu_crtc->dev, "%s mode: %d\n", __func__, mode);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		ipu_fb_enable(ipu_crtc);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		ipu_fb_disable(ipu_crtc);
		break;
	}
}

static int ipu_crtc_page_flip(struct drm_crtc *crtc,
		struct drm_framebuffer *fb,
		struct drm_pending_vblank_event *event,
		uint32_t page_flip_flags)
{
	return ipu_plane_page_flip(crtc->primary, fb, event, page_flip_flags);
}

static int ipu_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
				  struct drm_framebuffer *old_fb)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);

	ipu_crtc->plane[0].x = x;
	ipu_crtc->plane[0].y = y;

	return ipu_plane_page_flip(crtc->primary, crtc->primary->fb, NULL, 0);
}

/*
 * Normally the DRM Gamma API is used to program a color LUT that contains
 * gamma-corrected pixel values for red, green, and blue input pixel values
 * (normally in the range 0 to 255).
 *
 * However the i.MX6 LUT is only 256 entries, so it only supports 8 bpp
 * indexed pixel format. Therefore if the i.MX6 LUT were used to implement
 * gamma correction, th DRM framebuffer would have to use 8 bpp indexed pixel
 * format which is insufficient for most use cases. To support a gamma
 * correcting LUT with full RGB24 or YUV444 pixel formats, there would have
 * to be 3 separate 256-entry LUTs for each color component.
 *
 * But the i.MX6 does support gamma correction via a set of registers that
 * define a piecewise linear approximation to a luminance gamma correction
 * curve. This function uses this approach.
 *
 * The input pixel values to this function must be in a specific format
 * according to the i.MX6 reference manual (see Table 37-28 in the
 * Rev. 1 TRM, dated 04/2013). This info is reprinted here:
 *
 * "The required Gamma correction slope for a specific display should be
 * provided by the display manufacture. This information can be provided
 * in various forms, as graph or formula. The gamma correction input pixel
 * level (Gin) should be normalized to a maximum of 383. The gamma correction
 * output pixel level (Gout) should be normalized to a maximum of 255. Then
 * the following data should be collected:
 *
 * Table 37-28. Gamma correction values
 *   Gin   Gout
 *   ---   -----
 *     0   Gout0
 *     2   Gout1
 *     4   Gout2
 *     8   Gout3
 *    16   Gout4
 *    32   Gout5
 *    64   Gout6
 *    96   Gout7
 *   128   Gout8
 *   160   Gout9
 *   192   Gout10
 *   224   Gout11
 *   256   Gout12
 *   288   Gout13
 *   320   Gout14
 *   352   Gout15"
 *
 *
 * The 16 Gout values must be placed in the input lum[] array. The green
 * and blue input arrays are ignored.
 *
 * The gamma register values are then computed according to Table 37-29
 * in the Rev. 1 TRM.
 */
static void ipu_crtc_gamma_set(struct drm_crtc *crtc,
			       u16 *lum, u16 *g_unused, u16 *b_unused,
			       uint32_t start, uint32_t size)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);
	u32 m[DRM_IMX_GAMMA_SIZE], b[DRM_IMX_GAMMA_SIZE];
	int i;

	if (size != DRM_IMX_GAMMA_SIZE)
		return;

	for (i = 0; i < size; i++) {
		if (i == 0) {
			b[0] = lum[0];
			m[0] = 16 * (lum[1] - lum[0]);
		} else if (i == 15) {
			b[15] = lum[15];
			m[15] = 255 - lum[15];
		} else if (i < 5) {
			b[i] = 2 * lum[i] - lum[i+1];
			m[i] = (lum[i+1] - lum[i]) << (5 - i);
		} else {
			b[i] = lum[i];
			m[i] = lum[i+1] - lum[i];
		}
	}

	ipu_plane_gamma_set(&ipu_crtc->plane[0], true, m, b);
}

static const struct drm_crtc_funcs ipu_crtc_funcs = {
	.set_config = drm_crtc_helper_set_config,
	.destroy = drm_crtc_cleanup,
	.page_flip = ipu_crtc_page_flip,
	.gamma_set = ipu_crtc_gamma_set,
};

static int ipu_crtc_mode_set(struct drm_crtc *crtc,
			       struct drm_display_mode *orig_mode,
			       struct drm_display_mode *mode,
			       int x, int y,
			       struct drm_framebuffer *old_fb)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);
	int ret;
	struct ipu_di_signal_cfg sig_cfg = {};
	struct ipu_dc_if_map *out_pixel_map;
	u32 out_pixel_fmt;

	dev_dbg(ipu_crtc->dev, "%s: mode->hdisplay: %d\n", __func__,
			mode->hdisplay);
	dev_dbg(ipu_crtc->dev, "%s: mode->vdisplay: %d\n", __func__,
			mode->vdisplay);

	out_pixel_fmt = ipu_crtc->interface_pix_fmt;
	out_pixel_map = ipu_crtc->interface_pix_map;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		sig_cfg.interlaced = 1;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		sig_cfg.hsync_pol = 1;
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		sig_cfg.vsync_pol = 1;
	if (mode->flags & DRM_MODE_FLAG_PCLK)
		sig_cfg.clk_pol = 1;

	sig_cfg.enable_pol = 1;
	sig_cfg.width = mode->hdisplay;
	sig_cfg.height = mode->vdisplay;
	sig_cfg.pixel_fmt = out_pixel_fmt;
	sig_cfg.h_back_porch = mode->htotal - mode->hsync_end;
	sig_cfg.h_sync_len = mode->hsync_end - mode->hsync_start;
	sig_cfg.h_front_porch = mode->hsync_start - mode->hdisplay;

	sig_cfg.v_back_porch = mode->vtotal - mode->vsync_end;
	sig_cfg.v_sync_len = mode->vsync_end - mode->vsync_start;
	sig_cfg.v_front_porch = mode->vsync_start - mode->vdisplay;
	sig_cfg.pixelclock = mode->clock * 1000;
	sig_cfg.clkflags = ipu_crtc->di_clkflags;

	sig_cfg.v_to_h_sync = 0;

	sig_cfg.hsync_pin = ipu_crtc->di_hsync_pin;
	sig_cfg.vsync_pin = ipu_crtc->di_vsync_pin;

	ret = ipu_dc_init_sync(ipu_crtc->dc, ipu_crtc->di, sig_cfg.interlaced,
			out_pixel_fmt, out_pixel_map, mode->hdisplay);
	if (ret) {
		dev_err(ipu_crtc->dev,
				"initializing display controller failed with %d\n",
				ret);
		return ret;
	}

	ret = ipu_di_init_sync_panel(ipu_crtc->di, &sig_cfg);
	if (ret) {
		dev_err(ipu_crtc->dev,
				"initializing panel failed with %d\n", ret);
		return ret;
	}

	return ipu_plane_mode_set(&ipu_crtc->plane[0], crtc, mode,
				  crtc->primary->fb,
				  0, 0, mode->hdisplay, mode->vdisplay,
				  x, y, mode->hdisplay, mode->vdisplay);
}

static bool ipu_crtc_mode_fixup(struct drm_crtc *crtc,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void ipu_crtc_prepare(struct drm_crtc *crtc)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);

	ipu_fb_disable(ipu_crtc);
}

static void ipu_crtc_commit(struct drm_crtc *crtc)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);

	ipu_fb_enable(ipu_crtc);
}

static struct drm_crtc_helper_funcs ipu_helper_funcs = {
	.dpms = ipu_crtc_dpms,
	.mode_set_base = ipu_crtc_mode_set_base,
	.mode_fixup = ipu_crtc_mode_fixup,
	.mode_set = ipu_crtc_mode_set,
	.prepare = ipu_crtc_prepare,
	.commit = ipu_crtc_commit,
};

static int ipu_enable_vblank(struct drm_crtc *crtc, int pipe)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);
	struct ipu_plane *ipu_plane = pipe_to_plane(ipu_crtc, pipe);

	if (!ipu_plane)
		return -EINVAL;

	return ipu_plane_enable_vblank(ipu_plane);
}

static void ipu_disable_vblank(struct drm_crtc *crtc, int pipe)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);
	struct ipu_plane *ipu_plane = pipe_to_plane(ipu_crtc, pipe);

	if (!ipu_plane)
		return;

	ipu_plane_disable_vblank(ipu_plane);
}

static int ipu_set_interface_pix_fmt(struct drm_crtc *crtc, u32 encoder_type,
				     u32 pixfmt, struct ipu_dc_if_map *pixmap,
				     int hsync_pin, int vsync_pin)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);

	ipu_crtc->interface_pix_fmt = pixfmt;
	ipu_crtc->interface_pix_map = pixmap;
	ipu_crtc->di_hsync_pin = hsync_pin;
	ipu_crtc->di_vsync_pin = vsync_pin;

	switch (encoder_type) {
	case DRM_MODE_ENCODER_DAC:
	case DRM_MODE_ENCODER_TVDAC:
	case DRM_MODE_ENCODER_LVDS:
		ipu_crtc->di_clkflags = IPU_DI_CLKMODE_SYNC |
			IPU_DI_CLKMODE_EXT;
		break;
	case DRM_MODE_ENCODER_TMDS:
	case DRM_MODE_ENCODER_NONE:
		ipu_crtc->di_clkflags = 0;
		break;
	}

	return 0;
}

static int ipu_gamma_set(struct drm_crtc *crtc, bool enable, u32 *m, u32 *b)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);

	return ipu_plane_gamma_set(&ipu_crtc->plane[0], true, m, b);
}

static const struct imx_drm_crtc_helper_funcs ipu_crtc_helper_funcs = {
	.enable_vblank = ipu_enable_vblank,
	.disable_vblank = ipu_disable_vblank,
	.set_interface_pix_fmt = ipu_set_interface_pix_fmt,
	.gamma_set = ipu_gamma_set,
	.crtc_funcs = &ipu_crtc_funcs,
	.crtc_helper_funcs = &ipu_helper_funcs,
};

static int of_dev_node_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static int get_ipu(struct ipu_crtc *ipu_crtc, struct device_node *node)
{
	struct device_node *ipu_node;
	struct device *ipu_dev;
	int ret;

	ipu_node = of_parse_phandle(node, "ipu", 0);
	if (!ipu_node) {
		dev_err(ipu_crtc->dev, "missing ipu phandle!\n");
		return -ENODEV;
	}

	ipu_dev = bus_find_device(&platform_bus_type, NULL,
				  ipu_node, of_dev_node_match);
	of_node_put(ipu_node);

	if (!ipu_dev) {
		dev_err(ipu_crtc->dev, "failed to find ipu device!\n");
		return -ENODEV;
	}

	device_lock(ipu_dev);

	if (!ipu_dev->driver || !try_module_get(ipu_dev->driver->owner)) {
		ret = -EPROBE_DEFER;
		dev_warn(ipu_crtc->dev, "IPU driver not loaded\n");
		device_unlock(ipu_dev);
		goto dev_put;
	}

	ipu_crtc->ipu_dev = ipu_dev;
	ipu_crtc->ipu = dev_get_drvdata(ipu_dev);

	device_unlock(ipu_dev);
	return 0;
dev_put:
	put_device(ipu_dev);
	return ret;
}

static void ipu_put_resources(struct ipu_crtc *ipu_crtc)
{
	if (!IS_ERR_OR_NULL(ipu_crtc->dc)) {
		ipu_dc_put(ipu_crtc->dc);
		ipu_crtc->dc = NULL;
	}
	if (!IS_ERR_OR_NULL(ipu_crtc->di)) {
		ipu_di_put(ipu_crtc->di);
		ipu_crtc->di = NULL;
	}
	if (!IS_ERR_OR_NULL(ipu_crtc->ipu_dev)) {
		module_put(ipu_crtc->ipu_dev->driver->owner);
		put_device(ipu_crtc->ipu_dev);
		ipu_crtc->ipu_dev = NULL;
	}
}

static int ipu_get_resources(struct ipu_crtc *ipu_crtc,
			     struct device_node *np)
{
	u32 di;
	int ret;

	ret = get_ipu(ipu_crtc, np);
	if (ret) {
		dev_warn(ipu_crtc->dev, "could not get ipu\n");
		return ret;
	}

	/* get our port */
	ipu_crtc->port = of_get_child_by_name(np, "port");
	if (!ipu_crtc->port) {
		dev_err(ipu_crtc->dev, "could not get port\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "di", &di);
	if (ret < 0)
		goto err_out;

	ipu_crtc->di = ipu_di_get(ipu_crtc->ipu, di);
	if (IS_ERR(ipu_crtc->di)) {
		ret = PTR_ERR(ipu_crtc->di);
		ipu_crtc->di = NULL;
		goto err_out;
	}

	if (!of_find_property(np, "dual-plane", NULL)) {
		dev_info(ipu_crtc->dev, "single plane mode\n");
		ipu_crtc->ch = &sync_single_plane;
	} else {
		dev_info(ipu_crtc->dev, "dual plane mode\n");
		ipu_crtc->ch = &sync_dual_plane;
	}

	ipu_crtc->dc = ipu_dc_get(ipu_crtc->ipu, ipu_crtc->ch->dc);
	if (IS_ERR(ipu_crtc->dc)) {
		ret = PTR_ERR(ipu_crtc->dc);
		ipu_crtc->dc = NULL;
		goto err_out;
	}

	return 0;
err_out:
	ipu_put_resources(ipu_crtc);

	return ret;
}

static int ipu_crtc_init(struct ipu_crtc *ipu_crtc,
			 struct drm_device *drm)
{
	struct device_node *np = ipu_crtc->dev->of_node;
	int primary_pipe, overlay_pipe;
	int ret;

	ret = ipu_get_resources(ipu_crtc, np);
	if (ret) {
		dev_err(ipu_crtc->dev, "getting resources failed with %d.\n",
				ret);
		return ret;
	}

	ret = imx_drm_add_crtc(drm, &ipu_crtc->base, &ipu_crtc->plane[0].base,
			       &ipu_crtc->imx_crtc, &ipu_crtc_helper_funcs,
			       ipu_crtc->port);
	if (ret) {
		dev_err(ipu_crtc->dev, "adding crtc failed with %d.\n", ret);
		goto err_put_resources;
	}

	ipu_crtc->id = imx_drm_crtc_id(ipu_crtc->imx_crtc);
	primary_pipe = imx_drm_primary_plane_pipe(ipu_crtc->imx_crtc);

	ret = ipu_plane_init(&ipu_crtc->plane[0], drm,
			     ipu_crtc->ipu,
			     primary_pipe,
			     ipu_crtc->ch->dma[0],
			     ipu_crtc->ch->dp[0],
			     BIT(ipu_crtc->id), true);
	if (ret) {
		dev_err(ipu_crtc->dev, "init primary plane failed with %d\n",
			ret);
		goto err_remove_crtc;
	}

	ret = ipu_plane_get_resources(&ipu_crtc->plane[0]);
	if (ret) {
		dev_err(ipu_crtc->dev, "getting plane 0 resources failed with %d.\n",
			ret);
		goto err_remove_crtc;
	}

	/* If this crtc is using the DP, add an overlay plane */
	if (ipu_crtc->ch->dp[1] >= 0) {
		overlay_pipe = imx_drm_overlay_plane_pipe(ipu_crtc->imx_crtc);
		ret = ipu_plane_init(&ipu_crtc->plane[1], drm,
				     ipu_crtc->ipu,
				     overlay_pipe,
				     ipu_crtc->ch->dma[1],
				     ipu_crtc->ch->dp[1],
				     BIT(ipu_crtc->id), false);

		ipu_crtc->have_overlay = ret ? false : true;
	}

	return 0;

err_remove_crtc:
	imx_drm_remove_crtc(ipu_crtc->imx_crtc);
err_put_resources:
	ipu_put_resources(ipu_crtc);

	return ret;
}

static int ipu_drm_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm = data;
	struct ipu_crtc *ipu_crtc;
	int ret;

	ipu_crtc = devm_kzalloc(dev, sizeof(*ipu_crtc), GFP_KERNEL);
	if (!ipu_crtc)
		return -ENOMEM;

	ipu_crtc->dev = dev;

	ret = ipu_crtc_init(ipu_crtc, drm);
	if (ret)
		return ret;

	dev_set_drvdata(dev, ipu_crtc);
	dev_set_name(dev, "crtc%d", ipu_crtc->id);

	return 0;
}

static void ipu_drm_unbind(struct device *dev, struct device *master,
	void *data)
{
	struct ipu_crtc *ipu_crtc = dev_get_drvdata(dev);

	ipu_fb_disable(ipu_crtc);

	imx_drm_remove_crtc(ipu_crtc->imx_crtc);

	ipu_plane_put_resources(&ipu_crtc->plane[0]);
	ipu_put_resources(ipu_crtc);
}

static const struct component_ops ipu_crtc_ops = {
	.bind = ipu_drm_bind,
	.unbind = ipu_drm_unbind,
};

static int ipu_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	return component_add(dev, &ipu_crtc_ops);
}

static int ipu_drm_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &ipu_crtc_ops);
	return 0;
}

static struct of_device_id ipu_drm_dt_ids[] = {
	{ .compatible = "fsl,imx-ipuv3-crtc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ipu_drm_dt_ids);

static struct platform_driver ipu_drm_driver = {
	.driver = {
		.name = "imx-ipuv3-crtc",
		.of_match_table = ipu_drm_dt_ids,
	},
	.probe = ipu_drm_probe,
	.remove = ipu_drm_remove,
};
module_platform_driver(ipu_drm_driver);

MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-ipuv3-crtc");
