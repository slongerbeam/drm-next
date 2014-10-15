/*
 * include/uapi/drm/imx_drm.h
 *
 * Copyright (C) 2013-2014 Mentor Graphics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __IMX_DRM_H__
#define __IMX_DRM_H__

#include <drm/drm.h>

#define DRM_IMX_GAMMA_SIZE         16
struct drm_imx_gamma {
	bool     enable;
	uint32_t crtc_id;
	uint32_t m[DRM_IMX_GAMMA_SIZE];
	uint32_t b[DRM_IMX_GAMMA_SIZE];
};

#define DRM_IMX_SET_GAMMA    0x00

#define DRM_IOCTL_IMX_SET_GAMMA                                         \
	DRM_IOW(DRM_COMMAND_BASE + DRM_IMX_SET_GAMMA, struct drm_imx_gamma)

#endif /* __IMX_DRM_H__ */
