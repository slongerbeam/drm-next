#ifndef __IPUV3_PLANE_H__
#define __IPUV3_PLANE_H__

#include <drm/drm_crtc.h> /* drm_plane */

struct drm_plane;
struct drm_device;
struct ipu_soc;
struct drm_crtc;
struct drm_framebuffer;

struct ipuv3_channel;
struct dmfc_channel;
struct ipu_dp;

struct ipu_plane {
	struct drm_plane	base;
	int			pipe;

	struct ipu_soc		*ipu;
	struct ipuv3_channel	*ipu_ch;
	struct dmfc_channel	*dmfc;
	struct ipu_dp		*dp;

	int			dma;
	int			dp_flow;

	int			width;
	int			height;
	int			x;
	int			y;

	struct drm_property	*global_alpha_prop;
	struct drm_property	*colorkey_prop;
	bool			global_alpha_en;
	u32			global_alpha;
	bool			colorkey_en;
	u32			colorkey;

	struct drm_pending_vblank_event *page_flip_event;
	struct drm_framebuffer	*newfb;
	int			irq;

	bool			enabled;
};

int ipu_plane_init(struct ipu_plane *ipu_plane, struct drm_device *drm,
		   struct ipu_soc *ipu, int pipe, int dma, int dp,
		   unsigned int possible_crtcs, bool priv);

/* Init IDMAC, DMFC, DP */
int ipu_plane_mode_set(struct ipu_plane *plane, struct drm_crtc *crtc,
		       struct drm_display_mode *mode,
		       struct drm_framebuffer *fb, int crtc_x, int crtc_y,
		       unsigned int crtc_w, unsigned int crtc_h,
		       uint32_t src_x, uint32_t src_y, uint32_t src_w,
		       uint32_t src_h);

int ipu_plane_page_flip(struct drm_plane *plane,
			struct drm_framebuffer *fb,
			struct drm_pending_vblank_event *event,
			uint32_t flags);

int ipu_plane_enable_vblank(struct ipu_plane *ipu_plane);
void ipu_plane_disable_vblank(struct ipu_plane *ipu_plane);

void ipu_plane_enable(struct ipu_plane *plane);
void ipu_plane_disable(struct ipu_plane *plane);

int ipu_plane_get_resources(struct ipu_plane *plane);
void ipu_plane_put_resources(struct ipu_plane *plane);

int ipu_plane_gamma_set(struct ipu_plane *ipu_plane,
			bool enable, u32 *m, u32 *b);

#endif
