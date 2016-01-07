/*
 * Copyright (C) 2012-2014 Mentor Graphics Inc.
 * Copyright 2005-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/bitrev.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include "ipu-prv.h"

/* IC Register Offsets */
#define IC_CONF                 0x0000
#define IC_PRP_ENC_RSC          0x0004
#define IC_PRP_VF_RSC           0x0008
#define IC_PP_RSC               0x000C
#define IC_CMBP_1               0x0010
#define IC_CMBP_2               0x0014
#define IC_IDMAC_1              0x0018
#define IC_IDMAC_2              0x001C
#define IC_IDMAC_3              0x0020
#define IC_IDMAC_4              0x0024

/* IC Register Fields */
#define IC_CONF_PRPENC_EN       (1 << 0)
#define IC_CONF_PRPENC_CSC1     (1 << 1)
#define IC_CONF_PRPENC_ROT_EN   (1 << 2)
#define IC_CONF_PRPVF_EN        (1 << 8)
#define IC_CONF_PRPVF_CSC1      (1 << 9)
#define IC_CONF_PRPVF_CSC2      (1 << 10)
#define IC_CONF_PRPVF_CMB       (1 << 11)
#define IC_CONF_PRPVF_ROT_EN    (1 << 12)
#define IC_CONF_PP_EN           (1 << 16)
#define IC_CONF_PP_CSC1         (1 << 17)
#define IC_CONF_PP_CSC2         (1 << 18)
#define IC_CONF_PP_CMB          (1 << 19)
#define IC_CONF_PP_ROT_EN       (1 << 20)
#define IC_CONF_IC_GLB_LOC_A    (1 << 28)
#define IC_CONF_KEY_COLOR_EN    (1 << 29)
#define IC_CONF_RWS_EN          (1 << 30)
#define IC_CONF_CSI_MEM_WR_EN   (1 << 31)

#define IC_IDMAC_1_CB0_BURST_16         (1 << 0)
#define IC_IDMAC_1_CB1_BURST_16         (1 << 1)
#define IC_IDMAC_1_CB2_BURST_16         (1 << 2)
#define IC_IDMAC_1_CB3_BURST_16         (1 << 3)
#define IC_IDMAC_1_CB4_BURST_16         (1 << 4)
#define IC_IDMAC_1_CB5_BURST_16         (1 << 5)
#define IC_IDMAC_1_CB6_BURST_16         (1 << 6)
#define IC_IDMAC_1_CB7_BURST_16         (1 << 7)
#define IC_IDMAC_1_PRPENC_ROT_MASK      (0x7 << 11)
#define IC_IDMAC_1_PRPENC_ROT_OFFSET    11
#define IC_IDMAC_1_PRPVF_ROT_MASK       (0x7 << 14)
#define IC_IDMAC_1_PRPVF_ROT_OFFSET     14
#define IC_IDMAC_1_PP_ROT_MASK          (0x7 << 17)
#define IC_IDMAC_1_PP_ROT_OFFSET        17
#define IC_IDMAC_1_PP_FLIP_RS           (1 << 22)
#define IC_IDMAC_1_PRPVF_FLIP_RS        (1 << 21)
#define IC_IDMAC_1_PRPENC_FLIP_RS       (1 << 20)

#define IC_IDMAC_2_PRPENC_HEIGHT_MASK   (0x3ff << 0)
#define IC_IDMAC_2_PRPENC_HEIGHT_OFFSET 0
#define IC_IDMAC_2_PRPVF_HEIGHT_MASK    (0x3ff << 10)
#define IC_IDMAC_2_PRPVF_HEIGHT_OFFSET  10
#define IC_IDMAC_2_PP_HEIGHT_MASK       (0x3ff << 20)
#define IC_IDMAC_2_PP_HEIGHT_OFFSET     20

#define IC_IDMAC_3_PRPENC_WIDTH_MASK    (0x3ff << 0)
#define IC_IDMAC_3_PRPENC_WIDTH_OFFSET  0
#define IC_IDMAC_3_PRPVF_WIDTH_MASK     (0x3ff << 10)
#define IC_IDMAC_3_PRPVF_WIDTH_OFFSET   10
#define IC_IDMAC_3_PP_WIDTH_MASK        (0x3ff << 20)
#define IC_IDMAC_3_PP_WIDTH_OFFSET      20

/*
 * The IC Resizer has a restriction that the output frame from the
 * resizer must be 1024 or less in both width (pixels) and height
 * (lines).
 *
 * The image conversion support attempts to split up a conversion when
 * the desired output (converted) frame resolution exceeds the IC resizer
 * limit of 1024 in either dimension.
 *
 * If either dimension of the output frame exceeds the limit, the
 * dimension is split into 1, 2, or 4 equal stripes, for a maximum
 * of 4*4 or 16 tiles. A conversion is then carried out for each
 * tile (but taking care to pass the full frame stride length to
 * the DMA channel's parameter memory!). IDMA double-buffering is used
 * to convert each tile back-to-back when possible (see note below
 * when double_buffering boolean is set).
 *
 * Note that the input frame must be split up into the same number
 * of tiles as the output frame.
 */
#define MAX_STRIPES_W    4
#define MAX_STRIPES_H    4
#define MAX_TILES (MAX_STRIPES_W * MAX_STRIPES_H)

#define MIN_W     128
#define MIN_H     128
#define MAX_W     4096
#define MAX_H     4096

enum image_convert_type {
	IMAGE_CONVERT_IN = 0,
	IMAGE_CONVERT_OUT,
};

struct ic_task_regoffs {
	u32 rsc;
	u32 tpmem_csc[2];
};

struct ic_task_bitfields {
	u32 ic_conf_en;
	u32 ic_conf_rot_en;
	u32 ic_conf_cmb_en;
	u32 ic_conf_csc1_en;
	u32 ic_conf_csc2_en;
	u32 ic_cmb_galpha_bit;
};

struct ic_task_channels {
	int in;
	int out;
	int rot_in;
	int rot_out;
	int vdi_in_p;
	int vdi_in;
	int vdi_in_n;
};

static const struct ic_task_regoffs ic_task_reg[IC_NUM_TASKS] = {
	[IC_TASK_ENCODER] = {
		.rsc = IC_PRP_ENC_RSC,
		.tpmem_csc = {0x2008, 0},
	},
	[IC_TASK_VIEWFINDER] = {
		.rsc = IC_PRP_VF_RSC,
		.tpmem_csc = {0x4028, 0x4040},
	},
	[IC_TASK_POST_PROCESSOR] = {
		.rsc = IC_PP_RSC,
		.tpmem_csc = {0x6060, 0x6078},
	},
};

static const struct ic_task_bitfields ic_task_bit[IC_NUM_TASKS] = {
	[IC_TASK_ENCODER] = {
		.ic_conf_en = IC_CONF_PRPENC_EN,
		.ic_conf_rot_en = IC_CONF_PRPENC_ROT_EN,
		.ic_conf_cmb_en = 0,    /* NA */
		.ic_conf_csc1_en = IC_CONF_PRPENC_CSC1,
		.ic_conf_csc2_en = 0,   /* NA */
		.ic_cmb_galpha_bit = 0, /* NA */
	},
	[IC_TASK_VIEWFINDER] = {
		.ic_conf_en = IC_CONF_PRPVF_EN,
		.ic_conf_rot_en = IC_CONF_PRPVF_ROT_EN,
		.ic_conf_cmb_en = IC_CONF_PRPVF_CMB,
		.ic_conf_csc1_en = IC_CONF_PRPVF_CSC1,
		.ic_conf_csc2_en = IC_CONF_PRPVF_CSC2,
		.ic_cmb_galpha_bit = 0,
	},
	[IC_TASK_POST_PROCESSOR] = {
		.ic_conf_en = IC_CONF_PP_EN,
		.ic_conf_rot_en = IC_CONF_PP_ROT_EN,
		.ic_conf_cmb_en = IC_CONF_PP_CMB,
		.ic_conf_csc1_en = IC_CONF_PP_CSC1,
		.ic_conf_csc2_en = IC_CONF_PP_CSC2,
		.ic_cmb_galpha_bit = 8,
	},
};

static const struct ic_task_channels ic_task_ch[IC_NUM_TASKS] = {
	[IC_TASK_ENCODER] = {
		.out = IPUV3_CHANNEL_IC_PRP_ENC_MEM,
		.rot_in = IPUV3_CHANNEL_MEM_ROT_ENC,
		.rot_out = IPUV3_CHANNEL_ROT_ENC_MEM,
	},
	[IC_TASK_VIEWFINDER] = {
		.in = IPUV3_CHANNEL_MEM_IC_PRP_VF,
		.out = IPUV3_CHANNEL_IC_PRP_VF_MEM,
		.rot_in = IPUV3_CHANNEL_MEM_ROT_VF,
		.rot_out = IPUV3_CHANNEL_ROT_VF_MEM,
		.vdi_in_p = IPUV3_CHANNEL_MEM_VDI_PREV,
		.vdi_in = IPUV3_CHANNEL_MEM_VDI_CUR,
		.vdi_in_n = IPUV3_CHANNEL_MEM_VDI_NEXT,
	},
	[IC_TASK_POST_PROCESSOR] = {
		.in = IPUV3_CHANNEL_MEM_IC_PP,
		.out = IPUV3_CHANNEL_IC_PP_MEM,
		.rot_in = IPUV3_CHANNEL_MEM_ROT_PP,
		.rot_out = IPUV3_CHANNEL_ROT_PP_MEM,
	},
};

struct ipu_ic_dma_buf {
	void          *virt;
	dma_addr_t    phys;
	unsigned long len;
};

/* dimensions of one tile */
struct ipu_ic_tile {
	u32 width;
	u32 height;
	/* size and strides are in bytes */
	u32 size;
	u32 stride;
	u32 rot_stride;
	/* start Y or packed offset of this tile */
	u32 offset;
	/* offset from start to tile in U plane, for planar formats */
	u32 u_off;
	/* offset from start to tile in V plane, for planar formats */
	u32 v_off;
};

struct ipu_ic_pixfmt {
	char	*name;
	u32	fourcc;        /* V4L2 fourcc */
	int     bpp;           /* total bpp */
	int     y_depth;       /* depth of Y plane for planar formats */
	int     uv_width_dec;  /* decimation in width for U/V planes */
	int     uv_height_dec; /* decimation in height for U/V planes */
	bool    uv_swapped;    /* U and V planes are swapped */
	bool    uv_packed;     /* partial planar (U and V in same plane) */
};

struct ipu_ic_image {
	struct ipu_image base;
	enum image_convert_type type;

	const struct ipu_ic_pixfmt *fmt;
	unsigned int stride;

	/* # of rows (horizontal stripes) if dest height is > 1024 */
	unsigned int num_rows;
	/* # of columns (vertical stripes) if dest width is > 1024 */
	unsigned int num_cols;

	struct ipu_ic_tile tile[MAX_TILES];
};

struct image_converter_ctx;
struct image_converter;
struct ipu_ic_priv;
struct ipu_ic;

struct image_converter_run {
	struct image_converter_ctx *ctx;

	dma_addr_t in_phys;
	dma_addr_t out_phys;

	int status;

	struct list_head list;
};

struct image_converter_ctx {
	struct image_converter *cvt;

	image_converter_cb_t complete;
	void *complete_context;

	/* Source/destination image data and rotation mode */
	struct ipu_ic_image in;
	struct ipu_ic_image out;
	enum ipu_rotate_mode rot_mode;

	/* intermediate buffer for rotation */
	struct ipu_ic_dma_buf rot_intermediate[2];

	/* current buffer number for double buffering */
	int cur_buf_num;

	bool aborting;
	struct completion aborted;

	/* can we use double-buffering for this conversion operation? */
	bool double_buffering;
	/* num_rows * num_cols */
	unsigned int num_tiles;
	/* next tile to process */
	unsigned int next_tile;
	/* where to place converted tile in dest image */
	unsigned int out_tile_map[MAX_TILES];

	struct list_head list;
};

struct image_converter {
	struct ipu_ic *ic;

	struct ipuv3_channel *in_chan;
	struct ipuv3_channel *out_chan;
	struct ipuv3_channel *rotation_in_chan;
	struct ipuv3_channel *rotation_out_chan;

	/* the IPU end-of-frame irqs */
	int out_eof_irq;
	int rot_out_eof_irq;

	spinlock_t irqlock;

	/* list of convert contexts */
	struct list_head ctx_list;
	/* queue of conversion runs */
	struct list_head pending_q;
	/* queue of completed runs */
	struct list_head done_q;

	/* the current conversion run */
	struct image_converter_run *current_run;
};

struct ipu_ic {
	enum ipu_ic_task task;
	const struct ic_task_regoffs *reg;
	const struct ic_task_bitfields *bit;
	const struct ic_task_channels *ch;

	enum ipu_color_space in_cs, g_in_cs;
	enum ipu_color_space out_cs;
	bool graphics;
	bool rotation;

	struct image_converter cvt;

	struct ipu_ic_priv *priv;
};

struct ipu_ic_priv {
	void __iomem *base;
	void __iomem *tpmem_base;
	spinlock_t lock;
	struct ipu_soc *ipu;
	int use_count;
	int irt_use_count;
	struct ipu_ic task[IC_NUM_TASKS];
};

static inline u32 ipu_ic_read(struct ipu_ic *ic, unsigned offset)
{
	return readl(ic->priv->base + offset);
}

static inline void ipu_ic_write(struct ipu_ic *ic, u32 value, unsigned offset)
{
	writel(value, ic->priv->base + offset);
}

struct ic_csc_params {
	s16 coeff[3][3];	/* signed 9-bit integer coefficients */
	s16 offset[3];		/* signed 11+2-bit fixed point offset */
	u8 scale:2;		/* scale coefficients * 2^(scale-1) */
	bool sat:1;		/* saturate to (16, 235(Y) / 240(U, V)) */
};

/*
 * Y = R *  .299 + G *  .587 + B *  .114;
 * U = R * -.169 + G * -.332 + B *  .500 + 128.;
 * V = R *  .500 + G * -.419 + B * -.0813 + 128.;
 */
static const struct ic_csc_params ic_csc_rgb2ycbcr = {
	.coeff = {
		{ 77, 150, 29 },
		{ 469, 427, 128 },
		{ 128, 405, 491 },
	},
	.offset = { 0, 512, 512 },
	.scale = 1,
};

/* transparent RGB->RGB matrix for graphics combining */
static const struct ic_csc_params ic_csc_rgb2rgb = {
	.coeff = {
		{ 128, 0, 0 },
		{ 0, 128, 0 },
		{ 0, 0, 128 },
	},
	.scale = 2,
};

/*
 * R = (1.164 * (Y - 16)) + (1.596 * (Cr - 128));
 * G = (1.164 * (Y - 16)) - (0.392 * (Cb - 128)) - (0.813 * (Cr - 128));
 * B = (1.164 * (Y - 16)) + (2.017 * (Cb - 128);
 */
static const struct ic_csc_params ic_csc_ycbcr2rgb = {
	.coeff = {
		{ 149, 0, 204 },
		{ 149, 462, 408 },
		{ 149, 255, 0 },
	},
	.offset = { -446, 266, -554 },
	.scale = 2,
};

static int init_csc(struct ipu_ic *ic,
		    enum ipu_color_space inf,
		    enum ipu_color_space outf,
		    int csc_index)
{
	struct ipu_ic_priv *priv = ic->priv;
	const struct ic_csc_params *params;
	u32 __iomem *base;
	const u16 (*c)[3];
	const u16 *a;
	u32 param;

	base = (u32 __iomem *)
		(priv->tpmem_base + ic->reg->tpmem_csc[csc_index]);

	if (inf == IPUV3_COLORSPACE_YUV && outf == IPUV3_COLORSPACE_RGB)
		params = &ic_csc_ycbcr2rgb;
	else if (inf == IPUV3_COLORSPACE_RGB && outf == IPUV3_COLORSPACE_YUV)
		params = &ic_csc_rgb2ycbcr;
	else if (inf == IPUV3_COLORSPACE_RGB && outf == IPUV3_COLORSPACE_RGB)
		params = &ic_csc_rgb2rgb;
	else {
		dev_err(priv->ipu->dev, "Unsupported color space conversion\n");
		return -EINVAL;
	}

	/* Cast to unsigned */
	c = (const u16 (*)[3])params->coeff;
	a = (const u16 *)params->offset;

	param = ((a[0] & 0x1f) << 27) | ((c[0][0] & 0x1ff) << 18) |
		((c[1][1] & 0x1ff) << 9) | (c[2][2] & 0x1ff);
	writel(param, base++);

	param = ((a[0] & 0x1fe0) >> 5) | (params->scale << 8) |
		(params->sat << 9);
	writel(param, base++);

	param = ((a[1] & 0x1f) << 27) | ((c[0][1] & 0x1ff) << 18) |
		((c[1][0] & 0x1ff) << 9) | (c[2][0] & 0x1ff);
	writel(param, base++);

	param = ((a[1] & 0x1fe0) >> 5);
	writel(param, base++);

	param = ((a[2] & 0x1f) << 27) | ((c[0][2] & 0x1ff) << 18) |
		((c[1][2] & 0x1ff) << 9) | (c[2][1] & 0x1ff);
	writel(param, base++);

	param = ((a[2] & 0x1fe0) >> 5);
	writel(param, base++);

	return 0;
}

static int calc_resize_coeffs(struct ipu_ic *ic,
			      u32 in_size, u32 out_size,
			      u32 *resize_coeff,
			      u32 *downsize_coeff)
{
	struct ipu_ic_priv *priv = ic->priv;
	struct ipu_soc *ipu = priv->ipu;
	u32 temp_size, temp_downsize;

	/*
	 * Input size cannot be more than 4096, and output size cannot
	 * be more than 1024
	 */
	if (in_size > 4096) {
		dev_err(ipu->dev, "Unsupported resize (in_size > 4096)\n");
		return -EINVAL;
	}
	if (out_size > 1024) {
		dev_err(ipu->dev, "Unsupported resize (out_size > 1024)\n");
		return -EINVAL;
	}

	/* Cannot downsize more than 4:1 */
	if ((out_size << 2) < in_size) {
		dev_err(ipu->dev, "Unsupported downsize\n");
		return -EINVAL;
	}

	/* Compute downsizing coefficient */
	temp_downsize = 0;
	temp_size = in_size;
	while (((temp_size > 1024) || (temp_size >= out_size * 2)) &&
	       (temp_downsize < 2)) {
		temp_size >>= 1;
		temp_downsize++;
	}
	*downsize_coeff = temp_downsize;

	/*
	 * compute resizing coefficient using the following equation:
	 * resize_coeff = M * (SI - 1) / (SO - 1)
	 * where M = 2^13, SI = input size, SO = output size
	 */
	*resize_coeff = (8192L * (temp_size - 1)) / (out_size - 1);
	if (*resize_coeff >= 16384L) {
		dev_err(ipu->dev, "Warning! Overflow on resize coeff.\n");
		*resize_coeff = 0x3FFF;
	}

	return 0;
}

void ipu_ic_task_enable(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;
	unsigned long flags;
	u32 ic_conf;

	spin_lock_irqsave(&priv->lock, flags);

	ic_conf = ipu_ic_read(ic, IC_CONF);

	ic_conf |= ic->bit->ic_conf_en;

	if (ic->rotation)
		ic_conf |= ic->bit->ic_conf_rot_en;

	if (ic->in_cs != ic->out_cs)
		ic_conf |= ic->bit->ic_conf_csc1_en;

	if (ic->graphics) {
		ic_conf |= ic->bit->ic_conf_cmb_en;
		ic_conf |= ic->bit->ic_conf_csc1_en;

		if (ic->g_in_cs != ic->out_cs)
			ic_conf |= ic->bit->ic_conf_csc2_en;
	}

	ipu_ic_write(ic, ic_conf, IC_CONF);

	spin_unlock_irqrestore(&priv->lock, flags);
}
EXPORT_SYMBOL_GPL(ipu_ic_task_enable);

void ipu_ic_task_disable(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;
	unsigned long flags;
	u32 ic_conf;

	spin_lock_irqsave(&priv->lock, flags);

	ic_conf = ipu_ic_read(ic, IC_CONF);

	ic_conf &= ~(ic->bit->ic_conf_en |
		     ic->bit->ic_conf_csc1_en |
		     ic->bit->ic_conf_rot_en);
	if (ic->bit->ic_conf_csc2_en)
		ic_conf &= ~ic->bit->ic_conf_csc2_en;
	if (ic->bit->ic_conf_cmb_en)
		ic_conf &= ~ic->bit->ic_conf_cmb_en;

	ipu_ic_write(ic, ic_conf, IC_CONF);

	spin_unlock_irqrestore(&priv->lock, flags);
}
EXPORT_SYMBOL_GPL(ipu_ic_task_disable);

int ipu_ic_task_graphics_init(struct ipu_ic *ic,
			      enum ipu_color_space in_g_cs,
			      bool galpha_en, u32 galpha,
			      bool colorkey_en, u32 colorkey)
{
	struct ipu_ic_priv *priv = ic->priv;
	unsigned long flags;
	u32 reg, ic_conf;
	int ret = 0;

	if (ic->task == IC_TASK_ENCODER)
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, flags);

	ic_conf = ipu_ic_read(ic, IC_CONF);

	if (!(ic_conf & ic->bit->ic_conf_csc1_en)) {
		/* need transparent CSC1 conversion */
		ret = init_csc(ic, IPUV3_COLORSPACE_RGB,
			       IPUV3_COLORSPACE_RGB, 0);
		if (ret)
			goto unlock;
	}

	ic->g_in_cs = in_g_cs;

	if (ic->g_in_cs != ic->out_cs) {
		ret = init_csc(ic, ic->g_in_cs, ic->out_cs, 1);
		if (ret)
			goto unlock;
	}

	if (galpha_en) {
		ic_conf |= IC_CONF_IC_GLB_LOC_A;
		reg = ipu_ic_read(ic, IC_CMBP_1);
		reg &= ~(0xff << ic->bit->ic_cmb_galpha_bit);
		reg |= (galpha << ic->bit->ic_cmb_galpha_bit);
		ipu_ic_write(ic, reg, IC_CMBP_1);
	} else
		ic_conf &= ~IC_CONF_IC_GLB_LOC_A;

	if (colorkey_en) {
		ic_conf |= IC_CONF_KEY_COLOR_EN;
		ipu_ic_write(ic, colorkey, IC_CMBP_2);
	} else
		ic_conf &= ~IC_CONF_KEY_COLOR_EN;

	ipu_ic_write(ic, ic_conf, IC_CONF);

	ic->graphics = true;
unlock:
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(ipu_ic_task_graphics_init);

int ipu_ic_task_init(struct ipu_ic *ic,
		     int in_width, int in_height,
		     int out_width, int out_height,
		     enum ipu_color_space in_cs,
		     enum ipu_color_space out_cs)
{
	struct ipu_ic_priv *priv = ic->priv;
	u32 reg, downsize_coeff, resize_coeff;
	unsigned long flags;
	int ret = 0;

	/* Setup vertical resizing */
	ret = calc_resize_coeffs(ic, in_height, out_height,
				 &resize_coeff, &downsize_coeff);
	if (ret)
		return ret;

	reg = (downsize_coeff << 30) | (resize_coeff << 16);

	/* Setup horizontal resizing */
	ret = calc_resize_coeffs(ic, in_width, out_width,
				 &resize_coeff, &downsize_coeff);
	if (ret)
		return ret;

	reg |= (downsize_coeff << 14) | resize_coeff;

	spin_lock_irqsave(&priv->lock, flags);

	ipu_ic_write(ic, reg, ic->reg->rsc);

	/* Setup color space conversion */
	ic->in_cs = in_cs;
	ic->out_cs = out_cs;

	if (ic->in_cs != ic->out_cs) {
		ret = init_csc(ic, ic->in_cs, ic->out_cs, 0);
		if (ret)
			goto unlock;
	}

unlock:
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(ipu_ic_task_init);

int ipu_ic_task_idma_init(struct ipu_ic *ic, struct ipuv3_channel *channel,
			  u32 width, u32 height, int burst_size,
			  enum ipu_rotate_mode rot)
{
	struct ipu_ic_priv *priv = ic->priv;
	struct ipu_soc *ipu = priv->ipu;
	u32 ic_idmac_1, ic_idmac_2, ic_idmac_3;
	u32 temp_rot = bitrev8(rot) >> 5;
	bool need_hor_flip = false;
	unsigned long flags;
	int ret = 0;

	if ((burst_size != 8) && (burst_size != 16)) {
		dev_err(ipu->dev, "Illegal burst length for IC\n");
		return -EINVAL;
	}

	width--;
	height--;

	if (temp_rot & 0x2)	/* Need horizontal flip */
		need_hor_flip = true;

	spin_lock_irqsave(&priv->lock, flags);

	ic_idmac_1 = ipu_ic_read(ic, IC_IDMAC_1);
	ic_idmac_2 = ipu_ic_read(ic, IC_IDMAC_2);
	ic_idmac_3 = ipu_ic_read(ic, IC_IDMAC_3);

	switch (channel->num) {
	case IPUV3_CHANNEL_IC_PP_MEM:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB2_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB2_BURST_16;

		if (need_hor_flip)
			ic_idmac_1 |= IC_IDMAC_1_PP_FLIP_RS;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_PP_FLIP_RS;

		ic_idmac_2 &= ~IC_IDMAC_2_PP_HEIGHT_MASK;
		ic_idmac_2 |= height << IC_IDMAC_2_PP_HEIGHT_OFFSET;

		ic_idmac_3 &= ~IC_IDMAC_3_PP_WIDTH_MASK;
		ic_idmac_3 |= width << IC_IDMAC_3_PP_WIDTH_OFFSET;
		break;
	case IPUV3_CHANNEL_MEM_IC_PP:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB5_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB5_BURST_16;
		break;
	case IPUV3_CHANNEL_MEM_ROT_PP:
		ic_idmac_1 &= ~IC_IDMAC_1_PP_ROT_MASK;
		ic_idmac_1 |= temp_rot << IC_IDMAC_1_PP_ROT_OFFSET;
		break;
	case IPUV3_CHANNEL_MEM_IC_PRP_VF:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB6_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB6_BURST_16;
		break;
	case IPUV3_CHANNEL_IC_PRP_ENC_MEM:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB0_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB0_BURST_16;

		if (need_hor_flip)
			ic_idmac_1 |= IC_IDMAC_1_PRPENC_FLIP_RS;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_PRPENC_FLIP_RS;

		ic_idmac_2 &= ~IC_IDMAC_2_PRPENC_HEIGHT_MASK;
		ic_idmac_2 |= height << IC_IDMAC_2_PRPENC_HEIGHT_OFFSET;

		ic_idmac_3 &= ~IC_IDMAC_3_PRPENC_WIDTH_MASK;
		ic_idmac_3 |= width << IC_IDMAC_3_PRPENC_WIDTH_OFFSET;
		break;
	case IPUV3_CHANNEL_MEM_ROT_ENC:
		ic_idmac_1 &= ~IC_IDMAC_1_PRPENC_ROT_MASK;
		ic_idmac_1 |= temp_rot << IC_IDMAC_1_PRPENC_ROT_OFFSET;
		break;
	case IPUV3_CHANNEL_IC_PRP_VF_MEM:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB1_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB1_BURST_16;

		if (need_hor_flip)
			ic_idmac_1 |= IC_IDMAC_1_PRPVF_FLIP_RS;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_PRPVF_FLIP_RS;

		ic_idmac_2 &= ~IC_IDMAC_2_PRPVF_HEIGHT_MASK;
		ic_idmac_2 |= height << IC_IDMAC_2_PRPVF_HEIGHT_OFFSET;

		ic_idmac_3 &= ~IC_IDMAC_3_PRPVF_WIDTH_MASK;
		ic_idmac_3 |= width << IC_IDMAC_3_PRPVF_WIDTH_OFFSET;
		break;
	case IPUV3_CHANNEL_MEM_ROT_VF:
		ic_idmac_1 &= ~IC_IDMAC_1_PRPVF_ROT_MASK;
		ic_idmac_1 |= temp_rot << IC_IDMAC_1_PRPVF_ROT_OFFSET;
		break;
	case IPUV3_CHANNEL_G_MEM_IC_PRP_VF:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB3_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB3_BURST_16;
		break;
	case IPUV3_CHANNEL_G_MEM_IC_PP:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB4_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB4_BURST_16;
		break;
	case IPUV3_CHANNEL_VDI_MEM_IC_VF:
		if (burst_size == 16)
			ic_idmac_1 |= IC_IDMAC_1_CB7_BURST_16;
		else
			ic_idmac_1 &= ~IC_IDMAC_1_CB7_BURST_16;
		break;
	default:
		goto unlock;
	}

	ipu_ic_write(ic, ic_idmac_1, IC_IDMAC_1);
	ipu_ic_write(ic, ic_idmac_2, IC_IDMAC_2);
	ipu_ic_write(ic, ic_idmac_3, IC_IDMAC_3);

	if (ipu_rot_mode_is_irt(rot))
		ic->rotation = true;

unlock:
	spin_unlock_irqrestore(&priv->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(ipu_ic_task_idma_init);

static void ipu_irt_enable(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;

	if (!priv->irt_use_count)
		ipu_module_enable(priv->ipu, IPU_CONF_ROT_EN);

	priv->irt_use_count++;
}

static void ipu_irt_disable(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;

	if (priv->irt_use_count) {
		if (!--priv->irt_use_count)
			ipu_module_disable(priv->ipu, IPU_CONF_ROT_EN);
	}
}

/*
 * Complete image conversion support follows
 */

static const struct ipu_ic_pixfmt ipu_ic_formats[] = {
	{
		.name	= "RGB565",
		.fourcc	= V4L2_PIX_FMT_RGB565,
		.bpp    = 16,
	}, {
		.name	= "RGB24",
		.fourcc	= V4L2_PIX_FMT_RGB24,
		.bpp    = 24,
	}, {
		.name	= "BGR24",
		.fourcc	= V4L2_PIX_FMT_BGR24,
		.bpp    = 24,
	}, {
		.name	= "RGB32",
		.fourcc	= V4L2_PIX_FMT_RGB32,
		.bpp    = 32,
	}, {
		.name	= "BGR32",
		.fourcc	= V4L2_PIX_FMT_BGR32,
		.bpp    = 32,
	}, {
		.name	= "4:2:2 packed, YUYV",
		.fourcc	= V4L2_PIX_FMT_YUYV,
		.bpp    = 16,
		.uv_width_dec = 2,
		.uv_height_dec = 1,
	}, {
		.name	= "4:2:2 packed, UYVY",
		.fourcc	= V4L2_PIX_FMT_UYVY,
		.bpp    = 16,
		.uv_width_dec = 2,
		.uv_height_dec = 1,
	}, {
		.name	= "4:2:0 planar, YUV",
		.fourcc	= V4L2_PIX_FMT_YUV420,
		.bpp    = 12,
		.y_depth = 8,
		.uv_width_dec = 2,
		.uv_height_dec = 2,
	}, {
		.name	= "4:2:0 planar, YVU",
		.fourcc	= V4L2_PIX_FMT_YVU420,
		.bpp    = 12,
		.y_depth = 8,
		.uv_width_dec = 2,
		.uv_height_dec = 2,
		.uv_swapped = true,
	}, {
		.name   = "4:2:0 partial planar, NV12",
		.fourcc = V4L2_PIX_FMT_NV12,
		.bpp    = 12,
		.y_depth = 8,
		.uv_width_dec = 2,
		.uv_height_dec = 2,
		.uv_packed = true,
	}, {
		.name   = "4:2:2 planar, YUV",
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.bpp    = 16,
		.y_depth = 8,
		.uv_width_dec = 2,
		.uv_height_dec = 1,
	}, {
		.name   = "4:2:2 partial planar, NV16",
		.fourcc = V4L2_PIX_FMT_NV16,
		.bpp    = 16,
		.y_depth = 8,
		.uv_width_dec = 2,
		.uv_height_dec = 1,
		.uv_packed = true,
	},
};

static const struct ipu_ic_pixfmt *ipu_ic_get_format(u32 fourcc)
{
	const struct ipu_ic_pixfmt *ret = NULL;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ipu_ic_formats); i++) {
		if (ipu_ic_formats[i].fourcc == fourcc) {
			ret = &ipu_ic_formats[i];
			break;
		}
	}

	return ret;
}

static void ipu_ic_dump_format(struct image_converter_ctx *ctx,
			       struct ipu_ic_image *ic_image)
{
	struct ipu_ic_priv *priv = ctx->cvt->ic->priv;

	dev_dbg(priv->ipu->dev,
		"ctx %p: %s format: %dx%d (%dx%d tiles of size %dx%d), %c%c%c%c\n",
		ctx,
		ic_image->type == IMAGE_CONVERT_OUT ? "Output" : "Input",
		ic_image->base.pix.width, ic_image->base.pix.height,
		ic_image->num_cols, ic_image->num_rows,
		ic_image->tile[0].width, ic_image->tile[0].height,
		ic_image->fmt->fourcc & 0xff,
		(ic_image->fmt->fourcc >> 8) & 0xff,
		(ic_image->fmt->fourcc >> 16) & 0xff,
		(ic_image->fmt->fourcc >> 24) & 0xff);
}

int ipu_image_convert_enum_format(int index, const char **desc, u32 *fourcc)
{
	const struct ipu_ic_pixfmt *fmt;

	if (index >= (int)ARRAY_SIZE(ipu_ic_formats))
		return -EINVAL;

	/* Format found */
	fmt = &ipu_ic_formats[index];
	*desc = fmt->name;
	*fourcc = fmt->fourcc;
	return 0;
}
EXPORT_SYMBOL_GPL(ipu_image_convert_enum_format);

static void ipu_ic_free_dma_buf(struct ipu_ic_priv *priv,
				struct ipu_ic_dma_buf *buf)
{
	if (buf->virt)
		dma_free_coherent(priv->ipu->dev,
				  buf->len, buf->virt, buf->phys);
	buf->virt = NULL;
	buf->phys = 0;
}

static int ipu_ic_alloc_dma_buf(struct ipu_ic_priv *priv,
				struct ipu_ic_dma_buf *buf,
				int size)
{
	unsigned long newlen = PAGE_ALIGN(size);

	if (buf->virt) {
		if (buf->len == newlen)
			return 0;
		ipu_ic_free_dma_buf(priv, buf);
	}

	buf->len = newlen;
	buf->virt = dma_alloc_coherent(priv->ipu->dev, buf->len, &buf->phys,
				       GFP_DMA | GFP_KERNEL);
	if (!buf->virt) {
		dev_err(priv->ipu->dev, "failed to alloc dma buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static inline int ipu_ic_num_stripes(int dim)
{
	if (dim <= 1024)
		return 1;
	else if (dim <= 2048)
		return 2;
	else
		return 4;
}

static void ipu_ic_calc_tile_dimensions(struct image_converter_ctx *ctx,
					struct ipu_ic_image *image)
{
	int i;

	for (i = 0; i < ctx->num_tiles; i++) {
		struct ipu_ic_tile *tile = &image->tile[i];

		tile->height = image->base.pix.height / image->num_rows;
		tile->width = image->base.pix.width / image->num_cols;
		tile->size = ((tile->height * image->fmt->bpp) >> 3) *
			tile->width;

		if (image->fmt->y_depth) {
			tile->stride =
				(image->fmt->y_depth * tile->width) >> 3;
			tile->rot_stride =
				(image->fmt->y_depth * tile->height) >> 3;
		} else {
			tile->stride =
				(image->fmt->bpp * tile->width) >> 3;
			tile->rot_stride =
				(image->fmt->bpp * tile->height) >> 3;
		}
	}
}

/*
 * Use the rotation transformation to find the tile coordinates
 * (row, col) of a tile in the destination frame that corresponds
 * to the given tile coordinates of a source frame. The destination
 * coordinate is then converted to a tile index.
 */
static int ipu_ic_transform_tile_index(struct image_converter_ctx *ctx,
				       int src_row, int src_col)
{
	struct ipu_ic_priv *priv = ctx->cvt->ic->priv;
	struct ipu_ic_image *s_image = &ctx->in;
	struct ipu_ic_image *d_image = &ctx->out;
	int cos, sin, dst_row, dst_col;

	/* with no rotation it's a 1:1 mapping */
	if (ctx->rot_mode == IPU_ROTATE_NONE)
		return src_row * s_image->num_cols + src_col;

	if (ctx->rot_mode & IPU_ROT_BIT_90) {
		cos = 0;
		sin = 1;
	} else {
		cos = 1;
		sin = 0;
	}

	/*
	 * before doing the transform, first we have to translate
	 * source row,col for an origin in the center of s_image
	 */
	src_row *= 2;
	src_col *= 2;
	src_row -= s_image->num_rows - 1;
	src_col -= s_image->num_cols - 1;

	/* do the rotation transform */
	dst_col = src_col * cos - src_row * sin;
	dst_row = src_col * sin + src_row * cos;

	/* apply flip */
	if (ctx->rot_mode & IPU_ROT_BIT_HFLIP)
		dst_col = -dst_col;
	if (ctx->rot_mode & IPU_ROT_BIT_VFLIP)
		dst_row = -dst_row;

	dev_dbg(priv->ipu->dev, "ctx %p: [%d,%d] --> [%d,%d]\n",
		ctx, src_col, src_row, dst_col, dst_row);

	/*
	 * finally translate dest row,col using an origin in upper
	 * left of d_image
	 */
	dst_row += d_image->num_rows - 1;
	dst_col += d_image->num_cols - 1;
	dst_row /= 2;
	dst_col /= 2;

	return dst_row * d_image->num_cols + dst_col;
}

/*
 * Fill the out_tile_map[] with transformed destination tile indeces.
 */
static void ipu_ic_calc_out_tile_map(struct image_converter_ctx *ctx)
{
	struct ipu_ic_image *s_image = &ctx->in;
	unsigned int row, col, tile = 0;

	for (row = 0; row < s_image->num_rows; row++) {
		for (col = 0; col < s_image->num_cols; col++) {
			ctx->out_tile_map[tile] =
				ipu_ic_transform_tile_index(ctx, row, col);
			tile++;
		}
	}
}

static void ipu_ic_calc_tile_offsets_planar(struct image_converter_ctx *ctx,
					    struct ipu_ic_image *image)
{
	struct ipu_ic_priv *priv = ctx->cvt->ic->priv;
	const struct ipu_ic_pixfmt *fmt = image->fmt;
	unsigned int row, col, tile = 0;
	u32 H, w, h, y_depth, y_stride, uv_stride;
	u32 uv_row_off, uv_col_off, uv_off, u_off, v_off, tmp;
	u32 y_row_off, y_col_off, y_off;
	u32 y_size, uv_size;

	/* setup some convenience vars */
	H = image->base.pix.height;

	y_depth = fmt->y_depth;
	y_stride = image->stride;
	uv_stride = y_stride / fmt->uv_width_dec;
	if (fmt->uv_packed)
		uv_stride *= 2;

	y_size = H * y_stride;
	uv_size = y_size / (fmt->uv_width_dec * fmt->uv_height_dec);

	for (row = 0; row < image->num_rows; row++) {
		w = image->tile[tile].width;
		h = image->tile[tile].height;
		y_row_off = row * h * y_stride;
		uv_row_off = (row * h * uv_stride) / fmt->uv_height_dec;

		for (col = 0; col < image->num_cols; col++) {
			y_col_off = (col * w * y_depth) >> 3;
			uv_col_off = y_col_off / fmt->uv_width_dec;
			if (fmt->uv_packed)
				uv_col_off *= 2;

			y_off = y_row_off + y_col_off;
			uv_off = uv_row_off + uv_col_off;

			u_off = y_size - y_off + uv_off;
			v_off = (fmt->uv_packed) ? 0 : u_off + uv_size;
			if (fmt->uv_swapped) {
				tmp = u_off;
				u_off = v_off;
				v_off = tmp;
			}

			image->tile[tile].offset = y_off;
			image->tile[tile].u_off = u_off;
			image->tile[tile++].v_off = v_off;

			dev_dbg(priv->ipu->dev,
				"ctx %p: %s@[%d,%d]: y_off %08x, u_off %08x, v_off %08x\n",
				ctx, image->type == IMAGE_CONVERT_IN ?
				"Input" : "Output", row, col,
				y_off, u_off, v_off);
		}
	}
}

static void ipu_ic_calc_tile_offsets_packed(struct image_converter_ctx *ctx,
					    struct ipu_ic_image *image)
{
	struct ipu_ic_priv *priv = ctx->cvt->ic->priv;
	const struct ipu_ic_pixfmt *fmt = image->fmt;
	unsigned int row, col, tile = 0;
	u32 w, h, bpp, stride;
	u32 row_off, col_off;

	/* setup some convenience vars */
	stride = image->stride;
	bpp = fmt->bpp;

	for (row = 0; row < image->num_rows; row++) {
		w = image->tile[tile].width;
		h = image->tile[tile].height;
		row_off = row * h * stride;

		for (col = 0; col < image->num_cols; col++) {
			col_off = (col * w * bpp) >> 3;

			image->tile[tile].offset = row_off + col_off;
			image->tile[tile].u_off = 0;
			image->tile[tile++].v_off = 0;

			dev_dbg(priv->ipu->dev,
				"ctx %p: %s@[%d,%d]: phys %08x\n", ctx,
				image->type == IMAGE_CONVERT_IN ?
				"Input" : "Output", row, col,
				row_off + col_off);
		}
	}
}

static void ipu_ic_calc_tile_offsets(struct image_converter_ctx *ctx,
				     struct ipu_ic_image *image)
{
	if (image->fmt->y_depth)
		ipu_ic_calc_tile_offsets_planar(ctx, image);
	else
		ipu_ic_calc_tile_offsets_packed(ctx, image);
}

/*
 * return the number of runs in given queue (pending_q or done_q)
 * for this context. hold irqlock when calling.
 */
static int ipu_ic_get_run_count(struct image_converter_ctx *ctx,
				struct list_head *q)
{
	struct image_converter_run *run;
	int count = 0;

	list_for_each_entry(run, q, list) {
		if (run->ctx == ctx)
			count++;
	}

	return count;
}

/* hold irqlock when calling */
static void ipu_ic_convert_stop(struct image_converter_run *run)
{
	struct image_converter_ctx *ctx = run->ctx;
	struct image_converter *cvt = ctx->cvt;
	struct ipu_ic_priv *priv = cvt->ic->priv;

	dev_dbg(priv->ipu->dev, "%s: stopping ctx %p run %p\n",
		__func__, ctx, run);

	/* disable IC tasks and the channels */
	ipu_ic_task_disable(cvt->ic);
	ipu_idmac_disable_channel(cvt->in_chan);
	ipu_idmac_disable_channel(cvt->out_chan);

	if (ipu_rot_mode_is_irt(ctx->rot_mode)) {
		ipu_idmac_disable_channel(cvt->rotation_in_chan);
		ipu_idmac_disable_channel(cvt->rotation_out_chan);
		ipu_idmac_unlink(cvt->out_chan, cvt->rotation_in_chan);
	}

	ipu_ic_disable(cvt->ic);
}

/* hold irqlock when calling */
static void init_idmac_channel(struct image_converter_ctx *ctx,
			       struct ipuv3_channel *channel,
			       struct ipu_ic_image *image,
			       enum ipu_rotate_mode rot_mode,
			       bool rot_swap_width_height)
{
	struct image_converter *cvt = ctx->cvt;
	unsigned int burst_size;
	u32 width, height, stride;
	dma_addr_t addr0, addr1 = 0;
	struct ipu_image tile_image;
	unsigned int tile_idx[2];

	if (image->type == IMAGE_CONVERT_OUT) {
		tile_idx[0] = ctx->out_tile_map[0];
		tile_idx[1] = ctx->out_tile_map[1];
	} else {
		tile_idx[0] = 0;
		tile_idx[1] = 1;
	}

	if (rot_swap_width_height) {
		width = image->tile[0].height;
		height = image->tile[0].width;
		stride = image->tile[0].rot_stride;
		addr0 = ctx->rot_intermediate[0].phys;
		if (ctx->double_buffering)
			addr1 = ctx->rot_intermediate[1].phys;
	} else {
		width = image->tile[0].width;
		height = image->tile[0].height;
		stride = image->stride;
		addr0 = image->base.phys0 +
			image->tile[tile_idx[0]].offset;
		if (ctx->double_buffering)
			addr1 = image->base.phys0 +
				image->tile[tile_idx[1]].offset;
	}

	ipu_cpmem_zero(channel);

	memset(&tile_image, 0, sizeof(tile_image));
	tile_image.pix.width = tile_image.rect.width = width;
	tile_image.pix.height = tile_image.rect.height = height;
	tile_image.pix.bytesperline = stride;
	tile_image.pix.pixelformat =  image->fmt->fourcc;
	tile_image.phys0 = addr0;
	tile_image.phys1 = addr1;
	ipu_cpmem_set_image(channel, &tile_image);

	if (image->fmt->y_depth && !rot_swap_width_height)
		ipu_cpmem_set_uv_offset(channel,
					image->tile[tile_idx[0]].u_off,
					image->tile[tile_idx[0]].v_off);

	if (rot_mode)
		ipu_cpmem_set_rotation(channel, rot_mode);

	if (channel == cvt->rotation_in_chan ||
	    channel == cvt->rotation_out_chan) {
		burst_size = 8;
		ipu_cpmem_set_block_mode(channel);
	} else
		burst_size = (width % 16) ? 8 : 16;

	ipu_cpmem_set_burstsize(channel, burst_size);

	ipu_ic_task_idma_init(cvt->ic, channel, width, height,
			      burst_size, rot_mode);

	ipu_cpmem_set_axi_id(channel, 1);

	ipu_idmac_set_double_buffer(channel, ctx->double_buffering);
}

/* hold irqlock when calling */
static int ipu_ic_convert_start(struct image_converter_run *run)
{
	struct image_converter_ctx *ctx = run->ctx;
	struct image_converter *cvt = ctx->cvt;
	struct ipu_ic_priv *priv = cvt->ic->priv;
	struct ipu_ic_image *s_image = &ctx->in;
	struct ipu_ic_image *d_image = &ctx->out;
	enum ipu_color_space src_cs, dest_cs;
	unsigned int dest_width, dest_height;
	int ret;

	dev_dbg(priv->ipu->dev, "%s: starting ctx %p run %p\n",
		__func__, ctx, run);

	src_cs = ipu_pixelformat_to_colorspace(s_image->fmt->fourcc);
	dest_cs = ipu_pixelformat_to_colorspace(d_image->fmt->fourcc);

	if (ipu_rot_mode_is_irt(ctx->rot_mode)) {
		/* swap width/height for resizer */
		dest_width = d_image->tile[0].height;
		dest_height = d_image->tile[0].width;
	} else {
		dest_width = d_image->tile[0].width;
		dest_height = d_image->tile[0].height;
	}

	/* setup the IC resizer and CSC */
	ret = ipu_ic_task_init(cvt->ic,
			       s_image->tile[0].width,
			       s_image->tile[0].height,
			       dest_width,
			       dest_height,
			       src_cs, dest_cs);
	if (ret) {
		dev_err(priv->ipu->dev, "ipu_ic_task_init failed, %d\n", ret);
		return ret;
	}

	/* init the source MEM-->IC PP IDMAC channel */
	init_idmac_channel(ctx, cvt->in_chan, s_image,
			   IPU_ROTATE_NONE, false);

	if (ipu_rot_mode_is_irt(ctx->rot_mode)) {
		/* init the IC PP-->MEM IDMAC channel */
		init_idmac_channel(ctx, cvt->out_chan, d_image,
				   IPU_ROTATE_NONE, true);

		/* init the MEM-->IC PP ROT IDMAC channel */
		init_idmac_channel(ctx, cvt->rotation_in_chan, d_image,
				   ctx->rot_mode, true);

		/* init the destination IC PP ROT-->MEM IDMAC channel */
		init_idmac_channel(ctx, cvt->rotation_out_chan, d_image,
				   IPU_ROTATE_NONE, false);

		/* now link IC PP-->MEM to MEM-->IC PP ROT */
		ipu_idmac_link(cvt->out_chan, cvt->rotation_in_chan);
	} else {
		/* init the destination IC PP-->MEM IDMAC channel */
		init_idmac_channel(ctx, cvt->out_chan, d_image,
				   ctx->rot_mode, false);
	}

	/* enable the IC */
	ipu_ic_enable(cvt->ic);

	/* set buffers ready */
	ipu_idmac_select_buffer(cvt->in_chan, 0);
	ipu_idmac_select_buffer(cvt->out_chan, 0);
	if (ipu_rot_mode_is_irt(ctx->rot_mode))
		ipu_idmac_select_buffer(cvt->rotation_out_chan, 0);
	if (ctx->double_buffering) {
		ipu_idmac_select_buffer(cvt->in_chan, 1);
		ipu_idmac_select_buffer(cvt->out_chan, 1);
		if (ipu_rot_mode_is_irt(ctx->rot_mode))
			ipu_idmac_select_buffer(cvt->rotation_out_chan, 1);
	}

	/* enable the channels! */
	ipu_idmac_enable_channel(cvt->in_chan);
	ipu_idmac_enable_channel(cvt->out_chan);
	if (ipu_rot_mode_is_irt(ctx->rot_mode)) {
		ipu_idmac_enable_channel(cvt->rotation_in_chan);
		ipu_idmac_enable_channel(cvt->rotation_out_chan);
	}

	ipu_ic_task_enable(cvt->ic);

	ipu_cpmem_dump(cvt->in_chan);
	ipu_cpmem_dump(cvt->out_chan);
	if (ipu_rot_mode_is_irt(ctx->rot_mode)) {
		ipu_cpmem_dump(cvt->rotation_in_chan);
		ipu_cpmem_dump(cvt->rotation_out_chan);
	}

	ipu_dump(priv->ipu);

	return 0;
}

/* hold irqlock when calling */
static int ipu_ic_run(struct image_converter_run *run)
{
	struct image_converter_ctx *ctx = run->ctx;
	struct image_converter *cvt = ctx->cvt;

	ctx->in.base.phys0 = run->in_phys;
	ctx->out.base.phys0 = run->out_phys;

	ctx->cur_buf_num = 0;
	ctx->next_tile = 1;

	/* remove run from pending_q and set as current */
	list_del(&run->list);
	cvt->current_run = run;

	return ipu_ic_convert_start(run);
}

/* hold irqlock when calling */
static void ipu_ic_run_next(struct image_converter *cvt)
{
	struct ipu_ic_priv *priv = cvt->ic->priv;
	struct image_converter_run *run, *tmp;
	int ret;

	list_for_each_entry_safe(run, tmp, &cvt->pending_q, list) {
		/* skip contexts that are aborting */
		if (run->ctx->aborting) {
			dev_dbg(priv->ipu->dev,
				 "%s: skipping aborting ctx %p run %p\n",
				 __func__, run->ctx, run);
			continue;
		}

		ret = ipu_ic_run(run);
		if (!ret)
			break;

		/*
		 * something went wrong with start, add the run
		 * to done q and continue to the next run in the
		 * pending q.
		 */
		run->status = ret;
		list_add_tail(&run->list, &cvt->done_q);
		cvt->current_run = NULL;
	}
}

static void ipu_ic_empty_done_q(struct image_converter *cvt)
{
	struct ipu_ic_priv *priv = cvt->ic->priv;
	struct image_converter_run *run;
	unsigned long flags;

	spin_lock_irqsave(&cvt->irqlock, flags);

	while (!list_empty(&cvt->done_q)) {
		run = list_entry(cvt->done_q.next,
				 struct image_converter_run,
				 list);

		list_del(&run->list);

		dev_dbg(priv->ipu->dev,
			"%s: completing ctx %p run %p with %d\n",
			__func__, run->ctx, run, run->status);

		/* call the completion callback and free the run */
		spin_unlock_irqrestore(&cvt->irqlock, flags);
		run->ctx->complete(run->ctx->complete_context, run,
				   run->status);
		kfree(run);
		spin_lock_irqsave(&cvt->irqlock, flags);
	}

	spin_unlock_irqrestore(&cvt->irqlock, flags);
}

/*
 * the bottom half thread clears out the done_q, calling the
 * completion handler for each.
 */
static irqreturn_t ipu_ic_bh(int irq, void *dev_id)
{
	struct image_converter *cvt = dev_id;
	struct ipu_ic_priv *priv = cvt->ic->priv;
	struct image_converter_ctx *ctx;
	unsigned long flags;

	dev_dbg(priv->ipu->dev, "%s: enter\n", __func__);

	ipu_ic_empty_done_q(cvt);

	spin_lock_irqsave(&cvt->irqlock, flags);

	/*
	 * the done_q is cleared out, signal any contexts
	 * that are aborting that abort can complete.
	 */
	list_for_each_entry(ctx, &cvt->ctx_list, list) {
		if (ctx->aborting) {
			dev_dbg(priv->ipu->dev,
				 "%s: signaling abort for ctx %p\n",
				 __func__, ctx);
			complete(&ctx->aborted);
		}
	}

	spin_unlock_irqrestore(&cvt->irqlock, flags);

	dev_dbg(priv->ipu->dev, "%s: exit\n", __func__);
	return IRQ_HANDLED;
}

/* hold irqlock when calling */
static irqreturn_t ipu_ic_doirq(struct image_converter_run *run)
{
	struct image_converter_ctx *ctx = run->ctx;
	struct image_converter *cvt = ctx->cvt;
	struct ipu_ic_tile *src_tile, *dst_tile;
	struct ipu_ic_image *s_image = &ctx->in;
	struct ipu_ic_image *d_image = &ctx->out;
	struct ipuv3_channel *outch;
	unsigned int dst_idx;

	outch = ipu_rot_mode_is_irt(ctx->rot_mode) ?
		cvt->rotation_out_chan : cvt->out_chan;

	/*
	 * It is difficult to stop the channel DMA before the channels
	 * enter the paused state. Without double-buffering the channels
	 * are always in a paused state when the EOF irq occurs, so it
	 * is safe to stop the channels now. For double-buffering we
	 * just ignore the abort until the operation completes, when it
	 * is safe to shut down.
	 */
	if (ctx->aborting && !ctx->double_buffering) {
		ipu_ic_convert_stop(run);
		run->status = -EIO;
		goto done;
	}

	if (ctx->next_tile == ctx->num_tiles) {
		/*
		 * the conversion is complete
		 */
		ipu_ic_convert_stop(run);
		run->status = 0;
		goto done;
	}

	/*
	 * not done, place the next tile buffers.
	 */
	if (!ctx->double_buffering) {

		src_tile = &s_image->tile[ctx->next_tile];
		dst_idx = ctx->out_tile_map[ctx->next_tile];
		dst_tile = &d_image->tile[dst_idx];

		ipu_cpmem_set_buffer(cvt->in_chan, 0,
				     s_image->base.phys0 + src_tile->offset);
		ipu_cpmem_set_buffer(outch, 0,
				     d_image->base.phys0 + dst_tile->offset);
		if (s_image->fmt->y_depth)
			ipu_cpmem_set_uv_offset(cvt->in_chan,
						src_tile->u_off,
						src_tile->v_off);
		if (d_image->fmt->y_depth)
			ipu_cpmem_set_uv_offset(outch,
						dst_tile->u_off,
						dst_tile->v_off);

		ipu_idmac_select_buffer(cvt->in_chan, 0);
		ipu_idmac_select_buffer(outch, 0);

	} else if (ctx->next_tile < ctx->num_tiles - 1) {

		src_tile = &s_image->tile[ctx->next_tile + 1];
		dst_idx = ctx->out_tile_map[ctx->next_tile + 1];
		dst_tile = &d_image->tile[dst_idx];

		ipu_cpmem_set_buffer(cvt->in_chan, ctx->cur_buf_num,
				     s_image->base.phys0 + src_tile->offset);
		ipu_cpmem_set_buffer(outch, ctx->cur_buf_num,
				     d_image->base.phys0 + dst_tile->offset);

		ipu_idmac_select_buffer(cvt->in_chan, ctx->cur_buf_num);
		ipu_idmac_select_buffer(outch, ctx->cur_buf_num);

		ctx->cur_buf_num ^= 1;
	}

	ctx->next_tile++;
	return IRQ_HANDLED;
done:
	list_add_tail(&run->list, &cvt->done_q);
	cvt->current_run = NULL;
	ipu_ic_run_next(cvt);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t ipu_ic_norotate_irq(int irq, void *data)
{
	struct image_converter *cvt = data;
	struct image_converter_ctx *ctx;
	struct image_converter_run *run;
	unsigned long flags;
	irqreturn_t ret;

	spin_lock_irqsave(&cvt->irqlock, flags);

	/* get current run and its context */
	run = cvt->current_run;
	if (!run) {
		ret = IRQ_NONE;
		goto out;
	}

	ctx = run->ctx;

	if (ipu_rot_mode_is_irt(ctx->rot_mode)) {
		/* this is a rotation operation, just ignore */
		spin_unlock_irqrestore(&cvt->irqlock, flags);
		return IRQ_HANDLED;
	}

	ret = ipu_ic_doirq(run);
out:
	spin_unlock_irqrestore(&cvt->irqlock, flags);
	return ret;
}

static irqreturn_t ipu_ic_rotate_irq(int irq, void *data)
{
	struct image_converter *cvt = data;
	struct ipu_ic_priv *priv = cvt->ic->priv;
	struct image_converter_ctx *ctx;
	struct image_converter_run *run;
	unsigned long flags;
	irqreturn_t ret;

	spin_lock_irqsave(&cvt->irqlock, flags);

	/* get current run and its context */
	run = cvt->current_run;
	if (!run) {
		ret = IRQ_NONE;
		goto out;
	}

	ctx = run->ctx;

	if (!ipu_rot_mode_is_irt(ctx->rot_mode)) {
		/* this was NOT a rotation operation, shouldn't happen */
		dev_err(priv->ipu->dev, "Unexpected rotation interrupt\n");
		spin_unlock_irqrestore(&cvt->irqlock, flags);
		return IRQ_HANDLED;
	}

	ret = ipu_ic_doirq(run);
out:
	spin_unlock_irqrestore(&cvt->irqlock, flags);
	return ret;
}

/*
 * try to force the completion of runs for this ctx. Called when
 * abort wait times out in ipu_image_convert_abort().
 */
static void ipu_ic_force_abort(struct image_converter_ctx *ctx)
{
	struct image_converter *cvt = ctx->cvt;
	struct image_converter_run *run;
	unsigned long flags;

	spin_lock_irqsave(&cvt->irqlock, flags);

	run = cvt->current_run;
	if (run && run->ctx == ctx) {
		ipu_ic_convert_stop(run);
		run->status = -EIO;
		list_add_tail(&run->list, &cvt->done_q);
		cvt->current_run = NULL;
		ipu_ic_run_next(cvt);
	}

	spin_unlock_irqrestore(&cvt->irqlock, flags);

	ipu_ic_empty_done_q(cvt);
}

static void ipu_ic_release_ipu_resources(struct image_converter *cvt)
{
	if (cvt->out_eof_irq >= 0)
		free_irq(cvt->out_eof_irq, cvt);
	if (cvt->rot_out_eof_irq >= 0)
		free_irq(cvt->rot_out_eof_irq, cvt);

	if (!IS_ERR_OR_NULL(cvt->in_chan))
		ipu_idmac_put(cvt->in_chan);
	if (!IS_ERR_OR_NULL(cvt->out_chan))
		ipu_idmac_put(cvt->out_chan);
	if (!IS_ERR_OR_NULL(cvt->rotation_in_chan))
		ipu_idmac_put(cvt->rotation_in_chan);
	if (!IS_ERR_OR_NULL(cvt->rotation_out_chan))
		ipu_idmac_put(cvt->rotation_out_chan);

	cvt->in_chan = cvt->out_chan = cvt->rotation_in_chan =
		cvt->rotation_out_chan = NULL;
	cvt->out_eof_irq = cvt->rot_out_eof_irq = -1;
}

static int ipu_ic_get_ipu_resources(struct image_converter *cvt)
{
	const struct ic_task_channels *chan = cvt->ic->ch;
	struct ipu_ic_priv *priv = cvt->ic->priv;
	int ret;

	/* get IDMAC channels */
	cvt->in_chan = ipu_idmac_get(priv->ipu, chan->in);
	cvt->out_chan = ipu_idmac_get(priv->ipu, chan->out);
	if (IS_ERR(cvt->in_chan) || IS_ERR(cvt->out_chan)) {
		dev_err(priv->ipu->dev, "could not acquire idmac channels\n");
		ret = -EBUSY;
		goto err;
	}

	cvt->rotation_in_chan = ipu_idmac_get(priv->ipu, chan->rot_in);
	cvt->rotation_out_chan = ipu_idmac_get(priv->ipu, chan->rot_out);
	if (IS_ERR(cvt->rotation_in_chan) || IS_ERR(cvt->rotation_out_chan)) {
		dev_err(priv->ipu->dev,
			"could not acquire idmac rotation channels\n");
		ret = -EBUSY;
		goto err;
	}

	/* acquire the EOF interrupts */
	cvt->out_eof_irq = ipu_idmac_channel_irq(priv->ipu,
						cvt->out_chan,
						IPU_IRQ_EOF);

	ret = request_threaded_irq(cvt->out_eof_irq,
				   ipu_ic_norotate_irq, ipu_ic_bh,
				   0, "ipu-ic", cvt);
	if (ret < 0) {
		dev_err(priv->ipu->dev, "could not acquire irq %d\n",
			 cvt->out_eof_irq);
		cvt->out_eof_irq = -1;
		goto err;
	}

	cvt->rot_out_eof_irq = ipu_idmac_channel_irq(priv->ipu,
						     cvt->rotation_out_chan,
						     IPU_IRQ_EOF);

	ret = request_threaded_irq(cvt->rot_out_eof_irq,
				   ipu_ic_rotate_irq, ipu_ic_bh,
				   0, "ipu-ic", cvt);
	if (ret < 0) {
		dev_err(priv->ipu->dev, "could not acquire irq %d\n",
			cvt->rot_out_eof_irq);
		cvt->rot_out_eof_irq = -1;
		goto err;
	}

	return 0;
err:
	ipu_ic_release_ipu_resources(cvt);
	return ret;
}

static int ipu_ic_fill_image(struct image_converter_ctx *ctx,
			     struct ipu_ic_image *ic_image,
			     struct ipu_image *image,
			     enum image_convert_type type)
{
	struct ipu_ic_priv *priv = ctx->cvt->ic->priv;

	ic_image->base = *image;
	ic_image->type = type;

	ic_image->fmt = ipu_ic_get_format(image->pix.pixelformat);
	if (!ic_image->fmt) {
		dev_err(priv->ipu->dev, "pixelformat not supported for %s\n",
			type == IMAGE_CONVERT_OUT ? "Output" : "Input");
		return -EINVAL;
	}

	if (ic_image->fmt->y_depth)
		ic_image->stride = (ic_image->fmt->y_depth *
				    ic_image->base.pix.width) >> 3;
	else
		ic_image->stride  = ic_image->base.pix.bytesperline;

	ipu_ic_calc_tile_dimensions(ctx, ic_image);
	ipu_ic_calc_tile_offsets(ctx, ic_image);

	return 0;
}

/* borrowed from drivers/media/v4l2-core/v4l2-common.c */
static unsigned int clamp_align(unsigned int x, unsigned int min,
				unsigned int max, unsigned int align)
{
	/* Bits that must be zero to be aligned */
	unsigned int mask = ~((1 << align) - 1);

	/* Clamp to aligned min and max */
	x = clamp(x, (min + ~mask) & mask, max & mask);

	/* Round to nearest aligned value */
	if (align)
		x = (x + (1 << (align - 1))) & mask;

	return x;
}

/*
 * We have to adjust the tile width such that the tile physaddrs and
 * U and V plane offsets are multiples of 8 bytes as required by
 * the IPU DMA Controller. For the planar formats, this corresponds
 * to a pixel alignment of 16 (but use a more formal equation since
 * the variables are available). For all the packed formats, 8 is
 * good enough.
 */
static inline u32 tile_width_align(const struct ipu_ic_pixfmt *fmt)
{
	return fmt->y_depth ? (64 * fmt->uv_width_dec) / fmt->y_depth : 8;
}

/*
 * For tile height alignment, we have to ensure that the output tile
 * heights are multiples of 8 lines if the IRT is required by the
 * given rotation mode (the IRT performs rotations on 8x8 blocks
 * at a time). If the IRT is not used, or for input image tiles,
 * 2 lines are good enough.
 */
static inline u32 tile_height_align(enum image_convert_type type,
				    enum ipu_rotate_mode rot_mode)
{
	return (type == IMAGE_CONVERT_OUT &&
		ipu_rot_mode_is_irt(rot_mode)) ? 8 : 2;
}

/* Adjusts input/output images to IPU restrictions */
int ipu_image_convert_adjust(struct ipu_image *in, struct ipu_image *out,
			     enum ipu_rotate_mode rot_mode)
{
	const struct ipu_ic_pixfmt *infmt, *outfmt;
	unsigned int num_in_rows, num_in_cols;
	unsigned int num_out_rows, num_out_cols;
	u32 w_align, h_align;

	infmt = ipu_ic_get_format(in->pix.pixelformat);
	outfmt = ipu_ic_get_format(out->pix.pixelformat);

	/* set some defaults if needed */
	if (!infmt) {
		in->pix.pixelformat = V4L2_PIX_FMT_RGB24;
		infmt = ipu_ic_get_format(V4L2_PIX_FMT_RGB24);
	}
	if (!outfmt) {
		out->pix.pixelformat = V4L2_PIX_FMT_RGB24;
		outfmt = ipu_ic_get_format(V4L2_PIX_FMT_RGB24);
	}

	if (!in->pix.width || !in->pix.height) {
		in->pix.width = 640;
		in->pix.height = 480;
	}
	if (!out->pix.width || !out->pix.height) {
		out->pix.width = 640;
		out->pix.height = 480;
	}

	/* image converter does not handle fields */
	in->pix.field = out->pix.field = V4L2_FIELD_NONE;

	/* resizer cannot downsize more than 4:1 */
	if (ipu_rot_mode_is_irt(rot_mode)) {
		out->pix.height = max_t(__u32, out->pix.height,
					in->pix.width / 4);
		out->pix.width = max_t(__u32, out->pix.width,
				       in->pix.height / 4);
	} else {
		out->pix.width = max_t(__u32, out->pix.width,
				       in->pix.width / 4);
		out->pix.height = max_t(__u32, out->pix.height,
					in->pix.height / 4);
	}

	/* get tiling rows/cols from output format */
	num_out_rows = ipu_ic_num_stripes(out->pix.height);
	num_out_cols = ipu_ic_num_stripes(out->pix.width);
	if (ipu_rot_mode_is_irt(rot_mode)) {
		num_in_rows = num_out_cols;
		num_in_cols = num_out_rows;
	} else {
		num_in_rows = num_out_rows;
		num_in_cols = num_out_cols;
	}

	/* align input width/height */
	w_align = ilog2(tile_width_align(infmt) * num_in_cols);
	h_align = ilog2(tile_height_align(IMAGE_CONVERT_IN, rot_mode) *
			num_in_rows);
	in->pix.width = clamp_align(in->pix.width, MIN_W, MAX_W, w_align);
	in->pix.height = clamp_align(in->pix.height, MIN_H, MAX_H, h_align);

	/* align output width/height */
	w_align = ilog2(tile_width_align(outfmt) * num_out_cols);
	h_align = ilog2(tile_height_align(IMAGE_CONVERT_OUT, rot_mode) *
			num_out_rows);
	out->pix.width = clamp_align(out->pix.width, MIN_W, MAX_W, w_align);
	out->pix.height = clamp_align(out->pix.height, MIN_H, MAX_H, h_align);

	/* set input/output strides and image sizes */
	in->pix.bytesperline = (in->pix.width * infmt->bpp) >> 3;
	in->pix.sizeimage = in->pix.height * in->pix.bytesperline;
	out->pix.bytesperline = (out->pix.width * outfmt->bpp) >> 3;
	out->pix.sizeimage = out->pix.height * out->pix.bytesperline;

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_image_convert_adjust);

/*
 * this is used by ipu_image_convert_prepare() to verify set input and
 * output images are valid before starting the conversion. Clients can
 * also call it before calling ipu_image_convert_prepare().
 */
int ipu_image_convert_verify(struct ipu_image *in, struct ipu_image *out,
			     enum ipu_rotate_mode rot_mode)
{
	struct ipu_image testin, testout;
	int ret;

	testin = *in;
	testout = *out;

	ret = ipu_image_convert_adjust(&testin, &testout, rot_mode);
	if (ret)
		return ret;

	if (testin.pix.width != in->pix.width ||
	    testin.pix.height != in->pix.height ||
	    testout.pix.width != out->pix.width ||
	    testout.pix.height != out->pix.height)
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_image_convert_verify);

/*
 * Call ipu_image_convert_prepare() to prepare for the conversion of
 * given images and rotation mode. Returns a new conversion context.
 */
struct image_converter_ctx *
ipu_image_convert_prepare(struct ipu_ic *ic,
			  struct ipu_image *in, struct ipu_image *out,
			  enum ipu_rotate_mode rot_mode,
			  image_converter_cb_t complete,
			  void *complete_context)
{
	struct ipu_ic_priv *priv = ic->priv;
	struct image_converter *cvt = &ic->cvt;
	struct ipu_ic_image *s_image, *d_image;
	struct image_converter_ctx *ctx;
	unsigned long flags;
	bool get_res;
	int ret;

	if (!ic || !in || !out || !complete)
		return ERR_PTR(-EINVAL);

	/* verify the in/out images before continuing */
	ret = ipu_image_convert_verify(in, out, rot_mode);
	if (ret) {
		dev_err(priv->ipu->dev, "%s: in/out formats invalid\n",
			__func__);
		return ERR_PTR(ret);
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	dev_dbg(priv->ipu->dev, "%s: ctx %p\n", __func__, ctx);

	ctx->cvt = cvt;
	init_completion(&ctx->aborted);

	s_image = &ctx->in;
	d_image = &ctx->out;

	/* set tiling and rotation */
	d_image->num_rows = ipu_ic_num_stripes(out->pix.height);
	d_image->num_cols = ipu_ic_num_stripes(out->pix.width);
	if (ipu_rot_mode_is_irt(rot_mode)) {
		s_image->num_rows = d_image->num_cols;
		s_image->num_cols = d_image->num_rows;
	} else {
		s_image->num_rows = d_image->num_rows;
		s_image->num_cols = d_image->num_cols;
	}

	ctx->num_tiles = d_image->num_cols * d_image->num_rows;
	ctx->rot_mode = rot_mode;

	ret = ipu_ic_fill_image(ctx, s_image, in, IMAGE_CONVERT_IN);
	if (ret)
		goto out_free;
	ret = ipu_ic_fill_image(ctx, d_image, out, IMAGE_CONVERT_OUT);
	if (ret)
		goto out_free;

	ipu_ic_calc_out_tile_map(ctx);

	ipu_ic_dump_format(ctx, s_image);
	ipu_ic_dump_format(ctx, d_image);

	ctx->complete = complete;
	ctx->complete_context = complete_context;

	/*
	 * Can we use double-buffering for this operation? If there is
	 * only one tile (the whole image can be converted in a single
	 * operation) there's no point in using double-buffering. Also,
	 * the IPU's IDMAC channels allow only a single U and V plane
	 * offset shared between both buffers, but these offsets change
	 * for every tile, and therefore would have to be updated for
	 * each buffer which is not possible. So double-buffering is
	 * impossible when either the source or destination images are
	 * a planar format (YUV420, YUV422P, etc.).
	 */
	ctx->double_buffering = (ctx->num_tiles > 1 &&
				 !s_image->fmt->y_depth &&
				 !d_image->fmt->y_depth);

	if (ipu_rot_mode_is_irt(ctx->rot_mode)) {
		ret = ipu_ic_alloc_dma_buf(priv, &ctx->rot_intermediate[0],
					   d_image->tile[0].size);
		if (ret)
			goto out_free;
		if (ctx->double_buffering) {
			ret = ipu_ic_alloc_dma_buf(priv,
						   &ctx->rot_intermediate[1],
						   d_image->tile[0].size);
			if (ret)
				goto out_free_dmabuf0;
		}
	}

	spin_lock_irqsave(&cvt->irqlock, flags);

	get_res = list_empty(&cvt->ctx_list);

	list_add_tail(&ctx->list, &cvt->ctx_list);

	spin_unlock_irqrestore(&cvt->irqlock, flags);

	if (get_res) {
		ret = ipu_ic_get_ipu_resources(cvt);
		if (ret)
			goto out_free_dmabuf1;
	}

	return ctx;

out_free_dmabuf1:
	ipu_ic_free_dma_buf(priv, &ctx->rot_intermediate[1]);
	spin_lock_irqsave(&cvt->irqlock, flags);
	list_del(&ctx->list);
	spin_unlock_irqrestore(&cvt->irqlock, flags);
out_free_dmabuf0:
	ipu_ic_free_dma_buf(priv, &ctx->rot_intermediate[0]);
out_free:
	kfree(ctx);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(ipu_image_convert_prepare);

/*
 * Carry out a single image conversion. Only the physaddr's of the input
 * and output image buffers are needed. The conversion context must have
 * been created previously with ipu_image_convert_prepare(). Returns the
 * new run object.
 */
struct image_converter_run *
ipu_image_convert_run(struct image_converter_ctx *ctx,
		      dma_addr_t in_phys, dma_addr_t out_phys)
{
	struct image_converter *cvt = ctx->cvt;
	struct ipu_ic_priv *priv = cvt->ic->priv;
	struct image_converter_run *run;
	unsigned long flags;
	int ret = 0;

	run = kzalloc(sizeof(*run), GFP_KERNEL);
	if (!run)
		return ERR_PTR(-ENOMEM);

	run->ctx = ctx;
	run->in_phys = in_phys;
	run->out_phys = out_phys;

	dev_dbg(priv->ipu->dev, "%s: ctx %p run %p\n", __func__,
		ctx, run);

	spin_lock_irqsave(&cvt->irqlock, flags);

	if (ctx->aborting) {
		ret = -EIO;
		goto unlock;
	}

	list_add_tail(&run->list, &cvt->pending_q);

	if (!cvt->current_run) {
		ret = ipu_ic_run(run);
		if (ret)
			cvt->current_run = NULL;
	}
unlock:
	spin_unlock_irqrestore(&cvt->irqlock, flags);

	if (ret) {
		kfree(run);
		run = ERR_PTR(ret);
	}

	return run;
}
EXPORT_SYMBOL_GPL(ipu_image_convert_run);

/* Abort any active or pending conversions for this context */
void ipu_image_convert_abort(struct image_converter_ctx *ctx)
{
	struct image_converter *cvt = ctx->cvt;
	struct ipu_ic_priv *priv = cvt->ic->priv;
	struct image_converter_run *run, *active_run, *tmp;
	unsigned long flags;
	int run_count, ret;
	bool need_abort;

	reinit_completion(&ctx->aborted);

	spin_lock_irqsave(&cvt->irqlock, flags);

	/* move all remaining pending runs in this context to done_q */
	list_for_each_entry_safe(run, tmp, &cvt->pending_q, list) {
		if (run->ctx != ctx)
			continue;
		run->status = -EIO;
		list_move_tail(&run->list, &cvt->done_q);
	}

	run_count = ipu_ic_get_run_count(ctx, &cvt->done_q);
	active_run = (cvt->current_run && cvt->current_run->ctx == ctx) ?
		cvt->current_run : NULL;

	need_abort = (run_count || active_run);

	ctx->aborting = need_abort;

	spin_unlock_irqrestore(&cvt->irqlock, flags);

	if (!need_abort) {
		dev_dbg(priv->ipu->dev, "%s: no abort needed for ctx %p\n",
			__func__, ctx);
		return;
	}

	dev_dbg(priv->ipu->dev,
		 "%s: wait for completion: %d runs, active run %p\n",
		 __func__, run_count, active_run);

	ret = wait_for_completion_timeout(&ctx->aborted,
					  msecs_to_jiffies(10000));
	if (ret == 0) {
		dev_warn(priv->ipu->dev, "%s: timeout\n", __func__);
		ipu_ic_force_abort(ctx);
	}

	ctx->aborting = false;
}
EXPORT_SYMBOL_GPL(ipu_image_convert_abort);

/* Unprepare image conversion context */
void ipu_image_convert_unprepare(struct image_converter_ctx *ctx)
{
	struct image_converter *cvt = ctx->cvt;
	struct ipu_ic_priv *priv = cvt->ic->priv;
	unsigned long flags;
	bool put_res;

	/* make sure no runs are hanging around */
	ipu_image_convert_abort(ctx);

	dev_dbg(priv->ipu->dev, "%s: removing ctx %p\n", __func__, ctx);

	spin_lock_irqsave(&cvt->irqlock, flags);

	list_del(&ctx->list);

	put_res = list_empty(&cvt->ctx_list);

	spin_unlock_irqrestore(&cvt->irqlock, flags);

	if (put_res)
		ipu_ic_release_ipu_resources(cvt);

	ipu_ic_free_dma_buf(priv, &ctx->rot_intermediate[1]);
	ipu_ic_free_dma_buf(priv, &ctx->rot_intermediate[0]);

	kfree(ctx);
}
EXPORT_SYMBOL_GPL(ipu_image_convert_unprepare);

/*
 * "Canned" asynchronous single image conversion. On successful return
 * caller must call ipu_image_convert_unprepare() after conversion completes.
 * Returns the new conversion context.
 */
struct image_converter_ctx *
ipu_image_convert(struct ipu_ic *ic,
		  struct ipu_image *in, struct ipu_image *out,
		  enum ipu_rotate_mode rot_mode,
		  image_converter_cb_t complete,
		  void *complete_context)
{
	struct image_converter_ctx *ctx;
	struct image_converter_run *run;

	ctx = ipu_image_convert_prepare(ic, in, out, rot_mode,
					complete, complete_context);
	if (IS_ERR(ctx))
		return ctx;

	run = ipu_image_convert_run(ctx, in->phys0, out->phys0);
	if (IS_ERR(run)) {
		ipu_image_convert_unprepare(ctx);
		return ERR_PTR(PTR_ERR(run));
	}

	return ctx;
}
EXPORT_SYMBOL_GPL(ipu_image_convert);

/* "Canned" synchronous single image conversion */
static void image_convert_sync_complete(void *data,
					struct image_converter_run *run,
					int err)
{
	struct completion *comp = data;

	complete(comp);
}

int ipu_image_convert_sync(struct ipu_ic *ic,
			   struct ipu_image *in, struct ipu_image *out,
			   enum ipu_rotate_mode rot_mode)
{
	struct image_converter_ctx *ctx;
	struct completion comp;
	int ret;

	init_completion(&comp);

	ctx = ipu_image_convert(ic, in, out, rot_mode,
				image_convert_sync_complete, &comp);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = wait_for_completion_timeout(&comp, msecs_to_jiffies(10000));
	ret = (ret == 0) ? -ETIMEDOUT : 0;

	ipu_image_convert_unprepare(ctx);

	return ret;
}
EXPORT_SYMBOL_GPL(ipu_image_convert_sync);

int ipu_ic_enable(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	if (!priv->use_count)
		ipu_module_enable(priv->ipu, IPU_CONF_IC_EN);

	priv->use_count++;

	if (ic->rotation)
		ipu_irt_enable(ic);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_ic_enable);

int ipu_ic_disable(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);

	priv->use_count--;

	if (!priv->use_count)
		ipu_module_disable(priv->ipu, IPU_CONF_IC_EN);

	if (priv->use_count < 0)
		priv->use_count = 0;

	if (ic->rotation)
		ipu_irt_disable(ic);

	ic->rotation = ic->graphics = false;

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_ic_disable);

struct ipu_ic *ipu_ic_get(struct ipu_soc *ipu, enum ipu_ic_task task)
{
	struct ipu_ic_priv *priv = ipu->ic_priv;

	if (task >= IC_NUM_TASKS)
		return ERR_PTR(-EINVAL);

	return &priv->task[task];
}
EXPORT_SYMBOL_GPL(ipu_ic_get);

void ipu_ic_put(struct ipu_ic *ic)
{
}
EXPORT_SYMBOL_GPL(ipu_ic_put);

int ipu_ic_init(struct ipu_soc *ipu, struct device *dev,
		unsigned long base, unsigned long tpmem_base)
{
	struct ipu_ic_priv *priv;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ipu->ic_priv = priv;

	spin_lock_init(&priv->lock);

	priv->base = devm_ioremap(dev, base, PAGE_SIZE);
	if (!priv->base)
		return -ENOMEM;
	priv->tpmem_base = devm_ioremap(dev, tpmem_base, SZ_64K);
	if (!priv->tpmem_base)
		return -ENOMEM;

	dev_dbg(dev, "IC base: 0x%08lx remapped to %p\n", base, priv->base);

	priv->ipu = ipu;

	for (i = 0; i < IC_NUM_TASKS; i++) {
		struct ipu_ic *ic = &priv->task[i];
		struct image_converter *cvt = &ic->cvt;

		ic->task = i;
		ic->priv = priv;
		ic->reg = &ic_task_reg[i];
		ic->bit = &ic_task_bit[i];
		ic->ch = &ic_task_ch[i];

		cvt->ic = ic;
		spin_lock_init(&cvt->irqlock);
		INIT_LIST_HEAD(&cvt->ctx_list);
		INIT_LIST_HEAD(&cvt->pending_q);
		INIT_LIST_HEAD(&cvt->done_q);
		cvt->out_eof_irq = cvt->rot_out_eof_irq = -1;
	}

	return 0;
}

void ipu_ic_exit(struct ipu_soc *ipu)
{
}

void ipu_ic_dump(struct ipu_ic *ic)
{
	struct ipu_ic_priv *priv = ic->priv;
	struct ipu_soc *ipu = priv->ipu;

	dev_dbg(ipu->dev, "IC_CONF = \t0x%08X\n",
		ipu_ic_read(ic, IC_CONF));
	dev_dbg(ipu->dev, "IC_PRP_ENC_RSC = \t0x%08X\n",
		ipu_ic_read(ic, IC_PRP_ENC_RSC));
	dev_dbg(ipu->dev, "IC_PRP_VF_RSC = \t0x%08X\n",
		ipu_ic_read(ic, IC_PRP_VF_RSC));
	dev_dbg(ipu->dev, "IC_PP_RSC = \t0x%08X\n",
		ipu_ic_read(ic, IC_PP_RSC));
	dev_dbg(ipu->dev, "IC_CMBP_1 = \t0x%08X\n",
		ipu_ic_read(ic, IC_CMBP_1));
	dev_dbg(ipu->dev, "IC_CMBP_2 = \t0x%08X\n",
		ipu_ic_read(ic, IC_CMBP_2));
	dev_dbg(ipu->dev, "IC_IDMAC_1 = \t0x%08X\n",
		ipu_ic_read(ic, IC_IDMAC_1));
	dev_dbg(ipu->dev, "IC_IDMAC_2 = \t0x%08X\n",
		ipu_ic_read(ic, IC_IDMAC_2));
	dev_dbg(ipu->dev, "IC_IDMAC_3 = \t0x%08X\n",
		ipu_ic_read(ic, IC_IDMAC_3));
	dev_dbg(ipu->dev, "IC_IDMAC_4 = \t0x%08X\n",
		ipu_ic_read(ic, IC_IDMAC_4));
}
EXPORT_SYMBOL_GPL(ipu_ic_dump);
