/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Fullhan FH8626V100 Image Signal Processor registers.
 *
 * Copyright (C) 2026 Difan Zhang
 */

#ifndef _FH8626_ISP_REGS_H
#define _FH8626_ISP_REGS_H

#include <linux/bits.h>

#define ISP_CTRL			0x18
#define ISP_BAYER_FMT			0x24
#define ISP_PIPE_CTRL			0x28
/* ISP_PIPE_*_BYPASS bits disable their respective processing stages. */
#define ISP_PIPE_HLR_BYPASS		BIT(2)
#define ISP_PIPE_RGB_GAMMA_BYPASS	BIT(4)
#define ISP_PIPE_YNR_BYPASS		BIT(5)
#define ISP_PIPE_CNR_BYPASS		BIT(6)
#define ISP_PIPE_APC_BYPASS		BIT(7)
#define ISP_PIPE_PFC_BYPASS		BIT(8)
#define ISP_PIPE_Y_GAMMA_BYPASS		BIT(10)
/*
 * Set: enable writing the block luma/chroma mean statistics to memory.
 */
#define ISP_PIPE_YCMEAN_EN		BIT(13)
#define ISP_GAMMA_BANK(n)		(0x1000 + (n) * 0x180)
#define ISP_GAMMA_Y_BANK		3
#define ISP_GAMMA_WORDS			80
#define ISP_CCM_BASE			0x4c0
#define ISP_CCM_WORDS			6
#define ISP_CSC_CTRL			0x4f0
/* Luma tone curve input selection; cleared when both curve stages run. */
#define ISP_CSC_Y_CURVE_SEL		BIT(4)
#define ISP_BNR_CHANNEL(n)		(0x454 + 4 * (n))
#define ISP_INT_STATUS			0x04
#define ISP_INT_ENABLE			0x08
#define ISP_VPU_ENABLE			0x2c
#define ISP_INPUT_CTRL			0x68
#define ISP_INT_FRAME_END		BIT(0)
#define ISP_INT_VPU_END			BIT(2)
#define ISP_INT_OVERFLOW		BIT(3)
/* Only increments the pipe conflict counter. */
#define ISP_INT_PIPE_CONFLICT		BIT(4)
#define ISP_INT_PICTURE_START		BIT(13)
#define ISP_INT_VPU_START		BIT(15)
#define ISP_BLC_01			0x84
#define ISP_BLC_23			0x88
#define ISP_GAIN_0			0x168
#define ISP_GAIN_1			0x16c
#define ISP_WB_GAIN_01			0x224
#define ISP_WB_GAIN_23			0x228
/* Set: the zone statistics are written to the ISP_STAT_ADDR targets. */
#define ISP_BAYER_STATS_EN		BIT(13)
/* Zone statistics DMA targets of the two statistics banks */
#define ISP_STAT_ADDR0			0x1d8
#define ISP_STAT_ADDR1			0x1e8
/* Set: the block motion detector updates ISP_MOTION_MAP. */
#define ISP_BAYER_MOTION_STAT		BIT(16)
#define ISP_BAYER_NR3D_BYPASS		BIT(23)
/* Block motion bitmap, one bit per block, raster order. */
#define ISP_MOTION_MAP			0x1f4
#define ISP_NR3D_CTRL			0x44c
#define ISP_NR3D_FIRST_FRAME		BIT(14)
#define ISP_NR3D_NOISE_TH		0x450
#define ISP_NR3D_TAP_CFG		0x468
#define ISP_NR3D_COEFF_CFG		0x46c
#define ISP_NR3D_COEFF_STRIDE		0x488
#define ISP_NR3D_COEFF_ADDR		0x48c
#define ISP_NR3D_BUF_STRIDE		0x490
#define ISP_NR3D_BUF_ADDR		0x494
/*
 * Highlight chroma suppression: [15:0] luma threshold. The reset value is
 * zero, which treats every pixel as a highlight and removes all chroma.
 */
#define ISP_HLR_CFG			0x4bc
#define ISP_HLR_THRESHOLD_MASK		GENMASK(15, 0)
/* Cb/Cr output offsets, 12-bit; 0x400 is the 8-bit neutral 128. */
#define ISP_CHROMA_OFFSET		0x4ec
#define ISP_CHROMA_NEUTRAL		0x04000400
#define ISP_CSC_CTRL_FMT_MASK		GENMASK(1, 0)

/*
 * Luma noise filter. Three 16-bit thresholds select how strongly a
 * neighbour may differ from the centre pixel, one per filter band. A
 * piecewise-linear curve over the 8-bit luma (two knees) scales the
 * thresholds with brightness, and three 16-entry weight tables, one per
 * band, give the filter weight against the normalised difference.
 */
#define ISP_YNR_MODE			0x4f4
#define ISP_YNR_MODE_MASK		GENMASK(2, 0)
#define ISP_YNR_TH0			0x4f8
#define ISP_YNR_TH0_MASK		GENMASK(31, 16)
#define ISP_YNR_TH12			0x4fc
/* [13:4] slope from black to the first knee, Q8 */
#define ISP_YNR_CURVE0			0x514
#define ISP_YNR_SLOPE0_MASK		GENMASK(13, 4)
/* [31:24] first knee luma, [23:12] level there, [9:0] slope, Q8 */
#define ISP_YNR_CURVE1			0x518
/* [31:24] second knee luma, [23:12] level there and above */
#define ISP_YNR_CURVE2			0x51c
#define ISP_YNR_KNEE_MASK		GENMASK(31, 24)
#define ISP_YNR_LEVEL_MASK		GENMASK(23, 12)
#define ISP_YNR_SLOPE1_MASK		GENMASK(9, 0)
#define ISP_YNR_LUT(band)		(0xa0c + (band) * 0x20)
#define ISP_YNR_LUT_POINTS		16
#define ISP_YNR_BANDS			3

/*
 * Chroma noise filter: [25:18] and [9:2] of ISP_CNR_TH are the upper and
 * lower difference thresholds, [11:4] of ISP_CNR_CFG the filter strength.
 * ISP_CNR_CFG[1:0] must follow ISP_CSC_CTRL[1:0].
 */
#define ISP_CNR_TH			0x520
#define ISP_CNR_TH_HI_MASK		GENMASK(25, 18)
#define ISP_CNR_TH_LO_MASK		GENMASK(9, 2)
#define ISP_CNR_CFG			0x524
#define ISP_CNR_STRENGTH_MASK		GENMASK(11, 4)
#define ISP_CNR_FMT_MASK		GENMASK(1, 0)

/*
 * Edge enhancement with a detail (fine texture) path and an edge path.
 * ISP_APC_GAIN holds one gain per path; [31:24] and [23:16] of each path
 * configuration word limit the positive and negative enhancement.
 */
#define ISP_APC_GAIN			0x528
#define ISP_APC_DETAIL_GAIN_MASK	GENMASK(7, 0)
#define ISP_APC_EDGE_GAIN_MASK		GENMASK(15, 8)
#define ISP_APC_DETAIL_CFG		0x544
#define ISP_APC_EDGE_CFG		0x564
#define ISP_APC_STRENGTH_MASK		GENMASK(31, 16)

/*
 * Purple fringe correction. Chroma within an angular sector of the Cb/Cr
 * plane is desaturated near strong luma transitions. ISP_PFC_HUE holds the
 * sector centre as signed Q7 cosine [15:8] and sine [7:0]; ISP_PFC_RANGE0/1
 * hold tan(range / k) in Q8 for k = 1..6, three 10-bit fields per word.
 * ISP_PFC_TH0..2 are the luma, exposure and saturation gating thresholds.
 */
#define ISP_PFC_CTRL			0x59c
#define ISP_PFC_ENABLE			BIT(0)
#define ISP_PFC_SHIFT_MASK		GENMASK(7, 4)
#define ISP_PFC_HUE			0x5a0
#define ISP_PFC_RANGE0			0x5a4
#define ISP_PFC_RANGE1			0x5a8
#define ISP_PFC_TH(n)			(0x5ac + 4 * (n))
/* [9:0] contrast, Q6; [23:16] signed brightness offset. */
#define ISP_LUMA_ADJ			0x5c8
#define ISP_CONTRAST_MASK		GENMASK(9, 0)
#define ISP_BRIGHTNESS_MASK		GENMASK(23, 16)
/* [9:2] saturation, Q6. */
#define ISP_SATURATION			0x5d0
#define ISP_SATURATION_MASK		GENMASK(9, 2)

#endif /* _FH8626_ISP_REGS_H */
