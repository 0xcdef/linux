// SPDX-License-Identifier: GPL-2.0+
/*
 * FB driver for the ST7789 LCD Controller
 *
 * Copyright (C) 2013 Noralf Tronnes
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <video/mipi_display.h>

#include "fbtft.h"

#define DRVNAME "fb_st7789"
#define DEFAULT_GAMMA   "70 06 0C 08 09 27 2E 34 46 37 13 13 25 2A\n" \
			"70 04 08 09 07 03 2C 42 42 38 14 14 27 2C"

static const s16 default_init_sequence[] = {
	-1, MIPI_DCS_SOFT_RESET,
	-2, 150,                               /* delay */

	-1, MIPI_DCS_EXIT_SLEEP_MODE,
	-2, 500,                               /* delay */
	
	-1, MIPI_DCS_SET_ADDRESS_MODE, 0x0,
	-1, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_16BIT,

	-1, 0xB2, 0x05, 0x05, 0x00, 0x33, 0x33,

	-1, 0xB7, 0x23,
	-1, 0xBB, 0x22, 

	/* PWCTR1 - Power Control
	 * -4.6V, AUTO mode
	 */
	-1, 0xC0, 0x2C,

	/* PWCTR3 - Power Control
	 * Opamp current small, Boost frequency
	 */
	-1, 0xC2, 0x01,

	/* PWCTR4 - Power Control
	 * BCLK/2, Opamp current small & Medium low
	 */
	-1, 0xC3, 0x13,

	/* PWCTR5 - Power Control */
	-1, 0xC4, 0x20,

	/* VMCTR1 - Power Control */
	-1, 0xC6, 0x0F,

	-1, 0xD0, 0xA4, 0xA1,

	-1, MIPI_DCS_ENTER_INVERT_MODE,

	-1, MIPI_DCS_SET_DISPLAY_ON,
	-2, 100,                               /* delay */

	// Column/Row address
	//-1, 0x2A, 0x00, 0x34, 0x00, 0xBA,
	//-1, 0x2B, 0x00, 0x28, 0x01, 0x17,

	//-1, MIPI_DCS_ENTER_NORMAL_MODE,
	-2, 10,                               /* delay */

	/* end marker */
	-3
};

static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{

	switch(par->info->var.rotate)
	{
		case   0: xs+=40;xe+=40;ys+=53;ye+=53;
			   break;
		case  90: xs+=53;xe+=53;ys+=40;ye+=40;
			   break;
		case 180: xs+=40;xe+=40;ys+=53;ye+=53;
			   break;
		case 270: xs+=53;xe+=53;ys+=40;ye+=40;
			   break;
		default: break;
	}

	write_reg(par, MIPI_DCS_SET_COLUMN_ADDRESS,
		  xs >> 8, xs & 0xFF, xe >> 8, xe & 0xFF);

	write_reg(par, MIPI_DCS_SET_PAGE_ADDRESS,
		  ys >> 8, ys & 0xFF, ye >> 8, ye & 0xFF);

	write_reg(par, MIPI_DCS_WRITE_MEMORY_START);
}

#define MY BIT(7)
#define MX BIT(6)
#define MV BIT(5)
static int set_var(struct fbtft_par *par)
{
	/* MADCTL - Memory data access control
	 * RGB/BGR:
	 * 1. Mode selection pin SRGB
	 *    RGB H/W pin for color filter setting: 0=RGB, 1=BGR
	 * 2. MADCTL RGB bit
	 *    RGB-BGR ORDER color filter panel: 0=RGB, 1=BGR
	 */
	switch (par->info->var.rotate) {
	case 0:
		write_reg(par, MIPI_DCS_SET_ADDRESS_MODE,
			  MX | MY | (par->bgr << 3));
		break;
	case 270:
		write_reg(par, MIPI_DCS_SET_ADDRESS_MODE,
			  MY | MV | (par->bgr << 3));
		break;
	case 180:
		write_reg(par, MIPI_DCS_SET_ADDRESS_MODE,
			  par->bgr << 3);
		break;
	case 90:
		write_reg(par, MIPI_DCS_SET_ADDRESS_MODE,
			  MX | MV | (par->bgr << 3));
		break;
	}

	return 0;
}

/*
 * Gamma string format:
 * VRF0P VOS0P PK0P PK1P PK2P PK3P PK4P PK5P PK6P PK7P PK8P PK9P SELV0P SELV1P SELV62P SELV63P
 * VRF0N VOS0N PK0N PK1N PK2N PK3N PK4N PK5N PK6N PK7N PK8N PK9N SELV0N SELV1N SELV62N SELV63N
 */
#define CURVE(num, idx)  curves[(num) * par->gamma.num_values + (idx)]
static int set_gamma(struct fbtft_par *par, u32 *curves)
{
	int i, j;

	/* apply mask */
	for (i = 0; i < par->gamma.num_curves; i++)
		for (j = 0; j < par->gamma.num_values; j++)
			CURVE(i, j) &= 0x3f;

	for (i = 0; i < par->gamma.num_curves; i++)
		write_reg(par, 0xE0 + i,
			  CURVE(i, 0),  CURVE(i, 1),
			  CURVE(i, 2),  CURVE(i, 3),
			  CURVE(i, 4),  CURVE(i, 5),
			  CURVE(i, 6),  CURVE(i, 7),
			  CURVE(i, 8),  CURVE(i, 9),
			  CURVE(i, 10), CURVE(i, 11),
			  CURVE(i, 12), CURVE(i, 13),
			  CURVE(i, 14), CURVE(i, 15));

	return 0;
}

#undef CURVE

static struct fbtft_display display = {
	.regwidth = 8,
	.width = 240,
	.height = 135,
	.init_sequence = default_init_sequence,
	.gamma_num = 2,
	.gamma_len = 14,
	.gamma = DEFAULT_GAMMA,
	.fbtftops = {
		.set_addr_win = set_addr_win,
		.set_var = set_var,
		.set_gamma = set_gamma,
	},
};

FBTFT_REGISTER_DRIVER(DRVNAME, "sitronix,st7789", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:st7789");
MODULE_ALIAS("platform:st7789");

MODULE_DESCRIPTION("FB driver for the ST7789 LCD Controller");
MODULE_AUTHOR("Noralf Tronnes");
MODULE_LICENSE("GPL");
