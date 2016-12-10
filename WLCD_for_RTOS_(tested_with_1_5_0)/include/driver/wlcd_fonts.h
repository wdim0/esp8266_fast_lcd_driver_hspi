/*
 * Created by Martin Winkelhofer 03,11,12/2016
 * W-Dimension / wdim / wdim0 / winkelhofer.m@gmail.com / https://github.com/wdim0
 *    __   ________          ______   _______  ___________    __  ______   _______  ____ ___  ___ 
 *   / /  / ___/ _ \  ____  /  _/ /  /  _/ _ \|_  / / <  /  _/_/ /  _/ /  /  _/ _ \/ / /( _ )( _ )
 *  / /__/ /__/ // / /___/ _/ // /___/ / \_, //_ <_  _/ / _/_/  _/ // /___/ / \_, /_  _/ _  / _  |
 * /____/\___/____/       /___/____/___//___/____//_//_/ /_/   /___/____/___//___/ /_/ \___/\___/ 
 *
 * Driver for LCD controllers ILI9341 / ILI9488 (or compatible) using 4-wire SPI interface
 * hooked to ESP8266's HSPI interface (or using SW bit-banging for any GPIOs).
 * v1.50 (12/2016), RTOS version
 *
 * This file is part of WLCD - W-Dimension's LCD driver for ESP8266.
 *
 * WLCD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * WLCD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WLCD. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __WLCD_FONTS_H__
#define __WLCD_FONTS_H__

//==== font config of WLCD (edit only here)

//WLCD - definition of wlcd_Fonts[] array (used in wlcd.c by function wlcd_text_draw())
//To save memory, include only fonts you need

#include "wlcd_font_term_8x8_0_127.h"
//#include "wlcd_font_term_8x8_cp437.h"

#include "wlcd_font_termb_8x8_0_127.h"
//#include "wlcd_font_termb_8x8_cp437.h"

//#include "wlcd_font_term_8x16_0_127.h"
//#include "wlcd_font_term_8x16_cp437.h"

//#include "wlcd_font_fxs_8x15_16_127.h"
#include "wlcd_font_fxs_8x15_cp1250.h"

//#include "wlcd_font_cr_8x13_32_127.h"
//#include "wlcd_font_cr_8x13_cp1250.h"

#include "wlcd_font_cr_16x16_32_127.h"

#define WLCD_FONTS_CNT			4 //change to reflect count of fonts in wlcd_Fonts[]
const wlcd_font_descr_struct wlcd_Fonts[WLCD_FONTS_CNT] = {
	{ WLCD_FONT_TERM_8X8_0_127_CHAR_WIDTH, WLCD_FONT_TERM_8X8_0_127_CHAR_HEIGHT,
	  WLCD_FONT_TERM_8X8_0_127_FIRST_CHAR_CODE, WLCD_FONT_TERM_8X8_0_127_LAST_CHAR_CODE,
	  wlcd_font_term_8x8_0_127_Rects, wlcd_font_term_8x8_0_127_CharOffs
	},
	{ WLCD_FONT_TERMB_8X8_0_127_CHAR_WIDTH, WLCD_FONT_TERMB_8X8_0_127_CHAR_HEIGHT,
	  WLCD_FONT_TERMB_8X8_0_127_FIRST_CHAR_CODE, WLCD_FONT_TERMB_8X8_0_127_LAST_CHAR_CODE,
	  wlcd_font_termb_8x8_0_127_Rects, wlcd_font_termb_8x8_0_127_CharOffs
	},
	{ WLCD_FONT_FXS_8X15_CP1250_CHAR_WIDTH, WLCD_FONT_FXS_8X15_CP1250_CHAR_HEIGHT,
	  WLCD_FONT_FXS_8X15_CP1250_FIRST_CHAR_CODE, WLCD_FONT_FXS_8X15_CP1250_LAST_CHAR_CODE,
	  wlcd_font_fxs_8x15_cp1250_Rects, wlcd_font_fxs_8x15_cp1250_CharOffs
	},
	{ WLCD_FONT_CR_16X16_32_127_CHAR_WIDTH, WLCD_FONT_CR_16X16_32_127_CHAR_HEIGHT,
	  WLCD_FONT_CR_16X16_32_127_FIRST_CHAR_CODE, WLCD_FONT_CR_16X16_32_127_LAST_CHAR_CODE,
	  wlcd_font_cr_16x16_32_127_Rects, wlcd_font_cr_16x16_32_127_CharOffs
	}
};

//====

#endif
