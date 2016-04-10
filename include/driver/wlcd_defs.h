/*
 * Created by Martin Winkelhofer 03/2016
 * W-Dimension / wdim / wdim0 / maarty.w@gmail.com
 *  _      ____   ________       __    _
 * | | /| / / /  / ___/ _ \  ___/ /___(_)  _____ ____
 * | |/ |/ / /__/ /__/ // / / _  / __/ / |/ / -_) __/
 * |__/|__/____/\___/____/  \_,_/_/ /_/|___/\__/_/
 *
 * This file is part of WLCD - W-Dimension's LCD driver (for ESP8266).
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
#ifndef __WLCD_DEFS_H__
#define __WLCD_DEFS_H__

#include "spi_register.h" //must be included before esp_common.h to win (in esp_common.h, there's also #include for standard spi_register.h which is incomplete)
#include <espressif/esp_common.h>
#include "wlcd.h"

//WLCD - basic/global definitions

#ifdef WLCD_DO_DEBUG
#define DBG_WLCD(...)			printf( "wlcd: "__VA_ARGS__ )
#else
#define DBG_WLCD
#endif

typedef struct {
	const uint8_t CharWidth;
	const uint8_t CharHeight;
	const uint8_t FirstCharCode;
	const uint8_t LastCharCode;
	const uint16_t* Rects;
	const uint16_t* CharOffs;
} wlcd_font_descr_struct;

#define WLCD_IS_WRAP_CHAR(Ch)		(!( ((Ch)>127) || (((Ch)>='A')&&((Ch)<='Z')) || (((Ch)>='a')&&((Ch)<='z')) || (((Ch)>='0')&&((Ch)<='9')) || ((Ch)=='!') || ((Ch)=='?') || ((Ch)=='"') || ((Ch)=='\'') || ((Ch)=='_') || ((Ch)=='.') || ((Ch)==',') ))
#define WLCD_IS_NEWLINE_CHAR(Ch)	(((Ch)==13)||((Ch)==10))

//ILI9341 / ILI9488 common commands as defined in ILI9341_DS_V1.11.pdf / ILI9488_Preliminary_DS_V090.pdf from ILI TECHNOLOGY CORP
//regulative command set / standard command list:
#define WLCD_NO_OP							0x00
#define WLCD_SW_RESET						0x01
#define WLCD_READ_DISP_ID_INFO				0x04
#define WLCD_READ_DISP_STATUS				0x09
#define WLCD_READ_PWR_MODE					0x0A
#define WLCD_READ_MADCTL					0x0B
#define WLCD_READ_PIXEL_FORMAT				0x0C
#define WLCD_READ_IMAGE_FORMAT				0x0D
#define WLCD_READ_SIGNAL_MODE				0x0E
#define WLCD_READ_SELF_DIAG_RESULT			0x0F
#define WLCD_ENTER_SLEEP					0x10
#define WLCD_SLEEP_OUT						0x11
#define WLCD_PARTIAL_MODE_ON				0x12
#define WLCD_NORMAL_MODE_ON					0x13
#define WLCD_INVERSION_OFF					0x20
#define WLCD_INVERSION_ON					0x21
#define WLCD_DISP_OFF						0x28
#define WLCD_DISP_ON						0x29
#define WLCD_SET_COLUMN_ADDR_RANGE			0x2A
#define WLCD_SET_ROW_ADDR_RANGE				0x2B
#define WLCD_WRITE_DDRAM					0x2C
#define WLCD_READ_DDRAM						0x2E
#define WLCD_PARTIAL_AREA					0x30
#define WLCD_VERT_SCROLL_DEF				0x33
#define WLCD_TEAR_EFFECT_LN_OFF				0x34
#define WLCD_TEAR_EFFECT_LN_ON				0x35
#define WLCD_MEMORY_ACCESS_CTRL				0x36 //aka MADCTL
#define WLCD_VERT_SCROLL_START_ADDR			0x37
#define WLCD_IDLE_OFF						0x38
#define WLCD_IDLE_ON						0x39
#define WLCD_PIXEL_FORMAT_SET				0x3A
#define WLCD_WRITE_DDRAM_CONTINUE			0x3C
#define WLCD_READ_DDRAM_CONTINUE			0x3E
#define WLCD_SET_TEAR_SCANLINE				0x44
#define WLCD_GET_SCANLINE					0x45
#define WLCD_WRITE_DISP_BRIGHTNESS			0x51
#define WLCD_READ_DISP_BRIGHTNESS			0x52
#define WLCD_WRITE_CTRL_DISPLAY				0x53
#define WLCD_READ_CTRL_DISPLAY				0x54
#define WLCD_WRITE_CONTENT_ADAPT_BRIGH_CTRL	0x55
#define WLCD_READ_CONTENT_ADAPT_BRIGH_CTRL	0x56
#define WLCD_WRITE_CABC_MIN_BRIGHTNESS		0x5E
#define WLCD_READ_CABC_MIN_BRIGHTNESS		0x5F
#define WLCD_READ_ID1						0xDA
#define WLCD_READ_ID2						0xDB
#define WLCD_READ_ID3						0xDC
//extended command set:
#define WLCD_RGB_INTERFACE_SIG_CTRL			0xB0
#define WLCD_FRAME_CTRL_NORMAL_MODE			0xB1
#define WLCD_FRAME_CTRL_IDLE_MODE			0xB2
#define WLCD_FRAME_CTRL_PARTIAL_MODE		0xB3
#define WLCD_DISP_INVERSION_CTRL			0xB4
#define WLCD_BLANKING_PORCH_CTRL			0xB5
#define WLCD_DISPLAY_FUNCTION_CTRL			0xB6
#define WLCD_ENTRY_MODE_SET					0xB7
#define WLCD_POWER_CTRL_1					0xC0
#define WLCD_POWER_CTRL_2					0xC1
#define WLCD_VCOM_CTRL_1					0xC5
#define WLCD_NV_MEMORY_WRITE				0xD0
#define WLCD_NV_MEMORY_PROTECT_KEY			0xD1
#define WLCD_NV_MEMORY_STATUS_READ			0xD2
#define WLCD_READ_ID4						0xD3
#define WLCD_POSITIVE_GAMMA_CORR			0xE0
#define WLCD_NEGATIVE_GAMMA_CORR			0xE1
#define WLCD_DIGITAL_GAMMA_CTRL_1			0xE2
#define WLCD_DIGITAL_GAMMA_CTRL_2			0xE3
//
#if(WLCD_DISPLAY==WLCD_ILI9341)
	//commands specific to ILI9341 (based on mentioned pdf documents)
	#define WLCD_GAMMA_SET					0x26
	#define WLCD_COLOR_SET					0x2D
	//
	#define WLCD_BACKLIGHT_CTRL_1			0xB8
	#define WLCD_BACKLIGHT_CTRL_2			0xB9
	#define WLCD_BACKLIGHT_CTRL_3			0xBA
	#define WLCD_BACKLIGHT_CTRL_4			0xBB
	#define WLCD_BACKLIGHT_CTRL_5			0xBC
	#define WLCD_BACKLIGHT_CTRL_7			0xBE
	#define WLCD_BACKLIGHT_CTRL_8			0xBF
	#define WLCD_VCOM_CTRL_2				0xC7
	#define WLCD_INTERFACE_CTRL				0xF6
#elif(WLCD_DISPLAY==WLCD_ILI9488)
	//commands specific to ILI9488 (based on mentioned pdf documents)
	#define WLCD_READ_NUM_OF_DSI_ERRORS		0x05
	#define WLCD_ALL_PIXEL_OFF				0x22
	#define WLCD_ALL_PIXEL_ON				0x23
	#define WLCD_READ_AUTOMATIC_BRIGH		0x68
	//
	#define WLCD_COLOR_ENHANCE_CTRL_1		0xB9
	#define WLCD_COLOR_ENHANCE_CTRL_2		0xBA
	#define WLCD_POWER_CTRL_3				0xC2
	#define WLCD_POWER_CTRL_4				0xC3
	#define WLCD_POWER_CTRL_5				0xC4
	#define WLCD_CABC_CTRL_1				0xC6
	#define WLCD_CABC_CTRL_2				0xC8
	#define WLCD_CABC_CTRL_3				0xC9
	#define WLCD_CABC_CTRL_4				0xCA
	#define WLCD_CABC_CTRL_5				0xCB
	#define WLCD_CABC_CTRL_6				0xCC
	#define WLCD_CABC_CTRL_7				0xCD
	#define WLCD_CABC_CTRL_8				0xCE
	#define WLCD_CABC_CTRL_9				0xCF
	#define WLCD_ADJUST_CTRL_1				0xD7
	#define WLCD_SET_IMAGE_FUNCTION			0xE9
	#define WLCD_ADJUST_CTRL_2				0xF2
	#define WLCD_ADJUST_CTRL_3				0xF7
	#define WLCD_ADJUST_CTRL_4				0xF8
	#define WLCD_ADJUST_CTRL_5				0xF9
	#define WLCD_READ_SPI_COMMAND_SETTING	0xFB
	#define WLCD__ADJUST_CTRL_6				0xFC
#endif

//display rotation 0 deg.: set to 0,1,0,0,0; 90 deg. CW: set to 0,0,1,0,0; 180 deg. CW: set to 1,0,0,0,0; 270 deg. CW: set to 1,1,1,0,0
#if(WLCD_ORIENTATION==0)
	#define WLCD_ROW_ADDR_ORDER		0            //LCD controller memory read/write access direction - row (horizontal  axis) order
	#define WLCD_COL_ADDR_ORDER		1            // ... - column (vertical  axis) order
	#define WLCD_ROW_COL_EXCHANGE	0            // ... - exchange row and col.
	#define WLCD_VERT_REFRESH_ORD	0            // ... - vertical refresh direction control
	#define WLCD_HORIZ_REFRESH_ORD	0            // ... - horizontal refresh direction control
#elif(WLCD_ORIENTATION==90)
	#define WLCD_ROW_ADDR_ORDER		0
	#define WLCD_COL_ADDR_ORDER		0
	#define WLCD_ROW_COL_EXCHANGE	1
	#define WLCD_VERT_REFRESH_ORD	0
	#define WLCD_HORIZ_REFRESH_ORD	0
#elif(WLCD_ORIENTATION==180)
	#define WLCD_ROW_ADDR_ORDER		1
	#define WLCD_COL_ADDR_ORDER		0
	#define WLCD_ROW_COL_EXCHANGE	0
	#define WLCD_VERT_REFRESH_ORD	0
	#define WLCD_HORIZ_REFRESH_ORD	0
#elif(WLCD_ORIENTATION==270)
	#define WLCD_ROW_ADDR_ORDER		1
	#define WLCD_COL_ADDR_ORDER		1
	#define WLCD_ROW_COL_EXCHANGE	1
	#define WLCD_VERT_REFRESH_ORD	0
	#define WLCD_HORIZ_REFRESH_ORD	0
#else
	#define WLCD_ROW_ADDR_ORDER		0
	#define WLCD_COL_ADDR_ORDER		1
	#define WLCD_ROW_COL_EXCHANGE	0
	#define WLCD_VERT_REFRESH_ORD	0
	#define WLCD_HORIZ_REFRESH_ORD	0
#endif

#endif
