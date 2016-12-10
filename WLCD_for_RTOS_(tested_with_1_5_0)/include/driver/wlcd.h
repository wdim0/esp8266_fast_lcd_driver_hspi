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
 *
 * ----
 *
 * 4-wire SPI interface (5-wire with optional SDO line) => ILI9341's IM[3:0] = 1110 / ILI9488's IM[2:0] = 111
 *   ---------------------------        VDD   ----------------------------------
 *  |          ESP8266          |        |   |          LCD controller          |
 *  |          (master)         |        `->-| #RST         (slave)             |
 *  |                           |            |                                  |
 *  |       MTDO/GPIO15/HSPI_CS |------>-----| #CS (active L)                   |
 *  |     MTCK/GPIO13/HSPI_MOSI |------>-----| SDI/SDA                          |
 *  |     MTDI/GPIO12/HSPI_MISO |- - - <- - -| SDO (optional, see WLCD_NO_READ) |
 *  |      MTMS/GPIO14/HSPI_CLK |------>-----| >SCL (rising edge)               |
 *  |                     GPIO4 |------>-----| D/#C (data H / command L)        |
 *  |                           |            |                                  |
 *  |                           |            | (for other pins see PDF manual)  |
 *   ---------------------------              ----------------------------------
 *
 * SPI interface:
 * If WLCD_USE_HSPI is defined (fast, less CPU load):
 * - you can change only GPIOs for #CS and D/#C. The rest (HSPI controller) is hardwired inside ESP8266
 * - ! WLCD_SPI_CLK_PREDIV and WLCD_SPI_CLK_CNTDIV determines the speed of HSPI clock
 *   (configure properly / test to match your LCD controller specification)
 * If WLCD_USE_HSPI is not defined (SW bit-banging, slower, bigger CPU load):
 * - you can change all GPIOs (see WLCD MAIN CONFIG section below)
 * - the speed is determined by the speed of the CPU (see wlcd_write8_sw(...) / wlcd_read8_sw(...))
 *
 * Color depth:
 * Supported color depths are 16-bits / 18-bits per pixel (command 0x3A sets DBI[2:0] to 101 or 110).
 * Primary color depth is 16bpp - USE WLCD_16BPP FOR MAXIMUM SPEED (many optimizations take
 * advance of the fact, that we can move exactly two pixels by one 32-bit move).
 * Note: ILI9488 doesn't support 16bpp in SPI mode (DBI Type C mode), it supports
 * only 3bpp (DBI[2:0] = 001) and 18bpp (DBI[2:0] = 110). Ask ILItek why it was not implemented.
 * ILI9341 supports both 16bpp and 18bpp in SPI mode.
 *
 * WLCD images:
 * Use wlcd_img_gen tool to generate WLCD images from 24-bit / 16-bit BMPs. WLCD image can be
 * RLE compressed (uses 8-bit counter). The wlcd_img_gen tool by default tries both ways
 * and decides what's smaller. To produce smaller images, try to avoid smooth color gradients.
 * ! WLCD images can be 16bpp / 24bpp and this setting must correspond to used WLCD_*BPP !
 * - if you use WLCD_16BPP, generate WLCD image with parameter --16bpp
 * - if you use WLCD_18BPP, generate WLCD image with parameter --24bpp
 */
#ifndef __WLCD_H__
#define __WLCD_H__

#include <espressif/c_types.h>

//supported LCD controllers
#define WLCD_ILI9341			0  //ILI9341 LCD controller
#define WLCD_ILI9488_KDv4_HACK	1  //clone of ILI9488 LCD controller used in KeDei 3.5" LCD module v4.0 2016/1/1
#define WLCD_ILI9488			2  //ILI9488 LCD controller

//supported color depths
#define WLCD_16BPP				16 //16-bit per pixel (one pixel takes 2 bytes, R5G6B5) - FASTER (unfortunately, ILI9488 doesn't support 16bpp in SPI mode)
#define WLCD_18BPP				18 //18-bit per pixel (one pixel takes 3 bytes, R6G6B6 or R8G8B8 (last two bits are ignored by the LCD controller))

//==== main config of WLCD (edit only here)

#define WLCD_DISPLAY				WLCD_ILI9488 //choose one of supported LCD controllers

#define WLCD_BPP					WLCD_18BPP   //choose one of supported color depths - ! WLCD images must correspond

#define WLCD_ORIENTATION			90           //0 - default / 90 / 180 / 270 degrees of rotation (is managed only by setting the WLCD_MEMORY_ACCESS_CTRL aka MADCTL register => no SW rotation, no delay)
#define WLCD_PANEL_BGR_ORDER		1            //0 - default, LCD panel has RGB order; 1 - LCD panel has BGR order (is not related to data shifted via SPI - there's still RGB order)

#define	WLCD_USE_HSPI                            //use ESP8266's HSPI interface to communicate with LCD much faster. Comment the definition to use SW bit-banging - interface pins remain the same (see wlcd_init() for pin description)
#define WLCD_SPI_CLK_PREDIV			1            //HSPI CLK = CPU_CLK_FREQ (80 MHz by default) / (SPI_CLK_PREDIV*SPI_CLK_CNTDIV) => 80 / 2 = 40 MHz
#define WLCD_SPI_CLK_CNTDIV			2            // ... (20 MHz: PREDIV=2, CNTDIV=2; 40 MHz: PREDIV=1, CNTDIV=2 (! not 2, 1); 80 MHz: PREDIV=1, CNTDIV=1)

#define WLCD_GPIO_FOR_LCD_CS		15           //output GPIO for LCD #CS (active L)
#define WLCD_GPIO_FOR_LCD_D_C		4            //output GPIO for LCD D/#C (data H / command L)
//-- only for SW bit-banging (when WLCD_USE_HSPI is defined, these definitions has no effect)
#define WLCD_GPIO_FOR_LCD_SCL		14           //output GPIO for LCD >SCL (rising edge) - intentionally chosen to be GPIO14, the same as HSPI_CLK, so you can switch WLCD_USE_HSPI on/off and test both modes without physical re-wiring
#define WLCD_GPIO_FOR_LCD_SDO		12           //input GPIO for LCD SDO - intentionally chosen to be GPIO12, the same as HSPI_MISO, ...
#define WLCD_GPIO_FOR_LCD_SDI		13           //output GPIO for LCD SDI - intentionally chosen to be GPIO13, the same as HSPI_MOSI, ...
//--

//#define WLCD_NO_READ                           //uncomment this if the interface doesn't support reading operations - wlcd_img_get(...) will do nothing and return 0 (KeDei 3.5" LCD module uses shift registers => is unidirectional)

//#define WLCD_DO_DEBUG                          //uncomment this to output basic debug level msgs on TxD

//also don't forget to set/customize wlcd_Fonts[] array in wlcd_fonts.h

//==== (don't edit below this point)

#define WLCD_VERSION			"v1.50" //12/2016

typedef struct __packed {
	uint16_t B:5; //bit[4:0], <- to fill this, use WLCD_R8_TO_R5(...) macro (get R5 value from 8-bit R8 value)
	uint16_t G:6; //bit[10:5], ... WLCD_G8_TO_G6(...)
	uint16_t R:5; //bit[15:11], ... WLCD_B8_TO_B5(...)
} wlcd_R5G6B5_struct; //takes 2 bytes

typedef struct __packed {
	uint8_t B;
	uint8_t G;
	uint8_t R;
} wlcd_R8G8B8_struct; //takes 3 bytes

typedef enum __packed {
	WLCD_IMG_16BPP = 0,
	WLCD_IMG_24BPP
} wlcd_img_bpp_enum;

typedef struct __packed {
	uint16_t Width:14;     //bit[13:0] image width in pixels
	uint16_t BppCode:2;    //bit[15:14] <- wlcd_img_bpp_enum
	uint16_t Height:15;    //bit[14:0] image height in pixels
	uint16_t IsRLECmprs:1; //bit[15] RLE compression flag
} wlcd_img_info_struct; //takes 4 bytes

typedef enum __packed {
	WLCD_WRAP_NONE = 0, WLCD_WRAP_LETTERS, WLCD_WRAP_WORDS
} wlcd_wrap_style_enum;

typedef struct __packed {
	uint16_t X;          //where is the top left corner of the first char (where to start to draw the text on the display)
	uint16_t Y;          // ...
	union {              //color
		wlcd_R5G6B5_struct R5G6B5;
		wlcd_R8G8B8_struct R8G8B8;
		uint32_t u32t;   //<- to fill this, use WLCD_RGB_TO_COLOR(...) macro which is defined depending on WLCD_BPP mode (get color from 8-bit R, G, B values)
	} Color; //takes 4 bytes
	uint8_t FontIdx;     //used font - index to wlcd_Fonts[] array
	uint8_t FontZoomAdd; //font zoom: 0 = original (no zoom); 1 = 2x zoom; 2 = 3x zoom ...
	uint8_t BoldAdd;     //bold: 0 = original; 1 = make all pixels one pixel bigger horizontally; 2 = make all pixels two pixels bigger horizontally ...
	int8_t HSpc;         //horizontal char spacing addition: 0 = original; 1 = put next char 1px away to right; 2 = put next char 2px away to right ...
	int8_t VSpc;         //vertical ...
	uint16_t MaxW;       //width of text usable area - measured from X (wrapping is done in this area)
	uint16_t MaxH;       //height ...
	wlcd_wrap_style_enum WrapStyle;
} wlcd_text_draw_settings_struct;

#if(WLCD_DISPLAY==WLCD_ILI9341)
	#if((WLCD_ORIENTATION==0)||(WLCD_ORIENTATION==180))
		#define WLCD_WIDTH			240
		#define WLCD_HEIGHT			320
	#else
		#define WLCD_WIDTH			320
		#define WLCD_HEIGHT			240
	#endif
#elif((WLCD_DISPLAY==WLCD_ILI9488)||(WLCD_DISPLAY==WLCD_ILI9488_KDv4_HACK))
	#if((WLCD_ORIENTATION==0)||(WLCD_ORIENTATION==180))
		#define WLCD_WIDTH			320
		#define WLCD_HEIGHT			480
	#else
		#define WLCD_WIDTH			480
		#define WLCD_HEIGHT			320
	#endif
#endif

//---- provided functions

/* General meaning of Color parameter used in many functions:
 * Actual Color meaning is related to WLCD_BPP mode (see config section above).
 * It's uint32_t to make it universal for all color depths.
 * - if WLCD_BPP==WLCD_16BPP => Color is of wlcd_R5G6B5_struct structure / uint32_t filled by R5G6B5
 * - if WLCD_BPP==WLCD_18BPP => Color is of wlcd_R8G8B8_struct structure / uint32_t filled by R6G6B6 or R8G8B8
 *   (R6G6B6 or R8G8B8 are the same - last two bits are ignored by the LCD controller)
 * Use WLCD_RGB_TO_COLOR(...) macro to get uint32_t Color using R, G, B 8-bit values.
 * Use WLCD_COLOR(...) macro to convert wlcd_R5G6B5_struct / wlcd_R5G6B5_struct to uint32_t Color.
 */
#if(WLCD_BPP==WLCD_16BPP)
	#define WLCD_RGB_TO_COLOR(R, G, B)	(  ((((uint16_t)(R))&0xF8)<<8) | ((((uint16_t)(G))&0xFC)<<3) | (((uint16_t)(B))>>3) ) //R, G, B -> R5G6B5
#else
	#define WLCD_RGB_TO_COLOR(R, G, B)	(  (((uint32_t)(R))<<16) | (((uint16_t)(G))<<8) | (B) ) //R, G, B -> R8G8B8
#endif
//
#define WLCD_COLOR(AnyColorStruct)		(*((uint32_t*)&AnyColorStruct)) //wlcd_*_struct -> uint32_t Color
//
#define WLCD_R8_TO_R5(R8)				(R8>>3)
#define WLCD_G8_TO_G6(G8)				(G8>>2)
#define WLCD_B8_TO_B5(B8)				(B8>>3)

int8_t ICACHE_FLASH_ATTR wlcd_init(void);
//
char* ICACHE_FLASH_ATTR wlcd_get_version(void);
//
void ICACHE_FLASH_ATTR wlcd_set_drawing_rect(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height);
void ICACHE_FLASH_ATTR wlcd_write_dup_color(uint32_t Color, uint32_t PixelCount);
#define wlcd_rect_fill(X, Y, Width, Height, Color)	do { \
	wlcd_set_drawing_rect((X), (Y), (Width), (Height)); \
	wlcd_write_dup_color((Color), (uint32_t)(Width)*(Height)); \
	} while (0)
#define wlcd_pixel_draw(X, Y, Color)		wlcd_rect_fill((X), (Y), 1, 1, (Color))	//! for continuous stream of pixels use wlcd_write_dup_color(...) / wlcd_rect_fill(...) / wlcd_img_draw(...)
void ICACHE_FLASH_ATTR wlcd_line_draw(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint32_t Color);
//
void ICACHE_FLASH_ATTR wlcd_img_get_info(uint8_t* ImgData, wlcd_img_info_struct* RetImgInfo);
#define wlcd_img_get_width(ImgData)				(((wlcd_img_info_struct*)ImgData)->Width)
#define wlcd_img_get_height(ImgData)			(((wlcd_img_info_struct*)ImgData)->Height)
#define wlcd_img_get_bpp_code(ImgData)			(((wlcd_img_info_struct*)ImgData)->BppCode) //see wlcd_img_bpp_enum
#define wlcd_img_is_rle_compressed(ImgData)		(((wlcd_img_info_struct*)ImgData)->IsRLECmprs)
void ICACHE_FLASH_ATTR wlcd_img_draw(uint8_t* ImgData, uint16_t X, uint16_t Y); //! if one of these is true: 1) the image is not RLE compressed and we're using HSPI (WLCD_USE_HSPI is defined) OR 2) the image is RLE compressed and image color depth is 24bpp, then => ImgData must be 4-bytes aligned and allocated memory length must be multiples of 4. (see wlcd_write_buf_hspibuf64(...) for details why)
uint32_t ICACHE_FLASH_ATTR wlcd_img_get(uint32_t* Buf, uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height); //! if WLCD_USE_HSPI is defined, then Buf must be 4-bytes aligned and allocated memory length must be multiples of 4
//
//see wlcd_text_draw_settings_struct definition for more details
void ICACHE_FLASH_ATTR wlcd_text_draw_or_measure(char* Txt, wlcd_text_draw_settings_struct* S, uint8_t DontDrawJustMeasure, uint16_t* RetWidth, uint16_t* RetHeight);
#define wlcd_text_measure(Txt, S, RetWidth, RetHeight)	wlcd_text_draw_or_measure(Txt, S, 1, RetWidth, RetHeight)
#define wlcd_text_draw(Txt, S)							wlcd_text_draw_or_measure(Txt, S, 0, NULL, NULL)
uint16_t ICACHE_FLASH_ATTR wlcd_text_nchars_width(uint16_t NumOfChars, wlcd_text_draw_settings_struct* S);
uint16_t ICACHE_FLASH_ATTR wlcd_text_nrows_height(uint16_t NumOfRows, wlcd_text_draw_settings_struct* S);

//----

#endif
