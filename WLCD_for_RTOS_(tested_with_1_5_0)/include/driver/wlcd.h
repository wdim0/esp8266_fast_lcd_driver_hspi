/*
 * Created by Martin Winkelhofer 03,11/2016
 * W-Dimension / wdim / wdim0 / winkelhofer.m@gmail.com / https://github.com/wdim0
 *    __   ________          ______   _______  ___________    __  ______   _______  ____ ___  ___ 
 *   / /  / ___/ _ \  ____  /  _/ /  /  _/ _ \|_  / / <  /  _/_/ /  _/ /  /  _/ _ \/ / /( _ )( _ )
 *  / /__/ /__/ // / /___/ _/ // /___/ / \_, //_ <_  _/ / _/_/  _/ // /___/ / \_, /_  _/ _  / _  |
 * /____/\___/____/       /___/____/___//___/____//_//_/ /_/   /___/____/___//___/ /_/ \___/\___/ 
 *
 * Driver for LCD controllers ILI9341 / ILI9488 (or compatible).
 * v1.01 (11/2016)
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
 *   ---------------------------             ----------------------------
 *  |          ESP8266          |           |       LCD controller       |
 *  |          (master)         |           |           (slave)          |
 *  |                           |           |                            |
 *  |       MTDO/GPIO15/HSPI_CS |----->-----| CS (active L)              |
 *  |     MTCK/GPIO13/HSPI_MOSI |----->-----| MOSI                       |
 *  |     MTDI/GPIO12/HSPI_MISO |-----<-----| MISO (optional)            |
 *  |      MTMS/GPIO14/HSPI_CLK |----->-----| CLK (leading edge, act. H) |
 *  |                     GPIO4 |----->-----| D/C (data H / command L)   |
 *   ---------------------------             ----------------------------
 *
 *  If WLCD_USE_HSPI is defined, you can change only CS and D/C GPIOs. The rest (HSPI controller) is hardwired inside ESP8266.
 *  If WLCD_USE_HSPI is not defined, you can change all GPIOs (see WLCD MAIN CONFIG section below)
 *
 *  Mode is 16-bits per pixel ONLY, because we want to be as fast as possible (many optimizations
 *  take advance of the fact, that we can move exactly two pixels by one 32-bit move). That's why also
 *  WLCD image format (use wlcd_img_gen to generate from 24-bit of 16-bit BMPs) knows only 16 bpp images.
 *  WLCD image can be (and in most cases is) RLE compressed (8-bit counter).
 */
#ifndef __WLCD_H__
#define __WLCD_H__

#include <espressif/c_types.h>

//supported LCD controllers
#define WLCD_ILI9341		0
#define WLCD_ILI9488		1

//==== main config of WLCD (edit only here)

#define WLCD_DISPLAY				WLCD_ILI9488 //choose one of supported LCD controllers

#define WLCD_ORIENTATION			90           //0 - default / 90 / 180 / 270 degrees of rotation (is managed only by setting the WLCD_MEMORY_ACCESS_CTRL aka MADCTL register => no SW rotation, no delay)
#define WLCD_PANEL_BGR_ORDER		1            //0 - default, LCD panel has RGB order; 1 - LCD panel has BGR order (is not related to data shifted via SPI - there's still RGB order)

#define	WLCD_USE_HSPI                            //use ESP8266's HSPI interface to communicate with LCD much faster. Comment the definition to use SW bit-banging - interface pins remain the same (see wlcd_init() for pin description)
#define WLCD_SPI_CLK_PREDIV			1            //HSPI CLK = CPU_CLK_FREQ (80 MHz by default) / (SPI_CLK_PREDIV*SPI_CLK_CNTDIV) => 80 / 2 = 40 MHz
#define WLCD_SPI_CLK_CNTDIV			2            // ... (20 MHz: PREDIV=2, CNTDIV=2; 40 MHz: PREDIV=1, CNTDIV=2 (! not 2, 1); 80 MHz: PREDIV=1, CNTDIV=1)

#define WLCD_GPIO_FOR_LCD_CS		15           //output GPIO for LCD CS (active L)
#define WLCD_GPIO_FOR_LCD_D_C		4            //output GPIO for LCD D/C (data H / command L)
//-- only for SW bit-banging (when WLCD_USE_HSPI is defined, these definitions has no effect)
#define WLCD_GPIO_FOR_LCD_CLK		14           //output GPIO for LCD CLK (leading. edge, act. H) - intentionally chosen to be GPIO14, the same as HSPI_CLK, so you can switch WLCD_USE_HSPI on/off and test both modes
#define WLCD_GPIO_FOR_LCD_MISO		12           //input GPIO for LCD MISO - intentionally chosen to be GPIO12, the same as HSPI_MISO, ...
#define WLCD_GPIO_FOR_LCD_MOSI		13           //output GPIO for LCD MOSI - intentionally chosen to be GPIO13, the same as HSPI_MOSI, ...
//--

#define WLCD_NO_READ                             //uncomment this if your display module doesn't support reading operations - wlcd_img_get(...) will do nothing and return 0 (KeDei 3.5" module)

//#define WLCD_DO_DEBUG                            //uncomment this to output basic debug level msgs on TxD

//also don't forget to set/customize wlcd_Fonts[] array in wlcd_fonts.h

//====

#define WLCD_VERSION			"v1.01" //11/2016

typedef enum {
	WLCD_WRAP_NONE = 0, WLCD_WRAP_LETTERS, WLCD_WRAP_WORDS
} wlcd_wrap_style_enum;

typedef struct {
	uint16_t X;          //where is the top left corner of the first char (where to start to draw the text on the display)
	uint16_t Y;          // ...
	uint16_t R5G6B5;     //color
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
#elif(WLCD_DISPLAY==WLCD_ILI9488)
	#if((WLCD_ORIENTATION==0)||(WLCD_ORIENTATION==180))
		#define WLCD_WIDTH			320
		#define WLCD_HEIGHT			480
	#else
		#define WLCD_WIDTH			480
		#define WLCD_HEIGHT			320
	#endif
#endif

//---- provided functions

char* ICACHE_FLASH_ATTR wlcd_get_version(void);
//
int8_t ICACHE_FLASH_ATTR wlcd_init(void);
//
void /*ICACHE_FLASH_ATTR*/ wlcd_set_drawing_rect(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height);
void /*ICACHE_FLASH_ATTR*/ wlcd_write_dup_16bpp(uint16_t R5G6B5, uint32_t Dup16Count);
void /*ICACHE_FLASH_ATTR*/ wlcd_rect_fill_16bpp(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint16_t R5G6B5);
#define wlcd_pixel_draw(X, Y, R5G6B5)		wlcd_rect_fill_16bpp(X, Y, 1, 1, R5G6B5) //! for continuous stream of pixels use wlcd_write_dup_16bpp(...) or wlcd_img_draw(...)
void /*ICACHE_FLASH_ATTR*/ wlcd_line_draw(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint16_t R5G6B5);
//
void ICACHE_FLASH_ATTR wlcd_img_get_info(uint8_t* ImgData, uint16_t* RetImgW, uint16_t* RetImgH, uint8_t* IsRLECompressed);
#define wlcd_img_get_width(ImgData)				(*(uint16_t*)(ImgData))
#define wlcd_img_get_height(ImgData)			(*(((uint16_t*)(ImgData))+1) & 0x7FFF)
#define wlcd_img_is_rle_compressed(ImgData)		(((uint8_t*)ImgData)[3] & 0x80)
void ICACHE_FLASH_ATTR wlcd_img_draw(uint8_t* ImgData, uint16_t X, uint16_t Y); //! if the image is not RLE compressed and we're using HSPI (WLCD_USE_HSPI is defined), then ImgData must be 4-bytes aligned and allocated memory length must be multiples of 4
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
