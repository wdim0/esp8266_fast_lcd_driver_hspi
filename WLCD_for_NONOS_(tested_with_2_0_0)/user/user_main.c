/*
 * Created by Martin Winkelhofer 04,11/2016
 * W-Dimension / wdim / wdim0 / winkelhofer.m@gmail.com / https://github.com/wdim0
 */

#include <espmissingincludes.h> //most common / basic includes + missing includes (Espressif intentionally hides some of provided functions)
#include <user_interface.h>

#include <wlcd.h>
#include <wfof.h>
#include <wfof_idxs.h>
#include <wcommon.h>

#include "user_config.h"

//---- rf_cal sector (obligatory for all projects) ----------------------------

/* FunctionName : user_rf_cal_sector_set
 * Description  : SDK_V2.0.0+ reserved 4 sectors more - used for rf cal data and at paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                System param area - sector map prior SDK_V2.0.0+:     CDDD
 *                System param area - sector map for SDK_V2.0.0+  :    ACDDD
 *                                      or also with at parameters: ABBBCDDD
 *                A: (new) rf cal
 *                B: (new) at parameters
 *                C: rf init data (<-- ...SDK\bin\esp_init_data_default.bin)
 *                D: sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
 */
uint32_t ICACHE_FLASH_ATTR user_rf_cal_sector_set(void) {
	enum flash_size_map size_map = system_get_flash_size_map();
	uint32_t rf_cal_sec = 0;
	switch (size_map) {
		case FLASH_SIZE_4M_MAP_256_256:
			rf_cal_sec = 128 - 5;
			break;
		case FLASH_SIZE_8M_MAP_512_512:
			rf_cal_sec = 256 - 5;
			break;
		case FLASH_SIZE_16M_MAP_512_512:
		case FLASH_SIZE_16M_MAP_1024_1024:
			rf_cal_sec = 512 - 5;
			break;
		case FLASH_SIZE_32M_MAP_512_512:
		case FLASH_SIZE_32M_MAP_1024_1024:
			rf_cal_sec = 1024 - 5;
			break;
		default:
			rf_cal_sec = 0;
			break;
	}
	return rf_cal_sec;
}

//-----------------------------------------------------------------------------

void ICACHE_FLASH_ATTR user_init(void){
	DBG("user_init() started\n");
	//
	//prepare test vars and buffers
	wlcd_text_draw_settings_struct S;
	//
	char* Str = os_malloc(256);
	uint8_t* ImgData = os_malloc(11000); //arrays allocated by os_malloc() are also always 4-bytes aligned (at least on ESP8266 SDK). Make sure that the buffer is big enough
	if((Str==NULL)||(ImgData==NULL)) return; //os_malloc() failed => abort
	uint16_t W, H;
	uint32_t t, t2;
	uint16_t i;
	uint16_t DoneXOffs;
	uint16_t DoneYOffs;
	//
	//init display
	wlcd_init();
	//
#if(WLCD_BPP==WLCD_16BPP)
	wfof_get_file_data_fast(WFOF_IDX_WLCD_DEMO_16BPP, (uint32_t*)ImgData, 0, WFOF_SIZE_WLCD_DEMO_16BPP);
#else
	wfof_get_file_data_fast(WFOF_IDX_WLCD_DEMO_24BPP, (uint32_t*)ImgData, 0, WFOF_SIZE_WLCD_DEMO_24BPP);
#endif
	//
	while(1){
		//
		//---- WLCD test / demo intro (countdown)
		//
		//clear whole display (black)
		wlcd_rect_fill(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0);
		//
		//demo image
		wlcd_img_draw(ImgData, (WLCD_WIDTH-wlcd_img_get_width(ImgData))/2, ((WLCD_HEIGHT-wlcd_img_get_height(ImgData))/2)-10);
		//
		//test countdown
		S.X = 10;
		S.Y = 10;
		S.Color.u32t = WLCD_RGB_TO_COLOR(0xFF, 0x5C, 0x20);
		S.FontIdx = 2;
		S.FontZoomAdd = 0;
		S.BoldAdd = 0;
		S.HSpc = 0;
		S.VSpc = 0;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = 40;
		S.WrapStyle = WLCD_WRAP_NONE;
		//
		wlcd_text_measure("DONE", &S, &W, &H);
		DoneXOffs = (WLCD_WIDTH-W)/2;
		DoneYOffs = (WLCD_HEIGHT-H)/2;
		//
#if(WLCD_DISPLAY==WLCD_ILI9341)
		ets_sprintf(Str, "WLCD %s, ILI9341, %d bpp", wlcd_get_version(), WLCD_BPP);
#elif(WLCD_DISPLAY==WLCD_ILI9488_KDv4_HACK)
		ets_sprintf(Str, "WLCD %s, ILI9488_KDv4_HACK, %d bpp", wlcd_get_version(), WLCD_BPP);
#elif(WLCD_DISPLAY==WLCD_ILI9488)
		ets_sprintf(Str, "WLCD %s, ILI9488, %d bpp", wlcd_get_version(), WLCD_BPP);
#endif
		wlcd_text_draw(Str, &S);
		//
		S.X = 10;
		S.Y = 25;
#ifdef WLCD_USE_HSPI
		ets_sprintf(Str, "HW mode, HSPI CLK = %d kHz", ((uint32_t)system_get_cpu_freq()*1000) / WLCD_SPI_CLK_PREDIV / WLCD_SPI_CLK_CNTDIV);
#else
		ets_sprintf(Str, "SW mode");
#endif
		wlcd_text_draw(Str, &S);
		//
		S.X = 10;
		S.Y = WLCD_HEIGHT - 10 - wlcd_text_nrows_height(1, &S) - 3;
		ets_sprintf(Str, "Test starts in ");
		wlcd_text_measure(Str, &S, &W, &H);
		wlcd_text_draw(Str, &S);
		//wlcd_text_draw_or_measure(Str, &S, 0, &W, &H); //this is also possible instead of two lines above
		//
		S.FontZoomAdd = 1;
		S.X = 10 + W;
		S.Y = WLCD_HEIGHT - 10 - wlcd_text_nrows_height(1, &S);
		S.MaxW = WLCD_WIDTH - 10 - S.X;
		S.Color.u32t = WLCD_RGB_TO_COLOR(0xFF, 0xFF, 0xFF);
		for(i=50;i>0;i--){
			wlcd_rect_fill(S.X, S.Y, wlcd_text_nchars_width(5, &S), wlcd_text_nrows_height(1, &S), 0);
			//
			ets_sprintf(Str, "%d.%d s", i/10, i%10);
			wlcd_text_draw(Str, &S);
			//
			OS_DELAY_MS(100);
		}
		S.FontZoomAdd = 0;
		//
		//---- random lines / line speed test
		//
		//clear whole display (black)
		wlcd_rect_fill(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0);
		//
		//draw 1000 random lines
		t = system_get_time();
		for(i=0;i<1000;i++){
			wlcd_line_draw(
				RAND_0_TO_X(WLCD_WIDTH-1), RAND_0_TO_X(WLCD_HEIGHT-1),
				RAND_0_TO_X(WLCD_WIDTH-1), RAND_0_TO_X(WLCD_HEIGHT-1),
				WLCD_RGB_TO_COLOR(0b00111000, 0b00111100, 0b01111000));
			//
			if(i%50==0) system_soft_wdt_feed(); //prevent rebooting if this loop takes too long
		}
		t = system_get_time() - t;
		//
		//compensate for "rand()" calculation
		t2 = system_get_time();
		volatile uint16_t Tmp;
		for(i=0;i<1000;i++){
			Tmp = RAND_0_TO_X(WLCD_WIDTH-1);
			Tmp = RAND_0_TO_X(WLCD_HEIGHT-1);
			Tmp = RAND_0_TO_X(WLCD_WIDTH-1);
			Tmp = RAND_0_TO_X(WLCD_HEIGHT-1);
		}
		t2 = system_get_time() - t2;
		//
		S.Color.u32t = WLCD_RGB_TO_COLOR(0xFF, 0xFF, 0xFF);
		S.X = DoneXOffs;
		S.Y = DoneYOffs;
		wlcd_text_draw("DONE", &S);
		//
		OS_DELAY_MS(1000);
		//
		//clear whole display (black) and show stats
		wlcd_rect_fill(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0);
		//
		S.Color.u32t = WLCD_RGB_TO_COLOR(0b01111000, 0b01111100, 0b11111000);
		S.X = 10;
		S.Y = 10;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = WLCD_HEIGHT-20;
		S.WrapStyle = WLCD_WRAP_WORDS;
		ets_sprintf(Str, "1000 random lines on display %dx%d took %d ms (compensated for \"rand()\" calculation)", WLCD_WIDTH, WLCD_HEIGHT, (t-t2)/1000);
		wlcd_text_draw(Str, &S);
		OS_DELAY_MS(5000);
		//
		//---- image draw / image speed test
		//
#if(WLCD_BPP==WLCD_16BPP)
		wfof_get_file_data_fast(WFOF_IDX_SMILEY_50X50_U16BPP, (uint32_t*)ImgData, 0, WFOF_SIZE_SMILEY_50X50_U16BPP);
#else
		wfof_get_file_data_fast(WFOF_IDX_SMILEY_50X50_U24BPP, (uint32_t*)ImgData, 0, WFOF_SIZE_SMILEY_50X50_U24BPP);
#endif
		//
		//clear whole display (black)
		wlcd_rect_fill(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0);
		//
		t = system_get_time();
		for(i=0;i<1000;i++){
			wlcd_img_draw(ImgData, RAND_0_TO_X(WLCD_WIDTH-1-50), RAND_0_TO_X(WLCD_HEIGHT-1-50));
			//
			if(i%50==0) system_soft_wdt_feed(); //prevent rebooting if this loop takes too long
		}
		t = system_get_time() - t;
		//
		S.Color.u32t = 0;
		S.X = DoneXOffs;
		S.Y = DoneYOffs;
		wlcd_text_draw("DONE", &S);
		//
		OS_DELAY_MS(1000);
		//
		//clear whole display (black) and show stats
		wlcd_rect_fill(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0);
		//
		S.Color.u32t = WLCD_RGB_TO_COLOR(0b11111000, 0b01111100, 0b01111000);
		S.X = 10;
		S.Y = 10;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = WLCD_HEIGHT-20;
		S.WrapStyle = WLCD_WRAP_WORDS;
		ets_sprintf(Str, "1000 images (50x50 pixels, uncompressed image) took %d ms", t/1000);
		wlcd_text_draw(Str, &S);
		OS_DELAY_MS(5000);
		//
		//---- text draw / text speed test
		//
		//clear whole display (black)
		wlcd_rect_fill(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0);
		//
		ets_sprintf(Str, "This is a test of text drawing of 60 chars and with wrapping");
		t = system_get_time();
		for(i=0;i<100;i++){
			S.X = 10 + RAND_0_TO_X(10);
			S.Y = 10 + RAND_0_TO_X(100);
			S.MaxW = WLCD_WIDTH - 10 - S.X;
			S.MaxH = WLCD_HEIGHT - 10 - S.Y;
			S.Color.u32t = RAND_0_TO_X(0xFFFFFF);
			wlcd_text_draw(Str, &S);
			//
			if(i%50==0) system_soft_wdt_feed(); //prevent rebooting if this loop takes too long
		}
		t = system_get_time() - t;
		//
		S.Color.u32t = 0;
		S.X = DoneXOffs;
		S.Y = DoneYOffs;
		wlcd_text_draw("DONE", &S);
		//
		OS_DELAY_MS(1000);
		//
		//clear whole display (black) and show stats
		wlcd_rect_fill(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0);
		//
		S.Color.u32t = WLCD_RGB_TO_COLOR(0b11111000, 0b01111100, 0b11111000);
		S.X = 10;
		S.Y = 10;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = WLCD_HEIGHT-20;
		S.WrapStyle = WLCD_WRAP_WORDS;
		ets_sprintf(Str, "100 text sentences of 60 letters with word wrapping took %d ms", t/1000);
		wlcd_text_draw(Str, &S);
		OS_DELAY_MS(5000);
		//
		//---- text / font test - code page 1250 test
		//
		//clear whole display (black)
		wlcd_rect_fill(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0);
		//
		S.Color.u32t = WLCD_RGB_TO_COLOR(0xFF, 0x5C, 0x20);
		S.X = 10;
		S.Y = 10;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = WLCD_HEIGHT-20;
		S.WrapStyle = WLCD_WRAP_WORDS;
		wlcd_text_draw("This is a test of code page 1250 font text output:\n\nPøíšernì žluouèký kùò úpìl ïábelské ódy.", &S);
		//
		OS_DELAY_MS(4000);
		//
		//clear whole display (black)
		wlcd_rect_fill(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0);
		//
		S.Color.u32t = WLCD_RGB_TO_COLOR(0xFF, 0xFF, 0x00);
		S.FontZoomAdd = 1;
		wlcd_text_draw("Pøíšernì žluouèký kùò úpìl ïábelské ódy.", &S);
		//
		OS_DELAY_MS(3000);
		//
		//clear whole display (black)
		wlcd_rect_fill(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0);
		//
		S.Y = 10;
		S.Color.u32t = WLCD_RGB_TO_COLOR(0x00, 0xFF, 0);
		S.FontZoomAdd = 0;
		S.FontIdx = 0;
		wlcd_text_draw("This is some text.", &S);
		//
		S.Y = 30;
		S.Color.u32t = WLCD_RGB_TO_COLOR(0x00, 0xFF, 0xFF);
		S.BoldAdd = 1;
		wlcd_text_draw("And this is the same font bolder.", &S);
		//
		S.Y = 50;
		S.Color.u32t = WLCD_RGB_TO_COLOR(0b11000000, 0b00111100, 0b11111000);
		S.HSpc = 5;
		wlcd_text_draw("We can also add horiz./vert. spacing.", &S);
		//
		S.Y = 75;
		S.Color.u32t = WLCD_RGB_TO_COLOR(0b11111000, 0b00111100, 0b01111000);
		S.FontZoomAdd = 1;
		S.BoldAdd = 0;
		S.HSpc = 0;
		wlcd_text_draw("Or zoom ...", &S);
		//
		S.Y = 105;
		S.Color.u32t = WLCD_RGB_TO_COLOR(0b10010000, 0b11111100, 0b01111000);
		S.FontZoomAdd = 2;
		S.HSpc = 0;
		wlcd_text_draw("ZOOM", &S);
		//
		S.Y = 140;
		S.Color.u32t = WLCD_RGB_TO_COLOR(0b01000000, 0b10000000, 0b11111000);
		S.FontZoomAdd = 0;
		S.FontIdx = 3;
		wlcd_text_draw("Different font\n0123456789", &S);
		//
		S.Y = 180;
		S.Color.u32t = WLCD_RGB_TO_COLOR(0xFF, 0xFF, 0x00);
		S.FontIdx = 2;
		wlcd_text_draw("And another different font (you can generate your own fonts)", &S);
		//
		OS_DELAY_MS(6000);
		//
		//---- read display data RAM back into buffer as WLCD image and use it to draw many copies of that image
		//
#ifndef WLCD_NO_READ
		//clear whole display (black)
		wlcd_rect_fill(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0);
		//
		for(i=0;i<50;i++){
			wlcd_line_draw(
				RAND_0_TO_X(WLCD_WIDTH-1), RAND_0_TO_X(WLCD_HEIGHT-1),
				RAND_0_TO_X(WLCD_WIDTH-1), RAND_0_TO_X(WLCD_HEIGHT-1),
				WLCD_RGB_TO_COLOR(0b01111000, 0b01111100, 0b01111000));
		}
		//
		S.Color.u32t = WLCD_RGB_TO_COLOR(0b00001000, 0b11110100, 0b01111000);
		S.X = 10;
		S.Y = 10;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = WLCD_HEIGHT-20;
		S.WrapStyle = WLCD_WRAP_WORDS;
		wlcd_text_draw("It's also possible to read LCD memory into buffer - creating a WLCD image (uncompressed stream of pixels) and re-use this image", &S);
		//
		wlcd_line_draw(10   , 10   , 10+29, 10   , 0xFFFFFF);
		wlcd_line_draw(10+29, 10   , 10+29, 10+29, 0xFFFFFF);
		wlcd_line_draw(10+29, 10+29, 10   , 10+29, 0xFFFFFF);
		wlcd_line_draw(10   , 10+29, 10   , 10   , 0xFFFFFF);
		//
		uint32_t Ret = wlcd_img_get((uint32_t*)ImgData, 10, 10, 30, 30);
		for(i=0;i<WLCD_WIDTH-30;i+=35){
			wlcd_img_draw(ImgData, i, 100);
		}
		//
		OS_DELAY_MS(8000);
#endif
		//
		//---- logo, github address
		//
		//clear whole display (black)
		wlcd_rect_fill(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0);
		//
		//demo image
#if(WLCD_BPP==WLCD_16BPP)
		wfof_get_file_data_fast(WFOF_IDX_WLCD_DEMO_16BPP, (uint32_t*)ImgData, 0, WFOF_SIZE_WLCD_DEMO_16BPP);
#else
		wfof_get_file_data_fast(WFOF_IDX_WLCD_DEMO_24BPP, (uint32_t*)ImgData, 0, WFOF_SIZE_WLCD_DEMO_24BPP);
#endif
		wlcd_img_draw(ImgData, (WLCD_WIDTH-wlcd_img_get_width(ImgData))/2, ((WLCD_HEIGHT-wlcd_img_get_height(ImgData))/2)-20);
		//
		S.Color.u32t = WLCD_RGB_TO_COLOR(0xFF, 0x5C, 0x58);
		S.FontIdx = 1;
		S.X = 10;
		S.Y = WLCD_HEIGHT-40;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = 40;
		S.VSpc = 4;
		S.WrapStyle = WLCD_WRAP_WORDS;
		wlcd_text_draw("https://github.com/wdim0/esp8266_fast_lcd_driver_hspi", &S);
		//
		OS_DELAY_MS(6000);
		//
		//----
		//
	}
	//
	//clean-up (yes, we never get here .. but it's always good to keep things structured)
	os_free(Str);
	os_free(ImgData);
}
