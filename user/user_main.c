/*
 * Created by Martin Winkelhofer 04/2016
 * W-Dimension / wdim / wdim0 / maarty.w@gmail.com
 */

//#include <freertos/FreeRTOS.h>
//#include <freertos/task.h>
#include <espressif/esp_common.h>
#include <wlcd.h>
#include <wfof.h>
#include <wfof_idxs.h>

#include "user_config.h"

#define RAND_0_TO_X(x)			( (uint32_t)rand() / ((~(uint32_t)0)/(x)) ) //get uint32_t random number 0..x (including)

void ICACHE_FLASH_ATTR user_init(void){
	DBG("user_init() started\n");
	//
	//prepare test vars and buffers
	wlcd_text_draw_settings_struct S;
	//
	char* Str = malloc(256);
	uint8_t* ImgData = malloc(8000); //arrays allocated by malloc() are also always 4-bytes aligned (at least on ESP8266 SDK). Make sure that the buffer is big enough
	if((Str==NULL)||(ImgData==NULL)) return; //malloc() failed => abort
	uint16_t W, H;
	uint32_t t, t2;
	uint16_t i;
	uint16_t DoneXOffs;
	uint16_t DoneYOffs;
	//
	//init display
	wlcd_init();
	//
	wfof_get_file_data_fast(WFOF_IDX_WLCD_DEMO, (uint32_t*)ImgData, 0, WFOF_SIZE_WLCD_DEMO);
	//
	while(1){
		//
		//---- WLCD test / demo intro (countdown)
		//
		//clear whole display (black)
		wlcd_rect_fill_16bpp(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0x0000);
		//
		//demo image
		wlcd_img_draw(ImgData, (WLCD_WIDTH-wlcd_img_get_width(ImgData))/2, ((WLCD_HEIGHT-wlcd_img_get_height(ImgData))/2)-10);
		//
		//test countdown
		S.X = 10;
		S.Y = 10;
		S.R5G6B5 = 0xFAE4;
		S.FontIdx = 2;
		S.FontZoomAdd = 0;
		S.BoldAdd = 0;
		S.HSpc = 0;
		S.VSpc = 0;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = 20;
		S.WrapStyle = WLCD_WRAP_NONE;
		//
		wlcd_text_measure("DONE", &S, &W, &H);
		DoneXOffs = (WLCD_WIDTH-W)/2;
		DoneYOffs = (WLCD_HEIGHT-H)/2;
		//
		sprintf(Str, "WLCD %s", wlcd_get_version());
		wlcd_text_draw(Str, &S);
		//
		S.X = 10;
		S.Y = WLCD_HEIGHT - 10 - wlcd_text_nrows_height(1, &S) - 3;
		sprintf(Str, "Test starts in ");
		wlcd_text_measure(Str, &S, &W, &H);
		wlcd_text_draw(Str, &S);
		//wlcd_text_draw_or_measure(Str, &S, 0, &W, &H); //this is also possible instead of two lines above
		//
		S.FontZoomAdd = 1;
		S.X = 10 + W;
		S.Y = WLCD_HEIGHT - 10 - wlcd_text_nrows_height(1, &S);
		S.MaxW = WLCD_WIDTH - 10 - S.X;
		S.R5G6B5 = 0xFFFF;
		for(i=50;i>0;i--){
			wlcd_rect_fill_16bpp(S.X, S.Y, wlcd_text_nchars_width(5, &S), wlcd_text_nrows_height(1, &S), 0x0000);
			//
			sprintf(Str, "%d.%d s", i/10, i%10);
			wlcd_text_draw(Str, &S);
			//
			vTaskDelay(100 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
		}
		S.FontZoomAdd = 0;
		//
		//---- random lines / line speed test
		//
		//clear whole display (black)
		wlcd_rect_fill_16bpp(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0x0000);
		//
		//draw 1000 random lines
		t = system_get_time();
		for(i=0;i<1000;i++){
			wlcd_line_draw(
				RAND_0_TO_X(WLCD_WIDTH-1), RAND_0_TO_X(WLCD_HEIGHT-1),
				RAND_0_TO_X(WLCD_WIDTH-1), RAND_0_TO_X(WLCD_HEIGHT-1),
				0b0011100111101111);
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
		S.R5G6B5 = 0xFFFF;
		S.X = DoneXOffs;
		S.Y = DoneYOffs;
		wlcd_text_draw("DONE", &S);
		//
		vTaskDelay(1000 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
		//
		//clear whole display (black) and show stats
		wlcd_rect_fill_16bpp(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0x0000);
		//
		S.R5G6B5 = 0b0111101111111111;
		S.X = 10;
		S.Y = 10;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = WLCD_HEIGHT-20;
		S.WrapStyle = WLCD_WRAP_WORDS;
		sprintf(Str, "1000 random lines on display %dx%d took %d ms (compensated for \"rand()\" calculation)", WLCD_WIDTH, WLCD_HEIGHT, (t-t2)/1000);
		wlcd_text_draw(Str, &S);
		vTaskDelay(5000 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
		//
		//---- image draw / image speed test
		//
		wfof_get_file_data_fast(WFOF_IDX_SMILEY_50X50, (uint32_t*)ImgData, 0, WFOF_SIZE_SMILEY_50X50);
		//
		//clear whole display (black)
		wlcd_rect_fill_16bpp(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0x0000);
		//
		t = system_get_time();
		for(i=0;i<1000;i++){
			wlcd_img_draw(ImgData, RAND_0_TO_X(WLCD_WIDTH-1-50), RAND_0_TO_X(WLCD_HEIGHT-1-50));
			//
			if(i%50==0) system_soft_wdt_feed(); //prevent rebooting if this loop takes too long
		}
		t = system_get_time() - t;
		//
		S.R5G6B5 = 0x0000;
		S.X = DoneXOffs;
		S.Y = DoneYOffs;
		wlcd_text_draw("DONE", &S);
		//
		vTaskDelay(1000 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
		//
		//clear whole display (black) and show stats
		wlcd_rect_fill_16bpp(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0x0000);
		//
		S.R5G6B5 = 0b1111101111101111;
		S.X = 10;
		S.Y = 10;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = WLCD_HEIGHT-20;
		S.WrapStyle = WLCD_WRAP_WORDS;
		sprintf(Str, "1000 images (50x50 pixels, uncompressed R5G6B5) took %d ms", t/1000);
		wlcd_text_draw(Str, &S);
		vTaskDelay(5000 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
		//
		//---- text draw / text speed test
		//
		//clear whole display (black)
		wlcd_rect_fill_16bpp(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0x0000);
		//
		sprintf(Str, "This is a test of text drawing of 60 chars and with wrapping");
		t = system_get_time();
		for(i=0;i<100;i++){
			S.X = 10 + RAND_0_TO_X(10);
			S.Y = 10 + RAND_0_TO_X(100);
			S.MaxW = WLCD_WIDTH - 10 - S.X;
			S.MaxH = WLCD_HEIGHT - 10 - S.Y;
			S.R5G6B5 = RAND_0_TO_X(0xFFFF);
			wlcd_text_draw(Str, &S);
			//
			if(i%50==0) system_soft_wdt_feed(); //prevent rebooting if this loop takes too long
		}
		t = system_get_time() - t;
		//
		S.R5G6B5 = 0x0000;
		S.X = DoneXOffs;
		S.Y = DoneYOffs;
		wlcd_text_draw("DONE", &S);
		//
		vTaskDelay(1000 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
		//
		//clear whole display (black) and show stats
		wlcd_rect_fill_16bpp(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0x0000);
		//
		S.R5G6B5 = 0b1111101111111111;
		S.X = 10;
		S.Y = 10;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = WLCD_HEIGHT-20;
		S.WrapStyle = WLCD_WRAP_WORDS;
		sprintf(Str, "100 text sentences of 60 letters with word wrapping took %d ms", t/1000);
		wlcd_text_draw(Str, &S);
		vTaskDelay(5000 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
		//
		//---- text / font test - code page 1250 test
		//
		//clear whole display (black)
		wlcd_rect_fill_16bpp(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0x0000);
		//
		S.R5G6B5 = 0xFAE4;
		S.X = 10;
		S.Y = 10;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = WLCD_HEIGHT-20;
		S.WrapStyle = WLCD_WRAP_WORDS;
		wlcd_text_draw("This is a test of code page 1250 font text output:\n\nPøíšernì žluouèký kùò úpìl ïábelské ódy.", &S);
		//
		vTaskDelay(4000 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
		//
		//clear whole display (black)
		wlcd_rect_fill_16bpp(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0x0000);
		//
		S.R5G6B5 = 0b1111111111100000;
		S.FontZoomAdd = 1;
		wlcd_text_draw("Pøíšernì žluouèký kùò úpìl ïábelské ódy.", &S);
		//
		vTaskDelay(3000 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
		//
		//clear whole display (black)
		wlcd_rect_fill_16bpp(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0x0000);
		//
		S.Y = 10;
		S.R5G6B5 = 0b0000011111100000;
		S.FontZoomAdd = 0;
		S.FontIdx = 0;
		wlcd_text_draw("This is some text.", &S);
		//
		S.Y = 30;
		S.R5G6B5 = 0b0000011111111111;
		S.BoldAdd = 1;
		wlcd_text_draw("And this is the same font bolder.", &S);
		//
		S.Y = 50;
		S.R5G6B5 = 0b1100000111111111;
		S.HSpc = 5;
		wlcd_text_draw("We can also add horiz./vert. spacing.", &S);
		//
		S.Y = 75;
		S.R5G6B5 = 0b1111100111101111;
		S.FontZoomAdd = 1;
		S.BoldAdd = 0;
		S.HSpc = 0;
		wlcd_text_draw("Or zoom ...", &S);
		//
		S.Y = 105;
		S.R5G6B5 = 0b1001011111101111;
		S.FontZoomAdd = 2;
		S.HSpc = 0;
		wlcd_text_draw("ZOOM", &S);
		//
		S.Y = 140;
		S.R5G6B5 = 0b0100010000011111;
		S.FontZoomAdd = 0;
		S.FontIdx = 3;
		wlcd_text_draw("Different font\n0123456789", &S);
		//
		S.Y = 180;
		S.R5G6B5 = 0b1111111111100000;
		S.FontIdx = 2;
		wlcd_text_draw("And another different font (you can generate your own fonts)", &S);
		//
		vTaskDelay(6000 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
		//
		//---- read display data RAM back into buffer as WLCD image and use it to draw many copies of that image
		//
#ifndef WLCD_NO_READ
		//clear whole display (black)
		wlcd_rect_fill_16bpp(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0x0000);
		//
		for(i=0;i<50;i++){
			wlcd_line_draw(
				RAND_0_TO_X(WLCD_WIDTH-1), RAND_0_TO_X(WLCD_HEIGHT-1),
				RAND_0_TO_X(WLCD_WIDTH-1), RAND_0_TO_X(WLCD_HEIGHT-1),
				0b0111101111101111);
		}
		//
		S.R5G6B5 = 0x0FAF;
		S.X = 10;
		S.Y = 10;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = WLCD_HEIGHT-20;
		S.WrapStyle = WLCD_WRAP_WORDS;
		wlcd_text_draw("It's also possible to read LCD memory into buffer - creating a WLCD image (uncompressed R5G6B5 stream of pixels) and re-use this image", &S);
		//
		wlcd_line_draw(10   , 10   , 10+29, 10   , 0xFFFF);
		wlcd_line_draw(10+29, 10   , 10+29, 10+29, 0xFFFF);
		wlcd_line_draw(10+29, 10+29, 10   , 10+29, 0xFFFF);
		wlcd_line_draw(10   , 10+29, 10   , 10   , 0xFFFF);
		//
		uint32_t Ret = wlcd_img_get((uint32_t*)ImgData, 10, 10, 30, 30);
		for(i=0;i<WLCD_WIDTH-30;i+=35){
			wlcd_img_draw(ImgData, i, 100);
		}
		//
		vTaskDelay(8000 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
#endif
		//
		//---- logo, github address
		//
		//clear whole display (black)
		wlcd_rect_fill_16bpp(0,0, WLCD_WIDTH, WLCD_HEIGHT, 0x0000);
		//
		//demo image
		wfof_get_file_data_fast(WFOF_IDX_WLCD_DEMO, (uint32_t*)ImgData, 0, WFOF_SIZE_WLCD_DEMO);
		wlcd_img_draw(ImgData, (WLCD_WIDTH-wlcd_img_get_width(ImgData))/2, ((WLCD_HEIGHT-wlcd_img_get_height(ImgData))/2)-20);
		//
		S.R5G6B5 = 0xFAEB;
		S.FontIdx = 1;
		S.X = 10;
		S.Y = WLCD_HEIGHT-40;
		S.MaxW = WLCD_WIDTH-20;
		S.MaxH = 40;
		S.VSpc = 4;
		S.WrapStyle = WLCD_WRAP_WORDS;
		wlcd_text_draw("https://github.com/wdim0/esp8266_fast_lcd_driver_hspi", &S);
		//
		vTaskDelay(6000 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
		//
		//----
		//
	}
	//
	//clean-up (yes, we never get here .. but it's always good to keep things structured)
	free(Str);
	free(ImgData);
}
