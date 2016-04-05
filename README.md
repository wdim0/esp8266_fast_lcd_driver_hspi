#ILI9341 / ILI9488 LCD driver for ESP8266

Fast LCD driver written from scratch for ESP8266 to communicate with <b>ILI9341 (240x320) / ILI9488 (320x480)</b> compatible LCD controllers using ESP8266's HSPI (full 16 x 32-bit buffer) or SW bit-banging.<br />
Maximum effort was taken to create a fast driver.<br />
(TODO - ILI9488 is pending ... coming soon)<br />
(tested with ESP8266_RTOS_SDK 1.4.0)

[![esp8266_fast_lcd_driver_hspi](http://img.youtube.com/vi/E9Ds4IS-Ndk/0.jpg)](http://www.youtube.com/watch?v=E9Ds4IS-Ndk)

##Connection with LCD controller

	 ---------------------------             ----------------------------
	|          ESP8266          |           |       LCD controller       |
	|          (master)         |           |           (slave)          |
	|                           |           |                            |
	|       MTDO/GPIO15/HSPI_CS |----->-----| CS (active L)              |
	|     MTCK/GPIO13/HSPI_MOSI |----->-----| MOSI                       |
	|     MTDI/GPIO12/HSPI_MISO |-----<-----| MISO                       |
	|      MTMS/GPIO14/HSPI_CLK |----->-----| CLK (leading edge, act. H) |
	|                     GPIO4 |----->-----| D/C (data H / command L)   |
	 ---------------------------             ----------------------------

If WLCD_USE_HSPI is defined, you can change only CS and D/C GPIOs. The rest (HSPI controller) is hardwired inside ESP8266.<br />
If WLCD_USE_HSPI is not defined, you can change all GPIOs (see "WLCD MAIN CONFIG" section in wlcd.h).

##Main features

In HSPI mode (using ESP8266's HSPI hardware module) the whole 16 x 32-bit buffer (SPI_W0..15) is used for MISO/MOSI transactions and we're using 32-bit copy instructions to speed up the copy process.<br />
There's tradeoff because of that - if we're drawing image which is not RLE compressed (is just a continuous stream of R5G6B5 pixels) and we're using HSPI mode, then image data buffer must be 4-bytes aligned and allocated memory length must be multiples of 4.

You can choose your own HSPI clock rate (see "WLCD MAIN CONFIG" section in wlcd.h). My display (see YT video on the top) is happy up to 40 MHz SPI clock (including).

In SW bit-banging mode (use HSPI whenever possible) you can easily change all GPIOs what are used to communicate with LCD controller (see "WLCD MAIN CONFIG" section in wlcd.h).<br />
In HSPI you can change CS and D/C GPIOs.

Fonts are implemented using RREs (each letter is represented by rectangle(s), minimum possible). This gives us speed, easy font zooming / making font bolder, ...<br />
Fonts with code page 437 (USA, original IBM PC hardware) and code page 1250 (Central and East European Latin) included. If you need your own fonts / code pages, use fonter.c included as a starting point and assemble your own wlcd_font_*.h

RGB mode is 16-bits per pixel ONLY, because we want to be as fast as possible (many optimizations take advance of the fact, that we can move exactly two pixels by one 32-bit move). That's why also <b>WLCD image</b> format (use wlcd_img_gen to generate from 24-bit of 16-bit BMPs (TODO - need linux port of this tool)) knows only 16 bpp images. WLCD image can be (and in most cases is) RLE compressed (8-bit counter).

Function for drawing a line is heavily optimized for speed (addition of fixed point with precalculations, repeating of preconfigured HSPI transaction, ...). The speed-up is the more significant, the more the line is horizontal or vertical. Diagonal lines are the slowest (there is no other way than re-set drawing point for every pixel using wlcd_set_drawing_rect(...) (SET_COLUMN_ADDR_RANGE + SET_ROW_ADDR_RANGE)).

Orientation can be changed in 0 / 90 / 180 / 270 degrees of rotation (see "WLCD MAIN CONFIG" section in wlcd.h).

Reading of display data RAM right into R5G6B5 buffer of pixels is implemented (conversion of R6G6B6 to R5G6B5 on the fly).

Demo included

##Provided functions

	char* ICACHE_FLASH_ATTR wlcd_get_version(void);

	int8_t ICACHE_FLASH_ATTR wlcd_init(void);

	void /*ICACHE_FLASH_ATTR*/ wlcd_set_drawing_rect(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height);
	void /*ICACHE_FLASH_ATTR*/ wlcd_write_dup_16bpp(uint16_t R5G6B5, uint32_t Dup16Count);
	void /*ICACHE_FLASH_ATTR*/ wlcd_rect_fill_16bpp(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint16_t R5G6B5);
	#define wlcd_pixel_draw(X, Y, R5G6B5)		wlcd_rect_fill_16bpp(X, Y, 1, 1, R5G6B5) //! for continuous stream of pixels use wlcd_write_dup_16bpp(...) or wlcd_img_draw(...)
	void /*ICACHE_FLASH_ATTR*/ wlcd_line_draw(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint16_t R5G6B5);

	void ICACHE_FLASH_ATTR wlcd_img_get_info(uint8_t* ImgData, uint16_t* RetImgW, uint16_t* RetImgH, uint8_t* IsRLECompressed);
	#define wlcd_img_get_width(ImgData)				(*(uint16_t*)(ImgData))
	#define wlcd_img_get_height(ImgData)			(*(((uint16_t*)(ImgData))+1) & 0x7FFF)
	#define wlcd_img_is_rle_compressed(ImgData)		(((uint8_t*)ImgData)[3] & 0x80)
	void ICACHE_FLASH_ATTR wlcd_img_draw(uint8_t* ImgData, uint16_t X, uint16_t Y); //! if the image is not RLE compressed and we're using HSPI (WLCD_USE_HSPI is defined), then ImgData must be 4-bytes aligned and allocated memory length must be multiples of 4
	uint32_t ICACHE_FLASH_ATTR wlcd_img_get(uint32_t* Buf, uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height); //! if WLCD_USE_HSPI is defined, then Buf must be 4-bytes aligned and allocated memory length must be multiples of 4

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

	void ICACHE_FLASH_ATTR wlcd_text_draw_or_measure(char* Txt, wlcd_text_draw_settings_struct* S, uint8_t DontDrawJustMeasure, uint16_t* RetWidth, uint16_t* RetHeight);
	#define wlcd_text_measure(Txt, S, RetWidth, RetHeight)	wlcd_text_draw_or_measure(Txt, S, 1, RetWidth, RetHeight)
	#define wlcd_text_draw(Txt, S)							wlcd_text_draw_or_measure(Txt, S, 0, NULL, NULL)
	uint16_t ICACHE_FLASH_ATTR wlcd_text_nchars_width(uint16_t NumOfChars, wlcd_text_draw_settings_struct* S);
	uint16_t ICACHE_FLASH_ATTR wlcd_text_nrows_height(uint16_t NumOfRows, wlcd_text_draw_settings_struct* S);

(see wlcd.c for details)	
