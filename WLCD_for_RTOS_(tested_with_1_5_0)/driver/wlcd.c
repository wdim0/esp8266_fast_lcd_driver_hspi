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
 * Special thanks to:
 * http://d.av.id.au/blog/hardware-spi-hspi-command-data-registers/ (how to program HSPI)
 * https://github.com/MetalPhreak/ESP8266_SPI_Driver (how to program HSPI)
 * https://github.com/adafruit/Adafruit_ILI9341 (ILI9341 magic init codes)
 * https://github.com/cnlohr/pylotron/blob/master/fonter.c (turn PBM images of fonts into RREs C arrays - used to create wlcd_font_*.h files)
 *
 * ################################################################################
 * #     Communication with LCD controller / 4-line serial interface protocol     #
 * ################################################################################
 *
 * Writing 8-bit command (8-bit command + no parameters):
 * ======================================================
 *
 *            --------------- 8-bit command ---------------
 * SDI        C07   C06   C05   C04   C03   C02   C01   C00
 *              __    __    __    __    __    __    __    __
 * >SCL ???____|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |____????
 *
 * D/#C ????????????????????????????????????????????????_____????????
 *      _____                                                   _____
 * #CS       |_________________________________________________|
 *
 *
 * Writing 8-bit command + N parameters (8-bit command + N*8-bit parameters):
 * OR writing pixel data (8-bit command WLCD_WRITE_DDRAM, N pixels):
 * ==========================================================================
 * Note: each pixel takes 0.5/2/3 bytes depending on color depth selected by bit[2:0] (DBI bpp) in WLCD_PIXEL_FORMAT_SET register
 * bit[2:0] = 0b001 = 3 bits/pixel (8 colors), 2 pixels takes 1 byte - format 0b00RGBRGB
 * bit[2:0] = 0b101 = 16 bits / pixel (65536 colors), 1 pixel takes 2 bytes - format R5G6B5
 * bit[2:0] = 0b110 = 18 bits / pixel (65536 colors), 1 pixel takes 3 bytes - format R6G6B6 or R8G8B8 (lowest 2 bits in each byte are ignored)
 *
 *            --------------- 8-bit command ---------------   ------------- 8-bit parameter 1 -------------          ------------- 8-bit parameter N -------------
 * SDI        C07   C06   C05   C04   C03   C02   C01   C00   D07   D06   D05   D04   D03   D02   D01   D00     ...  D07   D06   D05   D04   D03   D02   D01   D00
 *              __    __    __    __    __    __    __    __    __    __    __    __    __    __    __    __           __    __    __    __    __    __    __    __
 * >SCL ???____|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |_  ...  _|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |____????
 *                                                                                                      _____                                                  _____
 * D/#C ????????????????????????????????????????????????_____???????????????????????????????????????????     ?  ...  ??????????????????????????????????????????     ????????
 *      _____                                                                                                                                                          _____
 * #CS       |________________________________________________________________________________________________  ...  _________________________________________________|
 *
 *
 * Reading 8-bit parameter (write 8-bit command + read 8-bit parameter):
 * =====================================================================
 *
 *            --------------- 8-bit command ---------------   -------------- 8-bit parameter --------------
 * SDI        C07   C06   C05   C04   C03   C02   C01   C00
 * SDO                                                        D07   D06   D05   D04   D03   D02   D01   D00
 *              __    __    __    __    __    __    __    __    __    __    __    __    __    __    __    __
 * >SCL ???____|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |____????
 *
 * D/#C ????????????????????????????????????????????????_____????????????????????????????????????????????????????????
 *      _____                                                                                                   _____
 * #CS       |_________________________________________________________________________________________________|
 *
 *
 * Reading 24-bit/32-bit parameter (write 8-bit command + read 24-bit/32-bit parameter):
 * =====================================================================================
 *
 *                                                            one dummy >SCL cycle !
 *            --------------- 8-bit command ---------------    |    -------------- 24-bit/32-bit parameter -------------
 * SDI        C07   C06   C05   C04   C03   C02   C01   C00    |
 *                                                             |    D23   D22   D21   D20     ...  D03   D02   D01   D00    <- (for 24-bit)
 * SDO                                                       -----  D31   D30   D29   D28     ...  D03   D02   D01   D00    <- (for 32-bit)
 *              __    __    __    __    __    __    __    __    __    __    __    __    __           __    __    __    __
 * >SCL ???____|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |__|  |_  ...  _|  |__|  |__|  |__|  |____????
 *
 * D/#C ????????????????????????????????????????????????_____???????????????????????????????  ...  ???????????????????????????????
 *      _____                                                                                                                _____
 * #CS       |______________________________________________________________________________  ...  _________________________|
 *
 *
 * Reading pixel data (write 8-bit command WLCD_READ_DDRAM + read N pixels):
 * =========================================================================
 * It's the same as reading 24-bit/32-bit parameter (see above), but:
 * - there must be 8 dummy >SCL cycles instead of one
 * - each pixel takes 24-bits (R6G6B6), no matter what color depth is selected by bit[2:0] (DBI bpp) in WLCD_PIXEL_FORMAT_SET register
 *
 * ################################################################################
 */

#include "driver/spi_register.h" //this must be included before "esp_common.h", because we need to use this more complete version (in standard spi_register.h some things are missing)
//#include <freertos/FreeRTOS.h>
//#include <freertos/task.h>
#include <espressif/esp_common.h>
#include <driver/gpio.h>

#include "wlcd_defs.h"
#include "wlcd_fonts.h"

//WLCD - MAIN

#define HSPI					1
#define HSPI_BUSY				(READ_PERI_REG(SPI_CMD(HSPI)) & SPI_USR)
#define HSPI_START				(SET_PERI_REG_MASK(SPI_CMD(HSPI), SPI_USR))

#define FLG_SEND_CMD			(1<<0) //send Cmd before data bytes
#define FLG_PULSE_CS_ON_START	(1<<1) //set #CS to inactive state before first transmission (reset SPI interface of LCD controller)
#define FLG_D_C					(1<<2) //used just in SW bit-banging mode - set D/#C line to log. value of this flag
#define FLG_DO_SHRINK_R6G6B6_TO_R5G6B5 (1<<3) //used in wlcd_read(...) - when reading LCD pixel data, LCD controller outputs only in R6G6B6 format (24-bits per one pixel). This causes that the Buf buffer is filled by R5G6B5

#define WLCD_CS_H				(GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1<<WLCD_GPIO_FOR_LCD_CS))    //LCD #CS (active L)
#define WLCD_CS_L				(GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1<<WLCD_GPIO_FOR_LCD_CS))    // ...
#define WLCD_D_C_H				(GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1<<WLCD_GPIO_FOR_LCD_D_C))   //LCD D/#C (data H / command L)
#define WLCD_D_C_L				(GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1<<WLCD_GPIO_FOR_LCD_D_C))   //

//-- only for SW bit-banging (when WLCD_USE_HSPI is defined, these definitions has no effect)
#define DELAY					//no delay
//#define DELAY					vTaskDelay(1) //debug - for really slow bit-banging using oscilloscope
#define WLCD_D_C(x)				((x)?WLCD_D_C_H:WLCD_D_C_L)
#define WLCD_SCL_H				(GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1<<WLCD_GPIO_FOR_LCD_SCL))   //LCD >SCL (rising edge)
#define WLCD_SCL_L				(GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1<<WLCD_GPIO_FOR_LCD_SCL))   // ...
#define WLCD_SDO				((GPIO_REG_READ(GPIO_IN_ADDRESS)&(BIT(WLCD_GPIO_FOR_LCD_SDO)))!=0)  //LCD SDO
#define WLCD_SDI_H				(GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1<<WLCD_GPIO_FOR_LCD_SDI))   //LCD SDI
#define WLCD_SDI_L				(GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1<<WLCD_GPIO_FOR_LCD_SDI))   // ...
#define WLCD_SDI(x)				((x)?WLCD_SDI_H:WLCD_SDI_L)                                         // ...
//--

//low level communication - HSPI

uint8_t /*ICACHE_FLASH_ATTR*/ wlcd_write_buf_hspibuf64(uint8_t Flags, uint8_t Cmd, uint32_t* Buf, uint32_t Bytes){
/* ============================================================================
 * Transmits Cmd byte (if requested by Flags) followed by Bytes (max 64) data bytes from Buf to LCD controller.
 * This function uses 32-bit copy from Buf -> SPI_W0..15 to be AS FAST AS POSSIBLE.
 * We're using entire HSPI buffer (SPI_W0..15) = 16 x 32-bit => max 64 data bytes can be sent in one call.
 * Relevant only when WLCD_USE_HSPI is defined.
 *
 * MAXIMUM EFFORT WAS TAKEN TO MAKE THIS AS FAST AS POSSIBLE
 *
 * Input:
 * - Flags - combination of FLG_* (see definition for details), relevant are only:
 *   - FLG_PULSE_CS_ON_START
 *   - FLG_SEND_CMD
 * - Cmd - 8-bit command for the LCD controller (relevant only if FLG_SEND_CMD flag set)
 * - Buf - pointer to data bytes to be transmitted - must be 4-bytes aligned, allocated memory length must be multiples of 4
 * - Bytes - number of data bytes to transmit (Cmd is not counted)
 *
 * Output:
 * - returns number of really transmitted data bytes (Cmd is not counted).
 *   Maximum is 64 bytes. If you get less than 64, the value will be equal to Bytes and it means
 *   that all data bytes were transmitted.
 *   0 means no data bytes were transmitted (just Cmd transmitted if FLG_SEND_CMD flag set)
 *
 * ============================================================================
 *
 * THE 4-BYTE ALIGNMENT "PROBLEM"
 * ------------------------------
 *
 * ESP8266 has "Unaligned Exception Option" active in the CPU core and any unaligned copy will cause
 * "Fatal exception (9)" (where 9 stands for "Attempt to load or store data at an address which
 * cannot be handled due to alignment" according to XTENSA LX 106 datasheet).
 * Why it needs this alignment? Performance! (no bit shifting needed when all copying is aligned).
 * =>
 * Buf must be 4-bytes aligned pointer (i.e. lowest 2 bits of the address must be 0).
 * Please use STORE_ATTR ("__attribute__ ((aligned(4)))") prefix when declaring arrays passed as Buf
 * or use os_malloc(...) - at least on ESP8266 the os_malloc(...) also allocates 4-byte aligned arrays.
 * uint8_t STORE_ATTR MyBuf[10*4];
 * uint32_t STORE_ATTR MyBuf[10];
 * uint8_t* MyBuf = (uint8_t*)os_malloc(10*4);
 * All of these will produce pointer to MyBuf that's 4-bytes aligned.
 *
 * You need to be sure, that when you make offsets to MyBuf base (which you pass as new Buf
 * to wlcd_write_buf_hspibuf64(...) when transmitting more than 64 data bytes), the offset is always
 * divisible by 4 without remainder.
 *
 * Also, because of 32-bit copy, the length of Buf must be multiples of 4. Yes, we'll waste 3 bytes of RAM
 * at maximum at the end of buffer in worse case scenario, but it's a pretty good tradeoff for speed.
 *
 * Example - writing 8-bit command + 257 data bytes to LCD controller:
 * ------------------------------
 *
 * #define MYBUF_BYTES		257 //let's choose this funny size for this example to make things more obvious
 * uint32_t STORE_ATTR MyBuf[(MYBUF_BYTES/4)+(MYBUF_BYTES % 4)?1:0]; //this will allocate 260 bytes (65 x uint32_t)
 *
 * //... fill MyBuf with bytes here, like it's an array of bytes. Fill in the same order as we want to transmit ...
 *
 * uint8_t Flags = FLG_SEND_CMD|FLG_PULSE_CS_ON_START;
 * uint8_t Cmd = 0xAB;
 * uint16_t BytesRemaining = MYBUF_BYTES;
 * while(BytesRemaining){
 *     BytesRemaining -= wlcd_write_buf_hspibuf64(Flags, Cmd, (uint32_t*)&(((uint8_t*)MyBuf)[MYBUF_BYTES-BytesRemaining]), BytesRemaining);
 *     Flags = 0;
 * }
 *
 * ------------------------------
 *
 * Notes about the HSPI controller and the order of bits:
 * ------------------------------------------------------
 *
 * The COMMAND segment write is SPI_WR_BYTE_ORDER independent => low byte of 16-bit command is always clocked out first.
 * Order is from MSB to LSB. Then high byte follows, again from MSB to LSB. This is also how we use SPI_W0..15 for MOSI/MISO.
 * Example:
 * If COMMAND value is 0xFBCD
 * - if number of command bits to transmit is 4 (set 4-1), output is 0xC
 * - if number of command bits to transmit is 8 (set 8-1), output is 0xCD
 * - if number of command bits to transmit is 9 (set 9-1), output is 0b110011011 (notice 9 bits, 0xCD + MSB from 0xFB)
 * - if number of command bits to transmit is 12 (set 12-1), output is 0xCDF
 *
 * The ADDR / MOSI / MISO segments output/input order is defined by SPI_WR_BYTE_ORDER/SPI_RD_BYTE_ORDER bit in SPI_USER(HSPI) register.
 * - if cleared (our case), the output order is LOW byte to HIGH byte, from MSB to LBS in all 4-bytes, so the transmission
 *   sequence is like this: b7..b0, b15..b8, b23..b16, b31..b24
 *
 *                                        SPI_W0 (32-bit register)
 *   Buf[0] Buf[1] Buf[2] Buf[3] -->  LOW_byte  byte1  byte2  HIGH_byte  --> HSPI MOSI will shift this out as 0x12345678 stream
 *   0x12   0x34   0x56   0x78          0x12    0x34   0x56     0x78
 *     ^___                               ^
 *         |                              |
 *   from this address we copy 32-bits to this using 32-bit copy (FAST)
 *
 * - if set, the output order is HIGH byte to LOW byte, from MSB to LBS in all 4-bytes. The transmission
 *   sequence is then: b31..b0
 *
 * ============================================================================
 */
	while(HSPI_BUSY); //wait for transaction to complete (if any)
	//
	if(Flags & FLG_PULSE_CS_ON_START) WLCD_CS_H; //set #CS to inactive state for a while => reset SPI interface of LCD controller
	//
	//-- put all precomputing here to make #CS inactive pulse longer
	uint8_t BytesToHSPIBuf = (Bytes>64)?64:Bytes; //we have SPI_W0..15 32-bit regs => max 64 bytes
	uint32_t* HSPIBuf;
	uint8_t BytesDone;
	//
	if(BytesToHSPIBuf){ //there are some data bytes to transmit, do necessary precomputing
		HSPIBuf = (uint32_t*)SPI_W0(HSPI);
		BytesDone = 0;
	}
	//--
	//
	//send 8-bit Cmd command. Send it separately because we need to manage D/#C => we cannot send everything at once
	if(Flags & FLG_SEND_CMD){
		//
		WLCD_CS_L;
		//
		CLEAR_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_ADDR|SPI_USR_MISO|SPI_USR_MOSI|SPI_USR_DUMMY); //we don't use ADDR, MISO, MOSI, DUMMY parts in the HSPI transaction
		SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_COMMAND); //we use just COMMAND part in the HSPI transaction now (we need to control D/#C)
		//
		WRITE_PERI_REG(SPI_USER2(HSPI),
			(((8-1)&SPI_USR_COMMAND_BITLEN)<<SPI_USR_COMMAND_BITLEN_S) | //number of command bits to transmit (lowest 8-bits from SPI_USER2)
			Cmd); //set lowest 8-bits in SPI_USER2
		//
		WLCD_D_C_L; //command
		HSPI_START; //start transmission of configured transaction - 8-bit Cmd command
	}
	//
	if(!BytesToHSPIBuf) return 0; //there are no data bytes to transmit
	//
	WLCD_CS_L;
	//
	while(HSPI_BUSY); //wait for transaction to complete (if any) (put this as "late" as possible in the program flow of this function - which is here)
	//
	//fill HSPI buffer (SPI_W* registers) as fast as possible - use 32-bit copy
	while(BytesDone<BytesToHSPIBuf){ //THIS LOOP IS CRITICAL (everything that could be done before or after is excluded from this loop. Also test in loop is intentionally just 8-bit)
		WRITE_PERI_REG(HSPIBuf++, *(Buf++));
		BytesDone+=4;
	}
	//
	//configure HSPI transaction and start transmission
	CLEAR_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_COMMAND|SPI_USR_ADDR|SPI_USR_MISO|SPI_USR_DUMMY); //we don't use COMMAND, ADDR, MISO, DUMMY parts in the HSPI transaction
	SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_MOSI); //we use just MOSI part in the HSPI transaction now
	//
	WRITE_PERI_REG(SPI_USER1(HSPI), ((((uint16_t)BytesToHSPIBuf*8)-1)&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S); //number of data bits (BytesToHSPIBuf*8) to transmit (from SPI_W0..15 32-bit registers => send buffer of max 64 bytes)
	//
	WLCD_D_C_H; //data
	HSPI_START; //start transmission of configured transaction. Now the HSPI will start to shift out all data bytes
	//
	return BytesToHSPIBuf;
}

void /*ICACHE_FLASH_ATTR*/ wlcd_write_dup16_hspi(uint8_t Flags, uint8_t Cmd, uint8_t Dup16H, uint8_t Dup16L, uint32_t Dup16Count){
/* Transmits Cmd byte (if requested by Flags) followed by 16-bit data bytes (Dup16H<<8 | Dup16L)
 * repeated Dup16Count times to LCD controller.
 * This is especially used for filling larger areas with one color in 16bpp (Dup16Count = number of pixels),
 * but also drawing RLE compressed WLCD image uses this (no intermediate buffer for uncompressed image).
 * Relevant only when WLCD_USE_HSPI is defined.
 *
 * MAXIMUM EFFORT WAS TAKEN TO MAKE THIS AS FAST AS POSSIBLE
 *
 * Input:
 * - Flags - combination of FLG_* (see definition for details), relevant are only:
 *   - FLG_PULSE_CS_ON_START
 *   - FLG_SEND_CMD
 * - Cmd - 8-bit command for the LCD controller (relevant only if FLG_SEND_CMD flag set)
 * - Dup16H - high 8-bits of 16-bit data (transmission order on MOSI line: first Dup16H then Dup16L, both from MBS to LSB)
 * - Dup16L - ... low 8-bits
 * - Dup16Count - how many times the 16-bit value should be repeated (for display in 16bpp mode Dup16Count = number of pixels)
 *
 * Output:
 * - none
 */
	while(HSPI_BUSY); //wait for transaction to complete (if any)
	//
	if(Flags & FLG_PULSE_CS_ON_START) WLCD_CS_H; //set #CS to inactive state for a while => reset SPI interface of LCD controller
	//
	//-- put all precomputing here to make #CS inactive pulse longer
	uint8_t Dup16sToHSPIBuf = (Dup16Count>32)?32:Dup16Count; //we have SPI_W0..15 32-bit regs => max 64 bytes => limit to 32 x 16 bits in one HSPI transaction
	uint32_t* HSPIBuf;
	uint8_t Dup16sDone;
	uint32_t STORE_ATTR TwoRevDup16s; //4-bytes aligned (STORE_ATTR, size is multiples of 4)
	//
	if(Dup16sToHSPIBuf){ //there are some data bytes to transmit, do necessary precomputing
		HSPIBuf = (uint32_t*)SPI_W0(HSPI);
		Dup16sDone = 0;
		TwoRevDup16s = (uint16_t)Dup16L<<8 | Dup16H;    //in memory it's stored like this: bit7..0, bit15..8, bit23..16, bit31..24 - in this order it will be transmitted by HSPI
		TwoRevDup16s = TwoRevDup16s | TwoRevDup16s<<16; // ...
	}
	//--
	//
	//send 8-bit Cmd command. Send it separately because we need to manage D/#C => we cannot send everything at once
	if(Flags & FLG_SEND_CMD){
		//
		WLCD_CS_L;
		//
		CLEAR_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_ADDR|SPI_USR_MISO|SPI_USR_MOSI|SPI_USR_DUMMY); //we don't use ADDR, MISO, MOSI, DUMMY parts in the HSPI transaction
		SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_COMMAND); //we use just COMMAND part in the HSPI transaction now (we need to control D/#C)
		//
		WRITE_PERI_REG(SPI_USER2(HSPI),
			(((8-1)&SPI_USR_COMMAND_BITLEN)<<SPI_USR_COMMAND_BITLEN_S) | //number of command bits to transmit (lowest 8-bits from SPI_USER2)
			Cmd); //set lowest 8-bits in SPI_USER2
		//
		WLCD_D_C_L; //command
		HSPI_START; //start transmission of configured transaction - 8-bit Cmd command
	}
	//
	if(!Dup16sToHSPIBuf) return; //there are no data bytes to transmit
	//
	WLCD_CS_L;
	//
	while(HSPI_BUSY); //wait for transaction to complete (if any) (put this as "late" as possible in the program flow of this function - which is here)
	//
	//fill HSPI buffer (SPI_W* registers) as fast as possible - use 32-bit copy
	while(Dup16sDone<Dup16sToHSPIBuf){ //THIS LOOP IS CRITICAL (everything that could be done before or after is excluded from this loop. Also test in loop is intentionally just 8-bit)
		WRITE_PERI_REG(HSPIBuf++, TwoRevDup16s);
		Dup16sDone+=2;
	}
	//
	//configure HSPI transaction and transmit Dup16sToHSPIBuf*16 bits from HSPI buffer and loop without filling the buffer again (it has bee filled already). Loop until all Dup16Count 16-bit data bytes were transmitted
	CLEAR_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_COMMAND|SPI_USR_ADDR|SPI_USR_MISO|SPI_USR_DUMMY); //we don't use COMMAND, ADDR, MISO, DUMMY parts in the HSPI transaction
	SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_MOSI); //we use just MOSI part in the HSPI transaction now
	//
	WLCD_D_C_H; //data
	//
	uint8_t LastDup16sToHSPIBuf = 0;
	while(Dup16sToHSPIBuf){ //THIS LOOP IS CRITICAL (everything that could be done before or after is excluded from this loop. Also tests in loop are intentionally just 8-bit)
		while(HSPI_BUSY); //wait for transaction to complete (if any)
		if(Dup16sToHSPIBuf!=LastDup16sToHSPIBuf) WRITE_PERI_REG(SPI_USER1(HSPI), ((((uint16_t)Dup16sToHSPIBuf*16)-1)&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S); //number of data bits (Dup16sToHSPIBuf*16) to transmit (from SPI_W0..15 32-bit registers => send buffer of max 32*16 = 512 bits = 64 bytes)
		HSPI_START; //start transmission of configured transaction. Now the HSPI will start to shift out all data bytes
		//
		Dup16Count -= Dup16sToHSPIBuf;
		LastDup16sToHSPIBuf = Dup16sToHSPIBuf;
		Dup16sToHSPIBuf = (Dup16Count>32)?32:Dup16Count; //we have SPI_W0..15 32-bit regs => max 64 bytes => limit to 32 x 16 bits in one HSPI transaction
	}
}

void /*ICACHE_FLASH_ATTR*/ wlcd_write_dup24_hspi(uint8_t Flags, uint8_t Cmd, uint8_t Dup24H, uint8_t Dup24M, uint8_t Dup24L, uint32_t Dup24Count){
/* Transmits Cmd byte (if requested by Flags) followed by 24-bit data bytes (Dup16H<<16 | Dup16M<<8 | Dup16L)
 * repeated Dup24Count times to LCD controller.
 * This is especially used for filling larger areas with one color in 18bpp (one pixel takes transmission
 * of 24 bits, Dup24Count = number of pixels), but also drawing RLE compressed WLCD image uses this
 * (no intermediate buffer for uncompressed image).
 * Relevant only when WLCD_USE_HSPI is defined.
 *
 * MAXIMUM EFFORT WAS TAKEN TO MAKE THIS AS FAST AS POSSIBLE
 *
 * Input:
 * - Flags - combination of FLG_* (see definition for details), relevant are only:
 *   - FLG_PULSE_CS_ON_START
 *   - FLG_SEND_CMD
 * - Cmd - 8-bit command for the LCD controller (relevant only if FLG_SEND_CMD flag set)
 * - Dup24H - bits [23:16] of 24-bit data (transmission order on MOSI line: first Dup24H then Dup24M then Dup24L, all from MBS to LSB)
 * - Dup24M - ... bits [15:8]
 * - Dup24L - ... bits [7:0]
 * - Dup24Count - how many times the 24-bit value should be repeated (for display in 18bpp mode Dup24Count = number of pixels)
 *
 * Output:
 * - none
 *
 * ============================================================================
 *
 * Usage of prefilled SPI_W0..15 32-bit regs / explanation of b32A, b32B, b32C and why we limit to 20 x 24 bits in one HSPI transaction:
 *
 * - we need 3 x 32-bit register to completely rotate
 *   |--------------|
 *    RGBR GBRG BRGB RGBR GBRG BRGB ... (in this context, R = Dup24H, G = Dup24M, B = Dup24L)
 *    b32A b32B b32C b32A b32B b32C ...
 *
 * - one complete rotation holds 4 x 24-bit value (4 pixels)
 *
 * - we have 16 x 32-bit SPI_W* registers => we can have 16 x 32 = 512 bits => we can have 512 / 24 = 21.3 24-bit value,
 *   but we need to limit for complete rotation to make SPI_W* filling process simple
 *  |------------ 16 x 32-bit register -------------|
 *   32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32
 *  |  4vals |  4vals |  4vals |  4vals |  4vals |
 *
 *  => we limit to 20 x 24 bits in one HSPI transaction
 *
 * ============================================================================
 */
	while(HSPI_BUSY); //wait for transaction to complete (if any)
	//
	if(Flags & FLG_PULSE_CS_ON_START) WLCD_CS_H; //set #CS to inactive state for a while => reset SPI interface of LCD controller
	//
	//-- put all precomputing here to make #CS inactive pulse longer
	uint8_t Dup24sToHSPIBuf = (Dup24Count>20)?20:Dup24Count; //limit to 20 x 24 bits in one HSPI transaction - see explanation above
	uint32_t* HSPIBuf;
	uint8_t Dup24sDone;
	uint32_t STORE_ATTR b32A; //4-bytes aligned (STORE_ATTR, size is multiples of 4)
	uint32_t STORE_ATTR b32B; // ...
	uint32_t STORE_ATTR b32C; // ...
	//
	if(Dup24sToHSPIBuf){ //there are some data bytes to transmit, do necessary precomputing
		HSPIBuf = (uint32_t*)SPI_W0(HSPI);
		Dup24sDone = 0;
		b32A = ((uint32_t)Dup24H << 24) | ((uint32_t)Dup24L << 16) | ((uint32_t)Dup24M << 8) | Dup24H; //in memory it's stored like this: bit7..0, bit15..8, bit23..16, bit31..24 - in this order it will be transmitted by HSPI
		b32B = ((uint32_t)Dup24M << 24) | ((uint32_t)Dup24H << 16) | ((uint32_t)Dup24L << 8) | Dup24M; // ...
		b32C = ((uint32_t)Dup24L << 24) | ((uint32_t)Dup24M << 16) | ((uint32_t)Dup24H << 8) | Dup24L; // ...
	}
	//--
	//
	//send 8-bit Cmd command. Send it separately because we need to manage D/#C => we cannot send everything at once
	if(Flags & FLG_SEND_CMD){
		//
		WLCD_CS_L;
		//
		CLEAR_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_ADDR|SPI_USR_MISO|SPI_USR_MOSI|SPI_USR_DUMMY); //we don't use ADDR, MISO, MOSI, DUMMY parts in the HSPI transaction
		SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_COMMAND); //we use just COMMAND part in the HSPI transaction now (we need to control D/#C)
		//
		WRITE_PERI_REG(SPI_USER2(HSPI),
			(((8-1)&SPI_USR_COMMAND_BITLEN)<<SPI_USR_COMMAND_BITLEN_S) | //number of command bits to transmit (lowest 8-bits from SPI_USER2)
			Cmd); //set lowest 8-bits in SPI_USER2
		//
		WLCD_D_C_L; //command
		HSPI_START; //start transmission of configured transaction - 8-bit Cmd command
	}
	//
	if(!Dup24sToHSPIBuf) return; //there are no data bytes to transmit
	//
	WLCD_CS_L;
	//
	while(HSPI_BUSY); //wait for transaction to complete (if any) (put this as "late" as possible in the program flow of this function - which is here)
	//
	//fill HSPI buffer (SPI_W* registers) as fast as possible - use 32-bit copy
	while(Dup24sDone<Dup24sToHSPIBuf){ //THIS LOOP IS CRITICAL (everything that could be done before or after is excluded from this loop. Also test in loop is intentionally just 8-bit)
		WRITE_PERI_REG(HSPIBuf++, b32A);
		WRITE_PERI_REG(HSPIBuf++, b32B);
		WRITE_PERI_REG(HSPIBuf++, b32C);
		Dup24sDone+=4;
	}
	//
	//configure HSPI transaction and transmit Dup24sToHSPIBuf*24 bits from HSPI buffer and loop without filling the buffer again (it has bee filled already). Loop until all Dup24Count 24-bit data bytes were transmitted
	CLEAR_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_COMMAND|SPI_USR_ADDR|SPI_USR_MISO|SPI_USR_DUMMY); //we don't use COMMAND, ADDR, MISO, DUMMY parts in the HSPI transaction
	SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_MOSI); //we use just MOSI part in the HSPI transaction now
	//
	WLCD_D_C_H; //data
	//
	uint8_t LastDup24sToHSPIBuf = 0;
	while(Dup24sToHSPIBuf){ //THIS LOOP IS CRITICAL (everything that could be done before or after is excluded from this loop. Also tests in loop are intentionally just 8-bit)
		while(HSPI_BUSY); //wait for transaction to complete (if any)
		if(Dup24sToHSPIBuf!=LastDup24sToHSPIBuf) WRITE_PERI_REG(SPI_USER1(HSPI), ((((uint16_t)Dup24sToHSPIBuf*24)-1)&SPI_USR_MOSI_BITLEN)<<SPI_USR_MOSI_BITLEN_S); //number of data bits (Dup24sToHSPIBuf*24) to transmit (from SPI_W0..15 32-bit registers => send buffer of max 20*24 = 480 bits = 60 bytes)
		HSPI_START; //start transmission of configured transaction. Now the HSPI will start to shift out all data bytes
		//
		Dup24Count -= Dup24sToHSPIBuf;
		LastDup24sToHSPIBuf = Dup24sToHSPIBuf;
		Dup24sToHSPIBuf = (Dup24Count>20)?20:Dup24Count; //limit to 20 x 24 bits in one HSPI transaction - see explanation above
	}
}

uint8_t /*ICACHE_FLASH_ATTR*/ wlcd_read_buf_hspibuf64(uint8_t Flags, uint8_t Cmd, uint8_t DummyCLKCnt, uint32_t* Buf, uint32_t Bytes, uint8_t* RetR5G6B5PixelsRead){
/* Transmits Cmd byte (if requested by Flags) and reads Bytes data bytes (max 64) from LCD controller to Buf.
 * This function uses 32-bit copy from SPI_W0..15 -> Buf to be AS FAST AS POSSIBLE.
 * We're using entire HSPI buffer (SPI_W0..15) = 16 x 32-bit => max 64 data bytes can be read in one call.
 * If FLG_DO_SHRINK_R6G6B6_TO_R5G6B5 flag is set, max read bytes from LCD controller is 60 to keep everything
 * aligned (offset and pixels) and the read data are converted from R6G6B6 pixels (read from LCD as 3 bytes)
 * down to R5G6B5 pixels (2 bytes). This means that we can read max 20 R5G6B5 pixels into Buf in one call.
 * Relevant only when WLCD_USE_HSPI is defined.
 *
 * Input:
 * - Flags - combination of FLG_* (see definition for details), relevant are only:
 *   - FLG_PULSE_CS_ON_START
 *   - FLG_SEND_CMD
 *   - FLG_DO_SHRINK_R6G6B6_TO_R5G6B5
 * - Cmd - 8-bit command for the LCD controller (relevant only if FLG_SEND_CMD flag set)
 * - DummyCLKCnt - number of dummy clocks before data bytes are read (before MISO part of transaction)
 * - Buf - pointer to buffer where data bytes are received - must be 4-bytes aligned, allocated memory length must be multiples of 4
 * - Bytes - number of data bytes to read from LCD controller
 *
 * Output:
 * - returns number of really read data bytes from LCD controller. 0 means no data bytes were read
 *   (just Cmd transmitted if FLG_SEND_CMD flag set)
 * If FLG_DO_SHRINK_R6G6B6_TO_R5G6B5 flag is NOT set:
 *   - read is limited to 64 bytes. If you get less than 64, the value will be equal to Bytes and it means
 *     that all data bytes were read
 *   - RetR5G6B5PixelsRead is not touched
 * If FLG_DO_SHRINK_R6G6B6_TO_R5G6B5 flag is set:
 *   - read is limited to 60 bytes. If you get less than 60, the value will be equal to Bytes and it means
 *     that all data bytes were read
 *   - RetR5G6B5PixelsRead is filled by number of complete R5G6B5 pixels filled into Buf
 */
	while(HSPI_BUSY); //wait for transaction to complete (if any)
	//
	if(Flags & FLG_PULSE_CS_ON_START) WLCD_CS_H; //set #CS to inactive state for a while => reset SPI interface of LCD controller
	//
	//-- put all precomputing here to make #CS inactive pulse longer
	uint8_t BytesToHSPIBuf;
	if(Flags & FLG_DO_SHRINK_R6G6B6_TO_R5G6B5) BytesToHSPIBuf = (Bytes>60)?60:Bytes; //we have SPI_W0..15 32-bit regs => max 64 bytes, BUT to have everything aligned (offset and pixels) in case of multiple calls, we allow max 60 (60 / 4 = 15; 60 / 3 = 20)
	else BytesToHSPIBuf = (Bytes>64)?64:Bytes; //we have SPI_W0..15 32-bit regs => max 64 bytes
	uint32_t* HSPIBuf;
	uint8_t BytesDone;
	//
	if(BytesToHSPIBuf){ //there are some data bytes to transmit, do necessary precomputing
		HSPIBuf = (uint32_t*)SPI_W0(HSPI);
		BytesDone = 0;
	}
	//--
	//
	//send 8-bit Cmd command. Send it separately because we need to manage D/#C => we cannot send everything at once
	if(Flags & FLG_SEND_CMD){
		//
		WLCD_CS_L;
		//
		CLEAR_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_ADDR|SPI_USR_MISO|SPI_USR_MOSI|SPI_USR_DUMMY); //we don't use ADDR, MISO, MOSI, DUMMY parts in the HSPI transaction
		SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_COMMAND); //we use just COMMAND part in the HSPI transaction now (we need to control D/#C)
		//
		WRITE_PERI_REG(SPI_USER2(HSPI),
			(((8-1)&SPI_USR_COMMAND_BITLEN)<<SPI_USR_COMMAND_BITLEN_S) | //number of command bits to transmit (lowest 8-bits from SPI_USER2)
			Cmd); //set lowest 8-bits in SPI_USER2
		//
		WLCD_D_C_L; //command
		HSPI_START; //start transmission of configured transaction - 8-bit Cmd command
	}
	//
	if(!BytesToHSPIBuf) return 0; //there are no data bytes to transmit
	//
	WLCD_CS_L;
	//
	while(HSPI_BUSY); //wait for transaction to complete (if any) (put this as "late" as possible in the program flow of this function - which is here)
	//
	//configure HSPI transaction and start reception
	CLEAR_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_COMMAND|SPI_USR_ADDR|SPI_USR_MOSI|SPI_USR_DUMMY); //we don't use COMMAND, ADDR, MOSI, DUMMY parts in the HSPI transaction
	SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_MISO); //we use just MISO part in the HSPI transaction now
	//
	if(DummyCLKCnt==0){
		WRITE_PERI_REG(SPI_USER1(HSPI), ((((uint16_t)BytesToHSPIBuf*8)-1)&SPI_USR_MISO_BITLEN)<<SPI_USR_MISO_BITLEN_S); //number of data bits (BytesToHSPIBuf*8) to receive (from SPI_W0..15 32-bit registers => send buffer of max 64 bytes)
	}
	else{
		SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_DUMMY); //we use also DUMMY part in the HSPI transaction now
		WRITE_PERI_REG(SPI_USER1(HSPI),
			(((((uint16_t)BytesToHSPIBuf*8)-1)&SPI_USR_MISO_BITLEN)<<SPI_USR_MISO_BITLEN_S) | //number of data bits (BytesToHSPIBuf*8) to receive (to SPI_W0..15 32-bit registers => receive buffer of max 64 bytes)
			(((DummyCLKCnt-1)&SPI_USR_DUMMY_CYCLELEN)<<SPI_USR_DUMMY_CYCLELEN_S) //number of dummy CLK cycles (DummyCLKCnt) to transmit before MISO
			);
	}
	//
	WLCD_D_C_H; //data
	HSPI_START; //start reception of configured transaction. Now the HSPI will start to shift in all data bytes
	//
	while(HSPI_BUSY); //wait for transaction to complete (if any)
	//
	//transfer HSPI buffer into Buf
	if(Flags & FLG_DO_SHRINK_R6G6B6_TO_R5G6B5){
		//convert R6G6B6 or R8G8B8 pixels in HSPI buffer into R5G6B5 pixels and store into Buf, fill RetR5G6B5PixelsRead
		*RetR5G6B5PixelsRead = 0;
		uint8_t ShrinkCtr = 0;
		uint16_t R5G6B5;
		uint8_t* HSPIBufBytePtr = (uint8_t*)HSPIBuf;
		uint8_t* BufBytePtr = (uint8_t*)Buf;
		while(BytesDone<BytesToHSPIBuf){
			switch(ShrinkCtr){
				case 0:
					R5G6B5 = *(HSPIBufBytePtr++)>>3;
					ShrinkCtr++;
					break;
				case 1:
					R5G6B5 <<= 6;
					R5G6B5 |= *(HSPIBufBytePtr++)>>2;
					ShrinkCtr++;
					break;
				case 2:
					R5G6B5 <<= 5;
					R5G6B5 |= *(HSPIBufBytePtr++)>>3;
					ShrinkCtr=0;
					*(BufBytePtr++) = R5G6B5>>8;
					*(BufBytePtr++) = R5G6B5;
					(*RetR5G6B5PixelsRead)++;
					break;
			}
			BytesDone++;
		}
	}
	else{
		//copy HSPI buffer (SPI_W* registers) into Buf as fast as possible - use 32-bit copy, don't touch RetR5G6B5PixelsRead
		while(BytesDone<BytesToHSPIBuf){ //THIS LOOP IS CRITICAL (everything that could be done before or after is excluded from this loop. Also test in loop is intentionally just 8-bit)
			*(Buf++) = READ_PERI_REG(HSPIBuf++);
			BytesDone+=4;
		}
	}
	return BytesToHSPIBuf;
}

//low level communication - SW bit-banging

void /*ICACHE_FLASH_ATTR*/ wlcd_write8_sw(uint8_t Flags, uint8_t Val){
/* Writes 8-bit Val to LCD controller using SW bit-banging.
 * Relevant only when WLCD_USE_HSPI is NOT defined.
 * Use HSPI whenever possible.
 *
 * Input:
 * - Flags - combination of FLG_* (see definition for details), relevant are only:
 *   - FLG_PULSE_CS_ON_START
 *   - FLG_D_C
 * - Val - 8-bit value to write
 *
 * Output:
 * - none
 */
	if(Flags & FLG_PULSE_CS_ON_START) WLCD_CS_H; //set #CS to inactive state for a while => reset SPI interface of LCD controller
	WLCD_D_C(Flags & FLG_D_C);
	WLCD_SCL_L;
	WLCD_CS_L;
	uint8_t Mask;
	for(Mask=0x80;Mask;Mask>>=1){ //THIS LOOP IS CRITICAL (MSB to LSB)
		WLCD_SCL_L;
		DELAY;
		WLCD_SDI(Val & Mask); //mask bit from Val
		WLCD_SCL_H;
		DELAY;
	}
	WLCD_SCL_L;
}

uint8_t /*ICACHE_FLASH_ATTR*/ wlcd_read8_sw(void){
/* Reads 8-bit Val from LCD controller using SW bit-banging.
 * Relevant only when WLCD_USE_HSPI is NOT defined.
 * Use HSPI whenever possible.
 *
 * Input:
 * - none
 *
 * Output:
 * - returns read 8-bit value
 */
	WLCD_SCL_L;
	WLCD_D_C_H;
	WLCD_CS_L;
	uint8_t Ctr;
	uint8_t Val = 0;
	for(Ctr=0;Ctr<8;Ctr++){
		Val<<=1;
		WLCD_SCL_L;
		DELAY;
		Val |= WLCD_SDO;
		WLCD_SCL_H;
		DELAY;
	}
	WLCD_SCL_L;
	return Val;
}

//low level communication - common write / read

uint32_t /*ICACHE_FLASH_ATTR*/ wlcd_write(uint8_t Flags, uint8_t Cmd, uint32_t* Buf, uint32_t Bytes){
/* Transmits Cmd byte (if requested by Flags) followed by Bytes data bytes from Buf to LCD controller.
 * Any number of bytes can be sent in one call of this function (handles subsequent calls of
 * wlcd_write_buf_hspibuf64(...) / wlcd_write8_sw(...)).
 *
 * Input:
 * - Flags - combination of FLG_* (see definition for details), relevant are only:
 *   - FLG_PULSE_CS_ON_START
 *   - FLG_SEND_CMD
 * - Cmd - 8-bit command for the LCD controller (relevant only if FLG_SEND_CMD flag set)
 * - Buf - pointer to data bytes to be transmitted - if WLCD_USE_HSPI is defined, then it must be 4-bytes aligned
 *         and allocated memory length must be multiples of 4
 * - Bytes - number of data bytes to transmit (Cmd is not counted)
 *
 * Output:
 * - returns number of really transmitted data bytes (Cmd is not counted).
 *   0 means no data bytes were transmitted (just Cmd transmitted if FLG_SEND_CMD flag set).
 *
 * For further details in WLCD_USE_HSPI mode see wlcd_write_buf_hspibuf64(...)
 */
#ifdef WLCD_USE_HSPI
	uint32_t BytesDone = 0;
	do{
		uint32_t BytesWr = wlcd_write_buf_hspibuf64(Flags, Cmd, (uint32_t*)&(((uint8_t*)Buf)[BytesDone]), Bytes-BytesDone); //if (Bytes-BytesDone>64) => returns only multiples of 4 - that's why we can index by BytesDone (pointer must be 4-bytes aligned)
		if(!BytesWr) break;
		Flags = 0;
		BytesDone += BytesWr;
	} while(BytesDone<Bytes);
	return BytesDone;
#else
	if(Flags & FLG_SEND_CMD) wlcd_write8_sw(Flags&(~(uint8_t)FLG_D_C), Cmd);
	uint32_t BytesDone = 0;
	while(Bytes--) wlcd_write8_sw(FLG_D_C, ((uint8_t*)Buf)[BytesDone++]);
	return BytesDone;
#endif
}

#define wlcd_cmd_only(Cmd)		wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, NULL, 0)

uint32_t /*ICACHE_FLASH_ATTR*/ wlcd_read(uint8_t Flags, uint8_t Cmd, uint8_t DummyCLKCnt, uint32_t* Buf, uint32_t Bytes){
/* Transmits Cmd byte (if requested by Flags) and reads Bytes data bytes from LCD controller to Buf.
 * Any number of bytes can be read in one call of this function (handles subsequent calls of
 * wlcd_read_buf_hspibuf64(...) / wlcd_read8_sw(...)).
 *
 * Input:
 * - Flags - combination of FLG_* (see definition for details), relevant are only:
 *   - FLG_PULSE_CS_ON_START
 *   - FLG_SEND_CMD
 *   - FLG_DO_SHRINK_R6G6B6_TO_R5G6B5
 * - Cmd - 8-bit command for the LCD controller (relevant only if FLG_SEND_CMD flag set)
 * - DummyCLKCnt - number of dummy clocks before data bytes are read
 * - Buf - pointer to buffer where data bytes are received - if WLCD_USE_HSPI is defined, then it must be 4-bytes aligned
 *         and allocated memory length must be multiples of 4
 * - Bytes - number of data bytes to read
 *
 * Output:
 * If FLG_DO_SHRINK_R6G6B6_TO_R5G6B5 flag is NOT set:
 * - returns number of really read data bytes (how many bytes were read from LCD controller and stored into Buf).
 * If FLG_DO_SHRINK_R6G6B6_TO_R5G6B5 flag is set:
 * - returns number of bytes stored into Buf in a meaning of valid R5G6B5 pixel data.
 *   Returned value / 2 = number of R5G6B5 pixels stored in Buf
 * In both cases: if the function returns 0, it means that no valid data were stored into Buf
 * (just Cmd transmitted if FLG_SEND_CMD flag set).
 */
	uint32_t BytesDone = 0;
#ifdef WLCD_USE_HSPI
	if(Flags & FLG_DO_SHRINK_R6G6B6_TO_R5G6B5){
		uint32_t PixelsDone = 0;
		do{
			uint8_t PixelsRd;
			uint8_t BytesRd = wlcd_read_buf_hspibuf64(Flags, Cmd, DummyCLKCnt, (uint32_t*)&(((uint16_t*)Buf)[PixelsDone]), Bytes-BytesDone, &PixelsRd); //if (Bytes-BytesDone>64) => returns only multiples of 4 - that's why we can index by BytesDone (pointer must be 4-bytes aligned)
			if(!BytesRd) break;
			Flags = FLG_DO_SHRINK_R6G6B6_TO_R5G6B5;
			DummyCLKCnt = 0;
			BytesDone += BytesRd;
			PixelsDone += PixelsRd;
		} while(BytesDone<Bytes);
		BytesDone = PixelsDone*2;
	}
	else{
		do{
			uint8_t BytesRd = wlcd_read_buf_hspibuf64(Flags, Cmd, DummyCLKCnt, (uint32_t*)&(((uint8_t*)Buf)[BytesDone]), Bytes-BytesDone, NULL); //if (Bytes-BytesDone>64) => returns only multiples of 4 - that's why we can index by BytesDone (pointer must be 4-bytes aligned)
			if(!BytesRd) break;
			Flags = 0;
			DummyCLKCnt = 0;
			BytesDone += BytesRd;
		} while(BytesDone<Bytes);
	}
#else
	if(Flags & FLG_SEND_CMD) wlcd_write8_sw(Flags&(~(uint8_t)FLG_D_C), Cmd); //command
	while(DummyCLKCnt--){ //dummy clocks between COMMAND and DATA
		DELAY;
		WLCD_SCL_H;
		DELAY;
		WLCD_SCL_L;
	}
	uint8_t* BufPtr = (uint8_t*)Buf;
	if(Flags & FLG_DO_SHRINK_R6G6B6_TO_R5G6B5){
		uint8_t ShrinkCtr = 0;
		uint16_t R5G6B5;
		while(Bytes--){
			switch(ShrinkCtr){
				case 0:
					R5G6B5 = wlcd_read8_sw()>>3;
					ShrinkCtr++;
					break;
				case 1:
					R5G6B5 <<= 6;
					R5G6B5 |= wlcd_read8_sw()>>2;
					ShrinkCtr++;
					break;
				case 2:
					R5G6B5 <<= 5;
					R5G6B5 |= wlcd_read8_sw()>>3;
					ShrinkCtr=0;
					*(BufPtr++) = R5G6B5>>8;
					*(BufPtr++) = R5G6B5;
					BytesDone+=2;
					break;
			}
		}
	}
	else{
		while(Bytes--){
			*(BufPtr++) = wlcd_read8_sw();
			BytesDone++;
		}
	}
#endif
	return BytesDone;
}

//LCD controller init

int8_t ICACHE_FLASH_ATTR wlcd_init(void){
/* Initializes ESP8266's HSPI/GPIOs for serial communication with LCD controller and initializes LCD controller.
 * Returns 0 if everything ok else return -1.
 */
	DBG_WLCD("wlcd_init(): WLCD version %s\n", WLCD_VERSION);
	//
	if((WLCD_SPI_CLK_PREDIV==0)||(WLCD_SPI_CLK_CNTDIV==0)) return -1;
#ifdef WLCD_USE_HSPI
	DBG_WLCD("wlcd_init(): initializing HSPI/GPIOs for LCD interface (HW mode, HSPI CLK = %d kHz)\n", ((uint32_t)system_get_cpu_freq()*1000) / WLCD_SPI_CLK_PREDIV / WLCD_SPI_CLK_CNTDIV);
	//
	//clear all HSPI registers and settings (except SPI_W0..15 data regs)
	WRITE_PERI_REG(SPI_CMD(HSPI), 0);
	WRITE_PERI_REG(SPI_ADDR(HSPI), 0);
	WRITE_PERI_REG(SPI_CTRL(HSPI), 0);
	WRITE_PERI_REG(SPI_CTRL1(HSPI), 0);
	WRITE_PERI_REG(SPI_RD_STATUS(HSPI), 0);
	WRITE_PERI_REG(SPI_CTRL2(HSPI), 0); //minimize #CS / MOSI / MISO delays - they're set after ESP8266 start-up (realized on oscilloscope - we want to squeeze max. speed out of HSPI and LCD controller is happy about that)
	WRITE_PERI_REG(SPI_CLOCK(HSPI), 0);
	WRITE_PERI_REG(SPI_USER(HSPI), 0);
	WRITE_PERI_REG(SPI_USER1(HSPI), 0);
	WRITE_PERI_REG(SPI_USER2(HSPI), 0);
	WRITE_PERI_REG(SPI_WR_STATUS(HSPI), 0);
	WRITE_PERI_REG(SPI_PIN(HSPI), 0);
	WRITE_PERI_REG(SPI_SLAVE(HSPI), 0);
	WRITE_PERI_REG(SPI_SLAVE1(HSPI), 0);
	WRITE_PERI_REG(SPI_SLAVE2(HSPI), 0);
	WRITE_PERI_REG(SPI_SLAVE3(HSPI), 0);
	WRITE_PERI_REG(SPI_EXT0(HSPI), 0);
	WRITE_PERI_REG(SPI_EXT1(HSPI), 0);
	WRITE_PERI_REG(SPI_EXT2(HSPI), 0);
	WRITE_PERI_REG(SPI_EXT3(HSPI), 0);
	//
	//configure HSPI, set HSPI clock to CPU_CLK_FREQ (80 MHz by default) / (SPI_CLK_PREDIV * SPI_CLK_CNTDIV)
	if((WLCD_SPI_CLK_PREDIV==1)&&(WLCD_SPI_CLK_CNTDIV==1)){ //HSPI clock = CPU_CLK_FREQ
		WRITE_PERI_REG(PERIPHS_IO_MUX_CONF_U, 0x305); //(set bit 9 if HSPI speed should be equal to sysclock (80 MHz))
		WRITE_PERI_REG(SPI_CLOCK(HSPI), SPI_CLK_EQU_SYSCLK);
	}
	else{ //HSPI clock < CPU_CLK_FREQ
		WRITE_PERI_REG(PERIPHS_IO_MUX_CONF_U, 0x105);
		WRITE_PERI_REG(SPI_CLOCK(HSPI),
			(((WLCD_SPI_CLK_PREDIV-1)&SPI_CLKDIV_PRE)<<SPI_CLKDIV_PRE_S)|
			(((WLCD_SPI_CLK_CNTDIV-1)&SPI_CLKCNT_N)<<SPI_CLKCNT_N_S)|
			(((WLCD_SPI_CLK_CNTDIV>>1)&SPI_CLKCNT_H)<<SPI_CLKCNT_H_S)|
			((0&SPI_CLKCNT_L)<<SPI_CLKCNT_L_S)
		);
	}
	//
	//attach HSPI module to ESP8266's output pins
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_HSPID_MOSI); //LCD_SDI
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_HSPIQ_MISO); //LCD_SDO
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_HSPI_CLK); //LCD_SCL
	//
#else
	DBG_WLCD("wlcd_init(): initializing GPIOs for LCD interface (SW mode)\n");
	//
	PIN_FUNC_SELECT(GPIO_PIN_REG(WLCD_GPIO_FOR_LCD_SDI), GPIO_FUNC(WLCD_GPIO_FOR_LCD_SDI)); //LCD_SDI
	GPIO_OUTPUT_SET(WLCD_GPIO_FOR_LCD_SDI, 0); //as output, set L
	//
	GPIO_DIS_OUTPUT(WLCD_GPIO_FOR_LCD_SDO); //as input
	PIN_FUNC_SELECT(GPIO_PIN_REG(WLCD_GPIO_FOR_LCD_SDO), GPIO_FUNC(WLCD_GPIO_FOR_LCD_SDO)); //LCD_SDO
	//
	PIN_FUNC_SELECT(GPIO_PIN_REG(WLCD_GPIO_FOR_LCD_SCL), GPIO_FUNC(WLCD_GPIO_FOR_LCD_SCL)); //LCD_SCL
	GPIO_OUTPUT_SET(WLCD_GPIO_FOR_LCD_SCL, 0); //as output, set L
#endif
	//
	PIN_FUNC_SELECT(GPIO_PIN_REG(WLCD_GPIO_FOR_LCD_CS), GPIO_FUNC(WLCD_GPIO_FOR_LCD_CS)); //LCD_CS
	GPIO_OUTPUT_SET(WLCD_GPIO_FOR_LCD_CS, 1); //as output, set H
	//
	PIN_FUNC_SELECT(GPIO_PIN_REG(WLCD_GPIO_FOR_LCD_D_C), GPIO_FUNC(WLCD_GPIO_FOR_LCD_D_C)); //LCD_D_C
	GPIO_OUTPUT_SET(WLCD_GPIO_FOR_LCD_D_C, 1); //as output, set H
	//
	//init LCD controller
#if(WLCD_DISPLAY==WLCD_ILI9341)
	DBG_WLCD("wlcd_init(): initializing display controller ILI9341\n");
	//
	wlcd_cmd_only(WLCD_SW_RESET);
	vTaskDelay(150 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
	//
	uint8_t* Buf = os_malloc(16);
	if(Buf==NULL){
		DBG_WLCD("wlcd_init(): can't allocate memory for write buffer\n");
		return -1;
	}
	uint8_t Cmd, Idx;
	//
	wlcd_cmd_only(WLCD_DISP_OFF);
	//
	//after frustrating try to init LCD from scratch, the magic init codes was copied from github.com/adafruit/Adafruit_ILI9341 - thanks!
	Idx = 0;
	Cmd = 0xEF;
	Buf[Idx++] = 0x03;
	Buf[Idx++] = 0x80;
	Buf[Idx++] = 0x02;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xCF;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0xC1;
	Buf[Idx++] = 0x30;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xED;
	Buf[Idx++] = 0x64;
	Buf[Idx++] = 0x03;
	Buf[Idx++] = 0x12;
	Buf[Idx++] = 0x81;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xE8;
	Buf[Idx++] = 0x85;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0x78;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xCB;
	Buf[Idx++] = 0x39;
	Buf[Idx++] = 0x2C;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0x34;
	Buf[Idx++] = 0x02;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xF7;
	Buf[Idx++] = 0x20;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xEA;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0x00;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_POWER_CTRL_1;
	Buf[Idx++] = 0x23; //VRH[5:0]
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_POWER_CTRL_2;
	Buf[Idx++] = 0x10; //SAP[2:0];BT[3:0]
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_VCOM_CTRL_1;
	Buf[Idx++] = 0x3E;
	Buf[Idx++] = 0x28;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_VCOM_CTRL_2;
	Buf[Idx++] = 0x86;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Cmd = WLCD_MEMORY_ACCESS_CTRL; //aka MADCTL
	Buf[0] = ((WLCD_ROW_ADDR_ORDER)<<7) |
			 ((WLCD_COL_ADDR_ORDER)<<6) |
			 ((WLCD_ROW_COL_EXCHANGE)<<5) |
			 ((WLCD_VERT_REFRESH_ORD)<<4) |
			 ((WLCD_PANEL_BGR_ORDER)<<3) |
			 ((WLCD_HORIZ_REFRESH_ORD)<<2);
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, 1);
	//
	Idx = 0;
	Cmd = WLCD_PIXEL_FORMAT_SET; //1-byte parameter: bit[6:4] = DPI bpp (not relevant for us); bit[2:0] = DBI bpp, this is how LCD controller will receive pixel data over SPI and expand to GRAM)
	#if(WLCD_BPP==WLCD_16BPP)
		Buf[Idx++] = 0x55; //16bpp DPI, 16bpp DBI
	#else
		Buf[Idx++] = 0x66; //18bpp DPI, 18bpp DBI
	#endif
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_FRAME_CTRL_NORMAL_MODE;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0x18;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_DISPLAY_FUNCTION_CTRL;
	Buf[Idx++] = 0x08;
	Buf[Idx++] = 0x82;
	Buf[Idx++] = 0x27;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xF2;
	Buf[Idx++] = 0x00;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_GAMMA_SET;
	Buf[Idx++] = 0x01;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_POSITIVE_GAMMA_CORR;
	Buf[Idx++] = 0x0F;
	Buf[Idx++] = 0x31;
	Buf[Idx++] = 0x2B;
	Buf[Idx++] = 0x0C;
	Buf[Idx++] = 0x0E;
	Buf[Idx++] = 0x08;
	Buf[Idx++] = 0x4E;
	Buf[Idx++] = 0xF1;
	Buf[Idx++] = 0x37;
	Buf[Idx++] = 0x07;
	Buf[Idx++] = 0x10;
	Buf[Idx++] = 0x03;
	Buf[Idx++] = 0x0E;
	Buf[Idx++] = 0x09;
	Buf[Idx++] = 0x00;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_NEGATIVE_GAMMA_CORR;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0x0E;
	Buf[Idx++] = 0x14;
	Buf[Idx++] = 0x03;
	Buf[Idx++] = 0x11;
	Buf[Idx++] = 0x07;
	Buf[Idx++] = 0x31;
	Buf[Idx++] = 0xC1;
	Buf[Idx++] = 0x48;
	Buf[Idx++] = 0x08;
	Buf[Idx++] = 0x0F;
	Buf[Idx++] = 0x0C;
	Buf[Idx++] = 0x31;
	Buf[Idx++] = 0x36;
	Buf[Idx++] = 0x0F;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	wlcd_cmd_only(WLCD_SLEEP_OUT);
	vTaskDelay(120 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
	//
	wlcd_cmd_only(WLCD_DISP_ON);
	//
	os_free(Buf);
#elif(WLCD_DISPLAY==WLCD_ILI9488_KDv4_HACK)
	DBG_WLCD("wlcd_init(): initializing clone of ILI9488 LCD controller used in KeDei 3.5\" LCD module v4.0 2016/1/1\n");
	//
	wlcd_cmd_only(WLCD_SW_RESET);
	vTaskDelay(150 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
	//
	uint8_t* Buf = os_malloc(16);
	if(Buf==NULL){
		DBG_WLCD("wlcd_init(): can't allocate memory for write buffer\n");
		return -1;
	}
	uint8_t Cmd, Idx;
	//
	wlcd_cmd_only(WLCD_DISP_OFF);
	//
	//this is copied from ILI9341 section above - the init of controllers is almost identical (except some undocumented commands as compared to PDF datasheet for ILI9488)
	Idx = 0;
	Cmd = 0xEF;
	Buf[Idx++] = 0x03;
	Buf[Idx++] = 0x80;
	Buf[Idx++] = 0x02;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xCF;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0xC1;
	Buf[Idx++] = 0x30;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xED;
	Buf[Idx++] = 0x64;
	Buf[Idx++] = 0x03;
	Buf[Idx++] = 0x12;
	Buf[Idx++] = 0x81;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xE8;
	Buf[Idx++] = 0x85;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0x78;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xCB;
	Buf[Idx++] = 0x39;
	Buf[Idx++] = 0x2C;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0x34;
	Buf[Idx++] = 0x02;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xF7;
	Buf[Idx++] = 0x20;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xEA;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0x00;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_POWER_CTRL_1;
	Buf[Idx++] = 0x23; //VRH[5:0]
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_POWER_CTRL_2;
	Buf[Idx++] = 0x10; //SAP[2:0];BT[3:0]
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_VCOM_CTRL_1;
	Buf[Idx++] = 0x3E;
	Buf[Idx++] = 0x28;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	/* this command is missing in ILI9488_Preliminary_DS_V090.pdf so I've commented it out (my KeDei display module is working fine with or without it)
	Idx = 0;
	Cmd = WLCD_VCOM_CTRL_2;
	Buf[Idx++] = 0x86;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	*/
	//
	Cmd = WLCD_MEMORY_ACCESS_CTRL; //aka MADCTL
	Buf[0] = ((WLCD_ROW_ADDR_ORDER)<<7) |
			 ((WLCD_COL_ADDR_ORDER)<<6) |
			 ((WLCD_ROW_COL_EXCHANGE)<<5) |
			 ((WLCD_VERT_REFRESH_ORD)<<4) |
			 ((WLCD_PANEL_BGR_ORDER)<<3) |
			 ((WLCD_HORIZ_REFRESH_ORD)<<2);
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, 1);
	//
	Idx = 0;
	Cmd = WLCD_PIXEL_FORMAT_SET; //1-byte parameter: bit[6:4] = DPI bpp (not relevant for us); bit[2:0] = DBI bpp, this is how LCD controller will receive pixel data over SPI and expand to GRAM)
	#if(WLCD_BPP==WLCD_16BPP)
		Buf[Idx++] = 0x55; //16bpp DPI, 16bpp DBI
	#else
		Buf[Idx++] = 0x66; //18bpp DPI, 18bpp DBI
	#endif
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_FRAME_CTRL_NORMAL_MODE;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0x18;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_DISPLAY_FUNCTION_CTRL;
	Buf[Idx++] = 0x08;
	Buf[Idx++] = 0x82;
	Buf[Idx++] = 0x27;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = 0xF2;
	Buf[Idx++] = 0x00;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	/* this command is missing in ILI9488_Preliminary_DS_V090.pdf so I've commented it out (my KeDei display module is working fine with or without it)
	Idx = 0;
	Cmd = WLCD_GAMMA_SET;
	Buf[Idx++] = 0x01;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	*/
	//
	Idx = 0;
	Cmd = WLCD_POSITIVE_GAMMA_CORR;
	Buf[Idx++] = 0x0F;
	Buf[Idx++] = 0x31;
	Buf[Idx++] = 0x2B;
	Buf[Idx++] = 0x0C;
	Buf[Idx++] = 0x0E;
	Buf[Idx++] = 0x08;
	Buf[Idx++] = 0x4E;
	Buf[Idx++] = 0xF1;
	Buf[Idx++] = 0x37;
	Buf[Idx++] = 0x07;
	Buf[Idx++] = 0x10;
	Buf[Idx++] = 0x03;
	Buf[Idx++] = 0x0E;
	Buf[Idx++] = 0x09;
	Buf[Idx++] = 0x00;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_NEGATIVE_GAMMA_CORR;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0x0E;
	Buf[Idx++] = 0x14;
	Buf[Idx++] = 0x03;
	Buf[Idx++] = 0x11;
	Buf[Idx++] = 0x07;
	Buf[Idx++] = 0x31;
	Buf[Idx++] = 0xC1;
	Buf[Idx++] = 0x48;
	Buf[Idx++] = 0x08;
	Buf[Idx++] = 0x0F;
	Buf[Idx++] = 0x0C;
	Buf[Idx++] = 0x31;
	Buf[Idx++] = 0x36;
	Buf[Idx++] = 0x0F;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	wlcd_cmd_only(WLCD_SLEEP_OUT);
	vTaskDelay(120 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
	//
	wlcd_cmd_only(WLCD_DISP_ON);
	//
	os_free(Buf);
#elif(WLCD_DISPLAY==WLCD_ILI9488)
	DBG_WLCD("wlcd_init(): initializing display controller ILI9488\n");
	//
	wlcd_cmd_only(WLCD_SW_RESET);
	vTaskDelay(150 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
	//
	uint8_t* Buf = os_malloc(16);
	if(Buf==NULL){
		DBG_WLCD("wlcd_init(): can't allocate memory for write buffer\n");
		return -1;
	}
	uint8_t Cmd, Idx;
	//
	wlcd_cmd_only(WLCD_DISP_OFF);
	//
	//created from scratch using PDF datasheet for ILI9488 / partially inspired by Adafruit library found over the internet
	//most of this code is just about setting default values, which should be already there after reset anyway, but we need to be sure
	Idx = 0;
	Cmd = WLCD_RGB_INTERFACE_SIG_CTRL;
	Buf[Idx++] = 0x00; //default (as on power on)
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_FRAME_CTRL_NORMAL_MODE;
	Buf[Idx++] = /*0xB0*/ 0xC0; //0xB0 = default (as on power on), frame rate 68.36 Hz; 0xC0 = frame rate 78.13 Hz
	Buf[Idx++] = 0x11;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_DISP_INVERSION_CTRL;
	Buf[Idx++] = 0x02; //default (as on power on)
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_DISPLAY_FUNCTION_CTRL;
	Buf[Idx++] = 0x02; //default (as on power on)
	Buf[Idx++] = 0x02;
	Buf[Idx++] = 0x3B;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_ENTRY_MODE_SET;
	Buf[Idx++] = 0x06 /*0xC6*/; //0x06 = default (as on power on)
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_POWER_CTRL_1;
	Buf[Idx++] = 0x0E /*0x17*/; //0x0E = default (as on power on)
	Buf[Idx++] = 0x0E /*0x15*/; // ...
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_POWER_CTRL_2;
	Buf[Idx++] = /*0x43*/ 0x46; //0x43 = default (as on power on) = DDVDH = VCI x 2, DDVDL = -(VCI x 2), VCL = -VCI, VGH = VCI x 5, VGL = -VCI x 5; DDVDH should be <= 6.0 V; VGH – VGL should be < 32 V
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_POWER_CTRL_3;
	Buf[Idx++] = 0x33; //default (as on power on)
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_VCOM_CTRL_1;
	Buf[Idx++] = 0x00; //default (as on power on)
	Buf[Idx++] = 0x04;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0x04;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Cmd = WLCD_MEMORY_ACCESS_CTRL; //aka MADCTL
	Buf[0] = ((WLCD_ROW_ADDR_ORDER)<<7) |
			 ((WLCD_COL_ADDR_ORDER)<<6) |
			 ((WLCD_ROW_COL_EXCHANGE)<<5) |
			 ((WLCD_VERT_REFRESH_ORD)<<4) |
			 ((WLCD_PANEL_BGR_ORDER)<<3) |
			 ((WLCD_HORIZ_REFRESH_ORD)<<2);
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, 1);
	//
	Idx = 0;
	Cmd = WLCD_PIXEL_FORMAT_SET; //1-byte parameter: bit[6:4] = DPI bpp (not relevant for us); bit[2:0] = DBI bpp, this is how LCD controller will receive pixel data over SPI and expand to GRAM)
	#if(WLCD_BPP==WLCD_16BPP)
		Buf[Idx++] = 0x55; //16bpp DPI, 16bpp DBI
	#else
		Buf[Idx++] = 0x66; //18bpp DPI, 18bpp DBI
	#endif
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	/* this produced worse colors than without it (at least on tested LCD module ER-TFT035-6)
	Idx = 0;
	Cmd = WLCD_POSITIVE_GAMMA_CORR;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0x03;
	Buf[Idx++] = 0x09;
	Buf[Idx++] = 0x08;
	Buf[Idx++] = 0x16;
	Buf[Idx++] = 0x0A;
	Buf[Idx++] = 0x3F;
	Buf[Idx++] = 0x78;
	Buf[Idx++] = 0x4C;
	Buf[Idx++] = 0x09;
	Buf[Idx++] = 0x0A;
	Buf[Idx++] = 0x08;
	Buf[Idx++] = 0x16;
	Buf[Idx++] = 0x1A;
	Buf[Idx++] = 0x0F;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_NEGATIVE_GAMMA_CORR;
	Buf[Idx++] = 0x00;
	Buf[Idx++] = 0X16;
	Buf[Idx++] = 0X19;
	Buf[Idx++] = 0x03;
	Buf[Idx++] = 0x0F;
	Buf[Idx++] = 0x05;
	Buf[Idx++] = 0x32;
	Buf[Idx++] = 0x45;
	Buf[Idx++] = 0x46;
	Buf[Idx++] = 0x04;
	Buf[Idx++] = 0x0E;
	Buf[Idx++] = 0x0D;
	Buf[Idx++] = 0x35;
	Buf[Idx++] = 0x37;
	Buf[Idx++] = 0x0F;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	*/
	//
	Idx = 0;
	Cmd = WLCD_SET_IMAGE_FUNCTION;
	Buf[Idx++] = 0x00; //default (as on power on)
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	/* brightness has no noticeable effect, ADJUST_CTRL_3 would be just about making sure that bit DSI_18_option is set (default state). We don't use DSI => commented out
	Idx = 0;
	Cmd = WLCD_WRITE_DISP_BRIGHTNESS;
	Buf[Idx++] = 0xFF;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_ADJUST_CTRL_3;
	Buf[Idx++] = 0xA9; //default (as on power on)
	Buf[Idx++] = 0x51;
	Buf[Idx++] = 0x2C;
	Buf[Idx++] = 0x82;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	*/
	//
	wlcd_cmd_only(WLCD_SLEEP_OUT);
	vTaskDelay(120 / portTICK_RATE_MS); //([ms] / portTICK_RATE_MS)
	//
	wlcd_cmd_only(WLCD_DISP_ON);
	//
	os_free(Buf);
#endif
	//
#ifdef WLCD_DO_DEBUG
	#ifndef WLCD_NO_READ
		uint32_t STORE_ATTR Val32;
		wlcd_read(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, WLCD_READ_DISP_STATUS, 1, &Val32, 4);
		DBG_WLCD("wlcd_init(): display status (hex): %02X %02X %02X %02X\n", ((uint8_t*)&Val32)[0], ((uint8_t*)&Val32)[1], ((uint8_t*)&Val32)[2], ((uint8_t*)&Val32)[3]);
	#endif
#endif
	//
	return 0;
}

//high level functions - common

char* ICACHE_FLASH_ATTR wlcd_get_version(void){
	return WLCD_VERSION;
}

void ICACHE_FLASH_ATTR wlcd_set_drawing_rect(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height){
/* Sets LCD controller drawing area and sets GRAM pointer position to top left corner of that area.
 * This area will be filled by incoming pixel data / will provide pixel data for outgoing pixel data.
 */
	//
	uint8_t STORE_ATTR Buf[4]; //4-bytes aligned (STORE_ATTR, size is multiples of 4)
	//
	uint8_t Idx = 0;
	uint8_t Cmd = WLCD_SET_COLUMN_ADDR_RANGE;
	Buf[Idx++] = X>>8;
	Buf[Idx++] = X;
	uint16_t Tmp = X+(Width-1);
	Buf[Idx++] = Tmp>>8;
	Buf[Idx++] = Tmp;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
	//
	Idx = 0;
	Cmd = WLCD_SET_ROW_ADDR_RANGE;
	Buf[Idx++] = Y>>8;
	Buf[Idx++] = Y;
	Tmp = Y+(Height-1);
	Buf[Idx++] = Tmp>>8;
	Buf[Idx++] = Tmp;
	wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, Cmd, (uint32_t*)Buf, Idx);
}

void ICACHE_FLASH_ATTR wlcd_write_dup_color(uint32_t Color, uint32_t PixelCount){
/* Fills LCD controller's GRAM by pixels of Color color as fast as possible.
 * Filling starts at actual GRAM pointer position / where the last write ended.
 * Fill order is always from left to right, from top to bottom and if overflow occurs, it rotates
 * back to start position (some LCD controllers have this overflow rotation feature configurable).
 */
#ifdef WLCD_USE_HSPI
	#if(WLCD_BPP==WLCD_16BPP)
		wlcd_write_dup16_hspi(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, WLCD_WRITE_DDRAM,
			Color>>8, Color,
			PixelCount);
	#else
		wlcd_write_dup24_hspi(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, WLCD_WRITE_DDRAM,
			((wlcd_R8G8B8_struct*)((uint8_t*)&Color))->R, ((wlcd_R8G8B8_struct*)((uint8_t*)&Color))->G, ((wlcd_R8G8B8_struct*)((uint8_t*)&Color))->B,
			PixelCount);
	#endif
#else
	#if(WLCD_BPP==WLCD_16BPP)
		if(PixelCount==0) return;
		uint8_t R5G6B5_H = Color>>8;
		uint8_t R5G6B5_L = Color;
		wlcd_write8_sw(FLG_PULSE_CS_ON_START, WLCD_WRITE_DDRAM); //command
		while(PixelCount-->0){ //THIS LOOP IS CRITICAL
			wlcd_write8_sw(FLG_D_C, R5G6B5_H); //data - bit[15:8] of Color
			wlcd_write8_sw(FLG_D_C, R5G6B5_L); //data - bit[7:0] of Color
		}
	#else
		if(PixelCount==0) return;
		uint8_t R = ((wlcd_R8G8B8_struct*)((uint8_t*)&Color))->R;
		uint8_t G = ((wlcd_R8G8B8_struct*)((uint8_t*)&Color))->G;
		uint8_t B = ((wlcd_R8G8B8_struct*)((uint8_t*)&Color))->B;
		wlcd_write8_sw(FLG_PULSE_CS_ON_START, WLCD_WRITE_DDRAM); //command
		while(PixelCount-->0){ //THIS LOOP IS CRITICAL
			wlcd_write8_sw(FLG_D_C, R); //data - bit[23:16] of Color
			wlcd_write8_sw(FLG_D_C, G); //data - bit[15:8] of Color
			wlcd_write8_sw(FLG_D_C, B); //data - bit[7:0] of Color
		}
	#endif
#endif
}

#define wlcd_rect_fill(X, Y, Width, Height, Color)	do { \
	wlcd_set_drawing_rect((X), (Y), (Width), (Height)); \
	wlcd_write_dup_color((Color), (uint32_t)(Width)*(Height)); \
} while (0)

#define wlcd_pixel_draw(X, Y, Color)		wlcd_rect_fill((X), (Y), 1, 1, (Color))	//! for continuous stream of pixels use wlcd_write_dup_color(...) / wlcd_rect_fill(...) / wlcd_img_draw(...) which are optimized for speed

void ICACHE_FLASH_ATTR wlcd_line_draw(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint32_t Color){
/* Draws line from X1,Y1 to X2,Y2.
 * OPTIMIZED FOR SPEED (addition of fixed point with precalculations, repeating of preconfigured HSPI transaction, ...).
 * The speed-up is the more significant, the more the line is horizontal or vertical. Diagonal lines are
 * the slowest (there is no other way than to re-set drawing point for every pixel that's not in the same row/column).
 */
#if(WLCD_BPP==WLCD_16BPP)
	uint8_t R5G6B5_H = Color>>8;
	uint8_t R5G6B5_L = Color;
#else
	uint8_t R = ((wlcd_R8G8B8_struct*)((uint8_t*)&Color))->R;
	uint8_t G = ((wlcd_R8G8B8_struct*)((uint8_t*)&Color))->G;
	uint8_t B = ((wlcd_R8G8B8_struct*)((uint8_t*)&Color))->B;
#endif
	int32_t ShortLen = Y2-Y1;
	int32_t LongLen = X2-X1;
	uint8_t YLonger = 0;
	if(abs(ShortLen)>abs(LongLen)){
		int32_t Swap32 = ShortLen;
		ShortLen = LongLen;
		LongLen = Swap32;
		YLonger = 1;
	}
	//now if YLonger is true => line is more vertical than horizontal
	//if YLonger is false => line is more horizontal than vertical
	//
	//for YLonger is true: assure drawing always from top to bottom (then speed-up using ULTRAFAST repeat of one pixel write is possible)
	//for YLonger is false: assure drawing always from left to right ...
	if(LongLen<0){
		int16_t Swap16 = X1;
		X1 = X2;
		X2 = Swap16;
		Swap16 = Y1;
		Y1 = Y2;
		Y2 = Swap16;
		ShortLen *= -1;
		LongLen *= -1;
	}
	//
	//compute AddBig
	int32_t AddBig = (LongLen==0) ? 0 : (ShortLen << 16) / LongLen;
	//
	//draw pixels
	uint8_t Flags = FLG_SEND_CMD|FLG_PULSE_CS_ON_START;
	if(YLonger){ //line is more vertical than horizontal
		LongLen += Y1;
		int32_t OldX = -1000888;
		int32_t XBig = ((int32_t)X1<<16)+0x8000;
		for(;Y1<=LongLen;++Y1){
			int32_t X = XBig >> 16;
			//
			//draw pixel to X,Y1
			if(X!=OldX){ //we must reconfigure drawing area (where the pixel will be drawn)
				wlcd_set_drawing_rect(X,Y1, 1,WLCD_HEIGHT-Y1); //configure up to bottom edge of display (we'll use this to dramatically speed the things up when line is near to vertical (when X doesn't change for many Y iterations))
				OldX = X;
				Flags = FLG_SEND_CMD|FLG_PULSE_CS_ON_START;
			}
			if(Flags){
#ifdef WLCD_USE_HSPI
	#if(WLCD_BPP==WLCD_16BPP)
				wlcd_write_dup16_hspi(Flags, WLCD_WRITE_DDRAM, R5G6B5_H, R5G6B5_L, 1);
	#else
				wlcd_write_dup24_hspi(Flags, WLCD_WRITE_DDRAM, R, G, B, 1);
	#endif
#else
	#if(WLCD_BPP==WLCD_16BPP)
				wlcd_write8_sw(FLG_PULSE_CS_ON_START, WLCD_WRITE_DDRAM); //command
				wlcd_write8_sw(FLG_D_C, R5G6B5_H); //data - bit[15:8] of Color
				wlcd_write8_sw(FLG_D_C, R5G6B5_L); //data - bit[7:0] of Color
	#else
				wlcd_write8_sw(FLG_PULSE_CS_ON_START, WLCD_WRITE_DDRAM); //command
				wlcd_write8_sw(FLG_D_C, R); //data - bit[23:16] of Color
				wlcd_write8_sw(FLG_D_C, G); //data - bit[15:8] of Color
				wlcd_write8_sw(FLG_D_C, B); //data - bit[7:0] of Color
	#endif
#endif
			}
			else{ //repeat of one pixel write is possible, because everything for HSPI transfer is set from previous call of wlcd_write_dup*_hspi(...) => we do as less as possible in HSPI mode ;)
#ifdef WLCD_USE_HSPI
				while(HSPI_BUSY); //wait for transaction to complete (if any)
				HSPI_START; //start transmission of configured transaction
#else
	#if(WLCD_BPP==WLCD_16BPP)
				wlcd_write8_sw(FLG_D_C, R5G6B5_H); //data - bit[15:8] of Color
				wlcd_write8_sw(FLG_D_C, R5G6B5_L); //data - bit[7:0] of Color
	#else
				wlcd_write8_sw(FLG_D_C, R); //data - bit[23:16] of Color
				wlcd_write8_sw(FLG_D_C, G); //data - bit[15:8] of Color
				wlcd_write8_sw(FLG_D_C, B); //data - bit[7:0] of Color
	#endif
#endif
			}
			Flags = 0;
			XBig += AddBig;
		}
		return;
	}
	//if we're here, line is more horizontal than vertical
	//
	LongLen += X1;
	int32_t OldY = -1000888;
	int32_t YBig = ((int32_t)Y1<<16)+0x8000;
	for(;X1<=LongLen;++X1){
		int32_t Y = YBig >> 16;
		//
		//draw pixel to X1,Y
		if(Y!=OldY){ //we must reconfigure drawing area (where the pixel will be drawn)
			wlcd_set_drawing_rect(X1,Y, WLCD_WIDTH-X1,1); //configure up to right edge of display (we'll use this to dramatically speed the things up when line is near to horizontal (when Y doesn't change for many X iterations))
			OldY = Y;
			Flags = FLG_SEND_CMD|FLG_PULSE_CS_ON_START;
		}
		if(Flags){
#ifdef WLCD_USE_HSPI
	#if(WLCD_BPP==WLCD_16BPP)
			wlcd_write_dup16_hspi(Flags, WLCD_WRITE_DDRAM, R5G6B5_H, R5G6B5_L, 1);
	#else
			wlcd_write_dup24_hspi(Flags, WLCD_WRITE_DDRAM, R, G, B, 1);
	#endif
#else
	#if(WLCD_BPP==WLCD_16BPP)
			wlcd_write8_sw(FLG_PULSE_CS_ON_START, WLCD_WRITE_DDRAM); //command
			wlcd_write8_sw(FLG_D_C, R5G6B5_H); //data - bit[15:8] of Color
			wlcd_write8_sw(FLG_D_C, R5G6B5_L); //data - bit[7:0] of Color
	#else
			wlcd_write8_sw(FLG_PULSE_CS_ON_START, WLCD_WRITE_DDRAM); //command
			wlcd_write8_sw(FLG_D_C, R); //data - bit[23:16] of Color
			wlcd_write8_sw(FLG_D_C, G); //data - bit[15:8] of Color
			wlcd_write8_sw(FLG_D_C, B); //data - bit[7:0] of Color
	#endif
#endif
		}
		else{ //repeat of one pixel write is possible, because everything for HSPI transfer is set from previous call of wlcd_write_dup*_hspi(...) => we do as less as possible in HSPI mode ;)
#ifdef WLCD_USE_HSPI
			while(HSPI_BUSY); //wait for transaction to complete (if any)
			HSPI_START; //start transmission of configured transaction
#else
	#if(WLCD_BPP==WLCD_16BPP)
			wlcd_write8_sw(FLG_D_C, R5G6B5_H); //data - bit[15:8] of Color
			wlcd_write8_sw(FLG_D_C, R5G6B5_L); //data - bit[7:0] of Color
	#else
			wlcd_write8_sw(FLG_D_C, R); //data - bit[23:16] of Color
			wlcd_write8_sw(FLG_D_C, G); //data - bit[15:8] of Color
			wlcd_write8_sw(FLG_D_C, B); //data - bit[7:0] of Color
	#endif
#endif
		}
		Flags = 0;
		YBig += AddBig;
	}
	return;
}

//high level functions - image related

void ICACHE_FLASH_ATTR wlcd_img_get_info(uint8_t* ImgData, wlcd_img_info_struct* RetImgInfo){
//Fills RetImgInfo containing info about WLCD image
	*RetImgInfo = *((wlcd_img_info_struct*)ImgData);
}

#define wlcd_img_get_width(ImgData)				(((wlcd_img_info_struct*)ImgData)->Width)
#define wlcd_img_get_height(ImgData)			(((wlcd_img_info_struct*)ImgData)->Height)
#define wlcd_img_get_bpp_code(ImgData)			(((wlcd_img_info_struct*)ImgData)->BppCode) //see wlcd_img_bpp_enum
#define wlcd_img_is_rle_compressed(ImgData)		(((wlcd_img_info_struct*)ImgData)->IsRLECmprs)

void ICACHE_FLASH_ATTR wlcd_img_draw(uint8_t* ImgData, uint16_t X, uint16_t Y){
/* Draws WLCD image, top left corner starting at X, Y position.
 *
 * WLCD images:
 * Use wlcd_img_gen tool to generate WLCD images from 24-bit / 16-bit BMPs. WLCD image can be
 * RLE compressed (uses 8-bit counter). The wlcd_img_gen tool by default tries both ways
 * and decides what's smaller. To produce smaller images, try to avoid smooth color gradients.
 * ! WLCD images can be 16bpp / 24bpp and this setting must correspond to used WLCD_*BPP !
 * - if you use WLCD_16BPP, generate WLCD image with parameter --16bpp
 * - if you use WLCD_18BPP, generate WLCD image with parameter --24bpp
 *
 * WLCD image format description:
 * --------------------------------------------
 * - First 4 bytes is the header of wlcd_img_info_struct structure
 *
 * - Data starts at offset 4 (ImgData[4])
 *
 * - if WLCD image is RLE compressed and color depth is 16bpp, then the data format is this:
 *   CHLCHLCHLCHL... (C = uint8_t RLE counter; H = uint8_t bit[15:8] of R5G6B5 pixel; L = uint8_t bit[7:0] of R5G6B5 pixel)
 *
 * - if WLCD image is RLE compressed and color depth is 24bpp, then the data format is this:
 *   CRGBCRGBCRGBCRGB... (C = uint8_t RLE counter; R = uint8_t bit[23:16] of R8G8B8 pixel; G = uint8_t bit[15:8] of R8G8B8 pixel; B = uint8_t bit[7:0] of R8G8B8 pixel)
 *
 * - if WLCD image is not RLE compressed and color depth is 16bpp, then the data format is this:
 *   HLHLHLHL... (H = uint8_t bit[15:8] of R5G6B5 pixel; L = uint8_t bit[7:0] of R5G6B5 pixel) => each pixel takes 2 bytes
 *
 * - if WLCD image is not RLE compressed and color depth is 18bpp, then the data format is this:
 *   CRGBCRGBCRGBCRGB... (R = uint8_t bit[23:16] of R8G8B8 pixel; G = uint8_t bit[15:8] of R8G8B8 pixel; B = uint8_t bit[7:0] of R8G8B8 pixel) => each pixel takes 3 bytes
 * --------------------------------------------
 *
 * ! If one of these is true:
 * - the image is not RLE compressed and we're using HSPI (WLCD_USE_HSPI is defined)
 * - the image is RLE compressed and image color depth is 24bpp
 * then ImgData must be 4-bytes aligned and allocated memory length must be multiples of 4.
 * (see wlcd_write_buf_hspibuf64(...) for details why).
 */
	wlcd_img_info_struct ImgInfo;
	wlcd_img_get_info(ImgData, &ImgInfo);
	uint16_t ImgW = ImgInfo.Width;
	uint16_t ImgH = ImgInfo.Height;
	if((ImgW==0)||(ImgH==0)) return;
	//
	wlcd_set_drawing_rect(X, Y, ImgW, ImgH);
	//
	int32_t Pixels = (uint32_t)ImgW*ImgH;
	uint8_t* DataPtr = ImgData+4;
	if(ImgInfo.IsRLECmprs){ //is RLE compressed
		switch(ImgInfo.BppCode) {
			case WLCD_IMG_16BPP:
				{ //---- RLE compressed, R5G6B5, 2 bytes per pixel -------------------------
					uint8_t RLECnt, R5G6B5_H, R5G6B5_L;
					uint8_t Flags = FLG_SEND_CMD|FLG_PULSE_CS_ON_START;
#ifndef WLCD_USE_HSPI
					wlcd_write8_sw(FLG_PULSE_CS_ON_START, WLCD_WRITE_DDRAM); //command
#endif
					while(Pixels>0){ //THIS LOOP IS (almost) CRITICAL
						RLECnt = *(DataPtr++);
						R5G6B5_H = *(DataPtr++);
						R5G6B5_L = *(DataPtr++);
#ifdef WLCD_USE_HSPI
						wlcd_write_dup16_hspi(Flags, WLCD_WRITE_DDRAM, R5G6B5_H, R5G6B5_L, RLECnt);
						Flags = 0;
#else
						uint8_t RLECnt2 = RLECnt;
						while(RLECnt2--){ //THIS LOOP IS CRITICAL
							wlcd_write8_sw(FLG_D_C, R5G6B5_H); //data - bit[15:8] of Color
							wlcd_write8_sw(FLG_D_C, R5G6B5_L); //data - bit[7:0] of Color
						}
#endif
						Pixels -= RLECnt;
					}
				} //------------------------------------------------------------------------
				break;
			case WLCD_IMG_24BPP:
				{ //---- RLE compressed, R8G8B8, 3 bytes per pixel -------------------------
					uint8_t RLECnt, R, G, B;
					uint8_t Flags = FLG_SEND_CMD|FLG_PULSE_CS_ON_START;
#ifndef WLCD_USE_HSPI
					wlcd_write8_sw(FLG_PULSE_CS_ON_START, WLCD_WRITE_DDRAM); //command
#endif
					while(Pixels>0){ //THIS LOOP IS (almost) CRITICAL
						RLECnt = *(DataPtr++);
						R = *(DataPtr++);
						G = *(DataPtr++);
						B = *(DataPtr++);
#ifdef WLCD_USE_HSPI
						wlcd_write_dup24_hspi(Flags, WLCD_WRITE_DDRAM, R, G, B, RLECnt);
						Flags = 0;
#else
						uint8_t RLECnt2 = RLECnt;
						while(RLECnt2--){ //THIS LOOP IS CRITICAL
							wlcd_write8_sw(FLG_D_C, R); //data - bit[23:16] of Color
							wlcd_write8_sw(FLG_D_C, G); //data - bit[15:8] of Color
							wlcd_write8_sw(FLG_D_C, B); //data - bit[7:0] of Color
						}
#endif
						Pixels -= RLECnt;
					}

				} //------------------------------------------------------------------------
				break;
		}
	}
	else{ //is not RLE compressed => is raw stream of pixels
		switch(ImgInfo.BppCode){
			case WLCD_IMG_16BPP: //stream of pixels, R5G6B5, 2 bytes per pixel
				wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, WLCD_WRITE_DDRAM, (uint32_t*)DataPtr, Pixels*2); //! DataPtr must be 4-bytes aligned
				break;
			case WLCD_IMG_24BPP: //stream of pixels, R8G8B8, 3 bytes per pixel
				wlcd_write(FLG_SEND_CMD|FLG_PULSE_CS_ON_START, WLCD_WRITE_DDRAM, (uint32_t*)DataPtr, Pixels*3); //! DataPtr must be 4-bytes aligned
				break;
		}
	}
}

uint32_t ICACHE_FLASH_ATTR wlcd_img_get(uint32_t* Buf, uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height){
/* Reads display GRAM (rectangle specified by X, Y, W, H) into buffer of pixels (Buf).
 * If WLCD_BPP==WLCD_16BPP then the data read are converted to R5G6B5 pixels on the fly.
 * If WLCD_BPP==WLCD_18BPP then the data read are stored as read from GRAM.
 * First 4 bytes of the Buf buffer are filled with the wlcd_img_info_struct, effectively creating an uncompressed
 * WLCD image in the buffer. The Buf buffer must have allocated at least:
 * - if WLCD_BPP==WLCD_16BPP => (Width*Height*2) + 4 bytes
 * - if WLCD_BPP==WLCD_18BPP => (Width*Height*3) + 4 bytes
 * ! If WLCD_USE_HSPI is defined, then Buf must be 4-bytes aligned and allocated memory length must be multiples of 4.
 * Returns number of pixel bytes stored in Buf.
 * - if returned value > 0 => add 4 bytes (header) to get complete size of WLCD image
 * - if WLCD_BPP==WLCD_16BPP => returned value / 2 = number of R5G6B5 pixels
 * - if WLCD_BPP==WLCD_18BPP => returned value / 3 = number of R6G6B6 or R8G8B8 pixels (last two bits are ignored by the LCD controller)
 * Returned 0 means no pixels were read / no image was read.
 */
#ifndef WLCD_NO_READ
	wlcd_set_drawing_rect(X, Y, Width, Height);
	//
	uint8_t Flags = FLG_SEND_CMD|FLG_PULSE_CS_ON_START;
	wlcd_img_info_struct ImgInfo;
	ImgInfo.Width = Width;
	ImgInfo.Height = Height;
#if(WLCD_BPP==WLCD_16BPP)
	ImgInfo.BppCode = WLCD_IMG_16BPP;
	Flags |= FLG_DO_SHRINK_R6G6B6_TO_R5G6B5;
#else
	ImgInfo.BppCode = WLCD_IMG_24BPP;
#endif
	ImgInfo.IsRLECmprs = false;
	//
	Buf[0] = *((uint32_t*)&ImgInfo);
	//
	return wlcd_read(Flags, WLCD_READ_DDRAM, 8, &Buf[1], (uint32_t)Width*Height*3);
#else
	return 0;
#endif
}

//high level functions - text related

void ICACHE_FLASH_ATTR wlcd_text_draw_or_measure(char* Txt, wlcd_text_draw_settings_struct* S, uint8_t DontDrawJustMeasure, uint16_t* RetWidth, uint16_t* RetHeight){
/* Draws/measures text Txt drawn with font wlcd_Fonts[S.FontIdx] at position S.X, S.Y and color S.Color.
 * Each letter is drawn using rectangles according to associated RRE array (rectangles are faster,
 * font definition occupies less space, easy zoom, bold, spacing, ...).
 * Spacing between letters can be increased/decreased using S.HSpc (horizontal), S.VSpc (vertical).
 * S.WrapStyle defines how the wrapping is done and the usable area for text (within which the
 * wrapping is done) is defined by S.MaxW, S.MaxH (in respect to S.X, S.Y).
 * The S.BoldAdd defines addition to rectangles width (simple and effective way how to make font
 * bolder): 0 = original; 1 = make all pixels one pixel bigger horizontally; 2 = two pixels bigger ...
 * The font can be zoomed using FontZoomAdd: 0 = no zoom; 1 = 2x zoom; 2 = 3x zoom ...
 * (the zooming can be done only in integer steps (fractions don't look good because of final rounding
 * to integer) and it doesn't affect S.HSpc, S.VSpc, S.BoldAdd).
 * Variables *RetWidth, *RetHeight are filled by the really used area for drawing the text (wrapping
 * aware). If RetWidth or RetHeight is pointer to NULL, filling of *RetWidth, *RetHeight is skipped.
 * DontDrawJustMeasure switch will turn off the actual drawing - this can be used to measure the
 * text before drawing (to know where to start drawing for right-aligned text or for any other purpose).
 */
	if((Txt==NULL)||(S->FontIdx>=WLCD_FONTS_CNT)) return;
	const wlcd_font_descr_struct* F = &wlcd_Fonts[S->FontIdx];
	uint8_t Zoom = 1 + S->FontZoomAdd;
	uint16_t X = S->X;
	uint16_t Y = S->Y;
	if(RetWidth!=NULL) *RetWidth=X;
	if(RetHeight!=NULL) *RetHeight=Y;
	wlcd_wrap_style_enum CurrWrapStyle = S->WrapStyle;
	uint16_t ChIdx = 0;
	uint16_t NextWrapChIdx = ChIdx;
	char Ch = Txt[ChIdx];
	uint8_t IsAfterWrapNewline = 0;
	while(Ch!=0){ //go through all chars in Txt until null termination encountered
		//
		if(WLCD_IS_NEWLINE_CHAR(Ch)){ //Ch is newline char => do "newline"
			Y += (F->CharHeight * Zoom) + S->VSpc;
			if((RetHeight!=NULL)&&(*RetHeight<Y)) *RetHeight=Y;
			X = S->X;
			if( (Y + (F->CharHeight * Zoom) - S->Y) > S->MaxH){ //new line is out of usable area => exit
				if(RetWidth!=NULL) *RetWidth -= S->X;
				if(RetHeight!=NULL) *RetHeight -= S->Y;
				return;
			}
			//
			Ch = Txt[++ChIdx];
			continue;
		}
		//
		//manage word wrapping
		if((CurrWrapStyle==WLCD_WRAP_WORDS)&&(ChIdx>=NextWrapChIdx)){
			NextWrapChIdx=ChIdx;
			while((Txt[NextWrapChIdx]!=0)&&(!WLCD_IS_WRAP_CHAR(Txt[NextWrapChIdx]))) NextWrapChIdx++;
			uint16_t WordW = F->CharWidth * Zoom;
			if(NextWrapChIdx>ChIdx) WordW = ( (NextWrapChIdx-ChIdx) * ((F->CharWidth * Zoom) + S->HSpc) ) - S->HSpc;
			//
			if(WordW>0){
				if(WordW > S->MaxW){ //word is wider than width of usable area => change CurrWrapStyle to WLCD_WRAP_LETTERS and fake new word width to F->CharWidth * Zoom
					CurrWrapStyle = WLCD_WRAP_LETTERS;
					WordW = F->CharWidth * Zoom;
				}
			}
			if( (X + WordW) > (S->MaxW + S->X) ){ //but the word doesn't fit the usable area => do "newline"
				Y += (F->CharHeight * Zoom) + S->VSpc;
				if((RetHeight!=NULL)&&(*RetHeight<Y)) *RetHeight=Y;
				X = S->X;
				if( (Y + (F->CharHeight * Zoom) - S->Y) > S->MaxH){ //new line is out of usable area => exit
					if(RetWidth!=NULL) *RetWidth -= S->X;
					if(RetHeight!=NULL) *RetHeight -= S->Y;
					return;
				}
				IsAfterWrapNewline = 1; //newline caused by wrapping
			}
		}
		//
		//draw char Ch if it's not space after newline caused by wrapping (if yes, space char is skipped)
		if(!((IsAfterWrapNewline)&&(Ch==32))){
			//
			//draw char Ch if we have font data for this char (else leave blank space and skip the char)
			//drawing uses wlcd_rect_fill_16bpp() - it's quite fast
			if((!DontDrawJustMeasure)&&(Ch>=F->FirstCharCode)&&(Ch<=F->LastCharCode)){
				uint8_t RectsCnt = F->CharOffs[Ch+1] - F->CharOffs[Ch];
				if(RectsCnt>0){
					uint16_t* RectPtr = (uint16_t*)&F->Rects[F->CharOffs[Ch]];
					while(RectsCnt--){
						uint16_t Rect = *(RectPtr++);
						//
						uint16_t DrawX = X+(((Rect>>0)&0x0F)*Zoom);
						uint16_t DrawY = Y+(((Rect>>4)&0x0F)*Zoom);
						uint8_t DrawW = ((((Rect>>8)&0x0F)+1)*Zoom) + S->BoldAdd;
						uint8_t DrawH = (((Rect>>12)&0x0F)+1)*Zoom;
						//
						wlcd_rect_fill(DrawX, DrawY, DrawW, DrawH, S->Color.u32t); //color meaning will be decided inside wlcd_write_dup_color(...) according to WLCD_BPP
					}
				}
			}
			//
			//increase X offset
			X += (F->CharWidth * Zoom) + S->HSpc;
			if( (RetWidth!=NULL) && (*RetWidth<X) && ((CurrWrapStyle==WLCD_WRAP_NONE)||((CurrWrapStyle!=WLCD_WRAP_NONE)&&(Ch!=32))) ) *RetWidth=X;
			IsAfterWrapNewline = 0; //clear flag
			//
			//check if the next letter will be still in the usable area (don't do this check here if CurrWrapStyle==WLCD_WRAP_WORDS, because then it's done in the code above)
			if((CurrWrapStyle!=WLCD_WRAP_WORDS)&&( (X + (F->CharWidth * Zoom)) > (S->MaxW + S->X) )){ //next letter doesn't fit the usable area
				if(CurrWrapStyle==WLCD_WRAP_LETTERS){ //we can wrap (CurrWrapStyle!=WLCD_WRAP_NONE) => do "newline"
					Y += (F->CharHeight * Zoom) + S->VSpc;
					if((RetHeight!=NULL)&&(*RetHeight<Y)) *RetHeight=Y;
					X = S->X;
					if( (Y + (F->CharHeight * Zoom) - S->Y) > S->MaxH){ //new line is out of usable area => exit
						if(RetWidth!=NULL) *RetWidth -= S->X;
						if(RetHeight!=NULL) *RetHeight -= S->Y;
						return;
					}
					IsAfterWrapNewline = 1; //newline caused by wrapping
				}
				else{ //we can't wrap (CurrWrapStyle==WLCD_WRAP_NONE) => exit
					if(RetWidth!=NULL) *RetWidth -= S->X;
					if(RetHeight!=NULL){ *RetHeight += (F->CharHeight * Zoom); *RetHeight -= S->Y; }
					return;
				}
			}
		}
		//
		//prepare for next loop
		Ch = Txt[++ChIdx];
	}
	//
	if(RetWidth!=NULL) *RetWidth -= S->X;
	if(RetHeight!=NULL){ *RetHeight += (F->CharHeight * Zoom); *RetHeight -= S->Y; }
}

uint16_t ICACHE_FLASH_ATTR wlcd_text_nchars_width(uint16_t NumOfChars, wlcd_text_draw_settings_struct* S){
	if(S->FontIdx>=WLCD_FONTS_CNT) return 0;
	const wlcd_font_descr_struct* F = &wlcd_Fonts[S->FontIdx];
	uint8_t Zoom = 1 + S->FontZoomAdd;
	return ( NumOfChars * ((F->CharWidth * Zoom) + S->HSpc) ) - ((NumOfChars>0) ? S->HSpc : 0);
}

uint16_t ICACHE_FLASH_ATTR wlcd_text_nrows_height(uint16_t NumOfRows, wlcd_text_draw_settings_struct* S){
	if(S->FontIdx>=WLCD_FONTS_CNT) return 0;
	const wlcd_font_descr_struct* F = &wlcd_Fonts[S->FontIdx];
	uint8_t Zoom = 1 + S->FontZoomAdd;
	return ( NumOfRows * ((F->CharHeight * Zoom) + S->VSpc) ) - ((NumOfRows>0) ? S->VSpc : 0);
}
