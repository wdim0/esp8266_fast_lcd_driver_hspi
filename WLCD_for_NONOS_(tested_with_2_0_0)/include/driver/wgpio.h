/*
 * Created by Martin Winkelhofer 11/2016
 * W-Dimension / wdim / wdim0 / winkelhofer.m@gmail.com / https://github.com/wdim0
 *                    _
 *  _    _____ ____  (_)__
 * | |/|/ / _ `/ _ \/ / _ \
 * |__,__/\_, / .__/_/\___/
 *       /___/_/
 *
 * Includes Espressif's standard gpio.h + introduces some handy GPIO functions.
 * Based on Espressif's RTOS SDK include/driver/gpio.h
 * v1.0 (11/2016)
 *
 * Provides:
 * --------------------------------------
 * GPIO_PIN_REG* - get PERIPHS_IO_MUX_* register name based on GPIO pin number
 * GPIO_FUNC* - get constant for PERIPHS_IO_MUX_* register that will set GPIO functionality
 * usage example:
 *		//OUTPUT - set GPIO15 as output, set log. level L
 *		GPIO_OUTPUT_SET(15, 0); //output, log. level L
 *		PIN_FUNC_SELECT(GPIO_PIN_REG(15), GPIO_FUNC(15)); //set pin MTDO (MUX of this pin can have GPIO15 functionality) as GPIO15
 *		//
 *		//macros for GPIO15 setting
 *		#define GPIO15_H			(GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, 1<<15))
 *		#define GPIO15_L			(GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, 1<<15))
 *		#define GPIO15(x)			((x)?GPIO15_H:GPIO15_L)
 *
 *		//INPUT - set GPIO12 as input
 *		GPIO_DIS_OUTPUT(12); //as input
 *		PIN_FUNC_SELECT(GPIO_PIN_REG(12), GPIO_FUNC(12)); //set pin MTDI (MUX of this pin can have GPIO12 functionality) as GPIO12
 *		//
 *		//macro for GPIO12 reading
 *		#define WLCD_MISO			((GPIO_REG_READ(GPIO_IN_ADDRESS)&(BIT(12)))!=0)
 *
 * --------------------------------------
 */
#ifndef __WGPIO_H__
#define __WGPIO_H__

#include <c_types.h>
#include <gpio.h>

#define WGPIO_VERSION			"v1.0" //11/2016

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_PIN_0				(BIT(0))
#define GPIO_PIN_1				(BIT(1))
#define GPIO_PIN_2				(BIT(2))
#define GPIO_PIN_3				(BIT(3))
#define GPIO_PIN_4				(BIT(4))
#define GPIO_PIN_5				(BIT(5))
#define GPIO_PIN_6				(BIT(6))
#define GPIO_PIN_7				(BIT(7))
#define GPIO_PIN_8				(BIT(8))
#define GPIO_PIN_9				(BIT(9))
#define GPIO_PIN_10				(BIT(10))
#define GPIO_PIN_11				(BIT(11))
#define GPIO_PIN_12				(BIT(12))
#define GPIO_PIN_13				(BIT(13))
#define GPIO_PIN_14				(BIT(14))
#define GPIO_PIN_15				(BIT(15))
#define GPIO_PIN_ALL			(0xFFFF)  //all PINs selected

//GPIO_PIN_REG* - get PERIPHS_IO_MUX_* register name based on GPIO pin number
#define GPIO_PIN_REG_0			PERIPHS_IO_MUX_GPIO0_U
#define GPIO_PIN_REG_1			PERIPHS_IO_MUX_U0TXD_U
#define GPIO_PIN_REG_2			PERIPHS_IO_MUX_GPIO2_U
#define GPIO_PIN_REG_3			PERIPHS_IO_MUX_U0RXD_U
#define GPIO_PIN_REG_4			PERIPHS_IO_MUX_GPIO4_U
#define GPIO_PIN_REG_5			PERIPHS_IO_MUX_GPIO5_U
#define GPIO_PIN_REG_6			PERIPHS_IO_MUX_SD_CLK_U
#define GPIO_PIN_REG_7			PERIPHS_IO_MUX_SD_DATA0_U
#define GPIO_PIN_REG_8			PERIPHS_IO_MUX_SD_DATA1_U
#define GPIO_PIN_REG_9			PERIPHS_IO_MUX_SD_DATA2_U
#define GPIO_PIN_REG_10			PERIPHS_IO_MUX_SD_DATA3_U
#define GPIO_PIN_REG_11			PERIPHS_IO_MUX_SD_CMD_U
#define GPIO_PIN_REG_12			PERIPHS_IO_MUX_MTDI_U
#define GPIO_PIN_REG_13			PERIPHS_IO_MUX_MTCK_U
#define GPIO_PIN_REG_14			PERIPHS_IO_MUX_MTMS_U
#define GPIO_PIN_REG_15			PERIPHS_IO_MUX_MTDO_U
//
#define GPIO_PIN_REG(i) \
	(i==0) ? GPIO_PIN_REG_0:  \
	(i==1) ? GPIO_PIN_REG_1:  \
	(i==2) ? GPIO_PIN_REG_2:  \
	(i==3) ? GPIO_PIN_REG_3:  \
	(i==4) ? GPIO_PIN_REG_4:  \
	(i==5) ? GPIO_PIN_REG_5:  \
	(i==6) ? GPIO_PIN_REG_6:  \
	(i==7) ? GPIO_PIN_REG_7:  \
	(i==8) ? GPIO_PIN_REG_8:  \
	(i==9) ? GPIO_PIN_REG_9:  \
	(i==10)? GPIO_PIN_REG_10: \
	(i==11)? GPIO_PIN_REG_11: \
	(i==12)? GPIO_PIN_REG_12: \
	(i==13)? GPIO_PIN_REG_13: \
	(i==14)? GPIO_PIN_REG_14: \
	GPIO_PIN_REG_15

//GPIO_FUNC* - get constant for PERIPHS_IO_MUX_* register that will set GPIO functionality
//added by wdim - analogous to GPIO_PIN_REG*
#define GPIO_FUNC_12			3
#define GPIO_FUNC_13			3
#define GPIO_FUNC_14			3
#define GPIO_FUNC_15			3
#define GPIO_FUNC_3				3
#define GPIO_FUNC_1				3
#define GPIO_FUNC_6				3
#define GPIO_FUNC_7				3
#define GPIO_FUNC_8				3
#define GPIO_FUNC_9				3
#define GPIO_FUNC_10			3
#define GPIO_FUNC_11			3
#define GPIO_FUNC_0				0
#define GPIO_FUNC_2				0
#define GPIO_FUNC_4				0
#define GPIO_FUNC_5				0
//
#define GPIO_FUNC(i) \
	(i==0) ? GPIO_FUNC_0:  \
	(i==1) ? GPIO_FUNC_1:  \
	(i==2) ? GPIO_FUNC_2:  \
	(i==3) ? GPIO_FUNC_3:  \
	(i==4) ? GPIO_FUNC_4:  \
	(i==5) ? GPIO_FUNC_5:  \
	(i==6) ? GPIO_FUNC_6:  \
	(i==7) ? GPIO_FUNC_7:  \
	(i==8) ? GPIO_FUNC_8:  \
	(i==9) ? GPIO_FUNC_9:  \
	(i==10)? GPIO_FUNC_10: \
	(i==11)? GPIO_FUNC_11: \
	(i==12)? GPIO_FUNC_12: \
	(i==13)? GPIO_FUNC_13: \
	(i==14)? GPIO_FUNC_14: \
	GPIO_FUNC_15

#ifdef __cplusplus
}
#endif

#endif
