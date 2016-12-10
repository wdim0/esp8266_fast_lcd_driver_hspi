/*
 * Created by Martin Winkelhofer 11,12/2016
 * W-Dimension / wdim / wdim0 / winkelhofer.m@gmail.com / https://github.com/wdim0
 *   _____
 *  / ___/__  __ _  __ _  ___  ___
 * / /__/ _ \/  ' \/  ' \/ _ \/ _ \
 * \___/\___/_/_/_/_/_/_/\___/_//_/
 *
 * Common macros and functions for ESP8266.
 * v1.1 (12/2016)
 *
 * This file is part of WCOMMON - W-Dimension's common macros and functions for ESP8266.
 *
 * WCOMMON is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * WCOMMON is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WCOMMON. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __WCOMMON_H__
#define __WCOMMON_H__

#include <c_types.h>

#define WCOMMON_VERSION			"v1.1" //12/2016

#define UPPER_CHAR(Ch)		( (((Ch)>='a')&&((Ch)<='z'))?(Ch)-('a'-'A'):(Ch) )
#define RAND_0_TO_X(x)		( (uint32_t)os_random() / ((~(uint32_t)0)/(x)) ) //get uint32_t random number 0..x (including)
#define NOT_FLAG8(x)		( (uint8_t)~(uint8_t)(x) )
#define NOT_FLAG16(x)		( (uint16_t)~(uint16_t)(x) )

//is there any better way for NONOS SDK? (which allows "OS" to process its own "tasks" while waiting?)
#define OS_DELAY_MS(x)		\
	do { \
		uint16_t i; \
		for(i=0;i<(x);i++) { \
			system_soft_wdt_feed(); \
			os_delay_us(1000); \
		} \
	} while(0)

#endif
