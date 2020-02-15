/********************************************************************************************************
 * @file     tl_common.h
 *
 * @brief    for TLSR chips
 *
 * @author	 BLE Group
 * @date     May. 12, 2018
 *
 * @par      Copyright (c) Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *			 The information contained herein is confidential and proprietary property of Telink
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in.
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/

#pragma once

#include "common/types.h"
#include "common/bit.h"
#include "common/utility.h"
#include "common/static_assert.h"
#include "common/assert.h"
#include "mcu/analog.h"
#include "common/compatibility.h"
#include "app_config.h" // "../vendor/common/user_config.h"
#include "mcu/compiler.h"
#include "mcu/register.h"
#include "mcu/gpio.h"

#include "mcu/cpu.h"
#include "mcu/dma.h"
#include "mcu/irq_i.h"
#include "mcu/clock.h"
#include "mcu/clock_i.h"
#include "mcu/random.h"
#include "common/breakpoint.h"
#include "common/log.h"

#include "drivers/flash.h"

#include "../vendor/common/myprintf.h"


//#include "../ble/ble_globals.h"

#ifdef WIN32
#include <stdio.h>
#include <string.h>
#else
#include "common/printf.h"
#include "common/string.h"
#endif

#define DEBUG_STOP()	{reg_tmr_ctrl = 0; reg_gpio_pb_ie = 0xff; while(1);}	// disable watchdog;  DP/DM en
#define DEBUG_SWS_EN()	{reg_tmr_ctrl = 0; reg_gpio_pb_ie = 0xff;}	// disable watchdog;  DP/DM en

