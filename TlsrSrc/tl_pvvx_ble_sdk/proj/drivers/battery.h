/********************************************************************************************************
 * @file     battery.h
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

#ifndef    VBAT_ALARM_THRES_MV
#define    VBAT_ALARM_THRES_MV         2000  ///2000 mV low battery alarm
#endif

#ifndef	   VBAT_NOM_MV
#define    VBAT_NOM_MV         		   3300  /// 33000 mV 100% voltage battery
#endif

#define    BATTERY_VOL_OK              0x00
#define    BATTERY_VOL_LOW             0x01

extern unsigned int batteryVolmV;
void battery_power_check(u16 alarm_vol_mv);
