/********************************************************************************************************
 * @file     pm_8266.h
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

#include "../proj/config/user_config.h"
//#include "../proj/mcu/config.h"

#if(__TL_LIB_8266__ || MCU_CORE_TYPE == MCU_CORE_8266)


#ifndef PM_TIM_RECOVER_MODE
#define PM_TIM_RECOVER_MODE			    	0
#endif

#if (PM_TIM_RECOVER_MODE)

typedef struct{
	unsigned int   tick_sysClk;
	unsigned int   tick_32k;
	unsigned int   recover_flag;
}pm_tim_recover_t;

extern _attribute_aligned_(4) pm_tim_recover_t			pm_timRecover;
#endif

static inline void usb_dp_pullup_en (int en)
{
	unsigned char dat = ReadAnalogReg(0x00);
	if (en) {
		dat &= ~BIT(4);
	}
	else {
		dat |= BIT(4);
	}

	WriteAnalogReg (0x00, dat);
}




#define SUSPEND_MODE	0
#define DEEPSLEEP_MODE	1





#define DEEP_ANA_REG0    0x34
#define DEEP_ANA_REG1    0x35
#define DEEP_ANA_REG2    0x36
#define DEEP_ANA_REG3    0x37
#define DEEP_ANA_REG4    0x38
#define DEEP_ANA_REG5    0x39

#define DEEP_ANA_REG6    0x3c
#define DEEP_ANA_REG7    0x3d
#define DEEP_ANA_REG8    0x3e


#define ADV_DEEP_FLG	 0x01
#define CONN_DEEP_FLG	 0x02



#define SYS_DEEP_ANA_REG	0x3a





#define 	BLT_RESET_WAKEUP_TIME_2000		1

#if (BLT_RESET_WAKEUP_TIME_2000)
	#define RESET_TIME_US	    	  2000
	#define EARLYWAKEUP_TIME_US       2100
	#define EMPTYRUN_TIME_US       	  2400
#elif(BLT_RESET_WAKEUP_TIME_2200)
	#define RESET_TIME_US	    	  2200
	#define EARLYWAKEUP_TIME_US       2300
	#define EMPTYRUN_TIME_US       	  2600
#elif(BLT_RESET_WAKEUP_TIME_2400)
	#define RESET_TIME_US	    	  2400
	#define EARLYWAKEUP_TIME_US       2500
	#define EMPTYRUN_TIME_US       	  2800
#elif(BLT_RESET_WAKEUP_TIME_2600)
	#define RESET_TIME_US	    	  2600
	#define EARLYWAKEUP_TIME_US       2700
	#define EMPTYRUN_TIME_US       	  3000
#elif(BLT_RESET_WAKEUP_TIME_2800)
	#define RESET_TIME_US	    	  2800
	#define EARLYWAKEUP_TIME_US       3900
	#define EMPTYRUN_TIME_US       	  3200
#else
#endif



typedef int (*suspend_handler_t)(void);
extern suspend_handler_t	func_before_suspend;


void cpu_wakeup_init(int);



typedef int (*cpu_pm_handler_t)(int,int, u32);
extern cpu_pm_handler_t cpu_sleep_wakeup;
//int cpu_sleep_wakeup (int deepsleep, int wakeup_src, u32 wakeup_tick);

int cpu_sleep_wakeup_32krc (int deepsleep, int wakeup_src, u32 wakeup_tick);
int cpu_sleep_wakeup_32kpad (int deepsleep, int wakeup_src, u32 wakeup_tick);


void blt_pm_ext32k_crystal_init(void);



enum {
	 PM_WAKEUP_CORE  = BIT(5),
	 PM_WAKEUP_TIMER = BIT(6),
	 PM_WAKEUP_COMP  = BIT(7),
	 PM_WAKEUP_PAD   = BIT(8),

	 PM_TIM_RECOVER_START =	BIT(14),
	 PM_TIM_RECOVER_END   =	BIT(15),
};


enum {
	 WAKEUP_STATUS_COMP   = BIT(0),  //wakeup by comparator
	 WAKEUP_STATUS_TIMER  = BIT(1),
	 WAKEUP_STATUS_CORE   = BIT(2),
	 WAKEUP_STATUS_PAD    = BIT(3),

	 STATUS_GPIO_ERR_NO_ENTER_PM = BIT(7),
};

#define 	WAKEUP_STATUS_TIMER_CORE	( WAKEUP_STATUS_TIMER | WAKEUP_STATUS_CORE)







/******************************* User Interface  ************************************/


void blc_pm_select_internal_32k_crystal(void);
void blc_pm_select_external_32k_crystal(void);

void bls_pm_registerFuncBeforeSuspend (suspend_handler_t func );

void cpu_set_gpio_wakeup (int pin, int pol, int en);



void blc_pm_disableFlashShutdown_when_suspend(void);

void cpu_stall_wakeup_by_timer0(u32 tick_stall);
void cpu_stall_wakeup_by_timer1(u32 tick_stall);
void cpu_stall_wakeup_by_timer2(u32 tick_stall);


#endif
