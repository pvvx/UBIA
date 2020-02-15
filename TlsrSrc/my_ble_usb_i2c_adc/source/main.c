/********************************************************************************************************
 * @file     main.c
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

#include "proj/tl_common.h"
#include "proj/mcu/watchdog_i.h"
//#include "vendor/common/user_config.h"
#include "proj_lib/rf_drv.h"
#include "proj_lib/pm.h"
#include "proj_lib/ble/blt_config.h"
#include "proj_lib/ble/ll/ll.h"
#include "usbCDC/drivers.h"
#include "app.h"

_attribute_ram_code_ void irq_handler(void)
{
#if USE_I2C_INA
	if((reg_irq_mask1 & FLD_IRQ_TMR1_EN) != 0 && (reg_irq_src & FLD_IRQ_TMR1_EN) != 0) {
		reg_tmr_sta = FLD_TMR_STA_TMR1; // clear irq status
		reg_irq_src =  FLD_IRQ_TMR1_EN;
#if defined(LED_POWER)
		LED_POWER_TOGLE();
#endif
		if(timer_flg)
			GetNewRegData();
	}
#endif
	if(usb_actived)
		USB_IrqHandle(reg_irq_src);
	else
		irq_blt_sdk_handler();
}

int main (void) {
	blc_pm_select_internal_32k_crystal();

	cpu_wakeup_init(CRYSTAL_TYPE);

	//	bls_ota_setFirmwareSizeAndOffset();

//	set_tick_per_us(CLOCK_SYS_CLOCK_HZ/1000000);

	clock_init();

	gpio_init();

	///NOTE:This function must be placed before the following function rf_drv_init().
	blc_app_loadCustomizedParameters();  //load customized freq_offset cap value and tp value

	user_init();

    irq_enable();

    if(usb_actived) {
    	while (1) {
#if (MODULE_WATCHDOG_ENABLE)
    		wd_clear(); //clear watch dog
#endif
    		main_usb_loop();
    	}
    } else {
    	while (1) {
#if (MODULE_WATCHDOG_ENABLE)
    		wd_clear(); //clear watch dog
#endif
    		main_ble_loop();
    	}
    }
}
