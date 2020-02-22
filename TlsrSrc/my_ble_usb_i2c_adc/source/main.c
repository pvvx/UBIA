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
#if (USE_BLE)
#include "ble.h"
#else
#include "inits.h"
#endif
#if (USE_USB_CDC)
#include "usb.h"
#endif
#if (USE_I2C_DEV)
#include "i2c_dev.h"
#endif
#include "app.h"

_attribute_ram_code_ void irq_handler(void)
{
#if (USE_I2C_DEV)
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
#if (USE_USB_CDC && USE_BLE)
	if(usb_actived)
		USB_IrqHandle(reg_irq_src);
	else
		irq_blt_sdk_handler();
#elif (USE_BLE)
	irq_blt_sdk_handler();
#elif (USE_USB_CDC)
	USB_IrqHandle(reg_irq_src);
#endif
}

int main (void) {
#if (USE_BLE)
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
#else // USE_USB_CDC
	// пнуть мозги SoC
	reg_rst_clk0 = FLD_CLK_SPI_EN | FLD_CLK_MCU_EN | FLD_CLK_ZB_EN;// After reset [0x60] = 0x130F3FE4
	// Утановить регистры SoC согласно флагам в хидерах
	StartUpInits();
	// задать задачи и т.д.
	user_init();
    irq_enable();
#endif // USE_BLE

#if (USE_USB_CDC && USE_BLE)
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
#elif (USE_BLE)
	while (1) {
#if (MODULE_WATCHDOG_ENABLE)
		wd_clear(); //clear watch dog
#endif
		main_ble_loop();
	}
#elif (USE_USB_CDC)
	while (1) {
#if (MODULE_WATCHDOG_ENABLE)
		wd_clear(); //clear watch dog
#endif
		main_usb_loop();
	}
#endif
}
