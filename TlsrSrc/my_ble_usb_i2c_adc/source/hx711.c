/*
 * hx711.c
 *
 *  Created on: 24.12.2019
 *      Author: pvvx
 */
#include "proj/tl_common.h"
#if USE_HX711
#include "hx711.h"

#ifndef HX711_SCK
#define HX711_SCK 	GPIO_PC0
#endif
#ifndef HX711_DOUT
#define HX711_DOUT	GPIO_PC1
#endif

u32 hx711_buf[HX711_BUF_CNT]; // 64 bytes
u8 hx711_wr = 0;
u8 hx711_rd = 0;
u8 hx711_mode = 0;

void hx711_go_sleep(void) {
//	bls_app_registerEventCallback(BLT_EV_FLAG_GPIO_EARLY_WAKEUP, NULL);
	hx711_mode = 0;
	hx711_wr = 0;
	hx711_rd = 0;
	gpio_setup_up_down_resistor(HX711_SCK, PM_PIN_PULLUP_1M);
	gpio_setup_up_down_resistor(HX711_DOUT, PM_PIN_PULLUP_1M);
#if (USE_BLE)
	gpio_set_wakeup(HX711_DOUT, 0, 0); // отключить пробуждение от hx711
#endif
}

//void hx711_wakeup(void) {
//	bls_app_registerEventCallback(BLT_EV_FLAG_GPIO_EARLY_WAKEUP, &ble_ev_gpio_wakeup);
//	hx711_wr = 0;
//	gpio_setup_up_down_resistor(HX711_SCK, PM_PIN_PULLDOWN_100K);
//	if(!sleep_mode) gpio_set_wakeup(HX711_DOUT, 0, 1);  // core(gpio) low wakeup suspend
//}


_attribute_ram_code_
int hx711_get_data(hx711_mode_t mode) {
		u32 i = mode;
		u32 x = 0x100 >> (mode & 3);

		u8 tmp_s = HX711_SCK & 0xff;
		u32 pcClkReg = (0x583+((HX711_SCK>>8)<<3)); // reg_gpio_out() register GPIO output

		u8 r = irq_disable();
		write_reg8(pcClkReg, read_reg8(pcClkReg) & (~tmp_s));

		reg_gpio_oen(HX711_SCK) &= ~tmp_s; // Enable PD_SCK output

		u8 tmp_d = HX711_DOUT & 0xff;
		reg_gpio_ie(HX711_DOUT) |= tmp_d; // Enable PD_DOUT input
		irq_restore(r);

		u32 pcRxReg = (0x580+((HX711_DOUT>>8)<<3)); // reg_gpio_in() register GPIO input

		u32 dout = 0;
//		sleep_us(1);

		while(i--) {
			sleep_us(1);
			r = irq_disable();
			write_reg8(pcClkReg, read_reg8(pcClkReg) | tmp_s);
			sleep_us(1);
			dout <<= 1;
			if(read_reg8(pcRxReg) & tmp_d)
				dout |= x;
			write_reg8(pcClkReg, read_reg8(pcClkReg) & (~tmp_s));
			irq_restore(r);
		}

		r = irq_disable();
		reg_gpio_oen(HX711_SCK) |= tmp_s; // Disable PD_SCK output
//		reg_gpio_ie(HX711_DOUT) &= ~tmp_d; // Disable PD_DOUT input
		irq_restore(r);
		return dout;
}

#endif // USE_HX711

