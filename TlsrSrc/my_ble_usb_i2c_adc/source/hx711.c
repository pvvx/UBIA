/*
 * hx711.c
 *
 *  Created on: 24.12.2019
 *      Author: pvvx
 */
#include "proj/tl_common.h"
#include "hx711.h"

#ifndef HX711_SCK
#define HX711_SCK 	GPIO_PC0
#endif
#ifndef HX711_DOUT
#define HX711_DOUT	GPIO_PC1
#endif

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
