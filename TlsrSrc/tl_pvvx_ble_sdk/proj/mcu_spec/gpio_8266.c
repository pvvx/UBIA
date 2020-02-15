/********************************************************************************************************
 * @file     gpio_8266.c
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


#include "../config/user_config.h"
#include "../mcu/config.h"

#if(__TL_LIB_8266__ || (MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8266))

#include "../common/types.h"
#include "../common/compatibility.h"
#include "../common/bit.h"
#include "../common/utility.h"
#include "../common/static_assert.h"
#include "../mcu/compiler.h"
#include "../mcu/register.h"
#include "../mcu/anareg.h"
#include "../mcu/analog.h"

#include "../mcu/gpio.h"

void gpio_set_wakeup(u32 pin, u32 level, int en)
{
    u8 bit = pin & 0xff;
    if (en) {
        BM_SET(reg_gpio_irq_wakeup_en(pin), bit);
    }
    else {
        BM_CLR(reg_gpio_irq_wakeup_en(pin), bit);
    }
    if(level){
        BM_CLR(reg_gpio_pol(pin), bit);
    }else{
        BM_SET(reg_gpio_pol(pin), bit);
    }
}



/************
 *
 * gpio:         indicate the pin
 * up_down:      0	  : float
 * 				 1    :	1M   pullup
 * 				 2    : 10K  pullup
 * 				 3    : 100K pulldown
 *
 *     BIT(7.6)   BIT(5.4)   BIT(3.2)   BIT(1.0)
mask_not 0x3f       0xcf	  0xf3       0xfc

 0a		 PA1         PA0
 0b		 PA5         PA4      PA3        PA2		0
 0c		 PB1         PB0      PA7        PA6
 0d		 PB5         PB4      PB3        PB2		1
 0e		 PC1         PC0      PB7        PB6
 0f		 PC5         PC4      PC3        PC2		2
 10		 PD1         PD0      PC7        PC6
 11		 PD5         PD4      PD3        PD2		3
 12		 PE1         PE0      PD7        PD6
 13		 PE5         PE4      PE3        PE2		4
 14		 PF1         PF0      PE7        PE6
 */
void gpio_setup_up_down_resistor(u32 gpio, u32 up_down) {
	u8 r_val;

	 if(up_down == PM_PIN_UP_DOWN_FLOAT) {
		 r_val = 0;
	 }
	 else if(up_down == PM_PIN_PULLUP_1M) {
		 r_val = PM_PIN_PULLUP_1M;
	 }
	 else if(up_down == PM_PIN_PULLUP_10K) {
		 r_val = PM_PIN_PULLUP_10K;
	 }
	 else {
		 r_val = PM_PIN_PULLDOWN_100K;
	 }


	 u8 pin = gpio & 0xff;

	 u8 base_ana_reg = 0x0b + ((gpio >> 8) << 1);

	 if(pin & 0x03) {
		 base_ana_reg -= 1;
	 }
	 else if(pin & 0xc0) {
		 base_ana_reg += 1;
	 }

	 u8 mask_not = 0xfc;   //default for  PX2  PX6
	 u8 shift_num = 0;

	 if(pin & 0x88) { //PX3  PX7
		  mask_not = 0xf3;
		  shift_num = 2;
	 }
	 else if(pin & 0x11) {   //PX0  PX4
		 mask_not = 0xcf;
		 shift_num = 4;
	 }
	 else if(pin & 0x22) {   //PX1  PX5
		 mask_not = 0x3f;
		 shift_num = 6;
	 }

	 analog_write(base_ana_reg, (analog_read(base_ana_reg) & mask_not) | (r_val << shift_num));
}


//reg_gpio_config_func0

void gpio_set_func(u32 pin, u32 func)
{
	u8	bit = pin & 0xff;
	if(func == AS_GPIO){
		BM_SET(reg_gpio_gpio_func(pin), bit);
		return;
	}else{
		BM_CLR(reg_gpio_gpio_func(pin), bit);
	}

	switch(pin){
	case GPIO_PA0:   //as sws
	case GPIO_PA2:   //as spi DI
	case GPIO_PA3:   //as spi clk
	case GPIO_PB2:   //as spi DO
	case GPIO_PB3:   //as spi CN/CS
	case GPIO_PB4:   //only gpio function
	case GPIO_PB5:   //as usb DM
	case GPIO_PB6:   //as usb DP
	case GPIO_PB7:   //as pwm
		break;

	case GPIO_PA1:   //
	case GPIO_PA4:
	case GPIO_PA5:
	case GPIO_PA6:
	case GPIO_PC5:
	case GPIO_PD2:
	case GPIO_PD3:
		if (func == AS_KS){
			BM_SET(reg_gpio_config_func(pin), bit);
		}
		else if(func == AS_DEBUG){
			BM_CLR(reg_gpio_config_func(pin), bit);
			BM_SET(reg_gpio_config_func6, BIT(4));
		}
		else if(func == AS_PWM){
			BM_CLR(reg_gpio_config_func(pin), bit);
			BM_CLR(reg_gpio_config_func6, BIT(4));   //clear 0x5b6[4]
		}
		break;

	case GPIO_PA7:   //as ks
		if (func == AS_KS){
			BM_SET(reg_gpio_config_func(pin), bit);
		}
		else if(func == AS_SWM){
			BM_CLR(reg_gpio_config_func(pin), bit);
		}
		break;

	case GPIO_PB0:
	case GPIO_PB1:
		if (func == AS_PWM){
			BM_CLR(reg_gpio_config_func6, BIT(0));   //clear 0x5b6[0]
			BM_CLR(reg_gpio_config_func7, BIT(0));   //clear 0x5b7[0]
		}
		break;


	case GPIO_PC0:
	case GPIO_PC1:
	case GPIO_PC3:
	case GPIO_PC4:
		if (func == AS_KS){
			BM_SET(reg_gpio_config_func(pin), bit);
		}
		else if(func == AS_PWM){
			BM_CLR(reg_gpio_config_func(pin), bit);
			BM_CLR(reg_gpio_config_func6, BIT(6)|BIT(7));   //clear 0x5b6[6] and 0x5b6[7]
		}
		break;

	case GPIO_PC2:
		if (func == AS_KS){
			BM_SET(reg_gpio_config_func(GPIO_PC2), bit);
		}
		else if (func == AS_PWM){
			BM_CLR(reg_gpio_config_func(GPIO_PC2), bit);
		}
		break;

	case GPIO_PC6:
	case GPIO_PC7:
		if (func == AS_KS){
			BM_SET(reg_gpio_config_func(pin), bit);
		}
		else if (func == AS_UART){
			BM_CLR(reg_gpio_config_func(pin), bit);
			BM_SET(reg_gpio_ie(pin), bit);  //enable input
		}
		break;

	case GPIO_PD0:  //NOTE, not sure
	case GPIO_PD1:
		if (func == AS_KS){
			BM_SET(reg_gpio_config_func(pin), bit);
		}
		else if(func == AS_UART){
			BM_CLR(reg_gpio_config_func(pin), bit);
			BM_SET(reg_gpio_config_func6, BIT(1));
			BM_SET(reg_gpio_ie(pin), bit);  //enable input
		}
		break;

	case GPIO_PD6:
		if (func == AS_KS){
			BM_SET(reg_gpio_config_func(GPIO_PD6), bit);
		}
		break;

	case GPIO_PD4:
	case GPIO_PD5:
	case GPIO_PD7:
	case GPIO_PE0:
	case GPIO_PE3:
		if (func == AS_KS){
			BM_SET(reg_gpio_config_func(pin), bit);
		}
		else if(func == AS_DEBUG){
			BM_CLR(reg_gpio_config_func(pin), bit);
			BM_SET(reg_gpio_config_func6, BIT(4));
		}
		break;

	case GPIO_PE1:
	case GPIO_PE2:
		if (func == AS_KS){
			BM_SET(reg_gpio_config_func(pin), bit);
		}
		else if (func == AS_DMIC){
			BM_CLR(reg_gpio_config_func(pin), bit);
			BM_SET(reg_gpio_ie(pin), bit);  //enable input
		}
		break;

	case GPIO_PE4:
	case GPIO_PE5:
		if (func == AS_KS){
			BM_SET(reg_gpio_config_func(pin), bit);
		}
		else if(func == AS_DEBUG){
			BM_CLR(reg_gpio_config_func(pin), bit);
			BM_SET(reg_gpio_config_func6, BIT(4));   //clear 0x5b6[4]
		}
		else if(func == AS_SDM){
			BM_CLR(reg_gpio_config_func(pin), bit);
			BM_CLR(reg_gpio_config_func6, BIT(4));   //clear 0x5b6[4]
			BM_SET(reg_gpio_ie(pin), bit);  //enable input
		}
		break;

	case GPIO_PE6:
		if(func == AS_KS){
			BM_SET(reg_gpio_config_func(GPIO_PE6), bit);
		}
		else if(func == AS_UART){
			BM_CLR(reg_gpio_config_func(GPIO_PE6), bit);
			BM_SET(reg_gpio_config_func6, BIT(5));
			BM_SET(reg_gpio_ie(GPIO_PE6), BIT(6));  //enable input
		}
		else if(func == AS_SPI){
			BM_CLR(reg_gpio_config_func(GPIO_PE6), bit);
			BM_CLR(reg_gpio_config_func6, BIT(5));
			BM_SET(reg_gpio_ie(GPIO_PE6), BIT(6));  //enable input
		}
		break;

	case GPIO_PE7:
		if (func == AS_KS){
			BM_SET(reg_gpio_config_func(GPIO_PE7), bit);
		}
		else if ((func == AS_I2C) || (func == AS_SPI)){
			BM_CLR(reg_gpio_config_func(GPIO_PE7), bit);
			BM_SET(reg_gpio_ie(GPIO_PE7), BIT(7));  //enable input
			BM_CLR(reg_gpio_oen(GPIO_PE7), BIT(7)); //enable output
		}
		break;

	case GPIO_PF0:
		if(func == AS_UART){
			BM_SET(reg_gpio_config_func6, BIT(5));
		}
		else if(func == AS_SPI){
			BM_CLR(reg_gpio_config_func6, BIT(5));
			BM_SET(reg_gpio_ie(GPIO_PF0), BIT(0));  //enable input
			//BM_CLR(reg_gpio_oen(GPIO_PF0), BIT(0)); //enable output
		}
		break;
	case GPIO_PF1:
		if(func == AS_I2C)
		{
			//BM_CLR(reg_gpio_config_func6, BIT(5));
			BM_SET(reg_gpio_ie(GPIO_PF1), BIT(1));  //enable input
			//BM_CLR(reg_gpio_oen(GPIO_PF1), BIT(1)); //enable output
		}
		break;
	default :
		break;
	}
}





#endif

