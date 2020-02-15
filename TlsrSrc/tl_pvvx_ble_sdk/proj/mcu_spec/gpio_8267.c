/********************************************************************************************************
 * @file     gpio_8267.c
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

//#if (1)

#if(__TL_LIB_8267__ || (MCU_CORE_TYPE == MCU_CORE_8267) || \
	__TL_LIB_8261__ || (MCU_CORE_TYPE == MCU_CORE_8261) || \
	__TL_LIB_8269__ || (MCU_CORE_TYPE == MCU_CORE_8269)	)

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



/************
 *
 * gpio:         indicate the pin
 * up_down:      0xFF    : float
 * 				 1    : 1M   pullup
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
 08		 PE3         PE2
 */
 
//if GPIO_DP,please check usb_dp_pullup_en() valid or not first.
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
    u8 mask_not = 0xfc;   //default for  PX2  PX6
    u8 shift_num = 0;

    if(GPIO_PE2 == gpio){
        base_ana_reg = 0x08;
        mask_not = 0xcf;
        shift_num = 4;
    }else if(GPIO_PE3 == gpio){
        base_ana_reg = 0x08;
        mask_not = 0x3f;
        shift_num = 6;
    }else{
        if(pin & 0x03) {
            base_ana_reg -= 1;
        }
        else if(pin & 0xc0) {
            base_ana_reg += 1;
        }
        else {
        }

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
    }

    if(GPIO_DP == gpio){
        //usb_dp_pullup_en (0);
    }

	analog_write(base_ana_reg, (analog_read(base_ana_reg) & mask_not) | (r_val << shift_num));
}


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

void gpio_set_func(u32 pin, u32 func)
{
	u8	bit = pin & 0xff;
	if(func == AS_GPIO){
		BM_SET(reg_gpio_gpio_func(pin), bit);
		return;
	}else{
		BM_CLR(reg_gpio_gpio_func(pin), bit);
	}


	//config gpio special func
	switch(pin)
	{
		case GPIO_PA0:
		{
			//1. DMIC(DI)
			//2. PWM0
			if(func == AS_DMIC){
				BM_SET(reg_gpio_config_func(GPIO_PA0), bit);
				BM_SET(reg_gpio_ie(GPIO_PA0), BIT(0));     //enable input
			}else if(func == AS_PWM){
				BM_CLR(reg_gpio_config_func(GPIO_PA0), bit);
			}
		}
		break;

		case GPIO_PA1:
		{
			//only DMIC(CLK) function, no need set
		}
		break;

		case GPIO_PA2:
		case GPIO_PA3:
		case GPIO_PA4:
		case GPIO_PA5:
		{
			// 			PA2			PA3			PA4			PA5
			//1. SPI: 	DO			DI			CK			CN
			//2, PWM: 	PWM0_N		PWM1		PWM1_N		PWM2_N
			if((func == AS_SPI) || (func == AS_I2C)){
				BM_SET(reg_gpio_config_func(pin), bit);
				BM_SET(reg_gpio_ie(pin), bit);   //enable input
				BM_CLR(reg_gpio_oen(pin), bit);  //enable output
			}else if(func == AS_PWM){
				BM_CLR(reg_gpio_config_func(pin), bit);
			}
		}
		break;


		case GPIO_PA6:
		{
			//only UART function, no need set
		}
		break;

		case GPIO_PA7:
		{
			//1. UART(TX)
			//2. SWM
			if(func == AS_UART){
				BM_SET(reg_gpio_config_func(pin), bit);
				BM_SET(reg_gpio_ie(GPIO_PA7), BIT(7));   //enable input
			}else if(func == AS_SWM){
				BM_CLR(reg_gpio_config_func(pin), bit);
			}
		}
		break;

		case GPIO_PB0:
		{
			//1. PWM2
			//2. SWS
			if(func == AS_PWM){
				BM_SET(reg_gpio_config_func(pin), bit);
			}else if(func == AS_SWS){
				BM_CLR(reg_gpio_config_func(pin), bit);
			}
		}
		break;


		case GPIO_PB1:
		{
			// PWM2_N
			if(func == AS_PWM){
				BM_CLR(reg_gpio_config_func(pin), bit);
			}
		}
		break;

		case GPIO_PB2:
		case GPIO_PB3:
		{
			// 			PB2			PB3
			//1. UART: 	TX			RX
			//2, PWM: 	PWM3		PWM3_N
			if(func == AS_UART){
				BM_SET(reg_gpio_config_func(pin), bit);
				BM_SET(reg_gpio_ie(pin), bit);  //enable input
			}else if(func == AS_PWM){
				BM_CLR(reg_gpio_config_func(pin), bit);
			}
		}
		break;


		case GPIO_PB4:
		case GPIO_PB5:
		case GPIO_PB6:
		case GPIO_PB7:
		{
			// 			PB4			PB5			PB6			PB7
			//1. SPI: 	CN			DO			DI			CK
			//2, PWM: 	PWM4		PWM4_N		PWM5		PWM5_N
			if((func == AS_SPI) || (func == AS_I2C)){
				BM_SET(reg_gpio_config_func(pin), bit);
				BM_SET(reg_gpio_ie(pin), bit);  //enable input
				BM_CLR(reg_gpio_oen(pin), bit); //enable output
			}else if(func == AS_PWM){
				BM_CLR(reg_gpio_config_func(pin), bit);
			}
		}
		break;


		case GPIO_PC0:
		case GPIO_PC1:
		{
			// 			PC0			PC1
			//1. I2C: 	DI			CK
			//2, PWM: 	PWM0		PWM1
			if(func == AS_I2C){
				BM_SET(reg_gpio_config_func(pin), bit);
				BM_SET(reg_gpio_ie(pin), bit);  //enable input
				BM_CLR(reg_gpio_oen(pin), bit); //enable output
			}else if(func == AS_PWM){
				BM_CLR(reg_gpio_config_func(pin), bit);
			}
		}
		break;


		case GPIO_PC2:
		case GPIO_PC3:
		case GPIO_PC4:
		case GPIO_PC5:
		{
			// 			PC2			PC3			PC4			PC5
			//1. UART: 	TX			RX			RTS			CTS
			//2, PWM: 	PWM2		PWM3		PWM4		PWM5
			if(func == AS_UART){
				BM_SET(reg_gpio_config_func(pin), bit);
				BM_SET(reg_gpio_ie(pin), bit);  //enable input
			}else if(func == AS_PWM){
				BM_CLR(reg_gpio_config_func(pin), bit);
			}
		}
		break;

		case GPIO_PC6:
		case GPIO_PC7:
		case GPIO_PD0:
		case GPIO_PD1:
		case GPIO_PD2:
		case GPIO_PD3:
		case GPIO_PD4:
		{
			break;
		}


		case GPIO_PD5:
		case GPIO_PD6:
		case GPIO_PD7:
		{
			//PD5	PD6		PD7
			//PWM0	PWM1	PWM2
			if(func == AS_PWM){
				BM_CLR(reg_gpio_config_func(pin), bit);
			}
		}
		break;

		case GPIO_PE0:
		case GPIO_PE1:
		{
			// 			PE0			PE1
			//1. PWM: 	PWM0		PWM1
			//2, SDM: 	SDM_P		SDM_N
			if(func == AS_PWM){
				BM_SET(reg_gpio_config_func(pin), bit);
			}else if(func == AS_SDM){
				BM_CLR(reg_gpio_config_func(pin), bit);
				BM_SET(reg_gpio_ie(pin), bit);  //enable input
			}
		}
		break;


		case GPIO_PE2:
		case GPIO_PE3:
		{
			//only USB func
			//PE2	PE3
			//DM	DP
		}
		break;


		case GPIO_PE4:
		case GPIO_PE5:
		case GPIO_PE6:
		case GPIO_PE7:
		{
			//only FLASH MSPI func
			//PE4	PE5		PE6		PE7
			//MSDO	MCLK	MSCN	MSDI
		}
		break;


		default:
			break;

	}
}

#endif

