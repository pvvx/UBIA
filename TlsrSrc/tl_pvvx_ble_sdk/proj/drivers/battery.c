/********************************************************************************************************
 * @file     battery.c
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

#include "../tl_common.h"
#include "battery.h"
#include "adc.h"

#include "../../proj_lib/pm.h"
//#include "vendor/826x_ble_remote/app_config.h"
#if(BATT_CHECK_ENABLE)

u8 adc_hw_initialized = 0;
unsigned int batteryVolmV;         //2^14 - 1 = 16383;

/***
 * the function can filter the not good data from the adc.
 * remove the maximum data and minimum data from the adc data.
 * then average the rest data to calculate the voltage of battery.
 **/
unsigned short filter_data(unsigned short* pData,unsigned char len)
{
	unsigned char index = 0,loop = 0;
	unsigned short temData = 0;
	//bubble sort,user can use your algorithm.it is just an example.
	for(loop = 0;loop <(len-1); loop++){
		for(index = 0;index <(len-loop-1); index++){
			if(pData[index]>pData[index+1]){
				temData = pData[index];
				pData[index] = pData[index+1];
				pData[index+1] = temData;
			}
		}
	}

	//remove the maximum and minimum data, then average the rest data.
	unsigned int data_sum = 0;
	unsigned char s_index = 0;
	for(s_index=1;s_index <(len-1);s_index++){
		data_sum += pData[s_index];
	}
	return (data_sum/(len-2));
}


/****
 * the function is used to check the voltage of battery.
 * when the battery voltage is less than 1.96v,
 * let the chip enter deep sleep mode.
 **/
#define  BATT_CHECK_CNT  8
void battery_power_check(u16 alarm_vol_mv)
{

	if(!adc_hw_initialized){

		adc_hw_initialized = 1;

		#if((MCU_CORE_TYPE == MCU_CORE_8261)||(MCU_CORE_TYPE == MCU_CORE_8267)||(MCU_CORE_TYPE == MCU_CORE_8269))
			adc_BatteryCheckInit(ADC_CLK_4M, 1, Battery_Chn_VCC, 0, SINGLEEND, RV_1P428, RES14, S_3);
		#elif(MCU_CORE_TYPE == MCU_CORE_8266)
			adc_Init(ADC_CLK_4M, ADC_CHN_C4, SINGLEEND, ADC_REF_VOL_1V3, ADC_SAMPLING_RES_14BIT, ADC_SAMPLING_CYCLE_6);
		#endif
	}


	unsigned short adcValue[BATT_CHECK_CNT] = {0};

	for(int adc_idx = 0; adc_idx < BATT_CHECK_CNT; adc_idx++){
		adcValue[adc_idx] = adc_SampleValueGet();
	}

	unsigned short average_data;
	average_data = filter_data(adcValue,BATT_CHECK_CNT);


	#if((MCU_CORE_TYPE == MCU_CORE_8261)||(MCU_CORE_TYPE == MCU_CORE_8267)||(MCU_CORE_TYPE == MCU_CORE_8269))
		#ifdef VBAT_ADC_K
			batteryVolmV = ((unsigned int)(VBAT_ADC_K) * (unsigned int)(average_data-128))/(16383-256); //2^14 - 1 = 16383;
		#else
			batteryVolmV = 3*(1428*(unsigned int)(average_data-128)/(16383-256)); //2^14 - 1 = 16383;
		#endif
	#elif(MCU_CORE_TYPE == MCU_CORE_8266)
		#ifdef VBAT_ADC_K
			batteryVolmV = ((unsigned int)(VBAT_ADC_K) * (unsigned int)average_data) >> 14;
		#else
			batteryVolmV = 3*((1300*(unsigned int)average_data) >> 14);
		#endif
	#endif
	// adc_hw_initialized = 3; // bit1 =1 -> calk batteryVolmV

	if(batteryVolmV < alarm_vol_mv){  //when battery voltage is lower than 2.0v, chip will enter deep sleep mode


		analog_write(DEEP_ANA_REG2, BATTERY_VOL_LOW);

		#if (BLT_APP_LED_ENABLE)
			gpio_set_output_en(GPIO_LED, 1); // output enable
			for(int k=0; k < 3; k++){
				gpio_write(GPIO_LED, LED_ON_LEVEL);
				sleep_us(200*1000);
				gpio_write(GPIO_LED,!LED_ON_LEVEL);
				sleep_us(200*1000);
			}
		#endif
		// MCU enters deepsleep mode when this function is executed, and it can be woke up by GPIO PAD.
		cpu_sleep_wakeup(1, PM_WAKEUP_PAD, 0);
	}
}

#endif

