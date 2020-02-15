/********************************************************************************************************
 * @file     adc_8266.c
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
#if (__TL_LIB_8266__ || (MCU_CORE_TYPE == MCU_CORE_8266))
#include "adc_8266.h"

#include "../../proj_lib/rf_drv.h"

u8 adc_clk_step_l  = 0;
u8 adc_chn_m_input = 0;

#define		EN_MANUALM			(reg_adc_ctrl = 0x20)

/*****
 * @brief init adc module. such as adc clock, input channel, resolution, reference voltage and so on.
 *        notice: adc clock: when the reference voltage is AVDD, the adc clock must be lower than 5Mhz.
 *        when the reference voltage is 1.4, the adc clock must be lower than 4Mhz.
 * @param[in] adc_clock    - enum ADC_CLK_t, set adc clock.
 * @param[in] chn          - enum ADC_INPUTCHN_t ,acd channel
 * @param[in] mode         - enum ADC_INPUTMODE_t
 * @param[in] ref_vol      - enum ADC_REFVOL_t, adc reference voltage.
 * @param[in] resolution   - enum ADC_RESOLUTION_t
 * @param[in] sample_cycle - enum ADC_SAMPCYC_t
 * @return    none
 */
void adc_Init(ADC_CLK_t adc_clock, ADC_INPUTCHN_t chn, ADC_INPUTMODE_t mode, ADC_REFVOL_t ref_vol, ADC_RESOLUTION_t resolution, ADC_SAMPCYC_t sample_cycle)
{
	/**set adc clock and enable adc clock**/
	adc_SetClkFreq(adc_clock);

	/**select the input channel**/
	adc_AnaChSet(chn);
	/**set the adc's mode**/
	adc_AnaModeSet(mode);
	/**set the reference voltage**/
	adc_RefVoltageSet(ref_vol);
	/**set resolution**/
	adc_ResSet(resolution);
	/**set sample cycle**/
	adc_SampleTimeSet(sample_cycle);

	EN_MANUALM;      // enable manual mode
}

u16 adc_SampleValueGet(void)  //adc_get
{
	reg_adc_chn1_outp = FLD_ADC_CHN_MANU_START;  // Set a run signal,start to run adc

	sleep_us(5);   // wait for data

	return (reg_adc_dat_byp_outp & 0x3FFF);// read data
}

/* between start and end must > 200us
 * ��������8266 A2оƬ
 * power down step��
 * 	1, change input to GND
 * 	2, lower adc clk
 * 	3, start adc
 * 	4, wait > 200us(we can do something else)
 * 	5, start adc again
 * 	6, wait adc latch state,the interval calculation base on system clk and adc clk
 * 	7, power down adc in analog register
 * 	9, recover adc setting
 * */
void adc_power_down(void){
	u8 adc_m_input = reg_adc_chn_m_sel;
	u8 adc_step_low = reg_adc_step_l;
	/*step 0*/
	write_reg8(0x2c,0x12);  //chnm: GND
	write_reg8(0x69,0x01);  //set adc clk = 192M/(192*4) = 0.25K
	write_reg8(0x35,0x80);  //start

	sleep_us(200);
	/*step 1*/
	write_reg8(0x35,0x80);  //start again
	sleep_us(26);             //14 us
	analog_write(0x06,analog_read(0x06) | 0x01);
	/*step 2*/
	reg_adc_chn_m_sel = adc_m_input;
	reg_adc_step_l = adc_step_low;//set adc clk= 192M * 16/(192 *4) = 4M
}

void adc_power_down_start(void){

	write_reg8(0x2c,0x12);  //chn: GND
	write_reg8(0x69,0x01);  //set adc clk = 192M/(192*4) = 0.25K
	write_reg8(0x35,0x80);  //start
}
void adc_power_down_end(void){
	write_reg8(0x35,0x80);  //start again
	sleep_us(26);             //14 us
	analog_write(0x06,analog_read(0x06) | 0x01);
}
void adc_setting_recover(void){
#if 1
	reg_adc_step_l = adc_clk_step_l;
	reg_adc_chn_m_sel = adc_chn_m_input;
#else
	reg_adc_chn_m_sel = FLD_ADC_CHN_C7;
	reg_adc_step_l = 16;//set adc clk= 192M * 16/(192 *4) = 4M

#endif
}

#endif

