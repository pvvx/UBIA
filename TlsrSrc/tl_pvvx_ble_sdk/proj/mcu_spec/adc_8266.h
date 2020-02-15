/********************************************************************************************************
 * @file     adc_8266.h
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


#if (__TL_LIB_8266__ || (MCU_CORE_TYPE == MCU_CORE_8266))

#ifndef ADC_8266_H_
#define ADC_8266_H_
#include "../config/user_config.h"

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

/** @addtogroup  HAL_ADC_Module ADC Module
 *  @{
 *  @details	ADC Module can used to sample battery voltage,temperature sensor,mono audio signals.
 */

#include "../common/types.h"

/*****
 *  adc reference voltage
 */
typedef enum{
	ADC_REF_VOL_1V3  = 0x00, //!< ADC Reference:1.3v
	ADC_REF_VOL_AVDD = 0x01, //!< ADC Reference:AVDD
} ADC_REFVOL_t;

/****
 *  adc resolution
 */
typedef enum{
	ADC_SAMPLING_RES_7BIT  = 0,		//!<ADC Sample Resolution Bits:adc_res_bits = 7
	ADC_SAMPLING_RES_9BIT  = 1,		//!<ADC Sample Resolution Bits:adc_res_bits = 9
	ADC_SAMPLING_RES_10BIT = 2,		//!<ADC Sample Resolution Bits:adc_res_bits = 10
	ADC_SAMPLING_RES_11BIT = 3,		//!<ADC Sample Resolution Bits:adc_res_bits = 11
	ADC_SAMPLING_RES_12BIT = 4,		//!<ADC Sample Resolution Bits:adc_res_bits = 12
	ADC_SAMPLING_RES_13BIT = 5,		//!<ADC Sample Resolution Bits:adc_res_bits = 13
	ADC_SAMPLING_RES_14BIT = 7,		//!<ADC Sample Resolution Bits:adc_res_bits = 14
} ADC_RESOLUTION_t;

/*****
 * adc sample cycle
 */
typedef enum{
	ADC_SAMPLING_CYCLE_3   = 0,		//!<Adc Sampling Cycle:3
	ADC_SAMPLING_CYCLE_6   = 1,		//!<Adc Sampling Cycle:6
	ADC_SAMPLING_CYCLE_9   = 2,		//!<Adc Sampling Cycle:9
	ADC_SAMPLING_CYCLE_12  = 3,		//!<Adc Sampling Cycle:12
	ADC_SAMPLING_CYCLE_18  = 4,		//!<Adc Sampling Cycle:18
	ADC_SAMPLING_CYCLE_24  = 5,		//!<Adc Sampling Cycle:24
	ADC_SAMPLING_CYCLE_48  = 6,		//!<Adc Sampling Cycle:48
	ADC_SAMPLING_CYCLE_144 = 7,		//!<Adc Sampling Cycle:144
} ADC_SAMPCYC_t;

/****
 *  ADC analog input channel selection
 */
typedef enum{
	ADC_CHN_D0				= 0x01,
	ADC_CHN_D1				= 0x02,
	ADC_CHN_D2				= 0x03,
	ADC_CHN_D3				= 0x04,
	ADC_CHN_D4				= 0x05,
	ADC_CHN_D5				= 0x06,
	ADC_CHN_C2				= 0x07,
	ADC_CHN_C3				= 0x08,
	ADC_CHN_C4				= 0x09,
	ADC_CHN_C5				= 0x0a,
	ADC_CHN_C6				= 0x0b,
	ADC_CHN_C7				= 0x0c,

	ADC_CHN_PGA_R			= 0x0d,
	ADC_CHN_PGA_L			= 0x0e,

	ADC_CHN_TEMP_POS		= 0x0f,
	ADC_CHN_TEMP_NEG		= 0x10,
	ADC_CHN_VBUS			= 0x11,
	ADC_CHN_GND				= 0x12,
} ADC_INPUTCHN_t;
/***
 * adc mode
 */
typedef enum{
	SINGLEEND = 0,
	INVERTD_5,
	INVERTC_3,
	CHN_PGA_L,
} ADC_INPUTMODE_t;

/****
 * adc clock must be lower than 5Mhz if the reference voltage is selected as AVDD and
 * must be lower than 4Mhz when reference voltage is selected as 1.4v.
 */
typedef enum{
	ADC_CLK_4M = 4,
	ADC_CLK_5M = 5,
} ADC_CLK_t;

/***
 * ADC Done Signal
 */
enum{
	ADC_DONE_SIGNAL_COUNTER,
	ADC_DONE_SIGNAL_RISING,
	ADC_DONE_SIGNAL_FALLING,
};

static inline void adc_SetClkFreq(u8 mhz){
	reg_adc_step_l = mhz*4;
	reg_adc_mod = MASK_VAL(FLD_ADC_MOD, 192*4, FLD_ADC_CLK_EN, 1);
}

static inline void adc_SetPeriod(void){
	reg_adc_period_chn0 = 77;
	reg_adc_period_chn12 = 8;
}

/********************************************************
*
*	@brief		set ADC reference voltage for the Misc and L channel
*
*	@param		adcCha - enum variable adc channel.
*				adcRF - enum variable of adc reference voltage.
*
*	@return		None
*/
static inline void adc_RefVoltageSet(ADC_REFVOL_t adcRF){
	unsigned char ref_vol;

	ref_vol = (unsigned char)adcRF;
	BM_CLR(reg_adc_ref,FLD_ADC_REF_M);
	reg_adc_ref |= MASK_VAL(FLD_ADC_REF_M,ref_vol);
}
/********************************************************
*
*	@brief		set ADC resolution for channel Misc
*
*	@param		adcRes - enum variable adc resolution.
*
*	@return		None
*/
static inline void adc_ResSet(ADC_RESOLUTION_t adcRes){
	unsigned char resN;
	resN = (unsigned char )adcRes;
	BM_CLR(reg_adc_samp_res,FLD_ADC_CHNM_SAMP_RESOL);
	reg_adc_samp_res |= MASK_VAL(FLD_ADC_CHNM_SAMP_RESOL,resN);
}
/********************************************************
*
*	@brief		set ADC sample time(the number of adc clocks for each sample)
*
*	@param		adcCha - enum variable adc channel.
*				adcST - enum variable of adc sample time.
*
*	@return		None
*/

static inline void adc_SampleTimeSet( ADC_SAMPCYC_t adcST){

	unsigned char st;
	st = (unsigned char)adcST;
	BM_CLR(reg_adc_samp_res,FLD_ADC_CHNM_SAMP_CYCLE);
	reg_adc_samp_res |= MASK_VAL(FLD_ADC_CHNM_SAMP_CYCLE,st);
}
/********************************************************
*
*	@brief		set ADC analog input channel
*
*	@param		adcCha - enum variable adc channel.
*				adcInCha - enum variable of adc input channel.
*
*	@return		None
*/
static inline void adc_AnaChSet(ADC_INPUTCHN_t adcInCha){
	unsigned char cnI;

	cnI = (unsigned char)adcInCha;
	BM_CLR(reg_adc_chn_m_sel,FLD_ADC_CHN_SEL);
	reg_adc_chn_m_sel |= MASK_VAL(FLD_ADC_CHN_SEL,cnI);
}
/********************************************************
*
*	@brief		set ADC input channel mode - signle-end or differential mode
*
*	@param		adcCha - enum variable adc channel.
*				inM - enum variable of ADCINPUTMODE.
*
*	@return		None
*/

static inline void adc_AnaModeSet( ADC_INPUTMODE_t inM){
	unsigned char cnM;

	cnM = (unsigned char)inM;
	BM_CLR(reg_adc_chn_m_sel,FLD_ADC_DIFF_CHN_SEL);
	reg_adc_chn_m_sel |= MASK_VAL(FLD_ADC_DIFF_CHN_SEL,cnM);
}
/***
 * Audio Mode
 */
//enum{
//	ADC_AUDIO_MODE_NONE,		//!< ADC Audio Mode Uselessly
//	ADC_AUDIO_MODE_MONO,		//!< ADC Audio Mono Mode
//	ADC_AUDIO_MODE_STEREO,		//!< ADC Audio Stereo Mode,8266 not supported this mode.
//};

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
void adc_Init(ADC_CLK_t adc_clock,ADC_INPUTCHN_t chn,ADC_INPUTMODE_t mode,ADC_REFVOL_t ref_vol,\
		      ADC_RESOLUTION_t resolution,ADC_SAMPCYC_t sample_cycle);

/**
 * @brief  		Start ADC and get the channel misc converter result
 *
 * @param		None
 *
 * @return  	ADC Result
 */
u16 adc_SampleValueGet(void);

/******
 *
 */
void adc_power_down(void);
void adc_power_down_start(void);
void adc_power_down_end(void);
void adc_setting_recover(void);

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif


#endif /* ADC_8266_H_ */
#endif /*#if(__TL_LIB_8266__ || (MCU_CORE_TYPE == MCU_CORE_8266))*/
