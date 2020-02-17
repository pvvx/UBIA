/********************************************************************************************************
 * @file     audio.h
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

#if 1 /* (__TL_LIB_8267__ || (MCU_CORE_TYPE == MCU_CORE_8267) || \
	__TL_LIB_8261__ || (MCU_CORE_TYPE == MCU_CORE_8261) || \
	__TL_LIB_8269__ || (MCU_CORE_TYPE == MCU_CORE_8269)) */
#ifndef audio_H
#define audio_H

//down sample rate
enum audio_deci_t{
	R1,R2,R3,R4,R5,R6,
	R7,R8,R16,R32,R64,R128,
};

enum audio_mode_t{
	DIFF_MODE,
	SINGLE_END,
};
enum audio_input_t{
	PGA_CH = 0,
	AUD_C0,
	AUD_C1,
	AUD_C6,
	AUD_C7,
	AUD_B0,
	AUD_B1,
	AUD_B2,
	AUD_B3,
	AUD_B4,
	AUD_B5,
	AUD_B6,
	AUD_B7,
	AUD_PGAVOM,
	AUD_PGAVOP,
};
enum {
	AUD_SINGLEEND,
	AUD_INVERTB_1,
	AUD_INVERTB_3,
	AUD_PGAVOPM,
};
enum{
	NO_AUDIO = 0,
	MONO_AUDIO,
	STEREO_AUDIO,
};
enum{
	AUD_VOLUME_MANUAL,
	AUD_VOLUME_AUTO,
};
enum{
	AUD_ADC_DONE_RISING = 1,
	AUD_ADC_DONE_FALLING = 2,
};

#define MANUAL_VOLUMN_SETTINGS			0x20
#define AUDIO_HPF_SHIFT                 0x09

/************************************************************
*	@param		m:		audio input mode, '1' diff; '0' single end
*				b:		battery check mode bit[7], battery input channel bit[5:0]
*
*/
#define AUDIOSE_SYSCLK48M_96KADC_16KSDM(m,b)			Audio_Init(m,b,AMIC,61,16,R6)

#define AUDIOSE_SYSCLK32M_96KADC_16KSDM(m,b)			Audio_Init(m,b,AMIC,50,8,R6)

#define AUDIOSE_SYSCLK24M_96KADC_16KSDM(m,b)			Audio_Init(m,b,AMIC,38,6,R6)

#define AUDIOSE_SYSCLK16M_48KADC_16KSDM(m,b)			Audio_Init(m,b,AMIC,35,12,R3)


#define MIC_FIFO_WPTR_EN()  	do{\
									BM_SET(reg_dfifo_ana_in,FLD_DFIFO_AUD_INPUT_MONO);\
									BM_CLR(reg_dfifo_ana_in,FLD_DFIFO_WPTR_CLR);\
								}while(0);

#define MIC_FIFO_WPTR_DIS()		do{\
									BM_CLR(reg_dfifo_ana_in,FLD_DFIFO_AUD_INPUT_MONO);\
									BM_SET(reg_dfifo_ana_in,FLD_DFIFO_WPTR_CLR);\
								}while(0);



#define DMIC_CFG_GPIO_PA0_PA1()    do{\
								       *(volatile unsigned char  *)0x800586 &= 0xfc;\
									   *(volatile unsigned char  *)0x8005b0 |= 0x01;\
								    }while(0)

/**
 * @brief     configure the mic buffer's address and size
 * @param[in] pbuff - the first address of SRAM buffer to store MIC data.
 * @param[in] size_buff - the size of pbuff.
 * @return    none
 */
void audio_config_mic_buf(signed short* pbuff,unsigned int size_buff);
/**
 * @brief     configure the SDM buffer's address and size
 * @param[in] pbuff - the first address of buffer SDM read data from.
 * @param[in] size_buff - the size of pbuff.
 * @return    none
 */
void audio_config_sdm_buf(signed short* pbuff, unsigned int size_buff);

/****
* brief: audio amic initial function. configure ADC corresponding parameters. set hpf,lpf and decimation ratio.
* param[in] mode_flag -- '1' differ mode ; '0' signal end mode
* param[in] misc_sys_tick -- system ticks of adc misc channel.
* param[in] l_sys_tick -- system tick of adc left channel
*/
void audio_amic_init(enum audio_mode_t mode_flag,unsigned short misc_sys_tick, unsigned short left_sys_tick,enum audio_deci_t d_samp,unsigned char fhs_source);

/************************************************************************************
*
*	@brief	audio input set function, select analog audio input channel, start the filters
*
*	@param	adc_ch:	if audio input as signle end mode, should identify an analog audio signal input channel, enum variable of ADCINPUTCH
*
*	@return	none
*/
void audio_amic_input_set(enum audio_input_t adc_ch);

/**
 * @brief     audio DMIC init function, config the speed of DMIC and downsample audio data to required speed.
 *            actually audio data is dmic_speed/d_samp.
 * @param[in] dmic_speed - set the DMIC speed. such as 1 indicate 1M and 2 indicate 2M.
 * @param[in] d_samp - set the decimation. ie div_speed.
 * @param[in]  fhs_source - the parameter is CLOCK_SYS_TYPE. avoid CLOCK_SYS_TYPE to be modified to other word.such as SYS_TYPE etc.
 *
 * @return    none.
 */
void audio_dmic_init(unsigned char dmic_speed, enum audio_deci_t d_samp,unsigned char fhs_source);

/**
*	@brief		reg0x30[1:0] 2 bits for fine tuning, divider for slow down sample rate
*	@param[in]	fine_tune - unsigned char fine_tune,range from 0 to 3
*	@return	    none
*/
void audio_finetune_sample_rate(unsigned char fine_tune);

/**
 *  @brief      tune decimation shift .i.e register 0xb04 in datasheet.
 *  @param[in]  deci_shift - range from 0 to 5.
 *  @return     none
 */
unsigned char audio_tune_deci_shift(unsigned char deci_shift);

/**
 *   @brief       tune the HPF shift .i.e register 0xb05 in datasheet.
 *   @param[in]   hpf_shift - range from 0 to 0x0f
 *   @return      none
 */
 unsigned char audio_tune_hpf_shift(unsigned char hpf_shift);

 /**
 *
 *	@brief	   sdm setting function, enable or disable the sdm output, configure SDM output paramaters
 *
 *	@param[in]	audio_out_en - audio output enable or disable set, '1' enable audio output; '0' disable output
 *	@param[in]	sample_rate - audio sampling rate, such as 16K,32k etc.
 *	@param[in]	sdm_clk -	  SDM clock, default to be 8Mhz
 *	@param[in]  fhs_source - the parameter is CLOCK_SYS_TYPE. avoid CLOCK_SYS_TYPE to be modified to other word.such as SYS_TYPE etc.
 *
 *	@return	none
 */
void audio_sdm_output_set(unsigned char audio_out_en,int sample_rate,unsigned char sdm_clk,unsigned char fhs_source);

/**
*	@brief	    set audio volume level
*	@param[in]	input_out_sel - select the tune channel, '1' tune ALC volume; '0' tune SDM output volume
*	@param[in]	volume_level - volume level
*	@return	    none
*/
void audio_volume_tune(unsigned char input_out_sel, unsigned char volume_level);

/*************************************************************
*
*	@brief	automatically gradual change volume
*
*	@param[in]	vol_step - volume change step, the high part is decrease step while the low part is increase step
*			    gradual_interval - volume increase interval
*
*	@return	none
*/
void audio_volume_step_adjust(unsigned char vol_step,unsigned short gradual_interval);

#endif
#endif
