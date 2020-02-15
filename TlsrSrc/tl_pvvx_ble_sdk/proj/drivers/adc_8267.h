/********************************************************************************************************
 * @file     adc_8267.h 
 *
 * @brief    for TLSR chips
 *
 * @author	 telink
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

#pragma once

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#if(/*__TL_LIB_8267__ || */(MCU_CORE_TYPE == MCU_CORE_8267))

//ADC channel
enum ADCCHANNEL{
	MISC,			
	LCHANNEL,		
};

//ADC reference voltage
enum ADCRFV{
	RV_1P428,		
	RV_AVDD,		
	RV_1P224,		
};
//ADC resolution 
enum ADCRESOLUTION{
	RES7,	
	RES9,
	RES10,
	RES11,
	RES12,
	RES13,
	RES14,
};

//ADC Sampling time
enum ADCST{
	S_3,			
	S_6,
	S_9,
	S_12,
	S_18,
	S_24,
	S_48,
	S_144,
};

//ADC analog input channel selection enum
enum ADCINPUTCH{
	NOINPUT,
	C0,
	C1,
	C6,
	C7,
	B0,
	B1,
	B2,
	B3,
	B4,
	B5,
	B6,
	B7,
	PGAVOM,
	PGAVOP,
	TEMSENSORN,
	TEMSENSORP,
	AVSS,
	OTVDD,//1/3 voltage division detection
};
//ADC channel input mode
enum ADCINPUTMODE{
	SINGLEEND,
	INVERTB_1,
	INVERTB_3,
	PGAVOPM,
};
//ADC done signal
enum ADCDONESIGNAL{
	RISING = 0x03,
	FALING = 0x02,
};

//ADC audio mode
enum ADCAUDIOM{
	NOAUDIO,
	MONO,
};

/********************************************************
*
*	@brief		set ADC resolution for channel Misc
*
*	@param		adcRes - enum variable adc resolution.
*
*	@return		None
*/
extern void adc_ResMiscSet(enum ADCRESOLUTION adcRes);

/********************************************************
*
*	@brief		set ADC resolution for channel L
*
*	@param		adcRes - enum variable adc resolution.
*
*	@return		None
*/
extern void adc_ResLSet(enum ADCRESOLUTION adcRes);

/********************************************************
*
*	@brief		set ADC input channel
*
*	@param		adcCha - enum variable adc channel.
*				adcInCha - enum variable of adc input channel.
*
*	@return		None
*/
extern void adc_AnaChSet(enum ADCCHANNEL adcCha, enum ADCINPUTCH adcInCha);



/***************************************************************************
*
*	@brief	This function must be called when the input channel selected to 1/3 volatage division. 
*			Set IO power supply for the 1/3 voltage division detection, there are two input sources of the 
*			IO input battery voltage, one through the VDDH and the other through the  ANA_B<7> pin.
*
*	@param	IOp - input power source '1' is the VDDH; '2' is the ANA_B<7>.
*
*	@return	'1' set success; '0' set error
*/
extern unsigned char adc_IOPowerSupplySet(unsigned char IOp);

/********************************************************
*
*	@brief		set ADC input channel mode - signle-end or differential mode
*
*	@param		adcCha - enum variable adc channel.
*				inM - enum variable of ADCINPUTMODE.
*
*	@return		None
*/
extern void adc_AnaModeSet(enum ADCCHANNEL adcCha, enum ADCINPUTMODE inM);

/********************************************************
*
*	@brief		set ADC sample time(the number of adc clocks for each sample)
*
*	@param		adcCha - enum variable adc channel.
*				adcST - enum variable of adc sample time.
*
*	@return		None
*/
extern void adc_SampleTimeSet(enum ADCCHANNEL adcCha, enum ADCST adcST);

/********************************************************
*
*	@brief		set ADC reference voltage for the Misc and L channel
*
*	@param		adcCha - enum variable adc channel.
*				adcRF - enum variable of adc reference voltage.
*
*	@return		None
*/
extern void adc_RefVoltageSet(enum ADCCHANNEL adcCha, enum ADCRFV adcRF);


/**********************************************************************
*	@brief	ADC initiate function, set the ADC clock details (3MHz) and start the ADC clock.
*			ADC clock relys on PLL, if the FHS isn't selected to 192M PLL (probably modified 
*			by other parts codes), adc initiation function will returns error.
*
*	@param	None
*
*	@return	setResult - '1' set success; '0' set error
*/
extern unsigned char adc_Init(void );
/********************************************************
*
*	@brief		Initiate function for the battery check function
*
*	@param		checkM - Battery check mode, '0' for battery dircetly connected to chip,
*				'1' for battery connected to chip via boost DCDC
*
*	@return		None
*/
extern void adc_chn_ref_init(void);

extern void adc_BatteryCheckInit(unsigned char checkM);
/********************************************************
*
*	@brief		get the battery value
*
*	@param		None
*
*	@return		unsigned long - return the sampling value multiplex 3
*/
extern unsigned short adc_BatteryValueGet(void);

/********************************************************
*
*	@brief		Initiate function for the temparture sensor
*
*	@param		None
*				
*	@return		None
*/
extern void adc_TemSensorInit(void);
/********************************************************
*
*	@brief		get the temperature sensor sampled value
*
*	@param		None
*
*	@return		unsigned short - return the adc sampled value 14bits significants
*/
extern unsigned short adc_TemValueGet(void);


extern void adc_ChannelInit(enum ADCINPUTCH channel,
	                enum ADCINPUTMODE mode,
	                enum ADCRFV ref,
	                enum ADCRESOLUTION res,
	                enum ADCST time) ;

extern unsigned short adc_SampleValueGet(void);
extern enum ADCINPUTCH adc_get_chn_by_gpio(unsigned int pin);
extern void adc_switch_chn(unsigned int pin);

unsigned int adc_ref_get();
void adc_set_chn_init();
unsigned short adc_val_get();

#endif

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

