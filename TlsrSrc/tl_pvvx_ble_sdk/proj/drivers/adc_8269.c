/********************************************************************************************************
 * @file     adc_8269.c 
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
#include "../tl_common.h"

#include "../mcu/register.h"
#include "../common/assert.h"
#include "../common/static_assert.h"
#include "../mcu/analog.h"
#include "syshw.h"
#include "adc.h"
#include "../mcu/clock.h"
#include "dfifo.h"
#include "../common/compatibility.h"
#include "../common/utility.h"
#include "../../proj/mcu/Gpio.h"

#if(/*__TL_LIB_8269__ || */(MCU_CORE_TYPE == MCU_CORE_8269))

#define     ADC_AUTO_MODE_EN    1   // for ADC_SET_CHN_ENABLE

//set period for Misc
#define		SET_P(v)			write_reg16(0x800030,(v<<2)&0x0FFF)

#define		EN_ADCCLK			(*(volatile unsigned char  *)0x80006b |= 0x80)
//set period for Misc
#define		SET_PFM(v)			write_reg16(0x800030,v)
//set period for L
#define		SET_PFL(v)			write_reg8(0x800032,v)
/*
//Skip R period
#define		SKIP_RP 			{		SETB(0x800033,0x10);\
										CLRB(0x800033,0x20);}
										*/
//Enable auto mode, v is the ADC channel
#define		EN_AUTOM(v)			(*(volatile unsigned char  *)0x800033 |= (1<<(3-v*3)))
//Select ADC mannul mode
#define		EN_MANUALM			write_reg8(0x800033,0x10)
//Enable audio output
#define		EN_AUDIOO			(*(volatile unsigned char  *)0x800033 |= 0x04)

#define		DIS_AUDIOO			(*(volatile unsigned char  *)0x800033 &= 0xFB)

//Start sampling and conversion process for mannual mode
#define		STARTSAMPLING		write_reg8(0x800035,0x80)

//Read sampling data
#define		READOUTPUTDATA		read_reg16(0x800038)


//Check adc status, busy return 1
#define		CHECKADCSTATUS		(((*(volatile unsigned char  *)0x80003a) & 0x01) ? 1:0)	
#define		CHECKADCSTATUS_DELAY    do{delay(5);}while(0)    // at least 4us for S_144

/********************************************************
*
*	@brief		set ADC reference voltage for the Misc and L channel
*
*	@param		adcCha - enum variable adc channel.
*				adcRF - enum variable of adc reference voltage.
*
*	@return		None
*/
void adc_RefVoltageSet(enum ADCCHANNEL adcCha, enum ADCRFV adcRF){
	unsigned char cn,st;
	cn = (unsigned char)adcCha;
	st = (unsigned char)adcRF;
	if(cn){
		*(volatile unsigned char  *)0x80002b &= 0xF3;
	
		*(volatile unsigned char  *)0x80002b |= (st<<2);
		}
	else{
		*(volatile unsigned char  *)0x80002b &= 0xFC;
	
		*(volatile unsigned char  *)0x80002b |= st;
		}
}


/********************************************************
*
*	@brief		set ADC resolution for channel Misc
*
*	@param		adcRes - enum variable adc resolution.
*
*	@return		None
*/
void adc_ResMiscSet(enum ADCRESOLUTION adcRes){
	unsigned char resN;
	resN = (unsigned char )adcRes;
	*(volatile unsigned char  *)0x80003c &= 0xC7;
	*(volatile unsigned char  *)0x80003c |= (resN<<3);
}

/********************************************************
*
*	@brief		set ADC resolution for channel L
*
*	@param		adcRes - enum variable adc resolution.
*
*	@return		None
*/
void adc_ResLSet(enum ADCRESOLUTION adcRes){
	unsigned char resN;
	resN = (unsigned char )adcRes;
	*(volatile unsigned char  *)0x80002f &= 0xF8;
	
	*(volatile unsigned char  *)0x80002f |= resN;

}

unsigned short adc_SampleValueGet(void){
#if ADC_AUTO_MODE_EN
    unsigned short sampledValue;

    while(!CHECKADCSTATUS);

    while(CHECKADCSTATUS);

    sampledValue = READOUTPUTDATA & 0x3FFF;

    return sampledValue;
    
#else
	//unsigned short sampledValue;

	STARTSAMPLING;

	
	while(CHECKADCSTATUS);
	CHECKADCSTATUS_DELAY;
		
	return (READOUTPUTDATA & 0x3FFF);
#endif
}

void adc_ResSet(enum ADCRESOLUTION adcRes){
	unsigned char resN;
	resN = (unsigned char )adcRes;
	*(volatile unsigned char  *)0x80003c &= 0xC7;
	*(volatile unsigned char  *)0x80003c |= (resN<<3);
}

void adc_ChannelInit(enum ADCINPUTCH channel,
	                enum ADCINPUTMODE mode,
	                enum ADCRFV ref,
	                enum ADCRESOLUTION res,
	                enum ADCST time) 
{
	/***1.set adc mode and input***/

	adc_AnaChSet(0,channel);
	adc_AnaModeSet(0,mode);
	
	/***2.set adc reference voltage***/	
	adc_RefVoltageSet(0,ref);     //Set reference voltage (V_REF)as  1.428V
	
	/***3.set adc resultion***/
	adc_ResSet(res);               //Set adc resolution to 14 bits, bit[14] to bit bit[1]
	
	/***4.set adc sample time***/
	adc_SampleTimeSet(0,time);          //set sample time

	#if ADC_AUTO_MODE_EN
	/***5.enable auto mode***/
	write_reg8(0x800033,0x88);
	#else
	/***5.enable manual mode***/
	write_reg8(0x800033,0x00);
	#endif
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

void adc_SampleTimeSet(enum ADCCHANNEL adcCha, enum ADCST adcST){

	unsigned char cn,st;
	cn = (unsigned char)adcCha;
	st = (unsigned char)adcST;
	
	*(volatile unsigned char  *)(0x80003c+cn) &= 0xF8;
	
	*(volatile unsigned char  *)(0x80003c+cn) |= st;
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
void adc_AnaChSet(enum ADCCHANNEL adcCha, enum ADCINPUTCH adcInCha){
	unsigned char cn,cnI;
	cn = (unsigned char)adcCha;
	cnI = (unsigned char)adcInCha;

	*(volatile unsigned char  *)(0x80002c+cn) &= 0xE0;
	*(volatile unsigned char  *)(0x80002c+cn) |= cnI;
}

/***************************************************************************
*
*	@brief	set IO power supply for the 1/3 voltage division detection, there are two input sources of the 
*			IO input battery voltage, one through the VDDH and the other through the  ANA_B<7> pin
*
*	@param	IOp - input power source '1' is the VDDH; '2' is the ANA_B<7>.
*
*	@return	'1' setting success; '0' set error
*/
unsigned char adc_IOPowerSupplySet(unsigned char IOp){
	unsigned char vv1;
	if(IOp>2||IOp<1){
		return 0;
	}
	else{
		vv1 = ReadAnalogReg(0x02);
		vv1 = vv1 & 0xcf;
		vv1 = vv1 | (IOp<<4);
		WriteAnalogReg(0x02,vv1);
		return 1;
	}
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
void adc_AnaModeSet(enum ADCCHANNEL adcCha, enum ADCINPUTMODE inM){
	unsigned char cn,cnM;
	cn = (unsigned char)adcCha;
	cnM = (unsigned char)inM;
	*(volatile unsigned char  *)(0x80002c+cn) &= 0x1F;
	*(volatile unsigned char  *)(0x80002c+cn) |= (cnM<<5);
}

//Set ADC done signal
void	setADCDoneSignal(enum ADCDONESIGNAL sig){
	unsigned char sn;
	sn = (unsigned char)sig;

	*(volatile unsigned char  *)0x800033 &= 0x3F;
	*(volatile unsigned char  *)0x800033 |= (sn<<6);
}

//Set ADC audio mode
void	setADCAudioMode(enum ADCAUDIOM audioM){
	unsigned char am;
	am = (unsigned char)audioM;

	*(volatile unsigned char  *)0x800033 &= 0xCF;
	*(volatile unsigned char  *)0x800033 |= (am<<4);
}

/**********************************************************************
*	@brief	ADC initiate function, set the ADC clock details (3MHz) and start the ADC clock.
*			ADC clock relys on PLL, if the FHS isn't selected to 192M PLL (probably modified 
*			by other parts codes), adc initiation function will returns error.
*
*	@param	None
*
*	@return	setResult - '1' set success; '0' set error
*/
unsigned char adc_Init(void ){

	unsigned char fhsBL,fhsBH;
	
	write_reg8(0x800069,0x03); // adc clk step as 3MHz
	write_reg8(0x80006a,0xc0); // adc clk divide mode

	fhsBL = read_reg8(0x800070)&0x01;//0x70[0]
	fhsBH = read_reg8(0x800066)&0x80;//0x66[7]
	fhsBL = fhsBL|fhsBH;
	if(fhsBL){//FHS not default set to 192MHz
		return 0;
	}
	write_reg8(0x800070,0x00);// sel adc clk source as 192M pll
	write_reg8(0x80006b,0x80); // adc clk enable
	WriteAnalogReg(0x88,0x0f);// select 192M clk output
	WriteAnalogReg(0x05,0x60);// power on pll
	WriteAnalogReg(0x06,0xfe);// power on sar

	#if ADC_AUTO_MODE_EN
	write_reg16(0x30,20*CLOCK_SYS_CLOCK_1US);    // set auto channel 0 (Misc) period
	#endif
	
	EN_ADCCLK;//Enable adc CLK

#if  ADC_TEMP_ENABLE
	extern void adc_adj_tp_init(void);
	adc_adj_tp_init();
#endif

	return 1;
}

void adc_chn_ref_init()
{
#if ADC_SET_CHN_ENABLE
	adc_ChannelInit(ADC_CHNM_ANA_INPUT_8267, SINGLEEND, ADC_CHNM_REF_SRC_8267, RES14, S_3);
#endif
#if ADC_ENABLE
	adc_BatteryCheckInit(1);	
#endif
}
/********************************************************
*
*	@brief		Initiate function for the battery check function
*
*	@param		checkM - Battery check mode, '0' for battery dircetly connected to chip,
*				'1' for battery connected to chip via boost DCDC
*
*	@return		None
*/
void adc_BatteryCheckInit(unsigned char checkM){
	//unsigned char vv1;
	write_reg8(0x80002c,0x12);//select "1/3 voltage division detection" as single-end input
	
	if(!checkM)	
		adc_IOPowerSupplySet(1);
	else	
		adc_IOPowerSupplySet(2);
		
	write_reg8(0x800033, 0x10);// must set to 0x10 to enable the mannual sample mode
	adc_RefVoltageSet(MISC,RV_1P224);//Set reference voltage (V_REF)as  1.224V
	
	adc_ResMiscSet(RES14);//Set adc resolution to 14 bits, bit[14] to bit bit[1]
	
	adc_SampleTimeSet(MISC,S_3);//set sample time
}
/********************************************************
*
*	@brief		get the battery value
*
*	@param		None
*
*	@return		unsigned long - return the sampling value
*/
unsigned short adc_BatteryValueGet(void){

	unsigned short sampledValue;
	
	STARTSAMPLING;
	
	while(CHECKADCSTATUS);
	CHECKADCSTATUS_DELAY;
	
	sampledValue = READOUTPUTDATA&0x3FFF;
	
	return sampledValue;
}
/********************************************************
*
*	@brief		Initiate function for the temparture sensor
*
*	@param		None
*				
*	@return		None
*/

void adc_TemSensorInit(void){
	adc_RefVoltageSet(MISC,RV_AVDD);
	adc_AnaChSet(MISC,TEMSENSORN);
	adc_ResMiscSet(RES14);
	adc_SampleTimeSet(MISC,S_6);
	EN_MANUALM;
}

/********************************************************
*
*	@brief		get the temperature sensor sampled value
*
*	@param		None
*
*	@return		unsigned short - return the adc sampled value 14bits significants
*/

unsigned short adc_TemValueGet(void){
	unsigned short sampledValue;
	STARTSAMPLING;
	while(CHECKADCSTATUS);
	CHECKADCSTATUS_DELAY;
	sampledValue = (unsigned long)(READOUTPUTDATA & 0x3FFF);		
	return sampledValue;
}

enum ADCINPUTCH adc_get_chn_by_gpio(unsigned int pin)
{
    enum ADCINPUTCH adc_chn = NOINPUT;
    switch(pin){
        default:
            break;
        case GPIO_PC0:
            adc_chn = C0;
            break;
        case GPIO_PC1:
            adc_chn = C1;
            break;
        case GPIO_PC6:
            adc_chn = C6;
            break;
        case GPIO_PC7:
            adc_chn = C7;
            break;
            
        case GPIO_PB0:
            adc_chn = B0;
            break;
        case GPIO_PB1:
            adc_chn = B1;
            break;
        case GPIO_PB2:
            adc_chn = B2;
            break;
        case GPIO_PB3:
            adc_chn = B3;
            break;
        case GPIO_PB4:
            adc_chn = B4;
            break;
        case GPIO_PB5:
            adc_chn = B5;
            break;
        case GPIO_PB6:
            adc_chn = B6;
            break;
        case GPIO_PB7:
            adc_chn = B7;
            break;
    }

    return adc_chn;
}

void adc_switch_chn(unsigned int pin)
{
    adc_AnaChSet(0,adc_get_chn_by_gpio(pin));
    sleep_us(20);   // at least 10us for 8267 / 8269
}

#if ADC_SET_CHN_ENABLE
unsigned int adc_ref_get()
{
    unsigned int v_ref = 3300;
    if (ADC_CHNM_REF_SRC_8269 == RV_AVDD){      // can not use Macro because of enum
        v_ref = 3300;
    }else if(ADC_CHNM_REF_SRC_8269 == RV_1P224){
        v_ref = 1224;
    }else if(ADC_CHNM_REF_SRC_8269 == RV_1P428){
        v_ref = 1428;
    }
    return v_ref;
}

void adc_set_chn_init()
{
	adc_Init();
	adc_ChannelInit(ADC_CHNM_ANA_INPUT_8269, SINGLEEND, ADC_CHNM_REF_SRC_8269, RES14, S_3);
}
#endif

unsigned short adc_val_get()
{
    return (adc_SampleValueGet() & 0x3FFF);
}
#endif

