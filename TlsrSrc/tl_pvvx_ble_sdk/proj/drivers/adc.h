/********************************************************************************************************
 * @file     adc.h 
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
/*
 * adc.h
 *
 *  Created on: 2015-12-10
 *      Author: Telink
 */

#ifndef ADC_H_
#define ADC_H_

#if(__TL_LIB_8266__ || MCU_CORE_TYPE == MCU_CORE_8266)
#include "adc_8266.h"
#elif(__TL_LIB_8263__ || MCU_CORE_TYPE == MCU_CORE_8263)
#include "adc_8263.h"
#elif(__TL_LIB_8267__ || MCU_CORE_TYPE == MCU_CORE_8267)
#include "adc_8267.h"
#elif(__TL_LIB_8269__ || MCU_CORE_TYPE == MCU_CORE_8269)
#include "adc_8269.h"
#elif(__TL_LIB_8258__ || MCU_CORE_TYPE == MCU_CORE_8258)
#include "adc_8258.h"
//#elif(__TL_LIB_8263__ || MCU_CORE_TYPE == MCU_CORE_8263)
//#include "adc_8263.h"
#endif



#endif /* ADC_H_ */
