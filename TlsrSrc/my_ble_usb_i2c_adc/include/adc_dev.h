/*
 * adc_dev.h
 *
 *  Created on: 22.02.2020
 *      Author: pvvx
 */

#ifndef ADC_DEV_H_
#define ADC_DEV_H_

#if (USE_INT_ADC)
#include "adc.h"

extern dev_adc_cfg_t cfg_adc;
extern const dev_adc_cfg_t def_cfg_adc;

void ADC_Stop(void);
int InitADCDevice(void);

#endif // USE_INT_ADC
#endif /* ADC_DEV_H_ */
