/*
 * adc_dev.c
 *
 *  Created on: 28.12.2019
 *      Author: pvvx
 */
#include "proj/tl_common.h"
#if USE_INT_ADC
#include "adc_dev.h"

dev_adc_cfg_t cfg_adc;
const dev_adc_cfg_t def_cfg_adc = { // 500 sps, PC4
		.pktcnt = 0,
		.chnl = 9,
		.sps = 500,
		.pga20db = 0,
		.pga2db5 = 0
};

void ADC_Stop(void) {
	cfg_adc.pktcnt  = 0;
	deinit_adc();
}

int InitADCDevice(void) {
	if(cfg_adc.pktcnt) {
		// start (new) counts
		all_rd_count = 0;
		not_send_count = 0;
		if(cfg_adc.pktcnt > SMPS_BLK_CNT)
			cfg_adc.pktcnt = SMPS_BLK_CNT;
		if(!init_adc_dfifo(&cfg_adc)) {
			cfg_adc.pktcnt  = 0;
			return 0;
		}
#if (USE_BLE)
		sleep_mode |= 2;
#endif
	}
	return 1;
}

#endif // USE_INT_ADC
