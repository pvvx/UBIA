/*
 * lfloader.c
 *
 *  Created on: 07.03.2020
 *      Author: pvvx
 */

#include "proj/tl_common.h"
#include "lfloader.h"

void load_floader(void) {
	uint32_t * p = (uint32_t *)FLOADER_FADDR;
	if(gpio_read(GPIO_SWS) == 0 && p[2] == 0x544c4e4b) {
		memcpy((void *)0x808000, (void *)p, FLOADER_SIZE);
		REG_ADDR8(0x602) = 0x88;
		while(1);
	}
}
