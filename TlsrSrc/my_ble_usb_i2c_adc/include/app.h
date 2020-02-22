/*
 * app.h
 *
 *  Created on: 05.11.2019
 *      Author: pvvx
 */

#ifndef APP_H_
#define APP_H_

void user_init();

void ExtDevPowerOn();
void ExtDevPowerOff();

static inline u32 clock_tik_exceed(u32 ref, u32 span_us){
	return ((u32)(clock_time() - ref) > span_us);
}

#endif /* APP_H_ */
