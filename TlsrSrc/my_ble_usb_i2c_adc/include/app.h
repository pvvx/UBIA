/*
 * app.h
 *
 *  Created on: 05.11.2019
 *      Author: pvvx
 */

#ifndef APP_H_
#define APP_H_

void user_init();
void GetNewRegData(void);
void main_usb_loop(void);
void main_ble_loop(void);

extern volatile u8 timer_flg;

#endif /* APP_H_ */
