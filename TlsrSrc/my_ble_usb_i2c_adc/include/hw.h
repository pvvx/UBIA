/*
 * hw.h
 *
 *  Created on: 17.02.2020
 *      Author: pvvx
 */

#ifndef _HW_H_
#define _HW_H_

// reg_prod_id	REG_ADDR16(0x7e)
enum {
  MCU_PROD_ID_8266 = 0x5325,
  MCU_PROD_ID_8267 = 0x5326,
  MCU_PROD_ID_8269 = 0x5327,
} MCU_PROD_ID;

#define reg_mcu_id	REG_ADDR8(0x7e)
enum {
  MCU_PROD_ID__8266 = 0x25,
  MCU_PROD_ID__8267 = 0x26,
  MCU_PROD_ID__8269 = 0x27,
} MCU_PROD__ID;

unsigned int get_fhs_clock_mhz(void);
unsigned int get_sys_clock_khz(void);

#endif /* _HW_H_ */
