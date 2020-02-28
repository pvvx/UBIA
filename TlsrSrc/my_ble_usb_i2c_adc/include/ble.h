/*
 * ble.h
 *
 *  Created on: 22.02.2020
 *      Author: pvvx
 */

#ifndef BLE_H_
#define BLE_H_
#include "proj_lib/rf_drv.h"
#include "proj_lib/pm.h"
#include "proj_lib/ble/ll/ll.h"
#include "proj_lib/ble/hci/hci.h"
#include "proj_lib/ble/blt_config.h"
#include "proj_lib/ble/trace.h"
#include "proj/mcu/pwm.h"
#include "proj_lib/ble/service/ble_ll_ota.h"
#include "proj/drivers/adc.h"
#include "proj/drivers/battery.h"
#include "proj_lib/ble/blt_config.h"
#include "proj_lib/ble/ble_smp.h"
#include "proj_lib/ble/ble_common.h"

typedef struct __attribute__((packed)){
  unsigned short flag;
  /** value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  unsigned short interval;
  /** Number of LL latency connection events (0x0000 - 0x03e8) */
  unsigned short latency;
  /** Connection Timeout (0x000A - 0x0C80 * 10 ms) */
  unsigned short timeout;
} cur_ble_con_t;


extern const attribute_t my_Attributes[];
extern ble_con_t ble_con_ini;
extern cur_ble_con_t cur_ble_con_ini;
extern const ble_con_t def_ble_con_ini;
extern ble_cfg_t ble_cfg_ini;
extern const ble_cfg_t def_ble_cfg_ini;

extern volatile unsigned char sleep_mode;

void ble_init(void);
void main_ble_loop(void);
int module_onReceiveData(void *par);

#endif /* BLE_H_ */
