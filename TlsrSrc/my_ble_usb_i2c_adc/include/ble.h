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

extern ble_con_t ble_con_ini;
extern ble_con_t cur_ble_con_ini;
extern const ble_con_t def_ble_con_ini;
extern ble_adv_t ble_adv_ini;
extern const ble_adv_t def_ble_adv_ini;

extern volatile unsigned char sleep_mode;

void ble_init(void);
void main_ble_loop(void);

#endif /* BLE_H_ */
