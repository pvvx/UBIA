/*
 * hx711.h
 *
 *  Created on: 26.12.2019
 *      Author: pvvx
 */

#ifndef HX711_H_
#define HX711_H_

typedef enum {
	HX711MODE_A128 = 25,
	HX711MODE_B32,
	HX711MODE_A64
} hx711_mode_t;

//#define HX711_SCK	GPIO_PC0
//#define HX711_DOUT  GPIO_PC1
#define HX711_BUF_CNT 16 // 8,16,32

extern u32 hx711_buf[HX711_BUF_CNT]; // 64 bytes

extern u8 hx711_wr;
extern u8 hx711_rd;
extern u8 hx711_mode;

//_attribute_ram_code_
int hx711_get_data(hx711_mode_t mode);
void hx711_go_sleep(void);

inline void hx711_gpio_go_sleep(void) {
//	bls_app_registerEventCallback(BLT_EV_FLAG_GPIO_EARLY_WAKEUP, NULL);
//	gpio_set_wakeup(HX711_DOUT, 0, 0); // отключить пробуждение от hx711
	gpio_setup_up_down_resistor(HX711_SCK, PM_PIN_PULLUP_1M);
}

inline void hx711_gpio_wakeup(void) {
	gpio_setup_up_down_resistor(HX711_SCK, PM_PIN_PULLDOWN_100K);
//	gpio_setup_up_down_resistor(HX711_DOUT, PM_PIN_UP_DOWN_FLOAT);
//	bls_app_registerEventCallback(BLT_EV_FLAG_GPIO_EARLY_WAKEUP, &ble_ev_gpio_wakeup);
//	gpio_set_wakeup(HX711_DOUT, 0, 1);  // core(gpio) low wakeup suspend
}




#endif /* HX711_H_ */
