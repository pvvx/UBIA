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

//_attribute_ram_code_
int hx711_get_data(hx711_mode_t mode);

#endif /* HX711_H_ */
