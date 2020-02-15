/*
 * ina2xx.h
 *
 *  Created on: 14.01.2020
 *      Author: pvvx
 */

#ifndef _I2CBUS_H_
#define _I2CBUS_H_

/* Universal I2C/SMBUS read-write transaction struct */
typedef struct _i2c_utr_t {
	unsigned char mode;	// bit0..6: number wr_byte for new START (bit7: =1 - generate STOP/START)
	unsigned char rdlen; // bit7: =1 - old read byte generate NACK, =0 - ACK
	unsigned char wrdata[1]; // i2c_addr_wr, wr_byte1, wr_byte2, wr_byte3, ... wr_byte126
} i2c_utr_t;

void I2CBusInit(unsigned int clk);
void I2CBusDeInit(void);
int I2CBusReadWord(unsigned char i2c_addr, unsigned char reg_addr, void *preg_data);
int I2CBusWriteWord(unsigned char i2c_addr, unsigned char reg_addr, unsigned short reg_data);
int I2CBusUtr(void * outdata, i2c_utr_t *tr, unsigned int wrlen);

#endif /* _I2CBUS_H_ */
