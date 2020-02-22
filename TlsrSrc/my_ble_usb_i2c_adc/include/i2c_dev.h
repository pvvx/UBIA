/*
 * i2c_dev.h
 *
 *  Created on: 22.02.2020
 *      Author: pvvx
 */

#ifndef I2C_DEV_H_
#define I2C_DEV_H_

#if (USE_I2C_DEV)

#include "i2cbus.h"
#include "eep_id.h"

// I2C DEV flags
extern volatile u8 timer_flg; // flag временного отключения чтения I2C regs по irq
extern u32 t_rd_us; // flag и время в us работы опроса без таймера

// I2C DEV config (store EEP_ID_I2C_CFG)
extern dev_i2c_cfg_t cfg_i2c;
// default config
extern dev_i2c_cfg_t def_cfg_i2c;
// I2C DEV read buffers
#define I2C_BUF_SIZE 256 // equ 256, 512, ...

extern u16 i2c_buf[I2C_BUF_SIZE]; // 512 bytes
#if (I2C_BUF_SIZE != 256)
extern u32 rd_next_cnt;
extern u32 i2c_buf_wr;
extern u32 i2c_buf_rd;
#else
extern u8 rd_next_cnt;
extern u8 i2c_buf_wr;
extern u8 i2c_buf_rd;
#endif

/* Flush I2C Buffer */
void FlushI2CBuf(void);
/* Read Timer (IRQ) Stop */
void Timer_Stop(void);
/* Timer Init (IRQ)*/
void Timer_Init(uint32_t period_us);
/* I2c device GetNewRegData
  Called from Timer Irq (!) */
void GetNewRegData(void);
/* I2C Device go Sleep */
void I2CDevSleep();
/* I2C Device WakeUp */
void I2CDevWakeUp();
/* Init I2C Device */
int InitI2CDevice(void);

#endif // USE_I2C_DEV
#endif /* I2C_DEV_H_ */
