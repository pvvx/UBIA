/*
 * i2c_dev.c
 *
 *  Created on: 22.02.2020
 *      Author: pvvx
 */
#include "proj/tl_common.h"
#if (USE_I2C_DEV)
#include "i2c_dev.h"
// I2C DEV flags
volatile u8 timer_flg = 0; // flag временного отключения чтения I2C regs по irq
u32 t_rd_us = 0; // flag и время в us работы опроса без таймера

// I2C DEV read buffers
#define I2C_BUF_SIZE 256 // equ 256, 512, ...
reg_rd_t * raddr;
u16 i2c_buf[I2C_BUF_SIZE]; // 512 bytes
#if (I2C_BUF_SIZE != 256)
u32 rd_next_cnt = 0;
u32 i2c_buf_wr = 0;
u32 i2c_buf_rd = 0;
#else
u8 rd_next_cnt = 0;
u8 i2c_buf_wr = 0;
u8 i2c_buf_rd = 0;
#endif

// I2C DEV config (store EEP_ID_I2C_CFG)
dev_i2c_cfg_t cfg_i2c;
// default config
dev_i2c_cfg_t def_cfg_i2c = {
		.pktcnt = 2, // max = SMPS_BLK_CNT;
		.multiplier = 0,
		.time = 10000, // us
		.clk_khz = 1000,
		.init[0].dev_addr = 0x80,
		.init[0].reg_addr = 0x00,
		.init[1].data = 0x399f,
		.init[1].dev_addr = 0x00,
		.init[1].reg_addr = 0x00,
		.init[1].data = 0x0000,
		.init[2].dev_addr = 0x00,
		.init[2].reg_addr = 0x00,
		.init[2].data = 0x0000,
		.init[3].dev_addr = 0x00,
		.init[3].reg_addr = 0x00,
		.init[3].data = 0x0000,
		.rd[0].dev_addr = 0x80,
		.rd[0].reg_addr = 0x01,
		.rd[1].dev_addr = 0x80,
		.rd[1].reg_addr = 0x02,
		.rd[2].dev_addr = 0x00,
		.rd[2].reg_addr = 0x00,
		.rd[3].dev_addr = 0x00,
		.rd[3].reg_addr = 0x00,
		.slp[0].dev_addr = 0x80,
		.slp[0].reg_addr = 0x00,
		.slp[0].data = 0x0000,
		.slp[1].dev_addr = 0x00,
		.slp[1].reg_addr = 0x00,
		.slp[1].data = 0x0000
};

/* Flush I2C Buffer */
void FlushI2CBuf(void) {
	u8 r = irq_disable();
	raddr = &cfg_i2c.rd[0]; //	rd_next_cnt = 0;
	i2c_buf_wr = 0;
	i2c_buf_rd = 0;
	irq_restore(r);
}
/* Read Timer (IRQ) Stop */
void Timer_Stop(void) {
	t_rd_us = 0;
	timer_flg = 0;
#if (USE_BLE)
	sleep_mode = 0;
#endif
	reg_tmr1_tick = 0;
	reg_tmr_sta = FLD_TMR_STA_TMR1;
	reg_irq_mask1 &= ~FLD_IRQ_TMR1_EN;
	reg_tmr_ctrl8 &= ~FLD_TMR1_EN;
}

/* Timer Init (IRQ)*/
void Timer_Init(uint32_t period_us) {
#if (USE_BLE)
	sleep_mode = 1; // not sleep
#endif
	reg_tmr1_tick = 0;
//	if(usb_actived) period_us CLOCK in USB!
	reg_tmr1_capt = period_us * CLOCK_SYS_CLOCK_1US;
	reg_tmr_ctrl8 |= FLD_TMR1_EN; // 	FLD_TMR1_MODE = 0
	reg_irq_mask1 |= FLD_IRQ_TMR1_EN;
	reg_tmr_sta = FLD_TMR_STA_TMR1; // clear irq status
}

/* I2c device GetNewRegData
  Called from Timer Irq (!) */
_attribute_ram_code_ void GetNewRegData(void) {
//	all_rd_count++;
	if(I2CBusReadWord(raddr->dev_addr, raddr->reg_addr, &i2c_buf[i2c_buf_wr++])){
		raddr++;
		if(!raddr->dev_addr || raddr > &cfg_i2c.rd[MAX_READ_REGS-1]) {
			raddr = &cfg_i2c.rd[0];
		}
	} else {
		Timer_Stop();
	}
}

_attribute_ram_code_ void TimerIrq(void) {
   	if(reg_irq_src & FLD_IRQ_TMR1_EN) {
		reg_tmr_sta = FLD_TMR_STA_TMR1; // clear irq status
   		reg_irq_src =  FLD_IRQ_TMR1_EN;
   		if(timer_flg)
   			GetNewRegData();
   	}
}


/* I2C Device go Sleep */
void I2CDevSleep() {
	cfg_i2c.pktcnt = 0;
	t_rd_us = 0;
	if(cfg_i2c.slp[0].dev_addr) {
		I2CBusInit(cfg_i2c.clk_khz, 0);
		I2CBusWriteWord(cfg_i2c.slp[0].dev_addr,cfg_i2c.slp[0].reg_addr, cfg_i2c.slp[0].data); // Power-Down (or Shutdown)
		if(cfg_i2c.slp[1].dev_addr)
			I2CBusWriteWord(cfg_i2c.slp[0].dev_addr,cfg_i2c.slp[0].reg_addr, cfg_i2c.slp[0].data); // Power-Down (or Shutdown)
	}
	I2CBusDeInit();
}

// GENERAL CALL RESET: internal reset, similar to a power-on-reset
// Please refer to the Phillips I2C document for more details of the General Call specifications.
static u8 general_call_reset[] = { 0, 0, 0, 6 };
/* I2C Device WakeUp */
void I2CDevWakeUp() {
	I2CBusInit(400, 0);
	I2CBusUtr(0, (i2c_utr_t *)&general_call_reset, 1);
}

/* Init I2C Device
 * t_us < connect interval * 2 -> no sleep
 * connect interval = 10 ?
 *  if t_us < 20000 -> no sleep -> timer On
 *  else set connect interval / 2
 */
int InitI2CDevice(void) {
	u32 i, x;
	u16 clk_khz;
	timer_flg = 0;
#if (USE_USB_CDC && USE_BLE)
	if(usb_actived) {
		i = 50;
		x = 30;
	}
	else
	{
		i = MIN_TSPS_US;
		x = SMPS_BLK_CNT;
	}
#elif (USE_BLE)
	i = MIN_TSPS_US;
	x = SMPS_BLK_CNT;
#elif (USE_USB_CDC)
	i = 50;
	x = 30;
#endif
	if(cfg_i2c.pktcnt > x) cfg_i2c.pktcnt = x;
	clk_khz = cfg_i2c.clk_khz & 0x7fff;
	if(clk_khz < 50 || clk_khz > 2500) {
		clk_khz = 1000;
		cfg_i2c.clk_khz &= 0x8000;
		cfg_i2c.clk_khz |= clk_khz;
	}
	I2CBusInit(clk_khz, cfg_i2c.clk_khz & 0x8000);
	t_rd_us = cfg_i2c.time << cfg_i2c.multiplier;
	// ~ 30 us при 1 MHz I2C CLK (5 us + 25 us)
	if(t_rd_us < i || t_rd_us > 0xffffffff/CLOCK_SYS_CLOCK_1US) {
		cfg_i2c.pktcnt = 0;
		Timer_Stop();
		return 0; // error t_us
	}
	FlushI2CBuf();
	if (cfg_i2c.pktcnt) {
		if(
#if	(USE_BLE && USE_USB_CDC)
			usb_actived ||
#endif
			t_rd_us < 10000) {

			Timer_Init(t_rd_us);
			t_rd_us = 0;
		}
		// start (new) counts
		all_rd_count = 0;
		not_send_count = 0;
	} else {
		Timer_Stop(); // там t_rd_us = 0;
		// выход по Stop (cfg_i2c.rd_count = 0), init dev i2c only
	}
	for(i = 0; i < MAX_INIT_REGS && cfg_i2c.init[i].dev_addr; i++) {
		if (!I2CBusWriteWord(cfg_i2c.init[i].dev_addr, cfg_i2c.init[i].reg_addr, cfg_i2c.init[i].data)) {
			cfg_i2c.pktcnt = 0;
			Timer_Stop();
			return 0; // error dev i2c
		}
	}
	return 1; // ok
}

#endif // USE_I2C_DEV
