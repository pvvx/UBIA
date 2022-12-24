/**
 ******************************************************************************
 * @file	 main.c
 * @author	 MCD Application Team
 * @version V4.0.0
 * @date	 21-January-2013
 * @brief	 Virtual Com Port Demo main file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *		   http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"

#include "rw_ini.h"
#include "smbus.h"

/* Private typedef -----------------------------------------------------------*/


// CMD_DEV_GRG Get reg I2C
typedef __packed struct {
	uint8_t dev_addr; // адрес на шине i2c
	uint8_t reg_addr; // номер регистра чтения
} reg_rd_t;

// CMD_DEV_SRG  Set reg I2C
typedef __packed struct {
	uint8_t dev_addr; // адрес на шине i2c
	uint8_t reg_addr; // номер регистра чтения
	uint16_t data; // значение для записи в регистр
} reg_wr_t;

#define MAX_INIT_REGS 4
#define MAX_READ_REGS 4

// Структура конфигурации опроса и инициализации устройства
// Выходной пакет непрерывного опроса формируется по данному описанию
// CMD_DEV_CFG Get/Set CFG/ini I2C & Start measure
typedef __packed struct {
	uint8_t pktcnt;  	// кол-во передаваемых значений из регистров в одном пакете передачи 
	uint8_t multiplier; // множитель периода опроса, time << multiplier
	uint16_t time; 		// период опроса регистров чтения в us 
	uint16_t clk_khz; 	// bit0..12: частота i2c шины в кГц 
	reg_wr_t init[MAX_INIT_REGS];
	reg_rd_t rd[MAX_READ_REGS];
	reg_wr_t slp[2];
} dev_cfg_t; 

// DEV command id (v0x21):
#define CMD_DEV_VER  0x00 // Get Ver
// I2C/SBUS cfg
#define CMD_DEV_CFG  0x01 // Get/Set CFG/ini I2C & Start measure
#define CMD_DEV_SCF  0x02 // Store CFG/ini I2C in Flash
// Status
#define CMD_DEV_STA  0x03 // Status
// BLE cfg
//#define CMD_DEV_CPU  0x04 // Connect parameters Update (BLE)
//#define CMD_DEV_ADV  0x05 // Advertising parameters Update (BLE)
//#define CMD_DEV_SBC  0x06 // Store CFG/ini BLE in Flash
// I2C/SMBUS out regs
#define CMD_DEV_I2C  0x07 // blk out regs i2c data
// ADC cfg
#define CMD_DEV_CAD  0x08 // Get/Set CFG/ini ADC & Start measure
#define CMD_DEV_SAD  0x09 // Store CFG/ini ADC in Flash
// ADC out samples
#define CMD_DEV_ADC  0x0A // blk out regs ADC data
// ADC set samples rate
#define CMD_DEV_SPS  0x0B // ADC set samples rate
// I2C rd/wr
#define CMD_DEV_UTR  0x0C // I2C read/write
// Power, Sleep
#define CMD_DEV_PWR  0x0D // Power On/Off, Sleep
// E
#define CMD_DEV_ERR  0x0f // Runtime Error
// I2C/SMBUS rd/wr regs
#define CMD_DEV_GRG  0x10 // Get reg I2C
#define CMD_DEV_SRG  0x11 // Set reg I2C
//
#define CMD_ERR_FLG   0x80 // send error cmd mask

// Runtime Errors
#define RTERR_USER	0x00

// CMD_DEV_ERR  0x0f // Runtime Error
typedef __packed struct _dev_err_t{
	uint16_t id; // тип ошибки (RTERR)
	uint16_t err; // номер/значение ошибки
} dev_err_t;

// CMD_DEV_STA  0x03 // Status
typedef __packed struct  _dev_sta_t{
	uint32_t rd_cnt; // счетчик samples
	uint32_t to_cnt; // счетчик не переданных или timeout при передаче samples
} dev_sta_t;

// Структура конфигурации опроса и инициализации устройства ADC
// Выходной пакет непрерывного опроса формируется по данному описанию
// CMD_DEV_CAD Get/Set CFG/ini ADC & Start measure
/*
typedef __packed struct _dev_adc_cfg_t {
	uint8_t pktcnt;	// минимальное кол-во передаваемых значений ADC в одном пакете передачи (автоподстройка до мах SMPS_BLK_CNT)
	uint8_t chnl; 	// Channel
	uint16_t sps; 	// период adc chl0
	uint8_t pga20db;
	uint8_t pga2db5;
} dev_adc_cfg_t; // [6]
extern dev_adc_cfg_t cfg_adc; // store in eep
*/

typedef  __packed struct{
	uint8_t size; // размер пакета
	uint8_t cmd;  // номер команды / тип пакета
} blk_head_t;
	
typedef  __packed struct _blk_cio_t{
	blk_head_t head;
	__packed union {
		uint8_t uc[VIRTUAL_COM_PORT_DATA_SIZE-sizeof(blk_head_t)];
		int8_t sc[VIRTUAL_COM_PORT_DATA_SIZE-sizeof(blk_head_t)];
		uint16_t ui[(VIRTUAL_COM_PORT_DATA_SIZE-sizeof(blk_head_t))/sizeof(uint16_t)];
		int16_t si[(VIRTUAL_COM_PORT_DATA_SIZE-sizeof(blk_head_t))/sizeof(int16_t)];
		uint32_t ud[(VIRTUAL_COM_PORT_DATA_SIZE-sizeof(blk_head_t))/sizeof(uint32_t)];
		int32_t sd[(VIRTUAL_COM_PORT_DATA_SIZE-sizeof(blk_head_t))/sizeof(uint32_t)];
		reg_wr_t reg;
		reg_rd_t rdr;
		dev_cfg_t cfg;
		dev_err_t err;
		dev_sta_t sta;
	} data;
} blk_cio_t;

/* Private define ------------------------------------------------------------*/
#define Timer_SetPreriod_us(x)	TIM2->ARR = x-1 /* Set the Autoreload value */
#define Timer_Stop()		TIM_Cmd(TIM2, DISABLE);
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
dev_cfg_t cfg_i2c;
reg_rd_t *raddr; // = &cfg_i2c.rd[0];
#define I2C_BUF_SIZE 256
uint16_t i2c_buf[I2C_BUF_SIZE];
#if I2C_BUF_SIZE == 256
uint8_t i2c_buf_rd;
volatile uint8_t i2c_buf_wr;
#else
uint32_t i2c_buf_rd;
volatile uint32_t i2c_buf_wr;
#endif

blk_cio_t send_pkt;

uint32_t not_send_count;
uint32_t all_rd_count;

volatile uint8_t timer_flg;
volatile uint8_t send_err = 0;

volatile uint32_t packet_sent = 1;
volatile uint32_t packet_receive = 1;

/* Extern variables ----------------------------------------------------------*/
extern __IO uint8_t Receive_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];
extern __IO uint32_t Receive_length;

void *memcpy2(void *dest, void *src, uint16_t len) {
	uint8_t *dest1 = dest;
	uint8_t *src1 = (uint8_t *) src;
	while (len--)
		*dest1++ = *src1++;
	return dest;
}

/* Code ----------------------------------------------------------------------*/

/*******************************************************************************
 * Function Name : Timer_Init.
 * Descriptioan  : Инициализация таймера с периодом 100 us.
 * Input		 : None.
 * Output		 : None.
 * Return		 : None.
 *******************************************************************************/
void Timer_Init(uint16_t period_us, uint16_t multiplier) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = period_us - 1;// 1 MHz down to 10 KHz (100 us)
	TIM_TimeBaseStructure.TIM_Prescaler = (72 << multiplier) - 1; // 72 MHz Clock down to 1 MHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
//	timer_count = 0;
}
/*******************************************************************************
 * Function Name : FlushI2CBuf
 * Descriptioan	 : Очистка буферов
 * Input		 : None.
 * Output		 : None.
 * Return		 : None.
 *******************************************************************************/
void FlushI2CBuf(void) {
	u8 r = timer_flg;
	timer_flg = 0;
	raddr = &cfg_i2c.rd[0]; //	rd_next_cnt = 0;
	i2c_buf_wr = 0;
	i2c_buf_rd = 0;
	timer_flg = r;
}
/*******************************************************************************
 * Function Name : GetNewRegData.
 * Descriptioan	 : Наполнение буферов передачи данных.
 * Input		 : None.
 * Output		 : None.
 * Return		 : None.
 *******************************************************************************/
void GetNewRegData(void) {
	if (SMBusReadWord(raddr->dev_addr, &i2c_buf[i2c_buf_wr++],
				raddr->reg_addr)) {
		#if (I2C_BUF_SIZE != 256)
		i2c_buf_wr &= I2C_BUF_SIZE - 1;
		#endif
		raddr++;
		if(!raddr->dev_addr || raddr > &cfg_i2c.rd[MAX_READ_REGS-1]) {
			raddr = &cfg_i2c.rd[0];
		}
	} else {
		Timer_Stop();
		send_err = 2;
	}
}
/*
void GetNewRegData(void) {
	reg_rd_t *raddr;
	blk_cio_t *p = &Send_Buffer1;

	if (rd_next_cnt >= cfg_i2c.rd_count)
		rd_next_cnt = 0;
	raddr = &cfg_i2c.rd[rd_next_cnt];
	rd_next_cnt++;
	if (SMBusReadWord(raddr->dev_addr, (uint16_t *)&p->data.ui[wr_data_cnt],
				raddr->reg_addr)) {
		all_send_count++;			
		wr_data_cnt++;
		if (wr_data_cnt >= wr_data_max) {
			wr_data_cnt = 0;
			rd_next_cnt = 0;
			if(Send_Flg == 0) {
				memcpy2(&Send_BufferO, &Send_Buffer1, sizeof(Send_BufferO));
				Send_Flg = 1;
			}
			else {
				++not_send_count;
				if (not_send_count > 10) { 
					Timer_Stop();
					send_err = 1;
				}
			}
		}
	}
	else {
		Timer_Stop();
		send_err = 2;
	}
}
*/

/*******************************************************************************
 * Function Name : InitExtDevice.
 * Descriptioan  : Инициализация таймера опроса и устройва на SMBus
 * Input		 : None.
 * Output		 : None.
 * Return		 : None.
 *******************************************************************************/
uint8_t InitExtDevice(void) {
	int i;
	if(cfg_i2c.pktcnt > sizeof(send_pkt.data.ui)) 
		cfg_i2c.pktcnt = sizeof(send_pkt.data.ui);
	cfg_i2c.clk_khz &= 0x1fff; 
	if(cfg_i2c.clk_khz < 100 || cfg_i2c.clk_khz > 1200) 
		cfg_i2c.clk_khz = SMBus_Speed/1000;
	SMBusInit(cfg_i2c.clk_khz * 1000);
	if(cfg_i2c.time < 63) {
		Timer_Stop();
		cfg_i2c.pktcnt = 0;
		return 0;
	}
	FlushI2CBuf();
	if (cfg_i2c.pktcnt) {
		Timer_Init(cfg_i2c.time, cfg_i2c.multiplier);
		// start (new) counts
		all_rd_count = 0;
		not_send_count = 0;
	} else {
		Timer_Stop(); // там t_rd_us = 0;
		// выход по Stop (cfg_i2c.pktcnt = 0), init dev i2c only
	}
	FlushI2CBuf();
	for(i = 0; i < MAX_INIT_REGS && cfg_i2c.init[i].dev_addr; i++) {
		if (!SMBusWriteWord(cfg_i2c.init[i].dev_addr, cfg_i2c.init[i].data,	cfg_i2c.init[i].reg_addr)) {
			cfg_i2c.pktcnt = 0;
			Timer_Stop();
			return 0; // error dev i2c
		}
	}
    return 1;
}
/*******************************************************************************
 * Function Name	 : main.
 * Descriptioan	 : Main routine.
 * Input			 : None.
 * Output		 : None.
 * Return		 : None.
 *******************************************************************************/
int main(void) {
	Set_System(); // in hw_config.c
// USB Init
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
	Delay_ms(75); // Wait stabilization power voltage
// CFG Init
	if (!ReadIniBlk(&cfg_i2c, sizeof(cfg_i2c))) {
//		cfg_i2c.rd_count = 0;
//		cfg_i2c.init_count = 0;
		cfg_i2c.time = 400;
		cfg_i2c.multiplier = 0;
		cfg_i2c.clk_khz = 400;
	}
	cfg_i2c.pktcnt = 0;
	SMBusInit(400000);
	SMBusSendPOR();
	InitExtDevice();
	while (1) {
		if(bDeviceState == CONFIGURED) {
			CDC_Receive_DATA();
			if (packet_sent) {
				blk_cio_t * pbufi = (blk_cio_t *) Receive_Buffer;
				if (Receive_length >= sizeof(blk_head_t)
				&& Receive_length >= pbufi->head.size + sizeof(blk_head_t)) {
					/*Check to see if we have data yet */
					switch (pbufi->head.cmd) {
						case CMD_DEV_VER: // Get Ver
							pbufi->data.ui[0] = 0x0016; // DevID = 0x0016
							pbufi->data.ui[1] = 0x0010; // Ver 0.0.1.0 = 0x0010
							Receive_length = 4 + sizeof(blk_head_t);
							break;
						case CMD_DEV_CFG: // Get/Set CFG/ini & Start measure
							if (pbufi->head.size) {
								timer_flg = 0;
								memcpy2(&cfg_i2c, &pbufi->data,
									(pbufi->head.size > sizeof(cfg_i2c))? sizeof(cfg_i2c) : pbufi->head.size);
								if (!InitExtDevice()) {
									pbufi->head.cmd |= CMD_ERR_FLG; // Error cmd
									Receive_length = 0 + sizeof(blk_head_t);
									break;
								} else
									timer_flg = 1;
							} 
							memcpy2(&pbufi->data, &cfg_i2c, sizeof(cfg_i2c));
							Receive_length = sizeof(cfg_i2c) + sizeof(blk_head_t);
							break;
						case CMD_DEV_SCF: // Store CFG/ini in Flash
							WriteIniBlk(&cfg_i2c, sizeof(cfg_i2c));
							Receive_length = sizeof(blk_head_t);
							break;
						case CMD_DEV_STA: // Status
							pbufi->data.sta.rd_cnt = all_rd_count;
							pbufi->data.sta.to_cnt = not_send_count;
							Receive_length = sizeof(dev_sta_t) + sizeof(blk_head_t);
							break;
						//-------
						case CMD_DEV_GRG: // Get reg
							timer_flg = 0;
							if (SMBusReadWord(pbufi->data.reg.dev_addr,
								(uint16_t *)&pbufi->data.reg.data, pbufi->data.reg.reg_addr)) {
								timer_flg = 1;
								Receive_length = sizeof(reg_wr_t) + sizeof(blk_head_t);
							} else {
								timer_flg = 1;
								pbufi->head.cmd |= CMD_ERR_FLG; // Error cmd
								Receive_length = 0 + sizeof(blk_head_t);
							};
							break;
						case CMD_DEV_SRG: // Set reg
							timer_flg = 0;
							if (SMBusWriteWord(pbufi->data.reg.dev_addr,
								pbufi->data.reg.data, pbufi->data.reg.reg_addr)) {
								timer_flg = 1;
								Receive_length = sizeof(reg_wr_t) + sizeof(blk_head_t);
							} else {
								timer_flg = 1;
								pbufi->head.cmd |= CMD_ERR_FLG; // Error cmd
								Receive_length = 0 + sizeof(blk_head_t);
							};
							break;
						default:
							pbufi->head.cmd |= CMD_ERR_FLG; // Error cmd
							Receive_length = 0 + sizeof(blk_head_t);
							break;
					};

					if(Receive_length) {
						pbufi->head.size = Receive_length - sizeof(blk_head_t);
						if(CDC_Send_DATA((uint8_t *)pbufi, Receive_length))
						Receive_length = 0;
					} 					
				} else // if (Receive_length >= sizeof(blk_head_t)
				if(send_err) {
					send_pkt.head.cmd = CMD_DEV_I2C | CMD_ERR_FLG;
					send_pkt.data.ud[0]	= all_rd_count;
					send_pkt.data.ud[1] = not_send_count;
					send_pkt.data.uc[8] = send_err;
					send_pkt.head.size = 9; // tx_len - sizeof(blk_head_t);
					CDC_Send_DATA((uint8_t *)&send_pkt, 9 + sizeof(blk_head_t));
					send_err = 0;
				} else // if(send_err)
				if(cfg_i2c.pktcnt
				&& ((i2c_buf_wr - i2c_buf_rd) & (I2C_BUF_SIZE - 1)) > cfg_i2c.pktcnt) {
					uint32_t i;
					for(i = 0; i < cfg_i2c.pktcnt ; i ++) {
						send_pkt.data.ui[i] = i2c_buf[i2c_buf_rd++];
						#if (I2C_BUF_SIZE != 256)
						i2c_buf_rd &= I2C_BUF_SIZE - 1;
						#endif
					}
					all_rd_count+=cfg_i2c.pktcnt;
					send_pkt.head.size = cfg_i2c.pktcnt*2;
					send_pkt.head.cmd = CMD_DEV_I2C;
					CDC_Send_DATA((uint8_t *)&send_pkt, cfg_i2c.pktcnt*2+sizeof(blk_head_t));
				}; 
			}; // if(packet_sent) 
		}; // if (bDeviceState == CONFIGURED)
	}; // while(1)
}

#ifdef USE_FULL_ASSERT
/*******************************************************************************
 * Function Name	 : assert_failed
 * Description	 : Reports the name of the source file and the source line number
 *				   where the assert_param error has occurred.
 * Input			 : - file: pointer to the source file name
 *				   - line: assert_param error line source number
 * Output		 : None
 * Return		 : None
 *******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1) {}
}
#endif

/*******************************************************************************
 * Function Name	 : TIM2_IRQHandler
 * Description	 : This function handles TIM2 global interrupt request.
 * Input			 : None
 * Output		 : None
 * Return		 : None
 *******************************************************************************/
void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		if(timer_flg)
			GetNewRegData();
	}
}

/*****************************END OF FILE****/
