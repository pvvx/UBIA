/*
 * cmd_cfg.h
 *
 *  Created on: 14.01.2020
 *      Author: pvvx
 */

#ifndef _CMD_CFG_H_
#define _CMD_CFG_H_

#include "proj/common/types.h"
#include "i2cbus.h"

//  packet 20 + 27 * 8 байт  = 236 ? (236-2)/2 = 117 sps
//  packet 20 + 27 * 7 байт  = 209 ? (209-2)/2 = 103 sps
//  packet 20 + 27 * 6 байт  = 182 ? (182-2)/2 = 90 sps
//#define MAX_DLE_DATA_SIZE 240 // 244
//#define MAX_MTU_DATA_SIZE 247

#define MIN_TSPS_US  200 // минимум для BLE 200 us

#define SMPS_BLK_CNT	116 // ((20+27*7-2)/2)&0xFFFE = 102 or ((20+27*8-2)/2)&0xFFFE = 116 (9 блоков)
#define DLE_DATA_SIZE (SMPS_BLK_CNT*2+2)
#define MTU_DATA_SIZE (DLE_DATA_SIZE+7) // -(3 байта заголовка ATT и 4 байта заголовка L2CAP)
#define MTU_RX_DATA_SIZE 63

// DEV command id:
#define CMD_DEV_VER  0x00 // Get Ver
// I2C/SBUS cfg
#define CMD_DEV_CFG  0x01 // Get/Set CFG/ini I2C & Start measure
// Save cfg
#define CMD_DEV_SCF  0x02 // Store CFG/ini in Flash
// Status
#define CMD_DEV_STA  0x03 // Status
// BLE cfg
#define CMD_DEV_CPU  0x04 // Connect parameters Update (BLE)
#define CMD_DEV_BLE  0x05 // BLE parameters Update (BLE)
// 0x06
// I2C/SMBUS out regs
#define CMD_DEV_I2C  0x07 // blk out regs i2c data
// ADC cfg
#define CMD_DEV_CAD  0x08 // Get/Set CFG/ini ADC & Start measure
// DAC cfg
#define CMD_DEV_DAC  0x09 // DAC cfg
// ADC out samples
#define CMD_DEV_ADC  0x0A // blk out regs ADC data
// TST device
#define CMD_DEV_TST  0x0B // blk out X data, cfg TST device
// I2C rd/wr
#define CMD_DEV_UTR  0x0C // I2C read/write
// Debug
#define CMD_DEV_DBG  0x0D // Debug
// Power, Sleep
#define CMD_DEV_PWR  0x0E // Power On/Off, Sleep
// Runtime Error
#define CMD_DEV_ERR  0x0F // Runtime Error
// I2C/SMBUS rd/wr regs
#define CMD_DEV_GRG  0x10 // Get reg I2C
#define CMD_DEV_SRG  0x11 // Set reg I2C
// UART
#define CMD_DEV_UAC  0x12 // Set UART
#define CMD_DEV_UAR  0x13 // Send/Receive UART

#define CMD_ERR_FLG   0x80 // send error cmd mask

// Runtime Errors
#define RTERR_USER	0x00
#define RTERR_MTEX  0x01 // blc_att_requestMtuSizeExchange()
#define RTERR_TOMT  0x02 // timeout blc_att_requestMtuSizeExchange() (>5 sec)
#define RTERR_PIND  0x03 // bls_att_pushIndicateData()
#define RTERR_UART  0x04 // UART Busy!

// CMD_DEV_ERR  0x0f // Runtime Error
typedef struct  __attribute__((packed)) _dev_err_t{
	uint16_t id; // тип ошибки (RTERR)
	uint16_t err; // номер/значение ошибки
} dev_err_t;
#define HX711_DATA_OUT 10
// CMD_DEV_TST HX711 blk out X data, cfg TST device
typedef struct  __attribute__((packed)) _hx711_out_t{
	uint32_t data[HX711_DATA_OUT];
} hx711_out_t;
typedef struct  __attribute__((packed)) _hx711_set_t{
	uint8_t mode; // 0 - off, 25, 26, 27
} hx711_set_t;

// CMD_DEV_GRG Get reg I2C
typedef struct  __attribute__((packed)) _reg_rd_t{
	uint8_t dev_addr; // адрес на шине i2c
	uint8_t reg_addr; // номер регистра чтения
} reg_rd_t;

// CMD_DEV_STA  Status
typedef struct  __attribute__((packed)) _dev_sta_t{
	uint32_t rd_cnt; // счетчик samples
	uint32_t to_cnt; // счетчик не переданных или timeout при передаче samples
} dev_sta_t;

// CMD_DEV_DBG  Debug
typedef struct  __attribute__((packed)) _dev_dbg_t{
	uint16_t rd_cnt; // счетчик байт чтения (hi byte - reserved)
	uint16_t addr; // адрес
} dev_dbg_t;

// CMD_DEV_UAC Set UART
typedef struct  __attribute__((packed)) _dev_uart_cfg_t{
	uint16_t baud;   // 3..53333 -> 300 baud to 4 Mbaud (16M/4,5,6,7,8..262144)
	uint8_t  stop;   // =0 ->1, =1 -> 1.5, =2 => 2
	uint8_t  parity; // =0 -> none, =1 -> even, =2 -> odd
} dev_uart_cfg_t;

// CMD_DEV_DAC Dac cfg
typedef struct  __attribute__((packed)) _dev_dac_cfg_t{
	int16_t value[1]; // значение вывода
	uint8_t mode;  // 0..4 !первая запись устанавливает внуренний уровень, последующие выводятся на выход GPIO!
	uint8_t slk_mhz;  // 1..16
	uint16_t step; // 0..3ff
	uint8_t volume; // 0..7f
} dev_dac_cfg_t;

// CMD_DEV_SRG  Set reg I2C
typedef struct __attribute__((packed)) _reg_wr_t{
	uint8_t dev_addr; // адрес на шине i2c
	uint8_t reg_addr; // номер регистра чтения
	uint16_t data; // значение для записи в регистр
} reg_wr_t;

// CMD_DEV_SCF  Store CFG/ini in Flash
typedef struct  __attribute__((packed)) _dev_scf_t{
	uint8_t i2c	: 1; //0x01
	uint8_t adc	: 1; //0x02
	uint8_t con	: 1; //0x04
	uint8_t adv	: 1; //0x08
	uint8_t dac	: 1; //0x10
	uint8_t uart: 1; //0x20
} dev_scf_t;
// Структура конфигурации опроса и инициализации устройства ADC
// Выходной пакет непрерывного опроса формируется по данному описанию
// CMD_DEV_CAD Get/Set CFG/ini ADC & Start measure
typedef struct __attribute__((packed)) _dev_adc_cfg_t {
	uint8_t pktcnt;	// минимальное кол-во передаваемых значений ADC в одном пакете передачи (автоподстройка до мах SMPS_BLK_CNT)
	uint8_t chnl; 	// Channel
	uint16_t sps; 	// период adc chl0
	uint8_t pga20db;
	uint8_t pga2db5;
} dev_adc_cfg_t; // [6]
extern dev_adc_cfg_t cfg_adc; // store in eep

// Структура конфигурации опроса и инициализации устройства I2C
// Выходной пакет непрерывного опроса формируется по данному описанию
#define MAX_INIT_REGS 4 // 1,2,4,8,..
#define MAX_READ_REGS 4 // 1,2,4,8,..
// CMD_DEV_CFG Get/Set CFG/ini I2C & Start measure
typedef struct __attribute__((packed)) _dev_i2c_cfg_t {
	uint8_t pktcnt;  	// кол-во передаваемых значений из регистров в одном пакете передачи
	uint8_t multiplier; // множитель периода опроса, time << multiplier
	uint16_t time; 		// период опроса регистров чтения в us
	uint16_t clk_khz; 	// частота i2c шины в kHz
	reg_wr_t init[MAX_INIT_REGS];
	reg_rd_t rd[MAX_READ_REGS];
	reg_wr_t slp[2];
} dev_i2c_cfg_t; // [38] 6 + 4*4 + 2*4 + 4*2

extern dev_i2c_cfg_t cfg_i2c; // store in eep

// CMD_DEV_PWR Power On/Off, Sleep
typedef struct  __attribute__((packed)) _dev_pwr_slp_t{
	uint16_t ExtDevPowerOn	: 1; //0x01
	uint16_t I2CDevWakeUp	: 1; //0x02
	uint16_t ExtDevPowerOff	: 1; //0x04
	uint16_t DAC_off		: 1; //0x08
	uint16_t I2CDevSleep	: 1; //0x10
	uint16_t ADC_Stop		: 1; //0x20
	uint16_t Uart_off		: 1; //0x40
	uint16_t Disconnect		: 1; //0x80
	uint16_t Sleep_off		: 1; //0x100
	uint16_t Sleep_CPU		: 1; //0x200
	uint16_t Sleep_On		: 1; //0x400
	uint16_t Reset			: 1; //0x800
	uint16_t Test			: 1; //0x8000
} dev_pwr_slp_t;

// BLE connection config
// CMD_DEV_CPU Connect parameters Update (BLE)
typedef struct __attribute__((packed)){
  /** Minimum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  unsigned short intervalMin;
  /** Maximum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  unsigned short intervalMax;
  /** Number of LL latency connection events (0x0000 - 0x03e8) */
  unsigned short latency;
  /** Connection Timeout (0x000A - 0x0C80 * 10 ms) */
  unsigned short timeout;
} ble_con_t;

//extern ble_con_t ble_con_ini; // store in eep

// CMD_DEV_BLE  BLE parameters Update (BLE)
typedef struct __attribute__((packed)) {
  /** Advertising Interval minimum value (interval * 625 us) */
  unsigned short intervalMin;
  /** Advertising Interval maximum value (interval * 625 us) */
  unsigned short intervalMax;

  unsigned char rf_power; // 0..16 8dB,4dB,0dB,-4dB,-10dB,-14dB,..RF_POWER_OFF

  unsigned char name[16];
} ble_cfg_t;

//extern ble_cfg_t ble_cfg_ini; // store in eep

typedef struct __attribute__((packed)) {
	uint8_t size; // размер пакета
	uint8_t cmd;  // номер команды / тип пакета
} blk_head_t;

#ifndef MTU_RX_DATA_SIZE
#define MTU_RX_DATA_SIZE (sizeof(blk_head_t) + sizeof(dev_i2c_cfg_t))
#endif
#if (MTU_RX_DATA_SIZE > 64)
#error MTU_RX_DATA_SIZE!
#endif

// IN CMD_DEV_UTR I2C read/write
/* см. Universal I2C/SMBUS read-write transaction struct in i2cbus.h
typedef struct _i2c_utr_t {
	unsigned char mode;	// bit0..6: number wr_byte for new START (bit7: =1 - generate STOP/START)
	unsigned char rdlen; // bit7: =1 - old read byte generate NACK, =0 - ACK
	unsigned char wrdata[1]; // i2c_addr_wr, wr_byte1, wr_byte2, wr_byte3, ... wr_byte126
} i2c_utr_t; */

// HEAD CMD_DEV_UTR I2C read/write
typedef struct __attribute__((packed)) _i2c_rd_t{
	uint8_t dev_addr; // адрес на шине i2c
	uint8_t reg_count; // кол-во регистров I2C / байт чтения, bit7 =1 -> end byte send ACK
} i2c_rd_t;
// OUT CMD_DEV_UTR I2C read/write
typedef struct __attribute__((packed)) _i2c_wr_t{
	uint8_t dev_addr; // адрес на шине i2c
	uint8_t rd_count; // кол-во регистров I2C / байт чтения после передачи, bit7 =1 -> end byte send ACK
	uint8_t data[MTU_RX_DATA_SIZE-sizeof(blk_head_t)-sizeof(i2c_rd_t)]; // значения регистров
} i2c_wr_t;

typedef struct __attribute__((packed)) _blk_tx_pkt_t{
	blk_head_t head;
	union __attribute__((packed)) {
		uint8_t uc[MTU_DATA_SIZE-sizeof(blk_head_t)];
		int8_t sc[MTU_DATA_SIZE-sizeof(blk_head_t)];
		uint16_t ui[(MTU_DATA_SIZE-sizeof(blk_head_t))/sizeof(uint16_t)];
		int16_t si[(MTU_DATA_SIZE-sizeof(blk_head_t))/sizeof(int16_t)];
		uint32_t ud[(MTU_DATA_SIZE-sizeof(blk_head_t))/sizeof(uint32_t)];
		int32_t sd[(MTU_DATA_SIZE-sizeof(blk_head_t))/sizeof(uint32_t)];
		dev_i2c_cfg_t ci2c;
		dev_adc_cfg_t cadc;
		ble_con_t con;
		ble_cfg_t ble;
		reg_wr_t reg;
		i2c_rd_t rd;
		i2c_wr_t wr;
		dev_err_t err;
		dev_sta_t sta;
		dev_dbg_t dbg;
		dev_scf_t scf;
		dev_dac_cfg_t dac;
		hx711_out_t hxo;
		dev_uart_cfg_t ua;
	} data;
} blk_tx_pkt_t;

typedef struct __attribute__((packed)) _blk_rx_pkt_t{
	blk_head_t head;
	union __attribute__((packed)) {
		uint8_t uc[MTU_RX_DATA_SIZE-sizeof(blk_head_t)];
		int8_t sc[MTU_RX_DATA_SIZE-sizeof(blk_head_t)];
		uint16_t ui[(MTU_RX_DATA_SIZE-sizeof(blk_head_t))/sizeof(uint16_t)];
		int16_t si[(MTU_RX_DATA_SIZE-sizeof(blk_head_t))/sizeof(int16_t)];
		uint32_t ud[(MTU_RX_DATA_SIZE-sizeof(blk_head_t))/sizeof(uint32_t)];
		int32_t sd[(MTU_RX_DATA_SIZE-sizeof(blk_head_t))/sizeof(uint32_t)];
		dev_i2c_cfg_t ci2c;
		dev_adc_cfg_t cadc;
		ble_con_t con;
		ble_cfg_t ble;
		reg_wr_t reg;
		i2c_rd_t rd;
		i2c_wr_t wr;
		i2c_utr_t utr;
		dev_pwr_slp_t pwr;
		dev_dbg_t dbg;
		dev_scf_t scf;
		dev_dac_cfg_t dac;
		hx711_set_t hxi;
		dev_uart_cfg_t ua;
	} data;
} blk_rx_pkt_t;

extern blk_rx_pkt_t read_pkt;
extern blk_tx_pkt_t send_pkt;

#if (USE_USB_CDC && USE_BLE)
volatile u8 usb_actived; // flag =1 -> usb, =0 -> ble
extern volatile unsigned char usb_actived;
int tst_usb_actived(void);
#endif

extern u32 all_rd_count; // count read
extern u32 not_send_count; // diag count

extern u8 rx_len; // flag - пришла команда в read_pkt
extern u8 tx_len; // flag - есть данные для передачи в send_pkt

void send_rtm_err(u16 err_id, u16 err);

/*******************************************************************************
 * Function Name : usb_ble_cmd_decode.
 * Description	 : Main loop routine.
 * Input		 : blk_tx_pkt_t * pbufo, blk_tx_pkt_t * pbufi, int rxlen
 * Return		 : txlen.
 *******************************************************************************/
unsigned int cmd_decode(blk_tx_pkt_t * pbufo, blk_rx_pkt_t * pbufi, unsigned int rxlen);

#endif /* _CMD_CFG_H_ */
