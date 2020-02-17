/*
 * app.c
 *
 *  Created on: 10.11.2019
 *      Author: pvvx
 */

#include "proj/tl_common.h"
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
#include "flash_eep.h"
#include "eep_id.h"
#include "usbCDC/drivers.h"
#if USE_I2C
#include "i2cbus.h"
#endif
#if (USE_ADC_TST)
#include "adc.h"
#endif
#include "dac.h"

static inline u32 clock_tik_exceed(u32 ref, u32 span_us){
	return ((u32)(clock_time() - ref) > span_us);
}

#if (BATT_SERVICE_ENABLE)
	extern u8  my_batVal[20];
	extern u16 batteryValueInCCC;
	extern u8 adc_hw_initialized;
#endif

extern u16 SppDataServer2ClientDataCCC;

u32 all_rd_count = 0; // status
u32 not_send_count = 0; // status
#define LOOP_MIN_CYCLE 0
#if LOOP_MIN_CYCLE
u32 ble_loop_count;
#endif

volatile u8 sleep_disable = 0; // flag, pm not sleep
u32 connection_time;
u8 device_in_connection_state = 0; // flag
u8 wrk_enable = 1; // flag connect & run device, in BLE mode
u8 wrk_tick = 0; // stages, in BLE main loop

u8 rx_len = 0; // flag - пришла команда в read_pkt
u8 tx_len = 0; // flag - есть данные для передачи в send_pkt

blk_rx_pkt_t read_pkt; // приемный буфер
blk_tx_pkt_t send_pkt; // буфер отправки

#if USE_I2C_INA
u32 t_rd_us = 0; // flag и время в us работы опроса без таймера
volatile u8 timer_flg = 0; // flag временного отключения чтения I2C regs по irq
// GENERAL CALL RESET: internal reset, similar to a power-on-reset
// Please refer to the Phillips I2C document for more details of the General Call specifications.
static u8 general_call_reset[] = { 0, 0, 0, 6 };
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
#endif // USE_I2C_INA

#if USE_USB_CDC
// Define USB rx/tx buffer
#define RX_BUF_LEN    USB_CDC_MAX_RX_BLK_SIZE // in bytes
#define TX_BUF_LEN    MTU_DATA_SIZE // in bytes

struct {
	unsigned char rx[RX_BUF_LEN]; // 64 bytes
	unsigned char tx[TX_BUF_LEN]; // 241 bytes
}usb_buf;

volatile u8 usb_actived; // flag
#endif // USE_USB_CDC

/*
const dev_config_t dev_cfg_def = {
	.vbat_min_mv = 10,
	.vbat_nom_mv = 3300,
#if MCU_CORE_TYPE == MCU_CORE_8266
	.vbat_adc_k = 1300*4,
#else
	.vbat_adc_k = 3*1428,
#endif
	.vbat_check_step_ms = 1000
};

dev_config_t dev_cfg;
*/

dev_i2c_cfg_t cfg_i2c;
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
dev_adc_cfg_t cfg_adc;
const dev_adc_cfg_t def_cfg_adc = { // 500 sps, PC4
		.pktcnt = 0,
		.chnl = 9,
		.sps = 500,
		.pga20db = 0,
		.pga2db5 = 0
};

ble_con_t ble_con_ini;
ble_con_t cur_ble_con_ini;
const ble_con_t def_ble_con_ini = {DEF_CONN_PARMS};
ble_adv_t ble_adv_ini;
const ble_adv_t def_ble_adv_ini = {DEF_ADV_INTERVAL};

my_fifo_t			blt_rxfifo;
u8					blt_rxfifo_b[];

my_fifo_t			blt_txfifo;
u8					blt_txfifo_b[];

MYFIFO_INIT(blt_rxfifo, 64, 8); 	// 512 bytes + headers
MYFIFO_INIT(blt_txfifo, 40, 16);	// 640 bytes + headers

const u8 ble_dev_name[8] = { BLE_DEV_NAME, 0 };

//////////////////////////////////////////////////////////////////////////////
//	Adv Packet, Response Packet
//////////////////////////////////////////////////////////////////////////////
u8	tbl_advData[] = { // https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile/
//	 0x08, 0x09, 't', 'B', 'L', 'E', 'x', 'x', 'x',
//	 sizeof(ble_dev_name), 0x09, BLE_DEV_NAME,
	 // 0x01: Основные характеристики Bluetooth
	 0x02, 0x01, 0x05, 							// BLE limited discoverable mode and BR/EDR not supported
	 // 0x19: Базовая спецификация Bluetooth
	 0x03, 0x19, 0x80, 0x01, 					// 384, Generic Remote Control, Generic category
#if	(BATT_SERVICE_ENABLE && SPP_SERVICE_ENABLE)
	 // 0x02: incomplete list of service class UUIDs (0x1812, 0x180F) // Human Interface Device, Battery Service
//	 0x05, 0x02, 0x12, 0x18, 0x0F, 0x18,
	 0x05, 0x02, 0x0f, 0x18, 0x00, 0xff,
#elif (SPP_SERVICE_ENABLE)
	 // 0x02: incomplete list of service class UUID (0xff00)
	 0x03, 0x02, (SERVICE_UUID_SPP>>8)& 0xff, SERVICE_UUID_SPP & 0xff,
#endif
#if 0
	 // 0xFF: Специфические данные производителя (Manufacturer-Specific Data Flag)
	 // [3,4] Company ID = 0x1234
	 0x03+4, 0xFF, 0x8A, 0x24,
	 0x03,0x02,0x01,0x00
};
#define advData_pu32 ((u32 *)(&tbl_advData[sizeof(tbl_advData)-4]))
#define advData_u32 (*advData_pu32)
#define advData_pu16 ((u16 *)(&tbl_advData[sizeof(tbl_advData)-4]))
#else
};
#endif

const u8 tbl_scanRsp[] = {
	// 0x09: «Полное локальное имя»
//	 0x08, 0x09, 't', 'B', 'L', 'E', 'x', 'x', 'x', //scan name "tBLExxx"
	 sizeof(ble_dev_name), 0x09, BLE_DEV_NAME
	};

//--------- < Test!
static inline void test(void) {
};
//--------- Test! >

void send_ble_err(u16 err_id, u16 err) {
	send_pkt.head.size = sizeof(dev_err_t);
	send_pkt.head.cmd = CMD_DEV_ERR;
	send_pkt.data.err.id = err_id;
	send_pkt.data.err.err = err;
	bls_att_pushIndicateData(SPP_Server2Client_INPUT_DP_H, (u8 *) &send_pkt, sizeof(blk_head_t) + sizeof(dev_err_t));
}

void send_usb_err(u16 err_id, u16 err) {
	send_pkt.head.size = sizeof(dev_err_t);
	send_pkt.head.cmd = CMD_DEV_ERR;
	send_pkt.data.err.id = err_id;
	send_pkt.data.err.err = err;
	if(USBCDC_IsAvailable()) {
		USBCDC_DataSend((u8 *) &send_pkt, sizeof(blk_head_t) + sizeof(dev_err_t));
	}
}
/*
 * rxFunc rx callback function
 * Called from Irq (!)
 */
_attribute_ram_code_ void USBCDC_RxCb(unsigned char *data, unsigned int len){
	if (len) { // есть данные?
#ifdef USB_LED_RX
		USB_LED_RX();
#endif
		if(rx_len == 0
			&& data
			&& len >= sizeof(blk_head_t)
			&& len <= MTU_RX_DATA_SIZE
			&& data[0]+sizeof(blk_head_t) <= len) {
			rx_len = data[0]+sizeof(blk_head_t);
			memcpy(&read_pkt, data, rx_len);
		}
		USBCDC_RxBufSet(usb_buf.rx); // назначить новый буфер (в данном приложении единственный)
	}
}

int tst_usb_actived(void) {
	usb_actived = gpio_read(GPIO_WAKEUP_MODULE) != 0;
	return usb_actived;
}

#if SIG_PROC_ENABLE
// @TODO надо запихать аналог-автомат ...
/*------------------------------------------------------------------- l2cap data pkt(SIG) ---------------------------------------------------*
 | stamp_time(4B) |llid nesn sn md |  pdu-len   | l2cap_len(2B)| chanId(2B)| Code(1B)|Id(1B)|Data-Len(2B) |           Result(2B)             |
 |                |   type(1B)     | rf_len(1B) |       L2CAP header       |          SIG pkt Header      |  SIG_Connection_param_Update_Rsp |
 |                |                |            |     0x0006   |    0x05   |   0x13  | 0x01 |  0x0002     |             0x0000               |
 |                |          data_headr         |                                                       payload                              |
 *-------------------------------------------------------------------------------------------------------------------------------------------*/
int att_sig_proc_handler(u16 connHandle, u8 * p)
{
	rf_pkt_l2cap_sig_connParaUpRsp_t* pp = (rf_pkt_l2cap_sig_connParaUpRsp_t*)p;
	static u8 conn_update_cnt;
	u8 sig_conn_param_update_rsp[9] = { 0x0A, 0x06, 0x00, 0x05, 0x00, 0x13, 0x01, 0x02, 0x00 };
	if(!memcmp(sig_conn_param_update_rsp, &pp->rf_len, 9) && ((pp->type&0b11) == 2)){ // l2cap data pkt, start pkt
		if(pp->result == 0x0000){
			printf("SIG: the LE master Host has accepted the connection parameters.\n");
			conn_update_cnt = 0;
		}
		else if(pp->result == 0x0001)
		{
			printf("SIG: the LE master Host has rejected the connection parameters..\n");
			printf("Current Connection interval: %d us.\n", bls_ll_getConnectionInterval() * 1250 );
			conn_update_cnt++;
            if(conn_update_cnt < 4){
            	printf("Slave sent update connPara req!\n");
            }
			if(conn_update_cnt == 1){
				bls_l2cap_requestConnParamUpdate (8, 16, 0, 400); // 18.75ms iOS
			}
			else if(conn_update_cnt == 2){
				bls_l2cap_requestConnParamUpdate (16, 32, 0, 400);
			}
			else if(conn_update_cnt == 3){
				bls_l2cap_requestConnParamUpdate (32, 60, 0, 400);
			}
			else{
				conn_update_cnt = 0;
				printf("Slave Connection Parameters Update table all tested and failed!\n");
			}
		}
	}
}
#endif

void ExtDevPowerOff() {
	gpio_setup_up_down_resistor(EXT_POWER_OFF, PM_PIN_PULLUP_10K);
	gpio_setup_up_down_resistor(EXT_POWER_4MA, PM_PIN_PULLDOWN_100K);
	gpio_set_data_strength(EXT_POWER_4MA, 0);
	gpio_set_output_en(EXT_POWER_4MA, 0);
	gpio_write(EXT_POWER_4MA, 0);
}

void ExtDevPowerOn() {
	gpio_setup_up_down_resistor(EXT_POWER_OFF, PM_PIN_PULLDOWN_100K);
	gpio_setup_up_down_resistor(EXT_POWER_4MA, PM_PIN_PULLUP_10K);
	gpio_write(EXT_POWER_4MA, 1);
//	gpio_set_data_strength(EXT_POWER_4MA, 0);
	gpio_set_output_en(EXT_POWER_4MA, 1);
	gpio_set_data_strength(EXT_POWER_4MA, 1);
}

#if USE_I2C_INA

void FlushI2CBuf(void) {
	u8 r = irq_disable();
	raddr = &cfg_i2c.rd[0]; //	rd_next_cnt = 0;
	i2c_buf_wr = 0;
	i2c_buf_rd = 0;
	irq_restore(r);
}

void Timer_Stop(void) {
	t_rd_us = 0;
	timer_flg = 0;
	sleep_disable = 0;
	reg_tmr1_tick = 0;
	reg_tmr_sta = FLD_TMR_STA_TMR1;
	reg_irq_mask1 &= ~FLD_IRQ_TMR1_EN;
	reg_tmr_ctrl8 &= ~FLD_TMR1_EN;
}

void Timer_Init(uint32_t period_us) {
	sleep_disable = 1;
	reg_tmr1_tick = 0;
//	if(usb_actived) period_us CLOCK in USB!
	reg_tmr1_capt = period_us * CLOCK_SYS_CLOCK_1US;
	reg_tmr_ctrl8 |= FLD_TMR1_EN; // 	FLD_TMR1_MODE = 0
	reg_irq_mask1 |= FLD_IRQ_TMR1_EN;
	reg_tmr_sta = FLD_TMR_STA_TMR1; // clear irq status
}

// I2c device GetNewRegData
// Called from Timer Irq (!)
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

void I2CDevSleep() {
	cfg_i2c.pktcnt = 0;
	t_rd_us = 0;
	if(cfg_i2c.slp[0].dev_addr) {
		I2CBusInit(cfg_i2c.clk_khz);
		I2CBusWriteWord(cfg_i2c.slp[0].dev_addr,cfg_i2c.slp[0].reg_addr, cfg_i2c.slp[0].data); // Power-Down (or Shutdown)
		if(cfg_i2c.slp[1].dev_addr)
			I2CBusWriteWord(cfg_i2c.slp[0].dev_addr,cfg_i2c.slp[0].reg_addr, cfg_i2c.slp[0].data); // Power-Down (or Shutdown)
	}
	I2CBusDeInit();
}

void I2CDevWakeUp() {
	I2CBusInit(400);
	I2CBusUtr(0, (i2c_utr_t *)&general_call_reset, 1);
}

/*
 * t_us < connect interval * 2 -> no sleep
 * connect interval = 10 ?
 *  if t_us < 20000 -> no sleep -> timer On
 *  else set connect interval / 2
 */
int InitI2CDevice(void) {
	u32 i, x;
	timer_flg = 0;
	if(usb_actived) {
		i = 50;
		x = 30;
	}
	else {
		i = MIN_TSPS_US;
		x = SMPS_BLK_CNT;
	}
	if(cfg_i2c.pktcnt > x) cfg_i2c.pktcnt = x;
	if(cfg_i2c.clk_khz < 50 || cfg_i2c.clk_khz > 2500)
		cfg_i2c.clk_khz = 1000;
	I2CBusInit(cfg_i2c.clk_khz);
	t_rd_us = cfg_i2c.time << cfg_i2c.multiplier;
	// ~ 30 us при 1 MHz I2C CLK (5 us + 25 us)
	if(t_rd_us < i || t_rd_us > 0xffffffff/CLOCK_SYS_CLOCK_1US) {
		cfg_i2c.pktcnt = 0;
		Timer_Stop();
		return 0; // error t_us
	}
	FlushI2CBuf();
	if (cfg_i2c.pktcnt) {
		if(usb_actived || t_rd_us < 10000) {
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
#endif

#if USE_ADC_TST

void ADC_Stop(void) {
	cfg_adc.pktcnt  = 0;
	deinit_adc();
}

int InitADCDevice(void) {
	if(cfg_adc.pktcnt) {
		// start (new) counts
		all_rd_count = 0;
		not_send_count = 0;
		if(cfg_adc.pktcnt > SMPS_BLK_CNT)
			cfg_adc.pktcnt = SMPS_BLK_CNT;
		if(!init_adc_dfifo(&cfg_adc)) {
			cfg_adc.pktcnt  = 0;
			return 0;
		}
	}
	return 1;
}

#endif

int module_onReceiveData(void *par)
{
	rf_packet_att_data_t *pp = par;
	u8 len = pp->l2cap - 3;
	blk_rx_pkt_t * p = (blk_rx_pkt_t *)&pp->dat;
//	mini_printf("\r\nonReceiveData %d\r\n", len);
	if(rx_len == 0 && len >= sizeof(blk_head_t) && len >= p->head.size + sizeof(blk_head_t)) {
		rx_len = p->head.size + sizeof(blk_head_t);
		memcpy(&read_pkt, p, rx_len);
	}
	return 0;
}

/*******************************************************************************
 * Function Name : usb_ble_cmd_decode.
 * Description	 : Main loop routine.
 * Input		 : blk_tx_pkt_t * pbufo, blk_tx_pkt_t * pbufi, int rxlen
 * Return		 : txlen.
 *******************************************************************************/
unsigned int cmd_decode(blk_tx_pkt_t * pbufo, blk_rx_pkt_t * pbufi, unsigned int rxlen) {
	unsigned int txlen = 0;
	u8 tmp;
//	if (rxlen >= sizeof(blk_head_t)) {
//		if (rxlen >= pbufi->head.size + sizeof(blk_head_t)) {
			pbufo->head.cmd = pbufi->head.cmd;
			switch (pbufi->head.cmd) {
			case CMD_DEV_VER: // Get Ver
#if USE_I2C_INA && !USE_ADC_TST
				pbufo->data.ui[0] = 0x1016; // DevID = 0x1016
				pbufo->data.ui[1] = 0x0010; // Ver 0.0.1.0 = 0x0010
#elif USE_ADC_TST && !USE_I2C_INA
				pbufo->data.ui[0] = 0x0020; // DevID = 0x0020
				pbufo->data.ui[1] = 0x0002; // Ver 0.0.0.2 = 0x0002
#else
				pbufo->data.ui[0] = 0x1021; // DevID = 0x1021
				pbufo->data.ui[1] = 0x0002; // Ver 1.2.3.4 = 0x1234
#endif					
				txlen = sizeof(u16) + sizeof(u16) + sizeof(blk_head_t);
				break;
#if USE_I2C_INA
			case CMD_DEV_CFG: // Get/Set CFG/ini & Start measure
				if (pbufi->head.size) {
					timer_flg = 0;
					memcpy(&cfg_i2c, &pbufi->data.ci2c,
						(pbufi->head.size > sizeof(cfg_i2c))? sizeof(cfg_i2c) : pbufi->head.size);
					if (!InitI2CDevice()) {
						pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
						txlen = 0 + sizeof(blk_head_t);
						break;
					}
					if(usb_actived || sleep_disable)
						timer_flg = 1;
				}
				memcpy(&pbufo->data, &cfg_i2c, sizeof(cfg_i2c));
				txlen = sizeof(cfg_i2c) + sizeof(blk_head_t);
				break;
			case CMD_DEV_SCF: // Store CFG/ini in Flash
				if(pbufi->head.size < sizeof(dev_scf_t)) {
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
					break;
				}
				pbufo->data.ud[0] = 0;
				if(pbufi->data.scf.i2c)
					pbufo->data.scf.i2c = flash_write_cfg(&cfg_i2c, EEP_ID_I2C_CFG, sizeof(cfg_i2c));
				if(pbufi->data.scf.adc)
					pbufo->data.scf.adc = flash_write_cfg(&cfg_adc, EEP_ID_ADC_CFG, sizeof(cfg_adc));
				if(pbufi->data.scf.con)
					pbufo->data.scf.con = flash_write_cfg(&ble_con_ini, EEP_ID_CON_CFG, sizeof(dev_cfg));
				if(pbufi->data.scf.adv)
					pbufo->data.scf.adv = flash_write_cfg(&ble_adv_ini, EEP_ID_ADV_CFG, sizeof(dev_cfg));
				txlen = sizeof(dev_scf_t) + sizeof(blk_head_t);
				break;
			//-------
			case CMD_DEV_GRG: // Get reg
				tmp = irq_disable();
				if (I2CBusReadWord(pbufi->data.reg.dev_addr, pbufi->data.reg.reg_addr,
					(uint16_t *)&pbufo->data.reg.data)) {
					pbufo->data.ui[0] = pbufi->data.ui[0];
					irq_restore(tmp);
					txlen = sizeof(reg_wr_t) + sizeof(blk_head_t);
				} else {
					irq_restore(tmp);
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
				}
				break;
			case CMD_DEV_SRG: // Set reg
				tmp = irq_disable();
				if (I2CBusWriteWord(pbufi->data.reg.dev_addr, pbufi->data.reg.reg_addr,
						pbufi->data.reg.data)) {
					pbufo->data.reg = pbufi->data.reg;
					irq_restore(tmp);
					txlen = sizeof(reg_wr_t) + sizeof(blk_head_t);
				} else {
					irq_restore(tmp);
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
				};
				break;
#endif					
#if USE_ADC_TST
			case CMD_DEV_CAD: // Get/Set CFG/ini ADC & Start measure
				if (pbufi->head.size) {
					memcpy(&cfg_adc, &pbufi->data.cadc,
						(pbufi->head.size > sizeof(cfg_adc))? sizeof(cfg_adc) : pbufi->head.size);
					if (!InitADCDevice()) {
						pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
						txlen = 0 + sizeof(blk_head_t);
						break;
					}
				}
				memcpy(&pbufo->data, &cfg_adc, sizeof(cfg_adc));
				txlen = sizeof(cfg_adc) + sizeof(blk_head_t);
				break;
#endif
			//-------
			case CMD_DEV_STA: // Status
				pbufo->data.sta.rd_cnt = all_rd_count;
				pbufo->data.sta.to_cnt = not_send_count;
				txlen = sizeof(blk_head_t) + sizeof(dev_sta_t);
				break;
			//-------
			case CMD_DEV_CPU: // Connect parameters Update
				memcpy(&pbufo->data, &ble_con_ini, sizeof(ble_con_ini));
				txlen = sizeof(blk_head_t) + sizeof(ble_con_ini);
				if(!usb_actived){
					if(pbufi->head.size) {
						if(pbufi->head.size > sizeof(ble_con_ini))
							pbufi->head.size = sizeof(ble_con_ini);
						tmp = sizeof(ble_con_ini) - pbufi->head.size;
						if(tmp)
							memcpy(&pbufi->data.uc[pbufi->head.size], ((u8*)&ble_con_ini) + pbufi->head.size, tmp);
						bls_l2cap_requestConnParamUpdate(
								pbufi->data.con.intervalMin,
								pbufi->data.con.intervalMax,
								pbufi->data.con.latency,
								pbufi->data.con.timeout);
					}
					cur_ble_con_ini.intervalMax = bls_ll_getConnectionInterval();
					cur_ble_con_ini.latency = bls_ll_getConnectionLatency();
					cur_ble_con_ini.timeout = bls_ll_getConnectionTimeout();
					memcpy(&pbufo->data.uc[sizeof(ble_con_ini)], &cur_ble_con_ini, sizeof(cur_ble_con_ini));
					txlen += sizeof(cur_ble_con_ini);
				}
				break;
			//--------
			case CMD_DEV_ADV: // Advertising parameters Update
				if(!usb_actived) {
					if (pbufi->head.size) {
						if(pbufi->head.size > sizeof(ble_adv_ini))
							pbufi->head.size = sizeof(ble_adv_ini);
						memcpy(&ble_adv_ini, &pbufi->data, pbufi->head.size);
						if(bls_ll_setAdvParam(
								ble_adv_ini.intervalMin,
								ble_adv_ini.intervalMax,
								ADV_TYPE_CONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC,
								0,  NULL, MY_APP_ADV_CHANNEL,
								ADV_FP_NONE) != BLE_SUCCESS) {
							memcpy(&ble_adv_ini, &ble_adv_ini, sizeof(ble_adv_ini));
							pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
							txlen = 0 + sizeof(blk_head_t);
							break;
						}
					}
					memcpy(&pbufo->data, &ble_adv_ini, sizeof(ble_adv_ini));
					txlen = sizeof(ble_adv_ini) + sizeof(blk_head_t);
				} else {
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
				}
				break;
#if USE_I2C
			case CMD_DEV_UTR: // I2C read/write
				txlen = pbufi->data.utr.rdlen & 0x7f;
				if(pbufi->head.size >= sizeof(i2c_utr_t)
					&&  txlen <= sizeof(pbufo->data.wr.data)
					&&	I2CBusUtr(&pbufo->data.wr.data,
							&pbufi->data.utr,
							pbufi->head.size - sizeof(i2c_utr_t)) // wrlen:  addr len - 1
							) {
					pbufo->data.wr.dev_addr = pbufi->data.utr.wrdata[0];
					pbufo->data.wr.rd_count = txlen;
					txlen += sizeof(i2c_rd_t) + sizeof(blk_head_t);
				} else {
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
				}
				break;
#endif // USE_I2C
			case CMD_DEV_PWR: // Power On/Off, Sleep
				if(pbufi->head.size < sizeof(dev_pwr_slp_t)) {
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
					break;
				}
				if(pbufi->data.pwr.ExtDevPowerOn) {
					ExtDevPowerOn();
				}
				if(pbufi->data.pwr.I2CDevWakeUp) {
					I2CDevWakeUp();
				}
				if(pbufi->data.pwr.ExtDevPowerOff) {
					ExtDevPowerOff();
				}
				if(pbufi->data.pwr.DAC_off) {
					sdm_off();
				}
				if(pbufi->data.pwr.I2CDevSleep) {
					Timer_Stop();
					I2CDevSleep();
				}
				if(pbufi->data.pwr.ADC_Stop) {
					ADC_Stop();
				}
				if(pbufi->data.pwr.Test) {
					test();
				}
				txlen = 0 + sizeof(blk_head_t);
				break;
			case CMD_DEV_DAC: // Dac cfg
				if(pbufi->head.size < sizeof(dev_dac_cfg_t) - 2) {
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
					break;
				}
				pbufo->data.uc[0] = dac_cmd(&pbufi->data.dac);
				txlen = 1 + sizeof(blk_head_t);
				break;
			case CMD_DEV_DBG: // Debug
				if(pbufi->head.size > sizeof(dev_dbg_t)) {
					memcpy((u8 *)0x800000 + pbufi->data.dbg.addr, &pbufi->data.ud[1], pbufi->head.size - sizeof(dev_dbg_t));
				} else if(pbufi->head.size < sizeof(dev_dbg_t)) {
					pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
					txlen = 0 + sizeof(blk_head_t);
					break;
				}
				txlen = pbufi->data.dbg.rd_cnt;
				if(txlen){
					if(txlen > sizeof(pbufi->data))
						txlen = sizeof(pbufi->data);
					memcpy(&pbufo->data, (u8 *)0x800000 + pbufi->data.dbg.addr, txlen);
				}
				txlen += sizeof(blk_head_t);
				break;
			default:
				pbufo->head.cmd |= CMD_ERR_FLG; // Error cmd
				txlen = 0 + sizeof(blk_head_t);
				break;
			};
			pbufo->head.size = txlen - sizeof(blk_head_t);
//		}
//		else
//			rxlen = 0;
//	}
	return txlen;
}

void app_suspend_exit(void);

int event_handler(u32 h, u8 *para, int n)
{
	if((h&HCI_FLAG_EVENT_TLK_MODULE)!= 0)			//module event
	{
		switch((u8)(h&0xff))
		{
			case BLT_EV_FLAG_SCAN_RSP:
				break;
			case BLT_EV_FLAG_CONNECT:
			{
//				printf("\n\r************** Connection event occured! **************\n\r");
#ifdef GREEN_LED
				gpio_write(GREEN_LED,ON);
#endif
				// task_connect();
				connection_time = clock_time();
				device_in_connection_state = 1;
#if BATT_SERVICE_ENABLE
				adc_hw_initialized = 0;
#endif
			}
				break;
			case BLT_EV_FLAG_TERMINATE:
			{
				cur_ble_con_ini.intervalMin = 0;
				sleep_disable = 0;
				device_in_connection_state = 0;
				SppDataServer2ClientDataCCC = 0;
				rx_len = 0;
				tx_len = 0;
//				printf("\n\r************** Terminate event occured! **************\n\r");
#ifdef GREEN_LED
				gpio_write(GREEN_LED,OFF);
#endif
			}
				break;
			case BLT_EV_FLAG_PAIRING_BEGIN:
			{
#if (SMP_PASSKEY_ENTRY)
	#if (PINCODE_RANDOM_ENABLE)//Randomly generated PINCODE, need to have the ability to print out it //printf, here
				u32 pinCode_random;
				u8 pc[7] = { '0','0','0','0','0','0', '\0'};
				generateRandomNum(4, (u8*)&pinCode_random);
				pinCode_random &= 999999;//0~999999
				pc[0] = (pinCode_random/100000) + '0';
				pc[1] = (pinCode_random%100000)/10000 + '0';
				pc[2] = ((pinCode_random%100000)%10000)/1000 + '0';
				pc[3] = (((pinCode_random%100000)%10000)%1000)/100 + '0';
				pc[4] = ((((pinCode_random%100000)%10000)%1000)%100)/10 + '0';
				pc[5] = pinCode_random%10 + '0';
				printf("PIN Code Number : %s\n", pc);

				blc_smp_enableAuthMITM (1, pinCode_random);//pincode
				blc_smp_setIoCapability (IO_CAPABLITY_DISPLAY_ONLY);
	#else
				//Popup dialog box on your phone , you need to enter the pincode:123456 to match
//				blc_smp_enableAuthMITM (1, 123456);//pincode
//				blc_smp_setIoCapability (IO_CAPABLITY_DISPLAY_ONLY);

/* ?			bls_l2cap_requestConnParamUpdate(
						ble_con_ini.intervalMin,
						ble_con_ini.intervalMax,
						ble_con_ini.latency,
						ble_con_ini.timeout); */
	#endif
#endif
			}
				break;
			case BLT_EV_FLAG_DATA_LENGTH_EXCHANGE:
				break;
			case BLT_EV_FLAG_PAIRING_END:
				break;
			case BLT_EV_FLAG_ENCRYPTION_CONN_DONE:
				break;
			case BLT_EV_FLAG_GPIO_EARLY_WAKEUP:
				break;
			case BLT_EV_FLAG_CHN_MAP_REQ:
				break;
			case BLT_EV_FLAG_CONN_PARA_REQ:
			{
				rf_packet_ll_updateConnPara_t p;
				cur_ble_con_ini.intervalMin |= 1;
				cur_ble_con_ini.intervalMax = p.interval; // new connection interval in unit of 1.25ms
				cur_ble_con_ini.latency = p.latency; // new connection latency
				cur_ble_con_ini.timeout = p.timeout; // new connection timeout in unit of 10ms
/*				//Slave received Master's LL_Connect_Update_Req pkt.
				rf_packet_ll_updateConnPara_t p;
				memcpy((u8*)&p.winSize, para, 11);

				printf("Receive Master's LL_Connect_Update_Req pkt.\n");
				printf("Connection interval:%dus.\n", p.interval*1250);
*/
			}
				break;
			case BLT_EV_FLAG_CHN_MAP_UPDATE:
			{
			}
				break;
			case BLT_EV_FLAG_CONN_PARA_UPDATE:
			{
				cur_ble_con_ini.intervalMin |= 2;
				cur_ble_con_ini.intervalMax = para[0] | para[1]<<8; // new connection interval in unit of 1.25ms
				cur_ble_con_ini.latency = para[2] | para[3]<<8; // new connection latency
				cur_ble_con_ini.timeout = para[4] | para[5]<<8; // new connection timeout in unit of 10ms
//				para[0] | para[1]<<8; // new connection interval in unit of 1.25ms
//				para[2] | para[3]<<8; // new connection latency
//				para[4] | para[5]<<8; // new connection timeout in unit of 10ms
/*
				printf("Update param event occur.\n");
				printf("Current Connection interval:%dus.\n", bls_ll_getConnectionInterval() * 1250);
*/
			}
				break;
			case BLT_EV_FLAG_ADV_DURATION_TIMEOUT:
				// set new advertising
			break;
			case BLT_EV_FLAG_SUSPEND_ENTER:
				if(sleep_disable) {
					bls_pm_setSuspendMask(SUSPEND_DISABLE);
					bls_pm_setManualLatency(0);
				}
				else {
				}
				break;
			case BLT_EV_FLAG_SUSPEND_EXIT:
#if LOOP_MIN_CYCLE
				ble_loop_count = 0;
#endif
#if USE_I2C_INA
				if(t_rd_us) {
					GetNewRegData();
				}
#endif
//				tick_wakeup = clock_time();
				break;
			default:
				break;
		}
	}
	return 0;
}

void entry_ota_mode(void)
{
	sleep_disable = 1;

	bls_ota_setTimeout(100 * 1000000); //set OTA timeout  100 S
#ifdef BLUE_LED
	gpio_write(BLUE_LED, ON); // LED on for indicate OTA mode
#endif
}

void show_ota_result(int result)
{
#if 0
	if(result == OTA_SUCCESS){
		for(int i=0; i< 8;i++){  //4Hz shine for  4 second
			gpio_write(BLUE_LED, 0);
			sleep_us(125000);
			gpio_write(BLUE_LED, 1);
			sleep_us(125000);
		}
	}
	else{
		for(int i=0; i< 8;i++){  //1Hz shine for  4 second
			gpio_write(BLUE_LED, 0);
			sleep_us(500000);
			gpio_write(BLUE_LED, 1);
			sleep_us(500000);
		}

		//write_reg8(0x8000,result); ;while(1);  //debug which err lead to OTA fail
	}


	gpio_write(BLUE_LED, 0);
#endif
#ifdef BLUE_LED
	gpio_write(BLUE_LED, OFF); //LED on for indicate OTA mode end ?
#endif
}

void led_init(void) {
#ifdef GREEN_LED
	gpio_write(GREEN_LED,OFF);
#endif
#ifdef BLUE_LED
	gpio_write(BLUE_LED,OFF);
#endif
}

int MtuSizeExchanged_callback(u16 conn, u16 txmtu) {
	(void)conn;
	if(wrk_tick == 0
		&& txmtu >= MTU_DATA_SIZE
		) {
		wrk_tick = 1;
		all_rd_count = 0;
		not_send_count = 0;
		rx_len = 0;
		tx_len = 0;
#if USE_I2C_INA
//		I2CDevWakeUp();
//		FlushI2CBuf();
#endif
#if USE_ADC_TST
//		reg_audio_wr_ptr = 0;
//		dfifo_rd_ptr = 0;
#endif
	}
	return 0;
}

void eep_init(void)
{
	if (flash_read_cfg(&ble_con_ini, EEP_ID_CON_CFG, sizeof(dev_cfg)) != sizeof(ble_con_ini)) {
		memcpy(&ble_con_ini, &def_ble_con_ini, sizeof(ble_con_ini));
	}
	if (flash_read_cfg(&ble_adv_ini, EEP_ID_ADV_CFG, sizeof(dev_cfg)) != sizeof(ble_adv_ini)) {
		memcpy(&ble_adv_ini, &def_ble_adv_ini, sizeof(ble_adv_ini));
	}
#if (BATT_SERVICE_ENABLE)
	if (flash_read_cfg(&dev_cfg, EEP_ID_DEV_CFG, sizeof(dev_cfg)) != sizeof(dev_cfg)) {
		dev_cfg = dev_cfg_def;
//		flash_write_cfg(&dev_cfg, EEP_ID_DEV_CFG, sizeof(dev_cfg));
	}
	if (dev_cfg.vbat_check_step_ms < 50)
		dev_cfg.vbat_check_step_ms = 50;
#endif
#if USE_I2C_INA
	if (flash_read_cfg(&cfg_i2c, EEP_ID_I2C_CFG, sizeof(cfg_i2c)) != sizeof(cfg_i2c)) {
		memcpy(&cfg_i2c, &def_cfg_i2c, sizeof(cfg_i2c));
	}
	cfg_i2c.pktcnt = 0; // read off
//	I2CDevSleep();
#endif
#if USE_ADC_TST
	if (flash_read_cfg(&cfg_adc, EEP_ID_ADC_CFG, sizeof(cfg_adc)) != sizeof(cfg_adc)) {
		memcpy(&cfg_adc, &def_cfg_adc, sizeof(cfg_adc));
	}
//	cfg_adc.pktcnt = 0;
	ADC_Stop();
#endif	
}

void ble_init(void) {
	rf_drv_init(CRYSTAL_TYPE);
#if (BLE_MODULE_PM_ENABLE)
	/***********************************************************************************
	 * These section must be before battery_power_check.
	 * Because when low battery,chip will entry deep.if placed after battery_power_check,
	 * it is possible that can not wake up chip.
	 * mcu can wake up module from suspend or deepsleep by pulling up GPIO_WAKEUP_MODULE
	 *  *******************************************************************************/
//	gpio_set_wakeup		(GPIO_WAKEUP_MODULE, 1, 1);  // core(gpio) high wakeup suspend
//	cpu_set_gpio_wakeup (GPIO_WAKEUP_MODULE, 1, 1);  // pad high wakeup deepsleep

//	GPIO_WAKEUP_MODULE_LOW; // gpio_setup_up_down_resistor(GPIO_PC5, PM_PIN_PULLDOWN_100K);
//	bls_pm_registerFuncBeforeSuspend(&app_suspend_enter);
#endif
#if (BATT_CHECK_ENABLE)  // battery check must do before OTA relative operation
	/*****************************************************************************************
	 Note: battery check must do before any flash write/erase operation, cause flash write/erase
		   under a low or unstable power supply will lead to error flash operation
		   Some module initialization may involve flash write/erase, include: OTA initialization,
				SMP initialization, ..
				So these initialization must be done after  battery check
	*****************************************************************************************/
	if(analog_read(DEEP_ANA_REG2) == BATTERY_VOL_LOW){
		battery_power_check(VBAT_ALARM_THRES_MV + 200); // 2.2V
	} else {
		battery_power_check(VBAT_ALARM_THRES_MV); // 2.0 V
	}
#endif
	///////////// BLE stack Initialization ////////////////
	/////////////////////////////////////////////////////////////
	u8  tbl_mac [] = {0xe1, 0xe1, 0xe2, 0xe3, 0xe4, 0xc7};
	u32 *pmac = (u32 *) CFG_ADR_MAC;
	if (*pmac != 0xffffffff)
		memcpy (tbl_mac, pmac, 6);
	else {
		//TODO : should write mac to flash after pair OK
		tbl_mac[0] = (u8)rand();
		tbl_mac[1] = (u8)rand();
		flash_write_page(CFG_ADR_MAC, 6, tbl_mac);
	}
	////// Controller Initialization  //////////
	blc_ll_initBasicMCU(tbl_mac);   // mandatory

	blc_ll_initAdvertising_module(tbl_mac); 	// adv module: 		 mandatory for BLE slave,
	blc_ll_initSlaveRole_module();				// slave module: 	 mandatory for BLE slave,
	blc_ll_initPowerManagement_module();        // pm module:      	 optional

	//ATT initialization
	if((MTU_RX_DATA_SIZE) > 23)
		blc_att_setRxMtuSize(MTU_RX_DATA_SIZE); 	// If not set RX MTU size, default is: 23 bytes.
//	blc_ll_exchangeDataLength(LL_LENGTH_RSP, DLE_DATA_SIZE);// LL_LENGTH_REQ, LL_LENGTH_RSP );
	////// Host Initialization  //////////
	extern void my_att_init();
	my_att_init(); // gatt initialization
	blc_l2cap_register_handler(blc_l2cap_packet_receive);  	// l2cap initialization
#if SIG_PROC_ENABLE
	blc_l2cap_reg_att_sig_hander(att_sig_proc_handler);         //register sig process handler
#endif
 	//// smp initialization ////
#if (BLE_MODULE_SECURITY_ENABLE)
	//Just work encryption: TK default is 0, that is, pin code defaults to 0, without setting
	//Passkey entry encryption: generate random numbers, or set the default pin code, processed in the event_handler function
	blc_smp_param_setBondingDeviceMaxNumber(4);  	//default is SMP_BONDING_DEVICE_MAX_NUM, can not bigger that this value
													//and this func must call before bls_smp_enableParing
	bls_smp_enableParing(SMP_PARING_CONN_TRRIGER);

	//blc_smp_enableScFlag(1);
	//blc_smp_setEcdhDebugMode(1);//debug
	#if 1
		blc_smp_enableAuthMITM (1, 123456);
	#endif
#else
	bls_smp_enableParing(SMP_PARING_DISABLE_TRRIGER);
#endif

	//HID_service_on_android7p0_init();  //hid device on android 7.0/7.1

	///////////////////// USER application initialization ///////////////////
	bls_ll_setScanRspData((u8 *)tbl_scanRsp, sizeof(tbl_scanRsp));
	bls_ll_setAdvData((u8 *)tbl_advData, sizeof(tbl_advData) );

	////////////////// config adv packet /////////////////////
	if(bls_ll_setAdvParam(
			ble_adv_ini.intervalMin,
			ble_adv_ini.intervalMax,
			ADV_TYPE_CONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC,
			0,  NULL,
			MY_APP_ADV_CHANNEL,
			ADV_FP_NONE) != BLE_SUCCESS) {
			cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_TIMER, clock_time() + 5000 * CLOCK_SYS_CLOCK_1MS); // <3.5 uA
			while(1);
	}
	bls_ll_setAdvEnable(1);  //adv enable
/*
	bls_l2cap_requestConnParamUpdate(
			ble_con_ini.intervalMin,
			ble_con_ini.intervalMax,
			ble_con_ini.latency,
			ble_con_ini.timeout);
*/
	rf_set_power_level_index(RF_POWER_8dBm);

//	bls_app_registerEventCallback(BLT_EV_FLAG_PAIRING_BEGIN, &ble_paring);
//	bls_app_registerEventCallback(BLT_EV_FLAG_PAIRING_END, &ddddd);	 //success or fail(with fail reason)

//	bls_app_registerEventCallback(BLT_EV_FLAG_CONNECT, &task_connect);
//	bls_app_registerEventCallback(BLT_EV_FLAG_TERMINATE, &ble_remote_terminate);

	bls_pm_setSuspendMask(SUSPEND_DISABLE);	//(SUSPEND_ADV | SUSPEND_CONN)

//	extern int event_handler(u32 h, u8 *para, int n);
	blc_hci_registerControllerEventHandler(event_handler);		//register event callback
	bls_hci_mod_setEventMask_cmd(0xfffff);			//enable all 18 events,event list see ble_ll.h

	// OTA init
	bls_ota_clearNewFwDataArea(); //must
	bls_ota_registerStartCmdCb(entry_ota_mode);
	bls_ota_registerResultIndicateCb(show_ota_result);

#if(__TL_LIB_8266__ || MCU_CORE_TYPE == MCU_CORE_8266)
	blc_pm_disableFlashShutdown_when_suspend();
#endif
	blc_att_registerMtuSizeExchangeCb(&MtuSizeExchanged_callback);
//	gpio_set_wakeup(GPIO_WAKEUP_MODULE, 1, 1);  // core(gpio) high wakeup suspend
//	cpu_set_gpio_wakeup(GPIO_WAKEUP_MODULE, 1, 1);  // pad high wakeup deepsleep
}

void user_init()
{
#if 0
	REG_ADDR8(0x74) = 0x53;
#if	(CHIP_TYPE == CHIP_TYPE_8269)
	REG_ADDR16(0x7e) = 0x08d1;
#else
	REG_ADDR16(0x7e) = 0x08d0; // = 0x5325
#endif
	REG_ADDR8(0x74) = 0x00;
#endif
	GPIO_WAKEUP_MODULE_LOW;
//	led_init();
	eep_init();
/////////////////////////////////////////////////////////////
	if (tst_usb_actived()) {
#if 1 // hw init TLSR8266/TLSR8269
		reg_rst_clk0 = 0
#if USE_SPI
				| FLD_CLK_SPI_EN
#endif
#if USE_I2C
//				| FLD_CLK_I2C_EN // включается в I2CBusInit()
#endif
#if USE_USB
				| FLD_CLK_USB_EN
				| FLD_CLK_USB_PHY_EN
#endif
				| FLD_CLK_MCU_EN
				| FLD_CLK_MAC_EN
#if USE_ADC_TST
				| FLD_CLK_ADC_EN
#endif
//				| FLD_CLK_ZB_EN
				;
		// reg_clk_en + reg_clk_sel + reg_i2s_step = 0x002C10f7
		REG_ADDR32(0x64) = 0x000C000 // FLD_CLK_SEL_DIV(0x0c), FLD_CLK_SEL_SRC(0x02)
		// reg_clk_sel [0x66], After reset = 0x06
		| ((MASK_VAL(FLD_CLK_SEL_DIV, CLOCK_PLL_CLOCK/CLOCK_SYS_CLOCK_HZ, FLD_CLK_SEL_SRC, CLOCK_SEL_HS_DIV))<<16)

				| FLD_CLK_GPIO_EN
				| FLD_CLK_ALGM_EN
#if USE_USB | USE_DMA
				| FLD_CLK_DMA_EN
#endif
#if USE_UART
				| FLD_CLK_UART_EN
#endif
#if USE_PWM
				| FLD_CLK_PWM_EN
#endif
#if USE_AES
				| FLD_CLK_AES_EN
#endif
//				| FLD_CLK_32K_TIMER_EN	// clk32k for system timer
				| FLD_CLK_PLL_EN
				| FLD_CLK_SWIRE_EN
//				| FLD_CLK_32K_QDEC_EN	// 32k for qdec
#if USE_AUD
				| FLD_CLK_AUD_EN
#endif
#if USE_DFIFO
				| FLD_CLK_DIFIO_EN
#endif
#if USE_KEYSCAN
				| FLD_CLK_KEYSCAN_EN
#endif
				| FLD_CLK_MCIC_EN
#if	USE_QDEC
				| FLD_CLK_QDEC_EN
#endif
				;
		REG_ADDR32(0x70) = 0 // = 0x04000400
		/* reg_fhs_sel [0x70], After reset = 0x00 */
			| (0) // bit1 FHS sel: 192M clock from pll | 32M clock from rc osc
		/* reg_dcdc_clk [0x71], After reset [0x71] = 0x04 */
			| ((1<<2)<<8)
		/* reg_?? [0x72], After reset [0x72] = 0x00 */
			| (0<<16) // watchdog reset status bit 0x72[0] = 1, manually clear - write '1'
		/* reg_clk_mux_cel [0x73], After reset  = 0x14
		* [0] clk32k select; 0: sel 32k osc 1: 32k pad
		* [1] dmic clock select, 1: select 32k (refer to bit[0] to decide which 32k ; 0: dmic clk div
		* [2] usb phy clock select, 1 : 192M divider 0: 48M pll
		* [7:4] r_lpr_div, decide system clock speed in low power mode	 */
			| ((1<<2)<<24);
#if CRYSTAL_TYPE == XTAL_12M
		//rf_set_12M_Crystal_2m_mode();
		reg_pll_ctrl_a = FLD_PLL_A_CAL_DONE_EN | 0x80;
		analog_write(0x099, 0xb1);
		analog_write(0x082, 0x20);
		analog_write(0x09e, 0xad);
#elif CRYSTAL_TYPE == XTAL_16M
		// rf_set_16M_Crystal_2m_mode();
		reg_pll_ctrl_a = FLD_PLL_A_CAL_DONE_EN;
		analog_write(0x099, 0x31);
		analog_write(0x082, 0x34);
		analog_write(0x09e, 0x82);
#else
#error Set CRYSTAL_TYPE!
#endif
#endif // hw init
		/* Initialize usb cdc */
		USB_Init();
		USBCDC_RxBufSet(usb_buf.rx);
		USBCDC_CBSet(USBCDC_RxCb, NULL); // CDC_TxDoneCb);
		usb_dp_pullup_en(1);
#if 0
		if (!usb_actived) reg_irq_mask &= ~(0
//			| FLD_IRQ_USB_PWDN_EN
			| FLD_IRQ_EP0_SETUP_EN
			| FLD_IRQ_EP0_DAT_EN
			| FLD_IRQ_EP0_STA_EN
			| FLD_IRQ_SET_INTF_EN
			| FLD_IRQ_EP_DATA_EN
//			| FLD_IRQ_USB_250US_EN
			| FLD_IRQ_USB_RST_EN );
#endif
	} else {
		ble_init();
		reg_rst_clk0 = 0
#if USE_SPI
				| FLD_CLK_SPI_EN
#endif
#if USE_I2C
//				| FLD_CLK_I2C_EN // включается в I2CBusInit()
#endif
#if USE_USB
//				| FLD_CLK_USB_EN
//				| FLD_CLK_USB_PHY_EN
#endif
				| FLD_CLK_MCU_EN
				| FLD_CLK_MAC_EN
#if USE_ADC_TST
//				| FLD_CLK_ADC_EN
#endif
				| FLD_CLK_ZB_EN
				;
		reg_clk_en = 0
				| FLD_CLK_GPIO_EN
				| FLD_CLK_ALGM_EN
#if USE_DMA
				| FLD_CLK_DMA_EN
#endif
#if USE_UART
				| FLD_CLK_UART_EN
#endif
#if USE_PWM
				| FLD_CLK_PWM_EN
#endif
#if USE_AES
//				| FLD_CLK_AES_EN
#endif
//				| FLD_CLK_32K_TIMER_EN	// clk32k for system timer
//				| FLD_CLK_PLL_EN
//				| FLD_CLK_SWIRE_EN
//				| FLD_CLK_32K_QDEC_EN	// 32k for qdec
#if USE_AUD
				| FLD_CLK_AUD_EN
#endif
#if USE_DFIFO
				| FLD_CLK_DIFIO_EN
#endif
#if USE_KEYSCAN
				| FLD_CLK_KEYSCAN_EN
#endif
				| FLD_CLK_MCIC_EN
#if	USE_QDEC
				| FLD_CLK_QDEC_EN
#endif
				;
	}
//	reg_dma_chn_en = 0; // ?
}

/////////////////////////////////////////////////////////////////////
// main ble loop flow
/////////////////////////////////////////////////////////////////////
void main_ble_loop() {
	static 	u32 tick_start; // для ожидания переключения MTU
	u32 i;
	////////////////////////////////////// BLE entry /////////////////////////////////
#if LOOP_MIN_CYCLE
	ble_loop_count++;
#endif
	blt_sdk_main_loop();
//	if(tst_usb_actived()) {
//		reg_pwdn_ctrl = FLD_PWDN_CTRL_REBOOT;
//		if(device_in_connection_state) bls_ll_terminateConnection(HCI_ERR_REMOTE_USER_TERM_CONN);
//		cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD | PM_WAKEUP_TIMER, clock_time() + 5 * CLOCK_SYS_CLOCK_1MS);
//	}
	if((device_in_connection_state // blc_ll_getCurrentState() == BLS_LINK_STATE_CONN
			&& SppDataServer2ClientDataCCC)) {
		if(!wrk_enable) {
//#if USE_I2C_INA
			ExtDevPowerOn();
//#endif
			//  packet_length 20 + 27 * x байт, MTU_DATA_SIZE = packet_length +7
			i = blc_att_requestMtuSizeExchange(BLS_CONN_HANDLE, MTU_DATA_SIZE);
			if (i != BLE_SUCCESS) {
				send_ble_err(RTERR_MTEX, i);
				wrk_tick = 0xff;
			}
			else {
				tick_start = clock_time();
				wrk_tick = 0;
				I2CDevWakeUp();
			}
			wrk_enable = 1;
			connection_time = clock_time();
		}
		else { // wrk_enable
			switch(wrk_tick) {
			case 1: // рабочий цикл
				if(tx_len) { // требуется передача
					if(blc_ll_getTxFifoNumber() < 10) {
						i = bls_att_pushIndicateData(SPP_Server2Client_INPUT_DP_H, (u8 *) &send_pkt, tx_len);
//						if(i == HCI_ERR_CONN_NOT_ESTABLISH) {
//							tx_len = 0;
//							wrk_tick = 0xff;
//						}
						if(i == BLE_SUCCESS) {
							tx_len = 0;
						} else if(i != ATT_ERR_PREVIOUS_INDICATE_DATA_HAS_NOT_CONFIRMED) {
							send_ble_err(RTERR_PIND, i);
							wrk_tick = 0xff;
						}
					}// else not_send_count++;
//					tick_wakeup = clock_time();
				}// else
				if(!tx_len) {
#if USE_ADC_TST
				if(cfg_adc.pktcnt // вывод ADC samples активен
//					&& blc_ll_getTxFifoNumber() < 10
#if 1
					&& (i = get_adc_dfifo((u16 *)&send_pkt.data.ui, cfg_adc.pktcnt, SMPS_BLK_CNT)) != 0) {
					all_rd_count += i;
					send_pkt.head.cmd = CMD_DEV_ADC;
					send_pkt.head.size = i*2;
					tx_len = i*2+sizeof(blk_head_t);
					if(blc_ll_getTxFifoNumber() < 10) {
						i = bls_att_pushIndicateData(SPP_Server2Client_INPUT_DP_H, (u8 *) &send_pkt, tx_len);
						if(i == BLE_SUCCESS) {
							tx_len = 0;
						} else if(i != ATT_ERR_PREVIOUS_INDICATE_DATA_HAS_NOT_CONFIRMED) {
							send_ble_err(RTERR_PIND, i);
							wrk_tick = 0xff;
						}
					}
#else
					&& (i = get_adc_dfifo((u16 *)&send_pkt.data.ui, cfg_adc.pktcnt, SMPS_BLK_CNT) != 0) {
					all_rd_count += cfg_adc.pktcnt;
					send_pkt.head.cmd = CMD_DEV_ADC;
					send_pkt.head.size = cfg_adc.pktcnt*2;
					tx_len = cfg_adc.pktcnt*2+sizeof(blk_head_t);
					if(blc_ll_getTxFifoNumber() < 10) {
						i = bls_att_pushIndicateData(SPP_Server2Client_INPUT_DP_H, (u8 *) &send_pkt, tx_len);
						if(i == BLE_SUCCESS) {
							tx_len = 0;
						} else if(i != ATT_ERR_PREVIOUS_INDICATE_DATA_HAS_NOT_CONFIRMED) {
							send_ble_err(RTERR_PIND, i);
							wrk_tick = 0xff;
						}
					}
#endif
				} else
#endif
#if USE_I2C_INA
				if(cfg_i2c.pktcnt
//					&&	blc_ll_getTxFifoNumber() < 10
					&& ((i2c_buf_wr - i2c_buf_rd) & (I2C_BUF_SIZE - 1)) > cfg_i2c.pktcnt) {
					all_rd_count += cfg_i2c.pktcnt;
					for(i = 0; i < cfg_i2c.pktcnt ; i ++) {
						send_pkt.data.si[i] = i2c_buf[i2c_buf_rd++];
#if (I2C_BUF_SIZE != 256)
						i2c_buf_rd &= I2C_BUF_SIZE - 1;
#endif
					}
					send_pkt.head.cmd = CMD_DEV_I2C;
					send_pkt.head.size = cfg_i2c.pktcnt*2;
					tx_len = cfg_i2c.pktcnt*2+sizeof(blk_head_t);
					if(blc_ll_getTxFifoNumber() < 10) {
						i = bls_att_pushIndicateData(SPP_Server2Client_INPUT_DP_H, (u8 *) &send_pkt, tx_len);
						if(i == BLE_SUCCESS) {
							tx_len = 0;
						} else if(i != ATT_ERR_PREVIOUS_INDICATE_DATA_HAS_NOT_CONFIRMED) {
							send_ble_err(RTERR_PIND, i);
							wrk_tick = 0xff;
						}
					}
				} else
#endif
				if(rx_len) { // пришла команда
					tx_len = cmd_decode(&send_pkt, &read_pkt, rx_len);
					connection_time = clock_time();
					rx_len = 0;
				} else if(clock_tik_exceed(connection_time, 25000000*CLOCK_SYS_CLOCK_1US)) { // > 25 sec ?
					wrk_tick = 0xff; // на отключение
				}
				} // if(!tx_len)
				break;
			case 0: // ожидание переключения MTU
				if(clock_tik_exceed(tick_start, 500000*CLOCK_SYS_CLOCK_1US)) { // > 500 ms ?
					send_ble_err(RTERR_TOMT, 5000); // time
					wrk_tick = 0xff; // на отключение
				}
				break;
			default: // отключение
				bls_ll_terminateConnection(HCI_ERR_REMOTE_USER_TERM_CONN);
				SppDataServer2ClientDataCCC = 0;
				break;
			}
		} // wrk_enable
	}
	else { // отключение
		if(wrk_enable) {
			sdm_off();
#if USE_I2C_INA		
			Timer_Stop();
			I2CDevSleep();
#endif
#if USE_ADC_TST
			ADC_Stop();
#endif			
			ExtDevPowerOff();
		}
		wrk_tick = 0xff;
		wrk_enable = 0;
		if(device_in_connection_state
			&& clock_tik_exceed(connection_time, 10000000*CLOCK_SYS_CLOCK_1US)) { // > 10 sec ?
			bls_ll_terminateConnection(HCI_ERR_REMOTE_USER_TERM_CONN);
			SppDataServer2ClientDataCCC = 0;
		}
	}
//-------------------------------------------------
	/* if(sleep_disable) в обр. события BLT_EV_FLAG_SUSPEND_ENTER */
	if(!sleep_disable) {
		if(!device_in_connection_state) {
			if(!gpio_read(KEY_K1)) {
				gpio_setup_up_down_resistor(KEY_K1, PM_PIN_PULLDOWN_100K);
				cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_TIMER, clock_time() + 30000 * CLOCK_SYS_CLOCK_1MS);
			}
			else if(!gpio_read(KEY_K2)) {
				gpio_setup_up_down_resistor(KEY_K2, PM_PIN_PULLDOWN_100K);
				gpio_set_wakeup		(KEY_K2, 1, 1);  // core(gpio) high wakeup suspend
				cpu_set_gpio_wakeup (KEY_K2, 1, 1);  // pad high wakeup deepsleep
				cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);
			}
		}
#if USE_ADC_TST
		if(cfg_adc.pktcnt) {
			bls_pm_setSuspendMask(SUSPEND_DISABLE);
/*
#if LOOP_MIN_CYCLE
			if(tx_len) // && ble_loop_count < LOOP_MIN_CYCLE)
				bls_pm_setSuspendMask(SUSPEND_DISABLE);
			else {
//				ble_loop_count += LOOP_MIN_CYCLE;
				bls_pm_setSuspendMask(MCU_STALL);
			}
#else
			bls_pm_setSuspendMask(MCU_STALL);
#endif
*/
		} else
#endif
		{
			bls_pm_setSuspendMask(MCU_STALL | SUSPEND_ADV | SUSPEND_CONN);
//			bls_pm_setSuspendMask(SUSPEND_ADV | SUSPEND_CONN);
		}
		bls_pm_setWakeupSource(PM_WAKEUP_CORE | PM_WAKEUP_TIMER);  // GPIO_WAKEUP_MODULE needs to be wakened, PM_WAKEUP_TIMER ?
	}
}
/////////////////////////////////////////////////////////////////////
// main usb loop flow
/////////////////////////////////////////////////////////////////////
#define usb_pwd wrk_enable
#define usb_pwup wrk_tick
void main_usb_loop() {
	u32 i;
	if(tx_len) { // есть данные для передачи
		if(USBCDC_IsAvailable()) {
			memcpy(&usb_buf.tx, &send_pkt, tx_len);
			USBCDC_DataSend((unsigned char *)&usb_buf.tx, tx_len);
			tx_len = 0;
		}
	} else
#if USE_ADC_TST
	if(cfg_adc.pktcnt
		&& (i = get_adc_dfifo((u16 *)&send_pkt.data.ui, cfg_adc.pktcnt, SMPS_BLK_CNT)) != 0) {
		all_rd_count += i;
		if(!USBCDC_IsAvailable())
			not_send_count++;
		send_pkt.head.size = i*2;
		send_pkt.head.cmd = CMD_DEV_ADC;
		tx_len = i*2+sizeof(blk_head_t);
	} else
#endif
#if USE_I2C_INA	
	if(cfg_i2c.pktcnt
//		&& timer_flg
		&& ((i2c_buf_wr - i2c_buf_rd) & (I2C_BUF_SIZE - 1)) > cfg_i2c.pktcnt) {
		for(i = 0; i < cfg_i2c.pktcnt ; i ++) {
			send_pkt.data.si[i] = i2c_buf[i2c_buf_rd];
			i2c_buf_rd++;
#if (I2C_BUF_SIZE != 256)
			i2c_buf_rd &= I2C_BUF_SIZE - 1;
#endif
		}
		all_rd_count+=cfg_i2c.pktcnt;
		if(!USBCDC_IsAvailable())
			not_send_count++;
		send_pkt.head.size = cfg_i2c.pktcnt*2;
		send_pkt.head.cmd = CMD_DEV_I2C;
		tx_len = cfg_i2c.pktcnt*2+sizeof(blk_head_t);
	} else
#endif
	if(rx_len) {
		tx_len = cmd_decode(&send_pkt, &read_pkt, rx_len);
		rx_len = 0;
	} else
	if(usb_pwd) { // Events: USB_SET_CTRL_UART DTR Off, USB_PWDN, USB_RESET
		sdm_off();
#if USE_I2C_INA
		Timer_Stop();
		I2CDevSleep();
#endif
#if USE_ADC_TST
		ADC_Stop();
#endif
		ExtDevPowerOff();
		usb_pwd = 0;
	}
	else if(usb_pwup) { // Events: USB_SET_CTRL_UART DTR Off, USB_PWDN, USB_RESET
		ExtDevPowerOn();
#if USE_I2C_INA
		sleep_us(100);
		I2CDevWakeUp();
#endif
		usb_pwup = 0;
	}
}

