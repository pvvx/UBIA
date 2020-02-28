/*
 * ble.c
 *
 *  Created on: 22.02.2020
 *      Author: pvvx
 */
#include "proj/tl_common.h"
#if (USE_BLE)
#include "ble.h"
#include "app.h"

#include "flash_eep.h"
#include "eep_id.h"
#if (USE_USB_CDC)
#include "usb.h"
#endif
#if (USE_I2C_DEV)
#include "i2c_dev.h"
#endif
#if (USE_INT_ADC)
#include "adc_dev.h"
#endif
#if (USE_HX711)
#include "hx711.h"
#endif
#if (USE_INT_DAC)
#include "dac.h"
#endif
#if	(USE_INT_UART)
#include "uart_dev.h"
#endif

extern u16 SppDataServer2ClientDataCCC;

#define CONNECTION_START_WAIT_TIME_US 	(10*1000000) // 10 sec
#define CONNECTION_PING_WAIT_TIME_US 	(25*1000000) // 25 sec

#define SET_TX_MTU  0

#define LOOP_MIN_CYCLE 0
#if LOOP_MIN_CYCLE
u32 ble_loop_count;
#endif

volatile unsigned char sleep_mode = 0; // flag, = 1 -> pm not sleep, = 2 -> cpu only sleep
u32 connection_ping_time;
u8 device_in_connection_state = 0; // flag
u8 wrk_enable = 1; // flag connect & run device, in BLE mode
u8 wrk_stage = 0; // stages, in BLE main loop

//----------------------------- BLE stack
my_fifo_t			blt_rxfifo;
u8					blt_rxfifo_b[];

my_fifo_t			blt_txfifo;
u8					blt_txfifo_b[];

MYFIFO_INIT(blt_rxfifo, 64, 8); 	// 512 bytes + headers
MYFIFO_INIT(blt_txfifo, 40, 16);	// 640 bytes + headers

ble_con_t ble_con_ini;
cur_ble_con_t cur_ble_con_ini;
const ble_con_t def_ble_con_ini = {DEF_CONN_PARMS};
ble_cfg_t ble_cfg_ini;
const ble_cfg_t def_ble_cfg_ini = {DEF_ADV_INTERVAL, RF_POWER_8dBm, {BLE_DEV_NAME}};
//const u8 ble_dev_name[8] = { BLE_DEV_NAME, 0 };

//////////////////////////////////////////////////////////////////////////////
//	Adv Packet, Response Packet
//////////////////////////////////////////////////////////////////////////////
const u8 tbl_advData[] = { // https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile/
//	 0x08, 0x09, 't', 'B', 'L', 'E', 'T', 'S', 'T',
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
	 // 0x02: incomplete list of service class UUID (0xffe0)
	 0x03, 0x02, SERVICE_UUID_SPP & 0xff, (SERVICE_UUID_SPP>>8)& 0xff,
#endif
#if 0
	 // 0xFF: Специфические данные производителя (Manufacturer-Specific Data Flag)
	 // [3,4] Company ID = 0x1234
	 0x03+4, 0xFF, 0x8A, 0x24,
	 0x03,0x02,0x01,0x00,
#endif
};

u8 tbl_scanRsp[sizeof(ble_cfg_ini.name) + 3] = {0};
//	= {
// 0x09: «Полное локальное имя»
// 0x08, 0x09, 't', 'B', 'L', 'E', 'x', 'x', 'x', //scan name "tBLExxx"
// + add_tbl_advData[]
//	};


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

/////////////////////////////////////////////////////////////////////
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

/////////////////////////////////////////////////////////////////////
void entry_ota_mode(void)
{
	sleep_mode = 1;

	bls_ota_setTimeout(100 * 1000000); //set OTA timeout  100 S
#ifdef BLUE_LED
	gpio_write(BLUE_LED, ON); // LED on for indicate OTA mode
#endif
}

/////////////////////////////////////////////////////////////////////
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
#if (SET_TX_MTU)
/////////////////////////////////////////////////////////////////////
int MtuSizeExchanged_callback(u16 conn, u16 txmtu) {
	(void)conn;
	if(wrk_stage == 0
		&& txmtu >= MTU_DATA_SIZE
		) {
		wrk_stage = 1;
	}
	return 0;
}
#endif
/////////////////////////////////////////////////////////////////////
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
				connection_ping_time = clock_time();
				device_in_connection_state = 1;
#if BATT_SERVICE_ENABLE
				adc_hw_initialized = 0;
#endif
			}
				break;
			case BLT_EV_FLAG_TERMINATE:
			{
				cur_ble_con_ini.flag = 0;
				sleep_mode = 0;
				device_in_connection_state = 0;
//				SppDataServer2ClientDataCCC = 0;
				rx_len = 0;
				tx_len = 0;
//				printf("\n\r************** Terminate event occured! **************\n\r");
#ifdef GREEN_LED
				gpio_write(GREEN_LED,OFF);
#endif
				bls_l2cap_requestConnParamUpdate(
							ble_con_ini.intervalMin,
							ble_con_ini.intervalMax,
							ble_con_ini.latency,
							ble_con_ini.timeout);
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

/*			bls_l2cap_requestConnParamUpdate(
						ble_con_ini.intervalMin,
						ble_con_ini.intervalMax,
						ble_con_ini.latency,
						ble_con_ini.timeout); */
	#endif
#endif
			}
				break;
			case BLT_EV_FLAG_DATA_LENGTH_EXCHANGE:
#if 0 // test
			{
				ll_data_extension_t *p = (ll_data_extension_t *)para;
				send_pkt.head.size = sizeof(ll_data_extension_t);
				send_pkt.head.cmd = CMD_DEV_ERR;
				memcpy(&send_pkt.data, p,  sizeof(ll_data_extension_t));
				tx_len = sizeof(blk_head_t) + sizeof(ll_data_extension_t);
//				bls_att_pushIndicateData(SPP_Server2Client_INPUT_DP_H, (u8 *) &send_pkt, sizeof(blk_head_t) + sizeof(ll_data_extension_t));
			}
#endif
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
//				rf_packet_ll_updateConnPara_t *p = (rf_packet_ll_updateConnPara_t *)para;
				cur_ble_con_ini.flag |= 1;
//				cur_ble_con_ini.interval = p->interval; // new connection interval in unit of 1.25ms
//				cur_ble_con_ini.latency = p->latency; // new connection latency
//				cur_ble_con_ini.timeout = p->timeout; // new connection timeout in unit of 10ms
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
				cur_ble_con_ini.flag |= 2;
//				cur_ble_con_ini.interval = para[0] | para[1]<<8; // new connection interval in unit of 1.25ms
//				cur_ble_con_ini.latency = para[2] | para[3]<<8; // new connection latency
//				cur_ble_con_ini.timeout = para[4] | para[5]<<8; // new connection timeout in unit of 10ms
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
				if(sleep_mode&1) {
					bls_pm_setSuspendMask(SUSPEND_DISABLE);
					bls_pm_setManualLatency(0);
				}
				else {
#if USE_HX711
				if(hx711_mode && !gpio_read(HX711_DOUT)) {
					hx711_buf[hx711_wr++] = hx711_get_data(hx711_mode);
					hx711_wr &= HX711_BUF_CNT-1;
					all_rd_count++;
				}
#endif
				}
				break;
			case BLT_EV_FLAG_SUSPEND_EXIT:
#if LOOP_MIN_CYCLE
				ble_loop_count = 0;
#endif
#if USE_I2C_DEV
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

/////////////////////////////////////////////////////////////////////
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
#if ((MTU_RX_DATA_SIZE) > 23)
		blc_att_setRxMtuSize(MTU_RX_DATA_SIZE); 	// If not set RX MTU size, default is: 23 bytes, max 241
#endif
#if (!SET_TX_MTU)
		blc_att_requestMtuSizeExchange(BLS_CONN_HANDLE, MTU_DATA_SIZE);
#endif
//	blc_ll_exchangeDataLength(LL_LENGTH_RSP, DLE_DATA_SIZE);// LL_LENGTH_REQ, LL_LENGTH_RSP );
	////// Host Initialization  //////////
	// gatt initialization

	bls_att_setAttributeTable ((u8 *)my_Attributes);

	for(tbl_scanRsp[0] = 0; tbl_scanRsp[0] < (sizeof(ble_cfg_ini.name)) && ble_cfg_ini.name[tbl_scanRsp[0]]; tbl_scanRsp[0]++) {
		tbl_scanRsp[tbl_scanRsp[0]+2] = ble_cfg_ini.name[tbl_scanRsp[0]];
	}
	tbl_scanRsp[1] = 0x09;
	tbl_scanRsp[tbl_scanRsp[0]+2] = 0;

	bls_att_setDeviceName((u8 *)&tbl_scanRsp[2], tbl_scanRsp[0]);

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
	tbl_scanRsp[0]++;
	bls_ll_setScanRspData((u8 *)tbl_scanRsp, tbl_scanRsp[0]+1);
	bls_ll_setAdvData((u8 *)tbl_advData, sizeof(tbl_advData) );

	////////////////// config adv packet /////////////////////
	if(bls_ll_setAdvParam(
			ble_cfg_ini.intervalMin,
			ble_cfg_ini.intervalMax,
			ADV_TYPE_CONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC,
			0,  NULL,
			MY_APP_ADV_CHANNEL,
			ADV_FP_NONE) != BLE_SUCCESS) {
			cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_TIMER, clock_time() + 5000 * CLOCK_SYS_CLOCK_1MS); // <3.5 uA
			while(1);
	}
/*
	bls_l2cap_requestConnParamUpdate(
			ble_con_ini.intervalMin,
			ble_con_ini.intervalMax,
			ble_con_ini.latency,
			ble_con_ini.timeout);
*/
	rf_set_power_level_index(ble_cfg_ini.rf_power); // = RF_POWER_8dBm

	bls_ll_setAdvEnable(1);  //adv enable

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
#if (SET_TX_MTU)
	blc_att_registerMtuSizeExchangeCb(&MtuSizeExchanged_callback);
#endif
//	gpio_set_wakeup(GPIO_WAKEUP_MODULE, 1, 1);  // core(gpio) high wakeup suspend
//	cpu_set_gpio_wakeup(GPIO_WAKEUP_MODULE, 1, 1);  // pad high wakeup deepsleep

	// отключить лишнее
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
#if USE_INT_ADC
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
/////////////////////////////////////////////////////////////////////
// main ble loop flow
/////////////////////////////////////////////////////////////////////
void main_ble_loop() {
#if (SET_TX_MTU)
	static 	u32 connection_wait_tick; // для ожидания переключения MTU
#endif
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
//#if USE_I2C_DEV
			ExtDevPowerOn();
//#endif
#if (USE_HX711)
//			hx711_wakeup();
#endif
			wrk_enable = 1;
			tx_len = 0;
			rx_len = 0;
			all_rd_count = 0;
			not_send_count = 0;
			connection_ping_time = clock_time();
#if (SET_TX_MTU) // Set MTU
			//  packet_length 20 + 27 * x байт, MTU_DATA_SIZE = packet_length +7
			i = blc_att_requestMtuSizeExchange(BLS_CONN_HANDLE, MTU_DATA_SIZE);
			if (i != BLE_SUCCESS) {
				send_ble_err(RTERR_MTEX, i);
				wrk_stage = 0xff;
			}
			else {
				connection_wait_tick = clock_time();
				wrk_stage = 0;
				tx_len = 0;
				rx_len = 0;
				all_rd_count = 0;
				not_send_count = 0;
				I2CDevWakeUp();
			}
#else
			wrk_stage = 0;
			I2CDevWakeUp();
#endif
		}
		else { // wrk_enable
			switch(wrk_stage) {
			case 1: // рабочий цикл
#if USE_HX711
				if(hx711_mode && !gpio_read(HX711_DOUT)) {
//					bls_pm_setSuspendMask(SUSPEND_DISABLE);
//					gpio_set_wakeup(HX711_DOUT, 0, 0);  // core(gpio) low wakeup disable
					hx711_buf[hx711_wr++] = hx711_get_data(hx711_mode);
					hx711_wr &= HX711_BUF_CNT-1;
					all_rd_count++;
				}
#endif
				if(tx_len) { // требуется передача
					if(blc_ll_getTxFifoNumber() < 10) {
						i = bls_att_pushIndicateData(SPP_Server2Client_INPUT_DP_H, (u8 *) &send_pkt, tx_len);
//						if(i == HCI_ERR_CONN_NOT_ESTABLISH) {
//							tx_len = 0;
//							wrk_stage = 0xff;
//						}
						if(i == BLE_SUCCESS) {
							tx_len = 0;
						} else if(i != ATT_ERR_PREVIOUS_INDICATE_DATA_HAS_NOT_CONFIRMED) {
							send_ble_err(RTERR_PIND, i);
							wrk_stage = 0xff;
						}
					}// else not_send_count++;
//					tick_wakeup = clock_time();
				}// else
				if(!tx_len) {
#if USE_INT_ADC
				if(cfg_adc.pktcnt // вывод ADC samples активен
//					&& blc_ll_getTxFifoNumber() < 10
					&& (i = get_adc_dfifo((u16 *)&send_pkt.data.ui, cfg_adc.pktcnt, SMPS_BLK_CNT)) != 0) {
					all_rd_count += i;
					send_pkt.head.cmd = CMD_DEV_ADC;
					send_pkt.head.size = i*2;
					tx_len = i*2+sizeof(blk_head_t);
				} else
#endif
#if USE_I2C_DEV
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
				} else
#endif
#if USE_HX711
				if(((hx711_wr - hx711_rd) & (HX711_BUF_CNT-1)) > HX711_DATA_OUT) {
					for(i = 0; i < HX711_DATA_OUT; i ++) {
						send_pkt.data.hxo.data[i] = hx711_buf[hx711_rd++];
						hx711_rd &= HX711_BUF_CNT-1;
					}
					send_pkt.head.cmd = CMD_DEV_TST;
					send_pkt.head.size = sizeof(hx711_out_t);
					tx_len = sizeof(hx711_out_t)+sizeof(blk_head_t);
				} else
#endif
#if (USE_INT_UART)
				if(reg_dma_rx_rdy0 & FLD_DMA_UART_RX) {
					tx_len = uart_rx_buff[0];
					memcpy(&send_pkt.data, &uart_rx_buff[4], tx_len);
					reg_dma_irq_src = FLD_DMA_UART_RX;
					send_pkt.head.cmd = CMD_DEV_UAR;
					send_pkt.head.size = tx_len;
					tx_len += sizeof(blk_head_t);
				} else
#endif
				if(rx_len) { // пришла команда
					tx_len = cmd_decode(&send_pkt, &read_pkt, rx_len);
					connection_ping_time = clock_time();
					rx_len = 0;
				} else if(clock_tik_exceed(connection_ping_time, CONNECTION_PING_WAIT_TIME_US*CLOCK_SYS_CLOCK_1US)) { // > 25 sec ?
					wrk_stage = 0xff; // на отключение
				}
				if(tx_len) { // требуется передача
					if(blc_ll_getTxFifoNumber() < 10) {
						i = bls_att_pushIndicateData(SPP_Server2Client_INPUT_DP_H, (u8 *) &send_pkt, tx_len);
//						if(i == HCI_ERR_CONN_NOT_ESTABLISH) {
//							tx_len = 0;
//							wrk_stage = 0xff;
//						}
						if(i == BLE_SUCCESS) {
							tx_len = 0;
						} else if(i != ATT_ERR_PREVIOUS_INDICATE_DATA_HAS_NOT_CONFIRMED) {
							send_ble_err(RTERR_PIND, i);
							wrk_stage = 0xff;
						}
					}// else not_send_count++;
	//					tick_wakeup = clock_time();
				}
				} // if(!tx_len)
				break;
			case 0: // ожидание ???
#if (SET_TX_MTU)
				// ожидание переключения MTU
				if(clock_tik_exceed(connection_wait_tick, 1500000*CLOCK_SYS_CLOCK_1US)) { // > 1500 ms ?
					send_ble_err(RTERR_TOMT, ATT_ERR_DATA_LENGTH_EXCEED_MTU_SIZE);
					wrk_stage = 0xff; // на отключение
				}
#else
				wrk_stage = 1;
#endif
				break;
			default: // отключение
				bls_ll_terminateConnection(HCI_ERR_REMOTE_USER_TERM_CONN);
//				SppDataServer2ClientDataCCC = 0;
				break;
			}
		} // wrk_enable
	}
	else { // отключение
		if(wrk_enable) {
			connection_ping_time = clock_time() + (CONNECTION_START_WAIT_TIME_US*CLOCK_SYS_CLOCK_1US - 250*CLOCK_SYS_CLOCK_1US);
#if USE_I2C_DEV
			Timer_Stop();
			I2CDevSleep();
#endif
#if USE_INT_ADC
			ADC_Stop();
#endif
#if (USE_HX711)
			hx711_go_sleep();
#endif
#if USE_INT_ADC
			uart_deinit();
#endif
			sdm_off();

			ExtDevPowerOff();
		} else	if(device_in_connection_state
			&& clock_tik_exceed(connection_ping_time, CONNECTION_START_WAIT_TIME_US*CLOCK_SYS_CLOCK_1US)) { // > 10 sec ?
			bls_ll_terminateConnection(HCI_ERR_REMOTE_USER_TERM_CONN);
	//			SppDataServer2ClientDataCCC = 0;
		}
		wrk_stage = 0;
		wrk_enable = 0;
	}
//-------------------------------------------------
	/* if(sleep_mode) в обр. события BLT_EV_FLAG_SUSPEND_ENTER */
	if((sleep_mode&1)==0) {
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
#if 1 // (USE_INT_ADC)
		if(sleep_mode&2) {
#if LOOP_MIN_CYCLE
			if(ble_loop_count < LOOP_MIN_CYCLE)
				bls_pm_setSuspendMask(SUSPEND_DISABLE);
			else {
				bls_pm_setSuspendMask(MCU_STALL);
			}
#else
			bls_pm_setSuspendMask(SUSPEND_DISABLE);
#endif
		} else
#endif
#if 1 // (USE_INT_UART  || USE_INT_DAC)
		if(sleep_mode&4) {
			bls_pm_setSuspendMask(MCU_STALL);
		} else
#endif
		{
			bls_pm_setSuspendMask(MCU_STALL | SUSPEND_ADV | SUSPEND_CONN);
		}
//		bls_pm_setWakeupSource(PM_WAKEUP_CORE | PM_WAKEUP_TIMER);  // GPIO_WAKEUP_MODULE needs to be wakened, PM_WAKEUP_TIMER ?
		bls_pm_setWakeupSource(PM_WAKEUP_CORE | PM_WAKEUP_TIMER | PM_WAKEUP_PAD);  // GPIO_WAKEUP_MODULE needs to be wakened, PM_WAKEUP_TIMER ?
	}
}

#endif // USE_BLE

