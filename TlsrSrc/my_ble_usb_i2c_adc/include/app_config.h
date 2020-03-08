/*
 * app_config.h
 *
 *  Created on: 10.11.2019
 *      Author: pvvx
 */
#pragma once

#ifndef _APP_CFG_H_
#define _APP_CFG_H_

#include "proj/mcu/config.h"

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

// Project type:
#define USE_BLE 		1
#define USE_USB_CDC 	1
#define USE_I2C_DEV 	1
#define USE_SPI_DEV 	1
#define USE_ADC_DEV 	1
#define USE_DAC_DEV 	1
#define USE_HX711 		1
#define USE_UART_DEV 	1
// -----

#define INT_DEV_ID	0x1021  // DevID = 0x1021
#define INT_DEV_VER 0x0008  // Ver 1.2.3.4 = 0x1234

////////// BLE product  Information  ////////////
#define DEV_NAME		"tBLETST"
#define BLE_DEV_NAME	't', 'B', 'L', 'E', 'T', 'S', 'T' // current project max name = 16 char!
////////// USB product  Information  ////////////
#define STRING_VENDOR        L"Telink"
#define STRING_PRODUCT       L"USB_BLE_TST"
#define STRING_SERIAL        L"Test123"
extern const unsigned char ble_dev_name[8];
// PnP_ID characteris -> см. my_PnPtrs[]

/*
typedef struct _dev_config_t {
	unsigned short vbat_min_mv; // = VBAT_ALARM_THRES
	unsigned short vbat_nom_mv; // = VBAT_NOM_MV
	unsigned short vbat_adc_k;  // if 8266: 1300*(r1+r2)/r2 else 3*1428 (8269)
	unsigned short vbat_check_step_ms; // in 1/1024 ms
//	unsigned short vbat_zero; // ?
//	unsigned short adv_time_ms; //
//  RF_POWER_8dBm
} dev_config_t;
extern dev_config_t dev_cfg;
*/

/////////////////////HCI ACCESS OPTIONS/////////////////////
#define HCI_USE_USB		0
#define HCI_USE_UART	1
#define HCI_USE_NONE	2
#define HCI_ACCESS		HCI_USE_NONE // HCI_USE_UART

extern unsigned char wrk_enable;
extern unsigned char wrk_stage;
extern volatile unsigned char sleep_mode; // flag, bit0 = 1 -> pm not sleep, bit1 = 1 -> SUSPEND_DISABLE, bit2 = 1 0-> MCU_STALL

/****** module CLK (& IRQ ?, & GPIO ?) *******/
/*
 * USE_WATCHDOG
 *  =0 - WATCHDOG off,
 *  !=0 StartupInits() включит WATCHDOG на USE_WATCHDOG us
 *
 *  минимум при CLK 32 MHz 8192 us (шаг WDT)
 *  максимум при CLK 32 MHz 134 209 536 us (16383*8192)
 */
//#define USE_WATCHDOG				(62*8192) // us

/* StartupInits */
#define USE_ADC			USE_ADC_DEV	// StartupInits() включит CLK ADC
#define USE_AES 		1	// StartupInits() включит CLK AES
#define USE_AUD 		0	// StartupInits() включит CLK AUD
#define USE_USB			1	// StartupInits() включит CLK USB
#define USE_DMA			1	// StartupInits() включит CLK DMA
#define USE_SPI 		1	// StartupInits() включит CLK SPI
#define USE_I2C 		USE_I2C_DEV	// StartupInits() включит CLK I2C
#define USE_UART 		0	// StartupInits() включит CLK UART
#define USE_PWM 		0	// StartupInits() включит CLK PWM
#define USE_KEYSCAN 	0 	// StartupInits() включит CLK ADC
#define USE_QDEC		0	// StartupInits() включит CLK QDEC
#define USE_DFIFO		0	// StartupInits() включит CLK DFIFO

#define USE_TIMER0 		0	// StartupInits() включит TIMER0 с циклом прерывания USE_TIMER0 us
#define USE_TIMER1 		0	// StartupInits() включит TIMER1 с циклом прерывания USE_TIMER1 us
#define USE_TIMER2 		0	// StartupInits() включит TIMER2 с циклом прерывания USE_TIMER2 us

/////////////////// MODULE /////////////////////////////////
#define BLE_MODULE_SECURITY_ENABLE          0
#define BLE_MODULE_PM_ENABLE				1
#define SPP_SERVICE_ENABLE					1
#define SERVICE_UUID_SPP 					0xffe0
#define BLE_MODULE_APPLICATION_ENABLE		1
#define BLE_MODULE_INDICATE_DATA_TO_MCU		1
#define BATT_CHECK_ENABLE       			0   // enable or disable battery voltage detection
#define SIG_PROC_ENABLE 					0   // To known if the Master accepted or rejected Connection_Parameters_Update or not!

#define BATT_SERVICE_ENABLE					BATT_CHECK_ENABLE
#define VBAT_ADC_K 				dev_cfg.vbat_adc_k
#define VBAT_ALARM_THRES_MV		dev_cfg.vbat_min_mv  // 2000 mV low battery alarm, 0% voltage battery
#define VBAT_NOM_MV         	dev_cfg.vbat_nom_mv  // 3300 mV 100% voltage battery

#define WEIGHT_SERVICE_ENABLE 				0
#define BLE_AUDIO_ENABLE					0	// only TLSR8269
#define BLE_ADC_ENABLE						0
#define USE_PGA								0

#define BLE_LOW_POWER						0	// =1 меняет тайминги по умолчанию на максимальные и ...

#if (USE_HX711)
typedef struct _weight_config_t {
	unsigned short check_step_ms; // in 1/1024 ms
	unsigned short coef;  //
	unsigned short zero;
} weight_config_t;

typedef struct _weight_val_t {
	unsigned short kg005;
	unsigned short idx;
	unsigned int adc[4];
}weight_val_t;
extern weight_config_t weight_cfg;
#endif // WEIGHT_SERVICE_ENABLE

typedef struct
{
  /** Minimum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  unsigned short intervalMin;
  /** Maximum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms) */
  unsigned short intervalMax;
  /** Number of LL latency connection events (0x0000 - 0x03e8) */
  unsigned short latency;
  /** Connection Timeout (0x000A - 0x0C80 * 10 ms) */
  unsigned short timeout;
} gap_periConnectParams_t;
#if BLE_LOW_POWER
#define DEF_CONN_PARMS 8, 8, 0, 100
#define DEF_ADV_INTERVAL ADV_INTERVAL_1_28_S, ADV_INTERVAL_1_28_S + 16
#else
//#define DEF_CONN_PARMS 50, 100, 0, 400
#define DEF_CONN_PARMS 20, 100, 0, 200 // 8*1.25 = 10 ms (=5 -> 7.5 ms cамый низкий разрешенный спецификацией !)
#define DEF_ADV_INTERVAL ADV_INTERVAL_400MS*2, ADV_INTERVAL_400MS*2+16
#endif
#define MY_APP_ADV_CHANNEL		BLT_ENABLE_ADV_ALL

extern gap_periConnectParams_t my_periConnParameters;

#define SMP_PASSKEY_ENTRY					1  // enable PINCODE
#define PINCODE_RANDOM_ENABLE				0  // =0 pincode:123456, =1 Randomly generated PINCODE

/////////////////// DEBUG  /////////////////////////////////
//826x module's pin simulate as a uart tx, Just for debugging
#define PRINT_DEBUG_INFO               		0	//open/close myprintf

//////////////////// PRINT DEBUG INFO ///////////////////////
#ifndef PRINT_DEBUG_INFO
#define PRINT_DEBUG_INFO                    0
#endif
/////////////////// Clock  /////////////////////////////////
//#define CLOCK_SYS_TYPE  		CLOCK_TYPE_PLL	//  one of the following:  CLOCK_TYPE_PLL, CLOCK_TYPE_OSC, CLOCK_TYPE_PAD, CLOCK_TYPE_ADC
//#define CLOCK_SYS_CLOCK_HZ  	16000000

/////////////////// watchdog  //////////////////////////////
#define MODULE_WATCHDOG_ENABLE		0
#define WATCHDOG_INIT_TIMEOUT		500  //ms

#define BOARD_JDY_10		1
#define BOARD_E104_BT05		2
#define BOARD_E104_BT10		3

#if	(BOARD == BOARD_JDY_10)
#include "board_jdy_10.h"
#define CRYSTAL_TYPE			XTAL_12M		//  extern 12M crystal
#elif (BOARD == BOARD_E104_BT05)
#include "board_e104_bt05.h"
#define CRYSTAL_TYPE			XTAL_16M		//  extern 16M crystal
#elif (BOARD == BOARD_E104_BT10)
#include "board_e104_bt10.h"
#define CRYSTAL_TYPE			XTAL_16M		//  extern 16M crystal
#else
#undef BOARD
#define BOARD BOARD_JDY_10
//#error Set BOARD!
#include "board_jdy_10.h"
#define CRYSTAL_TYPE			XTAL_12M		//  extern 12M crystal
#endif

//////////////////////////// MODULE PM GPIO	/////////////////////////////////
#if	(BOARD == BOARD_JDY_10)||(BOARD == BOARD_E104_BT05)
// Project use
//======================
//SWS			GPIO_PA0
//KEY_BLE/USB	GPIO_PA1 PWM3
//KEY_K2		GPIO_PA5 <STAT>
//KEY_K1		GPIO_PB0 PWM5
//UDM			GPIO_PB5 USB_DM
//UDP			GPIO_PB6 USB_DP
//EXT_POWER_OFF GPIO_PC0 PWM0 PGA_R
//ADCp			GPIO_PC1 ADC-+PGA_L
//ADC7			GPIO_PC2 ADC-CH7 PGA_R
//ADC9			GPIO_PC4 ADC-CH9
//RX 			GPIO_PC7 ADC-CH10
//TX 			GPIO_PC6 ADC-CH11
//EXT_POWER_4MA GPIO_PD4 <ADV_LED> ADC-CH5
//ADC6d			GPIO_PD5 <PWRC> ADC-CH6-DIFF
//DAC 			GPIO_PE5 SDMN
//(not used)	GPIO_PE6 RTS, SPI_CS
//SDA 			GPIO_PE7 SPI_DI
//(not used)	GPIO_PF0 CTS, SPI_DO
//SCL			GPIO_PF1 SPI_CK
//======================
//-------------------------- Not used
#define	PULL_WAKEUP_SRC_PE6			PM_PIN_PULLUP_1M
#define	PULL_WAKEUP_SRC_PF0			PM_PIN_PULLUP_1M
//-------------------------- Keys
#define	KEY_K1	 					GPIO_PB0 // K1 /DeepSleep 30 sec
#define PB0_FUNC					AS_GPIO
#define PB0_INPUT_ENABLE			1
#define PB0_OUTPUT_ENABLE			0
#define	PULL_WAKEUP_SRC_PB0			PM_PIN_PULLUP_1M

#define	KEY_K2	 					GPIO_PA5 // K2 WaikUp/DeepSleep
#define PA5_FUNC					AS_GPIO
#define PA5_INPUT_ENABLE			1
#define PA5_OUTPUT_ENABLE			0
#define	PULL_WAKEUP_SRC_PA5			PM_PIN_PULLUP_1M
//-------------------------- Key USB/BLE
#define KEY_BLE_USB					GPIO_PA1   // mcu wakeup module (Key PWRC) GPIO_WAKEUP_MODULE
#define	PA1_FUNC					AS_GPIO
#define PA1_INPUT_ENABLE			1
#define	PA1_OUTPUT_ENABLE			0
#define	PA1_DATA_OUT				0
#define	PULL_WAKEUP_SRC_PA1			PM_PIN_PULLDOWN_100K
//-------------------------- OutKey Power
#define EXT_POWER_OFF				GPIO_PC0	// =1 Pow.Off
#define PC0_FUNC					AS_GPIO
#define	PC0_OUTPUT_ENABLE			0
#define	PULL_WAKEUP_SRC_PC0			PM_PIN_PULLDOWN_100K
//-------------------------- Out Power 4 mA
#define EXT_POWER_4MA				GPIO_PD4	// =1 Pow.On
#define PD4_FUNC					AS_GPIO
#define PD4_DATA_STRENGTH			0	// Start 0.7 mA
#define	PULL_WAKEUP_SRC_PD4			PM_PIN_PULLDOWN_100K
//-------------------------- ADC
//#define	PC0_FUNC				AS_GPIO // ?
#define	PC1_FUNC					AS_GPIO	// ADC3, -diff ADC PC1/PC2
#define	PC2_FUNC					AS_GPIO // ADC2, +diff ADC PC2/PC1
#define	PC3_FUNC					AS_GPIO // --- none ---
#define	PC4_FUNC					AS_GPIO // ADC1
//#define	PC6_FUNC				AS_GPIO // ADCx TX
//#define	PC7_FUNC				AS_GPIO // ADCx RX
#define	PD5_FUNC					AS_GPIO // ADC0, -diff
//-------------------------- DAC
#define	PE5_FUNC					AS_GPIO // SDMN
//-------------------------- UART
#define UART_TX						GPIO_PC2	//TX
#define UART_TX						GPIO_PC2	//TX
#define PC6_FUNC					AS_GPIO
#define PC6_INPUT_ENABLE			1
#define	PC6_OUTPUT_ENABLE			0
#define	PULL_WAKEUP_SRC_PC6			PM_PIN_PULLUP_1M

#define UART_RX						GPIO_PC3	//RX
#define PC7_FUNC					AS_GPIO
#define PC7_INPUT_ENABLE			1
#define	PC7_OUTPUT_ENABLE			0
#define	PULL_WAKEUP_SRC_PC7			PM_PIN_PULLUP_1M
//------------------- HX711
#if (USE_HX711)
#define HX711_SCK					GPIO_PC6	//TX
#define HX711_DOUT  				GPIO_PC7	//RX
#endif // USE_HX711

#elif	(BOARD == BOARD_E104_BT10)
// Project use
//======================
//KEY_BLE/USB	GPIO_PA1: DMIC_CLK
//KEY_K2		GPIO_PD2: ADC-CH03
//KEY_K1		GPIO_PD3: ADC-CH04
//----------------------
//ADC			GPIO_PC4: UAR_RTS/PWM4/ADC-CH0?-PGA
//ADC			GPIO_PC5: UART_CTS/PWM5/ADC-CH0?-PGA
//----------------------
//EXT_POWER_OFF GPIO_PA0: DMIC_DI
//EXT_POWER_4MA GPIO_PB1: PWM2_N
//----------------------
//DAC 			GPIO_PE0: PWM0/SDM_P(SDMDAC)
//DAC 			GPIO_PE1: PWM1/SDM_N(SDMDAC)
//----------------------
//SDA 			GPIO_PA3: SPI_DI/I2C_SDA/PWM1
//SCL			GPIO_PA4: SPI_CK/I2C_SCK/PWM1_N
//----------------------
//SCS 			GPIO_PB4: SPI_CN/PWM4
//SDO 			GPIO_PB5: SPI_DO/PWM4_N
//SDI 			GPIO_PB6: SPI_DI/I2C_SDA/PWM5
//SCK			GPIO_PB7: SPI_CK/I2C_SCK/PWM5_N/R2R_ADC_BAT
//----------------------
//RX			GPIO_PC2: RX/PWM2/ADC-CH07
//TX 			GPIO_PC3: TX/PWM3/ADC-CH08
//----------------------
//UDM			GPIO_PE2: USB_DM
//UDP			GPIO_PE3: USB_DP
//----------------------
//SWM			GPIO_PA7: UART_RX/SWM
//SWS			GPIO_PB0: PWM2/SWS
//======================
//-------------------------- Keys
#define	KEY_K1	 					GPIO_PD2 // K1 /DeepSleep 30 sec
#define PD2_FUNC					AS_GPIO
#define PD2_INPUT_ENABLE			1
#define PD2_OUTPUT_ENABLE			0
#define	PULL_WAKEUP_SRC_PD2			PM_PIN_PULLUP_1M

#define	KEY_K2	 					GPIO_PD3 // K2 WaikUp/DeepSleep
#define PD3_FUNC					AS_GPIO
#define PD3_INPUT_ENABLE			1
#define PD3_OUTPUT_ENABLE			0
#define	PULL_WAKEUP_SRC_PD3			PM_PIN_PULLUP_1M
//-------------------------- Key USB/BLE
#define KEY_BLE_USB					GPIO_PA1   // mcu wakeup module (Key PWRC) GPIO_WAKEUP_MODULE
#define	PA1_FUNC					AS_GPIO
#define PA1_INPUT_ENABLE			1
#define	PA1_OUTPUT_ENABLE			0
#define	PA1_DATA_OUT				0
#define	PULL_WAKEUP_SRC_PA1			PM_PIN_PULLDOWN_100K
//-------------------------- OutKey Power
/*
#define EXT_POWER_OFF				GPIO_PA0	// =1 Pow.Off
#define PA0_FUNC					AS_GPIO
#define	PA0_OUTPUT_ENABLE			0
#define	PULL_WAKEUP_SRC_PA0			PM_PIN_PULLDOWN_100K
*/
//-------------------------- Out Power 4 mA
#define EXT_POWER_4MA				GPIO_PA0	// =1 Pow.On
#define PA0_FUNC					AS_GPIO
#define PA0_DATA_STRENGTH			0	// Start 0.7 mA
#define	PULL_WAKEUP_SRC_PA0			PM_PIN_PULLDOWN_100K
//-------------------------- ADC
#define	PB1_FUNC					AS_GPIO // ADC-CH06, diff CH20+x
#define	PB4_FUNC					AS_GPIO // ADC-CH09
#define	PB5_FUNC					AS_GPIO	// ADC-CH0A
#define	PB6_FUNC					AS_GPIO // ADC-CH0B
#define	PB7_FUNC					AS_GPIO // ADC-CH0C, CH13=VCC/3, CH14=PB7/3
#define	PC4_FUNC					AS_GPIO // ADC-CH0D/PGA+, CH6D=diff C4-C5
#define	PC5_FUNC					AS_GPIO // ADC-CH0E/PGA-
//-------------------------- DAC
#define	PE0_FUNC					AS_GPIO // SDMN
#define	PE1_FUNC					AS_GPIO // SDMP
//-------------------------- UART
#define UART_TX						GPIO_PC2	//TX
#define PC2_INPUT_ENABLE			1
#define	PC2_OUTPUT_ENABLE			0
#define	PULL_WAKEUP_SRC_PC2			PM_PIN_PULLUP_1M

#define UART_RX						GPIO_PC3	//RX
#define PC3_INPUT_ENABLE			1
#define	PC3_OUTPUT_ENABLE			0
#define	PULL_WAKEUP_SRC_PC3			PM_PIN_PULLUP_1M
//------------------- HX711
#if (USE_HX711)
#define HX711_SCK					GPIO_PC2	//TX
#define HX711_DOUT					GPIO_PC3	//RX
#endif // USE_HX711#elif	(BOARD == BOARD_E104_BT10)
//-------------------
#endif

////////////////////////// AUDIO CONFIG /////////////////////////////
#if (BLE_AUDIO_ENABLE)
	#define BLE_DMIC_ENABLE					0  //0: Amic   1: Dmic

	/////voice packet
	#define	ADPCM_PACKET_LEN				128
	#define TL_MIC_ADPCM_UNIT_SIZE			248  ///sample--2bytes

	#define	TL_MIC_32K_FIR_16K				1

	#if TL_MIC_32K_FIR_16K
		#define	TL_MIC_BUFFER_SIZE				1984
	#else
		#define	TL_MIC_BUFFER_SIZE				992
	#endif


	#if (BLE_DMIC_ENABLE)
		#define GPIO_DMIC_BIAS                  GPIO_PA3////note:if change, the following is changed,too.
		#define PA3_FUNC                        AS_GPIO
		#define DMIC_CLOCK_PIN_RATE             1 //need to be set based on DMIC spec. 1---1.024M;2---2.048M;4---4.096M
	#else
#if	(BOARD_E104_BT10)
		#define GPIO_AMIC_BIAS					GPIO_PC6 ////note:if change, the following is changed,too.
		#define PC6_FUNC                        AS_GPIO
#endif
	#endif

#endif

///////////////////////////////////// ATT  HANDLER define ///////////////////////////////////////
typedef enum
{
	ATT_H_START = 0,

	//// Gap ////
	/**********************************************************************************************/
	GenericAccess_PS_H, 					//UUID: 2800, 	VALUE: uuid 1800
	GenericAccess_DeviceName_CD_H,			//UUID: 2803, 	VALUE:  			Prop: Read | Notify
	GenericAccess_DeviceName_DP_H,			//UUID: 2A00,   VALUE: device name
	GenericAccess_Appearance_CD_H,			//UUID: 2803, 	VALUE:  			Prop: Read
	GenericAccess_Appearance_DP_H,			//UUID: 2A01,	VALUE: appearance
	CONN_PARAM_CD_H,						//UUID: 2803, 	VALUE:  			Prop: Read
	CONN_PARAM_DP_H,						//UUID: 2A04,   VALUE: connParameter

	//// gatt ////
	/**********************************************************************************************/
	GenericAttribute_PS_H,					//UUID: 2800, 	VALUE: uuid 1801
	GenericAttribute_ServiceChanged_CD_H,	//UUID: 2803, 	VALUE:  			Prop: Indicate
	GenericAttribute_ServiceChanged_DP_H,   //UUID:	2A05,	VALUE: service change
	GenericAttribute_ServiceChanged_CCB_H,	//UUID: 2902,	VALUE: serviceChangeCCC

	//// device information ////
	/**********************************************************************************************/
	DeviceInformation_PS_H,					//UUID: 2800, 	VALUE: uuid 180A
	DeviceInformation_pnpID_CD_H,			//UUID: 2803, 	VALUE:  			Prop: Read
	DeviceInformation_pnpID_DP_H,			//UUID: 2A50,	VALUE: PnPtrs

#if (SPP_SERVICE_ENABLE)
	//// TELIK_SPP ////
	/**********************************************************************************************/
	SPP_PS_H, 								//UUID: 2800, 	VALUE: telink audio service uuid

	//Server2Client
	SPP_Server2Client_INPUT_CD_H,			//UUID: 2803, 	VALUE:  			Prop: Read | Notify
	SPP_Server2Client_INPUT_DP_H,			//UUID: TELIK_SPP_Server2Client uuid,  VALUE: SppDataServer2ClientData
	SPP_Server2Client_INPUT_CCB_H,			//UUID: 2902 	VALUE: SppDataServer2ClientDataCCC
	SPP_Server2Client_INPUT_DESC_H,			//UUID: 2901, 	VALUE: TelinkSPPS2CDescriptor
/*
	//Client2Server
	SPP_Client2Server_OUT_CD_H,				//UUID: 2803, 	VALUE:  			Prop: Read | write_without_rsp
	SPP_Client2Server_OUT_DP_H,				//UUID: TELIK_SPP_Client2Server uuid,  VALUE: SppDataClient2ServerData
	SPP_Client2Server_DESC_H,				//UUID: 2901, 	VALUE: TelinkSPPC2SDescriptor
*/
#endif

#if	(BATT_SERVICE_ENABLE)
	//// battery service ////
	/**********************************************************************************************/
	BATT_PS_H, 								//UUID: 2800, 	VALUE: uuid 180f
	BATT_LEVEL_INPUT_CD_H,					//UUID: 2803, 	VALUE:  			Prop: Read | Notify
	BATT_LEVEL_INPUT_DP_H,					//UUID: 2A19 	VALUE: batVal
	BATT_LEVEL_INPUT_CCB_H,					//UUID: 2902, 	VALUE: batValCCC
#endif

#if	(WEIGHT_SERVICE_ENABLE)
	//// battery service ////
	/**********************************************************************************************/
	WEIGHT_PS_H,
	WEIGHT_LEVEL_INPUT_CD_H,
	WEIGHT_LEVEL_INPUT_DP_H,
	WEIGHT_LEVEL_INPUT_CCB_H,
#endif

#if (BLE_AUDIO_ENABLE)
	//// Audio ////
	/**********************************************************************************************/
	AUDIO_PS_H, 							//UUID: 2800, 	VALUE: telink audio service uuid

	//mic
	AUDIO_MIC_INPUT_CD_H,					//UUID: 2803, 	VALUE:  			Prop: Read | Notify
	AUDIO_MIC_INPUT_DP_H,					//UUID: telink mic uuid,  VALUE: micData
	AUDIO_MIC_INPUT_CCB_H,					//UUID: 2A19 	VALUE: micDataCCC
	AUDIO_MIC_INPUT_DESC_H,					//UUID: 2901, 	VALUE: micName

	//speaker
	AUDIO_SPEAKER_OUT_CD_H,					//UUID: 2803, 	VALUE:  			Prop: write_without_rsp
	AUDIO_SPEAKER_OUT_DP_H,					//UUID: telink speaker uuid,  VALUE: speakerData
	AUDIO_SPEAKEROUT_DESC_H,				//UUID: 2901, 	VALUE: speakerName
#endif

	//// Ota ////
	/**********************************************************************************************/
	OTA_PS_H, 								//UUID: 2800, 	VALUE: telink ota service uuid
	OTA_CMD_OUT_CD_H,						//UUID: 2803, 	VALUE:  			Prop: read | write_without_rsp
	OTA_CMD_OUT_DP_H,						//UUID: telink ota uuid,  VALUE: otaData
	OTA_CMD_OUT_DESC_H,						//UUID: 2901, 	VALUE: otaName

	ATT_END_H,

}ATT_HANDLE;

#ifndef CHIP_TYPE
#if(CHIP_TYPE == CHIP_TYPE_8266)
	#define MCU_CORE_TYPE	MCU_CORE_8266
#elif(CHIP_TYPE == CHIP_TYPE_8267)
	#define MCU_CORE_TYPE	MCU_CORE_8267
#elif(CHIP_TYPE == CHIP_TYPE_8261)
	#define MCU_CORE_TYPE	MCU_CORE_8261
#elif(CHIP_TYPE == CHIP_TYPE_8269)
	#define MCU_CORE_TYPE	MCU_CORE_8269
#else
	#define MCU_CORE_TYPE	1000
#endif
#endif
/////////////////// cmd struct   ////////////////
#include "cmd_cfg.h"
/////////////////// set default   ////////////////
#include "vendor/common/default_config.h"

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif // _APP_CFG_H_
