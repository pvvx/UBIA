#pragma once

//////////////////////////////////////////////////////////////////////////////
/**
 *  @brief  Definition for Device info
 */
#include "../../proj/mcu/analog.h"
#include "../rf_drv.h"
#include "../../proj/tl_common.h"

#define  MAX_DEV_NAME_LEN 				18

#ifndef DEV_NAME
#define DEV_NAME                        "tModule"
#endif

#define RAMCODE_OPTIMIZE_CONN_POWER_NEGLECT_ENABLE			0

/////////////////// Flash  Address Config ////////////////////////////
#if( __TL_LIB_8261__ || (MCU_CORE_TYPE == MCU_CORE_8261) )
	#ifndef		CFG_ADR_MAC
	#define		CFG_ADR_MAC						0x1F000
	#endif

	#ifndef		CUST_CAP_INFO_ADDR
	#define		CUST_CAP_INFO_ADDR				0x1E000
	#endif

	#ifndef		CUST_TP_INFO_ADDR
	#define		CUST_TP_INFO_ADDR				0x1E040
	#endif

#else  //8266 8267 8269
	#ifndef		CFG_ADR_MAC
	#define		CFG_ADR_MAC						0x76000
	#endif

	#ifndef		CUST_CAP_INFO_ADDR
	#define		CUST_CAP_INFO_ADDR				0x77000
	#endif

	#ifndef		CUST_TP_INFO_ADDR
	#define		CUST_TP_INFO_ADDR				0x77040
	#endif
#endif

typedef struct{
	u8 conn_mark;
	u8 ext_cap_en;
	u8 pad32k_en;
	u8 pm_enter_en;
}misc_para_t;

misc_para_t blt_miscParam;



static inline void blc_app_setExternalCrystalCapEnable(u8  en)
{
	blt_miscParam.ext_cap_en = en;
}



static inline void blc_app_loadCustomizedParameters(void)
{
	 if(!blt_miscParam.ext_cap_en)
	 {
		 //customize freq_offset adjust cap value, if not customized, default ana_81 is 0xd0
		 if( (*(unsigned char*) CUST_CAP_INFO_ADDR) != 0xff ){
			 //ana_81<4:0> is cap value(0x00 - 0x1f)
			 analog_write(0x81, (analog_read(0x81)&0xe0) | ((*(unsigned char*) CUST_CAP_INFO_ADDR)&0x1f) );
		 }
	 }


	 // customize TP0/TP1 for 1M
	 if( ((*(unsigned char*) (CUST_TP_INFO_ADDR)) != 0xff) && ((*(unsigned char*) (CUST_TP_INFO_ADDR+1)) != 0xff) ){
		 rf_update_tp_value(*(unsigned char*) (CUST_TP_INFO_ADDR), *(unsigned char*) (CUST_TP_INFO_ADDR+1));
	 }

	 ///2M mode just for 8269. 8267 for BQB 2M
	#if (__TL_LIB_8269__ || MCU_CORE_TYPE == MCU_CORE_8269 || __TL_LIB_8267__ ||  MCU_CORE_TYPE == MCU_CORE_8267)
		// customize TP0/TP1 for 2M
		if( ((*(unsigned char*) (CUST_TP_INFO_ADDR+2)) != 0xff) && ((*(unsigned char*) (CUST_TP_INFO_ADDR+3)) != 0xff) ){
		 rf_update_2m_tp_value(*(unsigned char*) (CUST_TP_INFO_ADDR+2), *(unsigned char*) (CUST_TP_INFO_ADDR+3));
		}
	#endif

}


#define	TELINK2C0_LIBRARY_ENABLE							0


#if(NANOSIC_LIBRARY_ENABLE)
	#define	BLS_PROC_MASTER_UPDATE_REQ_IN_IRQ_ENABLE	0
	#define FIX_HW_CRC24_EN                             1
#endif


#define	TELINK2C1_LIBRARY_ENABLE						    0


#if(OMNI_LIBRARY_ENABLE)
	#define	BLS_PROC_MASTER_UPDATE_REQ_IN_IRQ_ENABLE		0
	#define	BLS_BLE_RF_IRQ_TIMING_EXTREMELY_SHORT_EN		1
	#define	LE_AUTHENTICATED_PAYLOAD_TIMEOUT_SUPPORT_EN		1
#endif

#if(__TL_LIB_8266__ || MCU_CORE_TYPE == MCU_CORE_8266)
	#define LL_FEATURE_SUPPORT_LE_2M_PHY					0
	#define LL_SN_NESN_MANAGE_BY_SOFTWARE					0
#endif

#if(__TL_LIB_8261__ || MCU_CORE_TYPE == MCU_CORE_8261)
	#define SECURE_CONNECTION_ENABLE        				0
	#define BLE_CORE42_DATA_LENGTH_EXTENSION_ENABLE			0
	#define LL_FEATURE_SUPPORT_LE_2M_PHY					0
#endif

#ifndef SECURE_CONNECTION_ENABLE
#define SECURE_CONNECTION_ENABLE        				1
#endif

#ifndef BLE_CORE42_DATA_LENGTH_EXTENSION_ENABLE
#define BLE_CORE42_DATA_LENGTH_EXTENSION_ENABLE			1
#endif

#if(BLE_CORE42_DATA_LENGTH_EXTENSION_ENABLE)
#define DLE_LEN_GE100                                   1 //data length greater or equal 100 bytes. for windows size.
#endif


#ifndef LL_FEATURE_SUPPORT_LE_2M_PHY
#define LL_FEATURE_SUPPORT_LE_2M_PHY					1
#endif


#ifndef	LL_SN_NESN_MANAGE_BY_SOFTWARE
#define LL_SN_NESN_MANAGE_BY_SOFTWARE					1
#endif


//default ll_master_multi connection
#ifndef  LL_MASTER_SINGLE_CONNECTION
#define  LL_MASTER_SINGLE_CONNECTION					0
#endif

#ifndef  LL_MASTER_MULTI_CONNECTION
#define  LL_MASTER_MULTI_CONNECTION						0
#endif

#if (LL_MASTER_SINGLE_CONNECTION)
	#define	LL_CHNMAP_ADAPTIVE_FREQ_HOPPING_EN				0
#endif


#if (BLE_MODULE_LIB_ENABLE || BLE_MODULE_APPLICATION_ENABLE)  //for ble module
	#define		BLS_DMA_DATA_LOSS_DETECT_AND_SOLVE_ENABLE	1
	#define		BLS_SEND_TLK_MODULE_EVENT_ENABLE			1
#endif

//when rf dma & uart dma work together
#ifndef		BLS_DMA_DATA_LOSS_DETECT_AND_SOLVE_ENABLE
#define		BLS_DMA_DATA_LOSS_DETECT_AND_SOLVE_ENABLE		0
#endif

#ifndef		BLS_SEND_TLK_MODULE_EVENT_ENABLE
#define 	BLS_SEND_TLK_MODULE_EVENT_ENABLE				0
#endif



#ifndef		BLS_ADV_INTERVAL_CHECK_ENABLE
#define		BLS_ADV_INTERVAL_CHECK_ENABLE					0
#endif

#if LIB_TELINK_MESH_SCAN_MODE_ENABLE
#define		BLS_TELINK_MESH_SCAN_MODE_ENABLE				1
#endif

/////////////////  scan mode config  //////////////////////////
#ifndef		BLS_TELINK_MESH_SCAN_MODE_ENABLE
#define		BLS_TELINK_MESH_SCAN_MODE_ENABLE				0
#endif


#ifndef	BLS_BLE_RF_IRQ_TIMING_EXTREMELY_SHORT_EN
#define	BLS_BLE_RF_IRQ_TIMING_EXTREMELY_SHORT_EN		0
#endif


//conn param update/map update
#ifndef	BLS_PROC_MASTER_UPDATE_REQ_IN_IRQ_ENABLE
#define BLS_PROC_MASTER_UPDATE_REQ_IN_IRQ_ENABLE		1
#endif


#ifndef LE_AUTHENTICATED_PAYLOAD_TIMEOUT_SUPPORT_EN
#define LE_AUTHENTICATED_PAYLOAD_TIMEOUT_SUPPORT_EN		0
#endif

/////////////////////HCI UART variables///////////////////////////////////////
#define UART_DATA_LEN    64      // data max 252
typedef struct{
    unsigned int len;        // data max 252
    unsigned char data[UART_DATA_LEN];
}uart_data_t;


#define ATT_RSP_BIG_MTU_PROCESS_EN          1


///all fix code need be control by macro. Here is those macro.
/*
 * ��BLEģʽ���հ�ʱ��CRCУ��errorָʾλ���ڴ��еĿ��ܣ�ԭ���������ж���CRCУ����
 * 24-bit�еĸ�8λ������16-bitû�д��Ҹ�8-bit�д�ʱ���������У�����С��1/(2^16).
 * if enable soft crc24, must open the macro "LL_SN_NESN_MANAGE_BY_SOFTWARE" in ll_slave.c
 */
#ifndef FIX_HW_CRC24_EN
#define	FIX_HW_CRC24_EN						1
#endif


#define PA_IN_STACK_ENABLE					0

///////////////////////////////////////dbg channels///////////////////////////////////////////
#ifndef	DBG_CHN0_TOGGLE
#define DBG_CHN0_TOGGLE
#endif

#ifndef	DBG_CHN0_HIGH
#define DBG_CHN0_HIGH
#endif

#ifndef	DBG_CHN0_LOW
#define DBG_CHN0_LOW
#endif

#ifndef	DBG_CHN1_TOGGLE
#define DBG_CHN1_TOGGLE
#endif

#ifndef	DBG_CHN1_HIGH
#define DBG_CHN1_HIGH
#endif

#ifndef	DBG_CHN1_LOW
#define DBG_CHN1_LOW
#endif

#ifndef	DBG_CHN2_TOGGLE
#define DBG_CHN2_TOGGLE
#endif

#ifndef	DBG_CHN2_HIGH
#define DBG_CHN2_HIGH
#endif

#ifndef	DBG_CHN2_LOW
#define DBG_CHN2_LOW
#endif

#ifndef	DBG_CHN3_TOGGLE
#define DBG_CHN3_TOGGLE
#endif

#ifndef	DBG_CHN3_HIGH
#define DBG_CHN3_HIGH
#endif

#ifndef	DBG_CHN3_LOW
#define DBG_CHN3_LOW
#endif

#ifndef	DBG_CHN4_TOGGLE
#define DBG_CHN4_TOGGLE
#endif

#ifndef	DBG_CHN4_HIGH
#define DBG_CHN4_HIGH
#endif

#ifndef	DBG_CHN4_LOW
#define DBG_CHN4_LOW
#endif

#ifndef	DBG_CHN5_TOGGLE
#define DBG_CHN5_TOGGLE
#endif

#ifndef	DBG_CHN5_HIGH
#define DBG_CHN5_HIGH
#endif

#ifndef	DBG_CHN5_LOW
#define DBG_CHN5_LOW
#endif

#ifndef	DBG_CHN6_TOGGLE
#define DBG_CHN6_TOGGLE
#endif

#ifndef	DBG_CHN6_HIGH
#define DBG_CHN6_HIGH
#endif

#ifndef	DBG_CHN6_LOW
#define DBG_CHN6_LOW
#endif

#ifndef	DBG_CHN7_TOGGLE
#define DBG_CHN7_TOGGLE
#endif

#ifndef	DBG_CHN7_HIGH
#define DBG_CHN7_HIGH
#endif

#ifndef	DBG_CHN7_LOW
#define DBG_CHN7_LOW
#endif


