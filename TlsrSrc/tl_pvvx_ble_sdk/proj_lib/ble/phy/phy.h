/********************************************************************************************************
 * @file     phy.h
 *
 * @brief    for TLSR chips
 *
 * @author	 public@telink-semi.com;
 * @date     Feb. 1, 2018
 *
 * @par      Copyright (c) Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *			 The information contained herein is confidential and proprietary property of Telink
 * 		     Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *			 of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *			 Co., Ltd. and the licensee in separate contract or the terms described here-in.
 *           This heading MUST NOT be removed from this file.
 *
 * 			 Licensees are granted free, non-transferable use of the information in this
 *			 file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/

#ifndef PHY_H_
#define PHY_H_

#include "../../../proj/mcu/compiler.h"
#include "../ble_common.h"




/********************* Macro & Enumeration variables for Stack, user can not use!!!!  **********************/

#define LL_SLAVE_TX_SETTLE_1M	   		(0x66)
#define LL_SLAVE_TX_STL_2M   	   		(0x58)

#define LL_MASTER_TX_SETTLE_1M          (0x66)
#define LL_MASTER_TX_STL_2M             (0x59)

typedef struct{
	u8	llid;
	u8  rf_len;
	u8	opcode;
	u8	tx_phys;
	u8	rx_phys;
}rf_pkt_ll_phy_req_rsp_t;   //phy_req, phy_rsp


typedef struct{
	u8	llid;
	u8  rf_len;
	u8	opcode;
	u8	m_to_s_phy;
	u8	s_to_m_phy;
	u8 	instant0;
	u8 	instant1;
}rf_pkt_ll_phy_update_ind_t;   //phy_req, phy_rsp




typedef struct {
	u8	dft_tx_prefer_phys;
	u8 	dft_rx_prefer_phys;
	u8	dft_prefer_phy;
	u8	cur_llPhy;
}ll_phy_t;
extern _attribute_aligned_(4) ll_phy_t		bltPHYs;



//do not support Asymmetric PHYs, conn_phys = tx_phys & rx_phys
typedef struct {
	u8	conn_prefer_phys;  // conn_prefer_phys = tx_prefer_phys & rx_prefer_phys
	u8	conn_cur_phy;	   //
	u8	conn_next_phy;	   //
	u8	rsvd0;

	u8	phy_req_trigger;  // 1: means current device triggers phy_req, due to API "blc_ll_setPhy" called by Host or Application
	u8	phy_req_pending;
	u8	phy_update_pending;
	u8	rsvd1;

	u32	conn_update_phy;

}ll_conn_phy_t;

/**
 *  @brief  Return Parameters for "HCI LE Read PHY Command"
 */
typedef struct {
	u8         status;
	u8         handle[2];
	u8         tx_phy;
	u8         rx_phy;
} hci_le_readPhyCmd_retParam_t;

/**
 *  @brief  Command Parameters for "HCI LE Read Set Command"
 */

typedef struct {
	u16 connHandle;
	u8 	all_phys;
	u8 	tx_phys;
	u8 	rx_phys;
} hci_le_setPhyCmd_param_t;



/******************************* Macro & Enumeration variables for User ************************************/

typedef enum {
	BLE_PHY_1M 			= 0x01,
	BLE_PHY_2M 			= 0x02,
} le_phy_type_t;

typedef enum {
	PHY_PREFER_1M 		= BIT(0),
	PHY_PREFER_2M		= BIT(1),
} le_phy_prefer_type_t;



typedef enum {
    PHY_TRX_PREFER		= 0,					//has 	 preference among TX & RX PHYs
	PHY_TX_NO_PREFER 	= BIT(0),   			//has no preference among TX PHYs
	PHY_RX_NO_PREFER 	= BIT(1),				//has no preference among RX PHYs
	PHY_TRX_NO_PREFER 	= (BIT(0) | BIT(1)),	//has no preference among TX & RX PHYs
} le_phy_prefer_mask_t;







/************************************ User Interface  ******************************************************/
void    	blc_ll_init2MPhy_feature(void);


ble_sts_t  	blc_ll_setPhy(	u16 connHandle,	le_phy_prefer_mask_t all_phys, le_phy_prefer_type_t tx_phys, 	le_phy_prefer_type_t rx_phys  );






/*********************************** Stack Interface, user can not use!!! **********************************/
ble_sts_t 	blc_ll_setDefaultPhy(le_phy_prefer_mask_t all_phys, le_phy_prefer_type_t tx_phys, le_phy_prefer_type_t rx_phys);


ble_sts_t	blc_ll_readPhy(u16 connHandle, hci_le_readPhyCmd_retParam_t  *para);

ble_sts_t 	blc_ll_setDefaultPhy(le_phy_prefer_mask_t all_phys, le_phy_prefer_type_t tx_phys, le_phy_prefer_type_t rx_phys);	//set for the device

ble_sts_t   blc_hci_le_setPhy(hci_le_setPhyCmd_param_t* para);

int 		blt_reset_conn_phy_param(ll_conn_phy_t* pconn_phy);


void    	rf_ble_switch_phy (le_phy_type_t  phy_mode);




#endif /* PHY_H_ */
