/********************************************************************************************************
 * @file     ll_conn_phy.h
 *
 * @brief    for TLSR chips
 *
 * @author	 BLE Group
 * @date     July. 4, 2019
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

#ifndef LL_CONN_PHY_H_
#define LL_CONN_PHY_H_

#include "../ble_common.h"
#include "../../../proj/mcu/compiler.h"
#include "../phy/phy.h"

/******************************* Macro & Enumeration variables for Stack, user can not use!!!!  ********************************/
#if (LL_FEATURE_ENABLE_LE_2M_PHY )



extern _attribute_aligned_(4) ll_conn_phy_t   blt_conn_phy;




#endif













/********************************  Macro & Enumeration variables for User *****************************************************/













/********************************  User Interface  ****************************************************************************/













/*******************************  Stack Interface, user can not use!!! *******************************************************/
//2M/Coded PHY
void 		blt_ll_sendPhyReq(void);
int 		blt_ll_updateConnPhy(void);




#endif /* LL_CONN_CSA_H_ */
