/********************************************************************************************************
 * @file     dma.h
 *
 * @brief    for TLSR chips
 *
 * @author	 BLE Group
 * @date     May. 12, 2018
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

#pragma once

#include "../mcu/register.h"

static inline void dma_init(){
	reg_dma_chn_irq_msk =
			(RF_RX_USE_DMA_IRQ ? FLD_DMA_RF_RX : 0) | (RF_TX_USE_DMA_IRQ ? FLD_DMA_RF_TX : 0);
}

static inline void dma_irq_clear(){
	reg_dma_chn_irq_msk = 0;
}

static inline void dma_channel_enable_all(){
	reg_dma_chn_en = 0xff;
}

static inline void dma_channel_disable_all(){
	reg_dma_chn_en = 0;
}

