/********************************************************************************************************
 * @file     common_emi.c
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
#if 0 //pvvx
 
#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "common_emi.h"

extern void rf_power_enable(int );

//undefine RF_CHN_TABLE
#if 0
#undef RF_CHN_TABLE
#define RF_CHN_TABLE 0
#endif

#if 	(__PROJECT_VACUUM__ || __PROJECT_VACUUM_RECEIVER__)
const u8	tbl_test_mode_channel[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}; //chn_0 to chn_15
const u8	tbl_test_mode_sel[3] = {host_cmd_carrier, host_cmd_cd, host_cmd_rx};

#else
const u8	tbl_test_mode_channel[4] = {host_cmd_chn_l, host_cmd_chn_m, host_cmd_chn_h};
const u8	tbl_test_mode_sel[4] = {host_cmd_carrier, host_cmd_cd, host_cmd_rx, host_cmd_tx};
#endif

#define FIFO_DEPTH			(1*16)
#define STATE0				0x1234
#define STATE1				0x5678
#define STATE2				0xabcd
#define STATE3				0xef01

typedef struct{
    u32 start;
#if (FIFO_DEPTH > 0x08)
    u32 data[(FIFO_DEPTH - 8)>>2];
#endif
    u32 end;
    u32 cd_feed[2];
} fifo_emi_t;

#if 0
#if	(MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8266)
#define		PKT_BUFF_SIZE		256
#else
#define		PKT_BUFF_SIZE		80
#endif
#endif
extern unsigned char rf_rx_buff[];
fifo_emi_t *fifo_emi = (fifo_emi_t *)rf_rx_buff;          //re-use rf_rx_buff, PKT_BUFF_SIZE*2 must larger than sizeof (fifo_emi_t)
void (*pf_emi_rx_init) ( void ) = NULL;
void (*pf_emi_rx_recover) ( void ) = NULL;
void (*pf_emi_cust_init) ( void ) = NULL;

int m_pnGen(int state)
{
	int feed = 0;
	feed = (state&0x4000) >> 1;
	state ^= feed;
	state <<= 1;
	state = (state&0xfffe) + ((state&0x8000)>>15);
	return state;
}
void emi_cd_data_gen( void ){
    fifo_emi->end = (fifo_emi->cd_feed[0]<<16) + fifo_emi->cd_feed[1];
    //advance PN generator
    fifo_emi->cd_feed[0] = m_pnGen(fifo_emi->cd_feed[0]);
    fifo_emi->cd_feed[1] = m_pnGen(fifo_emi->cd_feed[1]);
}

static inline void emi_fifo_init( void ){
    fifo_emi->start = FIFO_DEPTH + 1;
    fifo_emi->end = STATE3;
    fifo_emi->cd_feed[0] = STATE0;
    fifo_emi->cd_feed[1] = STATE1;
}

#if(0)
void emi_process (u8 tx_power ) {
    static u8   flg_emi_init = 0;
    if( !flg_emi_init ){
        if ( pf_emi_cust_init != NULL )
           (*pf_emi_cust_init) ();
        emi_init( tx_power );
        flg_emi_init = 1;
        SetTxMode ( tbl_test_mode_channel[1], RF_CHN_TABLE | RF_SET_TX_MANAUL ); //rx mode may fail dongle cd
    }
    static u8 test_mode, test_mode_chn;
    test_mode = tbl_test_mode_sel[1];
    test_mode_chn = tbl_test_mode_channel[1];


    if (1) {  //test mode or channel changed
        if (TEST_MODE_CD) {      //carry + data
            emi_fifo_init();
            emi_cd_prepare();
            WaitUs(1000);                       //must delay
            //SetTxMode (test_mode_chn, RF_CHN_TABLE | RF_SET_TX_MANAUL);
            rf_set_channel (test_mode_chn, RF_CHN_TABLE );

    		write_reg8  (0x800f00, 0x80);				// stop
    		write_reg8  (0x800f14, 0 );			// number of retry

    		write_reg16 (0x800f0a, 350);


            //sleep_us(200);
            emi_cd_init ( &fifo_emi->start );
        	write_reg16 (0x800f00, 0x3f83);

        }
        else{
            emi_cd_recovery();
        }
    }

    static u32 emi_tick;
    while (!clock_time_exceed (emi_tick, 800000)) {
        if (TEST_MODE_CD) {
            emi_cd_data_gen ();
        }
    }
    emi_tick = clock_time ();
}
#else
void emi_process (u8 cmd, u8 chn_idx, u8 test_mode_sel, u8 *tx_pkt, u8 tx_power ) {
    static u8   flg_emi_init = 0;
    if( !flg_emi_init ){
        if ( pf_emi_cust_init != NULL )
           (*pf_emi_cust_init) ();
        emi_init( tx_power );
        flg_emi_init = 1;
        SetTxMode ( tbl_test_mode_channel[0], RF_CHN_TABLE | RF_SET_TX_MANAUL ); //rx mode may fail dongle cd
    }
    
    static u8 test_mode, test_mode_chn;
    if (cmd) {  //test mode or channel changed

        test_mode = tbl_test_mode_sel[test_mode_sel];
#if RF_CHN_TABLE        
        test_mode_chn = tbl_test_mode_channel[chn_idx];
#else
        test_mode_chn = chn_idx;
#endif
        rf_power_enable(1);      
        if (TEST_MODE_CARRIER) { //carry                
            emi_carrier_init ();
            //rf_set_channel (test_mode_chn, RF_CHN_TABLE | RF_SET_TX_MANAUL );
            rf_set_channel (test_mode_chn, RF_CHN_TABLE );
            rf_set_txmode();
            sleep_us(200);
            emi_carrier_generate();                  
        }
        else{
            emi_carrier_recovery();
        }

        if (TEST_MODE_CD) {      //carry + data
            emi_fifo_init();
            emi_cd_prepare();
            WaitUs(1000);                       //must delay
            //SetTxMode (test_mode_chn, RF_CHN_TABLE | RF_SET_TX_MANAUL);
            rf_set_channel (test_mode_chn, RF_CHN_TABLE );
            rf_set_txmode();
            sleep_us(200);
            emi_cd_init ( (u32)&fifo_emi->start );
        }
        else{
            emi_cd_recovery();
        }

        if (TEST_MODE_RX) {     //rx
            if ( pf_emi_rx_init != NULL )
                (*pf_emi_rx_init) ();
            SetRxMode (test_mode_chn, RF_CHN_TABLE);            
        }
        else{
            if ( pf_emi_rx_recover != NULL )
                (*pf_emi_rx_recover) ();
        }

        if (TEST_MODE_TX) { //tx
            SetTxMode (test_mode_chn, RF_CHN_TABLE);
        }
    }

    if (TEST_MODE_TX) {

        rf_power_enable(1);
        sleep_us(1000);
        rf_set_channel (test_mode_chn, RF_CHN_TABLE );
        rf_send_single_packet(tx_pkt);     
        sleep_us(500);

       rf_power_enable(0);
    }
    
    static u32 emi_tick;
    while (!clock_time_exceed (emi_tick, 8000)) {
        if (TEST_MODE_CD) {
            emi_cd_data_gen ();
        }
        if (TEST_MODE_RX) {
            //rf_recv_pkt ();
        }
    }
    emi_tick = clock_time ();
}
#endif

#endif // pvvx
