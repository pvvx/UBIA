/*
 * ll_slave.h
 *
 *  Created on: 2017-3-9
 *      Author: Administrator
 */

#ifndef LL_SLAVE_H_
#define LL_SLAVE_H_



typedef struct {
	u8		time_update_st;
	u8 		last_rf_len;
	u8		remoteFeatureReq;
	u8		interval_level;

	u8		ll_recentAvgRSSI;
	u8		conn_sn_master;
	u8		conn_update;
	u8		master_not_ack_slaveAckUpReq;

	u8		conn_new_param;
	u8		conn_winsize_next;
	u8 		conn_master_terminate;
	u8		conn_terminate_reason;

	u8 		conn_slave_terminate;
	u8		conn_terminate_pending;   // terminate_pending = master_terminate || slave_terminate
	u16 	rsvd1;


	u16		connHandle;
	u16		conn_inst;
	u16		conn_latency;
	u16     conn_offset_next;
	u16		conn_inst_next;
	u16		conn_interval_next; //standard value,  not * 1.25ms
	u16		conn_latency_next;
	u16		conn_timeout_next;  //standard value,  not *10ms


	u32		conn_access_code_revert;
	u32		conn_crc;
	u32		ll_remoteFeature; //not only one for BLE master, use connHandle to identify
	u32		connExpectTime;
	int		conn_interval_adjust;
	u32		conn_timeout;
	u32		conn_tick;
	u32		conn_interval;
	u32		conn_duration;

	u8		conn_chn;
	u8		conn_chn_hop;
	u8		conn_chn_map[5];
	u8		conn_chn_map_next[5];


	u32		conn_start_tick;
	int		conn_tolerance_time;
	u32		tick_1st_rx;
	u32		conn_brx_tick;
	u32 	conn_slaveTerminate_time;

#if (BLS_PROC_MASTER_UPDATE_REQ_IN_IRQ_ENABLE)
	u32		conn_pkt_rcvd;
	u32		conn_pkt_rcvd_no;
	u8 *	conn_pkt_dec_pending;
	int		conn_enc_dec_busy;
	int		conn_stop_brx;
#endif
} st_ll_conn_slave_t;


extern _attribute_aligned_(4) ble_crypt_para_t 	blc_cyrpt_para;


#if (NANOSIC_LIBRARY_ENABLE)
	typedef int (*ll_update_cmd_overtime_callback_t)(int);

	void blc_ll_register_ll_updateCmd_overT_cb(ll_update_cmd_overtime_callback_t cb);
#endif

/******************************* User Interface  ************************************/
void 		blc_ll_initSlaveRole_module(void);

ble_sts_t  	bls_ll_terminateConnection (u8 reason);

bool		bls_ll_isConnectState (void);

u16			bls_ll_getConnectionInterval(void);  // if return 0, means not in connection state
u16			bls_ll_getConnectionLatency(void);	 // if return 0, means not in connection state
u16			bls_ll_getConnectionTimeout(void);	 // if return 0, means not in connection state


int			bls_ll_requestConnBrxEventDisable(void);
void		bls_ll_disableConnBrxEvent(void);
void		bls_ll_restoreConnBrxEvent(void);





//ble module event
ble_sts_t 	bls_hci_mod_setEventMask_cmd(u32 evtMask);  //eventMask: module special




/************************* Stack Interface, user can not use!!! ***************************/
bool		bls_ll_pushTxFifo (int hanlde, u8 *p);
ble_sts_t  	bls_hci_reset(void);

ble_sts_t	bls_hci_receiveHostACLData(u16 connHandle, u8 PB_Flag, u8 BC_Flag, u8 *pData );

void	    blt_push_fifo_hold (u8 *p);



#endif /* LL_SLAVE_H_ */
