/********************************************************************************************************
 * @file     app_att.c
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

#include "proj/tl_common.h"
#include "proj_lib/ble/ll/ll.h"
#include "proj_lib/ble/blt_config.h"
#include "proj_lib/ble/service/ble_ll_ota.h"
#include "proj_lib/ble/att.h"
#include "proj_lib/ble/gap.h"

#if 1

const u16 clientCharacterCfgUUID = GATT_UUID_CLIENT_CHAR_CFG;

const u16 extReportRefUUID = GATT_UUID_EXT_REPORT_REF;

const u16 reportRefUUID = GATT_UUID_REPORT_REF;

const u16 characterPresentFormatUUID = GATT_UUID_CHAR_PRESENT_FORMAT;

const u16 my_primaryServiceUUID = GATT_UUID_PRIMARY_SERVICE;

static const u16 my_characterUUID = GATT_UUID_CHARACTER;

const u16 my_devServiceUUID = SERVICE_UUID_DEVICE_INFORMATION;

const u16 my_PnPUUID = CHARACTERISTIC_UUID_PNP_ID;

const u16 my_devNameUUID = GATT_UUID_DEVICE_NAME;

//device information
const u16 my_gapServiceUUID = SERVICE_UUID_GENERIC_ACCESS;
// Device Name Characteristic Properties
//static u8 my_devNameCharacter = CHAR_PROP_READ | CHAR_PROP_NOTIFY;
// Appearance Characteristic Properties
const u16 my_appearanceUIID = 0x2a01;
const u16 my_periConnParamUUID = 0x2a04;
//static u8 my_appearanceCharacter = CHAR_PROP_READ;
// Peripheral Preferred Connection Parameters Characteristic Properties
//static u8 my_periConnParamChar = CHAR_PROP_READ;
u16 my_appearance = GAP_APPEARE_UNKNOWN;

// gap_periConnectParams_t my_periConnParameters = {DEF_CONN_PARMS}; // 300, 305, 0, 400

const u16 my_gattServiceUUID = SERVICE_UUID_GENERIC_ATTRIBUTE;
const u8  serviceChangedProp = CHAR_PROP_INDICATE;
const u16 serviceChangeUIID = GATT_UUID_SERVICE_CHANGE;
u16 serviceChangeVal[2] = {0};
static u8 serviceChangeCCC[2]={0,0};

extern u8  ble_devName[];

/* Device Name Characteristic Properties
PnP_ID characteris
[0] Vendor ID Source
	=1 USB Implementer’s Forum assigned Vendor ID value
	=2 Bluetooth SIG assigned Company Identifier value from the Assigned Numbers document
[1,2] uint16 Vendor ID, Identifies the product vendor from the namespace in the Vendor ID Source
[3,4] uint16 Product ID, Manufacturer managed identifier for this product
[5,6] uint16 Product Version, Manufacturer managed version for this product */
const u8 my_PnPtrs[] = {0x02, 0x8a, 0x24, 0x66, 0x82, 0x01, 0x00};

//////////////////////// Battery /////////////////////////////////////////////////
const u16 my_batServiceUUID       			= SERVICE_UUID_BATTERY;
const u16 my_batCharUUID       				= CHARACTERISTIC_UUID_BATTERY_LEVEL;
//static
u8 batteryValueInCCC[2];
u8 my_batVal[20]; // 						= {99};

/////////////////////////////////////////////////////////
const u16 userdesc_UUID		= GATT_UUID_CHAR_USER_DESC;

//////////////////////// Weight /////////////////////////////////////////////////
#if	(WEIGHT_SERVICE_ENABLE)

#define CHARACTERISTIC_UUID_Weight					0x2A98
#define CHARACTERISTIC_UUID_Weight_Measurement		0x2A9D
#define CHARACTERISTIC_UUID_Weight_Scale_Feature	0x2A9E
//SERVICE_UUID           "00002a98-0000-1000-8000-00805f9b34fb" // Weight service UUID

const u16 my_WeightServiceUUID       			= SERVICE_UUID_WEIGHT_SCALE;
const u16 my_WeightCharUUID       				= CHARACTERISTIC_UUID_Weight;
//static
u8 WeightValueInCCC[2];
weight_val_t  my_WeightVal;// 						= 0x4E20; // in 0.005 kg (100/0.005)

#endif
////////////////// Audio ///////////////////////////////////////
#if (BLE_AUDIO_ENABLE)
const u8 my_AudioUUID[16]   = {TELINK_AUDIO_UUID_SERVICE};
const u8 my_MicUUID[16]		= {TELINK_MIC_DATA};
const u8 my_SpeakerUUID[16]	= {TELINK_SPEAKER_DATA};

static u8 		micDataCCC[2];
u8		my_MicData 		= 0x80;
u8		my_SpeakerData 	= 0x81;

const u8  my_MicName[] = {'M', 'i', 'c'};
const u8  my_SpeakerName[] = {'S', 'p', 'e', 'a', 'k', 'e', 'r'};
#endif

/////////////////////////////////////////spp/////////////////////////////////////
#if (SPP_SERVICE_ENABLE)
#ifndef SERVICE_UUID_SPP
#define SPP_UUID_SERVICE   			0x10,0x19,0x0d,0x0c,0x0b,0x0a, 0x09,0x08, 0x07,0x06, 0x05,0x04, 0x03,0x02,0x01,0x00		/* TELINK_SPP service */
#define SPP_DATA_SERVER2CLIENT 		0x10,0x2B,0x0d,0x0c,0x0b,0x0a, 0x09,0x08, 0x07,0x06, 0x05,0x04, 0x03,0x02,0x01,0x00 	/* TELINK_SPP data from server to client */
#define SPP_DATA_CLIENT2SERVER 		0x11,0x2B,0x0d,0x0c,0x0b,0x0a, 0x09,0x08, 0x07,0x06, 0x05,0x04, 0x03,0x02,0x01,0x00 	/* TELINK_SPP data from client to server */
//u8 TelinkSppServiceUUID[16]	      = TELINK_SPP_UUID_SERVICE;
//u8 TelinkSppDataServer2ClientUUID[16]    = TELINK_SPP_DATA_SERVER2CLIENT;
//u8 TelinkSppDataClient2ServerUUID[16]    = TELINK_SPP_DATA_CLIENT2SERVER;
#else
//SERVICE_UUID           "0000ffe0-0000-1000-8000-00805f9b34fb" // SSP service UUID
//CHARACTERISTIC_UUID_RX "0000ffe1-0000-1000-8000-00805f9b34fb"
//CHARACTERISTIC_UUID_TX "0000ffe1-0000-1000-8000-00805f9b34fb"
#define SPP_UUID_SERVICE			0xfb,0x34,0x9b,0x5f,0x80,0x00, 0x00,0x80, 0x00,0x10, 0x00,0x00, SERVICE_UUID_SPP&0xFF,SERVICE_UUID_SPP>>8,0x00,0x00
#define SPP_DATA_SERVER2CLIENT		0xfb,0x34,0x9b,0x5f,0x80,0x00, 0x00,0x80, 0x00,0x10, 0x00,0x00, (SERVICE_UUID_SPP&0xFF)+1,SERVICE_UUID_SPP>>8,0x00,0x00 // "Telink SPP: Module->Phone"
#define SPP_DATA_CLIENT2SERVER		0xfb,0x34,0x9b,0x5f,0x80,0x00, 0x00,0x80, 0x00,0x10, 0x00,0x00, (SERVICE_UUID_SPP&0xFF)+5,SERVICE_UUID_SPP>>8,0x00,0x00 // "Telink SPP: Phone->Module"
#endif

u8 TelinkSppServiceUUID[16]				 = {SPP_UUID_SERVICE};
u8 TelinkSppDataServer2ClientUUID[16]    = {SPP_DATA_SERVER2CLIENT}; // "Telink SPP: Module<->Phone"
//u8 TelinkSppDataClient2ServerUUID[16]    = {SPP_DATA_CLIENT2SERVER}; // "Telink SPP: Phone->Module"

//SPP data descriptor
const u8 TelinkSPPS2CDescriptor[] = "SPP: Module<->Phone";
//const u8 TelinkSPPC2SDescriptor[] = "SPP: Phone->Module";

// Spp data from Server to Client characteristic variables
//static u8 SppDataServer2ClientProp = CHAR_PROP_READ | CHAR_PROP_NOTIFY;
//u8 SppDataServer2ClientData[ATT_MTU_SIZE - 3];
//static
u8 SppDataServer2ClientDataCCC[2] = {0};

// Spp data from Client to Server characteristic variables
//CHAR_PROP_WRITE: Need response from slave, low transmission speed
//static u8 SppDataClient2ServerProp = CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP; //CHAR_PROP_WRITE;

//u8  SppDataClient2ServerData[ATT_MTU_SIZE - 3];

/*
 * If the return value is 1, Slave won�t respond with �Read Response/Read Blob Response� to Master.
 * If the return value is not 1, Slave will automatically read �attrLen� bytes of
 * data from the area pointed by the �pAttrValue�, and the data will be
 * responded to Master via �Read Response/Read Blob Response�.
 */
//u8  SppDataServer2ClientDataLen = 0;
int module_onSendData(void *par) {
	(void)par; // rf_packet_att_write_t * p
//	mini_printf("\r\nonSendData\r\n");
//	if (!SppDataServer2ClientDataLen) return 1;
	return 0;
}

extern int module_onReceiveData(void *par);
/*
int module_onReceiveData(void *par)
{
	rf_packet_att_write_t *pp = par;
	u8 len = pp->l2capLen - 3;
	u8 * p = &pp->opcode;
//	mini_printf("\r\nonReceiveData %d\r\n", len);
//	if(len > 2)
//	{
#if 0
		u32 n;
		static u32 sn = 0;
		memcpy (&n, &p->value, 4);
		if (sn != n)
		{
			sn = 0;
			bls_ll_terminateConnection (0x13);
		}
		else
		{
			sn = n + 1;
		}
#endif
		if(len >= 4 && len <= sizeof(ina226_cfg) + 2
			&& p[0] == BLK_IDX_CFG
			&& p[1] <= len
			) {

			memcpy(&blk_rx, &p[2], p[1]);
		}
//		extern int hci_send_data (u32 h, u8 *para, int n);
//		hci_send_data(header, &p->opcode, len + 3);		// HCI_FLAG_EVENT_TLK_MODULE

//	}
	return 0;
}
*/
#endif // #if (SPP_SERVICE_ENABLE)

// Include attribute (Battery service)
//static u16 include[3] = {0x0026, 0x0028, SERVICE_UUID_BATTERY};

#undef TELINK_SPP_DATA_OTA
#define TELINK_SPP_DATA_OTA 				0x12,0x2B,0x0d,0x0c,0x0b,0x0a,0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00
#undef TELINK_OTA_UUID_SERVICE
#define TELINK_OTA_UUID_SERVICE   			0x12,0x19,0x0d,0x0c,0x0b,0x0a,0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00

const u8 my_OtaServiceUUID[16]	= {TELINK_OTA_UUID_SERVICE};
const u8 my_OtaUUID[16]			= {TELINK_SPP_DATA_OTA};

//static u8 my_OtaProp		= CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP;
const u8  my_OtaName[] = {'O', 'T', 'A'};
u8	 	my_OtaData 		= 0x00;

// GAP attribute values
static const u8 my_devNameCharVal[5] = {
	CHAR_PROP_READ | CHAR_PROP_NOTIFY,
	U16_LO(GenericAccess_DeviceName_DP_H), U16_HI(GenericAccess_DeviceName_DP_H),
	U16_LO(GATT_UUID_DEVICE_NAME), U16_HI(GATT_UUID_DEVICE_NAME)
};
static const u8 my_appearanceCharVal[5] = {
	CHAR_PROP_READ,
	U16_LO(GenericAccess_Appearance_DP_H), U16_HI(GenericAccess_Appearance_DP_H),
	U16_LO(GATT_UUID_APPEARANCE), U16_HI(GATT_UUID_APPEARANCE)
};
static const u8 my_periConnParamCharVal[5] = {
	CHAR_PROP_READ,
	U16_LO(CONN_PARAM_DP_H), U16_HI(CONN_PARAM_DP_H),
	U16_LO(GATT_UUID_PERI_CONN_PARAM), U16_HI(GATT_UUID_PERI_CONN_PARAM)
};


// GATT attribute values
static const u8 my_serviceChangeCharVal[5] = {
	CHAR_PROP_INDICATE,
	U16_LO(GenericAttribute_ServiceChanged_DP_H), U16_HI(GenericAttribute_ServiceChanged_DP_H),
	U16_LO(GATT_UUID_SERVICE_CHANGE), U16_HI(GATT_UUID_SERVICE_CHANGE)
};

// device Information  attribute values
static const u8 my_PnCharVal[5] = {
	CHAR_PROP_READ,
	U16_LO(DeviceInformation_pnpID_DP_H), U16_HI(DeviceInformation_pnpID_DP_H),
	U16_LO(CHARACTERISTIC_UUID_PNP_ID), U16_HI(CHARACTERISTIC_UUID_PNP_ID)
};

#if (SPP_SERVICE_ENABLE)
// SPP attribute values
static const u8 my_SPP_S2C_CharVal[19] = {
//	CHAR_PROP_READ | CHAR_PROP_NOTIFY,
	CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP | CHAR_PROP_NOTIFY,
	U16_LO(SPP_Server2Client_INPUT_DP_H), U16_HI(SPP_Server2Client_INPUT_DP_H),
	SPP_DATA_SERVER2CLIENT
};
/*
static const u8 my_SPP_C2S_CharVal[19] = {
	CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP,
	U16_LO(SPP_Client2Server_OUT_DP_H), U16_HI(SPP_Client2Server_OUT_DP_H),
	SPP_DATA_CLIENT2SERVER
};
*/
#endif

#if	(BATT_SERVICE_ENABLE)
//// Battery attribute values
static const u8 my_batCharVal[5] = {
	CHAR_PROP_READ | CHAR_PROP_NOTIFY,
	U16_LO(BATT_LEVEL_INPUT_DP_H), U16_HI(BATT_LEVEL_INPUT_DP_H),
	U16_LO(CHARACTERISTIC_UUID_BATTERY_LEVEL), U16_HI(CHARACTERISTIC_UUID_BATTERY_LEVEL)
};
#endif
#if	(WEIGHT_SERVICE_ENABLE)
//// Weight attribute values
static const u8 my_WeightCharVal[5] = {
	CHAR_PROP_READ | CHAR_PROP_NOTIFY,
	U16_LO(WEIGHT_LEVEL_INPUT_DP_H), U16_HI(WEIGHT_LEVEL_INPUT_DP_H),
	U16_LO(CHARACTERISTIC_UUID_Weight), U16_HI(CHARACTERISTIC_UUID_Weight)
};
#endif
#if (BLE_AUDIO_ENABLE)
//// Audio attribute values
static const u8 my_MicCharVal[19] = {
	CHAR_PROP_READ | CHAR_PROP_NOTIFY,
	U16_LO(AUDIO_MIC_INPUT_DP_H), U16_HI(AUDIO_MIC_INPUT_DP_H),
	TELINK_MIC_DATA,
};
static const u8 my_SpeakerCharVal[19] = {
	CHAR_PROP_WRITE_WITHOUT_RSP,
	U16_LO(AUDIO_SPEAKER_OUT_DP_H), U16_HI(AUDIO_SPEAKER_OUT_DP_H),
	TELINK_SPEAKER_DATA,
};
#endif
// OTA attribute values
static const u8 my_OtaCharVal[19] = {
	CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP,
	U16_LO(OTA_CMD_OUT_DP_H), U16_HI(OTA_CMD_OUT_DP_H),
	TELINK_SPP_DATA_OTA
};


// TM : to modify
const attribute_t my_Attributes[] = {

	{ATT_END_H - 1, 0,0,0,0,0},	// total num of attribute

	// 0001 - 0007  gap
	{7,ATT_PERMISSIONS_READ,2,2,(u8*)(&my_primaryServiceUUID), 	(u8*)(&my_gapServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_devNameCharVal),(u8*)(&my_characterUUID), (u8*)(my_devNameCharVal), 0},
	{0,ATT_PERMISSIONS_READ,2,MAX_DEV_NAME_LEN, (u8*)(&my_devNameUUID), (u8*)(&ble_devName), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_appearanceCharVal),(u8*)(&my_characterUUID), (u8*)(my_appearanceCharVal), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof (my_appearance), (u8*)(&my_appearanceUIID), 	(u8*)(&my_appearance), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_periConnParamCharVal),(u8*)(&my_characterUUID), (u8*)(my_periConnParamCharVal), 0},
//	{0,ATT_PERMISSIONS_READ,2,sizeof (my_periConnParameters),(u8*)(&my_periConnParamUUID), 	(u8*)(&my_periConnParameters), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof (ble_con_ini),(u8*)(&my_periConnParamUUID), 	(u8*)(&ble_con_ini), 0},

	// 0008 - 000b gatt
	{4,ATT_PERMISSIONS_READ,2,2,(u8*)(&my_primaryServiceUUID), 	(u8*)(&my_gattServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_serviceChangeCharVal),(u8*)(&my_characterUUID), (u8*)(my_serviceChangeCharVal), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof (serviceChangeVal), (u8*)(&serviceChangeUIID), 	(u8*)(&serviceChangeVal), 0},
	{0,ATT_PERMISSIONS_RDWR,2,sizeof (serviceChangeCCC),(u8*)(&clientCharacterCfgUUID), (u8*)(serviceChangeCCC), 0},

	// 000c - 000e  device Information Service
	{3,ATT_PERMISSIONS_READ,2,2,(u8*)(&my_primaryServiceUUID), 	(u8*)(&my_devServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_PnCharVal),(u8*)(&my_characterUUID), (u8*)(my_PnCharVal), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_PnPtrs),(u8*)(&my_PnPUUID), (u8*)(my_PnPtrs), 0},


////////////////////////////////////// SPP Service /////////////////////////////////////////////////////
#if (SPP_SERVICE_ENABLE)
	{5,ATT_PERMISSIONS_READ,2,16,(u8*)(&my_primaryServiceUUID), 	(u8*)(&TelinkSppServiceUUID), 0},

	// Server2Client / UART RX/TX
	{0,ATT_PERMISSIONS_READ, 2,sizeof(my_SPP_S2C_CharVal),(u8*)(&my_characterUUID),	(u8*)(my_SPP_S2C_CharVal), 0},				//prop
	{0,ATT_PERMISSIONS_RDWR,16,MTU_RX_DATA_SIZE,(u8*)(&TelinkSppDataServer2ClientUUID), (u8*)&read_pkt, (att_readwrite_callback_t)&module_onReceiveData, 0}, //(att_readwrite_callback_t)&module_onSendData},	//value
	{0,ATT_PERMISSIONS_RDWR, 2,sizeof(SppDataServer2ClientDataCCC),(u8*)(&clientCharacterCfgUUID), 	(u8*)(SppDataServer2ClientDataCCC), 0},	//value
	{0,ATT_PERMISSIONS_READ, 2,sizeof(TelinkSPPS2CDescriptor),(u8*)&userdesc_UUID,(u8*)(&TelinkSPPS2CDescriptor)},
/*
	// Client2Server / UART TX
	{0,ATT_PERMISSIONS_READ, 2,sizeof(my_SPP_C2S_CharVal),(u8*)(&my_characterUUID), (u8*)(my_SPP_C2S_CharVal), 0},				//prop
	{0,ATT_PERMISSIONS_RDWR,16,ATT_MTU_SIZE - 3,(u8*)(&TelinkSppDataClient2ServerUUID), (u8*)&send_pkt, &module_onReceiveData, 0},	//value
	{0,ATT_PERMISSIONS_READ, 2,sizeof(TelinkSPPC2SDescriptor),(u8*)&userdesc_UUID,(u8*)(&TelinkSPPC2SDescriptor)},
*/
#endif

#if	(BATT_SERVICE_ENABLE)
	////////////////////////////////////// Battery Service /////////////////////////////////////////////////////
	{4,ATT_PERMISSIONS_READ,2,2,(u8*)(&my_primaryServiceUUID), 	(u8*)(&my_batServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_batCharVal),(u8*)(&my_characterUUID), (u8*)(my_batCharVal), 0},				//prop
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_batVal),(u8*)(&my_batCharUUID), (u8*)(my_batVal), 0},	//value
	{0,ATT_PERMISSIONS_RDWR,2,sizeof(batteryValueInCCC),(u8*)(&clientCharacterCfgUUID), (u8*)(batteryValueInCCC), 0},	//value
#endif

#if	(WEIGHT_SERVICE_ENABLE)
	////////////////////////////////////// Weight Service /////////////////////////////////////////////////////
	{4,ATT_PERMISSIONS_READ,2,2,(u8*)(&my_primaryServiceUUID), 	(u8*)(&my_WeightServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_WeightCharVal),(u8*)(&my_characterUUID), (u8*)(my_WeightCharVal), 0},				//prop
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_WeightVal),(u8*)(&my_WeightCharUUID), (u8*)(&my_WeightVal), 0},	//value
	{0,ATT_PERMISSIONS_RDWR,2,sizeof(WeightValueInCCC),(u8*)(&clientCharacterCfgUUID), (u8*)(WeightValueInCCC), 0},	//value
#endif

#if (BLE_AUDIO_ENABLE)
	////////////////////////////////////// Audio /////////////////////////////////////////////////////
	{8,ATT_PERMISSIONS_READ,2,16,(u8*)(&my_primaryServiceUUID), 	(u8*)(&my_AudioUUID), 0},

	// MIC
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_MicCharVal),(u8*)(&my_characterUUID), 		(u8*)(my_MicCharVal), 0},				//prop
	{0,ATT_PERMISSIONS_READ,16,sizeof(my_MicData),(u8*)(&my_MicUUID), 	(u8*)(&my_MicData), 0},	//value
	{0,ATT_PERMISSIONS_RDWR,2,sizeof(micDataCCC),(u8*)(&clientCharacterCfgUUID), 	(u8*)(micDataCCC), 0},	//value
	{0,ATT_PERMISSIONS_RDWR,2,sizeof (my_MicName),(u8*)(&userdesc_UUID), (u8*)(my_MicName), 0},

	// SPEAKER
	{0,ATT_PERMISSIONS_READ,2,sizeof(my_SpeakerCharVal),(u8*)(&my_characterUUID), 		(u8*)(my_SpeakerCharVal), 0},
	{0,ATT_PERMISSIONS_WRITE,16,sizeof(my_SpeakerData),(u8*)(&my_SpeakerUUID), 	(u8*)(&my_SpeakerData), 0},//value
	{0,ATT_PERMISSIONS_RDWR,2,sizeof (my_SpeakerName),(u8*)(&userdesc_UUID), (u8*)(my_SpeakerName), 0},
#endif
	// OTA
	{4,ATT_PERMISSIONS_READ, 2,16,(u8*)(&my_primaryServiceUUID), (u8*)(&my_OtaServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ, 2,sizeof(my_OtaCharVal),(u8*)(&my_characterUUID), (u8*)(my_OtaCharVal), 0},				//prop
	{0,ATT_PERMISSIONS_RDWR,16,sizeof(my_OtaData),(u8*)(&my_OtaUUID),	(&my_OtaData), &otaWrite, &otaRead},			//value
	{0,ATT_PERMISSIONS_READ, 2,sizeof(my_OtaName),(u8*)(&userdesc_UUID), (u8*)(my_OtaName), 0},

};

void my_att_init()
{
	bls_att_setAttributeTable ((u8 *)my_Attributes);
	bls_att_setDeviceName((u8 *)ble_dev_name, sizeof(ble_dev_name));
#if (BATT_SERVICE_ENABLE)
	my_batVal[0] = 99;
#endif
}

#endif
