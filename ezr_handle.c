#ifndef __PROTOCOL_DEFS_H__
#define __PROTOCOL_DEFS_H__

#include <stdio.h>
#include "define.h"

#define PROTO_MAGIC  0xA525DEAD

typedef enum _EProtocolCommand
{
    PROTO_CMD_START_FWUPGRADE,
    PROTO_CMD_GET_APP_INFO,
    PROTO_CMD_RESET_TO_APP,
    PROTO_CMD_Count
} EProtocolCommand;

typedef enum _EProtocolStatus
{
    PROTO_STATUS_OK,
    PROTO_STATUS_NACK_INVALID_CRC,
    PROTO_STATUS_NACK_INVALID_PARAM,
    PROTO_STATUS_NACK_UNKNOWN_COMMAND,
    PROTO_STATUS_NACK_NOT_IMPLEMENTED,
} EProtocolStatus;

typedef enum _EAppState
{
    APP_STATE_APPLICATION,
    APP_STATE_BOOTLOADER_READY,
    APP_STATE_BOOTLOADER_BUSY,
    APP_STATE_BOOTLOADER_SERVER_NOT_RESPONDING,
    APP_STATE_BOOTLOADER_DOWNLOAD_FAILED,
    APP_STATE_BOOTLOADER_BROKEN_FW_IMAGE,
    APP_STATE_BOOTLOADER_COPY_FW_IMAGE_FAILED,
    // TBD
} EAppState;

#pragma pack(push,1)

typedef struct _SProtoHeader
{
    uint32_t  magic;        // PROTO_MAGIC
    uint16_t  crc;          // CRC16_CCITT, starting value 0xFFFF
                            // Calculated over following data including payload.
    uint8_t   cmd;          // EProtocolCommand
    uint8_t   status;       // EProtocolStatus (used in responses)
} SProtoHeader;

// PROTO_CMD_START_FWUPGRADE
typedef struct _SProtoStartFwUpgradeRequest
{
    uint32_t  loggerId;
} SProtoStartFwUpgradeRequest;
 
// PROTO_CMD_GET_APP_INFO
typedef struct _SProtoStartGetAppInfoResponse
{
    uint8_t   appState;         // EAppState
    uint32_t  appVersion;       // current application version, or 0xFFFF if no valid application
    uint32_t  app2Version;      // backup application version, or 0xFFFF if no valid backup application
} SProtoStartGetAppInfoResponse; 

// PROTO_CMD_RESET_TO_APP
// Empty request/response
// Generic message
typedef struct _SProtoMessage
{
    SProtoHeader hdr;
    union {
        SProtoStartFwUpgradeRequest     requestFwUpgrade;
        SProtoStartGetAppInfoResponse   responseGetAppInfo;
    };
} SProtoMessage; 

#pragma pack(pop)

SProtoMessage msg2ezr;
// calculates CRC code of the specified buffer
unsigned short CRC16_CCITT(const unsigned char *data, unsigned short length, unsigned short crc)
{
	unsigned char b;

	while (length > 0)
	{
		b = *data;
		crc = (crc >> 8) | (crc << 8);
		crc ^= b;
		crc ^= (crc & 0xff) >> 4;
		crc ^= crc << 12;
		crc ^= (crc & 0xff) << 5;

		++data;
		--length;
	}
	
	return crc;
}

void SendCmnd2Ezr(EProtocolCommand cmd)
{
//    _SProtoHeader ph;
//    ph.magic = unsigned long l;
//    ph.cmd = cmd;
//    ph.payloadCrc = crc();
//    ph.payload = x;
    msg2ezr.hdr.magic = PROTO_MAGIC;
    msg2ezr.hdr.cmd = cmd;
    msg2ezr.hdr.status = 0;
    msg2ezr.hdr.crc = CRC16_CCITT();
    
    Long2Bytes(PROTO_MAGIC, ComBuf);
    ComBuf[4] = cmd;
    int2bytes(crc, &ComBuf[5]);  
    BytesToSend = 7;          
    bWait4WLSensor = TRUE;
    TransmitBuf(1);     
    nTimeCnt = 10;
}

void ParseAnswer(EProtocolCommand cmd)
{
    //check magicnum
    // check CRC
   //buffLen
   
    //RxUart1Buf[7]
    switch (cmd)
    {
    case PROTO_CMD_START_FWUPGRADE:
    break;
    case PROTO_CMD_GET_APP_INFO:
    break;
    case PROTO_CMD_RESET_TO_APP:
    break;
    case PROTO_CMD_Count:
    break;
    }    
}

void EzrConnect(EProtocolCommand cmd)
{
    SendCmnd2Ezr(cmd);
    while ((bCheckRxBuf == FALSE) && (nTimeCnt > 0));
    if (bCheckRxBuf == TRUE)
        ParseAnswer(cmd);
       
    
}
#endif
