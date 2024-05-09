#ifndef __PROTOCOL_DEFS_H__
#define __PROTOCOL_DEFS_H__

#include <stdint.h>

#define PROTO_MAGIC  0xA525DEAD
//#define PROTO_MAGIC  0xADDE25A5

typedef enum _EProtocolCommand
{
	PROTO_CMD_START_FWUPGRADE,
	PROTO_CMD_GET_APP_INFO,
	PROTO_CMD_RESET_TO_APP,
	PROTO_CMD_START_FWUPGRADE_EXT,     
    PROTO_CMD_PREPARE_TO_FWUPGRADE,
    PROTO_CMD_GET_DATA,   
	PROTO_CMD_VALVE_COMMANDS,  
    PROTO_CMD_VALVE_LIST,
	PROTO_CMD_Count
} EProtocolCommand;

typedef enum _EProtocolStatus
{
	PROTO_STATUS_OK,
	PROTO_STATUS_NACK_INVALID_CRC,
	PROTO_STATUS_NACK_INVALID_PARAM,
	PROTO_STATUS_NACK_UNKNOWN_COMMAND,
	PROTO_STATUS_NACK_NOT_IMPLEMENTED,    
    PROTO_STATUS_DO_NOT_ANSWER_SENSOR,    
    PROTO_STATUS_NO_LIST,
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

//#pragma pack(push,1)

typedef struct _SProtoHeader
{
	unsigned long  magic;		// PROTO_MAGIC
	unsigned int  crc;			// CRC16_CCITT, starting value 0xFFFF
							// Calculated over following data including payload.
	unsigned char   cmd;			// EProtocolCommand
	unsigned char   status;		// EProtocolStatus (used in responses)
} SProtoHeader;

// PROTO_CMD_START_FWUPGRADE
typedef struct _SProtoGetData
{
	char    loggerId[4];  
    unsigned char    vlvCnt;
} SProtoGetData;

// PROTO_CMD_START_FWUPGRADE
typedef struct _SProtoStartFwUpgradeExtRequest
{
	char    loggerId[4];      
    unsigned char    url[64];
    unsigned int    fw2upg;
} SProtoStartFwUpgradeExtRequest;

// PROTO_CMD_GET_APP_INFO
typedef struct _SProtoStartGetAppInfoResponse
{
	unsigned char   appState;			// EAppState
	unsigned long  appVersion;		// current application version, or 0xFFFF if no valid application
	unsigned long  app2Version;		// backup application version, or 0xFFFF if no valid backup application
} SProtoStartGetAppInfoResponse;

//PROTO_CMD_VALVE_COMMANDS
typedef struct _SProtoSendValveCtrlCmd
{
//    uint8_t 	    nCurHour;
    ValveCmd2EZR    cmdArr[5];       //
} SProtoSendValveCtrlCmd;

// PROTO_CMD_RESET_TO_APP
// Empty request/response

 
// Generic message
typedef struct _SProtoMessage
{
	SProtoHeader hdr;
	union {
		SProtoGetData		requestData;
		SProtoStartGetAppInfoResponse	responseGetAppInfo;
        SProtoStartFwUpgradeExtRequest  requestFwUpgradeExt; 
        SProtoSendValveCtrlCmd          SendVlvCmd;        
        unsigned long                   vlvList[MAX_CMD];
//        unsigned int                    startTime;
	};
} SProtoMessage;

//#pragma pack(pop)

#endif