//#include "define.h"
#include "utils.h"
#include "protocol.h"
#include "protocolDefs.h"
#include "crc.h"
#include "Valve_Manager.h"
#include "interrupts.h"
#include "HW_manager.h"

static SProtoMessage message;
BYTE ezrState;
extern long ezrVersion;
extern eeprom _tagAPPEEPROM AppEepromData;
extern flash unsigned char fEZRUpdateAddress[];
extern int nTimeCnt;
extern bit bWait4WLSensor;
extern char ComBuf[MAX_RX1_BUF_LEN];
extern int BytesToSend;
//extern unsigned int buffLen;
extern unsigned int nEzrFw2Upg;
//extern unsigned int g_sec2HndlPump;

void PROTOCOL_Init(void)
{    
    // open UART1               
    ENABLE_UART1(); 
    // reset buffer
    ResetUart1();
    // selct UART mode
    UART1Select(UART_RADIO_UHF);     
    delay_ms(50);   //(100);    
    // switch CTS
    WIRELESS_CTS_ON();   
    delay_ms(50);   //(100); 
    WIRELESS_CTS_OFF(); 
}

void PROTOCOL_DeInit(void)
{
    // switch CTS
    //WIRELESS_CTS_OFF();
    // close UART1               
    //DISABLE_UART1(); 
}

// Get response payload size by command code
static int GetResponseSize(EProtocolCommand cmd)
{
	switch (cmd)
	{
//	case PROTO_CMD_START_FWUPGRADE:
//		return 0;
		
	case PROTO_CMD_GET_APP_INFO:
		return sizeof(SProtoStartGetAppInfoResponse);
		
	case PROTO_CMD_RESET_TO_APP:
		return 0;
		
    case PROTO_CMD_VALVE_COMMANDS:
        return 0;     
    case PROTO_CMD_VALVE_LIST:
        return 0;
	default:
		return 0;
	}
}

// Get request payload size by command code
static int GetRequestSize(EProtocolCommand cmd)
{
	switch (cmd)
	{
//	case PROTO_CMD_START_FWUPGRADE:
    case PROTO_CMD_GET_DATA:
		return sizeof(SProtoGetData);   
        
	case PROTO_CMD_START_FWUPGRADE_EXT:
		return sizeof(SProtoStartFwUpgradeExtRequest);
		
	case PROTO_CMD_GET_APP_INFO:
		return 0;
		
	case PROTO_CMD_RESET_TO_APP:    
        return 0;
        
    case PROTO_CMD_VALVE_LIST:
		return (MAX_CMD * sizeof(unsigned long));
		        
    case PROTO_CMD_VALVE_COMMANDS:
        return sizeof(SProtoSendValveCtrlCmd);
	default:
		return 0;
	}
}

static uint16_t CalcMessageCrc(BYTE payloadSize)
{
	return CRC16_CCITT(&message.hdr.cmd, sizeof(message.hdr) - sizeof(message.hdr.magic) - sizeof(message.hdr.crc) + payloadSize, 0xFFFF);
}

static void SendRequest(void)
{
	BYTE payloadSize = GetRequestSize((EProtocolCommand)message.hdr.cmd);
	BYTE messageSize = sizeof(message.hdr) + payloadSize; 
    BYTE i;
    
	message.hdr.crc = CalcMessageCrc(payloadSize);

    for (i = 0; i < messageSize; i++)
        ComBuf[i] = ((const uint8_t *) &message) [i];   
    BytesToSend = messageSize;
    TransmitBuf(1);    
}

BYTE HandleResponse()
{
    BYTE i;  
    unsigned int  crc;  
    //copyy buffer into structure
    for (i = 0; i < buffLen; i++)     
    {                                               
        ((uint8_t *) &message) [i] = RxUart1Buf[i];    
    }          
    if (message.hdr.magic != PROTO_MAGIC)  
    {                      
        return FALSE;
    }               
    
    // calc crc    
	crc = CalcMessageCrc(GetResponseSize(message.hdr.cmd));
	if (message.hdr.crc != crc)
	{		
		return FALSE;
	}
	     
    if (message.hdr.status == PROTO_STATUS_OK)  
    {
        if (message.hdr.cmd == PROTO_CMD_GET_APP_INFO)    
        {
            //ezrState = RxUart1Buf[sizeof(message.hdr)];
		    ezrState = message.responseGetAppInfo.appState;
            ezrVersion = message.responseGetAppInfo.appVersion;
        }    
        return TRUE;      
    }        
    if (message.hdr.status == PROTO_STATUS_NO_LIST)
        g_bVlvListUpdated = true;
    return FALSE;
}

BYTE PROTOCOL_Task(EProtocolCommand cmd, BYTE bIsMore)
{
    BYTE len,i, n = 0;    
                                    
    memset(&message, 0, sizeof(SProtoMessage));
	message.hdr.magic = PROTO_MAGIC;         
    message.hdr.cmd = cmd;   
     
    if (/*(cmd == PROTO_CMD_START_FWUPGRADE) ||*/ (cmd == PROTO_CMD_GET_DATA))         
    {
        cpu_e2_to_MemCopy((BYTE*)message.requestData.loggerId, AppEepromData.eLoggerID, 4);   
        message.requestData.vlvCnt = 0;//GetVlvCnt();
    }        
    if (cmd == PROTO_CMD_START_FWUPGRADE_EXT)
    {
        cpu_e2_to_MemCopy((BYTE*)message.requestFwUpgradeExt.loggerId, AppEepromData.eLoggerID, 4); 
        //copy site   
        len = CopyFlashToBuf(message.requestFwUpgradeExt.url, fEZRUpdateAddress);
        message.requestFwUpgradeExt.url[len] = '\0';    
        message.requestFwUpgradeExt.fw2upg = nEzrFw2Upg;              
    }        
#ifndef NO_EZR
    if (cmd == PROTO_CMD_VALVE_COMMANDS)
    {       
//        message.SendVlvCmd.nCurHour = bIsMore;//readClockBuf[4];
        for (i = 0; i < 5; i++)                 
        {              
            message.SendVlvCmd.cmdArr[i] = GetNextValveCmd(); 
            if (message.SendVlvCmd.cmdArr[i].m_ID > 0)
                n++;  
            if (g_lGlobalCmd > 0)
                break;
        }           
        if (n == 0)
            return FALSE;
    }          
    if (cmd == PROTO_CMD_VALVE_LIST)
    {                    
        for (i = 0; i < MAX_CMD; i++) 
            message.vlvList[i] = vlvCmdArr[i].VCU_ID;
    }
#endif NO_EZR
    message.hdr.status = PROTO_STATUS_OK;//EzrShouldAnswer()? PROTO_STATUS_OK : PROTO_STATUS_DO_NOT_ANSWER_SENSOR;   
    bWait4WLSensor = TRUE;                       
    bCheckRx1Buf = FALSE;        
    rx1_buff_len = 0;
    SendRequest();       
    nTimeCnt = 15;
    while ((nTimeCnt > 0) && (bCheckRx1Buf == FALSE));
    if (bCheckRx1Buf == TRUE)     
    {                 
        return HandleResponse();
    }           
    return FALSE;
}


long GetEzrVer()
{
    return ezrVersion; 
}

void ResetEzrVerNum()
{
    ezrVersion = 0; 
}