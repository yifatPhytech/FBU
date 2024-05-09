//RS485_COM
#include <string.h>

//#include "define.h"
#include "utils.h"
#include "crc.h"
#include "RS485_Manager.h"
#include "Valve_Manager.h"
#include "protocol.h"
#include "data_manager.h"
#include "modem_manager.h"
#include "interrupts.h"
#include "HW_manager.h"


#define CCU_HEADER 0x0F
#define CBU_HEADER 0xF0
#define CMD_CLOSE 0
#define CMD_OPEN  1
#define HEADER_INDEX    0
#define SIZE_INDEX      1
#define CRC_INDEX       2
#define PAYLOAD_INDEX   4
#define PERMANENT_BYTE_CNT    5
#define PUMP_STATE_INDEX    43

extern eeprom _tagPortDefEEPROM ComponentArray[];
extern eeprom _tagAPPEEPROM AppEepromData;
extern flash unsigned char RomVersion[]; 
extern eeprom _tagFlowDefEEPROM FlowDef[];
 

extern bit g_LockUar1;
extern BYTE bEndOfCbuTask;
extern BYTE msrCurTask;
extern int BytesToSend;
extern int nTimeCnt;
extern char ComBuf[MAX_RX1_BUF_LEN];
extern DateTime g_LastCnctTime;
extern int iVoltage;
extern BYTE rssi_val;

static BYTE prevUART1Stat;
int g_nFailureCnt = 0;
extern volatile BYTE mainTask;
char g_bCBUPumpState[2];

//#define RS485_CTRL_RX() (PORTB.3 = 0);        //##
//#define RS485_CTRL_TX() (PORTB.3 = 1);
void BuildPackage(ERS485Command cmd, int prm)
{
    BYTE size = 1;  
    unsigned int crc;//, sec = 100;
                  
    memset(ComBuf,0, 200);
    ComBuf[HEADER_INDEX] = CCU_HEADER;//0x0'I';    // Start Of Frame     
    ComBuf[PAYLOAD_INDEX] = cmd;    //CMD_CODE[cmd];    
    switch (cmd)
    {
    case CMD_LED:
        int2bytes(prm, &ComBuf[PAYLOAD_INDEX+1]); 
        size += 2;   
        break;
    case CMD_SET_CBU_DEF:    
//        ComBuf[PAYLOAD_INDEX+1] = AppEepromData.eCbuGlblData[8];        
        cpu_e2_to_MemCopy(&ComBuf[PAYLOAD_INDEX+size], (eeprom char*)(&AppEepromData.eCbuGlblData[8]), CBU_SET_GLBL_CNFG_LEN); 
        size += CBU_SET_GLBL_CNFG_LEN;
        cpu_e2_to_MemCopy(&ComBuf[PAYLOAD_INDEX+size], (eeprom char*)(&ComponentArray[prm]), sizeof(_tagPortDefEEPROM) * MAX_PORTS_CBU);   
        size += sizeof(_tagPortDefEEPROM) * MAX_PORTS_CBU;
        cpu_e2_to_MemCopy(&ComBuf[PAYLOAD_INDEX+size], (eeprom char*)(&FlowDef[0]), sizeof(_tagFlowDefEEPROM) * 2);    
        size += sizeof(_tagFlowDefEEPROM)*2;
        break;        
    case CMD_GET_CBU_DEF:   
    break; 
    case CMD_FW_UPGRADE:
        cpu_e2_to_MemCopy( &ComBuf[PAYLOAD_INDEX+1], &AppEepromData.eLoggerID[0], 4); 
        int2bytes(prm, &ComBuf[PAYLOAD_INDEX+5]);               
        CopyFlashToBuf(&ComBuf[PAYLOAD_INDEX+6],fSWUpdateAddress);
        size = 39;
        break; 
        
    }                            
    ComBuf[SIZE_INDEX] = size;
    crc = CRC16_CCITT(&ComBuf[PAYLOAD_INDEX], size, 0xFFFF);            
    int2bytes(crc, &ComBuf[CRC_INDEX]);
    ComBuf[4+size] = '\n';      // End Of msg
    BytesToSend = size + PERMANENT_BYTE_CNT;
}
           

void InitCom(BYTE bSend)
{
    if (g_nFailureCnt >= 2)
    {
        UCSR1A=(0<<RXC1) | (0<<TXC1) | (0<<UDRE1) | (0<<FE1) | (0<<DOR1) | (0<<UPE1) | (1<<U2X1) | (0<<MPCM1);
        UCSR1C=(0<<UMSEL11) | (0<<UMSEL10) | (0<<UPM11) | (0<<UPM10) | (0<<USBS1) | (1<<UCSZ11) | (1<<UCSZ10) | (0<<UCPOL1);
        UBRR1H=0x00;      
        g_nFailureCnt = 0;
    }
//    #ifdef DebugMode
//        SendDebugMsg("\r\ninit RS485\0");
//    #endif  DebugMode    
//    bCheckRx1Buf = FALSE;    
    //read the rtc
//    GetRealTime();   
    ENABLE_UART1();  
    SetUART1BaudRate(RATE9600); //(RATE19200); 
    
    prevUART1Stat = (PORTD  & 0x30);    //>> 4) & 0x03);
    UART1Select(UART_RS485);       
//    RS485_CTRL_INT_ON();  
//    delay_ms(10);
//    RS485_CTRL_INT_OFF();
//    delay_ms(5);    
    ResetUart1();
    if (bSend == TRUE)
        RS485_CTRL_TX();     
    else
        RS485_CTRL_RX();  
              
    ///////////// WAKE-UP CBU //////////////     
    BuildPackage(CMD_WU, 0);
    TransmitBuf(1);    
    delay_ms(100);
//    bWait4RS485 = TRUE;
}

void DeInitCom()
{
//    bCheckRx1Buf = FALSE;
//    ENABLE_UART1();
//    RS485_CTRL_RX();        
    UART1Select(prevUART1Stat/*UART_DBG*/);       
    UART1Select(prevUART1Stat/*UART_DBG*/);       
    SetUART1BaudRate(RATE38400);
}

/*BYTE IsStringInside(flash char* buf1, BYTE size, char* buf2, BYTE maxSize)
{
    BYTE index1 = 0, index2 = 0, nCnt = 0, startIdx = 0;    
    #ifdef DebugMode
        SendDebugMsg("\r\nFIND STRING \0");      
        SendDebugMsg(buf1);
    #endif  DebugMode       
    do
    {    
        while (buf1[index1] != buf2[startIdx])
            startIdx++;
        
        index2 = startIdx;
            
        while ((index2 < maxSize) && (buf1[index1++] == buf2[index2++]))
        {            
            nCnt++;
            if (nCnt == size)   
            {
                #ifdef DebugMode
        SendDebugMsg("\r\nFOUND\0");
    #endif  DebugMode    
                return TRUE;
            }                       
        }
        index1 = 0;
    }      
    while (index2 < maxSize);
    return FALSE;
} */
/*
int GetNumber(char* buf, BYTE maxSize)
{
    BYTE index1 = 0;
    int nNum = 0;    
    #ifdef DebugMode
        SendDebugMsg("\r\nFIND number \0");      
    #endif  DebugMode       
   
    while (((buf[index1] < '0') || (buf[index1] > '9')) && (index1 < maxSize))
        index1++;
       
    if (index1 == maxSize) 
        return -1;
    nNum = buf[index1] - 0x30;    
    index1++;        
    while (((buf[index1] >= '0') && (buf[index1] <= '9')) && (index1 < maxSize))
    {        
        nNum *= 10;
        nNum += buf[index1++] - 0x30; 
    }  
     #ifdef DebugMode
        SendDebugMsg("\r\nnumber is: \0");
        PrintNum(nNum);      
    #endif  DebugMode              
    return nNum;
}  */

BYTE ValidateCBUHeader()
{
    int pctSize, crc,  crc2; //   crc1,
 //   BYTE i = 1, res = ERROR; 
    
    if (RxUart1Buf[HEADER_INDEX] != CBU_HEADER)  
    {
//        #ifdef DebugMode
//        SendDebugMsg("\r\nHeader Not fit\0");     
//        #endif DebugMode                       
        return ERROR;  
    }
    pctSize = RxUart1Buf[SIZE_INDEX];
    if (pctSize > buffLen)
    {
//        #ifdef DebugMode
//        SendDebugMsg("\r\nSize Not fit: buffLen, pctSize\0");  
//        PrintNum(buffLen);   
//        PrintNum(pctSize);   
//        #endif DebugMode                       
        return ERROR;  
    }
    crc = bytes2int(&RxUart1Buf[CRC_INDEX]);         
//    crc1 = CRC16_CCITT(&RxUart1Buf[PAYLOAD_INDEX], pctSize-1, 0xFFFF);  //kaufman CBU
    crc2 = CRC16_CCITT(&RxUart1Buf[PAYLOAD_INDEX], pctSize, 0xFFFF);      //Itay's CBU
    if (/*(crc != crc1) && */(crc != crc2))
    {
        #ifdef DebugMode
        SendDebugMsg("\r\nWrong CRC\0");     
        PrintNum(crc2);   
        #endif DebugMode   
        return ERROR;                  
    }    

//    if (crc == crc1)
//        g_nCbuVer = kaufmanCBU;
//    if (crc == crc2)
//        g_nCbuVer = ItayCBU;
 
    return TRUE;
}

BYTE ParseCbuRequest()
{
    BYTE  /*index = PAYLOAD_INDEX+1,*/ payloadSize = 0;//, cmd;  
    unsigned int crc;

    if (ValidateCBUHeader() != TRUE)
        return ERROR;        
        
    memset(ComBuf,0, 200);
    ComBuf[HEADER_INDEX] = CCU_HEADER;    
    ComBuf[PAYLOAD_INDEX] = RxUart1Buf[PAYLOAD_INDEX]-0x20;  //?               

    switch (RxUart1Buf[PAYLOAD_INDEX])
    {       
        case 97:    //CMD_ALERT: //'a':   
            SaveAlertData(&RxUart1Buf[PAYLOAD_INDEX+1]);  
            ComBuf[PAYLOAD_INDEX] = CMD_ALERT;     
            ComBuf[PAYLOAD_INDEX+1] = 1;
            payloadSize = 2;
        break;
//        case CMD_RST_CCU:       //'n':   //(CMD_CODE[CMD_RST_CCU]):    
//            ComBuf[PAYLOAD_INDEX+1] = 1;   
//            payloadSize = 2;
//        break;
/*        case 'L':   //CMD_CODE[CMD_SEND_PING]:       //todo - define
            ComBuf[PAYLOAD_INDEX+1] = 1;    
            payloadSize = 2;
        break;
        case 'P': //CMD_CODE[CMD_GET_CCU_GNRL_PRM]:   
            memcpy(&ComBuf[index], (char*)AppEepromData.eLoggerID[0], 4);
            index += 4;
            // ATMEL Version
            cpu_flash_to_MemCopy(&ComBuf[index], RomVersion, 4);
            index += 4;
            // EZR Version
            ULong2Bytes(GetEzrVer(), &ComBuf[index]);
            index += 4;    
            //Battery
            ComBuf[index++] = (unsigned char)((iVoltage >> 8) & 0xFF);     //address high
            ComBuf[index++] = (unsigned char)(iVoltage) ;                 //address low
            // RSSI
            ComBuf[index++] = rssi_val;              
            //network name     
  //          Long2Str(OprtTbl[curOprtIndex], &ComBuf[index]);    ///??  
            index += 4;    
            //last connecting time
            memcpy(&ComBuf[index] , &g_LastCnctTime, 6);
            index += 6; 
            payloadSize = index - PAYLOAD_INDEX;                     
        break;
        case 'Q':   //CMD_CODE[CMD_GET_CCU_COM_PRM]:  
            // URL
            cpu_e2_to_MemCopy( &ComBuf[index], AppEepromData.eIPorURLval1, 32);
            index += 32;
            //Port
            cpu_e2_to_MemCopy( &ComBuf[index], AppEepromData.ePORTval1, 4);
            index += 4;
            // APN
            cpu_e2_to_MemCopy( &ComBuf[index], AppEepromData.eAPN, 32);
            index += 32;

            // ICCID num.
    //        memcpy(&ComBuf[index],&ICCID[0], MAX_ICCID_LEN);   
            index += MAX_ICCID_LEN;

            // mobile net code
            cpu_e2_to_MemCopy( &ComBuf[index], AppEepromData.eMobileNetCode, 4);
            index += 4;

            // mobile country code
            cpu_e2_to_MemCopy( &ComBuf[index], AppEepromData.eMobileCntryCode, 4);
            index += 4;
            
            payloadSize = index - PAYLOAD_INDEX; 
        break;
        case 'R':   //CMD_CODE[CMD_SET_CCU_COM_PRM]:    
            MemCopy_to_cpu_e2(AppEepromData.eIPorURLval1, &RxUart1Buf[index], 32);    
            index += 32;
            MemCopy_to_cpu_e2(AppEepromData.ePORTval1, &RxUart1Buf[index], 4);  
            index += 4;
            MemCopy_to_cpu_e2(AppEepromData.eAPN, &RxUart1Buf[index], 32);  
            index += 32;                                    
            MemCopy_to_cpu_e2(AppEepromData.eMobileNetCode, &RxUart1Buf[index], 4);  
            index += 4;
            MemCopy_to_cpu_e2(AppEepromData.eMobileCntryCode, &RxUart1Buf[index], 4);  
            index += 4;
            ComBuf[PAYLOAD_INDEX+1] = 1;    
            payloadSize = 2;
        break;
//        case 'S':   //CMD_CODE[CMD_ALERT]:  
//            SaveAlertData(RxUart1Buf[index], bytes2int(&RxUart1Buf[index+1])); 
//            ComBuf[PAYLOAD_INDEX+1] = 1;    
//            payloadSize = 2;
//        break;
        case 'T':   //CMD_CODE[CMD_VCU_MNG]:  
//            if (RxUart1Buf[index] == 0)
//                InsertNewCmd(Bytes2ULong(&RxUart1Buf[index+1]),  0, 0, 0, 0, 0);  // stop vlv
//            else
//                InsertNewCmd(Bytes2ULong(&RxUart1Buf[index+1]), 60, 0, 0, 0, 0);  // start valve   // max duration?
        break;
        case 'U':   //CMD_CODE[CMD_VCU_STATUS]:   
        break;                                    */
        default:    
    }           
    ComBuf[SIZE_INDEX] = payloadSize;
    crc = CRC16_CCITT(&ComBuf[PAYLOAD_INDEX], payloadSize, 0xFFFF);            
    int2bytes(crc, &ComBuf[CRC_INDEX]);        
    ComBuf[4+payloadSize] = '\n'; 
    BytesToSend = payloadSize + PERMANENT_BYTE_CNT;
    return TRUE;
}

BYTE ParseCbuResponse(ERS485Command cmd)
{
//    BYTE  i, n; 
    BYTE n, res = ERROR;  
    
    if (ValidateCBUHeader() != TRUE)
        return ERROR;
   
    if (/*CMD_CODE[*/cmd != (RxUart1Buf[PAYLOAD_INDEX]))  // A --> a ?
    {       
 //       if ((CMD_CODE[cmd] != (RxUart1Buf[PAYLOAD_INDEX] - 0x20))  && (CMD_CODE[cmd] != (RxUart1Buf[PAYLOAD_INDEX] + 0x20)))
        {
            #ifdef DebugMode
            SendDebugMsg("\r\nWrong Response Command\0");     
            #endif DebugMode                       
            return ERROR;      
        }
    }
        
    switch (cmd)
    {       
    case CMD_RST_CBU:
        res = 1;    
    break; 
//    case CMD_PUMP1_MNG:
//    case CMD_PUMP2_MNG:
    case CMD_SET_CBU_DEF:
        if (RxUart1Buf[PAYLOAD_INDEX+1] == 1)
            res = 1;
        break;        
    case CMD_GET_CBU_DEF:     
        n = PAYLOAD_INDEX+1;
        MemCopy_to_cpu_e2( (char eeprom*)&AppEepromData.eCbuGlblData, &RxUart1Buf[n], CBU_GET_GLBL_CNFG_LEN);   
        n += CBU_GET_GLBL_CNFG_LEN;
        MemCopy_to_cpu_e2((char eeprom*)&ComponentArray, &RxUart1Buf[n],  MAX_PORTS_CBU * sizeof(_tagPortDefEEPROM));   
        n += MAX_PORTS_CBU * sizeof(_tagPortDefEEPROM); 
        MemCopy_to_cpu_e2((char eeprom*)&FlowDef[0].HighTreshold, (char*)(&RxUart1Buf[n]), sizeof(_tagFlowDefEEPROM)-1);   
        n += sizeof(_tagFlowDefEEPROM)-1;
        MemCopy_to_cpu_e2((char eeprom*)&FlowDef[1].HighTreshold, (char*)(&RxUart1Buf[n]), sizeof(_tagFlowDefEEPROM)-1);   
        g_bSendCBUMeta = TRUE;         
//        vlvCmdArr[MAX_CMD].VCU_ID = Bytes2ULong(&RxUart1Buf[PAYLOAD_INDEX+1]); 
        PrintSensorDef();
        PrintFlowData();
        res = 1;
        break;  
    
    case CMD_FW_UPGRADE:    
        res = RxUart1Buf[PAYLOAD_INDEX+1];    
        #ifdef DebugMode
        SendDebugMsg("\r\nstart FW update \0");   
        #endif DebugMode
        break;
    case CMD_GET_MNT_DATA:  
        SaveCBUPortData(&RxUart1Buf[PAYLOAD_INDEX+1]);        
//        memcpy(&g_bCBUPumpState, &RxUart1Buf[PAYLOAD_INDEX+PUMP_STATE_INDEX], 2); 
        res = 1; 
        break;   
        default:                                                                   
    }     
        
    #ifdef DebugMode
    SendDebugMsg("\r\nresult = \0");   
    PrintNum(res);  
    #endif DebugMode                       
    
    return res;
}

char SendRecRS485(ERS485Command cmd, int prm)
{
    BYTE i, res = 0;      
                     
//    if (IsZeroID(AppEepromData.eLoggerID))
//        if (cmd != CMD_GET_CBU_DEF)
//            return FALSE;
    #ifdef DebugMode
    SendDebugMsg("\r\nSend Rec RS485 cmd: \0");  
    PrintNum(cmd); 
    #endif DebugMode   
    DISABLE_CBU_INT();
    InitCom(TRUE);
    BuildPackage(cmd, prm);
    TransmitBuf(1);              
    delay_ms(1);     
    RS485_CTRL_RX();    
    nTimeCnt = 20;
    while ((nTimeCnt > 0) && (bCheckRx1Buf == FALSE));
    
    DeInitCom();    
     if (bCheckRx1Buf == TRUE)     
    {              
        #ifdef DebugMode
        SendDebugMsg("\r\nmsg arrived. buffLen= \0");     
        PrintNum(buffLen);    
        for (i = 0; i < buffLen; i++) 
        {
            PrintOnlyNum(RxUart1Buf[i]); 
            putchar1(',');  
        }
//            PrintNum(RxUart1Buf[i]);  
        #endif DebugMode           
        res = ParseCbuResponse(cmd);   
        g_nFailureCnt = 0;
    } 
    else       
     {              
        #ifdef DebugMode
        SendDebugMsg("\r\ngot nothing\0");     
        #endif DebugMode  
        if (cmd == CMD_GET_MNT_DATA)   
            g_nFailureCnt++;
     }       
                 
     ENABLE_CBU_INT();
     return res;
}

void RecSendRS485()
{
    #ifdef DebugMode
    BYTE RES = 0;
    #endif DebugMode    
    DISABLE_CBU_INT();
    InitCom(FALSE);   
    g_LockUar1 = TRUE;
    nTimeCnt = 15;
    while ((nTimeCnt > 0) && (bCheckRx1Buf == FALSE));   
    
    if (bCheckRx1Buf == TRUE)     
    {              
        if (ParseCbuRequest() == TRUE)
        {
            RS485_CTRL_TX();
            delay_ms(1);     
            TransmitBuf(1);  
            delay_ms(1);             
            RS485_CTRL_RX();        
            //DeInitCom();
            #ifdef DebugMode
            RES = 1;
            #endif DebugMode        
        } 
        #ifdef DebugMode
        else
            RES = 2;       
        #endif DebugMode        
    } 
    #ifdef DebugMode
    else 
    {      
        RES = 3;    
//        ComBuf[HEADER_INDEX] = CCU_HEADER;  
//        ComBuf[1] = 1;
//        ComBuf[2] = 2;
//        ComBuf[3] = 3;  
//        ComBuf[4] = '\n'; 
//        BytesToSend = 5;
//        RS485_CTRL_TX();
//            delay_ms(1);     
//            TransmitBuf(1);  
//            delay_ms(1);             
//            RS485_CTRL_RX();   
    }  
    #endif DebugMode  
    g_LockUar1 = FALSE;
//    g_fRS485Call = 0; 
    DeInitCom();  
    delay_ms(10);  
    ENABLE_CBU_INT(); 
    #ifdef DebugMode   
    
    SendDebugMsg("\r\nRecSendRS485 RES = \0");  
    PrintNum(RES);
    SendDebugMsg("\r\ncurrent mainTask = ");       
    PrintNum(mainTask);    
    #endif DebugMode   
}

 /*
void GetNextCBUTask()
{
    switch (msrCurTask)
    {
        case TASK_NONE:
            msrCurTask = TASK_CBU_INIT;
        break;
        case  TASK_CBU_INIT:
            msrCurTask = TASK_CBU_PUMP_STATUS;
        break;
        case TASK_CBU_PUMP_STATUS:  
            msrCurTask = TASK_CBU_PULSE;
        break;
        case TASK_CBU_PULSE:
            msrCurTask = TASK_CBU_4_20_1ST;
        break;
        case TASK_CBU_4_20_1ST:
            msrCurTask = TASK_CBU_4_20_2ND;
        break;
        case TASK_CBU_4_20_2ND:  
            msrCurTask = TASK_CBU_4_20_3RD;
        break;
        case TASK_CBU_4_20_3RD:  
            if (g_curTime.minute == 0)
                msrCurTask = TASK_CBU_BTR;
            else
                bEndOfCbuTask = TRUE;
        break;     
        case TASK_CBU_BTR:
            bEndOfCbuTask = TRUE;
        default:
    }
//    return CONTINUE;
}

*/