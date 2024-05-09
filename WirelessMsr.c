//measurements
//#include <stdio.h>
#include <stdbool.h>
//#include "define.h"
#include "utils.h"
#include "protocol.h"
#include "Valve_Manager.h"
#include "Rtc_Manager.h"
#include "data_manager.h"
#include "interrupts.h"
#include "HW_manager.h"
#include "Ezr_com_manager.h"

#define EZR_RST_ENABLE() (PORTA.5 = 1);        
#define EZR_RST_DISABLE() (PORTA.5 = 0);

#define TASK_MSR_READ       1
#define TASK_MSR_INIT       2
#define TASK_MSR_SAVE       3
#define TASK_MSR_CLOSURE    4
//#define TASK_MSR_LISTEN     5

#define ID      2   
#define MSG_TYPE_POS    6
#define EXT_MSG_POS     7

extern eeprom _tagAPPEEPROM AppEepromData;
flash unsigned char ANSWER_ACK[] = "ACKACKACK@";
static BYTE nBadAnswer;
static BYTE nCycles;
static BYTE nParseAns;
static BYTE fGotData;
static BYTE nNoDataCnt = 0;
//volatile BYTE fListenOrSend;


extern bit bWait4WLSensor;
bool bEndOfMeasureTask;
extern BYTE msrCurTask;
//extern BYTE g_bHighPrio;
#ifdef DebugMode
extern volatile BYTE mainTask;
#endif DebugMode
extern BYTE flgUart1Error;
extern int BytesToSend;
extern int nTimeCnt;
extern char ComBuf[MAX_RX1_BUF_LEN];

void ResetEZR()
{
    unsigned char curState;                            
    curState = PORTD;
    UART1Select(UART_NONE);
    EZR_RST_ENABLE();       // WIRELESS_PWR_DISABLE(); //
    delay_ms(50);
    EZR_RST_DISABLE();      //WIRELESS_PWR_ENABLE();  //
    PORTD = curState; 
}

#ifndef NO_EZR
void InitEZRCom()
{
//    bCheckRx1Buf = FALSE;
//    bWait4WLSensor = TRUE; 
    //read the rtc
//    GetRealTime();
    bEndOfMeasureTask = false;   
    ENABLE_UART1();
    UART1Select(UART_RADIO_UHF);   
    delay_ms(100);    
    // reset buffer
    ResetUart1();
    fGotData = 0;
    nBadAnswer = 0;
    nCycles = 0;           
}

void ReadData()
{
//    unsigned int i;
    bWait4WLSensor = TRUE;
    nTimeCnt = 20;  // 2 sec timeout
    nCycles++;          
    memset(RxUart1Buf,0, MAX_RX1_BUF_LEN);
    rx1_buff_len = 0;
    flgUart1Error = FALSE;
    WIRELESS_CTS_ON();        // allow receiver send data    
    delay_ms(50);         //200  
    WIRELESS_CTS_OFF();            
    PROTOCOL_Task(PROTO_CMD_GET_DATA, 0);  
}

BYTE ValidateEZRPacket()
{
    BYTE i, pcktSize, cs, msgType, index,msgLen;//, res;
    unsigned long lID;   
        
//    mainTask = TASK_MONITOR;   
    fGotData++;  
    
    if (flgUart1Error != 0)
    {
        flgUart1Error = 0;
    }

    #ifdef DebugMode
    SendDebugMsg("\r\nbuffLen: \0");  
    PrintNum(buffLen);
    #endif DebugMode

    if (buffLen < 10)
    {
        i = 0;
        if (strstr(RxUart1Buf, "NODATA")) 
        {
            fGotData = 0;
            return NO_DATA;
        }        
        if (strstr(RxUart1Buf, "NOLIST")) 
        {
            fGotData = 0;  
            g_bVlvListUpdated = true;
            return NO_DATA;
        }

//        do
//        {
//            if ((RxUart1Buf[i + 0] == 'N') &&
//                (RxUart1Buf[i + 1] == 'O') &&
//                (RxUart1Buf[i + 2] == 'D') &&
//                (RxUart1Buf[i + 3] == 'A') &&
//                (RxUart1Buf[i + 4] == 'T') &&
//                (RxUart1Buf[i + 5] == 'A'))    
//            i++;
//        }
//        while ((i + 5) < buffLen);
        return FALSE;
    }

   i = 0;
    while ((RxUart1Buf[i] != 0xAB) && (RxUart1Buf[i] != 0xAC) && (i < buffLen))
        i++;
    #ifdef DebugMode
    SendDebugMsg("\r\n0xAB index: \0");  
    PrintNum(i);
    #endif DebugMode
    if (RxUart1Buf[i] != 0xAB)
    {          
        if (RxUart1Buf[i] != 0xAC)    
            return FALSE;     
        else
            g_bVlvListUpdated = true;    
    }
    pcktSize = RxUart1Buf[i + SIZE_BYTE_INDEX];   

    #ifdef DebugMode
    SendDebugMsg("\r\npcktSize: \0");  
    PrintNum(pcktSize);
    #endif DebugMode
    if (buffLen < pcktSize + 1)    // 1 = number of bytes not include in size
    {
        return FALSE;
    }
    // verify check sum
    cs = CheckSum(&RxUart1Buf[i], pcktSize  ,1);
    if (cs != RxUart1Buf[pcktSize+i])
    {
    #ifdef DebugMode
    SendDebugMsg("\r\ncalc cs: \0");  
    PrintNum(cs);
    SendDebugMsg("\r\ngot cs: \0");  
    PrintNum(RxUart1Buf[pcktSize+i]);
    #endif DebugMode
        return FALSE;
    }      
                  
    index = i + FIRST_DATA_INDEX;
    do
    {
        msgType = RxUart1Buf[index];// == SENSOR_DATA_ID )
        // check msg type                                                                                
        if ((msgType != OBJ_NEW_VCU_DATA) && (msgType != OBJ_NOT_IN_LST_VCU_DATA))
        {
    #ifdef DebugMode
    SendDebugMsg("\r\nmsgType: \0");  
    PrintNum(msgType);
    #endif DebugMode
            return FALSE;
        }
        msgLen = RxUart1Buf[index+1];   
    #ifdef DebugMode
    SendDebugMsg("\r\nmsgLen: \0");  
    PrintNum(msgLen);
    #endif DebugMode
         
        lID = Bytes2ULong(&RxUart1Buf[index+ID]);
    #ifdef DebugMode
    SendDebugMsg("\r\nlID: \0");  
    PrintNum(lID);
    #endif DebugMode
        if (lID <= 500000)
        {
            #ifdef DebugMode
            SendDebugMsg("\r\nID<500000 is wrong. skip data\0");
            #endif DebugMode
                index += msgLen;
            continue;
        }              
        index += msgLen;
    }                      
    while (index < pcktSize);  // && (senIndex < MAX_DATA_FROM_RCVR));  // as long hasn't reach to end of data or got max num sensors for 1 cycle  
    return TRUE;
}

void SavePhytechData()
{
    BYTE i, pcktSize, index, msgType, msgLen;//, situation;
    unsigned long lID;  

//todo - remove
//    #ifdef DebugMode
//    mainTask = TASK_MONITOR;   
//    #endif DebugMode
    GetRealTime(); 
    i = 0;
    while ((RxUart1Buf[i] != 0xAB) && (RxUart1Buf[i] != 0xAC) &&  (i < buffLen))
        i++;
    pcktSize = RxUart1Buf[i + SIZE_BYTE_INDEX];
                
//    #ifdef DebugMode
//    SendDebugMsg("\r\npacket size: \0");  
//    PrintNum(pcktSize);
//    #endif DebugMode
    index = i + FIRST_DATA_INDEX;
    do
    {
        msgType = RxUart1Buf[index];   
    #ifdef DebugMode
    SendDebugMsg("\r\nmsgType: \0");  
    PrintNum(msgType);
    #endif DebugMode
        msgLen = RxUart1Buf[index+1];   
    #ifdef DebugMode
    SendDebugMsg("\r\npacket length: \0");  
    PrintNum(msgLen);
    #endif DebugMode
        
        lID = Bytes2ULong(&RxUart1Buf[index+ID]);  
         #ifdef DebugMode
        SendDebugMsg("\r\nID=\0");   
        PrintNum(lID);
        #endif DebugMode
        if (lID <= 500000)
        {
            #ifdef DebugMode
            SendDebugMsg("\r\nID=0 is wrong. skip data\0");   
            #endif DebugMode  
            if (msgLen == 0)
                return;
            index += msgLen;
            continue;
        }                                          
        #ifdef DebugMode       
        for (i = index; i< msgLen+index; i++)
        {
            PrintOnlyNum(RxUart1Buf[i]); 
            putchar1(',');             
        }
//        SendDebugMsg("\r\nindex+9: \0");     
//        PrintNum(RxUart1Buf[index+9]);           
//        SendDebugMsg("\r\nIndex+10: \0");     
//        PrintNum(RxUart1Buf[index+10]);  
        #endif DebugMode   
        // isolate situation from byte and save it            
        if (msgType == OBJ_NOT_IN_LST_VCU_DATA)    
            SaveVcuData(&RxUart1Buf[index+ID], msgLen-2, RxUart1Buf[index + MSG_TYPE_POS]); 
        if (msgType == OBJ_NEW_VCU_DATA)
        {
            // if got extra situation save it either
            if (RxUart1Buf[index + EXT_MSG_POS] != 0)     
            {                      
                // if got ACK-
                if ((RxUart1Buf[index + EXT_MSG_POS] & IRG_MSG_ACK) != 0)  
                {
                    SaveVcuData(&RxUart1Buf[index+ID], msgLen-2, 6); 
                }                             
                // if got HW error-
                if ((RxUart1Buf[index + EXT_MSG_POS] & IRG_MSG_HW_ERROR) != 0)    
                {
                    SaveVcuData(&RxUart1Buf[index+ID], msgLen-2, 11); 
                }    
                // if got PING-
                if ((RxUart1Buf[index + EXT_MSG_POS] & IRG_MSG_PING) != 0)    
                {
                    SaveVcuData(&RxUart1Buf[index+ID], msgLen-2, 9); 
                }                  
                // if got HW error 2-
                if ((RxUart1Buf[index + EXT_MSG_POS] & IRG_MSG_HW_ERROR_2) != 0)    
                    SaveVcuData(&RxUart1Buf[index+ID], msgLen-2, 12); 
            }    
            if (RxUart1Buf[index + MSG_TYPE_POS] != 0)
                SaveVcuData(&RxUart1Buf[index+ID], msgLen-2, RxUart1Buf[index + MSG_TYPE_POS]);  
            
    //        RxUart1Buf[index+9] = situation;
            UpdateVCUStatus(lID, RxUart1Buf[index + MSG_TYPE_POS],RxUart1Buf[index + EXT_MSG_POS]);    
        }
        index += msgLen;    
    }                          
    while (index < pcktSize);  // as long hasn't reach to end of data or got max num sensors for 1 cycle  
//    if (openPump != 0)
//        CheckVCUStatus();
    // todo - remove
//     #ifdef DebugMode
//    mainTask = TASK_EZR_COM;
//    #endif DebugMode
}

void AnswerReceiver()
{
    char cs;       
      
    CopyFlashToBuf(ComBuf, ANSWER_ACK);       
    ComBuf[3] = g_curTime.minute;    // copy minutes to ack
    ComBuf[4] = g_curTime.second;    // copy seconds to ack  
    //send logger id to receiver      
    cpu_e2_to_MemCopy(&ComBuf[5], &AppEepromData.eLoggerID[0], 4);    
    cs = CheckSum(ComBuf, 9, 1);   
    ComBuf[9] = cs;  
    BytesToSend = 10;
    TransmitBuf(1); 
}


BYTE GetNextMsrTask()
{
    switch (msrCurTask)
    {
        case TASK_NONE:
            msrCurTask = TASK_MSR_INIT;
        break;
        case  TASK_MSR_INIT:
//            if (fListenOrSend == 1)
                msrCurTask = TASK_MSR_READ; 
//            else 
//                msrCurTask = TASK_MSR_LISTEN;                
        break;  
//        case TASK_MSR_LISTEN:  
//            if (bCheckRx1Buf == TRUE)
//                msrCurTask = TASK_MSR_SAVE; 
//            else
//            {
//                if ((nTimeCnt > 0) && (g_cmdSndCnt > 0))
//                    return WAIT; 
//                else
//                    msrCurTask = TASK_MSR_CLOSURE;
//            }
//        break;
        case TASK_MSR_READ:
            if ((nCycles > 3) || (nBadAnswer > 3))  
            {                             
                msrCurTask = TASK_MSR_CLOSURE;
                break;
            }
            if (bCheckRx1Buf == TRUE)
                msrCurTask = TASK_MSR_SAVE;
            else
                if (nTimeCnt > 0)
                    return WAIT;
        break;
        case TASK_MSR_SAVE:   
//            if (fListenOrSend == 2)
//            {                        
//                ResetUart1();
//                msrCurTask = TASK_MSR_LISTEN;   
//                break;
//            }
            if (nParseAns == NO_DATA)
                msrCurTask = TASK_MSR_CLOSURE;
            else
                if ((nParseAns == TRUE) && (buffLen < MAX_RX1_BUF_LEN - 15))
                    msrCurTask = TASK_MSR_CLOSURE;
                else                        
                {
                    delay_ms(500);
                    msrCurTask = TASK_MSR_READ;
                }
        break;
        case TASK_MSR_CLOSURE:
            msrCurTask = TASK_NONE;
            bEndOfMeasureTask = true;
            bWait4WLSensor = FALSE;
        break;
        default:
    }
    return CONTINUE;
}

bool MeasureMain()
{
    BYTE i;
    if (GetNextMsrTask() == WAIT)
        return false;

    switch (msrCurTask)
    {
        case TASK_MSR_INIT:
            InitEZRCom();
        break;
        case TASK_MSR_READ:
            ReadData();
        break;
//        case TASK_MSR_LISTEN:      
//            break;
        case TASK_MSR_SAVE:
            bCheckRx1Buf = FALSE;
            nParseAns = ValidateEZRPacket();   
//            mainTask = TASK_EZR_COM;
           if (nParseAns == TRUE)
            {                      
                nNoDataCnt = 0;
                nCycles = 0;
                AnswerReceiver();   
                SavePhytechData();  
            }   
            else     
                if (nParseAns == FALSE)      
                {
                    nBadAnswer++;   
                     #ifdef DebugMode       
                     mainTask = TASK_MONITOR;
                    SendDebugMsg("\r\nVALIDATION FAIL. buffLen= \0");     
                    for (i = 0; i < buffLen; i++) 
                    {
                        PrintOnlyNum(RxUart1Buf[i]); 
                        putchar1(',');  
                    }               
                    mainTask = TASK_EZR_COM;  
                    UART1Select(UART_RADIO_UHF);   
                    #endif DebugMode     
                }   
                else
                    if (nParseAns == NO_DATA)
                        nNoDataCnt++;       
        break;
        case TASK_MSR_CLOSURE:       
            msrCurTask = TASK_NONE;
            bEndOfMeasureTask = true;
            bWait4WLSensor = FALSE;
             if (/*(nCycles > 3) || (nBadAnswer > 3) ||*/ (nNoDataCnt > 60))       // todo - check if need
             {
                ResetEZR();     
                nNoDataCnt = 0;
             }
            break;
        default:
    }
    return bEndOfMeasureTask;
}

#endif NO_EZR

