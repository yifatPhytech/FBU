#include "define.h"
#include "utils.h"
#include "Valve_Manager.h"
#include "Rtc_Manager.h"
#include "data_manager.h"
#include "modem_manager.h"
#include "interrupts.h"
#include "HW_manager.h"

#define GET_REQUEST 0
#define SET_REQUEST 1

#define REQ_ID      0           // LOGGER id
#define REQ_TIME    40
#define REQ_IP      41
#define REQ_PORT    42
#define REQ_APN     43
#define REQ_MCC     44
#define REQ_MNC     45
#define REQ_ROAMING 46
#define REQ_SCH     47  // START CONNECT HOUR
#define REQ_CPD     48  //CONNECTS PER DAY
#define REQ_CI      49  //CONNECTS INTERVAL
#define REQ_BTR     55
#define REQ_RSSI    56
#define REQ_TIMEZONE     57
#define REQ_VER     58
//#define REQ_NUM_SEN     59
#define REQ_DISCNCT     61
//#define REQ_EPOCH       64
#define REQ_ICCID       65
//#define REQ_MODEM       66
#define REQ_CONNECT     67
#define REQ_TEST_PUMP   68
#define REQ_CBU_VER     69


extern eeprom _tagAPPEEPROM AppEepromData;

flash unsigned char IdentificationStr[] = "BEEPBEEP@";
BYTE requestType;
BYTE requestLen;
BYTE requestIndex;
BYTE setResult;
long lTimeFromLastTask;
extern BYTE monitorCurTask;
//extern BYTE fModemModel;
extern flash unsigned char RomVersion[];
extern int iVoltage;
//extern BYTE rssi_val;
extern char ComBuf[MAX_RX1_BUF_LEN];
//extern char RxUart1Buf[MAX_RX1_BUF_LEN];
//extern DateTime g_curTime;
//extern int buffLen;
extern int BytesToSend;
extern int TimeLeftForWaiting;
extern char clockBuf[7]; 		 //buffer for all clock operation need
//extern bit bCheckRx1Buf;
extern bit bWaitForMonitorCmd;
extern bit bEndOfMonitorTask;
extern BYTE bMonitorConnected;

void SendConnectString()
{
    BytesToSend = CopyFlashToBuf(ComBuf, IdentificationStr);
    delay_ms(20);
    TransmitBuf(1);
    delay_ms(10);
}


BYTE CheckMonitorRequest()
{
    BYTE i, cs;
    requestLen = 0;
    requestType = 0;
    requestIndex = 0;

    if (!((RxUart1Buf[0] == 0xff) && (RxUart1Buf[1] == 0xff)))
        return FALSE;
    // if size of request is more than bytes arrived:
    requestLen = RxUart1Buf[2];
    if (requestLen > buffLen)//rx1_buff_len)
        return FALSE;
    i = IsSameID(&RxUart1Buf[3]);
    if (i == 0)
        return FALSE;
    requestType = RxUart1Buf[7];     // get or set
    requestIndex = RxUart1Buf[8];
    if (requestIndex < 10)
    {
        if (requestIndex == 0)    //0 for logger 1 and up - for the sensors
        {
            requestIndex = REQ_ID;
        }
    }

    cs = CheckSum(RxUart1Buf, requestLen, 1);
    if (cs != RxUart1Buf[requestLen])
        return FALSE;
    return TRUE;
}

void ExecuteGetCommand()
{
//    int n;
    BYTE len = 0, cs;//, i;
    ComBuf[0] = 0xff;
    ComBuf[1] = 0xff;
    cpu_e2_to_MemCopy( &ComBuf[3], &AppEepromData.eLoggerID[0], 4);
    ComBuf[7] = requestIndex;
    switch (requestIndex)
    {
        case REQ_ID:
            len = 4;
            cpu_e2_to_MemCopy( &ComBuf[8], &AppEepromData.eLoggerID[0], 4);
        break;
        case REQ_TIME:
            GetRealTime();
            ComBuf[8] = g_curTime.year;// =readClockBuf[0]; //year
            ComBuf[9] = g_curTime.month;// readClockBuf[1]; //month
            ComBuf[10] = g_curTime.day;// =  readClockBuf[2]; //day
            ComBuf[11] = g_curTime.hour;// = readClockBuf[4]; //hour
            ComBuf[12] = g_curTime.minute;// readClockBuf[5]; //minute
            len = 5;
        break;
        case REQ_IP:
            cpu_e2_to_MemCopy( &ComBuf[8], AppEepromData.eIPorURLval1, 32);
            len = 32;
        break;
        case REQ_PORT:
            cpu_e2_to_MemCopy( &ComBuf[8], AppEepromData.ePORTval1, 4);
            len = 4;
        break;
        case REQ_APN:
            cpu_e2_to_MemCopy( &ComBuf[8], AppEepromData.eAPN, 32);
            len = 32;
        break;
        case REQ_MCC:
            cpu_e2_to_MemCopy( &ComBuf[8], AppEepromData.eMobileCntryCode, 4);
            len = 4;
        break;
        case REQ_MNC:
            cpu_e2_to_MemCopy( &ComBuf[8], AppEepromData.eMobileNetCode, 4);
            len = 4;
        break;
        case REQ_ROAMING:
            ComBuf[8] = !AppEepromData.eUseCntrCode;
            len = 1;
        break;
        case REQ_SCH:
            ComBuf[8] = AppEepromData.eStartConnectionH;
            len = 1;
        break;
        case REQ_CPD:
            ComBuf[8] = AppEepromData.eConnectionInDay;
            len = 1;
        break;
        case REQ_CI:
            ComBuf[8] = AppEepromData.eConnectIntervalH;
            len = 1;
        break;
        case REQ_BTR:
            ComBuf[8] = (unsigned char)(iVoltage) ;                 //address low
            ComBuf[9] = (unsigned char)((iVoltage >> 8) & 0xFF);     //address high
             len = 2;
        break;
        case REQ_RSSI:
            ComBuf[8] = (unsigned char)(rssi_val) ;                 //address low
            ComBuf[9] = (unsigned char)((rssi_val >> 8) & 0xFF);     //address high
            len = 2;
        break;
        case REQ_TIMEZONE:
            int2bytes(AppEepromData.eTimeZoneOffset, &ComBuf[8]);
            len = 2;
        break;
        case REQ_VER:
            cpu_flash_to_MemCopy( &ComBuf[8], RomVersion, 4);
            len = 4;
        break;
        case REQ_ICCID:            
            CopyICCID(8);    
            len = MAX_ICCID_LEN;
        break; 
        case REQ_CBU_VER:          
            cpu_e2_to_MemCopy(&ComBuf[8], &AppEepromData.eCbuGlblData[4], 4);
            len  = 4;
            break;
//        case REQ_MODEM:
//            ComBuf[8] = 1;//fModemModel;
//            len = 1;
//        break;        
        default:
    }
    ComBuf[2] = len + 8;
    cs = CheckSum(ComBuf, len + 8, 1);
    ComBuf[len+8] = cs;
    BytesToSend = len + 9;
    TransmitBuf(1);
}

void SendBackResult()
{
    BYTE  cs;
    ComBuf[0] = 255;
    ComBuf[1] = 255;
    ComBuf[2] = 9;
    cpu_e2_to_MemCopy( &ComBuf[3], &AppEepromData.eLoggerID[0], 4);
    ComBuf[7] = requestIndex;
    ComBuf[8] = setResult;
    cs = CheckSum(ComBuf, 9, 1);
    ComBuf[9] = cs;
    BytesToSend = 10;
    TransmitBuf(1);
}

void ExecuteSetCommand()
{
    BYTE n;
    int t;

    setResult = TRUE;
    switch (requestIndex)
    {
        case REQ_ID:
                MemCopy_to_cpu_e2(&AppEepromData.eLoggerID[0], &RxUart1Buf[9], 4);
        break;
        case REQ_TIME:
            MemCopy( clockBuf, &RxUart1Buf[9], 3 ); //copy year month day
            MemCopy( &clockBuf[4], &RxUart1Buf[12], 2 );  //copy hour minute
            if(SetRealTime() == FAILURE)
            {
                setResult = FALSE;
            }
        break;
        case REQ_IP:
            MemCopy_to_cpu_e2(AppEepromData.eIPorURLval1, &RxUart1Buf[9], 32);
        break;
        case REQ_PORT:
            for (n = 0; n < 4; n++)
                if ((RxUart1Buf[n+9] < '0') || (RxUart1Buf[n+9] > '9'))  //if port is not a number
                    setResult = FALSE;
            if (setResult)
                MemCopy_to_cpu_e2(AppEepromData.ePORTval1, &RxUart1Buf[9], 4);
        break;
        case REQ_APN:
            MemCopy_to_cpu_e2(AppEepromData.eAPN, &RxUart1Buf[9], 32);
        break;
        case REQ_MCC:
            MemCopy_to_cpu_e2(AppEepromData.eMobileCntryCode, &RxUart1Buf[9], 4);
        break;
        case REQ_MNC:
            MemCopy_to_cpu_e2(AppEepromData.eMobileNetCode, &RxUart1Buf[9], 4);
        break;
        case REQ_ROAMING:
            // the opposite 0=>1 1=>0
            if (RxUart1Buf[9] == 0)
                AppEepromData.eUseCntrCode = 1;
            else
                if (RxUart1Buf[9] == 1)
                    AppEepromData.eUseCntrCode = 0;
                else
                    setResult = FALSE;
        break;
        case REQ_SCH:
            if ((RxUart1Buf[9] >= 0) && (RxUart1Buf[9] < 23))
                AppEepromData.eStartConnectionH = RxUart1Buf[9];
            else
                setResult = FALSE;
        break;
        case REQ_CPD:
            if ((RxUart1Buf[9] > 0) && (RxUart1Buf[9] <= 24))
                AppEepromData.eConnectionInDay = RxUart1Buf[9];
            else
                setResult = FALSE;
        break;
        case REQ_CI:
            if ((RxUart1Buf[9] >= 0) && (RxUart1Buf[9] < 24))
                AppEepromData.eConnectIntervalH = RxUart1Buf[9];
            else
                setResult = FALSE;
        break;
        case REQ_TIMEZONE:
            t = bytes2int(&RxUart1Buf[9]);
            if ((t >= -720) && (t <= 720))
                AppEepromData.eTimeZoneOffset = t;
            else
                setResult = FALSE;
        break;
        case REQ_VER:    //NOT AVAILABLE TO SET
            break;
        
        case REQ_DISCNCT:
            bMonitorConnected = FALSE;
            bWaitForMonitorCmd = FALSE;
            bEndOfMonitorTask = TRUE;  
            return;
        break;   
        case REQ_CONNECT:         
            InitVarsForConnecting();                    
            return;
        break;
        case REQ_TEST_PUMP:
            TestMainPump(1);    
            InitVarsForConnecting();
            return;
        break;
        default:
            setResult = FALSE;
    }
    SendBackResult();
}

void MonitorMain()
{
    if (monitorCurTask == TASK_MONITOR_CONNECT)
    {
        ENABLE_UART1();
        UART1Select(UART_DBG);
        bMonitorConnected = FALSE;
        //#ifdef DebugMode
        delay_ms(2000);
        //#endif DebugMode
        SendConnectString();
        lTimeFromLastTask = 0;
        monitorCurTask = TASK_MONITOR_WAIT;
    }
    else
        if (monitorCurTask == TASK_MONITOR_WAIT)
        {
            if ((bWaitForMonitorCmd == TRUE) && (TimeLeftForWaiting == (int)0) && (bMonitorConnected == FALSE))
            {
                bCheckRx1Buf = FALSE;
                bWaitForMonitorCmd = FALSE;
                bEndOfMonitorTask = TRUE;
            }
            // if flag of end of rx received is on
            if (bCheckRx1Buf == TRUE)
            {
                bCheckRx1Buf = FALSE;
                lTimeFromLastTask = 0;
                if (CheckMonitorRequest() == FALSE)
                {
                    if (bMonitorConnected == FALSE)     //maybe always on wrong buffer get out??
                        bEndOfMonitorTask = TRUE;
                    setResult = FALSE;
                }
                else
                {
                    bMonitorConnected = TRUE;
                    if (requestType == GET_REQUEST)
                        ExecuteGetCommand();
                    else
                        if (requestType == SET_REQUEST)
                            ExecuteSetCommand();
                }
            }
            else
            {
                lTimeFromLastTask++;
                if (lTimeFromLastTask >= 10000000)  // counter will be ~2,000,000 in one minute. after 5 minutes ~10,000,000.
                {
                    bWaitForMonitorCmd = FALSE;
                    bEndOfMonitorTask = TRUE;
                    bMonitorConnected = FALSE; 
                    lTimeFromLastTask = 0;
                }
            }
        }
}

