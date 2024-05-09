#ifdef Satellite
#ifndef SATELLITE_MODEM_MANAGER_C
#define SATELLITE_MODEM_MANAGER_C
#include <stdlib.h>
#include <math.h>
#include "define.h"

extern flash unsigned char RomVersion[];
flash unsigned char  AT_OK[] = "AT\r\n\0";             //5 bytes
flash unsigned char AT_SBDREGASK[] = "AT+SBDREG?\r\n\0";  // register status question
flash unsigned char AT_SBDREG[] = "AT+SBDREG\r\n\0";  // register status
flash unsigned char AT_PWR_DWN[] = "AT*F\r\n\0";    // power down
flash unsigned char AT_SBDWB[] = "AT+SBDWB=@\r\n\0";   // Transfer message to ISU        (1)
flash unsigned char AT_SBDI[] = "AT+SBDI\r\n\0";     // instructs the ISU to initiate an SBD transfer (2)
flash unsigned char AT_SBDD0[] = "AT+SBDD0\r\n\0";    // instructs the ISU to clear the message from the send buffer    (3)
flash unsigned char AT_SBDRB[] = "AT+SBDRB\r\n\0";    // instructs the ISU to transfer the received message             (4)
flash unsigned char AT_SBDMTA[] = "AT+SBDMTA=0\r\n\0";  //terminate SBDRING
flash unsigned char AT_E0[] = "ATE0\r\n\0";       //
flash unsigned char AT_K0[] = "AT&K0\r\n\0";       //
flash unsigned char AT_D0[] = "AT&D0\r\n\0";       //
flash unsigned char AT_W0[] = "AT&W0\r\n\0";       // Store the configuration as profile 0
flash unsigned char AT_Y0[] = "AT&Y0\r\n\0";       // Select profile 0 as the power-up default
//flash unsigned char AT_CLK[] = "AT+CCLK?\r\n\0";       //
flash unsigned char AT_CLK[] = "AT-MSSTM\r\n\0";       //
//flash unsigned char AT_SBDSX[] = "AT+SBDSX\r\n\0";  //
flash unsigned char AT_ECSQ[] = "AT+CSQ\r\n\0";       //get rssi
#ifdef DebugMode
flash unsigned char AT_CGSN[] = "AT+CGSN\r\n\0";       // Serial number
flash unsigned char AT_CGMR[] = "AT+CGMR\r\n\0";       // revision
#endif DebugMode

extern bit bCheckRxBuf;
extern bit bExtReset;
extern bit bWaitForModemAnswer;
extern bit bEndOfModemTask;
extern bit bNeedToWait4Answer;
extern bit bReset;
extern BYTE modemCurTask;
extern BYTE modemCurSubTask;
extern BYTE waitingTask;
extern BYTE dataSentOK;
extern BYTE toDoList;
extern BYTE ModemResponse;
extern unsigned int objToMsr;
extern BYTE rssi_val;
extern int BytesToSend;
extern int TimeLeftForWaiting;
extern unsigned int rx0_buff_len;
extern char e2_writeFlag;
extern char readClockBuf[];	         //buffer for data reading from clock
extern char DataBlock[];
extern char clockBuf[7]; 		 //buffer for all clock operation need
extern char ComBuf[MAX_SBD_BUF_LEN];
extern volatile char RxUart0Buf[MAX_RX_BUF_LEN];
extern char TxUart0Buf[];
extern int nMaxWaitingTime;
extern unsigned int pBread;		//pointer to last read sensor data block in ext_e2
extern int iVoltage;
extern int nTimeCnt;
int gTrnsID;
int gIndex;
extern unsigned char gUnreadValues;
extern unsigned int arrReadPointer[];
extern BYTE arrSendAlldata[];
//extern BYTE bCheckBatr;
extern BYTE nMaxFailuresNum;
extern BYTE failCnt;
extern BYTE initCnt;
extern BYTE bMakeReset;
BYTE nSndPckt;
extern BYTE bModemType;
extern BYTE bStlMdmStatus;
extern BYTE nSbdCntr;
//BYTE nRegFailureCntr;
//BYTE nSBDFailureCntr;
BYTE gReadType;
BYTE bMT_Status;
int MT_queued;
//extern BYTE nFailureCntr;
extern BYTE flagSucOrFail;
extern BYTE stlRegStatus;
extern BYTE strRegFailCnt;

extern eeprom char eLoggerID[]; //sensors id
extern eeprom unsigned char eStartConnectionH;      //first connection hour
extern eeprom int eTimeZoneOffset;                         //offset (in minutes) from UTC
//extern struct WirelessSensor WLSenArr[];
extern long SenIDArr[];

//send flash init string to modem
static void SendATCmd(flash unsigned char *bufToSend)
{
    int i;

    i = 0;
    //copy flash string to buff
    while ((bufToSend[i] != '\0') && (i < MAX_SBD_BUF_LEN))//cEndMark)
    {
         /*TxUart0Buf*/ComBuf[i] = bufToSend[i];
         i++;
    }
    BytesToSend = i ;

    //transmitt to local modem port
    TransmitBuf(0);
}

BYTE IsOKRes()
{
    int index = 0;
    while (index < rx0_buff_len-1)
    {
        if ((RxUart0Buf[index] == 'O') && (RxUart0Buf[index+1] == 'K'))
        {
//        #ifdef DebugMode
//        SendDebugMsg("\r\nOK found ");
//        #endif DebugMode
            return TRUE;
        }
        else
            index++;
    }
    #ifdef DebugMode
    SendDebugMsg("\r\nOK not found ");
    #endif DebugMode
    return FALSE;
}

int GetCheckSum( BYTE *buff, int length)
{
    int check_sum;

    check_sum = 0;
    while (length--)
    {
        check_sum += *buff++;
    }

    return (check_sum);
}

void SetSModemOn()
{
    // 5 volt on
    SATELITE_PWR_ENABLE();
    delay_ms(500);
    //power modem on
    //PORTC.2 = 1;
    SATELITE_PWR_ON();
    nTimeCnt = 50;    // wait 5 sec.
    initCnt++;
    ModemResponse = TASK_COMPLETE;
    #ifdef DebugMode
    SendDebugMsg("\r\nstlModemOn");
    #endif DebugMode
}

static void SetModemOff()
{
	//set power off
    SATELITE_PWR_OFF();
//    PORTD.7 = 0;
    delay_ms(10);
    ModemResponse = TASK_COMPLETE;
    #ifdef DebugMode
    SendDebugMsg("\r\nModemOff");
    #endif DebugMode
    // 5 volt off
    SATELITE_PWR_DISABLE();
}

void CheckNetworkSync()
{
    ModemResponse = TASK_FAILED;
    if (PINC.3 == 0)
    {
        ModemResponse = TASK_COMPLETE;
        #ifdef DebugMode
        SendDebugMsg("\r\nSync OK");
        #endif DebugMode
    }
    if (ModemResponse == TASK_FAILED)
        nTimeCnt = 100;    // in case of failure wait 10 sec.
    else
        nTimeCnt = 50;     // else - wait 5 sec
}

// build the string of data to send to IU
void BuildPcktStr()
{
    char b[4];
//    #ifdef DebugMode
//    SendDebugMsg("\r\nBuildPcktStr");
//    #endif DebugMode
    // ID
    Long2Bytes(SenIDArr[objToMsr].Id, b);
    PutString(&ComBuf[gIndex], b, 4);
    gIndex += 4;

//    //Position in logger
//    ComBuf[index++] = objToMst;     // set id of type

    //Type
    ComBuf[gIndex++] = GetSensorType();     // set id of type

    // interval
    ComBuf[gIndex++] = MSR_INTERVAL_DEV_10;//DataBlock[1]; //interval of block

    // num values in this buffer
    ComBuf[gIndex++] = gUnreadValues;

    // Timestamp
    PutString(&ComBuf[gIndex],&DataBlock[0], 5);
    gIndex += 5;
    // values:
    PutString(&ComBuf[gIndex],&DataBlock[5], gUnreadValues * 2);
    gIndex += gUnreadValues * 2;
}

void BuildPrmStr()
{
    BYTE cs;
    #ifdef DebugMode
    SendDebugMsg("\r\nBuildprmStr");
    #endif DebugMode
    //header
    ComBuf[0] = 0xFF;
    ComBuf[1] = 0xFF;
    ComBuf[2] = 0xFF;
    ComBuf[3] = 0xFF;

    gIndex = 4;

    // ID
    cpu_e2_to_MemCopy( &ComBuf[gIndex], &eLoggerID[0], 4);
    gIndex += 4;

    //time
    GetRealTime();
    ComBuf[gIndex++] = readClockBuf[0]; //year
    ComBuf[gIndex++] = readClockBuf[1]; //month
    ComBuf[gIndex++] = readClockBuf[2]; //day
    ComBuf[gIndex++] = readClockBuf[4]; //hour
    ComBuf[gIndex++] = readClockBuf[5]; //minute

    // connection Hour
    ComBuf[gIndex++] = eStartConnectionH;

    // Version
    cpu_flash_to_MemCopy(&ComBuf[gIndex], RomVersion, 4);
    gIndex += 4;

    //checksum
    cs = CheckSum(ComBuf, gIndex, 1);
    ComBuf[gIndex++] = cs;

    nSndPckt = 1;
}
// send AT+SBDWB=X, X is SBD length
void SendSBDWriteCmd(int len)
{
    int n;
    BYTE i;
    char s[4];
//    #ifdef DebugMode
//    SendDebugMsg("\r\nSendSBDWriteCmd ");
//    PrintNum(len);
//    #endif DebugMode
    //build the host address from url+port
    n = CopyFlashToBuf(TxUart0Buf/*ComBuf*/, AT_SBDWB);
    i = 0;
    do
    {
        s[i++] = (char)(len % 10);
        len = len / 10;
    }
    while (len > 0);

    for (; i > 0; i--)
       TxUart0Buf/*ComBuf*/[n++] = s[i-1] + 48;

    TxUart0Buf/*ComBuf*/[n++] = '\r';
    TxUart0Buf/*ComBuf*/[n++] = '\n';
    BytesToSend = n;
    TransmitBuf(0);
}

void BuildSBD()
{
    BYTE cs;
    unsigned int n;
    char b[2];
    BYTE overSize = FALSE;

    nSndPckt = 0;
    gTrnsID++;
    if (gTrnsID == MAX_INT)
        gTrnsID = 0;
    // reset read pointer array
    for (n = SENSOR1; n <= nMaxSenNum; n++)
        arrReadPointer[n] = MAX_ADDRESS; // set number that cant be real value;

    #ifdef DebugMode
    SendDebugMsg("\r\nBuildSBD");
    SendDebugMsg("\r\ntrsmition ID:");
    PrintNum(gTrnsID);
    #endif DebugMode

    gIndex = 0;
    // logger ID
    cpu_e2_to_MemCopy( &ComBuf[0], &eLoggerID[0], 4);
    gIndex += 4;
    // transmision ID
    int2bytes(gTrnsID, &ComBuf[gIndex]);
    gIndex += 2;

    //Battery
    ComBuf[gIndex++] = (unsigned char)((iVoltage >> 8) & 0xFF);     //address high
    ComBuf[gIndex++] = (unsigned char)(iVoltage) ;                 //address low

    //Time zone offset
    // because logger calc real time when get it from satelite, no need to send the offset.
    // so send 0 instead of it.
    // if, in any case, real time will be GMT, then need to send eTimeZoneOffset.
    int2bytes(eTimeZoneOffset, b);
    ComBuf[gIndex++] = b[1];    //( char)((eTimeZoneOffset >> 8) & 0xFF);     //address high
    ComBuf[gIndex++] = b[0];    //( char)(eTimeZoneOffset) ;                 //address low

    // skip over number of buffers byte/ add it at the end
    gIndex += 1;

    while ((objToMsr <= nMaxSenNum) && (overSize == FALSE))
    {
        if (GetMeasurments(gReadType) == TRUE)
        {
            // if adding the buffer won't cos deviation in buffer-to-sent size - add it
            if ((gIndex + (gUnreadValues * 2) + 13) < MAX_SBD_BUF_LEN)
            {
//                #ifdef DebugMode
//                SendDebugMsg("\r\nObjToMsr: ");
//                putchar1(objToMsr+0x30);
//                #endif DebugMode
                if (gUnreadValues > 0)
                {
                    // build post body
                    BuildPcktStr();
                    nSndPckt++;
                }
                if (IsMoreData() == FALSE)
                {
                    arrSendAlldata[objToMsr] = 1;
                    objToMsr++;
                    gReadType = 1;
                }
                else
                    gReadType = 2;
            }
            else
            {
                overSize = TRUE;
                gReadType = 1;  //3;  // not enough space in this SBD. read the packet again
            }
        }
        // if failed to get data - try next sensor
        else
        {
            objToMsr++;
            gReadType = 1;
        }
    }

    if (nSndPckt > 0)
    {
        ComBuf[10] = nSndPckt;
        cs = CheckSum(ComBuf, gIndex, 1);
        ComBuf[gIndex++] = cs;
    }
//    #ifdef DebugMode
//    for (i = 0; i < gIndex; i++)
//    {
//        putchar1(ComBuf[i]);
//        putchar1(',');
//    }
////    SendDebugMsg("\r\ngIndex: ");
////    PrintNum(gIndex);
//    #endif DebugMode
}

int Str2Int(char* s)
{
    int n = 0;
    int i = 0;

   while (s[i] != HASHTAG)
    {
        n*=10;
        n = n + (s[i] - 0x30);
        i++;
    }
    return n;
}

BYTE ParseServerRes()
{
    BYTE  i, tasks = 0, len, taskID, res = 0;  //n = 0,
//    int tmp;
    long l;
//    char buf[4];
    int index = 0, tz;

    #ifdef DebugMode
    SendDebugMsg("\r\nParse server result ");
    #endif DebugMode
    while ((index < rx0_buff_len-4) && (res == 0))
    {
        if ((RxUart0Buf[index] == eLoggerID[0]) &&
            (RxUart0Buf[index+1] == eLoggerID[1]) &&
            (RxUart0Buf[index+2] == eLoggerID[2]) &&
            (RxUart0Buf[index+3] == eLoggerID[3]))
            res = 1;
        else
            index++;
    }
    if (res == 0)
        return FALSE;
    index += 4;

    tasks = RxUart0Buf[index++];
    for (i = 0; i < tasks; i++)
    {
        taskID = RxUart0Buf[index++];
        len = RxUart0Buf[index++];  //PrmLen[taskID];
        switch (taskID)
        {
//            case 0:     //Set_Interval - make sure new interval is legal
//                SetNewInterval(RxUart0Buf[index]);
//                #ifdef DebugMode
//                SendDebugMsg("\r\nSet Interval");
//                putchar1(RxUart0Buf[index]);
//                #endif DebugMode
//                break;
            case UPDATE_DL_START:        //Start_DL   7
                if ((RxUart0Buf[index] >= 0) && (RxUart0Buf[index] <= 23))
                    eStartConnectionH = RxUart0Buf[index];
                #ifdef DebugMode
                SendDebugMsg("\r\nSet Start_DL");
                putchar1(eStartConnectionH);
                #endif DebugMode
            break;
            case UPDATE_COMMAND:       //reset memory (used to be GMT)
                if (RxUart0Buf[index] == COMMAND_INIT_MEMORY)
                    res = InitDataBlocks();
                else
                    if (RxUart0Buf[index] == COMMAND_MAKE_RESET)
                        bMakeReset = TRUE;
                    else
                        if (RxUart0Buf[index] == COMMAND_INIT_RESET)
                        {
                            res = InitDataBlocks();
                            bMakeReset = TRUE;
                        }
                        else
                            if (RxUart0Buf[index] == COMMAND_RTC_24)
                                SetRtc24Hour();
            break;
            case UPDATE_GMT:        //TIME ZONE OFFSET   11
                tz = RxUart0Buf[index];
                #ifdef DebugMode
                SendDebugMsg("\r\nSet TIME ZONE OFFSET");
                PrintNum(tz);
                #endif DebugMode
                tz <<= 8;
                tz += RxUart0Buf[index + 1];
                if ((tz >= -720) && (tz <= 720))
                    eTimeZoneOffset = tz;
                break;
                case UPDATE_ID:          // sensors ID   18
                     l = Bytes2Long(&RxUart0Buf[index]);
                    if (l > 0)
                    {
                        MemCopy_to_cpu_e2(&eLoggerID[0], &RxUart0Buf[index]  ,4);

                        // also make reset - so the new logger will register in db.
                        bMakeReset = TRUE;
                    }
                break;
                case UPDATE_EPOCH:        // update epoch    19
                    if (UpdateEpoch(&RxUart0Buf[index]) == FALSE)
                    {
                        #ifdef DebugMode
                        SendDebugMsg("\r\nSet new epoch failed");
                        #endif DebugMode
                    }
                break;
            default:
//                res = FALSE;
        }
        index += len;
    }
    if (tasks > 0)
        toDoList = DO_DATA_N_PRMS;
    return TRUE;
}

BYTE ParseRssi()
{
    BYTE i = 0, res = 0;
    #ifdef DebugMode
    SendDebugMsg("\r\nParseRssi ");
    #endif DebugMode

    while ((i < rx0_buff_len-1) && (res == 0))
    {
        if (RxUart0Buf[i++] == ':')
            res = 1;
    }
    if (res == 0)
        return FALSE;
    if (RxUart0Buf[i] != '0')
        return TRUE;
    #ifdef DebugMode
    SendDebugMsg("\r\nRSSI too low ");
    #endif DebugMode
    return FALSE;
}

//parse modem response. the answer structure is:
//+SBDIX: 0, 307, 1, 1, 10, 0
BYTE ParseSbdiRes()
{
    BYTE i = 0, res = 0, prm = 0;
    char num[6];
    int tmp, n= 0;

    bMT_Status = 0;
    //+SBDI: 1, 2173, 1, 87, 429, 0.
    #ifdef DebugMode
    SendDebugMsg("\r\nParseSbdiRes ");
    #endif DebugMode

    while ((i < rx0_buff_len-1) && (res == 0))
    {
        if (RxUart0Buf[i] == ':')
            res = 1;
        else
            i++;
    }
    if (res == 0)
        return FALSE;
    i += 2;
    prm = 1;
//        BYTE dotCntr = 0, n = 0, lastDotIndex = -1;
    while ((i < rx0_buff_len-1) && (prm <= 6))
    {
        if ((RxUart0Buf[i] >= '0') && (RxUart0Buf[i] <= '9'))
            num[n++] = RxUart0Buf[i++];
        else
        {
            if (n > 0)
            {
                num[n] = HASHTAG;
                tmp = Str2Int(num);
                switch (prm)
                {
                    case 1:
                        if (tmp == 1)
                        {
                            res = TRUE;
//                            if (waitingTask == SUB_TASK_MODEM_SND_DATA)
//                                nSbdCntr--;
                            #ifdef DebugMode
                            SendDebugMsg("\r\nSBDI OK ");
                            #endif DebugMode
                            // if all data of sensor was sent and SBDIX succeded - reset memory pointers of this sensor.
                            for (n = SENSOR1; n < nMaxSenNum; n++)
                                if (arrSendAlldata[n] == 1)
                                    ResetPointers(n, TRUE);
                        }
                        else                     //if ((tmp == 0) || (tmp == 2))
                            res = FALSE;
                    break;
//                    case 2:
//                        sbd->MTStatus = tmp;
//                    break;
                    case 3:
                        if (tmp == 1)
                            bMT_Status = 1;
                    break;
                    case 6:
                        MT_queued = tmp;
                        if (bMT_Status == 1)
                            MT_queued++;  // count the msg waiting at GSS + the one got from GSS
                    break;
                    default:
                        break;
                } //switch
                prm++;
                n = 0;
            }
            i++;
        } //else
    }
    return TRUE;
}

/*//THE FUNC GET string of 2 BYTES and convert them to decimal value in byte
BYTE Str2Byte(char* s)
{
    BYTE n = 0;

    n = s[0] - 0x30;
    n*=10;
    n += (s[1] - 0x30);

    return n;
}
*/
int GetRealNum(char c)
{
    if ((c >= '0') && (c <= '9'))
        return (c - 0x30);
    if ((c >= 'a') && (c <= 'f'))
        return (c - 0x57);
}

unsigned long HexStr2Long(char* str, int len)
{
    unsigned long l = 0, p = 1;
    int i, n, j;
    BYTE index;

    for (i = 0; i < len; i++)
    {
        // treat number from LSB
        index = len - i - 1;
        n = GetRealNum(str[index]);
        p = 1;
        for (j = 0; j < i; j++ )   // calc power of 16
            p *= 16;
        l += (n * p);
    }
    return l;
}

BYTE ParseSystemTime()
{
    BYTE i = 0, res = 0, j = 0;
    char num[9];
    unsigned long epoch;
    date_time_t system_date_time;

    while ((i < rx0_buff_len-17) && (res == 0))    //look for "no network service" result
    {
        if ((RxUart0Buf[i] == 'n') && (RxUart0Buf[i+1] == 'o') &&
            (RxUart0Buf[i+2] == ' ') && (RxUart0Buf[i+3] == 'n') &&
            (RxUart0Buf[i+4] == 'e') && (RxUart0Buf[i+5] == 't'))
            return TASK_FAILED;
        else
            i++;
    }
    i = 0;
    res = 0;
    //find first char of time (Max 8 char)
    while ((i <= rx0_buff_len-8) && (res == 0))
    {
        if (((RxUart0Buf[i] >= '0') && (RxUart0Buf[i] <= '9')) || ((RxUart0Buf[i] >= 'a') && (RxUart0Buf[i] <= 'f')))
            res = 1;
        else
            i++;
    }
    if (res == 0)   //"no network service" is the result
        return TASK_FAILED;
//    #ifdef DebugMode
//    SendDebugMsg("\r\ntime =\0");
//    #endif DebugMode
    while ((((RxUart0Buf[i] >= '0') && (RxUart0Buf[i] <= '9')) || ((RxUart0Buf[i] >= 'a') && (RxUart0Buf[i] <= 'f'))) && (j < 8))
    {
//    #ifdef DebugMode
//    putchar1(RxUart0Buf[i]);
//    #endif DebugMode
        num[j++] = RxUart0Buf[i++];
    }
 //   num[j] = '#';

   epoch = HexStr2Long(num, j);

//    epoch = Str2Long(num);
    ConvertEpoch2SysTime(&system_date_time, epoch);

    clockBuf[0] = system_date_time.year;
	clockBuf[1] = system_date_time.month;
	clockBuf[2] = system_date_time.day;
	clockBuf[4] = system_date_time.hour;
	clockBuf[5] = system_date_time.minute;
	clockBuf[6] = system_date_time.second;
    #ifdef DebugMode
    SendDebugMsg("\r\nsystem_date_time:\0");
    PrintNum(clockBuf[0]);
    PrintNum(clockBuf[1]);
    PrintNum(clockBuf[2]);
    PrintNum(clockBuf[4]);
    PrintNum(clockBuf[5]);
//    SendDebugMsg("\r\n\0");
    #endif DebugMode

    if(SetRealTime() == FAILURE)
    {
        #ifdef DebugMode
        SendDebugMsg("\r\nfailed to set time ");
        #endif DebugMode
    }
    return TASK_COMPLETE;
}

/*void ParseDateTime()
{
    BYTE n, i = 0, res = 0;
    char num[2];

    while ((i < rx0_buff_len-1) && (res == 0))
    {
        if (RxUart0Buf[i] == ':')
            res = 1;
        i++;
    }
    if (res == 0)
        return;
    for (n = 0; n < 7; n++)
    {
        if (n != 3)
        {
            num[0] = RxUart0Buf[i];
            num[1] = RxUart0Buf[i+1];
            clockBuf[n] = Str2Byte(num);
            i += 3;
        }
    }
    if(SetRealTime() == FAILURE)
    {
        #ifdef DebugMode
        SendDebugMsg("\r\nfailed to set time ");
        #endif DebugMode
    }
}
*/

static void ParseModemResponse()
{
    BYTE res = 0, index = 0;

    switch (modemCurSubTask)
    {
        case SUB_TASK_INIT_MODEM_OK:
//        case SUB_TASK_MODEM_CLOSE_MDM:
        case SUB_TASK_MODEM_INIT_E:
        case SUB_TASK_MODEM_INIT_K:
        case SUB_TASK_MODEM_INIT_D:
        case SUB_TASK_MODEM_INIT_W:
        case SUB_TASK_MODEM_INIT_Y:
        case SUB_TASK_SBD_MTA:
        case SUB_TASK_SBD_CLR_BUF:
        case SUB_TASK_CLOSE_PWR_DOWN:
        #ifdef DebugMode
        case SUB_TASK_MODEM_INIT_CGSN:
        case SUB_TASK_MODEM_INIT_CGMR:
        #endif DebugMode
            if (IsOKRes() == TRUE)
                ModemResponse = TASK_COMPLETE;
            else
                ModemResponse = TASK_FAILED;
        break;
        case SUB_TASK_REG_CSQ:
            if (ParseRssi() == TRUE)
                ModemResponse = TASK_COMPLETE;
            else
                ModemResponse = TASK_FAILED;
        break;
        case SUB_TASK_REG_QES:
        case SUB_TASK_REG:
            if (IsOKRes() == TRUE)
            {
                while ((index < rx0_buff_len) && (res == 0))
                {
                    if ((RxUart0Buf[index] == ':') && (RxUart0Buf[index+1] == '2'))
                        res = 1;
                    else
                        index++;
                }
                if (res == 1)
                {
                    ModemResponse = TASK_COMPLETE;
                    stlRegStatus = REG_OK;
                    strRegFailCnt = 0;
                }
                else
                {
                    ModemResponse = TASK_FAILED;
                    stlRegStatus = REG_FAILED;
                    strRegFailCnt++;
                }
            }
            else
                ModemResponse = TASK_FAILED;
        break;

        case SUB_TASK_SBD_SND:
            while ((index < rx0_buff_len-4) && (res == 0))
            {
                if ((RxUart0Buf[index] == 'R') && (RxUart0Buf[index+1] == 'E') &&
                    (RxUart0Buf[index+2] == 'A') && (RxUart0Buf[index+3] == 'D') &&
                    (RxUart0Buf[index+4] == 'Y'))
                    res = 1;
                else
                    index++;
            }
            if (res == 1)
            {
                ModemResponse = TASK_COMPLETE;
//                #ifdef DebugMode
//                SendDebugMsg("\r\nready found!");
//                #endif DebugMode
            }
            else
            {
                ModemResponse = TASK_FAILED;
                #ifdef DebugMode
                SendDebugMsg("\r\nready not found");
                #endif DebugMode
            }
        break;
        case SUB_TASK_SBD_SND_DATA:
//            #ifdef DebugMode
//            SendDebugMsg("\r\nrx0_buff_len after send data:");
//            PrintNum(rx0_buff_len);
//            #endif DebugMode
            while ((index < rx0_buff_len-1) && (res == 0))
            {
                if (RxUart0Buf[index] == '0')
                    res = 1;
                else
                    index++;
            }
            if (res == 1)
                ModemResponse = TASK_COMPLETE;
            else
                ModemResponse = TASK_FAILED;
        break;

        case SUB_TASK_SBD_INIT:
//            #ifdef DebugMode
//            SendDebugMsg("\r\nrx0_buff_len= ");
//            PrintNum(rx0_buff_len);
//            #endif DebugMode
            if (IsOKRes() == TRUE)
            {
                if (ParseSbdiRes() == TRUE)
                {
                    ModemResponse = TASK_COMPLETE;
                }
                else
                    ModemResponse = TASK_FAILED;
            }
            else
                ModemResponse = TASK_FAILED;
        break;
        case SUB_TASK_SBD_READ:
            if (ParseServerRes() == TRUE)
                 ModemResponse = TASK_COMPLETE;
            else
                 ModemResponse = TASK_FAILED;

        break;
        case TASK_MODEM_CLK:
            if (IsOKRes() == TRUE)
            {
                ModemResponse = ParseSystemTime();
                  //ParseDateTime();
            }
            else
                ModemResponse = TASK_FAILED;
        break;
    }
}

static BYTE GetNextTask()
{
    // first task-
    if (modemCurTask == TASK_NONE)
    {
        modemCurTask = TASK_MODEM_INIT;
        modemCurSubTask = SUB_TASK_INIT_MODEM_ON;   //SUB_TASK_INIT_5V_ON;//;
        initCnt = 0;
        return  CONTINUE;
    }
    if ((bWaitForModemAnswer == TRUE) && (TimeLeftForWaiting == 0))
    {
        #ifdef DebugMode
        SendDebugMsg("\r\ntime out ");
        #endif DebugMode
        bWaitForModemAnswer = FALSE;
        ModemResponse = TASK_FAILED;
    }
    // if flag of end of rx received is on
    if (bCheckRxBuf == TRUE)
    {
        bCheckRxBuf = FALSE;
        ParseModemResponse();
        delay_ms(500);
    }
    if (nTimeCnt > 0)
        return WAIT;

    switch (ModemResponse)
    {
        case NO_ANSWER:
            return WAIT;
        case TASK_COMPLETE:
        {
            switch (modemCurTask)
            {
                case TASK_MODEM_INIT:
                    switch (modemCurSubTask)
                    {
                        case SUB_TASK_INIT_5V_ON:
                            modemCurSubTask = SUB_TASK_INIT_MODEM_ON;
                        break;
                        case SUB_TASK_INIT_MODEM_ON:
                            modemCurSubTask = SUB_TASK_INIT_MODEM_OK;//SUB_TASK_INIT_MODEM_IGN;
                        break;
                        case SUB_TASK_INIT_MODEM_OK:
//                            TurnOnLed(LED_1, SUCCESS);
                            modemCurSubTask = SUB_TASK_MODEM_INIT_NET_SYNC;
                        break;
                        case SUB_TASK_MODEM_INIT_NET_SYNC:
                            if (bExtReset)
                            #ifdef DebugMode
                                modemCurSubTask = SUB_TASK_MODEM_INIT_CGSN;
                            #else
                                modemCurSubTask = SUB_TASK_MODEM_INIT_E;
                            #endif DebugMode
                            else
                            {
                                modemCurTask = TASK_MODEM_NET_REG;
                                modemCurSubTask = SUB_TASK_REG_CSQ;
                            }
                        break;
                        #ifdef DebugMode
                        case SUB_TASK_MODEM_INIT_CGSN:
                            modemCurSubTask = SUB_TASK_MODEM_INIT_CGMR;
                        break;
                        case SUB_TASK_MODEM_INIT_CGMR:
                            modemCurSubTask = SUB_TASK_MODEM_INIT_E;
                        break;
                        #endif DebugMode
                        case SUB_TASK_MODEM_INIT_E:
                            modemCurSubTask = SUB_TASK_MODEM_INIT_K;
                        break;
                        case SUB_TASK_MODEM_INIT_K:
                            modemCurSubTask = SUB_TASK_MODEM_INIT_D;
                        break;
                        case SUB_TASK_MODEM_INIT_D:
                            modemCurSubTask = SUB_TASK_MODEM_INIT_W;
                        break;
                        case SUB_TASK_MODEM_INIT_W:
                            modemCurSubTask = SUB_TASK_MODEM_INIT_Y;
                        break;
                        case SUB_TASK_MODEM_INIT_Y:
                            modemCurTask = TASK_MODEM_NET_REG;
                            modemCurSubTask = SUB_TASK_REG_CSQ;
                        break;
                    }
                break;
                case TASK_MODEM_NET_REG:
                    switch (modemCurSubTask)
                    {
                        case SUB_TASK_REG_CSQ:
                            modemCurSubTask = TASK_MODEM_CLK;
                        break;
                        case TASK_MODEM_CLK:
                            modemCurSubTask = SUB_TASK_REG_QES;
                        break;
                        case SUB_TASK_REG_QES:
                        case SUB_TASK_REG:
                            TurnOnLed(LED_2, SUCCESS);
                            modemCurTask = TASK_MODEM_SBD;
                            modemCurSubTask = SUB_TASK_SBD_MTA;
                        break;
                    }
                break;
                case TASK_MODEM_SBD:
                    switch  (modemCurSubTask)
                    {
                        case SUB_TASK_SBD_MTA:
                            modemCurSubTask = SUB_TASK_SBD_CLR_BUF;
                        break;
                        case SUB_TASK_SBD_CLR_BUF:
                            modemCurSubTask = SUB_TASK_SBD_BUILD;
                        break;
                        case SUB_TASK_SBD_BUILD:
                            if (nSndPckt > 0)
                                modemCurSubTask = SUB_TASK_SBD_SND;
                            else
                            {
//                                modemCurTask = TASK_MODEM_CLOSE;
//                                modemCurSubTask = SUB_TASK_CLOSE_PWR_DOWN;
                                if ((waitingTask == SUB_TASK_MODEM_SND_DATA) && (toDoList == DO_DATA_N_PRMS))
                                {
                                    waitingTask = SUB_TASK_MODEM_SND_PRM;
                                    modemCurSubTask = SUB_TASK_SBD_CLR_BUF;
                                    flagSucOrFail = SUCCESS;
                                }
                                else
                                {
                                    modemCurTask = TASK_MODEM_CLOSE;
                                    modemCurSubTask = SUB_TASK_CLOSE_PWR_DOWN;
                                }
//                                    modemCurSubTask = TASK_MODEM_CLK;
                            }
                        break;
                        case SUB_TASK_SBD_SND:
                            modemCurSubTask = SUB_TASK_SBD_SND_DATA;
                        break;
                        case SUB_TASK_SBD_SND_DATA:
                            modemCurSubTask = SUB_TASK_SBD_INIT;
                        break;
                        case SUB_TASK_SBD_INIT:
                        //if there is msg to read - first of all read it
                            if (bMT_Status == 1)
                                modemCurSubTask = SUB_TASK_SBD_READ;
                            else
                                //if sending data now
                                if (waitingTask == SUB_TASK_MODEM_SND_DATA)
                                    if (objToMsr <= nMaxSenNum)
                                        modemCurSubTask = SUB_TASK_SBD_CLR_BUF;
                                    else
                                    {
                                    // sign all data sent OK
//                                        dataSentOK = TRUE;
                                        //if (nSbdCntr == 0)
                                        flagSucOrFail = SUCCESS;
                                        if (toDoList == DO_DATA_N_PRMS)
                                        {
                                            waitingTask = SUB_TASK_MODEM_SND_PRM;
                                            modemCurSubTask = SUB_TASK_SBD_CLR_BUF;
                                        }
                                        else
                                        {
                                            modemCurTask = TASK_MODEM_CLOSE;
                                            modemCurSubTask = SUB_TASK_CLOSE_PWR_DOWN;
                                         //   modemCurSubTask = TASK_MODEM_CLK;
                                        }
                                    }
                                else
                                {
                                    flagSucOrFail = SUCCESS;    // if finished snd params ok
                                    modemCurTask = TASK_MODEM_CLOSE;
                                    modemCurSubTask = SUB_TASK_CLOSE_PWR_DOWN;
                                }
                        break;
                        case SUB_TASK_SBD_READ:
                            //IF NO MORE DATA TO READ
                            if (MT_queued == 0)
                            {
                                if (waitingTask == SUB_TASK_MODEM_SND_DATA)
                                {
                                    // if more data to send
                                    if (objToMsr <= nMaxSenNum)
                                        modemCurSubTask = SUB_TASK_SBD_CLR_BUF;
                                    else
                                    {
                                        flagSucOrFail = SUCCESS;
                                        if  (toDoList == DO_DATA_N_PRMS)
                                        {
                                            waitingTask = SUB_TASK_MODEM_SND_PRM;
                                            modemCurSubTask = SUB_TASK_SBD_CLR_BUF;
                                        }
                                        else
                                        {
                                            modemCurTask = TASK_MODEM_CLOSE;
                                            modemCurSubTask = SUB_TASK_CLOSE_PWR_DOWN;
                                        }
                                    }
                                }
                                else
                                {
                                    modemCurTask = TASK_MODEM_CLOSE;
                                    modemCurSubTask = SUB_TASK_CLOSE_PWR_DOWN;
                                }
                            }
                        break;
                    }
                break;
                case TASK_MODEM_CLOSE:
                    switch (modemCurSubTask)
                    {
                        case SUB_TASK_CLOSE_PWR_DOWN:
                            modemCurSubTask = SUB_TASK_CLOSE_MODEM_OFF;
                        break;
//not relevant - will never reach here
//                        case SUB_TASK_CLOSE_MODEM_OFF:
//                            if ((flagSucOrFail == SUCCESS) || (nFailureCntr >= 2))    // if nothing failed or failed more than once - do not continue
//                            {
//                                modemCurTask = TASK_NONE;
//                                modemCurSubTask = TASK_NONE;
//                            }
//                            else
//                            {
//                                modemCurTask = TASK_MODEM_INIT;
//                                modemCurSubTask = SUB_TASK_INIT_MODEM_ON;
//                            }
//                        break;
                    }
                break;
            }
            failCnt = 0;
            return CONTINUE;
        }
        case TASK_FAILED:
        {
            failCnt++;    // count num of failures
            #ifdef DebugMode
            putchar1(failCnt + 0x30);
            #endif DebugMode
            // if failed more than 2 times - quit
            if (failCnt >= nMaxFailuresNum)
            {
                switch (modemCurTask)
                {
                    case TASK_MODEM_INIT:
                        switch (modemCurSubTask)
                        {
                            case SUB_TASK_INIT_MODEM_OK:
//                                if (!bExtReset)
//                                    TurnOnLed(LED_1, FAILURE);
                                bStlMdmStatus = SATELLITE_MODEM_NOT_CONNECTED;
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_CLOSE_MODEM_OFF;
                            break;
                            case SUB_TASK_MODEM_INIT_NET_SYNC:
//                                modemCurSubTask = SUB_TASK_MODEM_INIT_K;
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_CLOSE_MODEM_OFF;
                            break;
                            #ifdef DebugMode
                            case SUB_TASK_MODEM_INIT_CGSN:
                                modemCurSubTask = SUB_TASK_MODEM_INIT_CGMR;
                            break;
                            case SUB_TASK_MODEM_INIT_CGMR:
                                modemCurSubTask = SUB_TASK_MODEM_INIT_E;
                            break;
                            #endif DebugMode
                            case SUB_TASK_MODEM_INIT_E:
                                modemCurSubTask = SUB_TASK_MODEM_INIT_K;
                            break;
                            case SUB_TASK_MODEM_INIT_K:
                                modemCurSubTask = SUB_TASK_MODEM_INIT_D;
                            break;
                            case SUB_TASK_MODEM_INIT_D:
                                modemCurSubTask = SUB_TASK_MODEM_INIT_W;
                            break;
                            case SUB_TASK_MODEM_INIT_W:
                                modemCurSubTask = SUB_TASK_MODEM_INIT_Y;
                            break;
                            case SUB_TASK_MODEM_INIT_Y:
                                modemCurTask = TASK_MODEM_NET_REG;
                                modemCurSubTask = SUB_TASK_REG_CSQ;
                            break;
                        }
                    break;
                    case TASK_MODEM_NET_REG:
                        switch (modemCurSubTask)
                        {
                            case SUB_TASK_REG_QES:
                                modemCurSubTask = SUB_TASK_REG;
                            break;
                            case TASK_MODEM_CLK:
                            case SUB_TASK_REG_CSQ:
                            case SUB_TASK_REG:
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_CLOSE_PWR_DOWN;
                                //TurnOnLed(LED_2, FAILURE);
                            break;
//                                modemCurTask = TASK_MODEM_CLOSE;
//                                modemCurSubTask = SUB_TASK_CLOSE_PWR_DOWN;
//                                // if its first failure - update status to failure
////                                if ((stlRegStatus == INITIAL_STATE) || (stlRegStatus == REG_OK))
////                                {
////                                    stlRegStatus = REG_FAILED;
////                                    strRegFailCnt = 1;
////                                }
////                                    //else - update num of failures
////                                else
////                                    strRegFailCnt++;
//                                TurnOnLed(LED_2, FAILURE);
//                            break;
                        }
                    break;
                    case TASK_MODEM_SBD:
                        switch  (modemCurSubTask)
                        {
                            case SUB_TASK_SBD_MTA:
                                modemCurSubTask = SUB_TASK_SBD_CLR_BUF;
                            break;
                            case SUB_TASK_SBD_CLR_BUF:
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_CLOSE_PWR_DOWN;
//                                TurnOnLed(LED_3, FAILURE);
//                                flagSucOrFail = FAILURE;
                            break;
                            case SUB_TASK_SBD_SND:
                            case SUB_TASK_SBD_SND_DATA:
                            case SUB_TASK_SBD_INIT:
                                if (waitingTask == SUB_TASK_MODEM_SND_DATA)
                                    ResetAllReadPointers();
                                //if there is bsd msg in isu waiting to b read - get it
                                if (bMT_Status == 1)
                                    modemCurSubTask = SUB_TASK_SBD_READ;
                                else
                                {
                                    modemCurTask = TASK_MODEM_CLOSE;
                                    modemCurSubTask = SUB_TASK_CLOSE_PWR_DOWN;
//                                    TurnOnLed(LED_3, FAILURE);
                                }
                            break;
                            case SUB_TASK_SBD_READ:     // todo - check if worth reconnect cos data may be lost anyway
                                if (/*(flagSucOrFail == SUCCESS) && */(objToMsr <= nMaxSenNum))  // if till now everything succeeded continue send data
                                        modemCurSubTask = SUB_TASK_SBD_CLR_BUF;
                                else
                                {
                                    modemCurTask = TASK_MODEM_CLOSE;
                                    modemCurSubTask = SUB_TASK_CLOSE_PWR_DOWN;
//                                    TurnOnLed(LED_3, FAILURE);
                                }
                            break;
                        }
                    break;
                    case TASK_MODEM_CLOSE:
                        switch (modemCurSubTask)
                        {
                            case SUB_TASK_CLOSE_PWR_DOWN:
//                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_CLOSE_MODEM_OFF;
                            break;
                            case SUB_TASK_CLOSE_MODEM_OFF:
                                modemCurTask = TASK_NONE;
                                modemCurSubTask = TASK_NONE;
                            break;
                        }
                    break;
                }
                failCnt = 0;
            }
            else
                if ((modemCurSubTask == SUB_TASK_REG_CSQ) || (modemCurSubTask ==  TASK_MODEM_CLK))
                {
                    nTimeCnt = 100;
                    #ifdef DebugMode
                    SendDebugMsg("\r\nwait 10 sec");
                    #endif DebugMode
                    ModemResponse = TASK_HOLD;
                    return WAIT;
                }
        }
        case TASK_HOLD:
        break;
        return CONTINUE;
        break;
    }
}

void StlModemMain()
{
    BYTE res;//, i;
    int cs;

    res = GetNextTask();
    if (res == WAIT)
        return;

    switch (modemCurTask)
    {
        case TASK_MODEM_INIT:
            switch (modemCurSubTask)
            {
                case SUB_TASK_INIT_MODEM_ON:
                    SetSModemOn();
                break;
                case SUB_TASK_INIT_MODEM_OK:
                    nMaxWaitingTime = 20;   // wait max 2 sec for answer
                    nMaxFailuresNum = 2;
                    bNeedToWait4Answer = TRUE;
                    //ask modem if OK
                    SendATCmd(AT_OK);
                break;
                case SUB_TASK_MODEM_INIT_NET_SYNC:
                    bStlMdmStatus = SATELLITE_MODEM_CONNECTED;  // if got OK from satelite modem, means its exist
                    nMaxFailuresNum = 18;   // @@@ 18
                    CheckNetworkSync();
                break;
                #ifdef DebugMode
                case SUB_TASK_MODEM_INIT_CGSN:
                    nMaxFailuresNum = 1;
                    SendATCmd(AT_CGSN);
                break;
                case SUB_TASK_MODEM_INIT_CGMR:
                    nMaxFailuresNum = 1;
                    SendATCmd(AT_CGMR);
                break;
                #endif DebugMode
                case SUB_TASK_MODEM_INIT_E:
                    SendATCmd(AT_E0);
                break;
                case SUB_TASK_MODEM_INIT_K:
                    SendATCmd(AT_K0);
                break;
                case SUB_TASK_MODEM_INIT_D:
                    SendATCmd(AT_D0);
                break;
                case SUB_TASK_MODEM_INIT_W:
                    SendATCmd(AT_W0);
                break;
                case SUB_TASK_MODEM_INIT_Y:
                    SendATCmd(AT_Y0);
                break;
            }
        break;
        case TASK_MODEM_NET_REG:
            switch (modemCurSubTask)
            {
                case SUB_TASK_REG_CSQ:
                    nMaxFailuresNum = 10;
                    nMaxWaitingTime = 100;  /// wait max 10 sec for answer
                    SendATCmd(AT_ECSQ);
                break;
                case TASK_MODEM_CLK:
                    nMaxWaitingTime = 20;  // wait max 2 sec for answer
                    SendATCmd(AT_CLK);
                break;
                case SUB_TASK_REG_QES:
                    nMaxFailuresNum = 1;
                    nMaxWaitingTime = 300;  // wait max 30 sec for answer
                    objToMsr = SENSOR1;
                    gReadType = 1;
                    SendATCmd(AT_SBDREGASK);
                break;
                case SUB_TASK_REG:
                    nMaxFailuresNum = 1;
                    SendATCmd(AT_SBDREG);
                break;
            }
        break;
        case TASK_MODEM_SBD:
            switch (modemCurSubTask)
            {
                case SUB_TASK_SBD_MTA:
                    nMaxFailuresNum = 1;
                    nMaxWaitingTime = 20;      // wait 2 sec
                    SendATCmd(AT_SBDMTA);
                break;
                case SUB_TASK_SBD_CLR_BUF:
                    nMaxFailuresNum = 2;
                    nMaxWaitingTime = 300;      // wait 30 sec
                    SendATCmd(AT_SBDD0);
                break;
                case SUB_TASK_SBD_BUILD:
                    if (waitingTask == SUB_TASK_MODEM_SND_DATA)
                        BuildSBD();
                    else
                        BuildPrmStr();
                    ModemResponse = TASK_COMPLETE;
                break;
                case SUB_TASK_SBD_SND:
                    nMaxFailuresNum = 1;
                    nMaxWaitingTime = 20;   // wait 2 sec
                    SendSBDWriteCmd(gIndex);
                break;
                case SUB_TASK_SBD_SND_DATA:
                    #ifdef DebugMode
                    putchar1(HASHTAG);
                    #endif DebugMode
                    nMaxWaitingTime = 300;      // wait 30 sec
                    //add check sum at the end of buffer
                    cs = GetCheckSum(ComBuf, gIndex);
                    ComBuf[gIndex++] = (unsigned char)((cs >> 8) & 0xFF);     //address high
                    ComBuf[gIndex++] = (unsigned char)(cs) ;                 //address low
                    BytesToSend = gIndex;
                    TransmitBuf(0);
                break;
                case SUB_TASK_SBD_INIT:
                    nMaxWaitingTime = 500;      // wait 50 sec
                    if (waitingTask == SUB_TASK_MODEM_SND_DATA)
                        nSbdCntr++;
                    SendATCmd(AT_SBDI);
                break;
                case SUB_TASK_SBD_READ:
                    MT_queued--;
                    nMaxWaitingTime = 100;      // wait 5 sec
                    SendATCmd(AT_SBDRB);
                break;
            }
        break;

        case TASK_MODEM_CLOSE:
            nMaxFailuresNum = 2;
            nMaxWaitingTime = 20;  // wait max 2 sec for response
            switch (modemCurSubTask)
            {
                case SUB_TASK_CLOSE_PWR_DOWN:
                    SendATCmd(AT_PWR_DWN);
                break;
                case SUB_TASK_CLOSE_MODEM_OFF:
                    nMaxFailuresNum = 1;
                    SetModemOff();
                    if (flagSucOrFail == FAILURE)
                    {
                        nFailureCntr++;
                        #ifdef DebugMode
                        SendDebugMsg("\r\nnFailureCntr++");
                        #endif DebugMode
                    }
                    else
                        TurnOnLed(LED_3, LED_ON);
                    bEndOfModemTask = TRUE;
                    //if sensor got task of reset sensor - do it now
                    if (bMakeReset == TRUE)
                    {
                        bReset = TRUE;
                        bMakeReset = FALSE;
                    }
                break;
            }
//        break;
        default:
    }
}

#endif SATELLITE_MODEM_MANAGER_C
#endif Satellite