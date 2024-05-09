#ifndef MODEM_MANAGER_C
#define MODEM_MANAGER_C
//#include "define.h"
#include "utils.h"
#include "protocol.h"
#include "Valve_Manager.h"
#include "cbrd1000v0.h"
#include "Rtc_Manager.h"
#include "data_manager.h"
#include "modem_manager.h"
#include "interrupts.h"
#include "HW_manager.h"
#include "Pump_manager.h"
#include "RS485_Manager.h"

//#define ISRAEL_QUICK_CONNECT
//#define SMART_SERVER

#define RAN_3G  1
#define RAN_4G  2

#define CMD_LEN 13

#define PRM_ANSWER_LEN  (MAX_PRM_TASKS + 5 + 6) //phy111+timestamp+parameters command size

#define TASK_MODEM_INIT         1
#define TASK_MODEM_CONNECT      3
#define TASK_MODEM_POST         4
#define TASK_MODEM_CLOSE        5
#define SUB_TASK_INIT_MODEM_OK  12

#define SUB_TASK_INIT_MODEM_REG     13
#define SUB_TASK_INIT_MODEM_RSSI    14
#define SUB_TASK_INIT_MODEM_COPS    15
#define SUB_TASK_MODEM_CHK_ICCID    16
#define SUB_TASK_INIT_MODEM_GET_COPS    18
#define SUB_TASK_INIT_MODEM_DELAY   19
#define SUB_TASK_INIT_MODEM_MONITOR 20

#define SUB_TASK_INIT_MODEM_COPS_MAN    24
#define SUB_TASK_INIT_MODEM_REG_STAT    25
//#define SUB_TASK_INIT_MODEM_HW_SHDN     26
#define SUB_TASK_DELAY                  27
#define SUB_TASK_INIT_MODEM_QSS         28
#define SUB_TASK_INIT_MODEM_REG_LTE     29
#define SUB_TASK_INIT_MODEM_COPS_4_MONITOR  22  

#define SUB_TASK_MODEM_CONNECT_ATCH     31
//#define SUB_TASK_MODEM_CONNECT_SETUP1   32
//#define SUB_TASK_MODEM_CONNECT_SETUP2   33
//#define SUB_TASK_MODEM_CONNECT_SETUP3   34
#define SUB_TASK_MODEM_CONNECT_IS_ACTIVE    34
#define SUB_TASK_MODEM_CONNECT_PDP_DEF      35
#define SUB_TASK_MODEM_CONNECT_ACTV         36
#define SUB_TASK_MODEM_CONNECT_START_DIAL   37
#define SUB_TASK_MODEM_CONNECT_DEACTV       38
#define SUB_TASK_MODEM_CONNECT_DELAY        39

#define SUB_TASK_MODEM_POST_PRM     41
#define SUB_TASK_MODEM_POST_CNFRM   44
#define SUB_TASK_MODEM_POST_LOCATION  45
#define SUB_TASK_MODEM_POST_ALERT   71
#define SUB_TASK_MODEM_POST_CBU     72
#define SUB_TASK_MODEM_POST_DATA    73
#define SUB_TASK_MODEM_POST_PUMP    74
#define SUB_TASK_MODEM_POST_VLV     47
#define SUB_TASK_MODEM_POST_CBU_META     48
#define SUB_TASK_MODEM_POST_VCU_LST 49

#define SUB_TASK_MODEM_CLOSE_EOD    51
#define SUB_TASK_MODEM_CLOSE_TCP    52
#define SUB_TASK_MODEM_CLOSE_MDM    53
#define SUB_TASK_MODEM_OFF          54
#define SUB_TASK_MODEM_IGN_ON       55
#define SUB_TASK_MODEM_IGN_OFF      56
#define SUB_TASK_MODEM_EXIT         57
//#define SUB_TASK_MODEM_CGMM         58

#define DO_DATA     0x40
#define DO_PARAMS   0x20
#define DO_VALVES   0x10

#define UPDATE_URL      1
#define UPDATE_PORT     2
#define UPDATE_APN      3
#define UPDATE_PSW      4
#define UPDATE_DL_START  7
#define UPDATE_DL_CYCLE  8
#define UPDATE_DL_INTERVAL  9
#define UPDATE_COMMAND  10
#define UPDATE_GMT      11
//#define UPDATE_LIMITS   12
//#define UPDATE_ARMED_LIMITS  13
#define UPDATE_USE_CC   14
#define UPDATE_ROAMING_DLY  15
#define UPDATE_MNC      16
#define UPDATE_MCC      17
#define UPDATE_ID       18
#define UPDATE_ENABLE_TMP_ALERT 19
#define UPDATE_TMP_ALERT_LMT    20
#define UPDATE_SOFTWARE_VER     21
#define UPDATE_VALVE_CMD        22
#define ENABLE_VALVE_CMD        23
#define UPDATE_VLV_LST          25
#define UPDATE_CBU_CNFG         26
#define UPDATE_CBU_CNFG2        27
#define MAX_PRM_TASKS           28

#define GET_UPDATE      1
#define CONFIRM_UPDATE  0

#define COMMAND_INIT_MEMORY     1
#define COMMAND_MAKE_RESET      2
#define COMMAND_INIT_RESET      3
#define COMMAND_RTC_24          4
#define COMMAND_INIT_MEM_SENSORS    5
#define COMMAND_GET_VLV_LST     6
#define COMMAND_OPEN_PUMP       7

#define ALL_VLV_COMMAND_PING    99
#define ALL_VLV_COMMAND_RST     98
#define ALL_VLV_COMMAND_TEST    97
#define ALL_VLV_COMMAND_STOP    94

#define MAX_BAUD_RATE_OPTIONS  7
#define SEC_4_GSM_IGNITION  1

typedef struct 
{
    unsigned short versionUpdate;
} _tagBLEEPROM;

bit bWaitForModemAnswer;
bit longAnswerExpected;
bit bEndOfModemTask;
bit bNeedToWait4Answer;
extern bit bEndOfMonitorTask;
BYTE bConnectOK;
BYTE findOK;
BYTE nFw2Upg;
extern BYTE btrStatus;
BYTE g_bModemConnect = 0;
extern bit overFlow = 0;
BYTE modemCurTask;
BYTE modemCurSubTask;
static BYTE post2send;
BYTE g_RAN;
BYTE g_bSendCBUMeta = FALSE;
BYTE g_bSendList = FALSE;
BYTE nErrorOnConnect;
//BYTE waitingTask;
//BYTE dataSentOK;
BYTE prmSentOK;
BYTE vlvSentOK = FALSE;
BYTE bSendPrms;
extern int BytesToSend;
BYTE ModemResponse;
//extern unsigned int objToMsr;
extern bit bReset;
extern unsigned int rx0_buff_len;
BYTE rssi_val;
BYTE UpdatePrmArr[MAX_PRM_TASKS];
extern DateTime g_LastCnctTime;
extern char DataBlock[];
extern char clockBuf[7]; 		 //buffer for all clock operation need
extern char ComBuf[MAX_RX1_BUF_LEN];
int nMaxWaitingTime;
extern int iVoltage;
extern int nTimeCnt;
extern volatile BYTE prevMainTask;
extern volatile BYTE g_bExtIntDtct;
#ifdef UseGPS
extern float g_fLat;
extern float g_fLon;
#endif UseGPS
BYTE fSwUpdate;
unsigned long OprtTbl[10] ;
BYTE AccessTech[10] ;
eeprom _tagBLEEPROM BlEepromData @0x0400;
eeprom _tagBLEEPROM BlEepromData = {0};
eeprom _tagPortDefEEPROM ComponentArray[MAX_PORTS_CBU] @0x300;
eeprom _tagFlowDefEEPROM FlowDef[2] @0x200;
extern volatile BYTE mainTask;
extern BYTE g_nTime2StartAT;

#ifdef SMART_SERVER  
char tmpBuf[45];
#endif SMART_SERVER  
bit bUpdateAddress;
char newURL[32];          
char newPORT[4];
char cEndMark = '\0';
BYTE nMaxFailuresNum;
BYTE prmUpdtIndex;
int  bufIndexToUpd;
BYTE failCnt;
BYTE initCnt;
BYTE bPostAnswered;
BYTE bMakeReset;
BYTE bTestPump;
BYTE nValveCmdCnt;
BYTE bIsMoreVlvCmd;
BYTE fGetCopsLst;
BYTE PrmLen[MAX_PRM_TASKS] = {1,32,4,32,6,32,32,1,1,1,1,0,8,8,1,1,4,4,4,1,2,2,0,1,0,200,150};    //task 11 not exist. skip the number
unsigned char ICCID[] = "********************";
unsigned char CellID[] = "*****";
BYTE taskAfterDelay;
char prevID[4];
BYTE numOprt = 0;
BYTE curOprtIndex;
extern unsigned int nEzrFw2Upg;
extern unsigned int nCBUFw2Upg;

//BYTE nConnectError = 0;
BYTE nRegDenied = 0;
BYTE fMdmAns;
//BYTE fModemModel;
static Ulong_bytes newId;
flash unsigned char PHYTECH_FILE_END_MARK[] = "phy111\r\n#";  //8  MUST BE AT LEAST 6 CHARS!!!!!!!!!
// update here rom version
//flash unsigned char MONTH_T = 7;
flash unsigned char YEAR_T = 24;
//flash unsigned char INDEX_T = 1;

#ifdef LitePlusDebug
flash unsigned char YEAR = YEAR_T + 100;
#else
flash unsigned char YEAR = YEAR_T;
#endif
flash unsigned char RomVersion[4] = {'U',5, YEAR, 10};   //__BUILD__

extern eeprom _tagAPPEEPROM AppEepromData;
flash unsigned char fSWUpdatePort[] = "80@"; 
flash unsigned char fSWUpdateAddress[] = "bootloader.phytech.com@";
//flash unsigned char fSWUpdateAddress[] = "bootloader-staging.herokuapp.com";
extern flash unsigned char fEZRUpdateAddress[];// = "phytech.dev.valigar.co.il@";
            
const BYTE BAUD_RATE_HIGH[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const BYTE BAUD_RATE_LOW[]  = {0x01, 0x03, 0x05, 0x0B, 0x17, 0x2F, 0xBF/*, 0x5F, 0x0F*/};              //0x5F = 2400     0x0F = 14400
#ifdef DebugMode
const long BAUD_RATE_LONG[] = {115200, 57600, 38400 , 19200 , 9600, 4800, 1200/*, 2400, 14400*/};
#endif DebugMode


flash unsigned char  AT_DELL_ECHO[] = "ATE0\r\n\0";
flash unsigned char  AT_SAVE[] = "AT&W\r\n\0"; 
flash unsigned char  AT_SET_BAUDRATE[] = "AT+IPR=9600;&W\r\n\0";   //"AT+IPR=19200;&W\r\n\0";
flash unsigned char  AT_IsModemOK[] = "AT\r\n\0";             //5 bytes
flash unsigned char  AT_IsModemReg[] = "AT+CREG?\r\n\0";     //10 bytes
flash unsigned char  AT_REG_UMTS_STATUS[] = "AT+CGREG?\r\n\0";
flash unsigned char  AT_REG_LTE_STATUS[] = "AT+CEREG?\r\n\0";
flash unsigned char  AT_COPS_AUTO_0[] = "AT+COPS=0\r\n\0";      //11 bytes
flash unsigned char  AT_COPS_LST[] = "AT+COPS=?\r\n\0";
flash unsigned char  AT_COPS_MAN[] = "AT+COPS=1,2,@";       //12 bytes. must have # at the end
flash unsigned char  AT_COPS_MAN_MONITOR[] = "AT+COPS=1,2,\"42503\"\r\n\0";       //only for monitor connecting
flash unsigned char  AT_COPS_ASK[] = "AT+COPS?\r\n\0";
flash unsigned char  AT_CELL_MONITOR[] = "AT+QNWINFO\r\n\0";   //"AT#MONI\r\n\0";
flash unsigned char  AT_CSQ[] = "AT+CSQ\r\n\0";               //8 bytes  AT+GMR-returns the software revision identification
flash unsigned char  AT_EOD[] = "+++\0";               //8 bytes  AT+GMR-returns the software revision identification
flash unsigned char  AT_QSS[] = "AT+QSIMSTAT?\r\n\0"; //"AT#QSS?\r\n\0";   
flash unsigned char AT_CCID[] = "AT+QCCID\r\n\0";       //"AT#CCID\r\n\0";
flash unsigned char AT_PWROFF[] = "AT+QPOWD\r\n\0";
//GPRS connecting commands:
flash unsigned char GPRS_ATTACH[] = "AT+CGATT=1\r\n\0";                               //12
flash unsigned char DEF_PDP_CNTXT[] = "AT+CGDCONT=1,\"IP\",@";    //39
flash unsigned char DEF_PDP_CNTXT_VZN[] = "AT+CGDCONT=1,\"IPV4V6\",@";    //39
//flash unsigned char DEF_PDP_CNTXT_LTE[] = "AT+CGDCONT=3,\"IP\",@";    //39
//flash unsigned char DEF_QULT_MIN_PROF[] = "AT+CGQMIN=1,0,0,0,0,0\r\n\0";                     //24
//flash unsigned char DEF_QULT_REQ_PROF[] = "AT+CGQREQ=1,0,0,0,0,0\r\n\0";                     //24
//flash unsigned char DEF_SCKT_CNFG[] = "AT#SCFG=1,1,1500,90,450,3\r\n\0";                      //24
//flash unsigned char DEF_SCKT_CNFG_LTE[] = "AT#SCFG=3,3,1500,90,450,3\r\n\0";                      //24  AT#SCFG=3,3,1500,90,600,50
flash unsigned char ACTIVATE_CNTXT[] = "AT+CGACT=1,1\r\n\0";    //"AT#SGACT=1,1\r\n\0";                                //14
flash unsigned char DEACTIVATE_CNTXT[] = "AT+CGACT=0,1\r\n\0";  //"AT#SGACT=1,0\r\n\0";                                //14
flash unsigned char ISACTIVATE_CNTXT[] = "AT+CGACT?\r\n\0";
flash unsigned char AT_TCP_OPN[] = "AT+QIOPEN=1,0,\"TCP\",@";           //"AT#SD=1,0,1020,\"phytech1.dyndns.org\"\r\n";   //40
//flash unsigned char AT_TCP_OPN_LTE[] = "AT#SD=3,0,@";           //"AT#SD=1,0,1020,\"phytech1.dyndns.org\"\r\n";   //40
flash unsigned char AT_TCP_CLS[] = "AT+QICLOSE=0\r\n\0";    //"AT#SH=1\r\n\0";                               //9
//flash unsigned char MODEM_CLS[] = "AT+QPOWD\r\n\0";  //"AT#SHDN\r\n\0";
// post commands
//flash unsigned char AT_POST_TITLE_PRM[] = "POST /api/sensor/loggerparamsezr HTTP/1.1\r\n#";  //                 api/file/sensorparams
flash unsigned char AT_POST_TITLE_PRM[] = "POST /api/sensor/loggerparamscs HTTP/1.1\r\n#";  // 
//flash unsigned char AT_POST_TITLE_PRM1[] = "POST /api/Get HTTP/1.1\r\n#";  //"GET /api/Get HTTP/1.1\r\n#";
//flash unsigned char AT_POST_TITLE_PRM1[] = "POST phytechtest.azurewebsites.net:443 /api/Get HTTP/1.1\r\n#";  //"GET /api/Get HTTP/1.1\r\n#";
//flash unsigned char AT_POST_TITLE_PRM12[] = "Host: phytechtest.azurewebsites.net:443\r\n#";
//flash unsigned char AT_POST_TITLE_VLV_V1[] = "POST /api/sensor/SensorValvUpdate HTTP/1.1\r\n#";  //                 
//flash unsigned char AT_POST_TITLE_VLV[] = "POST /api/sensor/getValvTaskV2 HTTP/1.1\r\n#";  //    
flash unsigned char AT_POST_TITLE_VLV[] = "POST /api/sensor/getCoolingTask HTTP/1.1\r\n#";  //       
//flash unsigned char AT_POST_TITLE_VLV[] = "POST /api/sensor/getBulkCoolingTask HTTP/1.1\r\n#";  //       
#ifdef SMART_SERVER 
flash unsigned char AT_POST_TITLE_CBU_DATA[] = "POST /data HTTP/1.1\r\n#";  //
#else
flash unsigned char AT_POST_TITLE_CBU_DATA[] = "POST /api/sensor/fbudata HTTP/1.1\r\n#";  //
#endif   
flash unsigned char AT_POST_TITLE_ALERT[] = "POST /api/sensor/cbualerts HTTP/1.1\r\n#";  //   
flash unsigned char AT_POST_TITLE_VCU_LST[] = "POST /api/sensor/ccumetadata HTTP/1.1\r\n#";  //  
//flash unsigned char AT_POST_TITLE_DATA_SHORT[] = "POST /api/sensor/loggerngshortex HTTP/1.1\r\n#";       
flash unsigned char AT_POST_TITLE_CBU_META_DATA[] = "POST /api/sensor/cbumetadatav3 HTTP/1.1\r\n#";  //   
//flash unsigned char AT_POST_TITLE_CBU_META_DATA_OLD[] = "POST /api/sensor/cbumetadata HTTP/1.1\r\n#";  //   

//flash unsigned char AT_POST_TYPE1[] = "Content-Type: text/html; charset=utf-8\r\n#";   //52
              
flash unsigned char AT_POST_TITLE_VCU_DATA[] = "POST /api/sensor/vcudata HTTP/1.1\r\n#";              // ValvSensorData
//flash unsigned char AT_POST_TITLE_PUMP_DATA[] = "POST /api/sensor/ValvSensorData HTTP/1.1\r\n#";
flash unsigned char AT_POST_TITLE_GETPRM[] = "POST /api/sensor/sensorupdate HTTP/1.1\r\n#";  //  41          api/file/postparamupdate
flash unsigned char AT_POST_TITLE_CNFRMPRM[] = "POST /api/sensor/sensorupdateconfirmation HTTP/1.1\r\n#";  //api/file/postparamupdateconfirmation
flash unsigned char AT_POST_TITLE_CNFRMVLV[] = "POST /api/sensor/sensorvlvcmdconfirmation HTTP/1.1\r\n#";  //api/file/postparamupdateconfirmation

flash unsigned char AT_GET_SW_STATUS_3[] = "GET http://@";  
flash unsigned char AT_GET_SW_STATUS_4[] = "/status/@";  //  
flash unsigned char AT_GET_SW_STATUS_5[] = "/rfstatus/@";  //  
flash unsigned char AT_GET_SW_STATUS_2[] = " HTTP/1.1\r\n@"; 
flash unsigned char AT_POST_CONN[] = "Connection: keep-alive\r\n#";                     //24
flash unsigned char AT_POST_TYPE[] = "Content-Type: multipart/form-data; boundary=#";   //52
flash unsigned char AT_POST_HOST[] = "Host: @";         //Host: phytech1.dyndns.org:1011\r\n#";             //32
flash unsigned char AT_POST_LENGTH1[] = "Content-Length: 257\r\n\r\n#";                     //BuildExtParamBuff  253
//flash unsigned char AT_POST_LENGTH2[] = "Content-Length: 11\r\n\r\n#";                     //BuildExtParamBuff  253
flash unsigned char AT_POST_LENGTH4[] = "Content-Length: @";                     //23
//flash unsigned char AT_POST_LENGTH3[] = "Content-Length: 130\r\n\r\n#";                     //23
//flash unsigned char AT_POST_LENGTH6[] = "Content-Length: 123\r\n\r\n#";                     //23
flash unsigned char AT_POST_FILE_HDR1[] = "Content-Disposition: form-data; name=\"file\"; #";             //45
flash unsigned char AT_POST_FILE_HDR_PRM[] = "filename=\"PARAMS.txt\"\r\n#";    //23
flash unsigned char AT_POST_FILE_HDR_DATA[] = "filename=\"VLVDATA.txt\"\r\n#";    //21
flash unsigned char AT_POST_FILE_HDR_UPDTPRM[] = "filename=\"UPDTPRM.txt\"\r\n#";    //29
//flash unsigned char AT_POST_FILE_HDR_GETDATA[] = "filename=\"GETPARAMPOST.txt\"\r\n#";    //29
flash unsigned char AT_POST_FILE_HDR_VALVE[] = "filename=\"VALVCMD.txt\"\r\n#";    //24
flash unsigned char AT_POST_FILE_HDR_ALERT[] = "filename=\"ALERTSS.txt\"\r\n#";    //24
flash unsigned char AT_POST_FILE_HDR_PUMP[] = "filename=\"PUMPACT.txt\"\r\n#";    //24
flash unsigned char AT_POST_FILE_HDR_CBU_DATA[] = "filename=\"CBUDATA.txt\"\r\n#";    //24
flash unsigned char AT_POST_FILE_HDR_CBU_META[] = "filename=\"CBUMETA.txt\"\r\n#";    //24
flash unsigned char AT_POST_FILE_HDR_VCU_LIST[] = "filename=\"VCULIST.txt\"\r\n#";    //24
flash unsigned char AT_POST_FILE_HDR2[] = "Content-Type: text/plain\r\n\r\n#";          //28
//flash unsigned char BACKUP_URL[] = "54.246.50.57#0000000000000000000"; //"63.34.115.252#000000000000000000";//"proxy.backup.phytech.com#0000000";
//flash unsigned char BACKUP_URL_LTE[] = "54.246.50.57#0000000000000000000";  // "63.34.115.252#000000000000000000";//"54.246.85.255#000000000000000000";;

BYTE SendRecATCmd(flash unsigned char *bufToSend, BYTE tOut);

void InitOperatorLst()
{
    BYTE i;              
    fGetCopsLst = FALSE;
    curOprtIndex = 0;
    numOprt = 0;
    for (i = 0; i < 10; i++)
        OprtTbl[i] = 0;
}

void InitVarsForConnecting(/*BYTE b4Vlv*/)
{
    unsigned int s;  
                    
    s = IsModemOn();                  
    #ifdef DebugMode   
    SendDebugMsg("\r\nInitVars 4 Connecting. vlvSentOK=\0");      
    PrintNum(vlvSentOK);       
    SendDebugMsg("\r\nmodem status is \0");      
    PrintNum(s);  
    #endif DebugMode   

    mainTask = TASK_MODEM;   
    g_RAN = RAN_3G;
    bSendPrms = 0;    
    nErrorOnConnect = 0;   
    if ((bExtReset == TRUE) || (btrStatus == BTR_STATUS_EMPTY) || 
        ((g_curTime.minute == 0)))     
        bSendPrms = DO_PARAMS;

    if (prevMainTask != TASK_MODEM)   
    {   
        if ((bExtReset == TRUE) || /*(vlvSentOK == FALSE) ||*/ (s == FALSE) || (g_bModemConnect == FALSE)) 
        {
            modemCurTask = TASK_NONE;     
//            fGetCopsLst = FALSE;      
            MODEM_PWR_ENABLE(); 
            #ifdef DebugMode
            SendDebugMsg("\r\npower on modem");
            #endif DebugMode               
            delay_ms(1000);  
        }
        else                                  
        {
            modemCurTask = TASK_MODEM_CONNECT;       
            modemCurSubTask = SUB_TASK_MODEM_CONNECT_ACTV;
            if (bSendPrms == DO_PARAMS)   
            {
                modemCurTask = TASK_MODEM_INIT;
                modemCurSubTask = SUB_TASK_INIT_MODEM_RSSI;       
            }
        }                    
    }                  
     #ifdef DebugMode  
    else
    { 
        SendDebugMsg("\r\ncontinue modem from last task=\0");      
        PrintNum(modemCurTask); 
        PrintNum(modemCurSubTask);       
    }           
    #endif DebugMode
    bEndOfModemTask = FALSE;
    bConnectOK = FALSE;
//    dataSentOK = FALSE;
    prmSentOK = FALSE;      
    vlvSentOK = FALSE;     
        
//    g_bHighPrio = FALSE;     
    post2send = POST_NONE;    
    fSwUpdate = 0;    
    ENABLE_UART0();
    #ifdef LiteDebug           
    ENABLE_UART1(); 
    UART1Select(UART_DBG); 
    #endif  LiteDebug   
    g_nTime2StartAT = 130;  
    nCBUFw2Upg = 0;     
    nEzrFw2Upg = 0;
}


//send flash init string to modem
void SendATCmd(flash unsigned char *bufToSend, int nWait4Answer, BYTE nRetry)
{
    unsigned int i;

    i = 0;
    //copy flash string to buff
    while ((bufToSend[i] != cEndMark) && (i < MAX_RX1_BUF_LEN))
    {
         ComBuf[i] = bufToSend[i];
         i++;
    }
    BytesToSend = i ;  
    nMaxWaitingTime = nWait4Answer;   // wait max 2 sec for answer
    nMaxFailuresNum = nRetry;
    //transmitt to local modem port
    TransmitBuf(0);
}

// translate rssi val from string to single byte
char ConvertRssiVal(void)
{
        BYTE  b = 0;
        int i;

        rssi_val = 100;
        for( i = 0; i < rx0_buff_len - 4 ; i++)
        {
            //if return "+CSQ:"
            if((RxUart0Buf[i] == 0x2B) &&
               (RxUart0Buf[i+1] == 0x43) &&
               (RxUart0Buf[i+2] == 0x53) &&
               (RxUart0Buf[i+3] == 0x51) &&
               (RxUart0Buf[i+4] == 0x3A))
            {
                b = 1;
                //serch for ',':
                // if rssi val is only 1 digit - take it as is
                if(RxUart0Buf[i+7] == 0x2C)
                    //rssi val is 1 byte
                    rssi_val = RxUart0Buf[i+6]-0x30;
                   // rssi_val = 0x30 + ComBuf[i+6];
                else
                    // if its 2 digits:
                    if(RxUart0Buf[i+8] == 0x2C)
                    {
                        rssi_val = RxUart0Buf[i+6]-0x30;
                        rssi_val *= 10;
                        rssi_val += RxUart0Buf[i+7]-0x30;
                    }
                break;
            }
        }
        // no answer found (+CSQ:)
        if (b == 0)
            return FALSE;

        // if rssi is very low:
        if (rssi_val < 10)
        {
            // if should have done only data - send also params
                bSendPrms = DO_PARAMS;//= DO_DATA_N_PRMS;     
                #ifdef DebugMode   
                SendDebugMsg("\r\nadd params 2\0");   
                #endif DebugMode   
        }

        return TRUE;
}

void TurnOnIgnition()
{
    #ifdef DebugMode
    SendDebugMsg("\r\nTurnOnIgnition");
    #endif DebugMode
    MODEM_IGNITION_ON();
    initCnt++;   
    delay_ms(1000);
    ModemResponse = TASK_COMPLETE;
}

void TurnOffIgnition()
{
    #ifdef DebugMode
    SendDebugMsg("\r\nTurnOffIgnition");
    #endif DebugMode
    MODEM_IGNITION_OFF();     
    TimeLeftForWaiting = 150;
    nMaxFailuresNum = 1;  
    bWaitForModemAnswer = TRUE;  
    bNeedToWait4Answer = TRUE; 
    bCheckRxBuf = FALSE;             
    ModemResponse = NO_ANSWER;
}

void ModemHwShdn()
{
    MODEM_PWR_DISABLE(); 
    g_bModemConnect = FALSE;
    ModemResponse = TASK_COMPLETE;
    #ifdef DebugMode
    SendDebugMsg("\r\nModemHwShdn");
    #endif DebugMode
}

char IsModemOn()     //MODEM_3G_PWR_MON
{
    if (!(PINC & (1<<MODEM_STATUS_PIN)))
    {
    #ifdef DebugMode
    SendDebugMsg("\r\nModemIsOn\r\n");
    #endif DebugMode
        return TRUE;
    }
    #ifdef DebugMode
    SendDebugMsg("\r\nModemIsOff");
    #endif DebugMode
    return FALSE;
}

//send init string AT+WIPCREATE=2,1,"109.253.23.173",1007 to mode
void SendString(unsigned char *bufToSend, BYTE bufLen)
{
    BYTE i;

    for (i = 0; i < bufLen; i++)
        ComBuf[i] = bufToSend[i];
        //transmitt to local modem port
    BytesToSend = bufLen;
    TransmitBuf(0);
}

void BuildAlertBuff()
{
    int index;
    BYTE cs;

    index = 0;      
    // CCU ID
    cpu_e2_to_MemCopy( &ComBuf[index], &AppEepromData.eLoggerID[0], 4);
    index += 4;                      
    
    // CBU ID   
    cpu_e2_to_MemCopy( &ComBuf[index], &AppEepromData.eCbuGlblData[0], 4);
    index += 4;

    // Timestamp  + values
    memcpy(&ComBuf[index],&DataBlock[0], ALERTS_MEMORY_PACKET_SIZE);
    index += ALERTS_MEMORY_PACKET_SIZE;      
    
     //check sum
    cs = CheckSum(ComBuf, index, 1);      
    ComBuf[index++] = cs;

    ComBuf[index++] = '\r';
    ComBuf[index++] = '\n';
    BytesToSend = index;
}

void BuildCBUDataBuff()
{
    int index;
    BYTE cs;

    index = 0;     
    // CCU ID
    cpu_e2_to_MemCopy( &ComBuf[index], &AppEepromData.eLoggerID[0], 4);
    index += 4;                      
    
    // CBU ID   
    cpu_e2_to_MemCopy( &ComBuf[index], &AppEepromData.eCbuGlblData[0], 4);
    index += 4;

    // Timestamp  + values
    memcpy(&ComBuf[index],&DataBlock[0], CBU_MNT_DATA_SIZE);
    index += CBU_MNT_DATA_SIZE;      
    
     //check sum
    cs = CheckSum(ComBuf, index, 1);      
    ComBuf[index++] = cs;

    ComBuf[index++] = '\r';
    ComBuf[index++] = '\n';
    BytesToSend = index;   
//    for (i = 0; i < BytesToSend; i++)
#ifdef SMART_SERVER   
    memcpy(&tmpBuf, &ComBuf, BytesToSend);   
    #endif SMART_SERVER   
}

// size:149
void BuildCBUMetaDataBuff()
{
    int index;
    BYTE cs;//, n, s = sizeof(_tagPortDefEEPROM); 

    index = 0;      
    // CCU ID
    cpu_e2_to_MemCopy( &ComBuf[index], &AppEepromData.eLoggerID[0], 4);
    index += 4;                      
    
    //  CBU ID, cbu version, dry contact, enable safety, pump operation mode
    cpu_e2_to_MemCopy( &ComBuf[index], &AppEepromData.eCbuGlblData[0] , CBU_GET_GLBL_CNFG_LEN);      
    index += CBU_GET_GLBL_CNFG_LEN;             

    //  ports  data  
    cpu_e2_to_MemCopy(&ComBuf[index], (eeprom char*)(&ComponentArray[0]), sizeof(_tagPortDefEEPROM) * MAX_PORTS_CBU);   //180 bytes
//    for (n = 0; n < 3; n++)
//    {
//    // CBU meta data  
//        cpu_e2_to_MemCopy( &ComBuf[index], (eeprom char*)&ComponentArray[n] , s);
//        
//        index += s;//16;                                   
//    }
      index += sizeof(_tagPortDefEEPROM) * MAX_PORTS_CBU;
//    // pulse counter data
    cpu_e2_to_MemCopy( &ComBuf[index], (eeprom char*)&FlowDef[0].HighTreshold , sizeof(_tagFlowDefEEPROM)-1);
    index += sizeof(_tagFlowDefEEPROM)-1;
    cpu_e2_to_MemCopy( &ComBuf[index], (eeprom char*)&FlowDef[1].HighTreshold , sizeof(_tagFlowDefEEPROM)-1);
    index += sizeof(_tagFlowDefEEPROM)-1;
                           
    memcpy(&ComBuf[index], (unsigned char*)&g_curTime, 5);
    index += 5;

     //check sum
    cs = CheckSum(ComBuf, index, 1);      
    ComBuf[index++] = cs;

    ComBuf[index++] = '\r';
    ComBuf[index++] = '\n';
    BytesToSend = index;
}

// get irrigation command buffer
void BuildValveBuff()       // size:8
{
    BYTE  cs, index = 0; 
             
    // ID
    cpu_e2_to_MemCopy( &ComBuf[index], &AppEepromData.eLoggerID[0], 4);
    index += 4;  
    ComBuf[index++] = UPDATE_VALVE_CMD;   //UPDATE_COOLING_CMD;//                        

    //check sum
    cs = CheckSum(ComBuf, index, 1);
    ComBuf[index++] = cs;

    ComBuf[index++] = '\r';
    ComBuf[index++] = '\n';
    BytesToSend = index;
}
void BuildVcuListPost()
{
#ifndef NO_EZR
    BYTE  cs, i, index = 0; 
             
    // ID
    cpu_e2_to_MemCopy( &ComBuf[index], &AppEepromData.eLoggerID[0], 4);
    index += 4; 
    ComBuf[index++] = MAX_CMD; 
    for (i = 0; i < MAX_CMD; i++) 
    {
        ULong2Bytes(vlvCmdArr[i].VCU_ID, &ComBuf[index]);  
        index += 4;  
    }
    cs = CheckSum(ComBuf, index, 1);   
    ComBuf[index++] = cs;
    ComBuf[index++] = '\r';
    ComBuf[index++] = '\n';
    BytesToSend = index;
#endif NO_EZR
}

void BuildVCUDataStr()
{
#ifndef NO_EZR
    int index = 0;
    BYTE cs;

    MemCopy( &ComBuf[index], &DataBlock[0], 34);
    index += 34;

    //check sum
    cs = CheckSum(ComBuf, index, 1);
    ComBuf[index++] = cs;

    ComBuf[index++] = '\r';
    ComBuf[index++] = '\n';
    BytesToSend = index;
#endif NO_EZR
}

void SendPostHost(BYTE fServer)
{
    BYTE i, n;
    //build the host address from url+port
    n = CopyFlashToBuf(ComBuf, AT_POST_HOST);
    i = 0;
    if (fServer == SERVER_BOOTLOADER) 
    {                  
//        if (fSwUpdate == 1)                                   
            i = CopyFlashToBuf(&ComBuf[n], fSWUpdateAddress);
//        if (fSwUpdate == 2)      
//
        n += i;
        ComBuf[n++] = '\r';
        ComBuf[n++] = '\n';
    }     
    if (fServer == SERVER_DATA) 
    {
        while ((AppEepromData.eIPorURLval1[i] != '#') && (i < 32))    //HASHTAG)  
        {
            ComBuf[i+n] = AppEepromData.eIPorURLval1[i];
            i++;
        }

        n += i;
        ComBuf[n++] = ':';
            cpu_e2_to_MemCopy(&ComBuf[n], AppEepromData.ePORTval1, 4);
        n += 4;
    }
    ComBuf[n++] = '\r';
    ComBuf[n++] = '\n';
    BytesToSend = n;
    TransmitBuf(0);
}

void SendPostLength(int len)
{
    BYTE i, n;
    char s[4];
    //build the host address from url+port
    n = CopyFlashToBuf(ComBuf, AT_POST_LENGTH4);
    i = 0;
    do
    {
        s[i++] = (char)(len % 10);
        len = len / 10;
    }
    while (len > 0);

    for (; i > 0; i--)
       ComBuf[n++] = s[i-1] + 48;

    ComBuf[n++] = '\r';
    ComBuf[n++] = '\n';
    ComBuf[n++] = '\r';
    ComBuf[n++] = '\n';
    BytesToSend = n;
    TransmitBuf(0);
}

void SendPostMsg(BYTE postType, flash unsigned char* title, int length, flash unsigned char* fileHeader)
{
    BYTE cs, nPctIndex = 0;
    int  nCnt;
    char endFileStr[10];
                        
     nCnt = GetEpromPcktCnt(postType);       
     if (nCnt > 20)
        nCnt = 20;
     // sign that for all next transmits - no need to wait for ana answer from modem
     bNeedToWait4Answer = FALSE;
     // preare end file mark string
     endFileStr[0] = '-';
     endFileStr[1] = '-';
     cpu_flash_to_MemCopy(&endFileStr[2], PHYTECH_FILE_END_MARK, 8);
     // send post header
     SendATCmd(title, 0,0);
     SendATCmd(AT_POST_CONN, 0,0);
     SendATCmd(AT_POST_TYPE, 0,0);
     SendATCmd(PHYTECH_FILE_END_MARK, 0,0);
     SendPostHost(SERVER_DATA);
     //SendATCmd(AT_POST_LENGTH5, 0,0);      //length  
     SendPostLength(nCnt * (107 + length) + 10);    
                    
     do
     {
         // send file header:
         SendString(endFileStr, 10);                //10
         SendATCmd(AT_POST_FILE_HDR1, 0,0);         //45
         SendATCmd(fileHeader, 0,0);                //24
         SendATCmd(AT_POST_FILE_HDR2, 0,0);         //28     
         // only read eeprom when post send data from eeprom
         if (postType <= (int)POST_PUMP_ACT)
            ReadPacket(postType);  
        // build post body          
        switch (postType)
        {                         
	    case POST_CBU_DATA:
		    BuildCBUDataBuff();         //       
            break;
	    case POST_GET_CMD:
		    BuildValveBuff();     
            break;
	    case POST_VCU_DATA:
            BuildVCUDataStr();   //BuildDataStr();// 36    
            break;
	    case POST_GET_UPDATE:   
        case POST_CNFRM_UPDATE:
        {
            cpu_e2_to_MemCopy(ComBuf, &AppEepromData.eLoggerID[0], 4);        //
            ComBuf[4] = prmUpdtIndex;                           //index+checksum+\r+\n = 4
            //check sum
            cs = CheckSum(ComBuf, 5, 1);
            //GPRSBuf[5] = 0;
            ComBuf[5] = cs;
            ComBuf[6] = '\r';
            ComBuf[7] = '\n';
            // send post
            BytesToSend = 8;  
            break;
        }         
        case POST_ALERT:    
            BuildAlertBuff();
            break;     
        case POST_PUMP_ACT: 
            BuildVCUDataStr();  //BuildPumpBuff();
            break;
	    case POST_CBU_META_DATA:    
//            if (g_nCbuVer == ItayCBU)
                BuildCBUMetaDataBuff();      
//            else   
//                BuildCBUMetaDataBuffOld();
            break; 
        case POST_VCU_LIST: 
            BuildVcuListPost();
            break;
        }   
         // send post
         TransmitBuf(0);         
     }
     while (++nPctIndex < nCnt); 
     
     bNeedToWait4Answer = TRUE;       
     nMaxWaitingTime = 300;  // wait max 30 sec fot response
     SendString(endFileStr, 10);                //10
              
    #ifdef SMART_SERVER   
    for (i =0; i < 45; i++)
    {  
        PrintNum(tmpBuf[i]);
        SendDebugMsg("\r\n \0"); 
    } 
    #endif SMART_SERVER  

//     lastByteIndex = 0;
     longAnswerExpected = 1;
     overFlow = 0;      
}

// build the string of parameters to send to web service
void BuildExtParamsBuff()
{
    int index;
    BYTE cs;
    char buf[4];

    index = 0;
    // ID
    cpu_e2_to_MemCopy( &ComBuf[index], &AppEepromData.eLoggerID[0], 4);
    index += 4;
    // URL
    cpu_e2_to_MemCopy( &ComBuf[index], AppEepromData.eIPorURLval1, 32);
    index += 32;
    // APN
    cpu_e2_to_MemCopy( &ComBuf[index], AppEepromData.eAPN, 32);
    index += 32;
    //Port
    cpu_e2_to_MemCopy( &ComBuf[index], AppEepromData.ePORTval1, 4);
    index += 4;

// ICCID num.
    memcpy(&ComBuf[index],&ICCID[0], MAX_ICCID_LEN);   
    index += MAX_ICCID_LEN;
//    for (i = 0; i < MAX_ICCID_LEN; i++)
//        ComBuf[index++] = ICCID[i];

    // Wakeup timing
    ComBuf[index++] = g_nFailureCnt;
    ComBuf[index++] = 0;    //GetVlvCnt();  
    ComBuf[index++] = AppEepromData.eConnectIntervalH;
    // RSSI
    ComBuf[index++] = rssi_val;
    //Battery
    ComBuf[index++] = (unsigned char)((iVoltage >> 8) & 0xFF);     //address high
    ComBuf[index++] = (unsigned char)(iVoltage) ;                 //address low
    //location :       
    Float2Bytes(/*g_fLat*/0, buf);
    MemCopy(&ComBuf[index], buf  ,4);
    index+= 4;

    Float2Bytes(/*g_fLon*/0, buf);
    MemCopy(&ComBuf[index], buf  ,4);
    index+= 4;

    Float2Bytes(0, buf);
    MemCopy(&ComBuf[index], buf  ,4);
    index+= 4;

    //roaming
    ComBuf[index++] = AppEepromData.eUseCntrCode;
    if (btrStatus == BTR_STATUS_EMPTY)    
        ComBuf[index++] = 0xFF;
    else
        ComBuf[index++] = AppEepromData.eRoamingDelay;

    // mobile net code
    cpu_e2_to_MemCopy( &ComBuf[index], AppEepromData.eMobileNetCode, 4);
    index += 4;

    // mobile country code
    cpu_e2_to_MemCopy( &ComBuf[index], AppEepromData.eMobileCntryCode, 4);
    index += 4;

    // Cell ID
    memcpy(&ComBuf[index],&CellID[0], 5);   
    index += 5;
    // Clock
    GetRealTime();

    ComBuf[index++] = g_curTime.year;// =readClockBuf[0]; //year
    ComBuf[index++] = g_curTime.month;// readClockBuf[1]; //month
    ComBuf[index++] = g_curTime.day;// = readClockBuf[2]; //day
    ComBuf[index++] = g_curTime.hour;// =readClockBuf[4]; //hour
    ComBuf[index++] = g_curTime.minute;// readClockBuf[5]; //minute

    // ATMEL Version
    cpu_flash_to_MemCopy(&ComBuf[index], RomVersion, 4);
    index += 4;
    // EZR Version
    ULong2Bytes(GetEzrVer(), &ComBuf[index]);
    index += 4;
    //check sum
    cs = CheckSum(ComBuf, index, 1);
    ComBuf[index++] = cs;

    ComBuf[index++] = '\r';
    ComBuf[index++] = '\n';
    BytesToSend = index;
}

/*void SendPostUpdate(BYTE bGetOrConfirm)
{
    char endFileStr[10];
    BYTE cs;

    // preare end file mark string
    endFileStr[0] = '-';
    endFileStr[1] = '-';
    cpu_flash_to_MemCopy(&endFileStr[2], PHYTECH_FILE_END_MARK, 8);

    bNeedToWait4Answer = FALSE;
    if (bGetOrConfirm)
        SendATCmd(AT_POST_TITLE_GETPRM, 0, 0);
    else
    {
        if (prmUpdtIndex == UPDATE_VALVE_CMD)
            SendATCmd(AT_POST_TITLE_CNFRMVLV,0, 0);  
        else
            SendATCmd(AT_POST_TITLE_CNFRMPRM, 0, 0);
    }   
    SendATCmd(AT_POST_CONN, 0,0);
    SendATCmd(AT_POST_TYPE, 0,0);
    SendATCmd(PHYTECH_FILE_END_MARK, 0,0);
    SendPostHost(SERVER_DATA);
    SendATCmd(AT_POST_LENGTH3, 0,0);

    // send file header:
    SendString(endFileStr, 10);            //10
    SendATCmd(AT_POST_FILE_HDR1, 0,0);       //45
    SendATCmd(AT_POST_FILE_HDR_GETDATA, 0,0);    //29
    SendATCmd(AT_POST_FILE_HDR2, 0,0);       //28

    // build post body
    // if confirming update logger ID
    if ((bGetOrConfirm == CONFIRM_UPDATE) && (prmUpdtIndex == UPDATE_ID))
        MemCopy(ComBuf, prevID, 4);        //send previous logger ID
    // all other cases - send ID
    else
        cpu_e2_to_MemCopy(ComBuf, &AppEepromData.eLoggerID[0], 4);        //
    ComBuf[4] = prmUpdtIndex;                           //index+checksum+\r+\n = 4
    //check sum
    cs = CheckSum(ComBuf, 5, 1);
    //GPRSBuf[5] = 0;
    ComBuf[5] = cs;
    ComBuf[6] = '\r';
    ComBuf[7] = '\n';
    // send post
    BytesToSend = 8;
    TransmitBuf(0);                //8
    bNeedToWait4Answer = TRUE;   
    nMaxWaitingTime = 300;  // wait max 30 sec fot response
    SendString(endFileStr, 10);            //10
    longAnswerExpected = 1;
    overFlow = 0;
}     */

void SendGetSWUpdate()
{
    BYTE s[10]; 
    char tmp[4];          
    BYTE i = 0 ,n,j;  
    long l;
    
    for (i = 0; i < 4; i++)
        tmp[i] = AppEepromData.eLoggerID[i];     
    i = 0;
    l = Bytes2ULong(tmp);

    while (l > 0)    
    {
        s[i] = l % 10;
        l = l / 10;   
        i++;
    } 
    n = CopyFlashToBuf(ComBuf, AT_GET_SW_STATUS_3); 
//    if (fSwUpdate == 1)                                   
        j = CopyFlashToBuf(&ComBuf[n], fSWUpdateAddress);
//    if (fSwUpdate == 2)      
//        j = CopyFlashToBuf(&ComBuf[n], fEZRUpdateAddress);   
    n += j;
    if (fSwUpdate == 1)                                   
        j = CopyFlashToBuf(&ComBuf[n], AT_GET_SW_STATUS_4);   
    if ((fSwUpdate == 2) || (fSwUpdate == 3))     
        j = CopyFlashToBuf(&ComBuf[n], AT_GET_SW_STATUS_5);   
    n += j;
    
    while (i > 0)
    {                     
        i--;
        ComBuf[n++] = s[i] + 0x30;     
    }                      

    i = CopyFlashToBuf(&ComBuf[n], AT_GET_SW_STATUS_2);

    BytesToSend = n + i;
    bNeedToWait4Answer = FALSE;  
    TransmitBuf(0); 
    SendATCmd(AT_POST_CONN, 0,0);
    bNeedToWait4Answer = TRUE; 
    nMaxWaitingTime = 300;  // wait max 30 sec fot response

    SendPostHost(SERVER_BOOTLOADER);

    longAnswerExpected = 1;
    overFlow = 0;
}

// build the string of data to send to web service
// for 1st service, func "sensordata": length was 63
// for extended service (Nir), func "loggersensordata": length was  58
// for service with limites: func "loggersensordatath": length is: 66  (limits+strips)
// to be: for service of wireless sensors: "LoggerWirelessSensorData". length is 70
// for wireless new sensors: "LoggerWirelessSensorData". length is 45
// for VALVES: "ValvSensorData". 36
void SendPostParam()
{
     char endFileStr[10];

     // sign that for all next transmits - no need to wait for ana answer from modem
     bNeedToWait4Answer = FALSE;
//     bSendData = 1;
     // preare end file mark string
     endFileStr[0] = '-';
     endFileStr[1] = '-';
     cpu_flash_to_MemCopy(&endFileStr[2], PHYTECH_FILE_END_MARK, 8);

     // send post header
     SendATCmd(AT_POST_TITLE_PRM, 0,0);     
    SendATCmd(AT_POST_CONN, 0,0);
    SendATCmd(AT_POST_TYPE, 0,0);     
    SendATCmd(PHYTECH_FILE_END_MARK, 0,0);
     SendPostHost(SERVER_DATA);     
    SendATCmd(AT_POST_LENGTH1, 0, 0);
     //GPRS_send_init_flash_string(AT_POST_HOST);

     // send file header:
     SendString(endFileStr, 10);         //10
     SendATCmd(AT_POST_FILE_HDR1, 0,0);       //45
     SendATCmd(AT_POST_FILE_HDR_PRM, 0,0);    //23
     SendATCmd(AT_POST_FILE_HDR2, 0,0);       //28

     // build post body                                    //110
     BuildExtParamsBuff();  //141       change MAX_TX_BUF_LEN to > 141  & AT_POST_LENGTH1 to fit size   
     TransmitBuf(0);
     bNeedToWait4Answer = TRUE;        
     nMaxWaitingTime = 300;  // wait max 30 sec fot response
     SendString(endFileStr, 10);            //10

     longAnswerExpected = 1;
     overFlow = 0;  
}

/*void SendTest()
{
     bNeedToWait4Answer = FALSE;

     SendATCmd(AT_POST_TITLE_PRM1, 0,0);        // https://checktlssupport.azurewebsites.net/api/GetTest    
    SendATCmd(AT_POST_TITLE_PRM12 , 0, 0);
//    SendPostHost(SERVER_DATA);
     SendATCmd(AT_POST_CONN, 0,0);
     bNeedToWait4Answer = TRUE;        
     nMaxWaitingTime = 30;  // wait max 3 sec fot response

     longAnswerExpected = 1;
     overFlow = 0;
} */

BYTE IsPrmToUpdate()
{
    BYTE i;
    for (i = 0; i < MAX_PRM_TASKS; i++)
        if (UpdatePrmArr[i] == '1')
        {
            if ((i != UPDATE_SOFTWARE_VER) || (g_bMainPumpOpen == FALSE))
            {
                prmUpdtIndex = i;  
                #ifdef DebugMode
                SendDebugMsg("\r\nprm to update:");
                PrintNum(prmUpdtIndex);
                #endif DebugMode    
                return TRUE;
            }
        }
    return FALSE;
}

BYTE GetBufferIndex(int i)
{
    BYTE b;

    if (i >= MAX_RX_BUF_LEN)
        b = (BYTE)i - MAX_RX_BUF_LEN;
    else
        b = i;
    return b;
}

BYTE IsAT()
{
    if (strstr(RxUart0Buf, "AT"))   
        if (strstr(RxUart0Buf, "O"))
            return TRUE;
    return FALSE;
}

BYTE IsOK()
{
    if (findOK == FOUND_OK)
        return TRUE;
    return FALSE;
}

BYTE IsPowerDown()
{
    if (strstr(RxUart0Buf, "POWERED DOWN"))  
            return TRUE;
    return FALSE;
}

BYTE IsConnect()
{
    if (strstr(RxUart0Buf, "CONNECT"))  
            return TRUE;
    return FALSE;
//    int index = 0;
//    while (index < rx0_buff_len-4)
//    {
//        if ((RxUart0Buf[index] == 'C') && (RxUart0Buf[index+1] == 'O') && (RxUart0Buf[index+2] == 'N') && (RxUart0Buf[index+3] == 'N'))
//            return TRUE;
//        else
//            index++;
//    }
//    return FALSE;
}

BYTE IsVerizon()
{
    if (strstr(RxUart0Buf, "Verizon"))        
    {                  
        g_RAN = RAN_4G;
        return TRUE;                        
    }
    return FALSE;
}

BYTE IsReady()
{
    if (strstr(RxUart0Buf, "RDY"))  
            return TRUE;
    return FALSE;
//    int index = 0;
//    while (index < rx0_buff_len-2)
//    {
//        if ((RxUart0Buf[index] == 'R') && (RxUart0Buf[index+1] == 'D') && (RxUart0Buf[index+2] == 'Y'))
//            return TRUE;
//        else
//            index++;
//    }
//    return FALSE;
}

BYTE IsSimDetected()
{
    BYTE index = 0;

    while (index < rx0_buff_len)
    {
        if (RxUart0Buf[index++] == ',')
            break;
    }
    // if , was found - lookon the digitt after:
    if (index < rx0_buff_len)
        if ((RxUart0Buf[index] == '1') || (RxUart0Buf[index] == '2'))
            return TRUE;
    return FALSE;
}

BYTE IsPDPActive()
{
    int index = 0;
    //find the , (comma \ psik)
    while (index < rx0_buff_len)
    {
        if ((RxUart0Buf[index] == '1') && (RxUart0Buf[index+1] == ','))
            break;                                           
        index++;
    }           
    index += 2;
    // if , was found - lookon the digitt after:
    if (index < rx0_buff_len)
    {
        if (RxUart0Buf[index] == '1')
        {
            return TRUE;
        }   
    }   
    return FALSE;
}

BYTE IsRegistOK()
{
    int index = 0;
    //find the , (comma \ psik)
    while (index < rx0_buff_len)
    {
        if (RxUart0Buf[index++] == ',')
            break;
    }
    // if , was found - lookon the digitt after:
    if (index < rx0_buff_len)
    {
        if ((RxUart0Buf[index] == '1') || (RxUart0Buf[index] == '5'))
        {
            //TurnOnLed(LED_2, SUCCESS);   
            HandleLed(LED_2, CBU_LED_ON);
            HandleLed(LED_3, CBU_LED_BLINK);
            return TRUE;
        }
        if (RxUart0Buf[index] == '3')
            nRegDenied++;
        else
            nRegDenied = 0;
        if ((nRegDenied >= (nMaxFailuresNum / 2)) && (!IsZeroID(AppEepromData.eLoggerID)))   // if its during monitor - do not cut number of connec retry
        {
            failCnt = nMaxFailuresNum;
            nRegDenied = 0;
        }
    }
    return FALSE;
}

void GetCellID()
{
    int n, i = 0;
    if (!IsOK())
        return;
    //find begining of SIM num
    while (!((RxUart0Buf[i] == 'I') && (RxUart0Buf[i+1] == 'd') && (RxUart0Buf[i+2] == ':')) && (i < rx0_buff_len))
        i++;
    i += 3;
    n = 0;
    do
    {
        CellID[n++] = RxUart0Buf[i++];
    }
    while ((RxUart0Buf[i] != ' ') && (n < 5));

    for (; n < 5; n++)
        CellID[n] = '*';
}

void GetICCID()
{
    int n, i = 0;
    if (!IsOK())
        return;
    //find begining of SIM num
    while (((RxUart0Buf[i] < '0') || (RxUart0Buf[i] > '9')) && (i < rx0_buff_len))
        i++;
     n = 0;
    do
    {
        ICCID[n++] = RxUart0Buf[i++];
    }
    while ((RxUart0Buf[i] >= '0') && (RxUart0Buf[i] <= '9') && (n < 20));
    for (; n < 20; n++)
        ICCID[n] = '*';
}

void CopyICCID(BYTE nStartIndex)
{
    BYTE i;
    for (i = 0; i < MAX_ICCID_LEN; i++)
        ComBuf[nStartIndex+i] = ICCID[i];
}

BYTE FillList(BYTE* map, BYTE vlvCnt)
{
    BYTE i, j, nNoSpaceCnt = 0, vlvIndex;         
    int bufIdx = 5;      
#ifndef NO_EZR
    
    for (i = 0; i < vlvCnt; i++)      
    {
        for (j = 0; j < 4; j++)
            newId.bVal[j] = ComBuf[bufIdx++];   
        if ((newId.lVal > 500000) && (newId.lVal < 5000000))
        {
            //insert each valve  
            vlvIndex = InsertVlv(newId.lVal);
            if (vlvIndex == MAX_CMD)
                nNoSpaceCnt++;     
            else
                map[vlvIndex] = 1;    
        }
    }      
#endif NO_EZR
    return nNoSpaceCnt;   
}

BYTE UpdateVlvList(/*BYTE bFirst*/)
{
#ifndef NO_EZR
    BYTE i, j, nNoSpaceCnt = 0, vlvCnt, cs;         
    int bufIdx = bufIndexToUpd;      
    BYTE INorOUT[MAX_CMD];     
    
    memset(INorOUT, 0, MAX_CMD);
    // get CCU ID
    for (j = 0; j < 4; j++)
        if (RxUart0Buf[GetBufferIndex(bufIdx++)] != AppEepromData.eLoggerID[j])   
        {       
            #ifdef DebugMode
            SendDebugMsg("\r\nwrong CCU ID ");
            #endif DebugMode  
            return 0;
        }
    #ifdef DebugMode
    SendDebugMsg("\r\nbufIndexToUpd= ");       
    PrintNum(bufIndexToUpd);
//    for (i = 0; i < MAX_RX_BUF_LEN; i++)   
//    {                          
//        PrintNum(RxUart0Buf[i]);    
//    }               
        SendDebugMsg("\r\nbuf only IDs= ");       
    #endif DebugMode  
    // init flag of each vlv if in list
//    InitValvesUpdateStatus();  
    vlvCnt = RxUart0Buf[GetBufferIndex(bufIdx++)];       
    if (vlvCnt > MAX_CMD)
        return 0;
    bufIdx = bufIndexToUpd;
    // copy all data to other buffer
    for (i = 0; i < vlvCnt*4 + 5; i++)   
    {                          
        ComBuf[i] = RxUart0Buf[GetBufferIndex(bufIdx++)];    
        PrintNum(ComBuf[i]);    
    }                                 
    cs = CheckSum(ComBuf, vlvCnt*4 + 5, 1);        
    if (cs != RxUart0Buf[GetBufferIndex(bufIdx++)])
    {                  
        #ifdef DebugMode
        SendDebugMsg("\r\nwrong cs ");       
        PrintNum(cs);    
        #endif DebugMode
        return 0;
    }
/*    bufIdx = 5;
    for (i = 0; i < vlvCnt; i++)      
    {
        for (j = 0; j < 4; j++)
            newId.bVal[j] = ComBuf[bufIdx++];// RxUart0Buf[GetBufferIndex(bufIdx++)];    
        if ((newId.lVal > 500000) && (newId.lVal < 5000000))
        {
            //insert each valve  
            vlvIndex = InsertVlv(newId.lVal);
            if (vlvIndex == MAX_CMD)
                nNoSpaceCnt++;     
            else
                INorOUT[vlvIndex] = 1;    
        }
    }  */
    nNoSpaceCnt = FillList(INorOUT, vlvCnt);       
    #ifdef DebugMode
    SendDebugMsg("\r\n#valves no space for: ");      
    PrintNum(nNoSpaceCnt);
    #endif DebugMode  
//    if (bFirst)      
    {
        //remove all valves not in list                 
        if (DeleteNotinListVlv(INorOUT) > 0) 
            // if removed and there were valves that has no space before - insert it now
            if (nNoSpaceCnt > 0)
               FillList(INorOUT, vlvCnt);   //UpdateVlvList(false);       
    }
    // init to 0 all indexes with no valves   
    for (i = VCU1; i < MAX_CMD; i++)     
        if (INorOUT[i] == 0) 
            SaveVCUIdAtEeprom(0, i);
#endif NO_EZR
    return 1;
}

BYTE  DoCommand(BYTE cmd)
{
    switch (cmd)
    {
        case COMMAND_INIT_MEMORY:          
            return ResetPointers(); 
            break;    
        case COMMAND_MAKE_RESET:
            bMakeReset = TRUE;     
            break;
        case COMMAND_OPEN_PUMP:                   
            bTestPump = TRUE;     
            break;                                      
        case COMMAND_INIT_RESET:
        {                  
            ResetPointers();  
            bMakeReset = TRUE;  
            break;
        }
        case COMMAND_RTC_24:
            SetRtc24Hour();     
            break;         
        case COMMAND_GET_VLV_LST:   
            g_bSendList = TRUE;
        break;
        case ALL_VLV_COMMAND_PING:
        case ALL_VLV_COMMAND_RST:
        case ALL_VLV_COMMAND_TEST:    
        case ALL_VLV_COMMAND_STOP:
            g_lGlobalCmd = cmd + 9900;
            break;   
        default:
            return FALSE;                  
    }                    
    return TRUE;
}

#ifdef DebugMode
void PrintFlowData()
{       
    BYTE i;
    for (i = 0; i < 2; i++)
    {
        SendDebugMsg("\r\nFlow sensor data: \0");  
        SendDebugMsg("\r\nHighTreshold: \0");  
        PrintNum(FlowDef[i].HighTreshold);
        SendDebugMsg("\r\nLowTreshold: \0");  
        PrintNum(FlowDef[i].LowTreshold);     
        SendDebugMsg("\r\nHighFillingTime: \0");  
        PrintNum(FlowDef[i].HighFillingTime);   
        SendDebugMsg("\r\nLowFillingTime: \0");  
        PrintNum(FlowDef[i].LowFillingTime);   
        SendDebugMsg("\r\nHighStablingTime: \0");  
        PrintNum(FlowDef[i].HighStablingTime);   
        SendDebugMsg("\r\nLowStablingTime: \0");  
        PrintNum(FlowDef[i].LowStablingTime);   
    }
}

void PrintOprtList()
{
    BYTE i;    
    for (i = 0; i < numOprt; i++)
    {
        SendDebugMsg("\r\nOperator: \0");  
        PrintNum(OprtTbl[i]); 
        SendDebugMsg("access tech: \0");  
        putchar1(AccessTech[i]);   
        SendDebugMsg("\r\n");
    }    
}
#endif DebugMode

BYTE UpdateCBUMeta()
{
    BYTE i, dataLen, cs;
    #ifdef DebugMode
    SendDebugMsg("\r\nconfig CBU META\r\n"); 
    #endif DebugMode  
    dataLen = (MAX_PORTS_CBU * sizeof(_tagPortDefEEPROM)) + (2 * sizeof(_tagFlowDefEEPROM)) + CBU_GET_GLBL_CNFG_LEN;  
    for (i = 0; i < dataLen; i++)   
    {                          
        ComBuf[i] = RxUart0Buf[GetBufferIndex(bufIndexToUpd++)];    
//        PrintNum(ComBuf[i]);
    }                  
    cs = CheckSum(ComBuf, dataLen, 1);     
    if (cs != RxUart0Buf[GetBufferIndex(bufIndexToUpd++)])
    {                  
        #ifdef DebugMode
        SendDebugMsg("\r\nwrong cs ");       
        PrintNum(cs);    
        #endif DebugMode
        return 0;
    }   
    for (i = 0; i < 4; i++)
        if (ComBuf[4+i] != AppEepromData.eCbuGlblData[i])    //AppEepromData.eLoggerID[i])   
        {       
            #ifdef DebugMode
            SendDebugMsg("\r\nwrong CCU ID ");
            #endif DebugMode  
            return 0;
        }
 
    MemCopy_to_cpu_e2((char eeprom*)&AppEepromData.eCbuGlblData[8], (char*)&ComBuf[8],  CBU_SET_GLBL_CNFG_LEN);             
    bufIndexToUpd = CBU_GET_GLBL_CNFG_LEN; //9;    
    
    MemCopy_to_cpu_e2((char eeprom*)&ComponentArray, (char*)&ComBuf[bufIndexToUpd],  MAX_PORTS_CBU * sizeof(_tagPortDefEEPROM));   
    bufIndexToUpd += (MAX_PORTS_CBU * sizeof(_tagPortDefEEPROM));                       
    
    MemCopy_to_cpu_e2((char eeprom*)&FlowDef, (char*)(&ComBuf[bufIndexToUpd]), 2*sizeof(_tagFlowDefEEPROM));          
             
    #ifdef DebugMode
    PrintSensorDef();        
    PrintFlowData(); 
    #endif DebugMode
    SendRecRS485(CMD_SET_CBU_DEF , 0);          
    SendRecRS485(CMD_GET_CBU_DEF, 0);      
//    #ifdef DebugMode
//    PrintSensorDef();        
//    PrintFlowData(); 
//    #endif DebugMode   
    g_bSendCBUMeta = TRUE;
    return TRUE;
}

/*BYTE UpdateCBUMetaOld()
{
    BYTE i, dataLen, cs;
    #ifdef DebugMode
    SendDebugMsg("\r\nconfig CBU META"); 
    #endif DebugMode  
    dataLen = 3*sizeof(_tagPortDefEEPROM) + sizeof(_tagFlowDefEEPROM) + 9;
    for (i = 0; i < dataLen; i++)   
    {                          
        ComBuf[i] = RxUart0Buf[GetBufferIndex(bufIndexToUpd++)];    
//        PrintNum(ComBuf[i]);
    }                  
    cs = CheckSum(ComBuf, dataLen, 1);     
    if (cs != RxUart0Buf[GetBufferIndex(bufIndexToUpd++)])
    {                  
        #ifdef DebugMode
        SendDebugMsg("\r\nwrong cs ");       
        PrintNum(cs);    
        #endif DebugMode
        return 0;
    }   
    for (i = 0; i < 4; i++)
        if (ComBuf[i] != AppEepromData.eLoggerID[i])   
        {       
            #ifdef DebugMode
            SendDebugMsg("\r\nwrong CCU ID ");
            #endif DebugMode  
            return 0;
        }
 
    AppEepromData.eCbuGlblData[8] = ComBuf[4];//RxUart0Buf[GetBufferIndex(bufIndexToUpd++)];
//    AppEepromData.eDelta = Bytes2Float(&ComBuf[5]);//&RxUart0Buf[GetBufferIndex(bufIndexToUpd)]);
    bufIndexToUpd =  9;  
    MemCopy_to_cpu_e2((char eeprom*)&ComponentArray, (char*)&ComBuf[bufIndexToUpd],  3*sizeof(_tagPortDefEEPROM));   

    bufIndexToUpd += 3*sizeof(_tagPortDefEEPROM);  
    MemCopy_to_cpu_e2((char eeprom*)&FlowDef, (char*)(&ComBuf[bufIndexToUpd]), sizeof(_tagFlowDefEEPROM));          
                    
    #ifdef DebugMode
    PrintSensorDef();        
    PrintFlowData(); 
    #endif DebugMode
//    for (i = 0; i < 3; i++)     
//    {
//        SendRecRS485(CMD_SET_PORT_DEF, i);      
//    //        delay_ms(50);
//    }
//    SendRecRS485(CMD_GET_PORT_DEF, 0);      
//    SendRecRS485(CMD_SET_FLOW_DEF, 0);     
    SendRecRS485(CMD_SET_CBU_DEF, 0);    
    
    #ifdef DebugMode
    PrintSensorDef();        
    PrintFlowData(); 
    #endif DebugMode   
    g_bSendCBUMeta = TRUE;
    return TRUE;
}                */

char UpdateParam()
{
    char new_data_buf[32];
    BYTE n, dataLen;
    char res = TRUE;
    signed char x;           
    

    if ((prmUpdtIndex != UPDATE_VLV_LST) && (prmUpdtIndex != UPDATE_CBU_CNFG) && (prmUpdtIndex != UPDATE_CBU_CNFG2))     
    {
        dataLen = PrmLen[prmUpdtIndex];
        if (bufIndexToUpd + dataLen > MAX_RX_BUF_LEN)
        {
            for ( n = 0; n < dataLen; n++)
                new_data_buf[n] = RxUart0Buf[GetBufferIndex(bufIndexToUpd++)];
        }
        else
        {
            for ( n = 0; n < dataLen; n++)
                new_data_buf[n] = RxUart0Buf[bufIndexToUpd++];
        }        
    }

    switch (prmUpdtIndex)
    {
        case UPDATE_URL:     // Server_IP_URL  1
            {
//                if (bUpdateAddress)                   // if updating url & port - save new url and wait till the new port will come.
                    MemCopy(newURL, new_data_buf, 32);
//                else
//                    MemCopy_to_cpu_e2(AppEepromData.eIPorURLval1, new_data_buf, 32);  //len might varied
            }
            break;
        case UPDATE_PORT:      // Server_Port  2
            for (n = 0; n < 4; n++)
                if ((new_data_buf[n] < '0') || (new_data_buf[n] > '9'))  //if port is not a number
                    res = FALSE;  
                MemCopy(newPORT, new_data_buf, 4);
//            if (res == TRUE)
//            {                                          // if all 4 digits are numbers - replace port
//                if (bUpdateAddress)                                       // if updating url & port
//                    MemCopy_to_cpu_e2(AppEepromData.eIPorURLval1, newURL, 32);    // now replace the url - after make sure port comes ok
//                MemCopy_to_cpu_e2(AppEepromData.ePORTval1, new_data_buf, 4);
//            }
            break;
        case UPDATE_APN:     // Access_Point_Name  3
            MemCopy_to_cpu_e2(AppEepromData.eAPN, new_data_buf, 32);
        break;        
        case UPDATE_DL_START:        //Start_DL  7
            if ((new_data_buf[0] >= 0) &&(new_data_buf[0] <= 23))
                AppEepromData.eStartConnectionH = new_data_buf[0];
            else
                res = FALSE;
        break;
        case UPDATE_DL_CYCLE:        //DL_Cycles 8
            if ((new_data_buf[0] > 0) && (new_data_buf[0] <= 24))
                AppEepromData.eConnectionInDay = new_data_buf[0];
            else
                res = FALSE;
        break;
        case UPDATE_DL_INTERVAL:        //DL_Interval 9
            if ((new_data_buf[0] >= 0) && (new_data_buf[0] < 24))
                AppEepromData.eConnectIntervalH = new_data_buf[0];
            else
                res = FALSE;
        break;
        case UPDATE_COMMAND:       //reset memory (used to be GMT) 10 
            res = DoCommand(new_data_buf[0]);  
            if (g_lGlobalCmd == CODE_STOP_ALL)   
            {
                CloseMainPump(TRUE);  
                CloseAllValve(); 
            }
        break;
        case UPDATE_USE_CC:        //USE_COUNTRY_CODE    14
            if ((new_data_buf[0] == 0) || (new_data_buf[0] == 1))
                AppEepromData.eUseCntrCode = new_data_buf[0];
            else
                res = FALSE;
        break;
        case UPDATE_ROAMING_DLY:        //ROAMING_DELAY  15
            x = (signed char)(new_data_buf[0]);
            if ((x >= 0) && (x <= 60))
                AppEepromData.eRoamingDelay = new_data_buf[0];
            else
                res = FALSE;
        break;
        case UPDATE_MNC:         //MOBILE_NET_CODE   16
            MemCopy_to_cpu_e2(AppEepromData.eMobileNetCode, new_data_buf, 4);
        break;
        case UPDATE_MCC:         //MOBILE_COUNTRY_CODE  17
            MemCopy_to_cpu_e2(AppEepromData.eMobileCntryCode, new_data_buf, 4);
        break;
        case UPDATE_ID:          // sensors ID    18
            // save previous Logger ID for confirm
            cpu_e2_to_MemCopy( prevID, &AppEepromData.eLoggerID[0], 4);
            MemCopy_to_cpu_e2(&AppEepromData.eLoggerID[0], new_data_buf  ,4);
            // also make reset - so the new logger will register in db.
            bMakeReset = TRUE;
        break;
        case UPDATE_ENABLE_TMP_ALERT:  
            if (new_data_buf[0] == FALSE) 
                AppEepromData.eOpenPumpWithDealy/*.eEnableTmpAlert*/ = 0;
            else               
                AppEepromData.eOpenPumpWithDealy = 1;
        break;
        case UPDATE_TMP_ALERT_LMT:
        break; 
        case UPDATE_SOFTWARE_VER:
            fSwUpdate = bytes2int(new_data_buf);     
            // only if battery is full enough start with FOTA. otherwize - dont even confirm getting the TASK 
            //if (btrStatus != BTR_STATUS_FULL)
            if ((iVoltage < BTR_FULL_LIMIT) || (IsPumpBusy() == TRUE))
            {   
                fSwUpdate = 0;
                res = FALSE;
            }
        break;  
        case UPDATE_VLV_LST:       
            g_bVlvListUpdated = true;   
            res = UpdateVlvList();
        break; 
        case UPDATE_CBU_CNFG2:      
//            res = UpdateCBUMetaOld();
        break;  
        case UPDATE_CBU_CNFG:
            res = UpdateCBUMeta();   
            break;
        default:
            res = FALSE;
    }       
    return res;
}

BYTE IsOperatorExist(long lNewOp, BYTE nTotalNum )
{
    BYTE i;
    for (i = 0; i <= nTotalNum; i++)
        if (lNewOp == OprtTbl[i])
            return TRUE;
    return FALSE;
}

char GetSec()
{
    int index = 3, n;
    char x;
                    
    if (overFlow == 0)                       //if result was less than maxlength
        n = rx0_buff_len;    // max bytes to check is num of bytes got in
    else
        n = MAX_RX_BUF_LEN;

    while (index < n-2)
    {                  
        if ((RxUart0Buf[index] == 'G') && (RxUart0Buf[index+1] == 'M') && (RxUart0Buf[index+2] == 'T'))
        {
            x = 10 * (RxUart0Buf[index-3] - 0x30) + (RxUart0Buf[index-2] - 0x30);   
            if (x > 59) 
                x = 0;
            return x;  
        }
        else
            index++;
    }
    return 0;
}


BYTE IsSpecialCmd(int x)
{
    if ((x == CODE_PING) || (x == CODE_RST) || (x == CODE_VLV_TST) || (x == CODE_STOP_ALL)) 
        return 1;
    return 0;
}

#ifndef NO_EZR
BYTE ParseValveCommandsWithDelay(int index)        
{
    BYTE cs, len, nCnt = 0, i, nCmds;
    BYTE b[4];  
    unsigned long lID; 
    int duration; 
    unsigned int cmdIdx;
    
    bIsMoreVlvCmd = FALSE;
    nCmds = RxUart0Buf[GetBufferIndex(index)];   
    #ifdef DebugMode  
    putchar1('N');
    PrintNum(nCmds);        //
    #endif DebugMode  
 
    if (nCmds == 0)
        return 0;  
    if (nCmds > 10)
        return 0xFF;
    len = nCmds * CMD_LEN + 2;   //should be +2 calc length of data - length of each command (11)*num cmds + byte of #cmds + byte of is more  
     //copy content to temp buffer     
//     memcpy(ComBuf, &RxUart0Buf[GetBufferIndex(index++)], len+1);
    for (i = 0; i < len+1; i++) 
    {
        ComBuf[i] = RxUart0Buf[GetBufferIndex(index++)];    
        #ifdef DebugMode
        PrintOnlyNum(ComBuf[i]); 
        putchar1(',');  
        #endif DebugMode
    } 
    #ifdef DebugMode
    putchar1('\n');  
    #endif DebugMode
    cs = CheckSum(ComBuf, len, 1);
    
    //if wrong cs
    if (cs != ComBuf[len])  
    {          
        return 0xFF;            
    }     
    bIsMoreVlvCmd = ComBuf[len-1];      
    index = 1;   
    
    while (nCnt < nCmds)
    {      
        for (i = 0; i < 4; i++)
            b[i] = ComBuf[index+i];
//        index += 4;
        lID = Bytes2ULong(b);  
        #ifdef DebugMode  
        PrintNum(lID);     
        #endif DebugMode  
        //check ID. if its not possible ID(wrong) - skip it. 
        if ((lID < 500000) && (lID != 0))
            return 1;   //xFF;
           
        //duration
        b[0] =  ComBuf[index+4]; 
        b[1] =  ComBuf[index+5]; 
        duration = bytes2int(b);    
        // not less than 3 minutes (only 0 allowed - stop)
        if ((duration < 4) && (duration > 0))
            return 1;                                     
        // not more than 3 days irrigation (only 9999 allowed - ping)
        if ((duration > MAX_IRRIGATION_MNT) && (IsSpecialCmd(duration) == 0))
            return 1;   
                 
        b[0] =  ComBuf[index+10]; 
        b[1] =  ComBuf[index+11]; 
        cmdIdx = bytes2int(b);    
        
        #ifdef DebugMode  
        putchar1('D');                 
        PrintNum(duration);
        #endif DebugMode              
        if (lID == (long)0)
        {                    
            if ((duration == CODE_PUMP_TST))
                bTestPump = TRUE;     
            if (IsSpecialCmd(duration)) //((duration == CODE_PING) || (duration == CODE_RST) || (duration == CODE_VLV_TST)) 
                g_lGlobalCmd = duration;   
            if (g_lGlobalCmd == CODE_STOP_ALL)   
            {
                CloseMainPump(TRUE);  
                CloseAllValve(); 
            }
        }    
        else        
            InsertExtNewCmd(lID, duration, ComBuf[index+6], ComBuf[index+7], ComBuf[index+8],ComBuf[index+9], cmdIdx);    //id, on time, off time, cycles, time, index
        index += CMD_LEN;
        nCnt++;
    }    
    return nCnt;
}
#endif NO_EZR

/*BYTE GetRandomSec()
{ 
    BYTE res = ((AppEepromData.eLoggerID[0] + AppEepromData.eLoggerID[1] + AppEepromData.eLoggerID[2] + AppEepromData.eLoggerID[3]) % 60);
    res += GetSec();
    if (res > 60)
        res -= 60; 
    return res;
} */
// check  if "phy111" is in buffer:
int CheckResult()
{
    int i = 0, n;     
    BYTE cs;
    char tmpClock[5];
    int index;
    int nMaxBytesToCheck;
     #ifdef DebugMode
    SendDebugMsg("\r\nCheckResult \0");       
    PrintNum(rx0_buff_len);
    #endif DebugMode   
    if (overFlow == 0)                       //if result was less than 64 bytes
        nMaxBytesToCheck = /*lastByteIndex*/rx0_buff_len;    // max bytes to check is num of bytes got in
    else
        nMaxBytesToCheck = MAX_RX_BUF_LEN;
    //look for the string "phy111" to sign its begining of results:
    index = -1;        
   
    do
    {
        if (overFlow == 0)                       //if result was less than 64 bytes
            n = i;
        else
           n = rx0_buff_len + i;
        if((RxUart0Buf[GetBufferIndex(n)] == PHYTECH_FILE_END_MARK[0]) &&       // P
        (RxUart0Buf[GetBufferIndex(n+1)] == PHYTECH_FILE_END_MARK[1]) &&        // h
        (RxUart0Buf[GetBufferIndex(n+2)] == PHYTECH_FILE_END_MARK[2]) &&        // y
        (RxUart0Buf[GetBufferIndex(n+3)] == PHYTECH_FILE_END_MARK[3]) &&        // 1
        (RxUart0Buf[GetBufferIndex(n+4)] == PHYTECH_FILE_END_MARK[4]) &&        // 1
        (RxUart0Buf[GetBufferIndex(n+5)] == PHYTECH_FILE_END_MARK[5]))          // 1
            index = GetBufferIndex(n+6);
        i++;
    }
    while ((i < nMaxBytesToCheck - 5) && (index == -1));

    // if hasnt find - return -1, means no data got back

    if (index == -1)
    {
        #ifdef DebugMode
        SendDebugMsg("\r\nPOST not OK1\0");  
        #endif DebugMode
        return -1;
    }
    bPostAnswered = TRUE;    // sign that post answered - even if failed - its an answer. no need to open different socket. connection is OK.
    // if found the sign of "phy111"- look on next bytes
    // if it "FAILED" string - then it means webservice return failed getting the parameters
    // eelse it is the begining of parameters sent from service, return the index

    i = index;
    if ((RxUart0Buf[GetBufferIndex(i)] == 0x46) &&          // F
        (RxUart0Buf[GetBufferIndex(i+1)] == 0x41) &&        // A
        (RxUart0Buf[GetBufferIndex(i+2)] == 0x49) &&        // I
        (RxUart0Buf[GetBufferIndex(i+3)] == 0x4c) &&        // L
        (RxUart0Buf[GetBufferIndex(i+4)] == 0x45) &&        // E
        (RxUart0Buf[GetBufferIndex(i+5)] == 0x44))          // D
        {
            #ifdef DebugMode
            SendDebugMsg("\r\nPOST not OK2\0");
            #endif DebugMode

            return -1;
        }
                  
    switch (modemCurSubTask)
    {
    // if its parameters post - save the amswer
    case SUB_TASK_MODEM_POST_PRM:
    {         
        for (i = 0; i < PRM_ANSWER_LEN; i++)         
        {                    
            ComBuf[i] = RxUart0Buf[GetBufferIndex(n++)];    
 //       PrintNum(ComBuf[i]);       
        }
                                   
        cs = CheckSum(ComBuf, PRM_ANSWER_LEN, 1);        
        if (cs != RxUart0Buf[GetBufferIndex(n)])  
        {  
            #ifdef DebugMode
            SendDebugMsg("\r\nWrong prm CS\0");   
            PrintNum(RxUart0Buf[GetBufferIndex(n)]);
            PrintNum(cs);
            #endif DebugMode
            return -1;
        }
        i = 6;
    ///////////////////////////
        for (n = 0; n < 5; n++)
            tmpClock[n] = ComBuf[i++];//RxUart0Buf[GetBufferIndex(i++)];
        if (!((tmpClock[0] == 0) && (tmpClock[1] == 0) && (tmpClock[2] == 0) && (tmpClock[3] == 0) && (tmpClock[4] == 0)))
        {
            MemCopy( clockBuf, &tmpClock[0], 3 ); //copy year month day
            MemCopy( &clockBuf[4], &tmpClock[3], 2 );  //copy hour minute    
            clockBuf[6] =  GetSec();  

            if(SetRealTime() == FAILURE)
            {
                //
                #ifdef DebugMode
                SendDebugMsg("\r\nSetRealTimeFailed\0");//set real time
                #endif DebugMode
            }
        }
        #ifdef DebugMode
        else
            SendDebugMsg("\r\nskip update clock\0");//set real time
        #endif DebugMode

        //i += 5;
        for ( n = 0; n < MAX_PRM_TASKS; n++)
            UpdatePrmArr[n] = ComBuf[i++];//RxUart0Buf[GetBufferIndex(i++)];

        if ((UpdatePrmArr[1] == '1') && (UpdatePrmArr[2] == '1'))
            bUpdateAddress = TRUE;
        else
            bUpdateAddress = FALSE;
        bMakeReset = FALSE; // init flag of make reset      
        bTestPump = FALSE; 
    }
    break;
    case SUB_TASK_MODEM_POST_UPD:
    {
        bufIndexToUpd = index;
        if (UpdateParam() != TRUE)
            index = -1;
    }
    break;
    // check if there is ok after phy111
    case SUB_TASK_MODEM_POST_DATA:
    case SUB_TASK_MODEM_POST_CBU:    
//    case SUB_TASK_MODEM_POST_PUMP:   
    {
        i = index;
        // first - save clock
        for (n = 0; n < 5; n++)
            tmpClock[n] = RxUart0Buf[GetBufferIndex(i++)];
        if (!((tmpClock[0] == -1) && (tmpClock[1] == -1) && (tmpClock[2] == -1) && (tmpClock[3] == -1) && (tmpClock[4] == -1)))
        {
            MemCopy( clockBuf, &tmpClock[0], 3 );       //update year month day
            MemCopy( &clockBuf[4], &tmpClock[3], 2 );   //update hour minute
            clockBuf[6] = GetSec();
        }
        // check if there is PENDING
        if ((RxUart0Buf[GetBufferIndex(i)] == 0x50) &&      //P
        (RxUart0Buf[GetBufferIndex(i+1)] == 0x45) &&        //E
        (RxUart0Buf[GetBufferIndex(i+2)] == 0x4e) &&        //N
        (RxUart0Buf[GetBufferIndex(i+3)] == 0x44) &&        //D
        (RxUart0Buf[GetBufferIndex(i+4)] == 0x49) &&        //I
        (RxUart0Buf[GetBufferIndex(i+5)] == 0x4E) &&        //N
        (RxUart0Buf[GetBufferIndex(i+6)] == 0x47) )         //G
        {
            bSendPrms = DO_PARAMS;//DO_DATA_N_PRMS;
            i += 7;      
            #ifdef DebugMode   
                SendDebugMsg("\r\nadd params 3\0");   
                #endif DebugMode   
        }
        // now check for OK
        if (!((RxUart0Buf[GetBufferIndex(i)] == 0x4f) && (RxUart0Buf[GetBufferIndex(i+1)] == 0x4b)))         // OK
        {
            #ifdef DebugMode
            SendDebugMsg("\r\nPOST not OK3\0");
            #endif DebugMode
            return -1;
        }
    }
    break;
#ifndef NO_EZR
    case SUB_TASK_MODEM_POST_VLV:    
        nValveCmdCnt = ParseValveCommandsWithDelay(index);   //ParseValveCommands(index);//   
        #ifdef DebugMode
        SendDebugMsg("\r\nparse vlv cmd\0"); 
        PrintNum(nValveCmdCnt);
        #endif DebugMode
        if (nValveCmdCnt == 0xFF)//    break;    
            return -1;           
        break;   
#endif NO_EZR
    case SUB_TASK_MODEM_POST_CBU_META:     
        g_bSendCBUMeta = FALSE;
    break;          
    case SUB_TASK_MODEM_POST_VCU_LST:
    break;
    default:      
    }
    #ifdef DebugMode
    SendDebugMsg("\r\nPOSTOK\0");
    #endif DebugMode

    return index;
}

/*BYTE Is4dModem()
{
    int index = 0;
    while (index < rx0_buff_len-8)
    {
        if ((RxUart0Buf[index] == MODEM_4D_MODEL[0]) && (RxUart0Buf[index+1] == MODEM_4D_MODEL[1]) &&
        (RxUart0Buf[index+2] == MODEM_4D_MODEL[2]) && (RxUart0Buf[index+3] == MODEM_4D_MODEL[3]) &&
        (RxUart0Buf[index+4] == MODEM_4D_MODEL[4]) && (RxUart0Buf[index+5] == MODEM_4D_MODEL[5]) &&
        (RxUart0Buf[index+6] == MODEM_4D_MODEL[6]) && (RxUart0Buf[index+7] == MODEM_4D_MODEL[7]) &&
        (RxUart0Buf[index+8] == MODEM_4D_MODEL[8]))
        {
            return TRUE;
        }
        else
            index++;
    }
    return FALSE;
}          */

void InitVersiontoUpdt()
{
    BlEepromData.versionUpdate = 0;
}

void ParseModemResponse()
{
    ModemResponse = TASK_FAILED;  
    switch (modemCurSubTask)
    {
        case SUB_TASK_INIT_MODEM_OK:
        case SUB_TASK_INIT_MODEM_COPS:
        case SUB_TASK_INIT_MODEM_COPS_MAN:
        case SUB_TASK_INIT_MODEM_RSSI:
        case SUB_TASK_MODEM_CONNECT_ATCH:
//        case SUB_TASK_MODEM_CONNECT_SETUP1:
//        case SUB_TASK_MODEM_CONNECT_SETUP2:
//        case SUB_TASK_MODEM_CONNECT_SETUP3:
        case SUB_TASK_MODEM_CONNECT_PDP_DEF:
        case SUB_TASK_MODEM_CONNECT_ACTV:
        case SUB_TASK_MODEM_CONNECT_DEACTV:
        case SUB_TASK_MODEM_CLOSE_EOD:
        case SUB_TASK_MODEM_CLOSE_TCP:
        case SUB_TASK_INIT_MODEM_COPS_4_MONITOR:  
        case SUB_TASK_MODEM_CONNECT_IS_ACTIVE:
            if (IsOK() == TRUE)
            {
                ModemResponse = TASK_COMPLETE;
                if (modemCurSubTask == SUB_TASK_INIT_MODEM_RSSI)
                    ConvertRssiVal();
            }
//            else
//                ModemResponse = TASK_FAILED;
        break;
        case SUB_TASK_MODEM_CLOSE_MDM:   
            if (IsPowerDown())     
                ModemResponse = TASK_COMPLETE;
            else     
                if (IsOK() == TRUE)   
                {     
                    ModemResponse = NO_ANSWER;  
                    bWaitForModemAnswer = TRUE;
                }              
        break;
        case SUB_TASK_INIT_MODEM_GET_COPS:        
            if (IsOK() == TRUE)
            {         
                IsVerizon();
                ModemResponse = TASK_COMPLETE;    
            }
        break;
        case SUB_TASK_INIT_MODEM_COPS_LST:
            if ((IsOK() == TRUE) || (curOprtIndex > 0))
            {
                ModemResponse = TASK_COMPLETE;    
                fGetCopsLst = TRUE;
            }
            else
            {
//                ModemResponse = TASK_FAILED;
                fMdmAns = TRUE;
            }
            
        break;
        case SUB_TASK_INIT_MODEM_REG:
        case SUB_TASK_INIT_MODEM_REG_STAT:
        case SUB_TASK_INIT_MODEM_REG_LTE:
            if (IsRegistOK() == TRUE)
                ModemResponse = TASK_COMPLETE;
//            else
//                ModemResponse = TASK_FAILED;
        break;
        case SUB_TASK_INIT_MODEM_QSS:
            if ((IsOK() == TRUE) && (IsSimDetected() == TRUE))
                ModemResponse = TASK_COMPLETE;
//            else
//                ModemResponse = TASK_FAILED;

        break;
//        case SUB_TASK_MODEM_CGMM:
//            if (IsOK() == TRUE)
//            {                          
//                ModemResponse = TASK_COMPLETE;
//            }
//        break;
        case SUB_TASK_MODEM_IGN_OFF: //SUB_TASK_MODEM_READY:     
            if (IsReady())  
            {     
                ModemResponse = TASK_COMPLETE;  
                HandleLed(LED_2, CBU_LED_BLINK); 
                #ifdef DebugMode
            SendDebugMsg("\r\nGot ready\0");
            #endif DebugMode  
            }
//            else
//                ModemResponse = TASK_FAILED;
        break;
        case SUB_TASK_INIT_MODEM_MONITOR:
            GetCellID();
            ModemResponse = TASK_COMPLETE;
        break;
        case SUB_TASK_MODEM_CHK_ICCID:
            GetICCID();
            ModemResponse = TASK_COMPLETE;
        break;
        case SUB_TASK_MODEM_CONNECT_START_DIAL:
            if (IsConnect() == TRUE)
            {                           
                HandleLed(LED_3, CBU_LED_ON);
                ModemResponse = TASK_COMPLETE;
                g_bModemConnect = TRUE;
//                nConnectError = 0;
            }
//            else
//                ModemResponse = TASK_FAILED;
        break;
        case SUB_TASK_MODEM_POST_PRM:
        case SUB_TASK_MODEM_POST_UPD:
        case SUB_TASK_MODEM_POST_CNFRM:
        case SUB_TASK_MODEM_POST_DATA:
        case SUB_TASK_MODEM_POST_VLV:      
        case SUB_TASK_MODEM_POST_CBU:   
        case SUB_TASK_MODEM_POST_CBU_META:  
        case SUB_TASK_MODEM_POST_ALERT:   
        case SUB_TASK_MODEM_POST_PUMP: 
        case SUB_TASK_MODEM_POST_VCU_LST:
            if (CheckResult() != -1)
//                ModemResponse = TASK_FAILED;
//            else
                ModemResponse = TASK_COMPLETE;
        break;
        case SUB_TASK_MODEM_GET_SW_UPDATE:
            if (IsOK() == TRUE)   
            {                               
              #ifdef DebugMode
            SendDebugMsg("\r\nGot OK for update ver\0");
            #endif DebugMode  
                ModemResponse = TASK_COMPLETE;                  
                            
                if  (nFw2Upg == 0)   
                {       
                    fSwUpdate = 0;
                    break;
                }
                if (fSwUpdate == 1)
                {
                    BlEepromData.versionUpdate = nFw2Upg;  
                    #ifdef DebugMode
                   SendDebugMsg("\r\nnew ver:\0");  
                   PrintNum(nFw2Upg);
                    #endif DebugMode
                }
                if (fSwUpdate == 2)
                {
                    nEzrFw2Upg = nFw2Upg;
                    #ifdef DebugMode
                    putchar1('*');
                    putchar1(nEzrFw2Upg);
                    putchar1('*'); 
                    #endif DebugMode
                }
                if (fSwUpdate == 3)
                {       
                    nCBUFw2Upg = nFw2Upg;
                }
 
            }
            else                      
            {
//                ModemResponse = TASK_FAILED;    
                fSwUpdate = 0;
            }
        
        break;
    }
}

//convert int less than 1000000 to string and send string to uart1
int Long2Str(long val, char* s)
{
    char tmp[10];
    BYTE i = 0, n = 0;
    do
    {
        tmp[i++] = (char)(val % 10);
        val = val / 10;
    }
    while (val > 0);
    for (; i > 0; i--)
        s[n++] = tmp[i-1] + 48;
    return n;
}

void SendOperatorSelection()
{
    BYTE i, n;
    //build the cops command  from country+network codes
    n = CopyFlashToBuf(ComBuf, AT_COPS_MAN);
    ComBuf[n++] = '"';
    if (AppEepromData.eUseCntrCode == 1)
    {
        i = 0;
        while (AppEepromData.eMobileCntryCode[i] != '#')    //HASHTAG)
        {
            ComBuf[i+n] = AppEepromData.eMobileCntryCode[i];
            i++;
        }
        n += i;
        i = 0;
        while (AppEepromData.eMobileNetCode[i] != '#')  //HASHTAG)
        {
            ComBuf[i+n] = AppEepromData.eMobileNetCode[i];
            i++;
        }
    }
    else
    {
        i = Long2Str(OprtTbl[curOprtIndex], &ComBuf[n]);
    }
    n += i;
    ComBuf[n++] = '"';
    ComBuf[n++] = '\r';
    ComBuf[n++] = '\n';
    BytesToSend = n;
    TransmitBuf(0);
}

void SendStartDial()
{
    BYTE i, n;
    // if connect server failed few times, maybe the address is wrong, - change IP/url address
//     if (fModemModel == MODEM_GE)
    //build the cops command  from country+network codes
        n = CopyFlashToBuf(ComBuf, AT_TCP_OPN);
//    else
//        n = CopyFlashToBuf(ComBuf, AT_TCP_OPN_LTE);
          
    i = 0;
          
    ComBuf[n++] = '"';
    if (fSwUpdate == 0)
        while ((AppEepromData.eIPorURLval1[i] != '#') && (i < 32))
        {
            ComBuf[i+n] = AppEepromData.eIPorURLval1[i];
            i++;
        }     
    else
//        if (fSwUpdate == 1)
            i = CopyFlashToBuf(&ComBuf[n],fSWUpdateAddress);
//        else
//            if (fSwUpdate == 2)
//                i = CopyFlashToBuf(&ComBuf[n],fEZRUpdateAddress);             
    n += i;
    ComBuf[n++] = '"';
    ComBuf[n++] = ',';
       
    if (fSwUpdate == 0) 
    {        
        cpu_e2_to_MemCopy(&ComBuf[n], AppEepromData.ePORTval1, 4);
        n += 4;   
    }   
    else        
    {        
        i = CopyFlashToBuf(&ComBuf[n], fSWUpdatePort);  
        n += i;   
    }   
    ComBuf[n++] = ',';
    ComBuf[n++] = '0';
    ComBuf[n++] = ',';
    ComBuf[n++] = '2';
    ComBuf[n++] = '\r';
    ComBuf[n++] = '\n';
    BytesToSend = n;
    TransmitBuf(0);
}

void SendPDPCntDef()
{
    BYTE i, n;
    //build the cops command  from country+network codes
    if (AccessTech[curOprtIndex] != '7')//(g_RAN == RAN_3G)
        n = CopyFlashToBuf(ComBuf, DEF_PDP_CNTXT);
    else
        n = CopyFlashToBuf(ComBuf, DEF_PDP_CNTXT_VZN);

    ComBuf[n++] = '"';
    i = 0;
    while ((AppEepromData.eAPN[i] != '#') && (i < 32))
    {
        ComBuf[i+n] = AppEepromData.eAPN[i];
        i++;
    }
    n += i;
    ComBuf[n++] = '"';
    ComBuf[n++] = '\r';
    ComBuf[n++] = '\n';
    BytesToSend = n;
    TransmitBuf(0);
}

void GetNextPost()
{
       
    #ifdef SMART_SERVER  
    if ( modemCurSubTask != SUB_TASK_MODEM_POST_CBU)        
        modemCurSubTask = SUB_TASK_MODEM_POST_CBU;
    else
    {
        modemCurTask = TASK_MODEM_CLOSE;
        modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;             
    } 
     return;   
    #endif  SMART_SERVER
    if ((fSwUpdate >= 1) && (fSwUpdate <= 3))
    {        
        modemCurSubTask = SUB_TASK_MODEM_GET_SW_UPDATE;  
        nFw2Upg = 0;   
        return;
    }                    
    post2send = GetNextEprom2Send(post2send);       
    #ifdef DebugMode
    SendDebugMsg("\r\nNEXT EEPROM to send: ");   
    PrintNum(post2send);
    #endif DebugMode   
    
//    if ((post2send != POST_NONE) )&& (modemCurSubTask != (70 + post2send)))   
    if ((post2send >= POST_ALERT) && (post2send <= POST_PUMP_ACT))
        modemCurSubTask = 70 + post2send;      
    else                    
        if ((g_bSendCBUMeta == TRUE))    
        {
                modemCurSubTask = SUB_TASK_MODEM_POST_CBU_META;      
                g_bSendCBUMeta = FALSE;
        }
        else                         
            if (g_bSendList == TRUE)  
            {
                modemCurSubTask = SUB_TASK_MODEM_POST_VCU_LST;      
                g_bSendList = FALSE;
            }        
#ifndef NO_EZR
            else
                if (modemCurSubTask != SUB_TASK_MODEM_POST_VLV)
                    modemCurSubTask = SUB_TASK_MODEM_POST_VLV;    
#endif NO_EZR
                else         
                    if (bSendPrms == DO_PARAMS) 
                    {
                        modemCurSubTask = SUB_TASK_MODEM_POST_PRM;  
                        bSendPrms = 0;//&= !DO_PARAMS;
                    }
                    else
                    {
                        modemCurTask = TASK_MODEM_CLOSE;
                        modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;             
                    } 
 //   Myprintf("NEXT post to send: %d \0", modemCurSubTask);              
}

BYTE GetNextTask()
{
    if (nTimeCnt > 0)
        return WAIT;

    // first task-
    if (modemCurTask == TASK_NONE)
    {
        modemCurTask = TASK_MODEM_INIT;
        if (IsModemOn())
            modemCurSubTask = SUB_TASK_INIT_MODEM_OK;
        else
            modemCurSubTask = SUB_TASK_MODEM_IGN_ON;
        initCnt = 0;
        return  CONTINUE;
    }                                
    if ((bWaitForModemAnswer == TRUE) && (TimeLeftForWaiting == (int)0))     
    {
        bWaitForModemAnswer = FALSE;
        ModemResponse = TASK_FAILED_NO_ANSWER;//TASK_FAILED;
    }
    // if flag of end of rx received is on
    if (bCheckRxBuf == TRUE)
    {
        bCheckRxBuf = FALSE;
        ParseModemResponse();
        delay_ms(20); //500
    }             
    else
        if  ((bWaitForModemAnswer == TRUE) && (TimeLeftForWaiting > (int)0))
            return WAIT;   
    
    if (ModemResponse == NO_ANSWER)
        return WAIT;
    if (g_bExtIntDtct == TRUE)
    {                        
        prevMainTask = TASK_MODEM;
        mainTask = TASK_WAKEUP;    
        return WAIT;
    }
                           
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
                        case SUB_TASK_MODEM_IGN_ON:    
                          
                             modemCurSubTask = SUB_TASK_MODEM_IGN_OFF;//SUB_TASK_DELAY;
                        break;
                        case SUB_TASK_MODEM_IGN_OFF:   
                                modemCurSubTask = SUB_TASK_INIT_MODEM_OK;
                        break;  
//                        case SUB_TASK_MODEM_CGMM:   //SUB_TASK_INIT_MODEM_OK:
//                            if (g_bModemConnect == FALSE)//(bExtReset == TRUE)
//                            {
//                                modemCurSubTask = SUB_TASK_INIT_MODEM_QSS;
////                                TurnOnLed(LED_2, LED_BLINK);
//                            }
//                            else
//                            {                                  
//                                if (AccessTech[curOprtIndex] == '7')
//                                    taskAfterDelay = SUB_TASK_INIT_MODEM_REG_LTE; 
//                                else
//                                    taskAfterDelay = SUB_TASK_INIT_MODEM_REG;
//                                modemCurSubTask = SUB_TASK_DELAY;
//                                nTimeCnt = 100; //delay before creg
//                            }
//                        break;
                        case SUB_TASK_INIT_MODEM_OK:                            
                            modemCurSubTask = modemCurSubTask = SUB_TASK_INIT_MODEM_QSS;//SUB_TASK_MODEM_CGMM;  
                        break;
                        case  SUB_TASK_INIT_MODEM_QSS:
                            modemCurSubTask = SUB_TASK_MODEM_CHK_ICCID;
                        break;
                        case  SUB_TASK_MODEM_CHK_ICCID: 
                            modemCurSubTask = SUB_TASK_INIT_MODEM_COPS;
                        break;
                        case  SUB_TASK_INIT_MODEM_COPS:           
                            if ((bExtReset == TRUE) && (bEndOfMonitorTask == FALSE) /*&& (fModemModel == MODEM_GE)*/)
                                modemCurSubTask = SUB_TASK_INIT_MODEM_COPS_4_MONITOR;
                            else
                                if (AppEepromData.eUseCntrCode == 1)
                                    modemCurSubTask = SUB_TASK_INIT_MODEM_COPS_MAN;
                                else
//                                    if (fModemModel == MODEM_GE)  
                                    #ifdef ISRAEL_QUICK_CONNECT
                                    modemCurSubTask = SUB_TASK_INIT_MODEM_COPS_4_MONITOR;  
                                    #else           
                                    if ((fGetCopsLst == TRUE) && (curOprtIndex < numOprt))
                                    {
                                        curOprtIndex++;  
                                        modemCurSubTask = SUB_TASK_INIT_MODEM_COPS_MAN;       
                                    }            
                                    else
                                        modemCurSubTask = SUB_TASK_INIT_MODEM_COPS_LST;//SUB_TASK_INIT_MODEM_COPS_4_MONITOR;   //       
                                    #endif
//                                    else
//                                    {
//                                        modemCurSubTask = SUB_TASK_DELAY;
//                                        nTimeCnt = 100; //delay before creg
//                                        taskAfterDelay = SUB_TASK_INIT_MODEM_REG_STAT;
//                                    }
                        break;
                        case SUB_TASK_INIT_MODEM_COPS_LST:     
                            if (numOprt > 0)              // if found at list 1 operator 
                            {
                                modemCurSubTask = SUB_TASK_INIT_MODEM_COPS_MAN; //SUB_TASK_INIT_MODEM_COPS_4_MONITOR;   //;  
                                PrintOprtList();
                            }
                            else
                                {
                                    modemCurTask = TASK_MODEM_CLOSE;
                                    modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                                }
                        break;
                        case SUB_TASK_INIT_MODEM_COPS_MAN:
                            modemCurSubTask = SUB_TASK_DELAY;
                            nTimeCnt = 100;                                           
                            if (AccessTech[curOprtIndex] == '7')
                                taskAfterDelay = SUB_TASK_INIT_MODEM_REG_LTE; 
                            else
                                taskAfterDelay = SUB_TASK_INIT_MODEM_REG;//SUB_TASK_INIT_MODEM_REG_STAT;
//                            taskAfterDelay = SUB_TASK_INIT_MODEM_REG_LTE;//SUB_TASK_INIT_MODEM_REG;
                        break;        
                        case SUB_TASK_INIT_MODEM_COPS_4_MONITOR: 
                            modemCurSubTask = SUB_TASK_INIT_MODEM_REG;
                        break;
                        case SUB_TASK_INIT_MODEM_REG:         
                            if (AccessTech[curOprtIndex] == '7')
                                modemCurSubTask = SUB_TASK_INIT_MODEM_REG_LTE; 
                            else
                                modemCurSubTask = SUB_TASK_INIT_MODEM_REG_STAT;
                        break;
                        case SUB_TASK_INIT_MODEM_REG_STAT:
//                            if (fModemModel == MODEM_SVL)   //(modemLTEModel == TRUE)
//                                modemCurSubTask = SUB_TASK_INIT_MODEM_REG_LTE;
//                            else
                                modemCurSubTask = SUB_TASK_INIT_MODEM_GET_COPS;
                        break;
                        case SUB_TASK_INIT_MODEM_REG_LTE:
                            modemCurSubTask = SUB_TASK_INIT_MODEM_GET_COPS;
                        break;
                        case SUB_TASK_INIT_MODEM_GET_COPS:  
//                            TurnOnLed(LED_2, LED_ON);         
//                            TurnOnLed(LED_3, LED_BLINK);
                            modemCurSubTask = SUB_TASK_INIT_MODEM_MONITOR;
                        break;
                        case SUB_TASK_INIT_MODEM_MONITOR:
                            modemCurSubTask = SUB_TASK_INIT_MODEM_RSSI;
                        break;
                        case SUB_TASK_INIT_MODEM_RSSI: //todo: check rssi and continue only if it over min
                            if (g_bModemConnect)
                            {
                                modemCurTask = TASK_MODEM_CONNECT;       
                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_START_DIAL;      
                            }           
                            else
                            {
                                modemCurTask = TASK_MODEM_CONNECT;
                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_ATCH;
                            }
                        break;
                        case SUB_TASK_DELAY:
                            if (nTimeCnt <= 0)
                                modemCurSubTask = taskAfterDelay;
                        break;
                        default:
                            modemCurTask = TASK_MODEM_CLOSE;
                            modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                        break;
                    }
                break;
                case TASK_MODEM_CONNECT:
                    switch (modemCurSubTask)
                    {
                        case SUB_TASK_MODEM_CONNECT_ATCH:
//                            if (bExtReset == FALSE)
//                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_DEACTV; //SUB_TASK_MODEM_CONNECT_ACTV;
//                            else
                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_PDP_DEF;   
                        break;
//                        case SUB_TASK_MODEM_CONNECT_SETUP1:
//                            modemCurSubTask = SUB_TASK_MODEM_CONNECT_SETUP2;
//                        break;
//                        case SUB_TASK_MODEM_CONNECT_SETUP2:
//                            modemCurSubTask = SUB_TASK_MODEM_CONNECT_DEACTV;
//                        break;
                        case SUB_TASK_MODEM_CONNECT_PDP_DEF:
                            modemCurSubTask = SUB_TASK_MODEM_CONNECT_IS_ACTIVE;    //SUB_TASK_MODEM_CONNECT_SETUP1;
                        break;            
                        case SUB_TASK_MODEM_CONNECT_IS_ACTIVE:  
                            if (IsPDPActive() == TRUE)
                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_START_DIAL;
                            else
                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_ACTV;     
                        break;
                        case SUB_TASK_MODEM_CONNECT_DEACTV:      
                            modemCurSubTask = SUB_TASK_MODEM_CONNECT_ACTV;
                        break;
                        case SUB_TASK_MODEM_CONNECT_ACTV:  
                              modemCurSubTask = SUB_TASK_MODEM_CONNECT_START_DIAL;
                        break;
                        case SUB_TASK_MODEM_CONNECT_START_DIAL:   
                            bConnectOK = TRUE;
                            // save last connecting time 
                            GetRealTime();
                            g_LastCnctTime = g_curTime;  
                            nErrorOnConnect = 0; 
                            // if its connecting without logger ID (monitor) - shutdown now 
                            if (IsZeroID(AppEepromData.eLoggerID))
                            {
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;  
                            }  
                            else
                            {
                                modemCurTask = TASK_MODEM_POST;       
                                GetNextPost();                                         
                            }
                        break;
                        default:
                            modemCurTask = TASK_MODEM_CLOSE;
                            modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                        break;
                    }
                break;
                case TASK_MODEM_POST:     
//                    fConnectOK = 1;
                    switch (modemCurSubTask)
                    {
                        case SUB_TASK_MODEM_POST_DATA:        
                        case SUB_TASK_MODEM_POST_CBU:
                        case SUB_TASK_MODEM_POST_ALERT:   
                        case SUB_TASK_MODEM_POST_PUMP:    
                            InitWritPntr(modemCurSubTask - 70);
                            GetNextPost(); 
                        break;
                        case SUB_TASK_MODEM_POST_CBU_META:   
                        case SUB_TASK_MODEM_POST_VCU_LST:
                            GetNextPost();
                        break;
                        case SUB_TASK_MODEM_POST_PRM:
                        case SUB_TASK_MODEM_POST_CNFRM:
                            if ((modemCurSubTask == SUB_TASK_MODEM_POST_CNFRM) && (bUpdateAddress))  // if updating url & port
                            {
                                if (prmUpdtIndex == UPDATE_URL)      // if finish to confirm new url,
                                {
                                    prmUpdtIndex = UPDATE_PORT;       // now confirm port
                                    break;
                                }
                                else
                                    if (prmUpdtIndex == UPDATE_PORT)          // if finish to confirm new port  - now switch to new address 
                                    {
                                        bUpdateAddress = FALSE;     //reset flag       
                                        MemCopy_to_cpu_e2(AppEepromData.eIPorURLval1, newURL, 32);    // now replace the url - after make sure port comes ok
                                        MemCopy_to_cpu_e2(AppEepromData.ePORTval1, newPORT, 4);  
                                    }
                            }         
                            else
                            {   
                                if (prmUpdtIndex == UPDATE_URL)  // after finish confirm to old url - swap to new   
                                    MemCopy_to_cpu_e2(AppEepromData.eIPorURLval1, newURL, 32);                                       
                            }
#ifndef NO_EZR
                            if ((modemCurSubTask == SUB_TASK_MODEM_POST_CNFRM) && (prmUpdtIndex == UPDATE_VALVE_CMD) && (bIsMoreVlvCmd == TRUE))
                            {     
                                modemCurSubTask = SUB_TASK_MODEM_POST_VLV;
                            }
                            else
#endif NO_EZR
                                // if there is more update to do
                                if (IsPrmToUpdate() == TRUE)
                                    modemCurSubTask = SUB_TASK_MODEM_POST_UPD;
                                else  // if no updates to do
                                {
                                    prmSentOK = TRUE;
                                    modemCurTask = TASK_MODEM_CLOSE;
                                    modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;
                                }
                        break;
                        case SUB_TASK_MODEM_POST_UPD:
                            if ((bUpdateAddress) && (prmUpdtIndex == UPDATE_URL))   // if updating url & port: and finish to get new url,
                                prmUpdtIndex = UPDATE_PORT;                          // right after get the new url - send post to get port (before confirm)
                            else
                            {
                                modemCurSubTask = SUB_TASK_MODEM_POST_CNFRM;
                                if ((bUpdateAddress) && (prmUpdtIndex == UPDATE_PORT))   // if updating url & port: and finish to get new port,
                                    prmUpdtIndex = UPDATE_URL;                          // now send the confirm to url (and later to port)
                            }
                        break; 
                        case SUB_TASK_MODEM_POST_VLV:         
                            if (nValveCmdCnt > 0)
                            {
                                modemCurSubTask = SUB_TASK_MODEM_POST_CNFRM; 
                                prmUpdtIndex = UPDATE_VALVE_CMD;   // UPDATE_COOLING_CMD;
                            }        
                            else      
                                if (bSendPrms == DO_PARAMS) 
                                {
                                    modemCurSubTask = SUB_TASK_MODEM_POST_PRM;  
                                    bSendPrms = 0;//&= !DO_PARAMS;
                                }
                                else
                                {
                                    modemCurTask = TASK_MODEM_CLOSE;
                                    modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;  
                                }
                            vlvSentOK = TRUE;
                        break;            
                            //prmSentOK = TRUE;     
//                           GetNextPost();
//                            modemCurTask = TASK_MODEM_CLOSE;
//                            modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;                        
//                        break;
                        case SUB_TASK_MODEM_GET_SW_UPDATE:                             
                            //if EZR firmware update
                            if ((fSwUpdate == 2) && (nEzrFw2Upg > 0))             
                            {        
                                ResetEZR(); 
                                DISABLE_CLOCK_INT();  //RESET_CLOCK_INT();
                                PROTOCOL_Init(); 
                                //send EZR  download_firmware command   
                                if (PROTOCOL_Task(PROTO_CMD_START_FWUPGRADE_EXT, 0/*PROTO_CMD_START_FWUPGRADE*/) == TRUE)  
                                {                  
                                    ResetUart1();      
                                    bCheckRxBuf = FALSE;
                                    mainTask = TASK_BRIDGE;  
                                    nTimeCnt = MAX_EMPTY_SEC; 
                                    return WAIT;                                    
                                }             
                                else            
                                {                      
                                    fSwUpdate = 0;
                                    PROTOCOL_DeInit();
                                    modemCurTask = TASK_MODEM_CLOSE;
                                    modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;
                                }   
                            }     
                            else 
                            {                     
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;   
                            }                                             
                        break;
                        default:
                            modemCurTask = TASK_MODEM_CLOSE;
                            modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;
                        break;
                    }
                break;
                case TASK_MODEM_CLOSE:
                    switch (modemCurSubTask)
                    {
                        case SUB_TASK_MODEM_CLOSE_EOD: 
                            if (nErrorOnConnect == TRUE)
                            {                                
                                modemCurTask = TASK_MODEM_CONNECT;
                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_START_DIAL;
                            }
                            else
                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_TCP;
                        break;
                        case SUB_TASK_MODEM_CLOSE_TCP:     
                            modemCurSubTask = SUB_TASK_MODEM_EXIT;  //SUB_TASK_MODEM_CLOSE_MDM;
                            // if connects for monitor - turn off. do not continue
                            if (IsZeroID(AppEepromData.eLoggerID))  
                            {
                                modemCurSubTask = SUB_TASK_MODEM_EXIT;
                                break;            
                            }
                            if ((bConnectOK == FALSE))  // ||  (IsZeroID(AppEepromData.eLoggerID)))     
                            {
                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;   
                                break;
                            }
                            if ((fSwUpdate == 3) && (nCBUFw2Upg > 0))   
                            {                     
                                if (SendRecRS485(CMD_FW_UPGRADE,nCBUFw2Upg) == 1)
                                {
                                    DISABLE_CLOCK_INT();
                                    InitCom(FALSE);
                                    bCheckRxBuf = FALSE;
                                    mainTask = TASK_BRIDGE;  
                                    nTimeCnt = MAX_EMPTY_SEC; 
                                    return WAIT;                    
                                }       
//                                else
//                                {
//                                    fSwUpdate = 0;
//                                    modemCurTask = TASK_MODEM_CLOSE;
//                                    modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;
//                                }
                            }          
                            else
//                            {                     
//                                modemCurTask = TASK_MODEM_CLOSE;
//                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;   
//                            }                                             
//                        break;
//                        default:
//                            modemCurTask = TASK_MODEM_CLOSE;
//                            modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;
//                        break;
//                    }
//                break;
//                case TASK_MODEM_CLOSE:
//                    switch (modemCurSubTask)
//                    {
//                        case SUB_TASK_MODEM_CLOSE_EOD: 
//                            if (nErrorOnConnect == TRUE)
//                            {                                
//                                modemCurTask = TASK_MODEM_CONNECT;
//                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_START_DIAL;
//                            }
//                            else
//                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_TCP;
//                        break;
//                        case SUB_TASK_MODEM_CLOSE_TCP:     
//                            modemCurSubTask = SUB_TASK_MODEM_EXIT;  //SUB_TASK_MODEM_CLOSE_MDM;
//                            // if connects for monitor - turn off. do not continue
//                            if (IsZeroID(AppEepromData.eLoggerID))
//                                break; 
                            // if should update atmel or EZR: 
                            if ((fSwUpdate >= 1) && (fSwUpdate <= 3))
                            {        
                                modemCurTask = TASK_MODEM_CONNECT;   //TASK_MODEM_POST;   //
                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_START_DIAL;  
                                //waitingTask = SUB_TASK_MODEM_GET_SW_UPDATE; //modemCurSubTask  
                                nFw2Upg = 0;
                            }                                                                
                        break;
                        case SUB_TASK_MODEM_CLOSE_MDM:
//                            delay_ms(1000);
                            if (IsModemOn()) 
                            {
                                taskAfterDelay = SUB_TASK_MODEM_OFF;
                                modemCurSubTask = SUB_TASK_DELAY;
                                nTimeCnt = 200;
                            }
                            else
                                modemCurSubTask = SUB_TASK_MODEM_OFF;
                        break;
                        case SUB_TASK_MODEM_OFF:    
                                if (IsModemOn())     
                                    delay_ms(1000); 
                                else
                                    modemCurSubTask = SUB_TASK_MODEM_EXIT;
                        break;
                        case SUB_TASK_DELAY:
                            delay_ms(500);
                            if ((nTimeCnt <= 0) || (!IsModemOn()))
                                modemCurSubTask = taskAfterDelay;
                        break;
                    }
                break;
            }
            failCnt = 0;  
            return CONTINUE;
            break;
        }
        case TASK_FAILED_NO_ANSWER: 
        {
            switch (modemCurTask)
            {
                case TASK_MODEM_INIT:                        
                case TASK_MODEM_CONNECT:     
                    modemCurTask = TASK_MODEM_CLOSE;   
                    modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                break;
                case TASK_MODEM_POST:  
                    nFw2Upg = 0;
                    switch (modemCurSubTask)    
                    {                      
                    case SUB_TASK_MODEM_POST_UPD:
                    case SUB_TASK_MODEM_POST_CNFRM:
                        //if trying to update address but failed because url is incorrect - skip over update port
                        if ((bUpdateAddress) && (prmUpdtIndex == UPDATE_URL))
                            UpdatePrmArr[2] = '0';
                    break;
                    case SUB_TASK_MODEM_POST_VLV:   
                    case SUB_TASK_MODEM_POST_CBU:  
                    case SUB_TASK_MODEM_POST_DATA: 
                    case SUB_TASK_MODEM_POST_ALERT:    
                    case SUB_TASK_MODEM_POST_PUMP:
                    case SUB_TASK_MODEM_POST_CBU_META:  
                    case SUB_TASK_MODEM_POST_VCU_LST:
                        ResetReadPointer(post2send);          // reset pointer of next read block to the first one    
                        GetNextPost();                            
                        break;  
                    }
                    modemCurTask = TASK_MODEM_CLOSE;
                    modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;
                break;
                case TASK_MODEM_CLOSE:
                    switch (modemCurSubTask)
                    {
                        case SUB_TASK_MODEM_CLOSE_EOD:
                            modemCurSubTask = SUB_TASK_MODEM_CLOSE_TCP;
                        break;
                        case SUB_TASK_MODEM_CLOSE_TCP:
                            modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                        break;
                        case SUB_TASK_MODEM_CLOSE_MDM:
                            modemCurSubTask = SUB_TASK_MODEM_OFF;
                        break;
                        default:
                            modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                        break;
                    }
                break;
            }
        }
        break;
        case TASK_FAILED:
        {
            failCnt++;    // count num of failures        
            //  if failure is cos service hasn't answered- no reason to try again.
//            if ((modemCurTask == TASK_MODEM_POST) && (bPostAnswered == FALSE))
//            {
//                failCnt = nMaxFailuresNum;
//            }

            // if failed more than 2 times - quit
            if (failCnt >= nMaxFailuresNum)
            {
                switch (modemCurTask)
                {
                    case TASK_MODEM_INIT:
                        switch (modemCurSubTask)
                        {
                            case SUB_TASK_INIT_MODEM_OK:
                                // if try only once to jig the iggnition pulse - try again, else- switch off.
                                if (initCnt < 2)
                                    modemCurSubTask = SUB_TASK_MODEM_IGN_ON;
                                else
                                {
                                    modemCurTask = TASK_MODEM_CLOSE;
                                    modemCurSubTask = SUB_TASK_MODEM_OFF;
                                }
                            break;     
                            case SUB_TASK_MODEM_IGN_OFF:   
                                modemCurSubTask = SUB_TASK_INIT_MODEM_OK;   
                                break;
                            case SUB_TASK_INIT_MODEM_COPS_MAN:      
                                if (curOprtIndex < numOprt) 
                                    curOprtIndex++;         
                                else      
                                {                    
                                    fGetCopsLst = FALSE;
                                    modemCurTask = TASK_MODEM_CLOSE;
                                    modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                                }
                            break;
                            case SUB_TASK_INIT_MODEM_QSS:
                            case SUB_TASK_INIT_MODEM_COPS_LST:                            
                            case SUB_TASK_INIT_MODEM_COPS_4_MONITOR: 
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                            break;
                            case SUB_TASK_MODEM_CHK_ICCID:
                                modemCurSubTask = SUB_TASK_INIT_MODEM_COPS;
                            break;
                            case  SUB_TASK_INIT_MODEM_COPS:    
                                if ((fGetCopsLst == TRUE) && (curOprtIndex < numOprt))
                                {
                                    curOprtIndex++;  
                                    modemCurSubTask = SUB_TASK_INIT_MODEM_COPS_MAN;       
                                }            
                                else
                                    modemCurSubTask = SUB_TASK_INIT_MODEM_COPS_LST;//SUB_TASK_INIT_MODEM_REG;
                            break;
                            case SUB_TASK_INIT_MODEM_REG:
                            case SUB_TASK_INIT_MODEM_REG_STAT:    
                            case SUB_TASK_INIT_MODEM_REG_LTE:
                            case SUB_TASK_INIT_MODEM_RSSI: 
//                                if ((bExtReset == TRUE) || (fGetCopsLst == TRUE))                                
                                    if (curOprtIndex < numOprt)  // && (bExtReset == TRUE))   
                                    {                                                       
                                        curOprtIndex++;        
                                        modemCurSubTask = SUB_TASK_INIT_MODEM_COPS_MAN;      
                                    }
                                    else
                                    {
//                                        curOprtIndex = 0;
//                                        numOprt = 0;  
                                        fGetCopsLst = FALSE;
                                        modemCurTask = TASK_MODEM_CLOSE;
                                        modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                                    }  
//                                else
//                                    if ((bExtReset == FALSE) && (fGetCopsLst == FALSE))    
//                                    {
//                                        modemCurSubTask = SUB_TASK_INIT_MODEM_COPS_LST;  
//                                        fGetCopsLst = TRUE;
//                                    }                                                                      
//                                break;
                            break;
                            case SUB_TASK_INIT_MODEM_GET_COPS:
                                modemCurSubTask = SUB_TASK_INIT_MODEM_MONITOR;//SUB_TASK_INIT_MODEM_NITZ
                            break;
                            case SUB_TASK_INIT_MODEM_MONITOR:
                                modemCurSubTask = SUB_TASK_INIT_MODEM_RSSI;
                            break;
                            default:
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                            break;
                        }
                    break;
                    case TASK_MODEM_CONNECT:
                        switch (modemCurSubTask)
                        {
                            case SUB_TASK_MODEM_CONNECT_ATCH:
//                                 if (bExtReset == FALSE)
                                    modemCurSubTask = SUB_TASK_MODEM_CONNECT_DEACTV; //SUB_TASK_MODEM_CONNECT_ACTV;  //
//                                else
//                                    modemCurSubTask = SUB_TASK_MODEM_CONNECT_PDP_DEF;  //SUB_TASK_MODEM_CONNECT_SETUP1;
                            break;
                            case SUB_TASK_MODEM_CONNECT_PDP_DEF:
                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_DEACTV;    //SUB_TASK_MODEM_CONNECT_SETUP1;
                            break;                 
                            case SUB_TASK_MODEM_CONNECT_IS_ACTIVE:   
                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_DEACTV;
                            break;
                            case SUB_TASK_MODEM_CONNECT_DEACTV:      
                                modemCurSubTask = SUB_TASK_MODEM_CONNECT_ACTV;
                            break;
                            case SUB_TASK_MODEM_CONNECT_ACTV:
                            case SUB_TASK_MODEM_CONNECT_START_DIAL:  
                            {                    
                                if (fGetCopsLst == TRUE)                                
                                    if (curOprtIndex < numOprt)  
                                    {          
                                        curOprtIndex++;                                        
                                        modemCurTask = TASK_MODEM_INIT;             
                                        modemCurSubTask = SUB_TASK_INIT_MODEM_COPS_MAN;                                               
                                    }
                                    else
                                    {        
                                        fGetCopsLst = FALSE;
//                                            curOprtIndex = 0;
//                                            numOprt = 0;
                                        modemCurTask = TASK_MODEM_CLOSE;
                                        modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                                    }  
                                else             
                                    modemCurSubTask = SUB_TASK_INIT_MODEM_COPS_LST; 
                            }   
                            break;
                            default:
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                            break;
                        }
                    break;
                    case TASK_MODEM_POST:
                        switch (modemCurSubTask)
                        {
                            case SUB_TASK_MODEM_POST_PRM: 
                            case SUB_TASK_MODEM_GET_SW_UPDATE: 
                            #ifdef DebugMode
                            SendDebugMsg("\r\nfail to update ");
                            #endif  DebugMode  
                                nFw2Upg = 0;
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;
                                break;
                            case SUB_TASK_MODEM_POST_UPD:
                            case SUB_TASK_MODEM_POST_CNFRM:
                                //if trying to update address but failed because url is incorrect - skip over update port
                                if ((bUpdateAddress) && (prmUpdtIndex == UPDATE_URL))
                                    UpdatePrmArr[2] = '0';
                                // if sensor gets answers just not expected once, and there is more update to do
                                if ((IsPrmToUpdate() == TRUE) && (bPostAnswered == TRUE))
                                    modemCurSubTask = SUB_TASK_MODEM_POST_UPD;
                                else
                                {
                                    modemCurTask = TASK_MODEM_CLOSE;
                                    modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;
                                    // if post answered but wrong answer - mark mission as done
                                    if (bPostAnswered == TRUE)
                                        prmSentOK = TRUE;
                                }
                                break; 
                            
                            case SUB_TASK_MODEM_POST_VLV:   
                            case SUB_TASK_MODEM_POST_CBU:  
                            case SUB_TASK_MODEM_POST_DATA: 
                            case SUB_TASK_MODEM_POST_ALERT: 
                            case SUB_TASK_MODEM_POST_PUMP:  
                            case SUB_TASK_MODEM_POST_CBU_META: 
                            case SUB_TASK_MODEM_POST_VCU_LST:
                                ResetReadPointer(post2send);          // reset pointer of next read block to the first one      
                                GetNextPost();                            
                            break;
                            default:
                                modemCurTask = TASK_MODEM_CLOSE;
                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;
                            break;
                        }
                    break;
                    case TASK_MODEM_CLOSE:
                        switch (modemCurSubTask)
                        {
                            case SUB_TASK_MODEM_CLOSE_EOD:
                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_TCP;
                            break;
                            case SUB_TASK_MODEM_CLOSE_TCP:
                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                            break;
                            case SUB_TASK_MODEM_CLOSE_MDM:
                                modemCurSubTask = SUB_TASK_MODEM_OFF;
                            break;
                            default:
                                modemCurSubTask = SUB_TASK_MODEM_CLOSE_MDM;
                            break;
                        }
                    break;
                }
                failCnt = 0;
            }
            else
            {
                if (modemCurSubTask == SUB_TASK_INIT_MODEM_COPS_LST)
                {
                    if (fMdmAns == TRUE) //if modem answer 'ERROR' - delay 10 sec, else no delay
                        delay_ms(5000);
                    else
                        return CONTINUE;
                }
                delay_ms(1000);
                // if fail on send data - change the read mod to 3 (send again the buffer)
//                if ((modemCurSubTask == SUB_TASK_MODEM_POST_DATA) && (failCnt == 1))
//                    ResetReadPointer();          // reset pointer of next read block to the first one
                if ((modemCurSubTask == SUB_TASK_INIT_MODEM_REG) ||
                (modemCurSubTask == SUB_TASK_INIT_MODEM_REG_STAT) ||
                (modemCurSubTask == SUB_TASK_INIT_MODEM_REG_LTE))
                    delay_ms(4000);
                if (modemCurSubTask == SUB_TASK_MODEM_CONNECT_START_DIAL)        
                {
                    delay_ms(3000);
                    if (nErrorOnConnect == 0)
                    {     
                        nErrorOnConnect++;
                        modemCurTask = TASK_MODEM_CLOSE;
                        modemCurSubTask = SUB_TASK_MODEM_CLOSE_EOD;                                 
                    }            
                }
                if (modemCurSubTask == SUB_TASK_MODEM_CONNECT_ACTV)
                    delay_ms(1000);
            }
               
//         #ifdef DebugMode
//         SendDebugMsg("\r\nnext task after failed on previous\0");
//        PrintNum(modemCurTask);
//        PrintNum(modemCurSubTask);
//        #endif DebugMode
        return CONTINUE;
        break;    
        }
    }
}

void ModemMain()
{
    BYTE res;//, n;
    BYTE prevTask = modemCurSubTask;
    res = GetNextTask();
    if (res == WAIT)
        return;

#ifdef DebugMode
    if (prevTask != modemCurSubTask)   
    {
        SendDebugMsg("\r\nnext task: ");   
        PrintNum(modemCurSubTask);
    }  
    #endif DebugMode  
    
//    nTimeCnt = 0;
    switch (modemCurTask)
    {
        case TASK_MODEM_INIT:
            switch (modemCurSubTask)
            {
                case SUB_TASK_MODEM_IGN_ON:
                    TurnOnIgnition();
                break;
                case SUB_TASK_MODEM_IGN_OFF:
                    TurnOffIgnition();
                break;          
//                case SUB_TASK_INIT_MODEM_HW_SHDN:
//                    ModemHwShdn();
//                break;
                case SUB_TASK_INIT_MODEM_OK:
                    cEndMark = '\0';
                    bNeedToWait4Answer = TRUE;
                    //ask modem if OK
                    SendATCmd(AT_IsModemOK, 45, 2);
                break;
                case SUB_TASK_INIT_MODEM_COPS:
                        SendATCmd(AT_COPS_AUTO_0,25,2);
                break;
                case SUB_TASK_MODEM_CHK_ICCID:
                    SendATCmd(AT_CCID,20,1);
                break;
                case SUB_TASK_INIT_MODEM_COPS_LST:
                    fMdmAns = FALSE;
                    InitOperatorLst();  
                    longAnswerExpected = 1;
                    SendATCmd(AT_COPS_LST,1200,5);
                break;
                case SUB_TASK_INIT_MODEM_QSS:
                    SendATCmd(AT_QSS,20,2);
                break;
//                case SUB_TASK_MODEM_CGMM:
//                    SendATCmd(AT_CGMM,20,1);
//                break;
                case SUB_TASK_INIT_MODEM_COPS_MAN:
                    nMaxWaitingTime = 1000;   // wait max 2.5 sec for answer 
                    nMaxFailuresNum = 1;
                    SendOperatorSelection();
                break;           
                case SUB_TASK_INIT_MODEM_COPS_4_MONITOR:
                    bNeedToWait4Answer = TRUE;
                    //ask modem if OK
                    SendATCmd(AT_COPS_MAN_MONITOR,1000,2);
                break;                
                case SUB_TASK_INIT_MODEM_REG:
                    //ask modem if sync to network
                    SendATCmd(AT_IsModemReg,25,30);
                break;
                case SUB_TASK_INIT_MODEM_REG_STAT:
                    SendATCmd(AT_REG_UMTS_STATUS,25,15);
                break;
                case SUB_TASK_INIT_MODEM_REG_LTE:
                    SendATCmd(AT_REG_LTE_STATUS,25,15);
                break;
                case SUB_TASK_INIT_MODEM_GET_COPS:
//                    nMaxFailuresNum = 1;
                    SendATCmd(AT_COPS_ASK,25,1);
                break;
                case SUB_TASK_INIT_MODEM_MONITOR:
                    longAnswerExpected = 1;
                    SendATCmd(AT_CELL_MONITOR,20,1);
                break;
                case SUB_TASK_INIT_MODEM_RSSI:
                    longAnswerExpected = 0;
                    //ask modem RSSI with network host
                    SendATCmd(AT_CSQ,25,1);
                break;
            }
        break;
        case TASK_MODEM_CONNECT:
            switch (modemCurSubTask)
            {
                case SUB_TASK_MODEM_CONNECT_ATCH:
//                    nMaxFailuresNum = 2;
                    SendATCmd(GPRS_ATTACH,20,2);
                break;
//                case SUB_TASK_MODEM_CONNECT_SETUP1:
//                    SendATCmd(DEF_QULT_MIN_PROF,20,2);
//                break;
//                case SUB_TASK_MODEM_CONNECT_SETUP2:
//                    SendATCmd(DEF_QULT_REQ_PROF,20,2);
//                break;
                case SUB_TASK_MODEM_CONNECT_PDP_DEF:
                    SendPDPCntDef();
                break;                        
                case SUB_TASK_MODEM_CONNECT_IS_ACTIVE:           
                    SendATCmd(ISACTIVATE_CNTXT, 20, 1);
                break;
                case SUB_TASK_MODEM_CONNECT_DEACTV:
//                    nMaxWaitingTime = 30;  //wait max 3 sec  
//                    nMaxFailuresNum = 1;      
                    SendATCmd(DEACTIVATE_CNTXT,1200,1);  
                break;
                case SUB_TASK_MODEM_CONNECT_ACTV:
//                    nMaxWaitingTime = 450;  //wait max 45 sec
//                    nMaxFailuresNum = 10;
                    SendATCmd(ACTIVATE_CNTXT, 1200, 2);
                break;
                case SUB_TASK_MODEM_CONNECT_START_DIAL:
                    nMaxWaitingTime = 400;  //wait max 30 sec
                    nMaxFailuresNum = 5;
                    bNeedToWait4Answer = TRUE;    
                    SendStartDial();
                break;
            }
        break;
        case TASK_MODEM_POST:
            cEndMark = '#'; // HASHTAG;
            nMaxWaitingTime = 300;  // wait max 30 sec fot response
            bPostAnswered = FALSE;
            switch (modemCurSubTask)
            {
                case SUB_TASK_MODEM_POST_PRM:
                    nMaxFailuresNum = 1;
                   SendPostParam();
//                    SendTest();
                    break;
                case SUB_TASK_MODEM_POST_UPD:
                    nMaxFailuresNum = 3;         //max 3 times try to get update & confirm
//                    SendPostUpdate(GET_UPDATE);                           
                    SendPostMsg(POST_GET_UPDATE, AT_POST_TITLE_GETPRM, 8, AT_POST_FILE_HDR_UPDTPRM);
                    UpdatePrmArr[prmUpdtIndex] = '0';
                    break;
                case SUB_TASK_MODEM_POST_CNFRM:
                    //UpdateParam();     
                    if (prmUpdtIndex == UPDATE_VALVE_CMD)     //UPDATE_COOLING_CMD;
                        SendPostMsg(POST_CNFRM_UPDATE, AT_POST_TITLE_CNFRMVLV, 8, AT_POST_FILE_HDR_UPDTPRM);
                    else
                        SendPostMsg(POST_CNFRM_UPDATE, AT_POST_TITLE_CNFRMPRM, 8, AT_POST_FILE_HDR_UPDTPRM);
//                    SendPostUpdate(CONFIRM_UPDATE);
                    break;
#ifndef NO_EZR
                case SUB_TASK_MODEM_POST_DATA: 
                    nMaxFailuresNum = 1;        
                    SendPostMsg(POST_VCU_DATA, AT_POST_TITLE_VCU_DATA, 37, AT_POST_FILE_HDR_DATA);
                    break;                
                case SUB_TASK_MODEM_POST_VLV:    
                {
                    nMaxFailuresNum = 2;  
                    SendPostMsg(POST_GET_CMD, AT_POST_TITLE_VLV, 8, AT_POST_FILE_HDR_VALVE);
                }
                break;     
#endif NO_EZR
                case SUB_TASK_MODEM_POST_CBU:     
//                    n = GetEpromPcktCnt(POST_CBU_DATA);   
                    //Myprintf("packets TO SEND: %d\0", n);             
                    SendPostMsg(POST_CBU_DATA, AT_POST_TITLE_CBU_DATA, 61, AT_POST_FILE_HDR_CBU_DATA);
                break;     
                case SUB_TASK_MODEM_POST_CBU_META:                
//                    if (g_nCbuVer == ItayCBU)
                        SendPostMsg(POST_CBU_META_DATA, AT_POST_TITLE_CBU_META_DATA, 231, AT_POST_FILE_HDR_CBU_META); 
//                    else
//                        SendPostMsg(POST_CBU_META_DATA, AT_POST_TITLE_CBU_META_DATA_OLD, 106, AT_POST_FILE_HDR_CBU_META, 1);
                break;   
                case SUB_TASK_MODEM_POST_VCU_LST:    
                    SendPostMsg(POST_VCU_LIST, AT_POST_TITLE_VCU_LST, 208, AT_POST_FILE_HDR_VCU_LIST); 
                break;  
                case SUB_TASK_MODEM_POST_ALERT:   
//                    n = GetEpromPcktCnt(POST_ALERT);         
                    SendPostMsg(POST_ALERT, AT_POST_TITLE_ALERT, 26, AT_POST_FILE_HDR_ALERT);
                break;
                case SUB_TASK_MODEM_POST_PUMP:   
//                    n = GetEpromPcktCnt(POST_PUMP_ACT);         
                    SendPostMsg(POST_PUMP_ACT, AT_POST_TITLE_VCU_DATA, 37 , AT_POST_FILE_HDR_PUMP);
                break;
                case SUB_TASK_MODEM_GET_SW_UPDATE:   
                    nMaxFailuresNum = 1;  
                  //  nTimeCnt = 100;
                 //   nMaxWaitingTime = 100; 
                    SendGetSWUpdate();
                    break;
            }
            break;
        case TASK_MODEM_CLOSE:
//            nMaxFailuresNum = 2;
//            nMaxWaitingTime = 40;  // wait max 4 sec for response
            cEndMark = '\0';

            switch (modemCurSubTask)
            {
                case SUB_TASK_MODEM_CLOSE_EOD:   
                    delay_ms(500);
                    SendATCmd(AT_EOD,40,2);
                break;
                case SUB_TASK_MODEM_CLOSE_TCP:
                    SendATCmd(AT_TCP_CLS,100,2);
                        // if there is number in version number
                    // if atmel firmware update:              
                    if ((fSwUpdate == 1) && (BlEepromData.versionUpdate != 0))
                        // make reset
                        while(1);   
                break;
                case SUB_TASK_MODEM_CLOSE_MDM:                                                                                   
                    SendATCmd(AT_PWROFF,60,2);
                    g_bModemConnect = FALSE;  
                break;
                case SUB_TASK_MODEM_OFF:
                    ModemHwShdn();
                break;
                case SUB_TASK_MODEM_EXIT:
                    bEndOfModemTask = TRUE;  
                    if (bTestPump == TRUE)
                    {
                        bTestPump = FALSE;
                        TestMainPump(TEST_DURATION);
                    }
                    //if sensor got task of reset sensor - do it now
                    if (bMakeReset == TRUE)
                    {
                        bReset = TRUE;
                        bMakeReset = FALSE;
                    }
                    if (bConnectOK == FALSE)              
                    {
                        #ifdef DebugMode
                        SendDebugMsg("\r\nconnect not OK");
                        #endif DebugMode                
//                        ModemHwShdn();     // switch off modem
                    }
                break;
            }
        break;
        default:
    }
}

void ShutDownModem()
{
    SendRecATCmd(AT_PWROFF, 20);       
    MODEM_PWR_DISABLE();      
//    MODEM_3G_SHUTDOWN_START();
//    delay_ms(200);
//    MODEM_3G_SHUTDOWN_STOP();
    #ifdef DebugMode
    SendDebugMsg("\r\nModem Hw Shdn");
    #endif DebugMode                      
}

BYTE SendRecATCmd(flash unsigned char *bufToSend, BYTE tOut)
{
    BYTE bRes = FALSE; 
    SendATCmd(bufToSend, tOut,1);
    nTimeCnt = tOut;
    while ((nTimeCnt > 0) && (bCheckRxBuf != TRUE));
    if (bCheckRxBuf == TRUE)
        bRes =  IsOK();
    #ifdef DebugMode
    if (!bRes)
        SendDebugMsg("\r\nfailed to send at cmd ");
    #endif DebugMode
    return bRes;
}

BYTE SetModemBaudRate()
{
    BYTE n = 0, brOK = FALSE;
    unsigned char MyBaudRateH, MyBaudRateL;     
    cEndMark = '\0';
    ENABLE_UART0();  

    //#ifdef DebugMode
    ENABLE_UART1();
    UART1Select(UART_DBG);     
    //#endif DebugMode
    MyBaudRateH = UBRR0H;
    MyBaudRateL = UBRR0L;   
     #ifdef DebugMode
    SendDebugMsg("\r\npower on MODEM ");
    #endif DebugMode
    MODEM_PWR_ENABLE(); 
    delay_ms(1000);
    bNeedToWait4Answer = TRUE;
    if (!IsModemOn())
    {               
         #ifdef DebugMode
        SendDebugMsg("\r\nmodem ignition");
        #endif DebugMode
        MODEM_IGNITION_ON();
        delay_ms(SEC_4_GSM_IGNITION * 1000);
        MODEM_IGNITION_OFF();
    }          
    delay_ms(11000);
    if (!IsModemOn())
        return FALSE;
 //   return TRUE;
    // find current baud rate
    do
    {
        #ifdef DebugMode
        putchar1(n+0x30);
        SendDebugMsg("\r\ntry to connect modem on baud rate ");
        PrintNum((long)BAUD_RATE_LONG[n]);
        #endif DebugMode
        bWaitForModemAnswer = FALSE;
        UBRR0H = BAUD_RATE_HIGH[n];
        UBRR0L = BAUD_RATE_LOW[n];
        delay_ms(250);
        brOK = SendRecATCmd(AT_IsModemOK, 40);
        if (!brOK)   
        {
            brOK = IsAT(); 
            if (!brOK) 
                n++;           
        }
    }
    while ((brOK == FALSE) && (n < MAX_BAUD_RATE_OPTIONS));

    if (brOK == FALSE)
    {
        #ifdef DebugMode
        SendDebugMsg("\r\nfailed to find baudrate  ");
//        PrintNum(BAUD_RATE_LONG[n]);
        #endif DebugMode

        return FALSE;
    }
    // change baud rate
    if (! SendRecATCmd(AT_DELL_ECHO, 20))
        return FALSE;
    if (! SendRecATCmd(AT_SAVE, 20))
        return FALSE;
    if (! SendRecATCmd(AT_SET_BAUDRATE, 50))        //setup modem to baudrate 19200
        return FALSE;
    if (! SendRecATCmd(AT_CCID, 50))       
        return FALSE;
    GetICCID();    
//    fModemModel = MODEM_GE; //MODEM_NONE;
//    if (! SendRecATCmd(AT_CGMM, 50))       
//        return FALSE;
//    if (Is4dModem() == TRUE)
//        fModemModel = MODEM_SVL;
//    else
//        fModemModel = MODEM_GE;                              
        
//    delay_ms(1000);
    // back to original baudrate
    UBRR0H = MyBaudRateH;
    UBRR0L = MyBaudRateL;   
    
    delay_ms(250);
    ShutDownModem();
    return TRUE;
}
#endif MODEM_MANAGER_C

