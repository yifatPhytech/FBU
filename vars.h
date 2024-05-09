#ifndef VARS_H
#define VARS_H

/*
typedef struct 
{
1    char eLoggerID[4];                 //0
2    unsigned char eStartConnectionH;   //4
3    unsigned char eConnectionInDay;    //5
4    unsigned char eConnectIntervalH;   //6
5    int eMinTmp4Alert;                 //7
6    unsigned char eEnableTmpAlert;     //9
7    unsigned short nMaxSenNum;         //10
8    unsigned short eTimeZoneOffset;    //12
9    BYTE eRoamingDelay;                //14
10   BYTE eUseCntrCode;                 //15
11   char eMobileNetCode[5];            //16
12   char eMobileCntryCode[5];          //21
13   char eIPorURLval1[33];             //26
14   char ePORTval1[5];
15   char eAPN[33];
} _tagAPPEEPROM;
*/
//                                      1        2  3  4  5   6  7  8  9   10   11     12         13                                14      15
eeprom _tagAPPEEPROM AppEepromData;// = {{0xED,0x3A,0x0B,0x00}, 1, 6, 4, 1, 0, 0, 0, 15, 0, "01#0", "425#", "proxy.phytech.com#00000000000000", "1018", "JTM2M#00000000000000000000000000"};
//                                      0        4  5  6  7  9  10 12 14  15  16

flash unsigned char fEZRUpdateAddress[] = "bootloader.phytech.com@";
bit bExtReset;
bit bReset = 0;
bit bWaitForMonitorCmd;
//bit bEndOfMeasureTask;
bit bEndOfMonitorTask;
bit bWait4WLSensor;
//bit bWait4RS485;
bit g_LockUar1;
BYTE bEndOfCbuTask;
BYTE msrCurTask;
BYTE monitorCurTask;
volatile BYTE mainTask;
volatile BYTE prevMainTask;
volatile BYTE g_bExtIntDtct;
BYTE bMonitorConnected;
BYTE g_bMainPumpOpen;
//BYTE g_HandlePump;
BYTE btrStatus;
PUMP_CMD g_PumpCmdNow;
//BYTE g_cmdSndCnt;
BYTE    g_bAfterModem;
BYTE g_nTime2StartAT;
unsigned char powerOnReset;
DateTime g_LastCnctTime;
char DataBlock[MAX_DATA_2_EPRM_SIZE];
volatile unsigned char eepromReadBuf[MAX_DATA_2_EPRM_SIZE];  //SENSOR_CNTRL_PRM_SIZE];	//buffer for eeprom read operation
char ComBuf[MAX_RX1_BUF_LEN]; // buffer for transmit (TX)
//char DbgBuf[50]; // buffer for debug
unsigned int nEzrFw2Upg = 0;
unsigned int nCBUFw2Upg = 0;
//unsigned int g_sec2HndlPump;
//unsigned int nTicks;
//int NextByteIndex;
int BytesToSend;
int iVoltage;
int nTimeCnt = -1;
//int g_timeFromLastConnect;
//int nMaxWaitingTime;
long ezrVersion;
//CBU_COMPONENTS g_curPort;

#ifdef UseGPS
float g_fLat;
float g_fLon;
#endif UseGPS

#endif VARS_H