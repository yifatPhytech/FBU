#ifndef MODEM_MANAGER_H
#define MODEM_MANAGER_H

#define MAX_ICCID_LEN   20
//#define UPDATE_VLV_LST          25


#define SUB_TASK_INIT_MODEM_COPS_LST    23
#define SUB_TASK_MODEM_GET_SW_UPDATE    46
#define SUB_TASK_MODEM_POST_UPD         43

#define MODEM_IGNITION_ON() (PORTC.4 = 1);         
#define MODEM_IGNITION_OFF() (PORTC.4 = 0);

#define MODEM_PWR_ENABLE() (PORTD.7 = 1);   
#define MODEM_PWR_DISABLE() (PORTD.7 = 0);

extern flash unsigned char fSWUpdateAddress[];
extern bit longAnswerExpected;
extern bit bWaitForModemAnswer;
extern bit bEndOfModemTask;
extern bit bNeedToWait4Answer;
extern bit overFlow;
extern BYTE findOK;
extern BYTE nFw2Upg;
extern BYTE g_bSendCBUMeta;
//extern BYTE fModemModel;
extern BYTE numOprt;
//extern BYTE fGetCopsLst;
extern BYTE rssi_val;
extern BYTE modemCurTask;
extern BYTE modemCurSubTask;
extern BYTE prmUpdtIndex;
extern BYTE g_bModemConnect;

extern BYTE ModemResponse;
extern BYTE fSwUpdate;
extern int nMaxWaitingTime;
extern unsigned long OprtTbl[10] ;
extern BYTE AccessTech[10] ;
/////////////////////////////////////////////
// modem_manager functions
////////////////////////////////////////////

void InitVarsForConnecting(/*BYTE b4Vlv*/);
void ModemMain();

char IsModemOn();

BYTE SetModemBaudRate();

void InitVersiontoUpdt();

void ShutDownModem();

BYTE IsOperatorExist(long lNewOp, char nTotalNum );

void CopyICCID(BYTE nStartIndex);

BYTE IsSpecialCmd(int x);

void InitOperatorLst();

void PrintFlowData();

#endif MODEM_MANAGER_H
