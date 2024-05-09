#ifndef __VALVE_MNG_C
#define __VALVE_MNG_C
#include "define.h"

#include <stdio.h>
#include <stdlib.h>

//#include "define.h"
#include "Valve_Manager.h"
#include "protocol.h"
#include "Rtc_Manager.h"
#include "data_manager.h"
#include "RS485_Manager.h"
#include "interrupts.h"
#include "utils.h"
#include "modem_manager.h"
#include "Pump_manager.h"

BYTE g_bVlvListUpdated = false;
unsigned int g_lGlobalCmd = 0;

#ifndef NO_EZR

//#define NOT_ONLY_LIST

#define MAX_SEND_EZR_RETRY  2
//#define MIN_DELAY_PUMP  (signed char)(-120)
//#define MAX_DELAY_PUMP  (signed char)120

//cmd status
#define STATUS_IDLE        0   
#define STATUS_CMD_IN      1   
#define STATUS_SENT_EZR    2   
//#define STATUS_CONFIRMED   3

//valve status
#define STATUS_VCU_OFF          0
#define STATUS_VCU_ON           1
#define STATUS_WAIT_2_START     2   
#define STATUS_WAIT_2_START_ON  3   

// valve extended parameters
#define EXT_START_NOW   1
#define EXT_PING        2
#define EXT_RST         4
#define EXT_TST        8

//ValveUnit vlvCmdArr[MAX_CMD+1];// = {{735980, {19, 11, 10, 20, 0}, 20, CMD_STS_DATA},{736380, {19, 11, 10, 20, 0}, 20, CMD_STS_DATA}} ;
//ValveUnit pumpAsVlv;
DateTime g_PumpCloseTime;
static BYTE g_cmdPumFailed = 0;
//BYTE g_cmdSndCnt;

//extern BYTE bMonitorConnected;
extern volatile BYTE mainTask;
extern BYTE g_bAfterModem;
extern int iVoltage;
extern eeprom _tagAPPEEPROM AppEepromData;

flash char *cmd_status_strings[] = {"STATUS_IDLE", "STATUS_CMD_IN", "STATUS_SENT_EZR","STATUS_WAIT_2_STOP"};
flash char *vlv_status_strings[] = {"STATUS_VCU_OFF", "STATUS_VCU_ON", "STATUS_WAIT_2_START", "STATUS_WAIT_2_START_ON"};


void PrintVlvStatus(BYTE i)
{
    if (vlvCmdArr[i].VCU_ID == 0)
        return;
    #ifdef DebugMode   
    SendDebugMsg("\r\nIndex\0");   
    PrintNum(i);
    SendDebugMsg("valve id: \0");   
    PrintNum(vlvCmdArr[i].VCU_ID);
    SendDebugMsg("command status: \0");   
    SendDebugMsg(cmd_status_strings[vlvCmdArr[i].cmdStatus]);
    SendDebugMsg(" valve status: \0");   
    SendDebugMsg(vlv_status_strings[vlvCmdArr[i].vlvStatus]);
                       
    SendDebugMsg("\r\n\0");     
    #endif DebugMode
}

BYTE GetVlvCnt()
{
    BYTE i, n = 0;
    for (i = VCU1; i < MAX_CMD; i++)
    {     
        if (vlvCmdArr[i].VCU_ID != 0)
            n++;
    }    
    return n;
} 

void InitCmd(CommandData* cmd)
{
    cmd->iDuration = 0;
    cmd->offTime = 0;
    cmd->cycles = 0;
    cmd->index = 0;
}

BYTE ClosePumpCmd(BYTE idx)
{
    DateTime dt;
    
//    CloseMainPump(TRUE);
    if (vlvCmdArr[idx].cmdData.cycles > 0)  
    {               
        
        dt = GetTimeAfterDelay(vlvCmdArr[idx].cmdData.offTime);  
        vlvCmdArr[idx].cmdData.startTime.hour = dt.hour;   
        vlvCmdArr[idx].cmdData.startTime.minute = dt.minute;     
        vlvCmdArr[idx].vlvStatus = STATUS_WAIT_2_START ; 
        vlvCmdArr[idx].iExtPrm = 0;
        return 1;  
    }              
    return 0;
}

void CloseValve(BYTE index)
{
    #ifdef DebugMode  
    SendDebugMsg("\r\nClose valve \0");   
    #endif DebugMode     
    if (/*(index == MAX_CMD) && */(ClosePumpCmd(index) == 1))
        return;
    vlvCmdArr[index].vlvStatus = STATUS_VCU_OFF;    
    vlvCmdArr[index].iExtPrm = 0;               
    if (vlvCmdArr[index].nextIrg.iDuration == 0)    //if (vlvCmdArr[index].nextIrg == NULL) 
        return;
      
    vlvCmdArr[index].cmdData = vlvCmdArr[index].nextIrg;
    if (index == MAX_CMD)  
        vlvCmdArr[MAX_CMD].vlvStatus = STATUS_WAIT_2_START; 
    else 
        vlvCmdArr[index].cmdStatus = STATUS_CMD_IN;   
    vlvCmdArr[index].nSec2Start = CalcScndsToStart(vlvCmdArr[index].cmdData.startTime) * 10;
    #ifdef DebugMode  
    SendDebugMsg("set next irrigation in \0");
    PrintNum(vlvCmdArr[index].nSec2Start);  
    #endif DebugMode 
//    free(vlvCmdArr[index].nextIrg);
    vlvCmdArr[index].nextIrg.iDuration = 0; // = NULL;//
}
#endif NO_EZR

void CloseAllValve()
{
#ifndef NO_EZR
    BYTE i;
    for (i = VCU1; i <= MAX_CMD; i++)
    {     
        if (vlvCmdArr[i].vlvStatus == STATUS_VCU_ON) 
            CloseValve(i);         
    }         
#endif NO_EZR
}

#ifndef NO_EZR
void InitVlvCmd(BYTE i)
{
    vlvCmdArr[i].VCU_ID = 0;      
    InitCmd(&(vlvCmdArr[i].cmdData));
//    vlvCmdArr[i].iDuration = 0;      
//    vlvCmdArr[i].offTime = 0;
//    vlvCmdArr[i].cycles = 0;
//    vlvCmdArr[i].index = 0;
    vlvCmdArr[i].cmdStatus = STATUS_IDLE;   
    vlvCmdArr[i].vlvStatus = STATUS_VCU_OFF;   
    vlvCmdArr[i].nSec2Start = 0;  
    vlvCmdArr[i].nextIrg.iDuration = 0; // = NULL;   
    vlvCmdArr[i].iExtPrm = 0;
}

void InitValvesArrays()
{
    BYTE i;
    for (i = VCU1; i <= MAX_CMD; i++)
    {     
        InitVlvCmd(i);    
    }         
}
#endif NO_EZR

BYTE DeleteNotinListVlv(BYTE* map)
{
    BYTE i, nFreeSpace = 0;
#ifndef NO_EZR
    for (i = 0; i < MAX_CMD; i++) 
        if ((vlvCmdArr[i].VCU_ID != 0) && (map[i] == 0)) //(vlvCmdArr[i].updateSts == 0)
        {           
            if ((vlvCmdArr[i].vlvStatus == STATUS_VCU_OFF))// || (vlvCmdArr[i].status == STATUS_IDLE) || (vlvCmdArr[i].status == STATUS_CMD_IN))  
            {                  
                #ifdef DebugMode                                 
                    SendDebugMsg("\r\ndelete vcu. not in list \0");     
                    PrintNum(vlvCmdArr[i].VCU_ID);  
                #endif DebugMode 
                InitVlvCmd(i); 
                nFreeSpace++;                    
            }       
            #ifdef DebugMode  
            else
            {                
                SendDebugMsg("\r\ncannot delete vcu. during work \0");     
                PrintNum(vlvCmdArr[i].VCU_ID);
            }   
            #endif DebugMode 
        }           
#endif NO_EZR         
    return nFreeSpace;
}
#ifndef NO_EZR
//the function get 2 timestamps and return:
// 1 if first timestamp is later than 2nd
// 2 if 2nd timestamp is later than 1st
//0 if both timestamps equal
char GetLaterTimestamp(DateTime dt1, DateTime dt2)
{
//#ifdef DebugMode   
//    SendDebugMsg("\r\nGetLaterTimestamp: stop time\0");     
//    PrintNum(dt1.year);
//    PrintNum(dt1.month);
//    PrintNum(dt1.day);
//    PrintNum(dt1.hour);
//    PrintNum(dt1.minute); 
//
//#endif DebugMode   
    // if command is for previous year - too old
    if (dt1.year > dt2.year)
        return 1;         
    // if command for next year - its OK
    if (dt1.year < dt2.year)
        return 2;  
    // if command for this year - continue checking:     
    // if command is for previous month - too old
    if (dt1.month > dt2.month)
        return 1;         
    // if command for next month - its not old
    if (dt1.month < dt2.month)
        return 2;  
    // if command for this month - continue checking:     
    // if command is for previous day - too old
    if (dt1.day > dt2.day)
        return 1;         
    // if command for next day - its not old
    if (dt1.day < dt2.day)
        return 2;  
    // if command for this day - continue checking:     
    // if first is later than 2nd
    if (dt1.hour > dt2.hour)
        return 1;         
    // if 2nd is later to 1st
    if (dt1.hour < dt2.hour)
        return 2;  
    //if it same hour    
    // if first is later than 2nd
    if (dt1.minute > dt2.minute)
        return 1;         
    // if 2nd is later to 1st
    if (dt1.minute < dt2.minute)
        return 2;         
    //if hasent reached minute        
    return 0;             
}

BYTE GetVCUIndex(unsigned long Id)
{
    BYTE i = VCU1;
    do
    { 
        if (vlvCmdArr[i].VCU_ID == Id)
            return i;
        i++;
    }
    while (i <= MAX_CMD);
    return MAX_CMD;
}

BYTE OpenCloseMainPump(ERS485Command cmd, int actionID, BYTE bOpen)
{
    BYTE n = 0, res; 
    unsigned int cmdIdx;
           
    do
    {
        res = SendRecRS485(cmd, bOpen);
        n++;                 
    }       
    while ((n < 3) && (res != 1));
    cmdIdx = GetPumpCmdIdx();                   
    SavePumpActionData(actionID, res, cmdIdx);
    if (res == 1)             
    {
        g_cmdPumFailed = 0;
    }
    else
        g_cmdPumFailed++;
    if (g_cmdPumFailed > 3)
    {
        g_cmdPumFailed = 0;   
        res = 1;
    }        
    g_bMngPumpNow = TRUE;
    return res;   
}
#endif NO_EZR
void CloseMainPump(int bForceClose)
{
#ifndef NO_EZR
    if ((g_bMainPumpOpen == FALSE) && (!bForceClose))
        return; 
    
    #ifdef DebugMode
    SendDebugMsg("\r\nClose Main Pump \0");
    #endif DebugMode    
    if (OpenCloseMainPump(CMD_PUMP1_MNG, 4, PUMP_CMD_OFF) == 1)
        g_bMainPumpOpen = FALSE;   
#endif NO_EZR
}

#ifndef NO_EZR
void OpenMainPump(int bForceOpen)
{
    #ifdef DebugMode
    SendDebugMsg("\r\nOpen Pump\0");
    #endif DebugMode
    if ((g_bMainPumpOpen == TRUE) && (bForceOpen == 0))
        return ; 
    if (OpenCloseMainPump(CMD_PUMP1_MNG, 2, PUMP_CMD_ON) == 1)             
        g_bMainPumpOpen = TRUE;    
    #ifdef DebugMode
    SendDebugMsg("\r\nPump Opened\0");
    #endif DebugMode    
}
#endif NO_EZR
void TestMainPump(BYTE nDur)
{
#ifndef NO_EZR
    OpenMainPump(TRUE);
    delay_ms(nDur * 1000);
    CloseMainPump(1);
#endif NO_EZR
}
#ifndef NO_EZR
// 1 if first timestamp is later than 2nd
// 2 if 2nd timestamp is later than 1st
//0 if both timestamps equal
void CalcPumpEndTime()
{
    DateTime dt;      
    BYTE i, n = 0;
    dt = g_curTime;     
    
    for (i = VCU1; i <= MAX_CMD; i++)       
    {               
        if (vlvCmdArr[i].vlvStatus == STATUS_VCU_ON)    
        {
            if (GetLaterTimestamp(dt, vlvCmdArr[i].stopTimeStamp) == 2)     
                dt = vlvCmdArr[i].stopTimeStamp;    
            n++;
        }
    }     
    g_PumpCloseTime = dt; 
    #ifdef DebugMode   
    SendDebugMsg("CalcPumpEndTime ");
    #endif DebugMode
    if ((n == 0) && (g_bMainPumpOpen == TRUE))
        CloseMainPump(0);
}

void CalcEndTime(BYTE index)
{
//    DateTime dt;      
    unsigned int  totalIrr;
//    dt = g_curTime;  
            
//    if (index < MAX_CMD)      
//        totalIrr = ((vlvCmdArr[index].cmdData.iDuration + vlvCmdArr[index].cmdData.offTime) * vlvCmdArr[index].cmdData.cycles) - vlvCmdArr[index].cmdData.offTime;
//    else        
    {
        totalIrr = vlvCmdArr[index].cmdData.iDuration;  
        vlvCmdArr[index].cmdData.cycles--;
    }
//    tmp = vlvCmdArr[index].cmdData.startTime.minute + totalIrr;    
    vlvCmdArr[index].stopTimeStamp = GetTimeAfterDelay(totalIrr);
    CalcPumpEndTime();     
}

unsigned long CalcScndsToStart(Time strtTime)
{
    int iMnt = 0;
    int t1, t2;   
    unsigned long l;
                  
    t1 = (g_curTime.hour * 60) + g_curTime.minute;
    t2 = (strtTime.hour * 60) + strtTime.minute;  
  //  printf("g_curTime minutes %d strtTime minutes %d", t1,t2);
    if (t1 == t2)
        return (nCntDown / 10);  
    iMnt = t2 - t1 - 1;    
    if (iMnt < 0)
        iMnt += 1440;
    l = iMnt;
    l *= 60; 
    l += (nCntDown / 10);
    
    return l;  
}

BYTE IsTimeStartIrg(Time strtTime)
{
    //int t1, t2;         
    if ((g_curTime.hour == strtTime.hour) && (g_curTime.minute == strtTime.minute)) 
        return 1;
    return 0;       
//    t1 = (g_curTime.hour * 60) + g_curTime.minute;
//    t2 = (strtTime.hour * 60) + strtTime.minute;  
//  //  printf("g_curTime minutes %d strtTime minutes %d", t1,t2);
//    if (t1 == t2)
//        return  1;
//    return 0;
}

void CheckVCUStatus()
{
    int  tmp;
    BYTE  i, t, bIsvlvOpen = 0;

    #ifdef DebugMode        
    SendDebugMsg("\r\nCheck VCUs status: ");
    #endif                   
//    GetRealTime();
    for (i = VCU1; i <= MAX_CMD; i++)       
    {               
        PrintVlvStatus(i);
        if (vlvCmdArr[i].vlvStatus >= STATUS_WAIT_2_START) 
        {                        
            tmp = IsTimeStartIrg(vlvCmdArr[i].cmdData.startTime);   
            if (vlvCmdArr[i].iExtPrm & EXT_START_NOW)  
                tmp = 1;

            #ifdef DebugMode 
            SendDebugMsg("\r\nSec 2 start vlv \0");     
            PrintNum(tmp);  
            #endif DebugMode
            if (tmp == 1)               
            {                            
                vlvCmdArr[i].vlvStatus = STATUS_VCU_ON;  
                CalcEndTime(i);  
                bIsvlvOpen = 1;                      
            }  
            else
                if (vlvCmdArr[i].vlvStatus == STATUS_WAIT_2_START_ON)
                    bIsvlvOpen = 1;              
        }     
        else
            if (vlvCmdArr[i].vlvStatus == STATUS_VCU_ON)
            {        
                t = GetLaterTimestamp(g_curTime, vlvCmdArr[i].stopTimeStamp);  
            #ifdef DebugMode   
            SendDebugMsg("\r\nGetLaterTimestamp g_curTime stopTimeStamp \0");     
            PrintNum(t);  
            #endif DebugMode

                if ((t == 1) || (t == 0)) 
                    CloseValve(i);     //vlvCmdArr[i].vlvStatus = STATUS_VCU_OFF;                    
                else
                    bIsvlvOpen = 1;                 
            }  
            else                                                  
                // if valve is close but wait for answer from valve and irrigation time arrived - sign it to start now
                if ((vlvCmdArr[i].cmdStatus == STATUS_SENT_EZR) && (vlvCmdArr[i].cmdData.iDuration > 0))   
                    if (IsTimeStartIrg(vlvCmdArr[i].cmdData.startTime) == 1)                           
                        vlvCmdArr[i].iExtPrm |= EXT_START_NOW;  
                                        
    }   
          
    // if pump doesnt has its own command
//    if (CheckPumpStatus(bIsvlvOpen) == 0)   
    {                    
        // if pump is off and should be on    
        if ((bIsvlvOpen == 1) && (g_bMainPumpOpen == FALSE))
            OpenMainPump(TRUE);    
        else
            //if pump is on but should be close
            if (g_bMainPumpOpen == TRUE)  
            {         
                if ((GetLaterTimestamp(g_curTime, g_PumpCloseTime) != 2) || (bIsvlvOpen == 0)) 
                    CloseMainPump(0); 
            }   
    }     
}

void UpdateVCUStatus(unsigned long id, /*BYTE stat,*/ BYTE situation, BYTE extMsg)
{
    BYTE index;//, res = 0;
//    int tmp;
             
    #ifdef DebugMode        
    SendDebugMsg("\r\nUpdate VCU Status: \0");     
    #endif DebugMode
    // find if there is cmmd for this ID already
    index = GetVCUIndex(id);   
    // if no - find first empty row
    if (index >= MAX_CMD)     
    {                          
        #ifdef NOT_ONLY_LIST
        index = GetVCUIndex(0);
        if (index >= MAX_CMD)  
            return;// MAX_CMD;  
        else
            vlvCmdArr[index].VCU_ID = id;  
        #else      
        #ifdef DebugMode        
        SendDebugMsg("\r\n VCU not in my list \0");     
        #endif DebugMode
        return;
        #endif  NOT_ONLY_LIST
//            SaveVCUIdAtEeprom(id, index);       
    }     
                      
     #ifdef DebugMode       
     SendDebugMsg("\r\nvlv index: \0");     
    PrintNum(index);           
    SendDebugMsg("\r\nSituation: \0");     
    PrintNum(situation);  
    #endif DebugMode
    if ((extMsg & IRG_MSG_ACK) != 0)  //if (situation == 6) 
    {               
        vlvCmdArr[index].cmdStatus = STATUS_IDLE;   
        if ((vlvCmdArr[index].cmdData.iDuration > 0) && (vlvCmdArr[index].cmdData.iDuration != CODE_VLV_TST))       
            if (vlvCmdArr[index].vlvStatus != STATUS_VCU_ON)
                vlvCmdArr[index].vlvStatus = STATUS_WAIT_2_START;  
            else
                vlvCmdArr[index].vlvStatus = STATUS_WAIT_2_START_ON ; 
        
    }                      
//    if ((situation & IRG_MSG_PING) != 0) 
//    {               
//        vlvCmdArr[index].cmdStatus = STATUS_IDLE;   
//       if(vlvCmdArr[index].iDuration == CODE_PING)   
//        {
//           vlvCmdArr[index].iDuration = 0;      
//           vlvCmdArr[index].nSec2Start = 0;   
//           vlvCmdArr[index].vlvStatus = STATUS_VCU_OFF;
//        }  
//    }
//    situation &= 0x0F;  
    if ((situation == IRG_VCU_ERROR))
    {               
        vlvCmdArr[index].cmdStatus = STATUS_IDLE;     
//        vlvCmdArr[index].iExtPrm = 0;
    }
    if ((vlvCmdArr[index].vlvStatus != STATUS_VCU_ON) && (vlvCmdArr[index].cmdData.iDuration > 0) && 
        ((situation == IRG_MSG_BUILD_OK) || (situation == IRG_MSG_BUILD_FAIL)))     
    {               
        vlvCmdArr[index].vlvStatus = STATUS_VCU_ON; 
        CalcEndTime(index);
    }
    // if valve send end of irrigation message - 
    if ((vlvCmdArr[index].vlvStatus == STATUS_VCU_ON) && 
    ((situation == IRG_MSG_DROP_OK) || (situation == IRG_MSG_DROP_FAIL) || (situation == IRG_MSG_INIT) || (situation == IRG_MSG_OFF)))     
    {               
        CloseValve(index);    
        CalcPumpEndTime();
    }
    if ((vlvCmdArr[index].vlvStatus >= STATUS_WAIT_2_START) && 
    ((situation == IRG_MSG_INIT) || (situation == IRG_MSG_OFF)))     
    {               
        CloseValve(index);    
        CalcPumpEndTime();
    }
}

BYTE InsertExtNewCmd(unsigned long id,  unsigned int dur, BYTE offTime, BYTE cycles, BYTE startHour, BYTE startMin, unsigned int cmdIndex)
{
    BYTE index;     
//    NextIrrigation* pNxtIrg = NULL;      
    Time t, t1;
    unsigned long n1, n2;    
    BYTE bIsNextIrg = FALSE;

    GetRealTime();   
     
    // find if there is cmmd for this ID already
    index = GetVCUIndex(id);   
    // if no - find first empty row
    if (index >= MAX_CMD)     
    {             
        //if its not VCU id (dirct comand to pump)
        if (id != vlvCmdArr[MAX_CMD].VCU_ID)  
        {
            index = InsertVlv(id);//GetVCUIndex(0);
            if (index >= MAX_CMD)  
            {                                    
                return MAX_CMD;   
            }         
        }
    }                       
    //todo - remove. gets id only in list
    #ifdef NOT_ONLY_LIST
    vlvCmdArr[index].VCU_ID = id;  
    SaveVCUIdAtEeprom(id, index);   
    #endif NOT_ONLY_LIST
    vlvCmdArr[index].iExtPrm &= ~(EXT_START_NOW);
//    #ifdef DebugMode  
//    Myprintf("index of VCU: %d\0", index);
//    #endif DebugMode    
    // if server send start time is 25:00 it means start now  
    if (((startHour == 99) && (startMin == 99)) && (dur <= MAX_IRRIGATION_MNT)) 
    {      
        startHour = g_curTime.hour;                            
        startMin = g_curTime.minute;
        if (dur > 0) 
            vlvCmdArr[index].iExtPrm |= EXT_START_NOW;
    }   
    t.hour = startHour;
    t.minute = startMin;                                       

    if (IsSpecialCmd(dur)) //(dur == CODE_PING) || (dur == CODE_RST) || (dur == CODE_VLV_TST))  
    {      
//        startHour = g_curTime.hour;                            
//        startMin = g_curTime.minute;
        switch (dur)
        {
            case CODE_PING:
                vlvCmdArr[index].iExtPrm |= EXT_PING;  
                break;
            case CODE_RST: 
                vlvCmdArr[index].iExtPrm |= EXT_RST;
                break; 
            case CODE_VLV_TST: 
                vlvCmdArr[index].iExtPrm |= EXT_TST;
                break; 
        }
        vlvCmdArr[index].cmdStatus = STATUS_CMD_IN;   
        vlvCmdArr[index].nSec2Start = nCntDown;       
//        vlvCmdArr[index].cmdData.startTime.minute = g_curTime.minute;
//        vlvCmdArr[index].cmdData.startTime.hour = g_curTime.hour;        
        #ifdef DebugMode  
        SendDebugMsg("\r\nsetup special irrigation\0");
        #endif DebugMode 
        return index;
    }   

    // if valve is already working - check if this command is during its work
    if ((vlvCmdArr[index].vlvStatus == STATUS_VCU_ON) && (dur != 0))  
    {                
        n1 = CalcScndsToStart(t);
        t1.hour = vlvCmdArr[index].stopTimeStamp.hour;
        t1.minute = vlvCmdArr[index].stopTimeStamp.minute;        
        n2 = CalcScndsToStart(t1); 
//        #ifdef DebugMode  
//        SendDebugMsg("\r\nseconds to start next irg: \0");
//        PrintNum(n1);      
//        SendDebugMsg("\r\nstop time: \0");
//        PrintNum(t1.hour);          
//        PrintNum(t1.minute);          
//        SendDebugMsg("\r\nseconds to end cur irg: \0");
//        PrintNum(n2);  
//        #endif DebugMode   
        // if it later -        
        if (n1 > n2)
            bIsNextIrg = TRUE;  
    }
    if (bIsNextIrg == FALSE)    
    {
        vlvCmdArr[index].cmdData.iDuration = dur;     
        vlvCmdArr[index].cmdStatus = STATUS_CMD_IN;   
        vlvCmdArr[index].cmdData.startTime = t; 
        vlvCmdArr[index].cmdData.offTime = offTime;    
        vlvCmdArr[index].cmdData.cycles = cycles;    
        vlvCmdArr[index].cmdData.index = cmdIndex;
        if (vlvCmdArr[index].cmdData.startTime.minute > 59)
        {         
            vlvCmdArr[index].cmdData.startTime.minute -= 60;        
            vlvCmdArr[index].cmdData.startTime.hour++;        
        }     
        if (index == MAX_CMD)              
        {
            vlvCmdArr[index].cmdStatus = STATUS_IDLE;  
            if (vlvCmdArr[index].vlvStatus  != STATUS_VCU_ON)
                vlvCmdArr[index].vlvStatus = STATUS_WAIT_2_START;     
            else
                vlvCmdArr[index].vlvStatus = STATUS_WAIT_2_START_ON; 
            if (vlvCmdArr[index].iExtPrm & EXT_START_NOW)
            {       
                vlvCmdArr[index].vlvStatus = STATUS_VCU_ON;  
                CalcEndTime(index);  
                OpenMainPump(TRUE);  
            }                        
        }     
        if (dur == 0)                                
        {                         
            vlvCmdArr[index].cmdData.startTime.minute = g_curTime.minute;
            vlvCmdArr[index].cmdData.startTime.hour = g_curTime.hour;
            vlvCmdArr[index].vlvStatus = STATUS_VCU_OFF;         
            CalcPumpEndTime();
        }
//        else     
        vlvCmdArr[index].nSec2Start = CalcScndsToStart(vlvCmdArr[index].cmdData.startTime) * 10;
        #ifdef DebugMode  
        SendDebugMsg("\r\nseconds to start: \0");
        PrintNum(vlvCmdArr[index].nSec2Start);  
        #endif DebugMode 
    } 
    else
    {           
//        #ifdef DebugMode  
//        SendDebugMsg("\r\nnext Irg \0");
//        #endif DebugMode      
        
//        while ((vlvCmdArr[index].nextIrg == NULL) && (n < 3))   
//        {                                                       
//            #ifdef DebugMode  
//            SendDebugMsg("\r\ntry to alloc\0");
//            #endif DebugMode 
//            vlvCmdArr[index].nextIrg = (CommandData*)malloc(sizeof(CommandData));  
//            n++;      
//        }
//        if (vlvCmdArr[index].nextIrg != NULL)
        { 
            vlvCmdArr[index].nextIrg.iDuration = dur;
            vlvCmdArr[index].nextIrg.startTime = t;   
            vlvCmdArr[index].nextIrg.offTime = offTime;    
            vlvCmdArr[index].nextIrg.cycles = cycles;    
            vlvCmdArr[index].nextIrg.index = cmdIndex;          
            #ifdef DebugMode  
            SendDebugMsg("\r\nsetup next irrigation\0");
            #endif DebugMode     
        }  
//        else
//        {
//            #ifdef DebugMode  
//            SendDebugMsg("\r\nallocate mem fail\0");
//            #endif DebugMode     
//        }
    }
    if (index == MAX_CMD)  
        SavePumpActionData(6, 1, cmdIndex);      

    return index;
}

BYTE InsertVlv(unsigned long id)
{
    BYTE index;             
    // find if there is cmmd for this ID already
    index = GetVCUIndex(id);   
    // if no - find first empty row
    if (index >= MAX_CMD)     
    {
        index = GetVCUIndex(0);
        if (index >= MAX_CMD)  
            return MAX_CMD;    
    SendDebugMsg("\r\nInsert  ");
        InitVlvCmd(index);
        vlvCmdArr[index].VCU_ID = id;  
        SaveVCUIdAtEeprom(id, index);   
    //        g_bVlvListUpdated = true;
    }  
    else
        SendDebugMsg("\r\nfound  ");
          
    #ifdef DebugMode
        SendDebugMsg("valve  ");
    PrintNum(id);
    SendDebugMsg("\r\nat index  ");
    PrintNum(index);
    #endif DebugMode        
//    vlvCmdArr[index].updateSts = 1;
    return index;                
}

ValveCmd2EZR GetNextValveCmd()                      
{
    BYTE i = VCU1;  //tmp, TMP2,       
    ValveCmd2EZR tmpCmd; 
    char l[4];

    tmpCmd.m_ID = 0;       
    tmpCmd.m_duration = 0;     
    tmpCmd.m_StartAfter = 0;   
                   
    if (g_lGlobalCmd > 0)
    {   
        cpu_e2_to_MemCopy(l, AppEepromData.eLoggerID, 4);  
        tmpCmd.m_ID = Bytes2ULong(l);
        tmpCmd.m_duration = g_lGlobalCmd;
        tmpCmd.m_StartAfter = 0;    
        tmpCmd.m_OffTime = 0; 
        tmpCmd.m_nCycles = 0; 
        tmpCmd.m_nCmdIdx = 0; 
//        g_lGlobalCmd = 0;
        return tmpCmd;
    }
    do
    {            
        if (vlvCmdArr[i].VCU_ID != 0)
        {
            if ((vlvCmdArr[i].cmdStatus == STATUS_CMD_IN) && (vlvCmdArr[i].nSec2Start < 655300))
            {           
                tmpCmd.m_ID = vlvCmdArr[i].VCU_ID;   
                // if special command - 
                if (vlvCmdArr[i].iExtPrm & 0xE)   
                {                    
                    switch (vlvCmdArr[i].iExtPrm & 0xE)
                    {
                        case EXT_PING:
                            tmpCmd.m_duration = CODE_PING;
                            break;
                        case EXT_RST: 
                            tmpCmd.m_duration = CODE_RST;
                            break; 
                        case EXT_TST: 
                            tmpCmd.m_duration = CODE_VLV_TST;
                            break; 
                    }         
                    vlvCmdArr[i].iExtPrm &= EXT_START_NOW;
                }                                  
                else
                    tmpCmd.m_duration = vlvCmdArr[i].cmdData.iDuration;                                    
                tmpCmd.m_StartAfter = (unsigned int)(vlvCmdArr[i].nSec2Start / 10);    // in atmel nSec2Start is not in seconds but in 100ms    
                tmpCmd.m_OffTime = vlvCmdArr[i].cmdData.offTime; 
                tmpCmd.m_nCycles = vlvCmdArr[i].cmdData.cycles; 
                tmpCmd.m_nCmdIdx = vlvCmdArr[i].cmdData.index; 
                vlvCmdArr[i].cmdStatus = STATUS_SENT_EZR;     
                return tmpCmd;  
            }   
        }
        i++;
    }
    while (i < MAX_CMD);
    return tmpCmd;    
}

BYTE GetCmdsCnt(/*DateTime curDT*/)
{
    BYTE i, n = 0;         
    if (g_lGlobalCmd > 0)
        return 1;
    
    for (i = VCU1; i < MAX_CMD; i++)    
    {                     
        if ((vlvCmdArr[i].VCU_ID != 0) && (vlvCmdArr[i].cmdStatus == STATUS_CMD_IN) && (vlvCmdArr[i].nSec2Start < 655300))  
        {
            n++;     
        }     
    }
    return n;    
}

BYTE SendCmds(BYTE totalCnt)
{
    BYTE i = VCU1, retryCnt = 0, sendCnt = 0, res,bIsMore;

    while ((sendCnt < totalCnt) && (retryCnt < MAX_SEND_EZR_RETRY))
    {                                                           
        PROTOCOL_Init();                
        retryCnt++;         
        bIsMore = (totalCnt - sendCnt) > 5 ? 1 : 0; 
        res = PROTOCOL_Task(PROTO_CMD_VALVE_COMMANDS, bIsMore); 
        if (g_lGlobalCmd > 0)
        {
            g_lGlobalCmd = 0;
            return res;
        }
#ifdef ValveDebug    
        putchar1('<');   
        PrintNum(vlvCmdArr[i].VCU_ID);   
        #endif ValveDebug       
        do
        { 
            if ((vlvCmdArr[i].VCU_ID != 0) && (vlvCmdArr[i].cmdStatus == STATUS_SENT_EZR))
            {                 
                if (res == TRUE)
                {          
                    sendCnt++;     
//                    g_cmdSndCnt++;  
                }   
                else   
                    vlvCmdArr[i].cmdStatus = STATUS_CMD_IN;  
            }    
            i++;
        }
        while (i < MAX_CMD);            
        delay_ms(50);                 
    }   // while     
    return sendCnt;       
}

BYTE SendVlvCmdToEzr()
{
    BYTE n;            
    if (g_bVlvListUpdated == true)
    {
    #ifdef DebugMode
    SendDebugMsg("\r\nupdate valves list");
    #endif DebugMode    
        PROTOCOL_Init();                                             
        if (PROTOCOL_Task(PROTO_CMD_VALVE_LIST, 0) == true)
            g_bVlvListUpdated = false;                          
        PROTOCOL_DeInit();  
        delay_ms(100);                          
    }
    #ifdef DebugMode
    SendDebugMsg("\r\nSend Vlv CmdTo Ezr");
    #endif DebugMode    
    n = GetCmdsCnt(/*curDT*/);                 
    
    return SendCmds(n);    
}
#endif NO_EZR
BYTE IsPumpBusy()
{
#ifndef NO_EZR
    BYTE i;
    for (i = VCU1; i <= MAX_CMD; i++)                                       
        if (vlvCmdArr[i].VCU_ID > 0)               
        //if status is anything but empty - cant do FW upgrade
            if (vlvCmdArr[i].vlvStatus != STATUS_VCU_OFF)    
                return TRUE;       
#endif NO_EZR
    return FALSE;
}

#ifndef NO_EZR
void DecreaseTime2End()
{ 
    unsigned int i;

    for (i = VCU1; i <= MAX_CMD; i++)
        if (vlvCmdArr[i].VCU_ID != 0)
            if (vlvCmdArr[i].nSec2Start > 0)   
                vlvCmdArr[i].nSec2Start--;     
}
#endif NO_EZR

#endif __VALVE_MNG_C