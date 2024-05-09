#include <stdio.h>
#include <stdlib.h>
#ifndef NO_EZR

#include "Pump_manager.h"
#include "Rtc_Manager.h"
#include "data_manager.h"


//PumpUnit pumpAsVlv;

BYTE g_bMngPumpNow;

const unsigned short DAYS_EACH_MONTH[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

void CheckPumpFitStatus()
{
    //if hasnt changd pump state currntly
    if (g_bMngPumpNow == FALSE)
     // if pump logic state at CBU is open, and at CCU - it closed      //TODO  add check for 2nd pump
        if ((g_bCBUPumpState[0] == 1) && (g_bMainPumpOpen == FALSE))            
            CloseMainPump(TRUE);
}

/*BYTE IsPumpCmd()
{
    return pumpAsVlv.IsCmd4Pump;
}   */

DateTime GetTimeAfterDelay(int delay)
{
    int tmp; 
    DateTime dt;      
    dt = g_curTime; 
                  
    tmp = g_curTime.minute + delay;  
     #ifdef DebugMode   
    SendDebugMsg("minutes+duration: \0");     
    PrintNum(tmp);
#endif DebugMode       
    if (tmp < 60)
        dt.minute = tmp;    
    else
        {              
            dt.minute = tmp % 60;//dt.minute % 60; 
            tmp = tmp / 60;
            tmp += dt.hour;
            if (tmp < 24)
                dt.hour = tmp;
            else
            {
                dt.day += tmp / 24;
                dt.hour = tmp % 24;
                if (dt.day > DAYS_EACH_MONTH[dt.month-1])
                {
                    dt.day -= DAYS_EACH_MONTH[dt.month-1];
                    dt.month++;
                    if (dt.month > 12)
                    {
                        dt.month = 1;
                        dt.year++;    
                    }
                } 
            }
        }         
    return dt;
}

/*void OpenPumpCmd()
{
    OpenMainPump(TRUE);
    pumpAsVlv.stopTimeStamp = GetTimeAfterDelay(pumpAsVlv.cmdData.iDuration);   
//    CalcPumpCmdEndTime();     
    pumpAsVlv.cmdData.cycles--; 
     #ifdef DebugMode   
    SendDebugMsg("\r\nCalc pump EndTime: \0");   
    #endif DebugMode   
}

BYTE CheckPumpStatus()
{
    int  tmp;
    BYTE  t;

    if (pumpAsVlv.IsCmd4Pump == FALSE)
        return 0; 
    #ifdef DebugMode        
    SendDebugMsg("\r\nCheck pump status: ");
    #endif                             
    if (g_bMainPumpOpen == FALSE)
    {             
        tmp = IsTimeStartIrg(pumpAsVlv.cmdData.startTime);   

        #ifdef DebugMode 
        SendDebugMsg("\r\nSec 2 start pump \0");     
        PrintNum(tmp);  
        #endif DebugMode
        if (tmp == 1) //(tmp < 60)               
        {                            
            OpenPumpCmd();
        }                
    }     
    else
    {        
        t = GetLaterTimestamp(g_curTime, pumpAsVlv.stopTimeStamp);  
        #ifdef DebugMode   
        SendDebugMsg("\r\nGetLaterTimestamp g_curTime stopTimeStamp \0");     
        PrintNum(t);  
        #endif DebugMode

        if ((t == 1) || (t == 0))  
        {
            ClosePumpCmd();   
        }      
    }   
        
    return pumpAsVlv.IsCmd4Pump;
} 

unsigned int GetPumpCmdIdx()
{
    return vlvCmdArr[MAX_CMD].cmdData.index;
//    if (pumpAsVlv.IsCmd4Pump == TRUE) 
//        return pumpAsVlv.cmdData.index;  
//    return 0;
}   */

#endif NO_EZR


