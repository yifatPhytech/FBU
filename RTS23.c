/*******************************************************
This program was created by the
CodeWizardAVR V2.60 Standard
Automatic Program Generator
© Copyright 1998-2012 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : WirelessLogger
Version : W.4.116.1
Date    : 17/04/2016
Author  : Yifat
Company : Phytech LTD
Comments:


Chip type               : ATmega644P
Program type            : Application
AVR Core Clock frequency: 3.685400 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 1024
*******************************************************/

#include <mega644p.h>
#include <stdio.h>
#include "define.h"
#include "vars.h"
#include "protocol.h"
#include "Valve_Manager.h"
#include "modem_manager.h"
#include "HW_manager.h"
#include "interrupts.h"
#include "utils.h"
#include "data_manager.h"
#include "Ezr_com_manager.h"
#include "Pump_manager.h"

// Declare your global variables here

void main(void)

{
    BYTE i, temp;    
    long l;          
    // Global enable interrupts
    #asm("sei")

    #asm ("wdr");         //reset the watchdog timer
//    WATCHDOG_ENABLE();     //set the watchdog
    //~~~~~~~~~~~check the startup reason~~~~~~~~~~~~~
    //the MCUSR should be set to 0x00 at the PowerDownSleep() function
    powerOnReset = FALSE;   // save reset source
    bExtReset = TRUE;
    bReset = FALSE;
    mainTask = TASK_NONE;
//    bPwrRst = FALSE;
//    tagSwitched = FALSE;
//    modemOnStartUp = FALSE;

    // Reset Source checking
    if (MCUSR & (1<<PORF)) //poweron reset
    {
        powerOnReset = TRUE;    
    }
    else
        if (MCUSR & (1<<EXTRF))     //ext. reset
        {
        }
        else
            if (MCUSR & (1<<WDRF))      //WATCH DOG RESET
            {
            }   
    temp =  MCUSR;

    MCUSR = 0x00; //reset the reset source flags to 0
//    if (powerOnReset)
//        DeepSleep();
    // init all IO's and vars
    InitProgram();          
    #ifdef DebugMode
    SendDebugMsg("\r\nreset couse: \0");
    PrintNum(temp);
    #endif DebugMode
//    InitVCUIdEeprom();
    if (IsZeroID(AppEepromData.eLoggerID))
    {                  
        InitVCUIdEeprom();
        if (SetModemBaudRate() == FALSE) 
        {   
            #ifdef DebugMode
            SendDebugMsg("\r\nSet Modem BaudRate failed \0");
            #endif DebugMode
            MODEM_PWR_DISABLE();  
            DeepSleep();
        }      
        else
            mainTask = TASK_WAKEUP;  
    }
    
    while (1)
    {       
        switch (mainTask)
        {           
        #ifndef NO_EZR
            case TASK_EZR_COM:
                if (MeasureMain() == true)
                //if (bEndOfMeasureTask == TRUE)
                {          
//                    bEndOfMeasureTask = FALSE;
                    // usually go to sleep after measure
                    mainTask = TASK_SLEEP;   
                    g_bAfterModem = FALSE;  
                    CheckVCUStatus();  
                    CheckPumpFitStatus();
//                    PrintVlvArrStatus(); 
                    #ifdef DebugMode
                    SendDebugMsg("\r\nfINISH MSR \0");
                    #endif DebugMode       
                     
                    l = GetEzrVer();      
                    if ((l & 0x000000FF) != 0x5A)// fit to receiver
                    {                           
                       #ifdef DebugMode
                        SendDebugMsg("\r\ncheck EZR ver\0");    
                        PrintNum(l & 0x000000FF);
                        #endif DebugMode
                              
                        PROTOCOL_Init();                    
                        PROTOCOL_Task(PROTO_CMD_GET_APP_INFO, 0);                         
                        //send EZR  GetApp command          
//                        i = 3;
//                        while ((PROTOCOL_Task(PROTO_CMD_GET_APP_INFO, 0) == FALSE) && (i-- > 0))  
                        {
//                            delay_ms(500);   
//                            PROTOCOL_Init(); 
                        }            
                        PROTOCOL_DeInit();            
                        #ifdef LiteDebug  
                        UART1Select(UART_DBG); 
                        PrintNum(GetEzrVer());
                        #endif LiteDebug  
                    }     
                    InitVarsForConnecting();
                }
                break;  
        #endif NO_EZR    
            case TASK_MONITOR:
                MonitorMain();
                if (bEndOfMonitorTask == TRUE)
                {         
                    //get ezr version
                    PROTOCOL_Init();                    
                    PROTOCOL_Task(PROTO_CMD_GET_APP_INFO, 0);                         
                    #ifndef DebugMode
                    DISABLE_UART1(); 
                    #endif DebugMode 
//                    CheckSizeOfEeprom();
//                    LoadIDs();
                    HandleLed(LED_1, CBU_LED_ON);
                    #ifdef DebugMode
                    SendDebugMsg("\r\nend Load ID\0");
                    #endif DebugMode
                    if (btrStatus == BTR_STATUS_FULL)
                    {                              
                        InitVarsForConnecting();
                    }
                    else                             
                    {          
                        delay_ms(3000);
                        DeepSleep();
                    }  
                }
            break;
            case TASK_MODEM:
                ModemMain();
                if (bEndOfModemTask == TRUE)
                {                    
                    if ((bExtReset == TRUE) && (bEndOfMonitorTask == FALSE))
                    {
                        mainTask = TASK_MONITOR;  
//                        monitorCurTask = TASK_MONITOR_CONNECT; 
                        break;
                    } 
                    bEndOfModemTask = FALSE;  
                    mainTask = TASK_SLEEP;                         
                    if (btrStatus == BTR_STATUS_EMPTY)    //todo - check
                        ShutDownModem();                
#ifndef     NO_EZR
                    else
                    {
                        if (SendVlvCmdToEzr() == TRUE)                       
                        {          
                            g_bAfterModem = TRUE; 
                        }              
                    } 
#endif     NO_EZR
                }
                break;      
            case TASK_SLEEP:  
//                if ((g_sec2HndlPump > 0) || (g_HandlePump == TRUE))
//                    break;
                PowerDownSleep();
                break;
            case TASK_WAKEUP:             
                if (g_bExtIntDtct == FALSE)
                    WakeUpProcedure();     
                else
                    g_bExtIntDtct = FALSE;
                GetNextMainTask();
                break;   
            #ifdef UseGPS
            case TASK_GPS:
                GPSMain();
                if (bEndOfGPSTask == TRUE)
                {
                    #ifdef DebugMode
                    SendDebugMsg("\r\nend of gps\0");
                    PrintNum((long)g_fLat);
                    PrintNum((long)g_fLon);
                    #endif DebugMode
                    bEndOfGPSTask = FALSE;
                    mainTask = TASK_EZR_COM;
                    msrCurTask = TASK_NONE;
                }
            break;       
            #endif UseGPS
            case TASK_BRIDGE:     
                if ((fSwUpdate == 0) || (nTimeCnt == (int)0)) // finished
                {       
                    mainTask = TASK_MODEM;
                    ModemResponse = TASK_COMPLETE; 
                    fSwUpdate = 0;      
                    ResetEzrVerNum();     
                    PROTOCOL_Init();
                    i = 3;  
                    do
                        delay_ms(500);                     
                    while ((PROTOCOL_Task(PROTO_CMD_GET_APP_INFO, 0) == FALSE) && (i-- > 0)) ;
                    
                    PROTOCOL_DeInit();
                    #ifdef LiteDebug  
                    UART1Select(UART_DBG); 
                    PrintNum(GetEzrVer());
                    #endif LiteDebug                                           
                }   
                else
                {                    
                    if (fSwUpdate == 2) 
                    {   
                    if (bCheckRx1Buf == TRUE)//(buffLen > 0)  
                    {      
                        bCheckRx1Buf = FALSE;
                        for (i = 0; i < /*rx1Len*/buffLen; i++)          
                            putchar0(/*rx1Buffer*/RxUart1Buf[i]);
                        /*rx1Len*/buffLen = 0; 
                        nTimeCnt = MAX_EMPTY_SEC;
                    }
                }         
                    if (fSwUpdate == 3)  
                    {       
                       /* if (g_fRS485Call == 2)    
                        {         
                            g_fRS485Call = 0;  */
                        if (bCheckRx1Buf == TRUE)  
                        {     
                            bCheckRx1Buf = FALSE;
                                for (i = 0; i < buffLen; i++)          
                                    putchar0(RxUart1Buf[i]);    
                            nTimeCnt = MAX_EMPTY_SEC;  
                            bCheckRxBuf = FALSE;   
                            bWaitForModemAnswer = TRUE; 
                            RS485_CTRL_TX();                            
                        }                   
                        if (bCheckRxBuf == TRUE)
                        {           
                            bCheckRxBuf = FALSE; 
                            RS485_CTRL_RX();
                        }   
                    }
                }         
            break;
            default:
                mainTask = TASK_SLEEP;
        }
           
//        if ((g_HandlePump == TRUE) && (bWaitForModemAnswer == FALSE))
//        {              
//            #ifdef DebugMode
//            SendDebugMsg("\r\nshould handle pump");
//            #endif DebugMode
//            g_HandlePump = FALSE;       
//            UpsideDownMainPump();
//        }             
        if (g_fRS485Call == 2 )
        {                       
            RecSendRS485();
            g_fRS485Call = 0;    
        }  
        if (bReset == TRUE)
        {
            #ifdef DebugMode
            putchar1('R');
            #endif DebugMode
            bReset = FALSE;
            bExtReset = TRUE; 
            InitProgram();
        }
              
//        if (fMinuteTimer)
//        {              
//            fMinuteTimer = false;
//            CheckVCUStatus();
//        }
        #asm ("wdr"); 		//reset the watchdog timer
    }
}

