/*******************************************************
This program was created by the
CodeWizardAVR V2.60 Standard
Automatic Program Generator
© Copyright 1998-2012 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : HiveScale
Version : 1.0.0.0
Date    : 27/01/2013
Author  : Yifat Sternberg
Company : Phytech Ltd.
Comments:


Chip type               : ATmega324P
Program type            : Application
AVR Core Clock frequency: 3.686400 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 512
*******************************************************/

#ifdef ATMEGA324
#include <mega324.h>
#else
#include <mega644p.h>
#endif ATMEGA324
#include <stdio.h>
#include "define.h"
#include "vars.h"

void main(void)
{
    // Global enable interrupts
    #asm("sei")

	#asm ("wdr"); 		//reset the watchdog timer
//    WATCHDOG_ENABLE(); 	//set the watchdog
    //~~~~~~~~~~~check the startup reason~~~~~~~~~~~~~
    //the MCUSR should be set to 0x00 at the PowerDownSleep() function
    bExtReset = FALSE;
    bReset = FALSE;
    mainTask = TASK_NONE;
    bPwrRst = FALSE;
    // Reset Source checking
    if (MCUSR & (1<<PORF)) //poweron reset
    {
        PORTA.5 = 1;
        delay_ms(500);
        PORTA.5 = 0;
        bExtReset = TRUE;
        bPwrRst = TRUE;
    }
    else
        if (MCUSR & (1<<EXTRF))     //ext. reset
        {
            //err_buf[EXT_RESET] = EXT_RESET;
            bExtReset = TRUE;
        }
        else
            if (MCUSR & (1<<WDRF))      //WATCH DOG RESET
            {
                //err_buf[WATCHDOG_RESET] = WATCHDOG_RESET;
                PORTA.7 = 1;
                delay_ms(500);
                PORTA.7 = 0;
            }

	MCUSR = 0x00; //reset the reset source flags to 0

    // init all IO's and vars

    InitProgram();
//    WakeUpProcedure();
    while (1)
    {
        switch (mainTask)
        {
            case TASK_MEASURE:
                MeasureMain();
                if (bEndOfMeasureTask == TRUE)
                {
                    bEndOfMeasureTask = FALSE;
                    if (bConnectNow == TRUE)
                        InitVarsForConnecting();
                    else
                        if (bExtReset == TRUE)
                        {
                            mainTask = TASK_MONITOR;
                            monitorCurTask = TASK_MONITOR_CONNECT;
                        }
                        else
                        {
                            mainTask = TASK_SLEEP;
//                            if (SendAlerts() == TRUE)      // todo - put back if needed
//                            {
//                                toDoList = DO_DATA;
//                                #ifdef DebugMode
//                                SendDebugMsg("\r\nSend Alert");
//                                #endif DebugMode
//                                if (IsConnectingTimeClose() == FALSE)
//                                    InitVarsForConnecting();
//                                #ifdef DebugMode
//                                else
//                                    SendDebugMsg("\r\nDelay connecting cos usual connection is in few minutes");
//                                #endif DebugMode
//                            }
                        }
                }
                break;
            case TASK_MONITOR:
                MonitorMain();
                if (bEndOfMonitorTask == TRUE)
                {
                    bEndOfMonitorTask = FALSE;
                    #ifdef DebugOnUart0
                    mainTask = TASK_SLEEP;
                    #else
                    InitVarsForConnecting();
                    #endif DebugOnUart0
                    #ifdef DebugMode
                    SendDebugMsg("\r\nEnd of Monitor\0");
                    #else
                    DISABLE_UART1();
                    #endif DebugMode
                }
            break;
            #ifdef BlueToothOption
            case TASK_BLUETOOTH:
                BlueToothMain();
                if (bEndOfBTTask == TRUE)
                {
                    PORTD.4 = 0;
                    bEndOfBTTask = FALSE;
                    mainTask = TASK_SLEEP;
                }
                break;
            #endif BlueToothOption
            case TASK_MODEM:
                ModemMain();
                if (bEndOfModemTask == TRUE)
                {
                    mainTask = TASK_SLEEP;
                    bEndOfModemTask = FALSE;
                    DISABLE_UART0();
                }
                break;
            case TASK_SLEEP:
                PowerDownSleep();
                break;
            case TASK_WAKEUP:
                WakeUpProcedure();
                break;
            default:
                mainTask = TASK_SLEEP;
        }

        if (bReset == TRUE)
        {
            #ifdef DebugMode
            putchar1('R');
            #endif DebugMode
            bReset = FALSE;
            bExtReset = TRUE;
            mainTask = TASK_WAKEUP;
//            InitProgram();
//            WakeUpProcedure();
        }
        #asm ("wdr"); 		//reset the watchdog timer
    }
}
