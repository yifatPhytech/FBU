//#include <stdbool.h>
#include "define.h"
#include "interrupts.h"
#include "Valve_Manager.h"
#include "modem_manager.h"
#include "HW_manager.h"

#define FOUND_STAGE1     1
#define FOUND_STAGE2     2
#define FOUND_STAGE3     3
#define FOUND_STAGE4     4

#define FOUND_1ST_COMMA     1
#define FOUND_2ND_COMMA     2
#define FOUND_OPEN_APOSTROPHE     3
#define FOUND_OPEN_BRACKET  4
#define FOUND_START_OPR     5
#define FOUND_STAT           6
#define FOUND_CLS_APOSTROPHE    7

#define FOUND_P     1
#define FOUND_H     2
#define FOUND_Y     3
#define FOUND_1     4
#define FOUND_PHY111    5

#define FOUND_VERIZON_1     10
#define FOUND_VERIZON_2     11
#define FOUND_VERIZON_3     12
#define FOUND_VERIZON_4     13
#define FOUND_VERIZON_5     14
#define FOUND_VERIZON_6     15

extern eeprom _tagAPPEEPROM AppEepromData;
//static char oneOperator[60];
static BYTE curIndex;
volatile BYTE findVer;
volatile BYTE findStatus;
volatile long curOprt;
//volatile BYTE oprtStat;
volatile BYTE NoRecDataU0Cnt;
volatile BYTE NoRecDataU1Cnt;
//bool fMinuteTimer;

//extern BYTE g_HandlePump;;
unsigned int rx1_buff_len;
unsigned int buffLen;
int NextByteIndex;
extern char ComBuf[MAX_RX1_BUF_LEN];
char DbgBuf[50]; 
volatile char RxUart0Buf[MAX_RX_BUF_LEN];
char RxUart1Buf[MAX_RX1_BUF_LEN];
extern int BytesToSend;
unsigned int rx0_buff_len;
extern volatile BYTE mainTask;
extern volatile BYTE prevMainTask;
extern volatile BYTE g_bExtIntDtct;
//extern BYTE bMonitorConnected;
int TimeLeftForWaiting;
bit bCheckRxBuf;
bit bCheckRx1Buf;
extern int nTimeCnt;
extern bit bWaitForModemAnswer;
extern bit bWaitForMonitorCmd;
extern bit bWait4WLSensor;
BYTE g_fRS485Call;
extern BYTE g_nTime2StartAT;
extern BYTE g_nTime2StartAT;
BYTE flgUart1Error;
unsigned int nCntDown;
//extern unsigned int    g_OneMntCnt;
//extern unsigned int g_sec2HndlPump;
extern PUMP_CMD g_PumpCmdNow;

// Timer 0 overflow interrupt service routine
interrupt [TIM0_OVF] void timer0_ovf_isr(void)
{
    if (bWaitForModemAnswer)
    {
        NoRecDataU0Cnt++;
        if (NoRecDataU0Cnt > 2)     //2
        {
            bWaitForModemAnswer = FALSE;
            bCheckRxBuf = TRUE;   
            longAnswerExpected = 0;
            DISABLE_TIMER0();     
        }
    }
//    if ((mainTask == TASK_BRIDGE) && (fSwUpdate == 3))
//        if (NoRecDataU0Cnt > 3) 
//            bCheckRxBuf = TRUE;   
    
}

// Timer1 output compare A interrupt service routine - EVERY 100 ML SECOND
interrupt [TIM1_COMPA] void timer1_compa_isr(void)
{
//    BYTE i;

    if(nTimeCnt > 0)
    {
         nTimeCnt--;
    }      
    if (g_nTime2StartAT > 0)
        g_nTime2StartAT--; 
     
    if (TimeLeftForWaiting > 0)
    {
        TimeLeftForWaiting--;
    }
   
    if (nCntDown > 0)
        nCntDown--;   
     
#ifndef     NO_EZR
    if (mainTask != TASK_BRIDGE)
    {
        DecreaseTime2End();
    }   // if (mainTask != TASK_BRIDGE)
#endif     NO_EZR
}

// Timer2 overflow interrupt service routine
interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{
//    if ((bWaitForMonitorCmd) || (bWait4WLSensor) || (mainTask == TASK_BRIDGE) || (bWait4RS485))
    {
        NoRecDataU1Cnt++;
        if (NoRecDataU1Cnt >= 4)
        {              
            bCheckRx1Buf = TRUE;
            bWaitForMonitorCmd = FALSE;
            bWait4WLSensor = FALSE;    
//            bWait4RS485 = FALSE;
            buffLen = rx1_buff_len;
            rx1_buff_len = 0;
            DISABLE_TIMER2();   
        }
    }        
    #ifdef UseGPS
    else
        if (bWaitForGPSData == TRUE)
            if (rx1_buff_len >= MINMEA_MAX_LENGTH)
            {
                bCheckRxBuf = TRUE;
                bWaitForGPSData = FALSE;
                buffLen = rx1_buff_len;
                DISABLE_TIMER2();
            }    
    #endif UseGPS
}

/*void AddOperator()
{
    BYTE i = 0, n = 0; 
    long curOprt = 0;      
    #ifdef DebugMode
    SendDebugMsg("\r\nAddOperator \0");
    #endif DebugMode
    do  
    {   
        if (oneOperator[i++] == ",")
            n++;        
    }
    while ((n < 3) && (i < curIndex));     
    i++;  

    #ifdef DebugMode
    SendDebugMsg("\r\nindex: \0");  
    PrintNum(i);   
    putchar1(oneOperator[i]);
    #endif DebugMode
      
    while ((i <= curIndex) && (oneOperator[i] != '"') && (oneOperator[i] >= '0') && (oneOperator[i] <= '9'))  
    {
        curOprt *= 10;
        curOprt += (oneOperator[i++] - 0x30);   
         #ifdef DebugMode
        SendDebugMsg("\r\nbuild Operator \0");  
        PrintNum(curOprt);
        #endif DebugMode  
    }

    if ((curOprt > 0) && (oneOperator[0] != 3))   
        if (IsOperatorExist(curOprt, numOprt) == FALSE)
        {   
            numOprt++;                
            OprtTbl[numOprt] = curOprt; 
             #ifdef DebugMode
            SendDebugMsg("\r\nnew Operator \0");  
            PrintNum(curOprt);
            #endif DebugMode  
        }                                                   
}         */

/*
AT+COPS=?
.           op name    op num
.+COPS: (2,"Cellcom",,"42502",2),(2,"Cellcom",,"42502",0),(3,"Orange IL",,"42501",2),(3,"IL Pelephone",,"42503",2),(3,"Orange IL",,"42501",0),(3,"PS, Wataniya Mobile",,"42506",0),,(0-4),(0,2)
.
.OK
*/
void HandleCopsRx(BYTE LastRxByte)
{                      
    if ((rx0_buff_len == 0U) && (overFlow == 0)) 
        findStatus = FOUND_NOTHING;        
    else            
        if ((findStatus == FOUND_NOTHING))
        {
            if (LastRxByte == '(') 
            {
                findStatus = FOUND_OPEN_BRACKET;   
                curIndex = 0;   
            }   
        }
        else       
            if (findStatus == FOUND_OPEN_BRACKET)  
            {                  
                if ((LastRxByte >= '0') && (LastRxByte <= '9')) 
                {  
                    if (LastRxByte == '3')    
                        findStatus = FOUND_NOTHING;
                    else
                        findStatus = FOUND_STAT;   
                }                  
            }
            else
            {
                if (findStatus == FOUND_STAT)  
                {
                    if (LastRxByte == ',')  
                    {
                        curIndex++;   
                        if (curIndex == 3)
                            findStatus = FOUND_START_OPR;                                       
                    }            
                }   
                else
                    if (findStatus == FOUND_START_OPR)
                        if (LastRxByte == '"')     
                        {
                            findStatus = FOUND_OPEN_APOSTROPHE;     
                            curOprt = 0;
                        }
                        else 
                            findStatus = FOUND_NOTHING;
                    else
                        if (findStatus == FOUND_OPEN_APOSTROPHE)   
                        {
                            if ((LastRxByte >= '0') && (LastRxByte <= '9'))    
                            {                            
                                curOprt *= 10;
                                curOprt = curOprt + (LastRxByte - 0x30);
                            }
                            else 
                                if (LastRxByte == '"')   
                                {                                                  
                                    if ((IsOperatorExist(curOprt, numOprt) == FALSE) && (curOprt > 0))// && (oprtStat != '3'))
                                    {   
                                        //numOprt++;                
                                        OprtTbl[numOprt++] = curOprt;     
                                        findStatus = FOUND_CLS_APOSTROPHE;
                                    } 
                                    else                                       
                                        findStatus = FOUND_NOTHING;   
//                                    oprtStat = 0;
                                }            
                        }
                        else                  
                            if (findStatus == FOUND_CLS_APOSTROPHE) 
                            {  
                                if ((LastRxByte >= '0') && (LastRxByte <= '9')) 
                                {
                                    AccessTech[numOprt-1] =  LastRxByte;
                                    findStatus = FOUND_NOTHING;       
                                }         
                                else 
                                    if (LastRxByte != ',')
                                        findStatus = FOUND_NOTHING;   
                            }
                            else
                                findStatus = FOUND_NOTHING;    
            }                                          
}                                         
     
/*void HandleVlvList(BYTE LastRxByte)
{
    if ((rx0_buff_len == 0U) && (overFlow == 0))  
    {
        findStatus = FOUND_NOTHING;  
        curIndex = 0;
    }       
    switch (findStatus)
    {         
        case FOUND_NOTHING:
            if (LastRxByte == 'p') 
                findStatus = FOUND_P;   
            break;
        case FOUND_P:
            if (LastRxByte == 'h') 
                findStatus = FOUND_H;  
            else 
                findStatus = FOUND_NOTHING;
            break;
        case FOUND_H:
            if (LastRxByte == 'y') 
                findStatus = FOUND_Y;  
            else 
                findStatus = FOUND_NOTHING;
            break;
        case FOUND_Y:       
        case FOUND_1:
            if (LastRxByte == '1') 
            {
                findStatus = FOUND_1;  
                curIndex++;   
                if (curIndex == 3)            
                {
                    findStatus = FOUND_PHY111;   
                    curIndex = 0;
                } 
            }
            else 
                findStatus = FOUND_NOTHING;
            break;
        case FOUND_PHY111:
        {  
            newId.bVal[curIndex++] = LastRxByte;
            if (curIndex == 4)
            {       
                curIndex = 0;
                InsertVlv(newId.lVal);
            }    
        }
        break;    
                
    }
}
 */  
// USART0 Receiver interrupt service routine
interrupt [USART0_RXC] void usart0_rx_isr(void)
{
    BYTE LastRxByte;
    char status;
    status = UCSR0A;
    LastRxByte = UDR0;       //read uart data register

    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
    {
        if (mainTask == TASK_BRIDGE)  
        {                        
            putchar1(LastRxByte);      
            nTimeCnt = MAX_EMPTY_SEC;
            if (fSwUpdate == 3)
            {
                ENABLE_TIMER0();
                NoRecDataU0Cnt = 0;
            }
            return;
        }
        //when getting answer for post what we need is end of data-not the begining so when reach to MAX_TX_BUF_LEN - start rewrite on the buffer from the begining.
        // the rx0_buff_len represent last index data got into it.
        if (longAnswerExpected)
        {
            // if reach to end of ComBuf - start from the begining
            if(rx0_buff_len >= MAX_RX_BUF_LEN)
            {
                overFlow = 1;
                rx0_buff_len = 0;
            }
            RxUart0Buf[rx0_buff_len++] = LastRxByte;
        }
        else                
        {
            //set the msg into ComBuf[] buffer
            if(rx0_buff_len >= MAX_RX_BUF_LEN)
                return; //exit the function
            RxUart0Buf[rx0_buff_len++] = LastRxByte;    
        }        
        #ifdef LiteDebug
        putchar1(LastRxByte); 
        #endif LiteDebug
        if ((findOK == FOUND_NOTHING) && (LastRxByte == 'O'))
            findOK = FOUND_O; 
        else
            if (findOK == FOUND_O)
                if (LastRxByte == 'K')      
                {
                    findOK = FOUND_OK;
                    findVer = FOUND_NOTHING; 
                }      
                else
                    findOK = FOUND_NOTHING; 
                    
        if (modemCurSubTask == SUB_TASK_INIT_MODEM_COPS_LST)
            HandleCopsRx(LastRxByte);    
//        if ((modemCurSubTask == SUB_TASK_MODEM_POST_UPD) && (prmUpdtIndex == UPDATE_VLV_LST))
//            HandleVlvList(LastRxByte);      
      
        if ((modemCurSubTask == SUB_TASK_MODEM_GET_SW_UPDATE) && (findOK == FOUND_OK))
        {
            if ((findVer == FOUND_NOTHING) && (LastRxByte == '\r'))
                findVer = FOUND_STAGE1;  
            else
                if (findVer == FOUND_STAGE1)
                    if (LastRxByte == '\n')
                        findVer = FOUND_STAGE2;
                    else 
                        findVer = FOUND_NOTHING;
                else
                    if (findVer == FOUND_STAGE2)
                        if (LastRxByte == '\r')
                            findVer = FOUND_STAGE3;
                        else 
                            findVer = FOUND_NOTHING;
                    else
                        if (findVer == FOUND_STAGE3)
                            if (LastRxByte == '\n')
                                findVer = FOUND_STAGE4;
                            else 
                                findVer = FOUND_NOTHING;
                        else
                            if (findVer == FOUND_STAGE4)  
                            {
                                if ((LastRxByte >= '0') && (LastRxByte <= '9')) 
                                   nFw2Upg = nFw2Upg * 10 + (LastRxByte - 0x30);
                            }
                            else       
                            {
                                findVer = FOUND_NOTHING;
                                nFw2Upg = 0;
                            }                                              
        }
        
        ENABLE_TIMER0();
    }
    NoRecDataU0Cnt = 0;
    // enable timer of receive
}

// USART1 Receiver interrupt service routine
interrupt [USART1_RXC] void usart1_rx_isr(void)
{
    char status, data;
    status = UCSR1A;
    data = UDR1;
    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
    {              
        //set the msg into RxUart1Buf[] buffer
        if(rx1_buff_len >= MAX_RX1_BUF_LEN)
            return; //exit the function
        RxUart1Buf[rx1_buff_len++] = data;
        ENABLE_TIMER2();
    }
    else   
    {                  
        if ((mainTask == TASK_BRIDGE) && (status & FRAMING_ERROR)) 
        {         
            fSwUpdate = 0;
            rx1_buff_len = 0;                
        }
        flgUart1Error = status;    
    }
    NoRecDataU1Cnt = 0;
}

// External Interrupt 2 service routine  // clock int
interrupt [EXT_INT2] void ext_int2_isr(void)
{
//    fMinuteTimer = true;   
    if (mainTask == TASK_SLEEP)  
    {
//        DISABLE_CLOCK_INT();
        PRR &= 0x00;         
         mainTask = TASK_WAKEUP;   
    }                
    else
    {
//        prevMainTask = mainTask; 
        if (mainTask == TASK_MODEM) 
            g_bExtIntDtct = TRUE;      
        else
            mainTask = TASK_WAKEUP; 
    }
//    mainTask = TASK_WAKEUP;   
    nCntDown = 600;         
}
/*
// Pin change 24-31 interrupt service routine
interrupt [PC_INT3] void pin_change_isr3(void)
{
// CHECK PORTD.6 direction:
    if (PIND.6 == 0)       
    {
        g_fRS485Call = 1; 
        if (mainTask == TASK_SLEEP)
        {
            PRR &= 0x00;
            mainTask = TASK_WAKEUP;   
        }
    }
}  */  

// Pin change 8-15 interrupt service routine
interrupt [PC_INT1] void pin_change_isr1(void)
{
// CHECK PORTB.1 direction:
    if (PINB.1 == 0)       
    {
        g_fRS485Call = 2; 
        if (mainTask == TASK_SLEEP)
        {
            PRR &= 0x00;
            mainTask = TASK_WAKEUP;
            g_fRS485Call = 1;    
        }
    }

}  


//void TransmitBuf(unsigned char* ComBuf, unsigned char BytesToSend, char iPortNum)
void TransmitBuf(char iPortNum)
{
//    int i;
    int nCnt, nbi = 0;

    NextByteIndex = 0;	// reset for Tx
    if (iPortNum == 0)
	    while (bWaitForModemAnswer == TRUE); //wait until rx0 end

    if ((iPortNum == 1)) // sending to monitor
    {
        rx1_buff_len = 0;
        while(BytesToSend-- )
        {
            while ((UCSR1A & DATA_REGISTER_EMPTY)==0);
 		    UDR1 = ComBuf[ NextByteIndex++];    // send next byte..
        }
    }

	if (iPortNum == 0)
    {
        // clear RX buf     
        memset(RxUart0Buf, 0, MAX_RX_BUF_LEN);
        rx0_buff_len = 0;  
        findOK = FOUND_NOTHING;
        {
            #ifdef LiteDebug  
//            if (mainTask == TASK_MODEM)          
            {
                nCnt = BytesToSend;
                while(nCnt-- )
                {
                    // wait for UART's shift register to finish sending byte
                    while(( UCSR1A & DATA_REGISTER_EMPTY)==0);
                    UDR1 = ComBuf[ nbi++ ];// send next byte..
                }                  
            }
            #endif //DebugMode      
            while(BytesToSend-- )
            {
                // wait for UART's shift register to finish sending byte
                while(( UCSR0A & DATA_REGISTER_EMPTY)==0);
                UDR0 = ComBuf[ NextByteIndex ];// send next byte..
                NextByteIndex++;
            }
        }
    }

    #ifdef DebugMode
    if (iPortNum == 2) //debug
    {
        while(BytesToSend-- )
        {
            while ((UCSR1A & DATA_REGISTER_EMPTY)==0);
            UDR1 = DbgBuf[ NextByteIndex++ ];// send next byte..
        }
        rx1_buff_len = 0;
    }
    #endif DebugMode
	NextByteIndex = 0;//prepare for Rx

    if (iPortNum == 0)
    {
        ModemResponse = NO_ANSWER;
	    TimeLeftForWaiting = nMaxWaitingTime;  

        NoRecDataU0Cnt = 0;
        if (bNeedToWait4Answer == TRUE)
        {
            bWaitForModemAnswer = TRUE;
        }    
        bCheckRxBuf = FALSE; 
    }
    if (iPortNum == 1)//else
    {
        NoRecDataU1Cnt = 0;
        if(mainTask == TASK_MONITOR)
        {
            TimeLeftForWaiting = MAX_WAIT_MNTR_SEC * 10;
            bWaitForMonitorCmd = TRUE;
        }
       bCheckRx1Buf = FALSE;
    }        
//    bCheckRxBuf = FALSE; 
}
