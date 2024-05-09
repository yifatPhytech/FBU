////////////////start of General.c file//////////////
#include <stdio.h>
#include <stdarg.h> 
#include "define.h"
#include "utils.h"
#include "protocol.h"
#include "Valve_Manager.h"
#include "Rtc_Manager.h"
#include "data_manager.h"
#include "RS485_Manager.h"
#include "modem_manager.h"
#include "HW_manager.h"
#include "interrupts.h"
#include "Ezr_com_manager.h"
#include "Pump_manager.h"


extern eeprom _tagPortDefEEPROM ComponentArray[];
extern flash unsigned char RomVersion[]; 
extern PUMP_CMD g_PumpCmdNow;
//extern CBU_COMPONENTS g_curPort;
extern volatile BYTE prevMainTask;
extern volatile BYTE g_bExtIntDtct;
extern volatile BYTE mainTask;
extern bit bWaitForModemAnswer;
extern bit bWaitForMonitorCmd;
extern bit bEndOfMonitorTask;
extern bit g_LockUar1;
extern BYTE monitorCurTask;
extern BYTE bMonitorConnected;
extern BYTE msrCurTask;
extern BYTE modemCurTask;
extern BYTE g_nTime2StartAT;
extern BYTE bEndOfCbuTask;
extern BYTE nRegDenied;
extern BYTE btrStatus;
extern char ComBuf[MAX_RX1_BUF_LEN];
extern int nTimeCnt;
extern int iVoltage;
extern int BytesToSend;
//

#pragma used+
void putchar1(char c)
{
    #ifdef DebugMode
    if ((mainTask == TASK_EZR_COM))// || (mainTask == TASK_EZR))
        return;  
    if (mainTask != TASK_BRIDGE)   
        UART1Select(UART_DBG);   
    #endif DebugMode
    while ((UCSR1A & DATA_REGISTER_EMPTY)==0);
    UDR1=c;
}
#pragma used-

#pragma used+

void putchar0(char c)
{
    while ((UCSR0A & DATA_REGISTER_EMPTY)==0);
    UDR0 = c;
}
#pragma used-

void ResetUart1()
{
    memset(RxUart1Buf,0, MAX_RX1_BUF_LEN);
    rx1_buff_len = 0;      
    buffLen = 0;  
    bCheckRx1Buf = FALSE;
}

#ifdef DebugMode
void PrintSensorDef()
{
    BYTE i, j;
    SendDebugMsg("\r\nPorts def: \0");  
    for (i = 0; i < MAX_PORTS_CBU; i++)
    {           
        SendDebugMsg("\r\nPort: \0");  
        PrintNum(ComponentArray[i].Id);
        SendDebugMsg("\r\nName: \0");          
        for (j = 0; j < 10; j++)
            putchar1(ComponentArray[i].Name[j]); 
        SendDebugMsg("\r\n:Type \0");  
        PrintNum(ComponentArray[i].SensorType);
        SendDebugMsg("\r\nConversion: \0");  
        PrintNum(ComponentArray[i].Conversion);
        SendDebugMsg("\r\nRange: \0");  
        PrintNum(ComponentArray[i].Range);     
        SendDebugMsg("\r\nHighTreshold: \0");  
        PrintNum(ComponentArray[i].HighTreshold);
        SendDebugMsg("\r\nLowTreshold: \0");  
        PrintNum(ComponentArray[i].LowTreshold);     
        SendDebugMsg("\r\nHighFillingTime: \0");  
        PrintNum(ComponentArray[i].HighFillingTime);   
        SendDebugMsg("\r\nLowFillingTime: \0");  
        PrintNum(ComponentArray[i].LowFillingTime);   
        SendDebugMsg("\r\nHighStablingTime: \0");  
        PrintNum(ComponentArray[i].HighStablingTime);   
        SendDebugMsg("\r\nLowStablingTime: \0");  
        PrintNum(ComponentArray[i].LowStablingTime);   
    }
}
#endif DebugMode      

void HandleLed(BYTE nLedIndex, BYTE ledCmd)
{
    int ledStat = 0;   
    if (bExtReset == FALSE)
        return;         
    if (IsZeroID(AppEepromData.eLoggerID))
        return;  
    ledStat = (nLedIndex | (ledCmd << 8));  
    SendRecRS485(CMD_LED, ledStat);
}

void SetUART1BaudRate(BYTE rate)
{
    if (rate == RATE9600)
        UBRR1L = 0x2F;
    if (rate == RATE38400)  
        UBRR1L=0x0B;
    if (rate == RATE19200)  
        UBRR1L=0x17;  
    delay_ms(100);
}

void InitPeripherals()
{
    // USART0 initialization
    // Communication Parameters: 8 Data, 1 Stop, No Parity
    // USART0 Receiver: On
    // USART0 Transmitter: On
    // USART0 Mode: Asynchronous
    // USART0 Baud Rate: 19200
    UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
    DISABLE_UART0();
    UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
    UBRR0H=0x00;
    UBRR0L=0x17;     //0x0B;        //0x01; //

    // USART1 initialization
    // Communication Parameters: 8 Data, 1 Stop, No Parity
    // USART1 Receiver: On
    // USART1 Transmitter: On
    // USART1 Mode: Asynchronous
    // USART1 Baud Rate: 19200
    UCSR1A=(0<<RXC1) | (0<<TXC1) | (0<<UDRE1) | (0<<FE1) | (0<<DOR1) | (0<<UPE1) | (1<<U2X1) | (0<<MPCM1);
//    DISABLE_UART1();
    UCSR1C=(0<<UMSEL11) | (0<<UMSEL10) | (0<<UPM11) | (0<<UPM10) | (0<<USBS1) | (1<<UCSZ11) | (1<<UCSZ10) | (0<<UCPOL1);
    UBRR1H=0x00;
//    UBRR1L=0x17;      //Baud Rate: 9600
//    UBRR1L=0x2F;        // 9600 x2
    SetUART1BaudRate(RATE38400);
//    UBRR1L=0x0B;        //Baudrate 38400 x2
//    UBRR1L=0x0B;       // Baud Rate: 19200

    // ADC initialization
    // ADC Clock frequency: 28.800 kHz
    // ADC Voltage Reference: 2.56V, cap. on AREF
    // ADC Auto Trigger Source: ADC Stopped
    // Digital input buffers on ADC0: On, ADC1: On, ADC2: On, ADC3: On, ADC4: On
    // ADC5: Off, ADC6: Off, ADC7: Off
    DIDR0=(1<<ADC7D) | (1<<ADC6D) | (1<<ADC5D) | (1<<ADC4D) | (1<<ADC3D) | (1<<ADC2D) | (0<<ADC1D) | (1<<ADC0D);
    ADMUX = ADC_VREF_TYPE;
    ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    ADCSRB=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);

    // SPI initialization
    // SPI disabled
    SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

    // TWI initialization
    // TWI disabled
    TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);
}

void DefineBtrStatus()
{
    BYTE prevBtrStatus = btrStatus;   
    
    if (iVoltage < BTR_EMPTY_LIMIT)         // <3500
        btrStatus = BTR_STATUS_EMPTY;
    else     
        if (iVoltage > BTR_FULL_LIMIT)               //>3600
            btrStatus = BTR_STATUS_FULL; 
        else             
            btrStatus = prevBtrStatus;

    #ifdef DebugMode
    SendDebugMsg("\r\nBtr Status= ");
    PrintNum(btrStatus);
    #endif DebugMode              
}

void WakeUpProcedure(void)
{
    BYTE prevBtrStatus = btrStatus;//, btrStatusCng = FALSE;  
    
//    if ((prevMainTask != TASK_SLEEP) && (bExtReset == FALSE))
//    {         
//         #ifdef DebugMode
//         SendDebugMsg("\r\nno need WakeUp ");
//        #endif DebugMode       
//        return;                                                                                                                                                         
//    }     
    InitPeripherals();
    ENABLE_TIMER1_COMPA();  
//    nTicks = 0;
    #ifdef DebugMode
    ENABLE_UART1();
    #endif DebugMode  
    //wd on
    #pragma optsize-
    #asm("wdr")
    WATCHDOG_ENABLE_STEP1();
    WATCHDOG_ENABLE_STEP2();
    #ifdef _OPTIMIZE_SIZE_
    #pragma optsize+
    #endif
//    bEndOfMeasureTask = FALSE;
    bEndOfModemTask = FALSE;
    bEndOfMonitorTask = TRUE;//FALSE;
    bEndOfCbuTask = FALSE;  
    g_bMngPumpNow = FALSE;  
    
	//set condition for rtc communication
    SPCR=0x00; //reset spi control register
    if(IsPowerFlagOn()) //check if clock power flag is on
    {
        delay_ms(500);
        InitRTC();    //initiate the clock
        SetRtc24Hour(); //config rtc to am-pm mode
    }          
//    if (PINA.3 == 0)
//        SendDebugMsg("\r\nCharging");
//    else               
//        SendDebugMsg("\r\nNO Charging");
//    n = PORTA;        
    if ((PORTA & (1 << PORTA3)) == 0)
        SendDebugMsg("\r\nCharging");
    else               
        SendDebugMsg("\r\nNO Charging");
    MeasureBatt();
    DefineBtrStatus();    
    ReadPointers();  

    if ((btrStatus == BTR_STATUS_EMPTY) && (prevBtrStatus != BTR_STATUS_EMPTY))  // if now battery is empty but wasnt before
    {                                                         
        UART1Select(UART_NONE);        
        WIRELESS_CTS_DISABLE();
        WIRELESS_CTS_ON();
        WIRELESS_PWR_DISABLE();    // switch off wireless
        #ifdef DebugMode
        SendDebugMsg("\r\nbattery too low. disable wireless\0");
        #endif DebugMode    
//        gClosePumpDelay = 0;  
        CloseMainPump(TRUE);       
//        btrStatusCng = TRUE;
    }
    else
    {
        if ((btrStatus != BTR_STATUS_EMPTY) && (prevBtrStatus == BTR_STATUS_EMPTY))     // if battery was empty but no more
        {                  
//            btrStatusCng = TRUE;
            WIRELESS_CTS_ENABLE();
            WIRELESS_CTS_OFF();
            WIRELESS_PWR_ENABLE();                                      // switch on wireless
            #ifdef DebugMode
            SendDebugMsg("\r\nbattery back normal. enable wireless\0");
            #endif DebugMode                      
        }
    }     
}

void GetNextMainTask()
{
    int wuAlert = 100;          

    mainTask = TASK_SLEEP;   
    GetRealTime();       
    #ifdef DebugMode
    SendDebugMsg("\r\nWakeup. Time is: \0");       
//    PrintTime(g_curTime);
    #endif DebugMode    
        
    if (g_fRS485Call == 1)
        g_fRS485Call = 2;
    else          
        if (bExtReset == FALSE)
        {               
            if ((btrStatus == BTR_STATUS_FULL))
            {       
                 if (SendRecRS485(CMD_GET_MNT_DATA, 1) != 1)  //if failed to get data from CBU - retry once   
                    SendRecRS485(CMD_GET_MNT_DATA, 1);
                if (g_curTime.minute == 0)           
                {
                    SendRecRS485(CMD_GET_CBU_DEF,0);    //CMD_GET_PORT_DEF,0); 
                    PrintSensorDef();   
                }       
                
//                #ifdef DebugMode
//                SendDebugMsg("\r\nGet data from EZR!\0");
//                #endif DebugMode   
                
//                mainTask = TASK_EZR_COM;
//                msrCurTask = TASK_NONE;    
                InitVarsForConnecting();
//                fListenOrSend = 1;                        
            }     
        }
        else
        {        
            SaveAlertData((unsigned char*)&wuAlert);      
            SendRecRS485(CMD_GET_CBU_DEF,0);    
//            SendRecRS485(CMD_PUMP1_MNG, PUMP_CMD_OFF);
//            SendRecRS485(CMD_PUMP2_MNG, PUMP_CMD_OFF);
              
            PrintSensorDef();                                
            mainTask = TASK_MONITOR;
            monitorCurTask = TASK_MONITOR_CONNECT; 
            bEndOfMonitorTask = FALSE;
        }
                                          
    bWaitForModemAnswer = FALSE;
    bCheckRxBuf = FALSE;
    bNeedToWait4Answer = TRUE;
    bWaitForMonitorCmd = FALSE;
    bCheckRx1Buf = FALSE;
    nTimeCnt = 0;      
    g_PumpCmdNow = PUMP_NONE; 
//    g_bHighPrio = FALSE;             
}

void WDT_off(void)
{
    //__disable_interrupt();
	#asm ("wdr"); 		//reset the watchdog
    /* Clear WDRF in MCUSR */
    MCUSR &= ~(1<<WDRF);
    /* Write logical one to WDCE and WDE */
    /* Keep old prescaler setting to prevent unintentional time-out */
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    /* Turn off WDT */
    WDTCSR = 0x00;
    //__enable_interrupt();
}

void PowerDownSleep( void )
{
//    BYTE modemStat;   
    #ifdef DebugMode
    SendDebugMsg("\r\nGo2Sleepõ g_bModemConnect = \0");     
    PrintNum(g_bModemConnect);
    #endif DebugMode                   
    HandleLed(LED_ALL, CBU_LED_OFF);
    bExtReset = FALSE;  
    prevMainTask = TASK_NONE; 
    g_bExtIntDtct = FALSE;   
    SavePointers();
    WDT_off();  
//    HandleLed(LED_2, CBU_LED_OFF);
//    HandleLed(LED_3, CBU_LED_OFF);

 //   TurnOffLed();     
//    modemStat = IsModemOn();
    //Direction - 0 = Input, 1= Output
    //State - if direction is Input:
    //                  1 = enable Pull up
    //                  0 = disable Pull up, become tri state
    // Input/Output Ports initialization for sleep         
     if (btrStatus == BTR_STATUS_EMPTY)    
     {
        // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
        DDRA=(0<<DDA7) | (1<<DDA6) | (1<<DDA5) | (0<<DDA4) | (0<<DDA3) | (1<<DDA2) | (0<<DDA1) | (0<<DDA0);
        // State: Bit7=0 Bit6=0 Bit5=0 Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
        PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (1<<PORTA3) | (1<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);      
     }
     else
     {
        // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
        DDRA=(1<<DDA7) | (1<<DDA6) | (1<<DDA5) | (0<<DDA4) | (0<<DDA3) | (1<<DDA2) | (0<<DDA1) | (0<<DDA0);
        // State: Bit7=0 Bit6=0 Bit5=0 Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
        PORTA=(1<<PORTA7) | (1<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (1<<PORTA3) | (1<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);      
    }
             
    // Port B initialization
    #ifdef BRD1000_V3
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=out Bit0=Out
    DDRB=(0<<DDB7) | (1<<DDB6) | (0<<DDB5) | (0<<DDB4) | (1<<DDB3) | (0<<DDB2) | (0<<DDB1) | (1<<DDB0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (1<<PORTB2) | (1<<PORTB1) | (1<<PORTB0);
    #else
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=out Bit0=Out
    DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (1<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (1<<PORTB2) | (1<<PORTB1) | (0<<PORTB0);
    #endif BRD1000_V3
    // Port C initialization
    // Function: Bit7=In Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=In Bit0=In
    DDRC=(0<<DDC7) | (0<<DDC6) | (1<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (0<<DDC1) | (1<<DDC0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=T Bit0=T
    PORTC=(0<<PORTC7) | (1<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (1<<PORTC0); 

    // Port D initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=In Bit2=In Bit1=In Bit0=In
    DDRD=(1<<DDD7) | (0<<DDD6) | (1<<DDD5) | (1<<DDD4) | (1<<DDD3) | (0<<DDD2) | (1<<DDD1) | (0<<DDD0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=T Bit2=T Bit1=T Bit0=T                       
    if (g_bModemConnect == TRUE)
        PORTD=(1<<PORTD7) | (1<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (1<<PORTD3) | (0<<PORTD2) | (1<<PORTD1) | (0<<PORTD0);
    else
        PORTD=(0<<PORTD7) | (1<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (1<<PORTD3) | (0<<PORTD2) | (1<<PORTD1) | (0<<PORTD0);


    DISABLE_UART1();
    DISABLE_TIMER1_COMPA();
    EIFR |= (1<<INTF2); //reset clock flag interrupt
    DisableClockIntr();
    ResetClockIntr();//(0);
    
    SMCR |= 4;
    SMCR |= 1;
    PRR = 0xFF;
    ENABLE_CLOCK_INT(); // enable external interrupt2
//    bPowerDown = TRUE;
    #asm
    sleep
    #endasm

    #asm ("nop\nop"); 		//wakeup from sleep mode
}

void InitProgram(void)
{
//    char i;             
    #ifdef DebugMode
    char l[4];      
    #endif DebugMode

    // Crystal Oscillator division factor: 1
    #pragma optsize-
    CLKPR=(1<<CLKPCE);
    CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
    #ifdef _OPTIMIZE_SIZE_
    #pragma optsize+
    #endif

    //Direction - 0 = Input, 1= Output
    //State - if direction is Input:
    //                  1 = enable Pull up
    //                  0 = disable Pull up, become tri state
    // Input/Output Ports initialization
    // Port A initialization         
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRA=(1<<DDA7) | (1<<DDA6) | (1<<DDA5) | (0<<DDA4) | (0<<DDA3) | (1<<DDA2) | (0<<DDA1) | (0<<DDA0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTA=(1<<PORTA7) | (1<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (1<<PORTA3) | (1<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);  

    // Port B initialization  
    #ifdef BRD1000_V3
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=out Bit0=Out
    DDRB=(0<<DDB7) | (1<<DDB6) | (0<<DDB5) | (0<<DDB4) | (1<<DDB3) | (0<<DDB2) | (0<<DDB1) | (1<<DDB0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (1<<PORTB2) | (1<<PORTB1) | (1<<PORTB0);    
    #else
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=out Bit0=Out
    DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (1<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (1<<PORTB2) | (1<<PORTB1) | (0<<PORTB0);    
    #endif    

    // Port C initialization
    // Function: Bit7=In Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=In Bit0=In
    DDRC=(0<<DDC7) | (0<<DDC6) | (1<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (0<<DDC1) | (1<<DDC0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=T Bit0=T
    PORTC=(0<<PORTC7) | (1<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (1<<PORTC0); 

    // Port D initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=In Bit2=In Bit1=In Bit0=In
    DDRD=(1<<DDD7) | (0<<DDD6) | (1<<DDD5) | (1<<DDD4) | (1<<DDD3) | (0<<DDD2) | (1<<DDD1) | (0<<DDD0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=T Bit2=T Bit1=T Bit0=T
    PORTD=(0<<PORTD7) | (1<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (1<<PORTD3) | (0<<PORTD2) | (1<<PORTD1) | (0<<PORTD0);

//    WIRELESS_PWR_ENABLE();
    // Timer/Counter 0 initialization
    // Clock source: System Clock
    // Clock value: 3.600 kHz
    // Mode: Normal top=0xFF
    // OC0A output: Disconnected
    // OC0B output: Disconnected
    // Timer Period: 71.111 ms
    TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
    DISABLE_TIMER0();
    //TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
    TCNT0=0x00;
    OCR0A=0x00;
    OCR0B=0x00;

    // Timer/Counter 1 initialization
    // Clock source: System Clock
    // Clock value: 460.800 kHz
    // Mode: CTC top=OCR1A
    // OC1A output: Disconnected
    // OC1B output: Disconnected
    // Noise Canceler: Off
    // Input Capture on Falling Edge
    // Timer Period: 0.1 s
    // Timer1 Overflow Interrupt: Off
    // Input Capture Interrupt: Off
    // Compare A Match Interrupt: On
    // Compare B Match Interrupt: Off
    TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
    TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
    TCNT1H=0x4C;
    TCNT1L=0x00;
    ICR1H=0x00;
    ICR1L=0x00;
    OCR1AH=0xB3;
    OCR1AL=0xFF;
    OCR1BH=0x00;
    OCR1BL=0x00;


    // Timer/Counter 2 initialization
    // Clock source: System Clock
    // Clock value: 3.600 kHz
    // Mode: Normal top=0xFF
    // OC2A output: Disconnected
    // OC2B output: Disconnected
    // Timer Period: 71.111 ms
    ASSR=(0<<EXCLK) | (0<<AS2);
    TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
    //TCCR2B=(0<<WGM22) | (0<<CS22) | (0<<CS21) | (0<<CS20);
    DISABLE_TIMER2();
    TCNT2=0x00;
    OCR2A=0x00;
    OCR2B=0x00;


    // Timer/Counter 0 Interrupt(s) initialization
    TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);

   // Timer/Counter 1 Interrupt(s) initialization
    TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);

    // Timer/Counter 2 Interrupt(s) initialization
    TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (1<<TOIE2);

    // External Interrupt(s) initialization
    // INT0: Off
    // INT1: Off
    // INT2: On
    // INT2 Mode: Falling Edge
    // Interrupt on any change on pins PCINT0-7: On
    // Interrupt on any change on pins PCINT8-15: Off
    // Interrupt on any change on pins PCINT16-23: Off
    // Interrupt on any change on pins PCINT24-31: Off
    EICRA=(1<<ISC21) | (0<<ISC20) | (0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
    EIMSK=(0<<INT2) | (0<<INT1) | (0<<INT0);
    EIFR=(1<<INTF2) | (0<<INTF1) | (0<<INTF0);
//    PCMSK1=(0<<PCINT15) | (0<<PCINT14) | (0<<PCINT13) | (1<<PCINT12) | (0<<PCINT11) | (0<<PCINT10) | (0<<PCINT9) | (0<<PCINT8);
//    PCICR=(0<<PCIE3) | (0<<PCIE2) | (1<<PCIE1) | (0<<PCIE0);
//    PCMSK0=(0<<PCINT7) | (0<<PCINT6) | (0<<PCINT5) | (0<<PCINT4) | (0<<PCINT3) | (1<<PCINT2) | (0<<PCINT1) | (0<<PCINT0);
//    PCICR=(0<<PCIE3) | (0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);
//    PCIFR=(0<<PCIF3) | (0<<PCIF2) | (0<<PCIF1) | (1<<PCIF0);

// pin change int on D6
//PCMSK3=(0<<PCINT31) | (1<<PCINT30) | (0<<PCINT29) | (0<<PCINT28) | (0<<PCINT27) | (0<<PCINT26) | (0<<PCINT25) | (0<<PCINT24);
//PCICR=(1<<PCIE3) | (0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);
//PCIFR=(1<<PCIF3) | (0<<PCIF2) | (0<<PCIF1) | (0<<PCIF0);
//////////////// pin change int on b1////////////////////
PCMSK1=(0<<PCINT15) | (0<<PCINT14) | (0<<PCINT13) | (0<<PCINT12) | (0<<PCINT11) | (0<<PCINT10) | (1<<PCINT9) | (0<<PCINT8);
PCICR=(0<<PCIE3) | (0<<PCIE2) | (1<<PCIE1) | (0<<PCIE0);
PCIFR=(0<<PCIF3) | (0<<PCIF2) | (1<<PCIF1) | (0<<PCIF0);
//////////////////  BOTH  /////////////////////////////////
//PCICR=(1<<PCIE3) | (0<<PCIE2) | (1<<PCIE1) | (0<<PCIE0);
//PCIFR=(1<<PCIF3) | (0<<PCIF2) | (1<<PCIF1) | (0<<PCIF0);
//////////////////////////////////////////////////////////


    InitPeripherals();
    // Analog Comparator initialization
    // Analog Comparator: Off
    ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
    ADCSRB=(0<<ACME);
    DIDR1=0x00;
    // Watchdog Timer initialization
    // Watchdog Timer Prescaler: OSC/1024k
    // Watchdog Timer interrupt: Off
    MCUSR |= (1<<WDRF);
#pragma optsize-
    #asm("wdr")
    WATCHDOG_ENABLE_STEP1();
    WATCHDOG_ENABLE_STEP2();
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif
    #ifdef DebugMode
    ENABLE_UART1();
    SendDebugMsg("\r\nexternal reset!\0");
    SendDebugMsg("\r\nLogger# \0"); 
    cpu_e2_to_MemCopy(l, AppEepromData.eLoggerID, 4);  
    PrintNum(Bytes2ULong(l));            
//    Myprintf("SW Version: %c.%d.%d.%d\0", RomVersion[0], RomVersion[1], RomVersion[2], RomVersion[3]);  
//    SendDebugMsg(RomVersion);
    #endif DebugMode      
    SetRtc24Hour(); //config rtc to am-pm mode
    ReadPointers();
    if (ValidatePointers() == FALSE)
        ResetPointers();       
    mainTask = TASK_WAKEUP;
    // if modem is on from any reason - first of all turn it off
//    if (IsModemOn() == TRUE)
//    {            
//        ShutDownModem();
//    }
    bMonitorConnected = FALSE;

    btrStatus = BTR_STATUS_EMPTY;   //SHUTDOUW;
    g_fRS485Call = 0;    
    g_LockUar1 = FALSE;   
    InitVersiontoUpdt();  
#ifndef     NO_EZR
    g_bMainPumpOpen = FALSE;  
    InitValvesArrays();   
    InitPumpCmdUnit();       
    LoadVCUIds();       
    g_bVlvListUpdated = true;
#endif     NO_EZR
    InitOperatorLst();
    ResetEzrVerNum();    
    HandleLed(LED_ALL, CBU_LED_OFF);   
    delay_ms(20); 
    HandleLed(LED_1, CBU_LED_BLINK);
}

BYTE UART1Select(BYTE newState)//uartTarget)
{
    unsigned char curState = PORTD;
    unsigned char  tmp; //newState,
/*    switch (uartTarget)
    {
    case UART_RADIO_UHF:
        newState = 0x0;
    break;
    case UART_RS485:
        newState = 0x10;
    break;
    case UART_DBG:
        newState = 0x20;
    break;
    case UART_NONE:
        newState = 0x30;
    break;
    }       */
    if (g_LockUar1 == TRUE)
        return FALSE;
    tmp = curState & 0x30;
    if (tmp == newState)
        return 0;
    curState = curState & ~0x30;
    newState = curState | newState;
    PORTD = newState; 
    return 1;
}

#ifdef LiteDebug
//convert int less than 1000000 to string and send string to uart1
void PrintNum(long val)
{
    char s[10];
    BYTE i = 0;
    if ((mainTask == TASK_EZR_COM)) //|| (mainTask == TASK_EZR))
        return;
    if (val < 0)
    {
        putchar1('-');
        val *= -1;
    }

    do
    {
        s[i++] = (char)(val % 10);
        val = val / 10;
    }
    while (val > 0);
    for (; i > 0; i--)
        putchar1(s[i-1] + 48);
    putchar1('#');  //HASHTAG);
    putchar1('\r\n');
}
 #endif LiteDebug    
 
#ifdef DebugMode    //LiteDebug    //
void SendDebugMsg(flash unsigned char *bufToSend)
{
    int i;
    if ((mainTask == TASK_EZR_COM))// || (mainTask == TASK_EZR))
        return;
    if (UART1Select(UART_DBG) == 1)
        delay_ms(30);//50
    i = 0;
    //copy flash string to buff
    while ((bufToSend[i] != '\0') && (i < MAX_RX1_BUF_LEN))
    {
         DbgBuf[i] = bufToSend[i];
         //putchar1(ComBuf[i]);
         i++;
    }
    BytesToSend = i ;
    //copy the ComBuf into eeprom (for debug)
    //MemCopy_to_cpu_e2(&Store_tx_buff[0], ComBuf, BytesToSend);
    //transmitt to local modem port
    TransmitBuf(2);
//    bWaitForModemAnswer = FALSE;
}

void PrintOnlyNum(long val)
{
    unsigned int i; 
    char t[10];

    i = 0;
    if(val < 0) 
    { 
        val = -val;
        putchar1('-'); 
    }   
    do
    {
        t[i++] = (char)(val % 10);
        val = val / 10;
    }
    while (val > 0);
    for (; i > 0; i--)
        putchar1(t[i-1] + 48);
}

/*void Myprintf(char* format,...) 
{ 
    char *traverse; 
    unsigned int i; 
//    char *s;         
    long val;
//    char t[10];
                  
    //Module 1: Initializing Myprintf's arguments 
    va_list arg; 
    va_start(arg, format); 

    putchar1('\r');
    putchar1('\n');

    for(traverse = format; *traverse != '\0'; traverse++) 
    { 
        while( *traverse != '%' ) 
        { 
            putchar1(*traverse);
            traverse++; 
        } 

        traverse++; 

        //Module 2: Fetching and executing arguments
        switch(*traverse) 
        { 
            case 'c' : i = va_arg(arg,int);     //Fetch char argument
                        putchar1(i);
                        break; 

            case 'd' : val = va_arg(arg,int);         //Fetch Decimal/Integer argument      
                        PrintOnlyNum(val);
                        
//                        i = 0;
//                        if(val < 0) 
//                        { 
//                            val = -val;
//                            putchar1('-'); 
//                        }   
//                        do
//                        {
//                            t[i++] = (char)(val % 10);
//                            val = val / 10;
//                        }
//                        while (val > 0);
//                        for (; i > 0; i--)
//                            putchar1(t[i-1] + 48);
//
//                        //puts(convert(i,10));   
                        break; 

//            case 'o': i = va_arg(arg,unsigned int); //Fetch Octal representation
//                        puts(convert(i,8));
//                        break; 
//
//            case 's': s = va_arg(arg,char *);       //Fetch string
//                        puts(s); 
//                        break; 
//
//            case 'x': i = va_arg(arg,unsigned int); //Fetch Hexadecimal representation
//                        puts(convert(i,16));
//                        break; 
        }   
    } 

    //Module 3: Closing argument list to necessary clean-up
    va_end(arg);     
    
}    

void PrintTime(DateTime dt)
{
//    Myprintf("time: %d.%d.%d %d:%d", dt.year, dt.month, dt.day, dt.hour, dt.minute);  
    #ifdef DebugMode   
    SendDebugMsg("Time: ");
    PrintNum(dt.hour);
    PrintNum(dt.minute);
    #endif DebugMode
}    */

/*void Myprintf(char* format,...) 
{ 
    char *traverse; 
    unsigned int n, i, idx = 0; 
//    char *s;         
    long val;
    char t[10];

//DbgBuf[i] 
                 
    //Module 1: Initializing Myprintf's arguments 
    va_list arg; 
    va_start(arg, format); 

    DbgBuf[i++] = '\r';
    DbgBuf[i++] = '\n';

    for(traverse = format; *traverse != '\0'; traverse++) 
    { 
        while( *traverse != '%' ) 
        { 
            DbgBuf[i++] = *traverse;
            traverse++; 
        } 

        traverse++; 

        //Module 2: Fetching and executing arguments
        switch(*traverse) 
        { 
            case 'c' : i = va_arg(arg,int);     //Fetch char argument
                        DbgBuf[i++] = i;
                        break; 

            case 'd' : val = va_arg(arg,int);         //Fetch Decimal/Integer argument      
                        //PrintOnlyNum(val); 
                        n = 0;
                        if(val < 0) 
                        { 
                            val = -val;
                            DbgBuf[i++] = '-'; 
                        }   
                        do
                        {
                            t[i++] = (char)(val % 10);
                            val = val / 10;
                        }
                        while (val > 0);
                        for (; i > 0; i--)
                            DbgBuf[i++] = (t[i-1] + 48);
                        
                        break; 

//            case 'o': i = va_arg(arg,unsigned int); //Fetch Octal representation
//                        puts(convert(i,8));
//                        break; 
//
//            case 's': s = va_arg(arg,char *);       //Fetch string
//                        puts(s); 
//                        break; 
//
//            case 'x': i = va_arg(arg,unsigned int); //Fetch Hexadecimal representation
//                        puts(convert(i,16));
//                        break; 
        }   
    } 

    //Module 3: Closing argument list to necessary clean-up
    va_end(arg); 
    rx1_buff_len = 0;  
    i = 0;
    while(idx-- )
    {
        while ((UCSR1A & DATA_REGISTER_EMPTY)==0);
        UDR1 = DbgBuf[i++];    // send next byte..
    }
     
    
} */

/*void MyprintList(char* format,...) 
{ 
    unsigned int i, j, max; 
    int num;
    char t[10];

    //Module 1: Initializing Myprintf's arguments 
    va_list arg; 
    va_start(arg, format); 

    max = va_arg(arg,int);
    for(j = 0; j < max; j++) 
    { 
        num = va_arg(arg,int);         //Fetch Decimal/Integer argument      
        i = 0;
        if(val < 0) 
        { 
            val = -val;
            putchar1('-'); 
        }   
        do
        {
            t[i++] = (char)(val % 10);
            val = val / 10;
        }
        while (val > 0);
        for (; i > 0; i--)
            putchar1(t[i-1] + 48);
        putchar1(',');                        
    } 

    //Module 3: Closing argument list to necessary clean-up
    va_end(arg); 
}     */

#endif DebugMode    //LiteDebug    //

void DeepSleep( void )
{
    #ifdef DebugMode
    SendDebugMsg("\r\nGo2DeepSleep\0");
    #endif DebugMode
    bExtReset = TRUE;             
    HandleLed(LED_ALL, CBU_LED_OFF);
    bExtReset = FALSE;             
    
    // Port A initialization         
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In
    DDRA=(1<<DDA7) | (1<<DDA6) | (1<<DDA5) | (0<<DDA4) | (0<<DDA3) | (1<<DDA2) | (0<<DDA1) | (0<<DDA0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTA=(1<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (1<<PORTA3) | (1<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);  

    // Port B initialization         
    #ifdef BRD1000_V3
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=out Bit0=Out
    DDRB=(0<<DDB7) | (1<<DDB6) | (0<<DDB5) | (0<<DDB4) | (1<<DDB3) | (0<<DDB2) | (1<<DDB1) | (1<<DDB0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (1<<PORTB2) | (0<<PORTB1) | (1<<PORTB0);   
    #else
    // Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=out Bit0=Out
    DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (1<<DDB3) | (0<<DDB2) | (1<<DDB1) | (0<<DDB0);
    // State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T
    PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (1<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);   
    #endif

    // Port C initialization
    // Function: Bit7=In Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=In Bit0=In
    DDRC=(0<<DDC7) | (0<<DDC6) | (1<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (0<<DDC1) | (1<<DDC0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=T Bit0=T
    PORTC=(0<<PORTC7) | (1<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (1<<PORTC0); 

    // Port D initialization
    // Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=In Bit2=In Bit1=In Bit0=In
    DDRD=(1<<DDD7) | (0<<DDD6) | (1<<DDD5) | (1<<DDD4) | (1<<DDD3) | (0<<DDD2) | (1<<DDD1) | (0<<DDD0);
    // State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=T Bit2=T Bit1=T Bit0=T
    PORTD=(0<<PORTD7) | (1<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (1<<PORTD3) | (0<<PORTD2) | (1<<PORTD1) | (0<<PORTD0);

//    TurnOffLed();
    WDT_off();

    DISABLE_UART1();
    WIRELESS_PWR_DISABLE();
//    ENABLE_TAG_INT();
    DISABLE_TIMER1_COMPA();
    EIFR |= (1<<INTF2); //reset clock flag interrupt
    DisableClockIntr();

    SMCR |= 4;
    SMCR |= 1;
    PRR = 0xFF;

    #asm
    sleep
    #endasm

    #asm ("nop\nop"); 		//wakeup from sleep mode
}


////////////////end of general.c////////////////