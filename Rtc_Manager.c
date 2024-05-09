#ifndef __RTC_C
#define __RTC_C
#include <bcd.h>    //for bcd 2 bin conversion
#include "define.h"
#include "Rtc_Manager.h"
#include "i2c_bus.h"

//declare local & global variables
char clockBuf[7];          //buffer for all clock operation need
char cmdByte;             //buffer for the command byte
char readClockBuf[7];             //buffer for data reading from clock
extern char e2_writeFlag;
DateTime g_curTime;

void RtcConfig(char cmd, char prm)
{
    do
    {
        clockBuf[0] = prm;
        SendBuf(cmd , 1, clockBuf);
        cmdByte = cmd + 1;	    // change command to read
        clockBuf[0] = 0x00;	    // init buf
        GetBuf(cmdByte, 1, clockBuf);      // read rtc
    }
    while (!(clockBuf[0] == prm));             // check config was set OK
}

//return the buf with upsidedown bits order for the use of data "writen" into the rtc
unsigned char ByteUpsideDown(unsigned char byteBuf)
{
    unsigned char i;
    unsigned char temp = 0;
    unsigned char bufTemp;
    bufTemp = byteBuf;
    for(i = 0; i < 8; i++)
    {
        temp = temp << 1;
        temp |=  (bufTemp & 0x01);
        bufTemp = bufTemp >> 1;
    }
	return temp;
}

//arrange the clock buf before setiing time into rtc
void SetClockBuf(void)
{
	unsigned char i, temp;

	//the clockBuf will be set by the communication module
	//change the fuffer to bcd format and up side down
	for(i = 0; i < 7; i++)
	{
		temp = clockBuf[i];
	 	clockBuf[i] = bin2bcd(temp);
		temp = clockBuf[i];
		clockBuf[i] = ByteUpsideDown(temp);
	}
}

//power on initialization ruthin
//check if power flag is on, if yes you should preform reset to r.t.cclock
unsigned char IsPowerFlagOn(void)
{
    BYTE res = FALSE;
    cmdByte = 0x61;	 //status register 1

    do
    {
        res = GetBuf(cmdByte, 1, readClockBuf);
    }
    while (res == FALSE);

    if (readClockBuf[0] & 0x01)
        return TRUE;

	return FALSE;
}

void ResetCommand(void)
{
    do
    {
        cmdByte = 0x60;	 //status register 1 access
        clockBuf[0] = 0xC0;
        SendBuf(cmdByte , 1, clockBuf); // FALSE)	 //if SendBuf function faild
        cmdByte = 0x61;
        clockBuf[0] = 0;
        GetBuf(cmdByte , 1, clockBuf);
    }
    while (!(clockBuf[0] & 0x40));		//   24 hours config is set
}

void SetRtc24Hour()
{
    RtcConfig(0x60, 0x40);
}

//set the status register for interrupt
void ResetClockIntr()
{
    RtcConfig(0x62, 0x0C);  // 0x0C - cancel 32Khz to EZR         //RtcConfig(0x62, 0x1C);
    
}

void DisableClockIntr(void)
{
    RtcConfig(0x62, 0x00);
    RtcConfig(0x62, 0x10);
}

unsigned char SetRealTime(void)
{
    BYTE bCheck = FALSE;

	cmdByte = 0x64;   //preper real Time setting command
    // check if hour is in PM period
    if (clockBuf[4] > 11)
    {
        bCheck = TRUE;
    }
    if (!((clockBuf[1] >= 1) && (clockBuf[1] <= 12)) && ((clockBuf[2] >= 1) && (clockBuf[2] <= 31))
    && ((clockBuf[4] >= 0) && (clockBuf[4] <24)) && ((clockBuf[5] >= 0) && (clockBuf[5] < 60))) // valid month ,valid day in month ,valid hour, valid minute
    {
        #ifdef DebugMode
        SendDebugMsg("\r\ninvalid data\0");
        #endif DebugMode
        return FAILURE;
    }

    SetClockBuf(); //set clockBuf for time setting
	if(SendBuf(cmdByte , 7, clockBuf) == FALSE) //if SendBuf function faild
	{
		return FAILURE;
	}
    if (bCheck)
    {
        if (ReadTime() == SUCCESS)
        {
            // if hour after setup is 0-i.e. clock couldn't get pm hour
            if (readClockBuf[4] == 0)
            {
                SetRtc24Hour(); //config rtc to am-pm mode
            }
        }
    }
	return SUCCESS;
}

//read the clock data into buffer
unsigned char ReadTime(void)
{
    unsigned char i, temp;
	cmdByte = 0x65;			//read time data registers

	if(GetBuf(cmdByte , 7, readClockBuf) == FALSE)	//if getBuf function faild
	{
        return FAILURE;
	}

	//change the hour reading (ignor the bit 7,6)
	readClockBuf[4] = readClockBuf[4] & 0xFC;
	//change the buffer to normal reading order
	for(i = 0; i < 7; i++)
	{
		readClockBuf[i] = ByteUpsideDown(readClockBuf[i]);
		temp = readClockBuf[i];
	 	readClockBuf[i] = bcd2bin(temp);
	}

	//calculate the time from day start in minutes
    g_curTime.year = readClockBuf[0];
    g_curTime.month = readClockBuf[1];
    g_curTime.day = readClockBuf[2];
    g_curTime.hour = readClockBuf[4];
    g_curTime.minute = readClockBuf[5];
    g_curTime.second = readClockBuf[6]; 
	return SUCCESS;
}

//initiate the rtc at program startup
void InitRTC()
{
    ResetCommand();
    ResetClockIntr(); //1 = freq.interupt; 0 = per minute adge interrupt
    ReadTime();
}

//read the rtc
void GetRealTime(void)
{
	if(e2_writeFlag) //ignor reading if ext_c2 is busy
		return;

    SPCR = 0x00; //reset spi control register

   	ReadTime();
}
#endif  __RTC_C
