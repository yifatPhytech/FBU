#ifndef __RTC_H
#define __RTC_H

extern char clockBuf[7];          //buffer for all clock operation need
extern char cmdByte;             //buffer for the command byte
extern char readClockBuf[7];             //buffer for data reading from clock
extern DateTime g_curTime;

/////////////////////////////////////////////
// RTC functions
////////////////////////////////////////////
//power on initialization ruthin
//check if power flag is on, if yes you should preform reset to r.t.cclock
unsigned char IsPowerFlagOn(void);

//unsigned char ResetCommand(void);
void ResetCommand(void);

//unsigned char DisableClockIntr(void);
void DisableClockIntr(void);

unsigned char SetRealTime(void);

//read the clock data into buffer
unsigned char ReadTime(void);

//initiate the rtc at program startup
void InitRTC();

//unsigned char ResetClockIntr(unsigned char);
void ResetClockIntr();

void SetRtc24Hour();

void GetRealTime(void);
#endif __RTC_H
