/*
 * epoch.c
 *
 *  Created on: Jan 11, 2015
 *      Author: reuben
 */
#ifdef Satellite
//#include "epoch.h"
#include "define.h"

// Iridium Epoch Time : 11/05/2014 14:23:55
eeprom date_time_t ee_epoch_time = {55,23,14,11,5,14};
extern eeprom int eTimeZoneOffset;                         //offset (in minutes) from UTC

const unsigned short days[4][12] =
{
    {   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335},
    { 366, 397, 425, 456, 486, 517, 547, 578, 609, 639, 670, 700},
    { 731, 762, 790, 821, 851, 882, 912, 943, 974,1004,1035,1065},
    {1096,1127,1155,1186,1216,1247,1277,1308,1339,1369,1400,1430},
};

unsigned long date_time_to_epoch(eeprom date_time_t* date_time)
{
	unsigned long x,y;
    unsigned char second = date_time->second;  // 0-59
    unsigned char minute = date_time->minute;  // 0-59
    unsigned char hour   = date_time->hour;    // 0-23
    unsigned char day    = date_time->day-1;   // 0-30
    unsigned char month  = date_time->month-1; // 0-11
    unsigned char year   = date_time->year;    // 0-99
    x = year/4*(365*4+1)+ days[year%4][month] + day;
    y = x*24 + hour;
    x = y*60 + minute;
    y = x*60 + second;
    //return (((year/4*(365*4+1)+days[year%4][month]+day)*24+hour)*60+minute)*60+second;
    return y;
}

void epoch_to_date_time(date_time_t* date_time, unsigned long epoch)
{
    unsigned int years;
    unsigned int year;
    unsigned int month;

    date_time->second = epoch%60;
    epoch /= 60;
    epoch += eTimeZoneOffset;   // subtract timezone offset to get real time
    date_time->minute = epoch%60;
    epoch /= 60;
    date_time->hour   = epoch%24;
    epoch /= 24;

    years = epoch/(365*4+1)*4;
    epoch %= 365*4+1;

    for (year=3; year>0; year--)
    {
        if (epoch >= days[year][0])
            break;
    }

    for (month=11; month>0; month--)
    {
        if (epoch >= days[year][month])
            break;
    }

    date_time->year  = years+year;
    date_time->month = month+1;
    date_time->day   = epoch-days[year][month]+1;
}

void ConvertEpoch2SysTime(date_time_t* local_date_time, unsigned long epoch_counter)
{
    unsigned long iridium_epoch_time_seconds;

    epoch_counter = (epoch_counter/100)*9;

    iridium_epoch_time_seconds = date_time_to_epoch(&ee_epoch_time);

    epoch_to_date_time(local_date_time , epoch_counter + iridium_epoch_time_seconds);
}

char UpdateEpoch(char* newEpoch)
{
    if (newEpoch[0] > 59)
        return FALSE;
    if (newEpoch[1] > 59)
        return FALSE;
    if (newEpoch[2] > 23)
        return FALSE;
    if ((newEpoch[3] < 1) || (newEpoch[3] > 31))
        return FALSE;
    if ((newEpoch[4] < 1) || (newEpoch[4] > 12))
        return FALSE;
    ee_epoch_time.second = newEpoch[0];   //seconds
    ee_epoch_time.minute = newEpoch[1];   //minutes
    ee_epoch_time.hour = newEpoch[2];   //hours
    ee_epoch_time.day = newEpoch[3];   //day
    ee_epoch_time.month = newEpoch[4];   //month
    ee_epoch_time.second = newEpoch[5];   //year
    #ifdef DebugMode
    SendDebugMsg("\r\nSet new epoch");
    PrintNum(ee_epoch_time.second);
    PrintNum(ee_epoch_time.minute);
    PrintNum(ee_epoch_time.hour);
    PrintNum(ee_epoch_time.day);
    PrintNum(ee_epoch_time.month);
    PrintNum(ee_epoch_time.year);
    #endif DebugMode
    return TRUE;
}

#endif Satellite