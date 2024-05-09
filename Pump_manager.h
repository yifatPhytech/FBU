#ifndef PUMP_MANAGER_H
#define PUMP_MANAGER_H
#include "Valve_Manager.h"

//typedef struct _PumpUnit
//{
//    CommandData     cmdData; 
//    BYTE            IsCmd4Pump;    
//    DateTime        stopTimeStamp;     
//    CommandData*    nextIrg;
//}PumpUnit;

extern BYTE g_bMngPumpNow;

//BYTE SetPumpCmd(unsigned int dur, BYTE offTime, BYTE cycles, BYTE startHour, BYTE startMin, unsigned int cmdIndex, BYTE pumpIdx);

//BYTE CheckPumpStatus();

DateTime GetTimeAfterDelay(int delay);

//void InitPumpCmdUnit();

//BYTE IsPumpCmd();

unsigned int GetPumpCmdIdx();

void CheckPumpFitStatus();

#endif PUMP_MANAGER_H
