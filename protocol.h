#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "protocolDefs.h"

#define PROTOCOL_RECV_MESSAGE_TIMEOUT	500	// [msec]

void PROTOCOL_Init(void);
void PROTOCOL_DeInit(void);
BYTE PROTOCOL_Task(EProtocolCommand cmd, BYTE bIsMore);
long GetEzrVer();
void ResetEzrVerNum();
#endif