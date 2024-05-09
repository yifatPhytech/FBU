#ifndef __RS485_MNG_H
#define __RS485_MNG_H

#include <string.h>
#include "define.h"

typedef enum _PumpCmd
{
    PUMP_CMD_OFF, 
    PUMP_CMD_ON,
}PumpCmd;

typedef enum _RS485Command
{
    CMD_GET_MNT_DATA ,     //0
    CMD_SET_FACTOR_DEF,    //1   
    CMD_GET_FACTOR_DEF,    //2   
    CMD_SET_CBU_DEF,       //3
    CMD_GET_CBU_DEF,       
    CMD_RST_CCU,           //5      
    CMD_RST_CBU,                   
    CMD_ALERT,                   
    CMD_FW_UPGRADE,        //8
    CMD_RST_FLOW,
    CMD_WU,  
    CMD_GET_FOTA_FAIL_RSN,
    CMD_LED,               //12
    CMD_Count
} ERS485Command;

extern char g_bCBUPumpState[2];     

#define ENABLE_CBU_INT()   PCICR=(0<<PCIE3) | (0<<PCIE2) | (1<<PCIE1) | (0<<PCIE0); // PCICR=(1<<PCIE3) | (0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);
#define DISABLE_CBU_INT()   (PCICR=(0<<PCIE3) | (0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0));
#define RS485_CTRL_TX() (PORTB.3 = 1);
#define RS485_CTRL_RX() (PORTB.3 = 0);        //##

#define CBU_GET_GLBL_CNFG_LEN   15  //ID (4) version (4), dry_contact_1, dry_contact_2, Safety, pump1 mode, pump1 mode, battery threshold (2)
#define CBU_SET_GLBL_CNFG_LEN   7  // dry_contact_1, dry_contact_2, Safety, pump1 mode, pump2 mode, battery threshold (2)
#define MAX_PORTS_CBU   5 

char SendRecRS485(ERS485Command cmd, int prm);

void RecSendRS485();

void InitCom(unsigned char bSend);

extern int g_nFailureCnt;

//extern char g_nCbuVer;

#endif __RS485_MNG_H
