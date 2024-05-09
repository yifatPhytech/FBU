//smart_data_manager.c file (update 03-01-01)
///////// start of data manager module /////////
//#include "define.h"
#include "utils.h"
#include "Valve_Manager.h"
#include "data_manager.h"
#include "i2c_bus.h"
#include "eeprom.h"
#include "Rtc_Manager.h"

#define VCU_ID_START                40
#define VCU_MEMORY_START            240
#define CMPNT_MEMORY_START          20240   
#define PUMP_ACTION_MEMORY_START    40240    
#define ALERTS_MEMORY_START         60240
#define VCU_MEMORY_END            (CMPNT_MEMORY_START - VCU_PACKET_SIZE)
#define CMPNT_MEMORY_END          (PUMP_ACTION_MEMORY_START - CBU_MNT_DATA_SIZE)
#define PUMP_ACTION_MEMORY_END    (ALERTS_MEMORY_START - PUMP_ACTION_PACKET_SIZE) 
#define ALERTS_MEMORY_END         65500

extern eeprom _tagAPPEEPROM AppEepromData;
//extern DateTime g_curTime;
extern BYTE g_bHighPrio;
static _ExtEpromPointers  eprmPonter;
static unsigned int pOriginalReadBlock;
extern char e2_writeFlag;
extern char DataBlock[];
extern volatile unsigned char eepromReadBuf[MAX_DATA_2_EPRM_SIZE];   //SENSOR_CNTRL_PRM_SIZE];	//buffer for eeprom read operation
//extern unsigned int objToMsr;

BYTE IsAddressOnPageLimit(unsigned int iAddress, int iSize)
{
    unsigned int x = (iAddress + iSize) % 64;
    if ((x >= iSize) || (x == (unsigned int)0))
        return FALSE;
    return TRUE; 
}

/*//write 2 bytes data into ext_e2
unsigned int WriteIntoExte2(int intData, unsigned int intAddress, char sensorMeasur)
{
	char helpBuf[2];
                        
    if(sensorMeasur) //if it is sensor measur data to be writen into e2
        //move pWrite to next write position
        intAddress += 2;          
        
    //set data into 2 bytes
    int2bytes(intData, helpBuf);
      
    if (IsAddressOnPageLimit(intAddress, 2))  
    {                            
                //write data onto ext_e2
        if(!(e2_writePage(intAddress, 1, &helpBuf[0])))
            return 1;//if writing into ext_e2 faild exit faild
        if(!(e2_writePage(intAddress+1, 1, &helpBuf[1])))
            return 1;//if writing into ext_e2 faild exit faild
    }
    else
        //write data onto ext_e2
        if(!(e2_writePage(intAddress, 2, helpBuf)))
            return 1;//if writing into ext_e2 faild exit faild
	//else
	return intAddress;
}  */

//write 2 bytes data into ext_e2
BYTE WriteBufIntoExte2(char * data, unsigned int intAddress, int length)
{
	unsigned int y ,x;
                        
    if (IsAddressOnPageLimit(intAddress, length))  
    {                        
        x = (intAddress + length) % 64;  // num of bytes override 
        y = length - x;             // num of bytes not override
        //write data onto ext_e2
        if(!(e2_writePage(intAddress, y , data)))
            return FALSE;//if writing into ext_e2 faild exit faild
        if(!(e2_writePage(intAddress+y, x, &data[y])))
            return FALSE;//if writing into ext_e2 faild exit faild
    }
    else
        //write data onto ext_e2
        if(!(e2_writePage(intAddress, length, data)))
            return FALSE;//if writing into ext_e2 faild exit faild
	//else
	return TRUE;
}

#ifndef     NO_EZR
char LoadVCUIds()
{
	BYTE i, j = 0;//, n , more_data;
	unsigned int tmpRead;

    #ifdef DebugMode
    SendDebugMsg("\r\nLoadVCUIds");
    #endif DebugMode 
	tmpRead = VCU_ID_START;

	//disable other operation while run this function
    e2_writeFlag = 1;

    //read whole data block
	for(i = 0; i < 10; i++)           //PCKT_LNGTH / 7 = 3
	{
		//read ext e2 (from data block into read buf)
		if(!(e2_readSeqBytes(tmpRead, 20)))
		{
			e2_writeFlag = 0; 	// enable other use
			return FALSE;		//if reading fail, exit the program
		}
		//copy data from read buf into sensDataBlock
		for(j = 0; j < 5; j++)
		{
		    vlvCmdArr[(i*5) + j].VCU_ID = Bytes2ULong(&eepromReadBuf[j*4]);   
//            #ifdef DebugMode
//            SendDebugMsg("vlvCmdArr[");
//            PrintNum((i*5) + j);         
//            SendDebugMsg("] = ");    
//            PrintNum(vlvCmdArr[(i*5) + j].VCU_ID);    
//            #endif DebugMode 
		}
		tmpRead += 20; 			//move pointer 8 bytes
	}
         
    for (i = 0; i < MAX_CMD; i++)     
    {                  
        if ((vlvCmdArr[i].VCU_ID < 500000) || (vlvCmdArr[i].VCU_ID > 5000000))
            SaveVCUIdAtEeprom(0, i);       
    }
	e2_writeFlag = 0; 				// enable other use                  

	return TRUE;
}
#endif     NO_EZR

BYTE ReadPointers()
{
//    char tmpRead[POINTERS_SIZE];
    if(!(e2_readSeqBytes(0, POINTERS_SIZE)))
    {
        e2_writeFlag = 0; 	// enable other use
        return FALSE;		//if reading fail, exit the program
    }           
    memcpy((char*)&eprmPonter, eepromReadBuf, POINTERS_SIZE);
    #ifdef DebugMode
    SendDebugMsg("\r\nReadPointers");   
    SendDebugMsg("\r\npVCUDataWrite: ");
    PrintNum(eprmPonter.pVCUDataWrite);
    SendDebugMsg("\r\npVCUDataRead: ");
    PrintNum(eprmPonter.pVCUDataRead);  
    SendDebugMsg("\r\npVCUDataOverlap: ");
    PrintNum(eprmPonter.pVCUDataOverlap);      
    SendDebugMsg("\r\npCmpsDataWrite: ");
    PrintNum(eprmPonter.pCmpsDataWrite);
    SendDebugMsg("\r\npCmpsDataRead: ");
    PrintNum(eprmPonter.pCmpsDataRead);  
    SendDebugMsg("\r\npCmpsDataOverlap: ");
    PrintNum(eprmPonter.pCmpsDataOverlap);      
    SendDebugMsg("\r\npPmpActionWrite: ");
    PrintNum(eprmPonter.pPmpActionWrite);
    SendDebugMsg("\r\npPmpActionRead: ");
    PrintNum(eprmPonter.pPmpActionRead);
    SendDebugMsg("\r\npAlertWrite: ");
    PrintNum(eprmPonter.pAlertWrite);
    SendDebugMsg("\r\npAlertRead: ");
    PrintNum(eprmPonter.pAlertRead);
    #endif DebugMode  
    return TRUE;
}

char SavePointers()
{
/*    #ifdef DebugMode
    SendDebugMsg("\r\nSave Pointers");   
    SendDebugMsg("\r\npVCUDataWrite: ");
    PrintNum(eprmPonter.pVCUDataWrite);
    SendDebugMsg("\r\npVCUDataRead: ");
    PrintNum(eprmPonter.pVCUDataRead);
   
  SendDebugMsg("\r\npCmpsDataWrite: ");
    PrintNum(eprmPonter.pCmpsDataWrite);
    SendDebugMsg("\r\npCmpsDataRead: ");
    PrintNum(eprmPonter.pCmpsDataRead);
   SendDebugMsg("\r\npPmpActionWrite: ");
    PrintNum(eprmPonter.pPmpActionWrite);
    SendDebugMsg("\r\npPmpActionRead: ");
    PrintNum(eprmPonter.pPmpActionRead);
    SendDebugMsg("\r\npAlertWrite: ");
    PrintNum(eprmPonter.pAlertWrite);
    SendDebugMsg("\r\npAlertRead: ");
    PrintNum(eprmPonter.pAlertRead);
   
   #endif DebugMode */  
    return WriteBufIntoExte2((char *)&eprmPonter, 0, POINTERS_SIZE); 
}

char ResetPointers()
{
    #ifdef DebugMode
    SendDebugMsg("\r\nResetPointers");
    #endif DebugMode 
    eprmPonter.pVCUDataWrite = VCU_MEMORY_START; 
    eprmPonter.pVCUDataRead = VCU_MEMORY_START;   
    eprmPonter.pPmpActionWrite = PUMP_ACTION_MEMORY_START;
    eprmPonter.pPmpActionRead = PUMP_ACTION_MEMORY_START;
    eprmPonter.pCmpsDataWrite = CMPNT_MEMORY_START;
    eprmPonter.pCmpsDataRead = CMPNT_MEMORY_START; 
    eprmPonter.pAlertWrite = ALERTS_MEMORY_START;
    eprmPonter.pAlertRead = ALERTS_MEMORY_START;
    return SavePointers(); 
}

void SaveVCUIdAtEeprom(unsigned long id, BYTE idx)
{
    char temp[4]; 
    unsigned int    address;
    
    ULong2Bytes(id, &temp[0]); 
    address = 4 * idx;   
    address += VCU_ID_START;
    WriteBufIntoExte2(temp, address , 4); 
}

void InitVCUIdEeprom()
{
    BYTE i;
    memset(DataBlock, 0, VCU_PACKET_SIZE);      
    for (i = 0; i < 5; i++)
        WriteBufIntoExte2(DataBlock, VCU_ID_START+(40*i) , 40); 
}

//the function will copy one data block from ext_e2 (pBread address)
//into 'sensDataBlock' buffer in ram
//return 1 (succsess) or 0 (failure)
/*char CopyBlockIntoRam()
{
	BYTE i;//, j;//, n , more_data;
	unsigned int tmpRead;
             
    pOriginalReadBlock = eprmPonter.pVCUDataRead;
	tmpRead = eprmPonter.pVCUDataRead;
    #ifdef DebugMode
    SendDebugMsg("\r\nCopyBlockIntoRam ");
    SendDebugMsg("\r\npBread=  ");
    PrintNum(eprmPonter.pVCUDataRead);
    #endif DebugMode 
	//disable other operation while run this function
    e2_writeFlag = 1;

    //read whole data block
	for(i = 0; i < 2; i++)           //PCKT_LNGTH / 7 = 3
	{
		//read ext e2 (from data block into read buf)
		if(!(e2_readSeqBytes(tmpRead, 20)))
		{
			e2_writeFlag = 0; 	// enable other use
			return FALSE;		//if reading fail, exit the program
		}
		//copy data from read buf into sensDataBlock  
        memcpy(&DataBlock[i*20], &eepromReadBuf, 20); 
		tmpRead += 20; 			//move pointer 8 bytes
	}

	e2_writeFlag = 0; 				// enable other use                  
    eprmPonter.pVCUDataRead += VCU_PACKET_SIZE;
	return TRUE;
}*/

char CopyBlockIntoRam(unsigned int readPnt, BYTE size)
{
	unsigned int tmpRead = readPnt;

	//disable other operation while run this function
    e2_writeFlag = 1;

    //read whole data block
    //read ext e2 (from data block into read buf)
    if(!(e2_readSeqBytes(tmpRead, size)))
    {
        e2_writeFlag = 0; 	// enable other use
        return FALSE;		//if reading fail, exit the program
    }
    //copy data from read buf into sensDataBlock  
    memcpy(&DataBlock, &eepromReadBuf, size);
	
	e2_writeFlag = 0; 				// enable other use                  

	return TRUE;
}

#ifndef NO_EZR
/* prepare new whole packet:
    set the timestamp, than the first new data, and then reset all other 7 data-s to ff7f - default value.
    write all of this at once to the eeprom 
*/ 
char SaveVcuData(char* data, BYTE len, BYTE situation)
{
    char headerArr[VCU_PACKET_SIZE];   
    int noData = 0xff7f;   
    BYTE i, idx = 0;    
                               
    memset(&headerArr[idx], 0, VCU_PACKET_SIZE);     
  
    memcpy(&headerArr[idx], data, 4);      //id      
    idx += 4;                                           

    headerArr[idx++] = 0;                 //inner index
    headerArr[idx++] = 180;                 //type
    cpu_e2_to_MemCopy( &headerArr[idx], &AppEepromData.eLoggerID[0], 4);    // CCU id
    idx += 4;
    memcpy(&headerArr[idx], &g_curTime, 5);   // timstamp    
    idx += 5;
    headerArr[idx++] = situation;              // msg type
    memcpy(&headerArr[idx], &data[6], len-6);  // rssi, btr, vlv stat, cmd index, data
    idx += (len - 6);            
         
    // add another 7 data
    for (i = 0; i < ((24 - len) / 2); i++)
    {
        int2bytes(noData, &headerArr[idx]);
        idx += 2;  
    }  

    if (WriteBufIntoExte2(headerArr, eprmPonter.pVCUDataWrite, VCU_PACKET_SIZE) == FALSE)
        return FALSE;
    eprmPonter.pVCUDataWrite += VCU_PACKET_SIZE;    
    if (eprmPonter.pVCUDataWrite > VCU_MEMORY_END)  
    {
        eprmPonter.pVCUDataWrite = VCU_MEMORY_START;        
        eprmPonter.pVCUDataOverlap = 1;
    }
//    g_bHighPrio = TRUE;
    return TRUE;    //SavePointers();  
}
#endif NO_EZR

BYTE GetNextEprom2Send(BYTE prevPost)
{
    if (prevPost <= POST_ALERT) 
        if (eprmPonter.pAlertRead != eprmPonter.pAlertWrite)
        {             
            pOriginalReadBlock = eprmPonter.pAlertRead;
            return POST_ALERT;                       
        }  
    if (prevPost <= POST_CBU_DATA)
        if (eprmPonter.pCmpsDataRead != eprmPonter.pCmpsDataWrite)
        {             
            pOriginalReadBlock = eprmPonter.pCmpsDataRead;
            return POST_CBU_DATA;  
        }    
    if (prevPost <= POST_VCU_DATA)                 
        if (eprmPonter.pVCUDataRead != eprmPonter.pVCUDataWrite)
        {             
            pOriginalReadBlock = eprmPonter.pVCUDataRead;
            return POST_VCU_DATA;     
        }                          
    if (prevPost <= POST_PUMP_ACT) 
        if (eprmPonter.pPmpActionRead != eprmPonter.pPmpActionWrite)
        {             
            pOriginalReadBlock = eprmPonter.pPmpActionRead;
            return POST_PUMP_ACT;   
        }                   
    return POST_GET_CMD;
}


int GetEpromPcktCnt(BYTE nType)
{
     switch (nType)
    {       
        case POST_ALERT:       
            return (eprmPonter.pAlertWrite - eprmPonter.pAlertRead) / ALERTS_MEMORY_PACKET_SIZE;
        break;
        case POST_CBU_DATA:   
//            #ifdef DebugMode
//            SendDebugMsg("\r\npCmpsDataWrite= \0");   
//            PrintNum(eprmPonter.pCmpsDataWrite);
//            SendDebugMsg("\r\npCmpsDataRead= \0");   
//            PrintNum(eprmPonter.pCmpsDataRead);            
//            #endif DebugMode     
            if (eprmPonter.pCmpsDataOverlap == 0)
                return (eprmPonter.pCmpsDataWrite - eprmPonter.pCmpsDataRead) / CBU_MNT_DATA_SIZE;
            else        
                return (CMPNT_MEMORY_END - eprmPonter.pVCUDataRead) / VCU_PACKET_SIZE;
        break;
        case POST_VCU_DATA:              
            if (eprmPonter.pVCUDataOverlap == 0)
                return (eprmPonter.pVCUDataWrite - eprmPonter.pVCUDataRead) / VCU_PACKET_SIZE;     
            else        
                return (CMPNT_MEMORY_END - eprmPonter.pVCUDataRead) / VCU_PACKET_SIZE;                
        break;
        case POST_PUMP_ACT:
            return (eprmPonter.pPmpActionWrite - eprmPonter.pPmpActionRead) / PUMP_ACTION_PACKET_SIZE;
        break;     
        default:
            return 1;
    }
}

#ifndef NO_EZR
char SavePumpActionData(int action, char res, unsigned int cmdIdx)
{
    char headerArr[PUMP_ACTION_PACKET_SIZE];   
    BYTE idx = 0, i;     
    int noData = 0xff7f;   
                               
    memset(&headerArr[idx], 0, PUMP_ACTION_PACKET_SIZE);     
  
    cpu_e2_to_MemCopy(&headerArr[idx], &AppEepromData.eCbuGlblData[0], 4);      //id      
    idx += 4;
    headerArr[idx++] = 0;                 //inner index
    headerArr[idx++] = 204;                 //type
    cpu_e2_to_MemCopy( &headerArr[idx], &AppEepromData.eLoggerID[0], 4);    // CCU id
    idx += 4;
    memcpy(&headerArr[idx], &g_curTime, 5);   // timstamp    
    idx += 5;
    headerArr[idx++] = action;              // msg type         
    headerArr[idx++] = 0;              // rssi 
    headerArr[idx++] = 0;              // btr 
    headerArr[idx++] = 0;              // btr 
    headerArr[idx++] = res;               //g_pumpStat[0];              // pump stat  
    memcpy( &headerArr[idx], &cmdIdx, 2);   // cmd index     
    idx += 2;
    for (i = 0; i < 6; i++)
    {
        int2bytes(noData, &headerArr[idx]);
        idx += 2;  
    }  

    if (WriteBufIntoExte2(headerArr, eprmPonter.pPmpActionWrite, PUMP_ACTION_PACKET_SIZE) == FALSE)
        return FALSE;
    eprmPonter.pPmpActionWrite += PUMP_ACTION_PACKET_SIZE;   
    if (eprmPonter.pPmpActionWrite > PUMP_ACTION_MEMORY_END)
        eprmPonter.pPmpActionWrite = PUMP_ACTION_MEMORY_START; 
    
    g_bHighPrio = TRUE;
    return TRUE;    //SavePointers();
}
#endif NO_EZR

char SaveCBUPortData(char * data)
{
    char headerArr[CBU_MNT_DATA_SIZE];   
    BYTE idx = 0;    
    
//    ReadPointers(); 
    headerArr[idx++] = g_curTime.year;     
	headerArr[idx++] = g_curTime.month;    
	headerArr[idx++] = g_curTime.day;      
	headerArr[idx++] = g_curTime.hour;                                      
    headerArr[idx++] = g_curTime.minute; 
    memcpy(&headerArr[idx], data, CBU_MNT_DATA_IN_SIZE);    
         
    if (WriteBufIntoExte2(headerArr, eprmPonter.pCmpsDataWrite, CBU_MNT_DATA_SIZE) == FALSE)    
    {
        return FALSE;
    }              
    eprmPonter.pCmpsDataWrite += CBU_MNT_DATA_SIZE; 
    if (eprmPonter.pCmpsDataWrite > CMPNT_MEMORY_END)
    {
        eprmPonter.pCmpsDataWrite = CMPNT_MEMORY_START;   
        eprmPonter.pCmpsDataOverlap = 1;
    }
//    #ifdef DebugMode
//    SendDebugMsg("\r\npCmpsDataWrite \0");
//    PrintNum(eprmPonter.pCmpsDataWrite);
//    #endif DebugMode 
//    g_bHighPrio = TRUE;
    return TRUE;    //SavePointers();
}

char SaveAlertData(char * alert)
{
    char headerArr[ALERTS_MEMORY_PACKET_SIZE];   
    BYTE idx = 0;    
    
    memset(&headerArr[idx], 0, ALERTS_MEMORY_PACKET_SIZE);     
//    ReadPointers(); 
    headerArr[idx++] = g_curTime.year;     
	headerArr[idx++] = g_curTime.month;    
	headerArr[idx++] = g_curTime.day;      
	headerArr[idx++] = g_curTime.hour;                                      
    headerArr[idx++] = g_curTime.minute;   
//    headerArr[idx++] = alert;
//    headerArr[idx++] = 0;
    memcpy(&headerArr[idx], alert, 2);
    idx += 2;
                                   
    if (WriteBufIntoExte2(headerArr, eprmPonter.pAlertWrite, ALERTS_MEMORY_PACKET_SIZE) == FALSE)    
    {
        return FALSE;
    }              
    eprmPonter.pAlertWrite += ALERTS_MEMORY_PACKET_SIZE;    
    if (eprmPonter.pAlertWrite > ALERTS_MEMORY_END)
        eprmPonter.pAlertWrite = ALERTS_MEMORY_START;
//    g_bHighPrio = TRUE;
    return TRUE;    //SavePointers();
}

/*BYTE ReadVCUPacket()
{
//    BYTE i;
    if (eprmPonter.pVCUDataWrite == eprmPonter.pVCUDataRead)
        return NO_DATA;
	if(CopyBlockIntoRam() == FALSE) //copy data block at the pBread address
    {
		return FALSE;
    }
    // if the block is the last one to send - keep the inside address  
    if (eprmPonter.pVCUDataWrite == eprmPonter.pVCUDataRead)
        eprmPonter.pVCUDataWrite = eprmPonter.pVCUDataRead = VCU_MEMORY_START;    

	//save control parameters into ext_e2
	if (SavePointers() == FALSE)
    {
		return FALSE;
    }

	return TRUE;
}   */

char ResetReadPointer(BYTE nPacketType)
{
	//read control parameters from cpu_e2 into ram
	//arrange the pointers and parameters for manipulation
//	if (ReadPointers() != TRUE)
//		return FALSE;   
        
    #ifdef DebugMode
    SendDebugMsg("\r\nReset Read Pointer");
    #endif DebugMode 
    switch (nPacketType)
    {       
        case POST_PUMP_ACT:
            eprmPonter.pPmpActionRead = pOriginalReadBlock;  
        break;
        case POST_CBU_DATA:
            eprmPonter.pCmpsDataRead = pOriginalReadBlock; 
        break;
        case POST_ALERT:
            eprmPonter.pAlertRead = pOriginalReadBlock; 
        break;   
        case POST_VCU_DATA:      
            eprmPonter.pVCUDataRead = pOriginalReadBlock;    
        default:
            return FALSE;
    }
    
//    return SavePointers();
}

void InitWritPntr(BYTE nPacketType)
{
    switch (nPacketType)
    {       
        case POST_PUMP_ACT:
            if (eprmPonter.pPmpActionRead == eprmPonter.pPmpActionWrite)
            {              
                eprmPonter.pPmpActionRead = PUMP_ACTION_MEMORY_START;
                eprmPonter.pPmpActionWrite = PUMP_ACTION_MEMORY_START;           
            }
        break;
        case POST_CBU_DATA:     
            if (eprmPonter.pCmpsDataRead == eprmPonter.pCmpsDataWrite)
            {              
                eprmPonter.pCmpsDataRead = CMPNT_MEMORY_START;            
                eprmPonter.pCmpsDataWrite = CMPNT_MEMORY_START;        
                eprmPonter.pCmpsDataOverlap = 0;
            }       
        break;
        case POST_ALERT:
            if (eprmPonter.pAlertRead == eprmPonter.pAlertWrite)
            {              
                eprmPonter.pAlertRead = ALERTS_MEMORY_START;
                eprmPonter.pAlertWrite = ALERTS_MEMORY_START;
            }
        break;     
        case POST_VCU_DATA: 
            if (eprmPonter.pVCUDataRead == eprmPonter.pVCUDataWrite)
            {              
                eprmPonter.pVCUDataRead = VCU_MEMORY_START;
                eprmPonter.pVCUDataWrite = VCU_MEMORY_START;   
                eprmPonter.pVCUDataOverlap = 0;
            }       
        break;     
        default:
            ;
    }
}

BYTE ReadPacket(BYTE nPacketType)
{
    unsigned int pRead; 
    BYTE nSize;
        
    switch (nPacketType)
    {       
        case POST_PUMP_ACT:
            pRead = eprmPonter.pPmpActionRead;  
            nSize = PUMP_ACTION_PACKET_SIZE;
        break;
        case POST_CBU_DATA:
            pRead = eprmPonter.pCmpsDataRead; 
            nSize = CBU_MNT_DATA_SIZE;
        break;
        case POST_ALERT:
            pRead = eprmPonter.pAlertRead; 
            nSize = ALERTS_MEMORY_PACKET_SIZE;
        break;   
        case POST_VCU_DATA:      
            pRead = eprmPonter.pVCUDataRead;    
            nSize = VCU_PACKET_SIZE;  
        break;          
        default:
            return FALSE;
    }
           
    #ifdef DebugMode
    SendDebugMsg("\r\nReadPacket of type ");
    PrintNum(nPacketType);
    SendDebugMsg("\r\npRead=  ");
    PrintNum(pRead);
    #endif DebugMode 
	if(CopyBlockIntoRam(pRead, nSize ) == FALSE) //copy data block at the pBread address
    {
		return FALSE;
    }
    
    switch (nPacketType)
    {       
        case POST_PUMP_ACT:
            eprmPonter.pPmpActionRead += PUMP_ACTION_PACKET_SIZE;  
            if (eprmPonter.pPmpActionRead >= PUMP_ACTION_MEMORY_END)
                eprmPonter.pPmpActionRead = PUMP_ACTION_MEMORY_START;      
        break;
        case POST_CBU_DATA:
            eprmPonter.pCmpsDataRead += CBU_MNT_DATA_SIZE;   
            if (eprmPonter.pCmpsDataRead >= CMPNT_MEMORY_END)  
            {
                eprmPonter.pCmpsDataRead = CMPNT_MEMORY_START;   
                eprmPonter.pCmpsDataOverlap = 0;                  
            }     
        break;
        case POST_ALERT:
            eprmPonter.pAlertRead += ALERTS_MEMORY_PACKET_SIZE;  
            if (eprmPonter.pAlertRead >= ALERTS_MEMORY_END)
                eprmPonter.pAlertRead = ALERTS_MEMORY_START;      
        break;     
        case POST_VCU_DATA:     
            eprmPonter.pVCUDataRead += VCU_PACKET_SIZE; 
            if (eprmPonter.pVCUDataRead > VCU_MEMORY_END) 
            { 
                eprmPonter.pVCUDataRead = VCU_MEMORY_START;  
                eprmPonter.pVCUDataOverlap = 0;
            } 
            
        break;     
        default:
            return FALSE;
    }
    // if the block is the last one to send - keep the inside address        
//    if (pCBwrite == pCBread)
//        pCBwrite = pCBread = CMPNT_MEMORY_START;

	//save control parameters into ext_e2
//	if (SavePointers() == FALSE)
//    {
//		return FALSE;
//    }

	return TRUE;
}

/*
 VCU_MEMORY_START           240
 CMPNT_MEMORY_START         20240      
 PUMP_ACTION_MEMORY_START   40240   
 ALERTS_MEMORY_START        60240

*/
BYTE ValidatePointers()      
{
    if ((eprmPonter.pVCUDataWrite < VCU_MEMORY_START) || (eprmPonter.pVCUDataWrite >= VCU_MEMORY_END)) 
        return FALSE;
    if ((eprmPonter.pVCUDataRead < VCU_MEMORY_START) || (eprmPonter.pVCUDataRead >= VCU_MEMORY_END)) 
        return FALSE;
    if ((eprmPonter.pPmpActionWrite < PUMP_ACTION_MEMORY_START) || (eprmPonter.pPmpActionWrite >= PUMP_ACTION_MEMORY_END)) 
        return FALSE;
    if ((eprmPonter.pPmpActionRead < PUMP_ACTION_MEMORY_START) || (eprmPonter.pPmpActionRead >= PUMP_ACTION_MEMORY_END)) 
        return FALSE;
    if ((eprmPonter.pCmpsDataWrite < CMPNT_MEMORY_START) || (eprmPonter.pCmpsDataWrite >= CMPNT_MEMORY_END)) 
        return FALSE;
    if ((eprmPonter.pCmpsDataRead < CMPNT_MEMORY_START) || (eprmPonter.pCmpsDataRead >= CMPNT_MEMORY_END)) 
        return FALSE;
    if ((eprmPonter.pAlertWrite < ALERTS_MEMORY_START) || (eprmPonter.pAlertWrite >= ALERTS_MEMORY_END)) 
        return FALSE;
    if ((eprmPonter.pAlertRead < ALERTS_MEMORY_START) || (eprmPonter.pAlertRead >= ALERTS_MEMORY_END)) 
        return FALSE;
    return TRUE;
}

///////// end of data manager module ///////////
////////////////////////////////////////////////

