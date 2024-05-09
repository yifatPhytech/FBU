//smart_data_manager.c file (update 03-01-01)
///////// start of data manager module /////////
#include "define.h"
#include "Valve_Manager.h"
extern eeprom _tagAPPEEPROM AppEepromData;
extern DateTime g_curTime;
//extern long SenIDArr[];
unsigned int pBwrite;		//pointer to current write sensor data block in ext_e2
unsigned int pBread;		//pointer to last read sensor data block in ext_e2
unsigned int pWrite;		//pointer to last sensor data write in ext_e2
unsigned int pRead;		//pointer to sensor's last read data in ext_e2
unsigned int pStart_space;	//pointer to start sensor data space in ext_e2
unsigned int pEnd_space;	//pointer to end sensor data space in ext_e2
unsigned int pSens_ext_e2;	//pointer to current sensor control parameters in e
unsigned int pOriginalReadBlock;
extern char e2_writeFlag;
extern int nUnreadBlocks;
extern char DataBlock[];
extern char SenParams[];
extern volatile unsigned char eepromReadBuf[SENSOR_CNTRL_PRM_SIZE];	//buffer for eeprom read operation
extern unsigned int objToMsr;
static int prevMsrHour;
static int msrHour;
extern BYTE unitType;
extern BYTE eepromSize;
#ifdef Satellite
extern unsigned int arrReadPointer[];
extern unsigned char gUnreadValues;
#endif Satellite


void CheckSizeOfEeprom()
{
    char origFirst,origMid, c;       
         
    //save original data
    if (e2_readSeqBytes(0x00, 1) == FAILURE)
        return;
    origFirst = eepromReadBuf[0];
    if (e2_readSeqBytes(0x8000, 1) == FAILURE)
        return;
    origMid = eepromReadBuf[0];
    
    // write special values     
    c = 0;   
    if (e2_writePage(0x00, 1, &c) == FAILURE)
        return;
    c = 0x81;
    if (e2_writePage(0x8000, 1, &c) == FAILURE)
        return;   
    c = 0xFF;
    // read value and check if saved in right place
    e2_readSeqBytes(0x00, 1);
    c = eepromReadBuf[0];  
    
    if (c == 0x00)   
    {
        eepromSize = EXT_EEPROM_64K;   
        // write back original values     
        e2_writePage(0x8000, 1, &origMid);  
        AppEepromData.nMaxSenNum = MAX_WL_SEN_NUM_64K;
        #ifdef DebugMode
        SendDebugMsg("\r\nsize of eeprom is 64K\0");
        #endif DebugMode
    }
    else  
    {
        eepromSize = EXT_EEPROM_32K;   
        AppEepromData.nMaxSenNum = MAX_WL_SEN_NUM_32K;
        #ifdef DebugMode
        SendDebugMsg("\r\nsize of eeprom is 32K\0");
        #endif DebugMode
    }
        
    // write back original values     
    e2_writePage(0x00, 1, &origFirst);  
    #ifdef DebugMode
    SendDebugMsg("\r\nnum sensors: \0");   
    PrintNum(AppEepromData.nMaxSenNum-1);
    #endif DebugMode
}


BYTE IsAddressOnPageLimit(unsigned int iAddress, int iSize)
{
    unsigned int x = (iAddress + iSize) % 64;
    if ((x >= iSize) || (x == (unsigned int)0))
        return FALSE;
    return TRUE; 
}

//write 2 bytes data into ext_e2
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
}

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

/* prepare new whole packet:
    set the timestamp, than the first new data, and then reset all other 7 data-s to ff7f - default value.
    write all of this at once to the eeprom 
*/ 
char WriteFirstData(DateTime dt, int data/*, BYTE type, BYTE bCheckType*/)
{
    char headerArr[PCKT_LNGTH];   
    int i, resetData = 0xff7f;       
    
    headerArr[0] = dt.year;     
	headerArr[1] = dt.month;    
	headerArr[2] = dt.day;      
	headerArr[3] = dt.hour;                                      
    headerArr[4] = dt.minute;      
         
    int2bytes(data, &headerArr[5]); 
    for(i = 1; i < MAX_DATA_PER_PCKT; i++) //write into temp data block (2 bytes x 8 = 16 bytes) 
        int2bytes(resetData, &headerArr[5 + (i*2)]); 
          
    return (WriteBufIntoExte2(headerArr, pBwrite, PCKT_LNGTH));
}

//the function will set address into 'pSens_ext_e2' and 'pSens_cpu_e2'
// (int global variables define in data manager module)
void SetCotrolParamAddress()
{
    // POINT TO 16 BYTES BEFORE END OF DATA BLOCK
    pSens_ext_e2 = SENSOR_MEMORY_START + ((objToMsr + 1) * SENSOR_MEMORY_SIZE) - SENSOR_CNTRL_PRM_SIZE;
}

//set next pBread position
//end of reading space is the first block (including first block)
//return 1 if there is new pBread address or 0 if not
char NextpReadAddress()
{
	//if read block + 48 > data end space
	if((pBread + PCKT_LNGTH) > pEnd_space)
 	{
		pBread = pStart_space;
		return TRUE;
	}

	pBread += PCKT_LNGTH;
	return TRUE;
}

//the function will copy one data block from ext_e2 (pBread address)
//into 'sensDataBlock' buffer in ram
//return 1 (succsess) or 0 (failure)
char CopyBlockIntoRam()
{
	BYTE i, j;//, n , more_data;
	unsigned int tmpRead;

	tmpRead = pBread;

	//disable other operation while run this function
    e2_writeFlag = 1;

    //read whole data block
	for(i = 0; i < 3; i++)           //PCKT_LNGTH / 7 = 3
	{
		//read ext e2 (from data block into read buf)
		if(!(e2_readSeqBytes(tmpRead, 7)))
		{
			e2_writeFlag = 0; 	// enable other use
			return FALSE;		//if reading fail, exit the program
		}
		//copy data from read buf into sensDataBlock
		for(j = 0; j < 7; j++)
		{
		    DataBlock[j +(i*7)] = eepromReadBuf[j];
		}
		tmpRead += 7; 			//move pointer 8 bytes
	}

	e2_writeFlag = 0; 				// enable other use                  
    #ifdef Satellite
    //calculate num of real data (not dummy 0xff7f) in current read block
    n = 0;
    do
    {
        if ((DataBlock[PACKET_HEADER_SIZE + (n * 2)] == 0x7f) && (DataBlock[PACKET_HEADER_SIZE + 1 + (n * 2)] == 0xff))
            break;
        n++;
    }
    while (n < MAX_DATA_PER_PCKT);

    gUnreadValues = n;       
    #endif Satellite
	return TRUE;
}

BYTE IsNextHour()
{
    BYTE curHour = (int)g_curTime.hour;//readClockBuf[4];
    if ((prevMsrHour+1) == curHour)
        return TRUE;
    else
        if ((prevMsrHour == 23) && (curHour == 0))
            return TRUE;      

    return FALSE;
}

char PointersInRange()
{
    if ((pBwrite < pStart_space) || (pBwrite > pEnd_space))
    {
        #ifdef DebugMode
        SendDebugMsg("\r\nPointers Not in Range ");
        SendDebugMsg("\r\npBwrite ");
        PrintNum(pBwrite);
        SendDebugMsg("\r\npStart_space ");
        PrintNum(pStart_space);
        SendDebugMsg("\r\npEnd_space ");
        PrintNum(pEnd_space);
        #endif DebugMode
        return FALSE;
    }
    if ((pWrite < pStart_space) || (pWrite > pEnd_space))
    {
        #ifdef DebugMode
        SendDebugMsg("\r\nPointers Not in Range ");
        SendDebugMsg("\r\npWrite ");
        PrintNum(pWrite);
        SendDebugMsg("\r\npStart_space ");
        PrintNum(pStart_space);
        SendDebugMsg("\r\npEnd_space ");
        PrintNum(pEnd_space);
        #endif DebugMode
        return FALSE;
    }
    if ((pBread < pStart_space) || (pBread > pEnd_space))
    {
        #ifdef DebugMode
        SendDebugMsg("\r\nPointers Not in Range ");
        SendDebugMsg("\r\npBread ");
        PrintNum(pBread);
        SendDebugMsg("\r\npStart_space ");
        PrintNum(pStart_space);
        SendDebugMsg("\r\npEnd_space ");
        PrintNum(pEnd_space);
        #endif DebugMode
        return FALSE;
    }
    if ((pRead < pStart_space) || (pRead > pEnd_space))
    {
        #ifdef DebugMode
        SendDebugMsg("\r\nPointers Not in Range ");
        SendDebugMsg("\r\npRead ");
        PrintNum(pRead);
        SendDebugMsg("\r\npStart_space ");
        PrintNum(pStart_space);
        SendDebugMsg("\r\npEnd_space ");
        PrintNum(pEnd_space);
        #endif DebugMode
        return FALSE;
    }
    return TRUE;
}

char ReadExte2ToRam(char checkID)
{
    BYTE bRes, n = 0;
    long l_IdInEprm, l_IdInMem;            
    
    if (objToMsr >= AppEepromData.nMaxSenNum)
        return FALSE;

    //read control parameters from cpu_e2 into ram
    SetCotrolParamAddress();
	//read the sensor control parameters from ext_e2 into the ram read_buf
    do
    {
	    bRes = e2_readSeqBytes(pSens_ext_e2, CONTROL_PARAM_LENGTH);
        n++;
        
        delay_ms(10);//50
    }
    while ((bRes == FAILURE) && (n < 3));

    if (bRes == FAILURE)
    {
        #ifdef DebugMode
        SendDebugMsg("\r\nFailed e2_readSeqBytes. reset pointers ");
        #endif DebugMode
        if (ResetPointers(/*objToMsr,*/ FALSE) == TRUE)
            return RESET_POINTERS;      //FALSE;
        else
            return FALSE;
    }

	//sets pointer of current writing block
	pBwrite = bytes2int(eepromReadBuf);
	//set pointer of current writing index
	pWrite = bytes2int(eepromReadBuf+2);
	//sets pointer of current reading block
	pBread = bytes2int(eepromReadBuf+4);
	//sets pointer of current reading index
	pRead = bytes2int(eepromReadBuf+6);
	//set pointer to start data space
    pStart_space = SENSOR_MEMORY_START + (objToMsr * SENSOR_MEMORY_SIZE);
	//pStart_space = bytes2int(eepromReadBuf+8);
	//set pointer to end data space
    pEnd_space = pStart_space + SENSOR_MEMORY_SIZE - SENSOR_CNTRL_PRM_SIZE - 1;
	//pEnd_space = bytes2int(eepromReadBuf+10);
    // set sensor ID
    l_IdInEprm = Bytes2Long(eepromReadBuf+8);
	//set last cycle time into variable
//	last_cycle_min = bytes2int(&eepromReadBuf[12]);        
    prevMsrHour =  bytes2int(&eepromReadBuf[12]);   
    unitType = eepromReadBuf[14];
//    prevMsr =  bytes2int(&eepromReadBuf[14]);
             
    if (PointersInRange() == FALSE)
    {
        if (ResetPointers(/*objToMsr,*/ FALSE) == TRUE)
            return RESET_POINTERS;      //FALSE;
        else
            return FALSE;      
    }
    if (checkID)     
    {       
        if (objToMsr < MAX_CMD)
            l_IdInMem = GetCmdID(objToMsr);   
//        else                                                         
//            l_IdInMem = SenIDArr[objToMsr-MAX_CMD];
            
        if (l_IdInEprm != l_IdInMem)
        {
            #ifdef DebugMode
            SendDebugMsg("\r\nID of sensor in mem is ");
            PrintNum(l_IdInMem);
            SendDebugMsg("\r\nID in eeprom is ");
            PrintNum(l_IdInEprm);
            #endif DebugMode       
//            SetCmdID(objToMsr, l); //SenIDArr[objToMsr] = l;
            return MISMATCH_ID;
        }              
    }
    return TRUE;
}

char ReadSensorParams()
{
    BYTE bRes, n = 0, i;

	//read the sensor control parameters from ext_e2 into the ram read_buf
    do
    {
	    bRes = e2_readSeqBytes(pSens_ext_e2 + SENSOR_PARAM_INDEX, SENSOR_PARAM_LENGTH);
        n++;        
        delay_ms(50);
    }
    while ((bRes == FAILURE) && (n < 3));

    if (bRes == FAILURE)
    {
        #ifdef DebugMode
        SendDebugMsg("\r\nFailed ReadSensorParams. reset pointers ");
        #endif DebugMode
        return FALSE;
    }

	//sets pointer of current writing block
    for (i = 0; i < SENSOR_PARAM_LENGTH; i++)
        SenParams[i] = eepromReadBuf[i];
    return TRUE;
}

//open new data block
//set the write block pointer to the new place
char CreateNewDataBlock()
{
    	//if write block + 48 = data space end
	if ((pBwrite + PCKT_LNGTH) > pEnd_space)  
    // if couldnt find aother free space in memory 
            // start from begining of buffer
            pBwrite = pStart_space;   
    else
        pBwrite += PCKT_LNGTH;

    // if after increasing pBwrite it reaches pBread - means the pBwrite completed a whole writing cycle to the buffer without reading.
    //  increase the pBread pointer by one packet
    if (pBwrite == pBread)
        NextpReadAddress();

	//set write pointer to block first data
	//pWrite points 2 step before the next writing point
	pWrite = pBwrite + PACKET_HEADER_SIZE - 2;

	return TRUE;
}

char SaveControlParam()
{
    char temp[CONTROL_PARAM_LENGTH];

	//~~~~~~~~~~~~~~~~~~~~~~~~~~    
    int2bytes(pBwrite, &temp[0]);  
    int2bytes(pWrite, &temp[2]);  
    int2bytes(pBread, &temp[4]);  
    int2bytes(pRead, &temp[6]);        
    if (objToMsr < MAX_CMD)
        Long2Bytes(GetCmdID(objToMsr)/*SenIDArr[objToMsr]*/, &temp[8]);
//    else
//        Long2Bytes(SenIDArr[objToMsr-MAX_CMD], &temp[8]);  
    int2bytes(msrHour, &temp[12]);  
    temp[14] = unitType;
    temp[15] = 0;   
    return WriteBufIntoExte2(temp, pSens_ext_e2, CONTROL_PARAM_LENGTH); 
}

char SaveSensorParams(int volt, BYTE type, BYTE rssi)
{                                                        
    char temp[4];
    unsigned int p = pSens_ext_e2 + SENSOR_PARAM_INDEX; //index of params after gps
        
    // save last battery
    int2bytes(volt , &temp[0]);  
    temp[2] = type;
    temp[3] = rssi;
    return (WriteBufIntoExte2(temp, p, 4));
}

//save sensors measurments results in the external eeprom
//the function return (1 = success) or (0 = failue)
//char SaveMeasurments(int data, char *fData, char cType)
char SaveMeasurments(BYTE nType, int nMsr,  unsigned int nBtr, BYTE nRssi)
{
	unsigned int temp_address;//, data;      
    char bFirstData = FALSE;   
    ValveCmd vlv;      
    BYTE res = TRUE; 
         
    #ifdef DebugMode
    SendDebugMsg("\r\nSaveMeasurme");
    SendDebugMsg("\r\nobjToMsr= ");
    PrintNum(objToMsr);
    #endif DebugMode
    if (nType == TYPE_VALVE)
    {
        vlv  = GetCmd(objToMsr); 
        if (vlv.lSenID == 0UL)
            return FALSE;  
    }                
//    else                
//    {
//        if (SenIDArr[objToMsr-MAX_CMD] == (unsigned long)0)
//            return FALSE;
//    }
    
//    #ifdef DebugMode
//    SendDebugMsg("\r\nSenIDArr[objToMsr]=");
//    PrintNum(SenIDArr[objToMsr-MAX_CMD]);  
//    PrintNum(nMsr);
//    #endif DebugMode                    
   
	//read control parameters from cpu_e2 into ram
	//arrange the pointers and parameters for manipulation
	if(ReadExte2ToRam(TRUE) == FALSE)
    {
        #ifdef DebugMode
        SendDebugMsg("\r\nSaveMeasurme failed1 ");
        #endif DebugMode
		return FALSE;
    }  
    //check if it is the first writing data cycle
    //if((pBfirst == pBwrite)&&(pWrite == pBwrite + 6)) //for the 24-04-01 version
    if(pWrite == (pBwrite + PACKET_HEADER_SIZE - 2))
    {          
        bFirstData = TRUE;        
    }
	else //it is not the first data writing cycle
	{            
        //check if its not next sequence measure or write block is full     
        if((IsNextHour() == FALSE) || ((pWrite + 2) >= (pBwrite + PCKT_LNGTH)) || (nType == TYPE_VALVE))// && ((nMsr & 0x100) == 0x0)))  
		{                
			CreateNewDataBlock();
            bFirstData = TRUE;
		}
	}
    
    if (bFirstData)  
    {            
         if(WriteFirstData(g_curTime, nMsr/*vlv.nMsr*/))//, TRUE))     
            pWrite = pBwrite + PACKET_HEADER_SIZE;
        else
            return FALSE;
    }
    else   
    {            
        #ifdef DebugMode
        SendDebugMsg("\r\nWriteData");
        #endif DebugMode
        temp_address = WriteIntoExte2(/*vlv.nMsr*/nMsr, pWrite, 1);

	    if (temp_address != 1)
		    pWrite = temp_address;       
        else
        {
            #ifdef DebugMode
            SendDebugMsg("\r\nSaveMeasurme failed5 ");
            #endif DebugMode
            return FALSE;
        }
    }     
    msrHour = (int)g_curTime.hour;//readClockBuf[4];  
    unitType = nType;
    if (SaveControlParam() == FALSE)
    {
        #ifdef DebugMode
        SendDebugMsg("\r\nSaveMeasurme failed6 ");
        #endif DebugMode
        return FALSE;
    }
    if (nBtr > 0)            
        res = SaveSensorParams(nBtr, nType, nRssi); 

    if (res == FALSE)
    {                                                                              
        #ifdef DebugMode
        SendDebugMsg("\r\nSaveMeasurme failed7 ");
        #endif DebugMode
        return FALSE;
    }

	return TRUE;
}

char SaveHistoryMeasurments(char* hstr)
{
	unsigned int temp_address, i;
    int nData;      
    int tmpValues[5];   
    BYTE bHasnumber;             
    char hour, day, month, year;   
    DateTime dt;

    #ifdef DebugMode
    SendDebugMsg("\r\nSaveHistoryMeasurments ");
    #endif DebugMode
    
	//read control parameters from cpu_e2 into ram
	//arrange the pointers and parameters for manipulation
	if(ReadExte2ToRam(TRUE) == FALSE)
    {
		return FALSE;
    }  
                      
    for (i = 0; i < 5; i++)
        tmpValues[i] = bytes2int(&hstr[i*2]);
      
    nData = 4;          
    do
    {
        bHasnumber = 0;
        do
        {
            if (tmpValues[nData] == -9999)
                nData--;
            else
                bHasnumber = 1;
        }
        while ((nData >= 0) && (bHasnumber == 0));  
        if (nData < 0)
            break;   
        // create new block   
        if(pWrite != (pBwrite + PACKET_HEADER_SIZE - 2))      
            CreateNewDataBlock();      

        //calc hour of data  
        year = g_curTime.year; 
        month = g_curTime.month;
        day = g_curTime.day;
        if (g_curTime.hour >= (nData+1))
    	    hour = g_curTime.hour - (nData+1);  
        else                                   
        {          
            hour = 24 + (g_curTime.hour - (nData+1)); // go to prev day  
            if (day > 1)               // if day is not first in month
                day--;
            else                                  // if it first day of month - go to prev month...
            {                 
                if (month > 1)               // if its not first month                
                    month--;                  // one month before     
                else                     
                {                                       //if its 1/1/20XX   
                    month = 12;               // go to december of prev year   
                    year--;
                }
                if ((month == 12) || (month == 10) ||(month == 8) || 
                    (month == 7)||(month == 5) || (month == 3)  || (month == 1))          //- January - March - May  - July - August  - October - December                
                    day = 31;       //31st day of prev month                           
                else
                    if ((month == 11) || (month == 9) ||(month == 6) || (month == 4))  //- April - June  - September - November                        
                        day = 30;       //31st day of prev month                       
                    else
						if (month == 2)
							day = 28;       //31st day of prev month   
            }            
        }  
        dt.year = year;
        dt.month = month;
        dt.day = day;
        dt.hour = hour;
        dt.minute = 0;
                                                   
        if(WriteFirstData(dt, tmpValues[nData--]))//, TRUE))   
            pWrite = pBwrite + PACKET_HEADER_SIZE;
        else
            return FALSE;    
        while (nData >= 0)  
        {                     
            if (tmpValues[nData] == -9999)
                break;
            temp_address = WriteIntoExte2(tmpValues[nData--], pWrite, 1);
            hour++;
            if (temp_address != 1)
                pWrite = temp_address;       
            else
            {
                return FALSE;
            }       
        }
    }  
    while (nData >= 0);   
    // set the last msr hour        
    msrHour = hour;//(int)readClockBuf[4];
    if (SaveControlParam() == FALSE)
    {
        return FALSE;
    }   

	return TRUE;
}

char ResetReadPointer()
{
	//read control parameters from cpu_e2 into ram
	//arrange the pointers and parameters for manipulation
	if(ReadExte2ToRam(TRUE) != TRUE)
		return FALSE;
    pBread = pOriginalReadBlock;
    pRead = pBread; 
    // save backthe correct hour 
    msrHour = prevMsrHour;
    if (SaveControlParam() == FALSE)
        return FALSE;

	return TRUE;
}

//get sensoer measurments results from the external eeprom
//read_mode = 0 - send all data (from 'first' block)
//read_mode = 1 - send un read data (from 'read' block)
//read_mode = 2 - next data block
//read_mode = 3 - last data block again
//arrange in 'sensDataBlock': more data|records; status; time; data;
//return 1 (succsess) or 0 (failure)
char GetMeasurments(char read_mode)
{
    unsigned int n; 
    ValveCmd vlv;   
    #ifdef Satellite
    gUnreadValues = 0;  
    #endif Satellite
       
    #ifdef DebugMode
    SendDebugMsg("\r\nGetMeasurments ObjToMsr = \0");
    PrintNum(objToMsr);    
    #endif DebugMode   
      
    if (objToMsr < MAX_CMD)
    {
        vlv  = GetCmd(objToMsr);
      
        if (vlv.lSenID == 0UL)
            return FALSE;
    }
//    else
//        if (SenIDArr[objToMsr-MAX_CMD] == (unsigned long)0)
//            return FALSE; 
                
//     #ifdef DebugMode
//    SendDebugMsg("\r\nSenIDArr[objToMsr] = \0");
//    PrintNum(SenIDArr[objToMsr-MAX_CMD]);    
//    #endif DebugMode   
      
	//read control parameters from cpu_e2 ('pSens_ext_e2' address) into ram
	//arrange the pointers and parameters for manipulation
	if(ReadExte2ToRam(TRUE) == FALSE)
    {
		return FALSE;
    }
    //check if there is any data block in eeprom
	if(pWrite == (pBwrite + PACKET_HEADER_SIZE - 2)) //there is no data records yet
	{                      
        #ifdef DebugMode
        SendDebugMsg("\r\nno data to read 1\0");
        #endif DebugMode 
		return NO_DATA;
	}

   	//set the proper address for data block to be read
	switch(read_mode)
	{
		case 1:
            if ((pBwrite == pBread) && (pWrite == pRead))
            {      
                return NO_DATA;
            }
            ReadSensorParams();                             
            if (pBwrite >= pBread)
                n = pBwrite - pBread;
            else
            {
                n = pEnd_space - pBread + 1;
                n += pBwrite - pStart_space;
            }
            n /= PCKT_LNGTH;

            nUnreadBlocks = n + 1;
                
            if (nUnreadBlocks > MAX_PCKTS_PER_SENSOR)
                nUnreadBlocks = 1;
            #ifdef Satellite
            // for satellite modem
            arrReadPointer[objToMsr] = pBread;
            #endif Satellite
            // for GSM modem:
            pOriginalReadBlock = pBread;
        break;
		case 2:
               //if read block = write block
            if(pBread == pBwrite)
		        break;
            else
                NextpReadAddress();
            break;
        //dont chang pBread, start read from last block send
		case 3:
           break;
		default:
            return FALSE; //exit the function (failure)
	}                       
   
	if(CopyBlockIntoRam() == FALSE) //copy data block at the pBread address
    {
		return FALSE;
    }

    // if the block is the last one to send - keep the inside address
    if (pBwrite == pBread)
        pRead = pWrite;

	//save control parameters into ext_e2
	if (SaveControlParam() == FALSE)
    {
		return FALSE;
    }

	return TRUE;
}

#ifdef Satellite
char ResetAllReadPointers()    //
{
    BYTE n = objToMsr;

    for (objToMsr = SENSOR1; objToMsr <= nMaxSenNum; objToMsr++)
    {
        if (arrReadPointer[objToMsr] != MAX_ADDRESS) // if send sensor data
        {
            #ifdef DebugMode
            SendDebugMsg("\r\nreset read pointer of sensor ");
            printNum(objToMsr);
            #endif DebugMode
            //read control parameters from cpu_e2 into ram
            //arrange the pointers and parameters for manipulation
            if(ReadExte2ToRam(TRUE) != TRUE)
                return FALSE;
            pBread = arrReadPointer[objToMsr];  // retrieve the read pointer
            pRead = pBread;
            if (SaveControlParam() == FALSE)
                return FALSE;
        }
    }
    // retrieve  objToMsr to original value
    objToMsr = n;
	return TRUE;
}

// check if there are more packets with data to this sensor and save the transmition ID in the header of sent block
char IsMoreData()
{
	//read control parameters from cpu_e2 ('pSens_ext_e2' address) into ram
	//arrange the pointers and parameters for manipulation
	if(ReadExte2ToRam(TRUE) != TRUE)
		return FALSE;

    //if read block = write block
    if(pBread == pBwrite)
        return FALSE;

    return TRUE;
}
#endif Satellite

//check validity of sensors data pointers and interval
long PointersValidate()
{
	//check sensor pointers validate:
    // no need to check. function ReadExte2ToRam already checks ranges
	if(ReadExte2ToRam(FALSE) == FALSE)
    {
		return FALSE;
    }                           
    //ointers valid - return sensor ID
    return Bytes2Long(eepromReadBuf+8);
}

char ResetPointers(/*unsigned int senIndex, */char bResetID)
{
    if (objToMsr >= AppEepromData.nMaxSenNum)
        return FALSE;    
    //set pointer to first data block
    pStart_space = SENSOR_MEMORY_START + (objToMsr * SENSOR_MEMORY_SIZE);
    //set pointer of write data block
    pBwrite = pStart_space;    //sens1_data_start;
    //set write pointer
    pWrite = pStart_space; //sens1_data_start;
    pWrite += (PACKET_HEADER_SIZE - 2);
    //set read block pointer
    pBread = pStart_space; //sens1_data_start;
    pRead =  pStart_space;
    //set pointer to end data space
    pEnd_space = pStart_space + SENSOR_MEMORY_SIZE - SENSOR_CNTRL_PRM_SIZE - 1;
    //set parameters for sensor
    pSens_ext_e2 = pEnd_space + 1;  
    if (bResetID == TRUE) 
    {
        if (objToMsr < MAX_CMD)     
            InitVlvCmd(objToMsr);
//        else
//            SenIDArr[objToMsr-MAX_CMD] = 0;
    }        
    // last msr hour
    msrHour = 0;
    unitType = 0;           
    //save control parameters into ext_e2
    return SaveControlParam();          
}

//initiate the data blocks in ext_e2 for all sensors
char InitDataBlocks()
{
//    unsigned int senIndex;
    for (objToMsr = VCU1; objToMsr < AppEepromData.nMaxSenNum; objToMsr++)   
    {                          
        ResetPointers( TRUE);  
    }
	//else if ok
	return TRUE;
}

///////// end of data manager module ///////////
////////////////////////////////////////////////

