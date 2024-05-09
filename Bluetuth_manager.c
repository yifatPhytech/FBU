#ifndef BLUETOOTH_MANAGER_C
#define BLUETOOTH_MANAGER_C

#include "define.h"
#ifdef BlueToothOption

#define HEADER          0xFF
#define ACK_CODE        0x05
#define NCK_CODE        0x06
BYTE Command;
BYTE MessageId;
BYTE subCmd;
BYTE dataLen;
BYTE curLedIndex = 0;
extern flash unsigned char NumSensors;
extern eeprom char unique_id[]; //sensors id
extern eeprom char cpue2_interval_1;
extern bit bMoreDataToSend;
extern bit bCheckRxBuf;
extern char clockBuf[7]; 		 //buffer for all clock operation need
extern char bEndOfBTTask;;
extern char ComBuf[MAX_TX_BUF_LEN];
extern char RxUart1Buf[MAX_RX_BUF_LEN];
extern char DataBlock[48];
extern char readClockBuf[];	         //buffer for data reading from clock
extern BYTE rx1_buff_len;
extern BYTE BytesToSend;extern BYTE bMonitorConnected;extern int nMaxWaitBtResponse;
extern BYTE monitorCurTask;
extern BYTE objToMsr;
extern int iVoltage;
extern unsigned int nextCompare;
extern int iLastMsr[];
//extern BYTE rssi_val;
//extern bit bReset;
//BYTE setResult;
//extern BYTE TimeLeftForWaiting;
//extern int SensorResult;       //save the measuring result into variable
//extern int iVoltage;


void SendBTConnectString()
{
    //send start_monitoring msg to pc
    //255 255 2 2 0 '$' cs
    ComBuf[0] = ComBuf[1] = 0xff; //HEADER;
    ComBuf[2] = 0x02;   //TRANSCEIVER_SOH;
    ComBuf[3] = 2;//data length
    ComBuf[4] = 0;//msg type - monitoring box
    ComBuf[5] = '$';//msg code
    ComBuf[6] = CheckSum( &ComBuf[4], 2, 1 );

    BytesToSend = 7;
    TransmitBuf(1);
//    DISABLE_TIMER1();
//    TCNT1H=0x00;
//    TCNT1L=0x00;
//    OCR1AH=0x01;
//    OCR1AL=0x68;
//    nextCompare = 0x168;
//    ENABLE_TIMER1();
}

BYTE FindSensorID(void)
{
	BYTE i,j,sensorFound;

	objToMsr = 0;
	for( i = 0; i < NumSensors; i++ )
    {
		sensorFound = TRUE;
		for( j = 0; j < 4 ; j++)
        {
			if(!(RxUart1Buf[((int)6+j)] == unique_id[((int)i*4) + j] ))
            {
				sensorFound = FALSE;
			}
		}
		if (sensorFound)
        {
			objToMsr = i; //sensor number used for registration & rx/tx
			return TRUE;
		}
	}
	return FALSE;
}

void WrapBuffer()
{
    ComBuf[0] = HEADER;
	ComBuf[1] = HEADER;
	ComBuf[2] = 0x02;
	ComBuf[3] = 9 + dataLen;//inner layer fixed bytes + data bytes(-1 CS);
	ComBuf[4] = 0x01;   //1
	ComBuf[5] = 126;
    cpu_e2_to_MemCopy(&ComBuf[6], &unique_id[4*objToMsr],4);
    ComBuf[10] = MessageId;
    ComBuf[11] = dataLen + 3;
    ComBuf[12] = Command + 32; //  V->v P->p D->d
    ComBuf[13] = dataLen + 1;
    ComBuf[14] = subCmd;
    ComBuf[15 + dataLen] = CheckSum(&ComBuf[5], dataLen + 3, 0);
    ComBuf[16 + dataLen] = CheckSum(&ComBuf[4], dataLen + 12, 1);
}

void ExecuteGetDataCommand()
{
    if (subCmd == 4)   //if no more data to send
        dataLen = 2;
    else
        dataLen = 48;
    WrapBuffer();
    if (subCmd != 4)
    {
        GetMeasurments(subCmd);
        //ComBuf[11] = LengthOfData;  255 255 2 11 1 126 18 0 0 0 34 2 100 0 40 65
        ComBuf[11] = 48;
        MemCopy(&ComBuf[12], DataBlock, 48);
        // instead of interval send if there is more data (148=0x94) or not (20=0x14)
        if (bMoreDataToSend)
            ComBuf[13] = 148;
        else
            ComBuf[13] = 20;
        ComBuf[12 + dataLen] = CheckSum(&ComBuf[5], dataLen + 3, 0);
        ComBuf[13 + dataLen] = CheckSum(&ComBuf[4], dataLen + 12, 1);
        BytesToSend = 62;
    }
    else
    {
        ComBuf[11] = 2;
        ComBuf[12] = Command + 32; //  V->v P->p D->d
        ComBuf[13] = 0;
        ComBuf[14] = CheckSum(&ComBuf[5], 9, 0);
        ComBuf[15] = CheckSum(&ComBuf[4], 11, 1);
        BytesToSend = 16;
    }
    TransmitBuf(1);
}

void ExecuteGetParamCommand()
{
    int i;
    dataLen = 0;
    switch(subCmd)
    {
        case 3: //set sensor1/2_last_value into 2 bytes
            int2bytes(iLastMsr[objToMsr], &ComBuf[15]);
            dataLen = 2;
            break;

        case 4: //return the sensor type
            SetSensorType(&ComBuf[15]);
            dataLen = 10;
            break;

	    case 7:
            GetRealTime();
            ComBuf[15] = readClockBuf[0]; //year
            ComBuf[16] = readClockBuf[1]; //month
            ComBuf[17] = readClockBuf[2]; //day
            ComBuf[18] = readClockBuf[4]; //hour
            ComBuf[19] = readClockBuf[5]; //minute
            ComBuf[20] = 0;   // dummy
            dataLen = 6;
            break;

	    case 8:	//return the interval
			ComBuf[15] = cpue2_interval_1;
	        dataLen = 1;
            break;

        case 'e': //return sensor id
            //disable interrupts
            #asm ("cli")
            delay_ms(1);
            for (i = 0; i < NumSensors; i++)
                cpu_e2_to_MemCopy( &ComBuf[15 + (i * 4)], &unique_id[i*4], 4 );
            dataLen = 4 * NumSensors;
            objToMsr = 0;
            //enable iterrupts
            #asm ("sei")
            delay_ms(10);
            break;

        default:
            break;
    }
    // if no data to send back - return and do nothing
    if (dataLen == 0)
    {
        rx1_buff_len = 0;        // if no transmit- init var of buff len manually.
        return;
    }
	WrapBuffer();
    BytesToSend = dataLen + 16;
    TransmitBuf(1);
}

void ExecuteSetParamCommand(char param_type, char* new_data_buf)
{
    char updateOk = FALSE;

	if (param_type == 7) //7 = set clock parameter
	{

        MemCopy( clockBuf, new_data_buf, 3 ); //copy year month day
        MemCopy( &clockBuf[4], new_data_buf+3, 2 );  //copy hour minute
		if(SetRealTime() == SUCCESS)
        {
            updateOk = TRUE;
        }
    }

	if(param_type == 8) //8 = set interval parameter
	{
        //check if new interval > 0
		if(new_data_buf[0] > 0)
		{
            //disable interrupts
            #asm ("cli")
            if (SetNewInterval(new_data_buf[0]) == TRUE)
                updateOk = TRUE;
            //enable iterrupts
            #asm ("sei")
		}
	}
    dataLen = 2;
    WrapBuffer();
    if (updateOk == TRUE)
		ComBuf[13] = ACK_CODE;
	else
	 	ComBuf[13] = NCK_CODE;
    if (param_type == 7) // if set clock - return back rssi (=0) and battery
    {
        ComBuf[14] = 0;
        int2bytes(iVoltage,&ComBuf[15]);
    }
    ComBuf[17] = CheckSum(&ComBuf[5], 12, 0);
    ComBuf[18] = CheckSum(&ComBuf[4], 14, 1);
    BytesToSend = 19;
    TransmitBuf(1);
}

BYTE CheckBlueToothRequest()
{
    BYTE cs, msgLen;
    if (!((RxUart1Buf[0] == 0xff) && (RxUart1Buf[1] == 0xff)))
        return FALSE;
    if (RxUart1Buf[2] != 0x02)
        return FALSE;
    // if size of request is more than bytes arrived:
    if (RxUart1Buf[3] > rx1_buff_len)
        return FALSE;
    // if msg is for this sensor
    if (FindSensorID() == FALSE)
        // or if id is still unknown to
        if (!((RxUart1Buf[6] == 0) && (RxUart1Buf[7] == 0) && (RxUart1Buf[8] == 0) && (RxUart1Buf[9] == 0)))
            return FALSE;
    MessageId = RxUart1Buf[10];
    Command = RxUart1Buf[12];
    subCmd = RxUart1Buf[13];
    msgLen = RxUart1Buf[3];
    cs = CheckSum(&RxUart1Buf[5], msgLen - 2, 0);
    if (cs != RxUart1Buf[msgLen + 3])
        return FALSE;
    cs = CheckSum(&RxUart1Buf[4], msgLen, 1);
    if (cs != RxUart1Buf[msgLen + 4])
        return FALSE;

    return TRUE;
}

char IsBlueToothConnect()
{
    char res = FALSE;
    PINB.1 = 1;
    delay_ms(2);
    if (PINB.1 == 0)
        res = TRUE;
    PINB.1 = 0;
    return res;
}

void SwitchLeds()
{
    delay_ms(500);
    switch (curLedIndex)
    {
        case 1:
            if (PORTA.5 == 0)
                PORTA.5 = 1;
            else
            {
                PORTA.5 = 0;
                curLedIndex++;
            }
        break;
        case 2:
            if (PORTA.6 == 0)
                PORTA.6 = 1;
            else
            {
                PORTA.6 = 0;
                curLedIndex++;
            }
        break;
        case 3:
            if (PORTA.7 == 0)
                PORTA.7 = 1;
            else
            {
                PORTA.7 = 0;
                curLedIndex = 1;
            }
        break;
        default:
            curLedIndex = 1;
        break;
    }
}

void BlueToothMain()
{
    if (nMaxWaitBtResponse == 0)
    {
        bEndOfBTTask = TRUE;
        return;
    }
    if (bCheckRxBuf == TRUE)    // if data recieved
    {
        nMaxWaitBtResponse = 1200;
        bMonitorConnected = TRUE;
        bCheckRxBuf = FALSE;
        if (CheckBlueToothRequest() == TRUE)    // check request - if ok - respond
        {

            if (Command == 'D')
                ExecuteGetDataCommand();
            if (Command == 'V')
                ExecuteGetParamCommand();
            if (Command == 'P')
                ExecuteSetParamCommand(RxUart1Buf[14], &RxUart1Buf[15]);
        }
    }
    if (bMonitorConnected == FALSE)              // if hasnt connected yet - send connection string
    {
        SendBTConnectString();
        delay_ms(1000);
    }
    SwitchLeds();
}

/*
// checks if ID of sensor is the same as got from monitor:
// return value:
// 1- same ID
// 2 - id is 0
// 0 - different ID

BYTE IsSameID()
{
    if ((unique_id[0] == RxUart1Buf[3]) &&
        (unique_id[1] == RxUart1Buf[4]) &&
        (unique_id[2] == RxUart1Buf[5]) &&
        (unique_id[3] == RxUart1Buf[6]))
        return 1;
    if ((RxUart1Buf[3] == 0) &&
        (RxUart1Buf[4] == 0) &&
        (RxUart1Buf[5] == 0) &&
        (RxUart1Buf[6] == 0))
        return 2;
    return 0;
}
   */
/*
//check the rx msg context
//modem
BYTE check_rx_msg()
{
	BYTE MoreBytesExpected;

	//msg configuration:
	//byte[0 ]= 0xff
	//byte[1 ]= 0xff
	//byte[2 ]= TRANSCEIVER_SOH
	//byte[3 ]= #bytes (not relevant)
	//byte[4 ]= DATA_MODE
	//byte[5 ]= CONSETRATOR_SOH
	//byte[6 ]= sensor id msb
	//byte[7 ]= sensor id
	//byte[8 ]= sensor id
	//byte[9 ]= sensor id lsb
	//byte[10 ]= MessageId
	//byte[11 ]= length (of data)
	//byte[12]= command letter
	//byte[13]= length of data (netto)
	//byte[14 ]= start of data
	//byte[? ]= ....
	//byte[? ]= data
	//byte[? ]= CheckSum

        //for pc monitoring proccess: if monitoring input detected
        //if(MonitoringIsPlugged)
        //        TimeOut = 40; //reset time out value
        if(MonitoringIsPlugged == 1)
        {
            if(ComBuf[2] == TRANSCEIVER_SOH)//first byte - look for SOH = 0x02
            {
                //do not enter here again
                MonitoringIsPlugged = 2;
            }
        }

	//this part Deal with High layer message from "pc"
	#ifdef GPRSModem
        //for GPRS: check if it is keep alive msg "31 31 31 31 31 5A"
	if((ComBuf[0] == 0x31) && (ComBuf[5] == 0x5A))
	{
		Command = ComBuf[5];  //"Z"
		return RX_RECEIVE_OK;
	}
        #endif GPRSModem
	MessageId = ComBuf[10];
	//check if message is for us (sensor  id 1 or 2)
    if(!(ForThisHost()))
        return RX_FAILED;

//	MoreBytesExpected = ComBuf[11]; //MoreBytesExpected = len + check sum byte
//
//	//if check sum byte == calculated check sum
//	if( ComBuf[((int)12 + MoreBytesExpected)] == (CheckSum(ComBuf + 5, 7 + MoreBytesExpected, 0)))
//	{
//                //copy the ComBuf into eeprom (for debug)
//                //MemCopy_to_cpu_e2(&Store_rx_buff[0], ComBuf, rx_buff_len);
//		Command = ComBuf[12];  //get command for good message
//		return RX_RECEIVE_OK;
//	}
	return RX_FAILED;
}
//-------
//communication.c
BYTE check_rx_msg()
{
	//this part Deal with High layer message from "PDR"
	if(!(ComBuf[2] == 0x02))//first byte - look for SOH
		return RX_FAILED;
	//else if(ComBuf[2] == TRANSCEIVER_SOH)
	MessageId = ComBuf[10];

	if(!(ComBuf[4] == DATA_MODE))
		return RX_FAILED;

	if(!(ComBuf[5] == CONSETRATOR_SOH))//expect - SOH
		return RX_FAILED;

	//check if message is for us (sensor  id 1 or 2)
	if(!(ForThisHost()))
		return RX_FAILED;

    	//this part not in use?
	if(ComBuf[10] & 0x80)//check if message should return to pdr
		MessageId &= 0x7F;//ignor msb

	MoreBytesExpected = ComBuf[11]; //MoreBytesExpected = len + check sum byte

	//if check sum byte == calculated check sum
	if( ComBuf[((int)12 + MoreBytesExpected)] == (CheckSum(ComBuf + 5, 7 + MoreBytesExpected, 0)))
	{
		Command = ComBuf[12];  //get command for good message
		return RX_RECEIVE_OK;
	}
	return RX_FAILED;
}
//---------------------------------------
//check command function
void check_command()
{
	switch( Command )
	{
	case ('D') :// Get sensor data (RxSensorNumber 0 = sensor #1
	            //RxSensorNumber 1 = sensor #2; RxSensorNumber 2 = sensor #3
				if(get_measurments( ComBuf[13], (RxSensorNumber + 1) ))
					break;
				//else
				return; //exit function without sending msg to pdr
	case ('P') :// Set sensor parameter( parameter, data)
	//char SetSensorParameter(char param_type, char* new_data_buf);
				if(!(SensorIsRegistered[RxSensorNumber] == 2))
					SensorIsRegistered[RxSensorNumber] ++;
				SetSensorParameter(ComBuf[14], &ComBuf[15]);
				break;
	case ('V') :// Get sensor parameter/value
				GetSensorParameter( RxSensorNumber + 1, ComBuf[13] );
				break;
	default: 	return;
	}

	WrapBuffer();

	//BATT_LEDTEST
	//test for the Batt.Low sensing procedure.
	//the Batt.Low test take place at the higest sensor current consumption - the  RF Transmition.
	//set portb.1 as input for battery check (08-03-02)
	if (Command == 'D') 	//test Batt. Low if it is during "Download data" command
	{
		DDRB.1 = 0; 	//set portB.1 as input
		PORTB.1 = 1; 	//input pullup resistor
		TransmitBuf();
		//delay of 30 mSec. before test (sancronized with RF TX operation)
		delay_ms(30);
//		error_decoding(); //check the battery voltage & arrange the status byte
	}
	else
		TransmitBuf(); 		//normal function operation
}
*/
/*
void WrapBuffer(void)
{
	ComBuf[0] = HEADER;
	ComBuf[1] = HEADER;
	ComBuf[2] = TRANSCEIVER_SOH;
	ComBuf[3] = 9 + LengthOfData;//inner layer fixed bytes + data bytes(-1 CS);
	ComBuf[4] = DATA_MODE;
	ComBuf[5] = CONSETRATOR_SOH;

	#ifdef RTU_Interrupting
	//it is interrupting sensor sending its interrupting msg
	// or it is RTU return ACK to its master
	if((send_preambel == 1) || (request_rtu_operation == 1))
	{
	        //destination rtu id
	        cpu_e2_to_MemCopy( &ComBuf[6], &DestenationRtuID[0], 4);
	        //reset flag
	        send_preambel = 0;
	}
	else
	        MemCopy( &ComBuf[6],&SensorAddress[(int)RxSensorNumber * 4],4 );
 	#else
	MemCopy( &ComBuf[6],&SensorAddress[(int)RxSensorNumber * 4],4 );
	//ComBuf[6] = SensorAddress[1][0];
	//ComBuf[7] = SensorAddress[1][1];
	//ComBuf[8] = SensorAddress[1][2];
	//ComBuf[9] = SensorAddress[1][3];
	#endif RTU_Interrupting

	ComBuf[10] = MessageId;
	ComBuf[11] = LengthOfData;
	//ComBuf[12] - should build data packet from this point

	//add this option for test mode
	#ifdef PREV_DATA_BLOCK
	if(WakeupFlag == 1)
	{
		//set faulty checksum. now the pc will returen with reply
		ComBuf[12 + LengthOfData] = 1;
		WakeupFlag = 0; //reset the flag
	}
	else
		// Low  Layer check sum : '7' = SOH1, ADDRESS(4), MSG_ID, LEN.
		ComBuf[12 + LengthOfData] = CheckSum(&ComBuf[5],(7+LengthOfData), 0);//fixed
	#else
	// Low  Layer check sum : '7' = SOH1, ADDRESS(4), MSG_ID, LEN.
	ComBuf[((int)12 + LengthOfData)] = CheckSum(&ComBuf[5],(7+LengthOfData), 0);//fixed
	#endif PREV_DATA_BLOCK

	ComBuf[((int)13 + LengthOfData)] = CheckSum(&ComBuf[4],(9+LengthOfData), 1);//CS + fixed
	// High Layer check sum : '11'= '7' + SOH2, LEN, MSG_TYPE,CHECKSUM

	#ifdef localComm_sensor	//use for local pdr sensor only
		BytesToSend = 16 + LengthOfData;// '14' = fixed characters + length of data(+2 bytes for 485)
	#else
   		BytesToSend = 14 + LengthOfData;// '14' = fixed characters + length of data
 	#endif
}
*/

#endif BlueToothOption

#endif BLUETOOTH_MANAGER_C
