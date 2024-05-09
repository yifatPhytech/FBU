#include "utils.h"

int_bytes union_b2i;
Ulong_bytes lb;
float_bytes fb;

//the function recieve pointer to buf
//buf[0]= lo_byte, buf[1]=  hi_byte
//the function return int (address) combined from 2 bytes
// LITTLE ENDIAN
int bytes2int(char* buf)
{
    //set 2 bytes into union
    union_b2i.bval[0] = buf[0];
    union_b2i.bval[1] = buf[1];

    return union_b2i.ival;
}

//the function recieve int (address) and pointer to buf
//the function set into buf[0] the hi_address_byte
//and into buf[1] the lo_address_byte
void int2bytes(int int_pointer, char* buf)
{
    union_b2i.ival = int_pointer;
    //set 2 bytes into buf
    buf[0] = union_b2i.bval[0];
    buf[1] = union_b2i.bval[1];
}

long Bytes2ULong(char* buf)
{
	//set 4 bytes into union
	lb.bVal[0] = buf[0];
	lb.bVal[1] = buf[1];
	lb.bVal[2] = buf[2];
	lb.bVal[3] = buf[3];

	return lb.lVal;
}

void ULong2Bytes(unsigned long l, char* buf)
{
	lb.lVal = l;
	//set 2 bytes into buf
	buf[0] = lb.bVal[0];
	buf[1] = lb.bVal[1];
	buf[2] = lb.bVal[2];
	buf[3] = lb.bVal[3];
}

//the function recieve float (address) and pointer to buf
//the function set into buf[0] the hi_address_byte
//and into buf[1] the lo_address_byte
void Float2Bytes(float f, char* buf)
{
	fb.fVal = f;
	//set 4 bytes into buf
	buf[0] = fb.bVal[0];
	buf[1] = fb.bVal[1];
	buf[2] = fb.bVal[2];
	buf[3] = fb.bVal[3];
}

//float Bytes2Float(char* buf)
//{
//    memcpy(fb.bVal, buf, 4);
//	return fb.fVal;
//}

//copy from cpu e2 into buf
void cpu_e2_to_MemCopy( BYTE* to, char eeprom* from, BYTE length)
{
	BYTE i;
	for(i = 0; i < length; i++)
		to[i] = from[i];
}

//copy from ram buf into cpu e2
void MemCopy_to_cpu_e2( char eeprom* to, BYTE* from, BYTE length)
{
	BYTE i;
	for(i = 0; i < length; i++)
		to[i] = from[i];
}

void MemCopy( BYTE* to, BYTE* from, BYTE length)
{
	BYTE i;
	for(i = 0; i < length; i++)
		to[i] = from[i];
}

//copy from cpu flash into buf
void cpu_flash_to_MemCopy( BYTE* to, char flash* from, BYTE length)
{
	BYTE i;
	for(i = 0; i < length; i++)
		to[i] = from[i];
}

//copy flash buf to buffer. return num of  copy bytes
BYTE CopyFlashToBuf( BYTE* to, char flash* from)
{
    BYTE index = 0;
    while (from[index] != '@')
    {
        to[index] = from[index];                      //"425#"
        index++;
    }
    return index;
}

//checksum with parameter
// 0 = check_sum ^
// 1 = check_sum +
BYTE CheckSum( BYTE *buff, BYTE length, BYTE param )
{
    BYTE check_sum;

    check_sum = 0;
    while (length--)
    {
        //filter the "246 (F6)" for gprs modem ??
        if(param)
        {
            check_sum += *buff++;
        }
        else
            check_sum ^= *buff++;
    }
    return (check_sum);
}

BYTE IsZeroID(eeprom char* p)
{
    if ((p[0] == 0) && (p[1] == 0) && (p[2] == 0) && (p[3] == 0))
        return TRUE;
    return FALSE;
}

// checks if ID of sensor is the same as got from monitor:
// return value:
// 1- same ID
// 2 - id is 0
// 0 - different ID
BYTE IsSameID(char* cID)
{
    if ((AppEepromData.eLoggerID[0] == cID[0]) &&
        (AppEepromData.eLoggerID[1] == cID[1]) &&
        (AppEepromData.eLoggerID[2] == cID[2]) &&
        (AppEepromData.eLoggerID[3] == cID[3]))
        return 1;
    if ((cID[0] == 0) &&
        (cID[1] == 0) &&
        (cID[2] == 0) &&
        (cID[3] == 0))
        {
            return 2;
        }
    return 0;
}
