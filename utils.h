#include "define.h"

extern eeprom _tagAPPEEPROM AppEepromData;

typedef union {
    int ival;
    unsigned char bval[2];
} int_bytes;

typedef union {
    float fVal;
    char bVal[4];
} float_bytes;

typedef union {
    unsigned long lVal;
    char bVal[4];
} Ulong_bytes;


BYTE CheckSum( BYTE *buff, BYTE length, BYTE param );

void cpu_e2_to_MemCopy( BYTE* to, char eeprom* from, BYTE length);

void cpu_flash_to_MemCopy( BYTE* to, char flash* from, BYTE length);

BYTE CopyFlashToBuf( BYTE* to, char flash* from);

void MemCopy( BYTE* to, BYTE* from, BYTE length);

int bytes2int(char* buf);

void int2bytes(int int_pointer, char* buf);

long Bytes2ULong(char* buf);

void ULong2Bytes(unsigned long l, char* buf);

void Float2Bytes(float f, char* buf);

//float Bytes2Float(char* buf);

void MemCopy_to_cpu_e2( char eeprom* to, BYTE* from, BYTE length);

BYTE IsZeroID(eeprom char* p);

BYTE IsSameID(char* cID);
