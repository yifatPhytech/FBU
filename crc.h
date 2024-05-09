#ifndef __CRC_H__
#define __CRC_H__

// calculates CRC code of the specified buffer
unsigned short CRC16_CCITT(const unsigned char *data, unsigned short length, unsigned short crc);

#endif
