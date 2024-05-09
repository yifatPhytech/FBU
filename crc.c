#include "crc.h"

// calculates CRC code of the specified buffer
unsigned short CRC16_CCITT(const unsigned char *data, unsigned short length, unsigned short crc)
{
	unsigned char b;

	while (length > 0)
	{
		b = *data;
		crc = (crc >> 8) | (crc << 8);
		crc ^= b;
		crc ^= (crc & 0xff) >> 4;
		crc ^= crc << 12;
		crc ^= (crc & 0xff) << 5;

		++data;
		--length;
	}
	
	return crc;
}
