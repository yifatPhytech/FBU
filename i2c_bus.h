//i2c_bus.h file
#ifndef _I2C_BUS.H
#define _I2C_BUS.H

/////////////////////////////////////////////
// i2c_bus functions
////////////////////////////////////////////

unsigned char SendBuf(unsigned char adress, int length, unsigned char  *buffer);

unsigned char GetBuf(unsigned char adress, int length, unsigned char *buffer) ;

#endif _I2C_BUS.H
