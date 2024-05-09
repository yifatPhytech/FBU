//i2c_bus.c file
#ifndef _I2C_BUS.C
#define _I2C_BUS.C
/////////////////////////////////////////////////
//unsigned char SendBuf_dac(unsigned char adress, int length, unsigned char  *buffer);

//#include <mega324.h>
#include <mega644p.h>
#include <delay.h>
#include "define.h"
#include "i2c_bus.h"

////////// implementation for i2c_bus ///////////
//
//i2c_bus communication with s3531 rtc and 24c16 eeprom

//-------- interface functions for atmel 8535 cpu -------
void SCL_ON(void)  {DDRC.0 = 1; PORTC.0 = 1;} //PC7 = output = 1
void SCL_OFF(void) {DDRC.0 = 1; PORTC.0 = 0;} //PD7 = output = 0
void SDA_ON(void)  {DDRC.1 = 1; PORTC.1 = 1;} //PC0 = output = 1
void SDA_OFF(void) {DDRC.1 = 1; PORTC.1 = 0;} //PC0 = output = 0
void SCL_OUT(void) {DDRC.0 = 1;}  //PC0 = output
//void SCL_IN(void)  {DDRC.0 = 0; PORTC.0 = 1;}  //PC0 = INPUT
void SDA_OUT(void) {DDRC.1 = 1;}  //PC0 = output
void SDA_IN(void)  {DDRC.1 = 0; PORTC.1 = 1;} //PC0 = input = high
char SDA_READ(void){return PINC.1;}  //read PB0 status (0/1)
//-------- end of interface functions for atmel 8535 cpu -------

void SendStartBit(void)
{
    // Make shure that CLOCK and DATA pins are in output direction
    SCL_OUT();
    SDA_OUT();

    // Prepare start condition
    SCL_OFF();
    SDA_ON();
    // Set CLOCK to high
    SCL_ON();
    // Delay for 8 microsecond
    delay_us(10);
    // Set DATA from high to low while CLOCK high  (Define START condition)
    SDA_OFF();
    // Delay for 8 microsecond
    delay_us(10);
    // Clock to low again..
    SCL_OFF();
    // Leave The clock LOW, prepare for byte transmit.
}

void SendStopBit(void)
{
    // Set DATA for output
    SDA_OUT();
    // Data to 0
    SDA_OFF();
    // Clock to HI
    SCL_ON();
    // Delay for 8 microsecond
    delay_us(10);
    // Set data from low to high, while clock hi (Define STOP condition)
    SDA_ON();
    // Leave data and clock HIGH, marking "Bus not busy".
}

void SendByte(unsigned char send_byte)
{

    unsigned char mask;

    // Set DATA for output
    SDA_OUT();
    for (mask = 0x80; mask; mask >>= 1)
    {
        if (send_byte & mask)
            SDA_ON();
        else
            SDA_OFF();
        SCL_ON();
        // Delay for 8 microsecond
    	delay_us(10);
        SCL_OFF();
        delay_us(10); //(09-05-02)
        // !!! &&&& CHECK IF WE NEED DELAY HERE !!!!
    }
}

unsigned char TestAck(void)
{
    int counter;

    // Set Data for input
    SDA_IN();

    SCL_ON();
    for (counter = 200; counter; counter--)
    if (!SDA_READ())
    {
        SCL_OFF();

        return TRUE;   // Acknowledge received OK
        // Return, and leave clock LOW, for next byte.
    }
    return FALSE;     // Acknowledge NOT received,
    // Return, and leave clock HIGH.
}

void SendAck(void)
{
    SDA_OUT();
    // Set data to LOW
    SDA_OFF();
    // Generate Clock pulse
    SCL_ON();
    // Delay for 8 microsecond
    delay_us(10);
    SCL_OFF();
    delay_us(10);
    // Leave data HI, Prepare for receiving
    SDA_ON();
}
/*
void SendNack(void)
{
    SDA_OUT();
    // Set data to HIGH
    SDA_ON();
    // Generate Clock pulse
    SCL_ON();
    // Delay for 8 microsecond
    delay_us(10);
    SCL_OFF();
    delay_us(10);
    // Leave data HI, Prepare for receiving
//    SDA_ON();
}
*/
unsigned char RecByte(void)
{
    unsigned char received_byte;
    unsigned char counter;
    unsigned char read_bit;

    received_byte = 0;
    SDA_IN();             // Enable receiving
    for (counter = 0; counter < 8; counter++)
    {
        SCL_ON();
        delay_us(10);
        read_bit = 0;
        if(SDA_READ())
        	read_bit = 1;
        //read_bit = ((SDA_READ()) ? 1 : 0);
        received_byte = (received_byte << 1) + read_bit;
        SCL_OFF();
    	delay_us(10);
    }

    return (received_byte);

}

// Return TRUE for success !! FALSE otherwise
unsigned char SendBuf(unsigned char adress, int length, unsigned char  *buffer)
{
    int current_byte;
//    #ifdef DebugMode
//    SendDebugMsg("\r\nSendBuf\0");//set real time
//    #endif DebugMode

    SendStartBit();
    SendByte(adress);
    if (TestAck() == FALSE)
    {
        // No ACK,  Send fail..
        #ifdef DebugMode
        SendDebugMsg("\r\nsend address failed\0");//set real time
        #endif DebugMode
        SendStopBit();
        return FALSE;
    }

    for (current_byte = 0; current_byte < length; current_byte++)
    {
//    #ifdef DebugMode
//    putchar1(buffer[current_byte]);
//    #endif DebugMode
        SendByte(buffer[current_byte]);
        //SendByte(*(buffer));
        //buffer++;
        if (TestAck() == FALSE)
        { // No ACK,  Send fail..
            #ifdef DebugMode
            SendDebugMsg("\r\nsend byte failed\0");//set real time
//            putchar1(current_byte);
            #endif DebugMode
            SendStopBit();
            return FALSE;
        }
    }
    SendStopBit();
    delay_ms(5);
    return TRUE;
}

// Return TRUE for success !! FALSE otherwise
unsigned char GetBuf(unsigned char adress, int length, unsigned char *buffer)
{
    int counter;

    if (length <= 0)
        return TRUE;    // No bytes to read !! (Success ??!!)
    SendStartBit();
    SendByte(adress);
    if (TestAck() == FALSE)  // No Ack From device !!
    {
        SendStopBit();
        return FALSE;
    }
    length--;    // we read the last byte outside the for loop !!
    for (counter = 0; counter < length; counter++)
    {
        *(buffer++) = RecByte();
        SendAck();
    }
    // And now, the last byte
    *buffer = RecByte();
    SendStopBit();
    return TRUE;
}

/*
// Return TRUE for success !! FALSE otherwise
unsigned char GetBufClk(unsigned char adress, int length, unsigned char *buffer)
{
    int counter;

    if (length <= 0)
        return TRUE;    // No bytes to read !! (Success ??!!)
    SendStartBit();
    SendByte(adress);
    if (TestAck() == FALSE)  // No Ack From device !!
    {
        SendStopBit();
        #ifdef DebugMode
        SendDebugMsg("\r\nSend address failed\0");//set real time
        #endif DebugMode
        return FALSE;
    }
    // wait till clock is high
    SCL_IN();
    while (PINC.0 == 0);
    SCL_OUT();
    #ifdef DebugMode
    SendDebugMsg("\r\nClock is high ");
    #endif DebugMode

    delay_ms(1);
    length--;    // we read the last byte outside the for loop !!
    for (counter = 0; counter < length; counter++)
    {
        *(buffer++) = RecByte();
        SendAck();
    }
    // And now, the last byte
    *(buffer++) = RecByte();

    SendNack();
    SendStopBit();
    return TRUE;
}
*/
#endif //_I2C_BUS.C
///////// end of i2c_bus implementation /////////
/////////////////////////////////////////////////