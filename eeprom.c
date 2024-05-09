//////////////////////////////////////////////////////
///////// start 64Kb eeprom implementation /////////
#include "define.h"
#include "i2c_bus.h"
#include "eeprom.h"

//declare local & global variables
unsigned char eepromWriteBuf[MAX_DATA_2_EPRM_SIZE+2]; 	//buffer for eeprom write operation
unsigned char e2cmdByte;			//buffer for the eeprom command byte
char e2_writeFlag;
//extern char err_buf[ERR_BUF_SIZE];
extern volatile unsigned char eepromReadBuf[];	//buffer for eeprom read operation

//command buffer configuration: code=1010; blockNum=0 to 7; lsb=1
//char setReadCmd(char blockNum) {return 0xA1 | (blockNum<<1);}
//char setWriteCmd(char blockNum) {return 0xA0 | (blockNum<<1);}

//write address eeprom page
char e2_writePage(unsigned int address, char write_length, char* string_1)
{
    //char blockNum, i;
    char i;
    e2_writeFlag = 1; //block use of i2c by clock   
    if (write_length > MAX_DATA_2_EPRM_SIZE)    
        return FAILURE;

    eepromWriteBuf[0] = (unsigned char)((address >> 8) & 0xFF);     //address high
    eepromWriteBuf[1]  = (unsigned char)(address) ;                 //address low

    //set the eeprom block num to be writen to
    address >>= 8;
    //blockNum = (char)address;

    SPCR=0x00; //reset spi control register
    e2cmdByte = 0xA0;  // | (blockNum<<1);//set write command

    //copy data to be write onto eeprom into buffer
    for(i = 2; i < (write_length +2); i++)
        eepromWriteBuf[i] = string_1[i-2];

    //send buffer to be writen on eeprom
    if(SendBuf(e2cmdByte , i, eepromWriteBuf) == FALSE)  //if SendBuf function faild
    {
//		err_buf[EE_FAIL] = EE_FAIL;
		return FAILURE;
    }
    //10 ms delay
    delay_ms(10);

    e2_writeFlag = 0; //un block use of i2c by clock
//    err_buf[EE_FAIL] = NO_ERROR;
    return SUCCESS;
}

//read sequential address eeprom bytes
char e2_readSeqBytes(unsigned int address, char read_length)
{
    // if try to read more than writing buffer - return  failed  
    if (read_length > MAX_DATA_2_EPRM_SIZE)  //SENSOR_CNTRL_PRM_SIZE)
        return FAILURE;
    //char blockNum;
    e2_writeFlag = 1; //block use of i2c by clock

    eepromWriteBuf[0] = (unsigned char)((address >> 8) & 0xFF);     //address high
    eepromWriteBuf[1]  = (unsigned char)(address) ;                 //address low

    //set the eeprom block num to be writen to
    address >>= 8;
    //blockNum = (char)address;

    SPCR=0x00; //reset spi control register
    e2cmdByte = 0xA0;// | (blockNum<<1);//set write command

    //send read command and address to eeprom
    if(SendBuf(e2cmdByte , 2, eepromWriteBuf) == FALSE)  //if SendBuf function faild
    {
//        err_buf[EE_FAIL] = EE_FAIL;
        return FAILURE;
    }

    e2cmdByte = 0xA1;// | (blockNum<<1); //set read command
    if(GetBuf(e2cmdByte , read_length, eepromReadBuf) == FALSE)//if GetBuf function faild
    {
//        err_buf[EE_FAIL] = EE_FAIL;
        return FAILURE;
    }
    e2_writeFlag = 0; //un block use of i2c by clock
//    err_buf[EE_FAIL] = NO_ERROR;
	return SUCCESS;
}
///////// end of 24c16b eeprom implementation /////////
