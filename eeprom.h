extern char e2_writeFlag;

/////////////////////////////////////////////
// eeprom functions
////////////////////////////////////////////

char e2_readSeqBytes(unsigned int address, char read_length);

char e2_writePage(unsigned int address, char write_length, char* string_1);
