
#define SEND_ALERT  1
#define SEND_PUMP   2
#define SEND_VCU    4
#define SEND_CMPNT  8

typedef struct
{
    unsigned int    pVCUDataWrite;
    unsigned int    pVCUDataRead;
    unsigned int    pPmpActionWrite;
    unsigned int    pPmpActionRead;
    unsigned int    pCmpsDataWrite;
    unsigned int    pCmpsDataRead;
    unsigned int    pAlertWrite;
    unsigned int    pAlertRead;  
    BYTE            pVCUDataOverlap;
    BYTE            pCmpsDataOverlap;
} _ExtEpromPointers;
/////////////////////////////////////////////
// Data_manager functions
////////////////////////////////////////////
char SaveVcuData(char* data, BYTE len, BYTE situation);

//char SaveVcuStatus(unsigned long id, BYTE rssi, unsigned int status);

char SavePumpActionData(int action, char res, unsigned int cmdIdx);

char SaveCBUPortData(char * data);

char SaveAlertData(char * alert);

//BYTE ReadVCUPacket();

//char LoadVCUIds();

void SaveVCUIdAtEeprom(unsigned long id, BYTE idx);

BYTE ValidatePointers();

//BYTE GetEprom2Send();

BYTE GetNextEprom2Send(BYTE prevPost);

int GetEpromPcktCnt(BYTE nType);

BYTE ReadPacket(BYTE nPacketType);

void InitWritPntr(BYTE nPacketType);

//arrange the saving of sensors measurments results in the external eeprom
//the function return (1 = success) or (0 = failue)
//char SaveMeasurments(BYTE nType, int nMsr,  unsigned int nBtr, BYTE nRssi);

//char SaveHistoryMeasurments(char*);

//char GetMeasurments(char read_mode);

//char InitDataBlocks();

//long PointersValidate();

char ResetPointers();

BYTE ReadPointers();

char SavePointers();

char ResetReadPointer(BYTE nPacketType);

void InitVCUIdEeprom();