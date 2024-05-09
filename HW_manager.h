
#define LED_1   0
#define LED_2   1
#define LED_3   2
#define LED_ALL 3
#define CBU_LED_OFF     0
#define CBU_LED_ON      1
#define CBU_LED_BLINK   2

extern bit bExtReset;


/////////////////////////////////////////////
// uart functions
////////////////////////////////////////////

//void TransmitBuf(char iPortNum);

void ResetUart1();

void putchar0(char c);

BYTE UART1Select(BYTE newState);

void SetUART1BaudRate(BYTE rate);

void putchar1(char c);

void HandleLed(BYTE nLedIndex, BYTE ledCmd);

/////////////////////////////////////////////
// general functions
////////////////////////////////////////////

void WakeUpProcedure(void);

void GetNextMainTask();

void PowerDownSleep ( void );

void PrintSensorDef();

void InitProgram( void );

void DeepSleep( void );

void ResetEZR();


