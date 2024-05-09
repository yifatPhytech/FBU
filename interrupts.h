#include <stdbool.h>

//extern bool fMinuteTimer;
extern BYTE flgUart1Error;
extern unsigned int rx0_buff_len;
extern bit bCheckRxBuf;
extern bit bCheckRx1Buf;
extern int TimeLeftForWaiting;
extern BYTE g_fRS485Call;
extern unsigned int rx1_buff_len;
extern volatile char RxUart0Buf[MAX_RX_BUF_LEN];
extern char RxUart1Buf[MAX_RX1_BUF_LEN];
extern unsigned int buffLen;
extern unsigned int nCntDown;
extern char DbgBuf[50]; 

#define ENABLE_TIMER2()    (TCCR2B=(0<<WGM22) | (1<<CS22) | (1<<CS21) | (1<<CS20));
#define DISABLE_TIMER2()   (TCCR2B=(0<<WGM22) | (0<<CS22) | (0<<CS21) | (0<<CS20));

void TransmitBuf(char iPortNum);