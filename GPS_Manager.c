#ifdef UseGPS
#include "define.h"

extern unsigned int buffLen;
extern bit bWaitForGPSData;
extern char RxUart1Buf[MAX_RX1_BUF_LEN];
extern float g_fLat;
extern float g_fLon;
extern int nTimeCnt;
extern bit bCheckRxBuf;
extern unsigned int rx1_buff_len;
extern BYTE msrCurTask;
extern volatile BYTE bEndOfGPSTask;

int nSecOfStartRead;
BYTE b_gotLocation;

int GetDirection(BYTE c)
{
    int res;
    switch (c)
    {
      case 'N':
      case 'E':
          res =  1;
          break;
      case 'S':
      case 'W':
          res = -1;
          break;
      default:
          res = 0;
    }
    return res;
}

void ClearTmpBuf(BYTE* s)
{
    int i;
    for (i = 0; i < 10; i++)
		s[i] = 0;
}

BYTE VarifyChecksum(BYTE * buf, BYTE len)
{
    BYTE i = 0, sum = 0, n;
    while ((buf[i] != '*') && (i < len))
    {
        if ((buf[i] != '*') && (buf[i] != 'I') && (buf[i] != '$'))
            sum ^= buf[i];
        i++;
    }
    i++;
    if ((buf[i] >= '0') && (buf[i] <= '9'))// || ((buf[i] >= 'A') && (buf[i] <= 'F')))
        n = buf[i] - 0x30;
    if ((buf[i] >= 'A') && (buf[i] <= 'F'))
        n = buf[i] - 0x37;
    i++;
    if ((buf[i] >= '0') && (buf[i] <= '9'))
        n = n * 16 + (buf[i++] - 0x30);
    if ((buf[i] >= 'A') && (buf[i] <= 'F'))
        n = n * 16 - 0x37;
    if (n == sum)
        return TRUE;
	return FALSE;
}

BYTE IsEndOfField(BYTE c)
{
	if ((c == ',') || (c == '*'))
		return TRUE;
	else
		return FALSE;
}

float Convert2Float(BYTE* s, BYTE l)
{
	BYTE i;
	float f1 = 0, f2 = 0;
	BYTE left = 1;
	int n = 1;

	for (i = 0; i < l; i++)
		if ((s[i] >= '0') && (s[i] <= '9'))
			if (left)
				f1 = f1 * 10 + (s[i] - 0x30);
			else
			{
				f2 = f2 * 10 + (s[i] - 0x30);
				n *= 10;
			}
		else
			if (s[i] == '.')
				left = 0;
	f2 /= n;
	f1 += f2;
	return f1;
}

void ParseNmea()
{
    //                              $GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
//  uint8_t line[MINMEA_MAX_LENGTH] = "$GPGGA,183730,3907.356,N,100,W,1,05,1.6,646.4,M,-24.1,M,,*75";;
//    char line[MINMEA_MAX_LENGTH] = "$GNGGA,,,,,,0,00,99.99,,,,,,*56";;
    BYTE line[MINMEA_MAX_LENGTH];
    BYTE tmp[10];
    BYTE  t, index;
    BYTE res = FALSE;
    BYTE bGGA = FALSE;
    int d, i;
    BYTE uartRxCnt = buffLen;
    float f;
//    #ifdef DebugMode
//    SendDebugMsg("\r\nParseNmea: ");
//    #endif DebugMode

#ifdef LINE_EXAMPLE
    char exampleline[MINMEA_MAX_LENGTH] = "$GPGGA,183730,3907.356,N,12102.482,W,1,05,1.6,646.4,M,-24.1,M,,*75";
    uartRxCnt = 66;
    for (i = 0; i < uartRxCnt; i++)
        line[i] = exampleline[i];
#else
    if (uartRxCnt > MINMEA_MAX_LENGTH)
        uartRxCnt = MINMEA_MAX_LENGTH;
    for (i = 0; i < uartRxCnt; i++)
    {
        line[i] = RxUart1Buf[i];
//        #ifdef DebugMode
//        putchar1(line[i]);
//        #endif DebugMode

    }
#endif LINE_EXAMPLE


  // check until end of buffer or got answer
    index = 0;
    while (((index + 5) < uartRxCnt) && (res == FALSE))
    {
        // found beginning of GGA message
        if ((line[index] == '$') &&
            (line[index+3] == 'G') &&
            (line[index+4] == 'G') &&
            (line[index+5] == 'A'))
            bGGA = TRUE;
        index++;
        if ((index < MINMEA_MAX_LENGTH - 6) && (bGGA == TRUE))
        {
            bGGA = FALSE;
            if (VarifyChecksum(&line[index], uartRxCnt) == FALSE)
            {
                index += 6;
                continue;
            }
            index += 6;
            t = 0;
            while ((!IsEndOfField(line[index])) && (index < uartRxCnt))
              tmp[t++] = line[index++];
            if (t >= 6)
            {
              //Convert2Time(tmp,t);
            }
            ClearTmpBuf(tmp);
            index++;
            t = 0;
            while ((!IsEndOfField(line[index])) && (index < uartRxCnt))
              tmp[t++] = line[index++];
            if (t)
            {
                g_fLat = Convert2Float(tmp,t);
            }
            ClearTmpBuf(tmp);
            index++;
            if (line[index] != ',')
            {
                d = GetDirection(line[index++]);
                g_fLat *= d;
            }
            index++;
            t = 0;
            while ((!IsEndOfField(line[index])) && (index < uartRxCnt))
              tmp[t++] = line[index++];
            if (t)
            {
                g_fLon = Convert2Float(tmp,t);
            }
            ClearTmpBuf(tmp);
            index++;
            if (line[index] != ',')
            {
                d = GetDirection(line[index++]);
                g_fLon *= d;
            }
            index++;
            t = 0;
            // GPS quality indicator
            if (line[index] != ',')
            {
                t = line[index++];
                //if result is invalid - abort
                if (t == 0)
                    continue;
            }
            index++;
            // skip parameter
            while ((!IsEndOfField(line[index])) && (index < uartRxCnt))
                index++;
            index++;
            // get dilution parameter
            t = 0;
            while ((!IsEndOfField(line[index])) && (index < uartRxCnt))
                tmp[t++] = line[index++];
            f = Convert2Float(tmp,t);
            if ((t > 0) && (f < 6))
            {
                b_gotLocation = TRUE;
//                #ifdef DebugMode
//                SendDebugMsg("FOUND GOOD LOCATION\r\n");
//                #endif DebugMode

            }
        }
    }
}

void InitGPSVars()
{
//    TurnOnLed(LED_1, LED_BLINK);
    b_gotLocation = FALSE;
    // set baud rate to 9600 x2
    SetUART1BaudRate(RATE9600);
    ENABLE_UART1();
    UART1Select(UART_GPS);
    //   power to uart
    GPS_PWR_ON();
    delay_ms(5000);
    GPS_IGN_ON();
    delay_ms(500);
    GPS_IGN_OFF();
    nTimeCnt = MAX_SEC_4_GPS * 10;
//    TurnOffLed();
}

void GPSOff()
{
    //   power off uart
    GPS_PWR_OFF();
    SetUART1BaudRate(RATE38400);
    UART1Select(UART_RADIO_UHF);
    TurnOnLed(LED_1, LED_ON);
    #ifndef DebugMode
    DISABLE_UART1(); 
    #endif DebugMode
}

BYTE GetNextGPSTask()
{
    switch (msrCurTask)
    {
        case TASK_NONE:
            msrCurTask = TASK_GPS_INIT;
        break;
        case  TASK_GPS_INIT:
            msrCurTask = TASK_GPS_START_READ;
        break;
        case TASK_GPS_START_READ:
            if (bCheckRxBuf == TRUE)
                msrCurTask = TASK_GPS_PARSE;
            else
                if (nTimeCnt > 0)
                {            
                    if ((nSecOfStartRead - nTimeCnt) >= 100) 
                        msrCurTask = TASK_GPS_CLOSURE;
                    else
                        return WAIT;  
                }
                else
                    msrCurTask = TASK_GPS_CLOSURE;
        break;
        case TASK_GPS_PARSE:
            if (b_gotLocation)
            {
                msrCurTask = TASK_GPS_CLOSURE;
            }
            else
                if (nTimeCnt > 0)
                    msrCurTask = TASK_GPS_START_READ;
                else
                {
                    msrCurTask = TASK_GPS_CLOSURE;
                }
        break;
        default:
    }
    return CONTINUE;
}

void GPSMain()
{
    if (GetNextGPSTask() == WAIT)
        return;

    switch (msrCurTask)
    {
        case TASK_GPS_INIT:
            InitGPSVars();
        break;
        case TASK_GPS_START_READ:
//            #ifdef DebugMode
//            UART1Select(UART_GPS);
//            #endif DebugMode
            rx1_buff_len = 0;
            bCheckRxBuf = FALSE;
            bWaitForGPSData = TRUE; 
            nSecOfStartRead = nTimeCnt;
        break;
        case TASK_GPS_PARSE:
            bCheckRxBuf = FALSE;
            ParseNmea();
        break;
        case TASK_GPS_CLOSURE:
            GPSOff();
            msrCurTask = TASK_NONE;
            bEndOfGPSTask = TRUE;
            bWaitForGPSData = FALSE;
            break;
        default:
    }
}
#endif UseGPS