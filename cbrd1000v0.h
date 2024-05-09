/*
 * cbrd1000v0.h
 *
 *  Created on: Apr 14, 2022
 *      Author: Reuben
 */

#ifndef CBRD1000V0_H_
#define CBRD1000V0_H_


/*	PORTA	*/
#define GPIO_MCU_SYS_VTG_PIN				0	// A ADC0: external input voltage  monitor: VEXT/?
#define GPIO_MCU_BAT_VTG_PIN 				1	// A ADC1: Battery voltage monitor: VBATT/2
#define GPIO_MCU_READ_VTG_EN_PIN			2	// O Battery voltage samples enable.
#define GPIO_MCU_CHRG_SRC_IN_PIN			3	// I battery charge indicator
#define GPIO_MCU_CHRG_ISET_PIN				4	// O select charge current: 0 - 100mA; 1 - 500mA
#define GPIO_MCU_RADIO_RST_EN_PIN			5	// O reset ezr32 module radio
#define GPIO_MCU_RADIO_PWR_EN_PIN			6	// O radio radio power on/off
#define GPIO_MCU_RADIO_CTS_PIN				7	// O get packet data from module radio

/*	PORTB	*/
#define	GPIO_MCU_BAT_SYSOFF_PIN				0	// O
#define	GPIO_MCU_EXT_RTS_PIN				1	// O send low edge pulse - command to external box
#define	GPIO_MCU_RTC_INT2_PIN				2	// I EXINT2 - real clock time - CLOCK_INT=0: each 60sec
#define	GPIO_MCU_RS485_CTRL_PIN				3	// O rs485 transition enable
#define	GPIO_MCU_RSRV_IOC_PIN				4	// Not connection
#define	GPIO_MOSI_PIN						5	// I MOS PROGRAMMER. CODE
#define	GPIO_MISO_PIN						6	// I MISO PROGRAMMER. CODE
#define	GPIO_SCK_PIN						7	// I SCK PROGRAMMER. CODE

/*	PORTC	*/
#define	GPIO_MCU_I2C_SCL_PIN				0	// O 	I2C_SCL	TWI: two wire interface - clock
#define	GPIO_MCU_I2C_SDA_PIN				1	// I/O 	I2C_SDA	TWI: two wire interface - data
#define	GPIO_MCU_EC21_WAKEUP_PIN			2	// O 	notify to modem i'm sleeping
#define	GPIO_MCU_EC21_AP_READY_PIN			3	// I 	modem is ready
#define	GPIO_MCU_EC21_IGNITION_PIN			4	// O 	modem power ignition on (Pulse 5sec)
#define	GPIO_MCU_EC21_RESET_PIN  			5	// O 	modem reset
#define	GPIO_MCU_EC21_STATUS_PIN			6	// I 	modem status - ready to send at  command
#define	GPIO_MCU_EC21_RI_INT_PIN			7	// I 	modem notify data incoming

/*	PORTD	*/
#define	GPIO_MCU_RXD0_PIN					0	// I UART0-RDX - COMMON GSM/SATELLITE
#define	GPIO_MCU_TXD0_PIN					1	// O UART0-TDX - COMMON GSM/SATELLITE
#define	GPIO_MCU_RXD1_PIN					2	// I UART1-RDX
#define	GPIO_MCU_TXD1_PIN					3	// O UART1-TDX
#define	GPIO_MCU_UART1_S0_PIN				4	// 00 	- UART_RADIO_UHF
#define	GPIO_MCU_UART1_S1_PIN				5	// 01 	- UART_GPS
												// 10 	- UART_DBG
												// 11 	- NONE
#define	GPIO_MCU_RS485_RXD_INT_PIN			6	// I notify incoming data from external box
#define	MCU_MODEM_PWR_EN					7	// 0 switch modem power ldo


#define EC21_IGNITION_ON (PORTC.GPIO_MCU_EC21_IGNITION_PIN = 1);
#define EC21_IGNITION_OFF (PORTC.GPIO_MCU_EC21_IGNITION_PIN = 0);

// bit = 1,2,3 : on_off = 0,1
//#define LED_ACTIVATION(bit, on_off)				(on_off) ? (sbi( PORTA, ((bit)+4))) : (cbi( PORTA,((bit)+4)))



#endif /* CBRD1000V0_H_ */
