/*
 * megaBoorLoader.h
 *
 *  Created on: 5 de dic. de 2016
 *      Author: pablo
 */

#ifndef MBL_H_
#define MBL_H_


#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <compat/deprecated.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/twi.h>

typedef unsigned char  u08;
typedef   signed char  s08;
typedef unsigned short u16;
typedef   signed short s16;

#define FALSE	0
#define TRUE	-1

#define UARTCTL_PORT	PORTD
#define UARTCTL_PIN		PIND
#define UARTCTL			4
#define UARTCTL_DDR		DDRD
#define UARTCTL_MASK	0x10

/* baud rate register value calculation */
#define	CPU_FREQ	8000000
#define	BAUD_RATE	115200
#define	BRREG_VALUE	3

/* definitions for UART control */
#define	BAUD_RATE_LOW_REG	UBRR1L
#define	UART_CONTROL_REG	UCSR1B
#define	ENABLE_TRANSMITTER_BIT	TXEN1
#define	ENABLE_RECEIVER_BIT	RXEN1
#define	UART_STATUS_REG	UCSR1A
#define	TRANSMIT_COMPLETE_BIT	TXC1
#define	RECEIVE_COMPLETE_BIT	RXC1
#define	UART_DATA_REG	UDR1

/* Constants for writing to UCSRC. */
#define serUCSRC_SELECT					( ( u08 ) 0x00 )
#define serEIGHT_DATA_BITS				( ( u08 ) 0x06 )

void MCP_init(void);
void UART_init(void);
void pvb_sendchar(unsigned char c);
unsigned char pvb_recchar(void);

void pvb_bootLoader_avr109(void);

#define BV(bit)			(1<<(bit))


#endif /* MBL_H_ */
