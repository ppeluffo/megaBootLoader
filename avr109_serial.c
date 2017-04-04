/*
 * MBL_serial.c
 *
 *  Created on: 1 de feb. de 2017
 *      Author: pablo
 */


#include "avr109.h"

//------------------------------------------------------------------------------------
// UART BOOT FUNCTIONS
//------------------------------------------------------------------------------------
void UART_init(void)
{

u16 bauddiv;

	//bauddiv = 8;		// 115200
	bauddiv = 103;	// 9600

	// UARTS enable
	sbi(UARTCTL_DDR, UARTCTL);
	cbi(UARTCTL_PORT, UARTCTL);

	outb(UBRR1L, bauddiv);
	outb(UBRR1H, bauddiv>>8);
	outb(UCSR1B, BV(RXEN1)|BV(TXEN1));
	UCSR1C = ( serUCSRC_SELECT | serEIGHT_DATA_BITS );
	outb(UCSR1A, BV(U2X1));


}
//------------------------------------------------------------------------------------
void pvb_sendchar(unsigned char c)
{
	// Envio el caracter c por la UART y espero que termine

	UART_DATA_REG = c;                                   		// prepare transmission
	while (!(UART_STATUS_REG & (1 << TRANSMIT_COMPLETE_BIT)));	// wait until byte sendt
	UART_STATUS_REG |= (1 << TRANSMIT_COMPLETE_BIT);          	// delete TXCflag
}
//------------------------------------------------------------------------------------
unsigned char pvb_recchar(void)
{
	// Espero recibir un caracter en la UART.

	while(!(UART_STATUS_REG & (1 << RECEIVE_COMPLETE_BIT)));  // wait for data
	return UART_DATA_REG;
}
//------------------------------------------------------------------------------------
char getch(void)
{
	// Espero recibir un caracter en la UART.

uint32_t count = 0;

	retransmit_flag = FALSE;

	while(!(UART_STATUS_REG & (1 << RECEIVE_COMPLETE_BIT)))  // wait for data
	{
		count++;
		if (count > MAX_WAIT_IN_CYCLES) //
		{
			retransmit_flag = TRUE;
			break;
		}
	}

	return UART_DATA_REG;
}
//------------------------------------------------------------------------------------


