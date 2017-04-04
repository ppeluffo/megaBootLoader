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
#define	BAUD_RATE_LOW_REG		UBRR1L
#define	UART_CONTROL_REG		UCSR1B
#define	ENABLE_TRANSMITTER_BIT	TXEN1
#define	ENABLE_RECEIVER_BIT		RXEN1
#define	UART_STATUS_REG			UCSR1A
#define	TRANSMIT_COMPLETE_BIT	TXC1
#define	RECEIVE_COMPLETE_BIT	RXC1
#define	UART_DATA_REG			UDR1

// Pin de control de reedSwitch
/* define pin for enter-self-prog-mode */
#define PROG_PORT		PORTD
#define PROG_PIN		PIND
#define PROG_BIT		7
#define PROG_DDR		DDRD
#define PROG_MASK		0x80

/* Constants for writing to UCSRC. */
#define serUCSRC_SELECT					( ( u08 ) 0x00 )
#define serEIGHT_DATA_BITS				( ( u08 ) 0x06 )

void MCP_init(void);
void UART_init(void);
void pvb_sendchar(unsigned char c);
unsigned char pvb_recchar(void);

void pvb_bootLoader_avr109(void);

#define BV(bit)			(1<<(bit))

#define ADDR_T unsigned long

//Here we calculate the wait period inside getch(). Too few cycles and the XBee may not be able to send the character in time. Too long and your sketch will take a long time to boot after powerup.
#define CPU_SPEED	8000000
#define MAX_CHARACTER_WAIT	15 //10 works. 20 works. 5 throws all sorts of retries, but will work.
#define MAX_WAIT_IN_CYCLES ( ((MAX_CHARACTER_WAIT * 8) * CPU_SPEED) / BAUD_RATE )

uint8_t incoming_page_data[256];
uint8_t page_length;
uint8_t retransmit_flag;

//----------------------------------------------------------------------------------------
/* definitions for SPM control */
#define	SPMCR_REG	SPMCSR
#define	PAGESIZE	256	// 256
#define	APP_END	122880
#define	LARGE_MEMORY

/* definitions for device recognition ATmega1284P */
#define	PARTCODE	0x44
#define	SIGNATURE_BYTE_1	0x1E
#define	SIGNATURE_BYTE_2	0x97
#define	SIGNATURE_BYTE_3	0x05

#if defined(GET_LOCK_BITS)    /* avr-libc >= 1.2.5 */
#define _GET_LOCK_BITS() boot_lock_fuse_bits_get(GET_LOCK_BITS)
#define _GET_LOW_FUSES() boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS)
#define _GET_HIGH_FUSES() boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS)
#define _GET_EXTENDED_FUSES() boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS)
#endif /* defined(GET_LOCK_BITS) */

#define _SET_LOCK_BITS(data) boot_lock_bits_set(~data)
#define _ENABLE_RWW_SECTION() boot_rww_enable()

#define _WAIT_FOR_SPM() boot_spm_busy_wait()

#define _LOAD_PROGRAM_MEMORY(addr) pgm_read_byte_far(addr)

#define _FILL_TEMP_WORD(addr,data) boot_page_fill(addr, data)
#define _PAGE_ERASE(addr) boot_page_erase(addr)
#define _PAGE_WRITE(addr) boot_page_write(addr)

/* BLOCKSIZE should be chosen so that the following holds: BLOCKSIZE*n = PAGESIZE,  where n=1,2,3... */
#define BLOCKSIZE PAGESIZE

#endif /* MBL_H_ */
