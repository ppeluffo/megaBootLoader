/*
 * MBL_twi.c
 *
 *  Created on: 7 de dic. de 2016
 *      Author: pablo
 */

#include "avr109.h"

#define SCL		0
#define SDA		1
#define I2C_MAXTRIES	5

#define ACK 1
#define NACK 0

// Identificacion en el bus I2C de los MCP
#define MCP0_ADDR			0x40	// MCP23008

#define MCP0_PORT	PORTD
#define MCP0_PIN	PIND
#define MCP0_BIT	5
#define MCP0_DDR	DDRD
#define MCP0_MASK	0x20

// Registros MCP0
#define MCP0_IODIR		0x00
#define MCP0_IPOL		0x01
#define MCP0_GPINTEN	0x02
#define MCP0_DEFVAL		0x03
#define MCP0_INTCON		0x04
#define MCP0_IOCON		0x05
#define MCP0_GPPU		0x06
#define MCP0_INTF		0x07
#define MCP0_INTCAP		0x08
#define MCP0_GPIO 		0x09
#define MCP0_OLAT 		0x0A

// Bits del MCP0
#define MCP0_GPIO_IGPRSDCD			1	// IN
#define MCP0_GPIO_IGPRSRI			2	// IN
#define MCP0_GPIO_OGPRSSW			3	// OUT
#define MCP0_GPIO_OTERMPWR			4
#define MCP0_GPIO_OGPRSPWR			5
#define MCP0_GPIO_OLED				6

//------------------------------------------------------------------------------------
void i2c_init(void);
s08 I2C_masterWrite ( const u08 devAddress, const u08 devAddressLength, const u16 byteAddress, char *pvBuffer, size_t xBytes );
static void pvI2C_setBitRate(int bitrateKHz);
void pvI2C_sendStart(void);
void pvI2C_sendStop(void);
void pvI2C_sendByte(u08 data);
void pvI2C_readByte(u08 ackFlag);
u08 pvI2C_getStatus(void);
s08 pvI2C_waitForComplete(void);
void pvI2C_disable(void);

//------------------------------------------------------------------------------------
void MCP_init(void)
{
	// inicializa el MCP23008 de la placa de logica
	// NO CONTROLO ERRORES.

u08 data;
size_t xReturn = 0U;
u16 val = 0;
u08 xBytes = 0;

	i2c_init();

	// Inicializo los pines del micro como entradas para las interrupciones del MCP.
	cbi(MCP0_DDR, MCP0_BIT);

	// MCP0_IODIR: inputs(1)/outputs(0)
	data = 0;
	data |= ( BV(MCP0_GPIO_IGPRSDCD) | BV(MCP0_GPIO_IGPRSRI) );
	xBytes = sizeof(data);
	I2C_masterWrite(MCP0_ADDR, 1, MCP0_IODIR, &data, xBytes);

	// MCP0_IPOL: polaridad normal
	data = 0;
	xBytes = sizeof(data);
	I2C_masterWrite(MCP0_ADDR, 1,MCP0_IPOL, &data, xBytes);

	// MCP0_GPINTEN: inputs interrupt on change.
	data = 0;
	//data |= ( BV(MCP_GPIO_IGPRSDCD) | BV(MCP_GPIO_IGPRSRI) | BV(MCP_GPIO_ITERMPWRSW) );
	data |=  BV(MCP0_GPIO_IGPRSDCD);
	xBytes = sizeof(data);
	I2C_masterWrite(MCP0_ADDR, 1,MCP0_GPINTEN, &data, xBytes);

	// MCP0_INTCON: Compara contra su valor anterior
	data = 0;
	xBytes = sizeof(data);
	I2C_masterWrite(MCP0_ADDR, 1, MCP0_INTCON, &data, xBytes);

	// MCP0_IOCON: INT active H
	data = 2;
	xBytes = sizeof(data);
	I2C_masterWrite(MCP0_ADDR, 1, MCP0_IOCON, &data, xBytes);

	// TERMPWR ON
	// Al arrancar prendo la terminal para los logs iniciales.
	data = 0;
	data |= BV(MCP0_GPIO_OTERMPWR);	// TERMPWR = 1
	xBytes = sizeof(data);
	I2C_masterWrite(MCP0_ADDR, 1, MCP0_OLAT, &data, xBytes);

}
//------------------------------------------------------------------------------------
void i2c_init(void)
{
	// set pull-up resistors on I2C bus pins
	//sbi(PORTC, 0);  // i2c SCL
	//sbi(PORTC, 1);  // i2c SDA
	PORTC |= 1 << SCL;
	PORTC |= 1 << SDA;
	// set i2c bit rate to 100KHz
	pvI2C_setBitRate(100);
}
//------------------------------------------------------------------------------------
s08 I2C_masterWrite ( const u08 devAddress, const u08 devAddressLength, const u16 byteAddress, char *pvBuffer, size_t xBytes  )
{

u08 tryes = 0;
u08 i2c_status;
char txbyte;
s08 retV = FALSE;
u08 i;

i2c_retry:

	if (tryes++ >= I2C_MAXTRIES) goto i2c_quit;

	// Pass1) START: Debe retornar 0x08 (I2C_START) o 0x10 (I2C_REP_START) en condiciones normales
	pvI2C_sendStart();
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	i2c_status = pvI2C_getStatus();
	if ( i2c_status == TW_MT_ARB_LOST) goto i2c_retry;
	if ( (i2c_status != TW_START) && (i2c_status != TW_REP_START) ) goto i2c_quit;

	// Pass2) (SLA_W) Send slave address. Debo recibir 0x18 ( SLA_ACK )
	txbyte = devAddress | TW_WRITE;
	pvI2C_sendByte(txbyte);
	if ( !pvI2C_waitForComplete() )
		goto i2c_quit;
	i2c_status = pvI2C_getStatus();
	// Check the TWSR status
	if ((i2c_status == TW_MT_SLA_NACK) || (i2c_status == TW_MT_ARB_LOST)) goto i2c_retry;
	if (i2c_status != TW_MT_SLA_ACK) goto i2c_quit;

	// Pass3) Envio la direccion fisica donde comenzar a escribir.
	// En las memorias es una direccion de 2 bytes.En el DS1344 o el BusController es de 1 byte
	// Envio primero el High 8 bit i2c address. Debo recibir 0x28 ( DATA_ACK)
	if ( devAddressLength == 2 ) {
		txbyte = (byteAddress) >> 8;
		pvI2C_sendByte(txbyte);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
		if (i2c_status != TW_MT_DATA_ACK) goto i2c_quit;
	}

	// Envio el Low 8 byte i2c address.
	if ( devAddressLength >= 1 ) {
		txbyte = (byteAddress) & 0x00FF;
		pvI2C_sendByte(txbyte);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
		if (i2c_status != TW_MT_DATA_ACK) goto i2c_quit;
	}

	// Pass4) Trasmito todos los bytes del buffer. Debo recibir 0x28 (DATA_ACK)
	for ( i=0; i < xBytes; i++ ) {
		txbyte = *pvBuffer++;
		pvI2C_sendByte(txbyte);
		if ( !pvI2C_waitForComplete() )
			goto i2c_quit;
		i2c_status = pvI2C_getStatus();
		if (i2c_status != TW_MT_DATA_ACK) goto i2c_quit;
	}

	retV = TRUE;

i2c_quit:

	// Pass5) STOP
	pvI2C_sendStop();

	// En caso de error libero la interface.
	if (retV == FALSE)
		pvI2C_disable();

	return(retV);


}
//------------------------------------------------------------------------------------
// FUNCIONES AUXILIARES PRIVADAS DE I2C
//------------------------------------------------------------------------------------
static void pvI2C_setBitRate(int bitrateKHz)
{
int bitrate_div;

	// SCL freq = F_CPU/(16+2*TWBR*4^TWPS)

	// set TWPS to zero
	//cbi(TWSR, TWPS0);
	//cbi(TWSR, TWPS1);
	((TWSR) &= ~(1 << (TWPS0)));
	((TWSR) &= ~(1 << (TWPS1)));
	// SCL freq = F_CPU/(16+2*TWBR))

	// calculate bitrate division
	bitrate_div = ((F_CPU/1000l)/bitrateKHz);
	if(bitrate_div >= 16) {
		bitrate_div = (bitrate_div-16)/2;
	}

	TWBR = bitrate_div;
}
//------------------------------------------------------------------------------------
void pvI2C_sendStart(void)
{
	// Genera la condicion de START en el bus I2C.
	// TWCR.TWEN = 1 : Habilita la interface TWI
	// TWCR.TWSTA = 1: Genera un START
	// TWCR.TWINT = 1: Borra la flag e inicia la operacion del TWI
	//
	// Cuando el bus termino el START, prende la flag TWCR.TWINT y el codigo
	// de status debe ser 0x08 o 0x10.
	//

	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

}
//-----------------------------------------------------------------------------------
void pvI2C_sendStop(void)
{
	// Genera la condicion STOP en el bus I2C.
	// !!! El TWINT NO ES SETEADO LUEGO DE UN STOP
	// por lo tanto no debo esperar que termine.

	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

}
//------------------------------------------------------------------------------------
void pvI2C_sendByte(u08 data)
{

	// Envia el data por el bus I2C.
	TWDR = data;
	// Habilita la trasmision y el envio de un ACK.
	TWCR = (1 << TWINT) | (1 << TWEN);

}
//------------------------------------------------------------------------------------
void pvI2C_readByte(u08 ackFlag)
{
	// begin receive over i2c
	if( ackFlag )
	{
		// ackFlag = TRUE: ACK the recevied data
		TWCR = (1 << TWEA) | (1 << TWINT) | (1 << TWEN);
	}
	else
	{
		// ackFlag = FALSE: NACK the recevied data
		TWCR = (1 << TWINT) | (1 << TWEN);
	}

}
//------------------------------------------------------------------------------------
u08 pvI2C_getStatus(void)
{
	// retieve current i2c status from i2c TWSR
	return( TWSR & 0xF8 );
}
//------------------------------------------------------------------------------------
s08 pvI2C_waitForComplete(void)
{

	// wait for i2c interface to complete operation
	while( !(TWCR & (1<<TWINT)) )
		;

	return(TRUE);
}
//------------------------------------------------------------------------------------
void pvI2C_disable(void)
{
	// Deshabilito la interfase TWI
	TWCR &= ~(1 << TWEN);

}
//------------------------------------------------------------------------------------
