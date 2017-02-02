/*
 * MBL_main.c
 *
 *  Created on: 5 de dic. de 2016
 *      Author: pablo
 *
 *  Programa de bootloader para ATMEGA1284P.
 *  EL programa arranca y configura el puerto serial de la UART,
 *  el TWI para prender la UART / Bluetooth y el TIMER
 *  para controlar el timeout sin actividad.
 *  Luego queda a la espera de un comando.
 *  Si lo recibe lo ejecuta y reinicia el timer.
 *  Si llega el timeout, salta a ejecutar el programa de la aplicacion
 *  principal.
 *
 *  Basado en la application note AVR109.
 *  Lo probamos con el programa de PC AVRDUDE configurandole un programador
 *  tipo avr109.
 *
 *  El codigo para grabarlo hay que tener en cuenta donde comienza el bootloader del
 *  micro en cuestion.
 *  En el caso del ATmega1284P, comienza en 0x7000 ( ver doc, p297 ), pero esta dado en
 *  words, por lo que pasado a bytes queda en 0x1E000.
 *  Para que el linker genere el codigo correcto ( direcciones correctas ) debemos configurar
 *  en el Eclipse, en settings->linker->otherArguments debemos poner la linea -Ttext=0x1E000
 *  Luego esto se compila y al instalarlo, se graba en el area de boot.
 *
 *  Si quiero grabar esto junto con el programa ppal, debo hacer un cat de ambos en un archivo, y
 *  borrar el registro de fin de archivo que queda en el medio.
 *
 */

#include <MBL.h>

void (*funcptr)( void ) = 0x0000; // Set up function pointer to RESET vector.

//------------------------------------------------------------------------------------
int main(void)
{


	cli();
	asm volatile ("clr __zero_reg__");
    MCUSR = 0;

	wdt_reset();
	wdt_disable();

	MCP_init();
	UART_init();

	pvb_sendchar('B');
	pvb_sendchar('O');
	pvb_sendchar('O');
	pvb_sendchar('T');
	pvb_sendchar('0');
	pvb_sendchar('7');
	pvb_sendchar('\r');
	pvb_sendchar('\n');

	while(1) {

		// Si recibi un caracter, salgo
		if  ( UART_STATUS_REG & (1 << RECEIVE_COMPLETE_BIT) ) {
			break;
		}

		// Si expiro el tiempo, salgo a la aplicacion
 //   	sei();
 //       funcptr(); // Jump to Reset vector 0x0000 in Application Section.
        //asm("jmp 0000");

	}

	pvb_bootLoader_avr109();


}
//------------------------------------------------------------------------------------
// FUNCION DE AUTO PROGRAMACION AVR109.
//------------------------------------------------------------------------------------
void pvb_bootLoader_avr109(void)
{
unsigned char val;

   /* Main loop */
	for(;;) {

		val = pvb_recchar(); // Wait for command character.

        // Exit bootloader.
        if(val=='E') {
        	pvb_sendchar('\r');
        	sei();
            funcptr(); // Jump to Reset vector 0x0000 in Application Section.
            //asm("jmp 0000");
        }

		// Check autoincrement status.
		val++;
		pvb_sendchar(val);

	}

}
//------------------------------------------------------------------------------------

