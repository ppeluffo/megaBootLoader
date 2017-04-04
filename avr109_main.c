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
 *  ( lo hace el script makeBootLoader.sh )
 *
 *  avrdude -c avr109 -p m1284p -P /dev/ttyUSB0 -b 9600 -U flash:w:"/home/pablo/Spymovil/workspace/sp5KV5/Release/sp5KV5.hex":a
 *
 *  avrdude -c avr109 -p m1284p -P /dev/rfcomm0 -b 9600 -U flash:w:"/home/pablo/Spymovil/workspace/sp5KV5/Release/sp5KV5.hex":a -v
 *
 *  Grabacion en el DLG:
 *  El bootloader funciona correctamente si se trabaja por medio del puerto serial /dev/ttyUSBx.
 *  Cuando lo usamos a traves del BT no funciona bien.
 *
 *  Un problema que veo es que el avrdude luego de realizar una operacion manda un commando "E" que hace que
 *  el datalogger entre en modo normal. Cuando quiero dar un nuevo comando, ya no lo va a reconocer.
 *  Una alternativa es usar el minicom, resetear el dlg y salir del minicom con CTL-ALT-Q de modo de no enviar
 *  un comando de salida/reset.
 *
 *  Cuando doy el comando por BT de leer los fuses, funciona bien por BT.
 *  avrdude -c avr109 -p m1284p -P /dev/rfcomm1 -b 115200 -U lfuse:r:low_fuse_val.hex:h -U hfuse:r:high_fuse_val.hex:h -v -v -v -v
 *
 *  Cuando a continuacion doy el comando de grabar la memoria, ( por ttyUSB funciona bien ), no funciona por BT.
 *  Podemos pensar que hay un tema de troughtput, pero el problema es que no responde al primer comando.
 *  A [00][00]
 *  La prueba realizada es hacer un null-modem en el dlg entre el BT y el serial y probar con 2 minicoms transferir
 *  archivos.
 *  Se hace sin problema lo que significa que no habria problema de troughput del BT.
 *
 *
 */

#include "avr109.h"

void (*funcptr)( void ) = 0x0000; // Set up function pointer to RESET vector.
unsigned char BlockLoad(unsigned int size, unsigned char mem, ADDR_T *address);
void BlockRead(unsigned int size, unsigned char mem, ADDR_T *address);

//------------------------------------------------------------------------------------
int main(void)
{

u08 progSignal;

	cli();
	asm volatile ("clr __zero_reg__");
    MCUSR = 0;

	wdt_reset();
	wdt_disable();

	// El pin de control es entrada
	cbi(PROG_DDR, PROG_BIT);

	MCP_init();
	UART_init();

	/* Branch to bootloader or application code? */
	progSignal = ( ( PROG_PIN & _BV(PROG_BIT) ) >> PROG_BIT);
	pvb_sendchar('a');
	pvb_sendchar('v');
	pvb_sendchar('r');
	pvb_sendchar('1');
	pvb_sendchar('0');
	pvb_sendchar('9');
	pvb_sendchar('_');
	pvb_sendchar(0x30 + progSignal);
	pvb_sendchar('\r');
	pvb_sendchar('\n');

    if( progSignal == 1 ) // If PROGPIN is pulled high, enter programmingmode.
    {

    	pvb_sendchar('B');
    	pvb_sendchar('O');
    	pvb_sendchar('O');
    	pvb_sendchar('T');
    	pvb_sendchar('l');
    	pvb_sendchar('o');
    	pvb_sendchar('a');
    	pvb_sendchar('d');
    	pvb_sendchar('e');
    	pvb_sendchar('r');
    	pvb_sendchar(' ');
    	pvb_sendchar('R');
    	pvb_sendchar('0');
    	pvb_sendchar('1');
    	pvb_sendchar(' ');
    	pvb_sendchar('@');
    	pvb_sendchar(' ');
    	pvb_sendchar('2');
    	pvb_sendchar('0');
    	pvb_sendchar('1');
    	pvb_sendchar('7');
    	pvb_sendchar('/');
    	pvb_sendchar('0');
    	pvb_sendchar('4');
    	pvb_sendchar('/');
    	pvb_sendchar('0');
    	pvb_sendchar('4');
    	pvb_sendchar('\r');
    	pvb_sendchar('\n');

    	/* Main loop */
    	pvb_bootLoader_avr109();

    } else {
    	_WAIT_FOR_SPM();
    	_ENABLE_RWW_SECTION();
		sei();
    	funcptr(); // Jump to Reset vector 0x0000 in Application Section.
    }
}
//------------------------------------------------------------------------------------
void pvb_bootLoader_avr109(void)
{

unsigned char val;
unsigned int temp_int;
ADDR_T address;

	for(;;) {

		val = pvb_recchar(); // Wait for command character.

		// Check autoincrement status.
		if(val=='a')	{
			pvb_sendchar('Y'); // Yes, we do autoincrement.

		// Set address...
		} else if(val=='A')  {
			// NOTE: Flash addresses are given in words, not bytes.
			address=(pvb_recchar()<<8) | pvb_recchar(); // Read address high and low byte.
			pvb_sendchar('\r'); // Send OK back.

		// Chip erase.
		} else if(val=='e') {
			for(address = 0; address < APP_END;address += PAGESIZE)
			{ // NOTE: Here we use address as a byte-address, not word-address, for convenience.
				_WAIT_FOR_SPM();
				_PAGE_ERASE( address );

			}
			pvb_sendchar('\r'); // Send OK back.

		// Check block load support.
		}  else if(val=='b') {
			pvb_sendchar('Y'); // Report block load supported.
			pvb_sendchar((BLOCKSIZE>>8) & 0xFF); // MSB first.
			pvb_sendchar(BLOCKSIZE&0xFF); // Report BLOCKSIZE (bytes).

		// Start block load.
		} else if(val=='B') {
			temp_int = (pvb_recchar()<<8) | pvb_recchar(); // Get block size.
			val = pvb_recchar(); // Get memtype.
			pvb_sendchar( BlockLoad(temp_int,val,&address) ); // Block load.

		// Start block read.
		} else if(val=='g') {
			temp_int = (pvb_recchar()<<8) | pvb_recchar(); // Get block size.
			val = pvb_recchar(); // Get memtype
			BlockRead(temp_int,val,&address); // Block read

			// Read program memory.
		} else if(val=='R') {
			// Send high byte, then low byte of flash word.
			_WAIT_FOR_SPM();
			_ENABLE_RWW_SECTION();
			pvb_sendchar( _LOAD_PROGRAM_MEMORY( (address << 1)+1 ) );
			pvb_sendchar( _LOAD_PROGRAM_MEMORY( (address << 1)+0 ) );
			address++; // Auto-advance to next Flash word.

		// Write program memory, low byte.
		} else if(val=='c') {
			// NOTE: Always use this command before sending high byte.
			temp_int=pvb_recchar(); // Get low byte for later _FILL_TEMP_WORD.
			pvb_sendchar('\r'); // Send OK back.

		// Write program memory, high byte.
		} else if(val=='C') {
			temp_int |= ( pvb_recchar()<<8); // Get and insert high byte.
			_WAIT_FOR_SPM();
			_FILL_TEMP_WORD( (address << 1), temp_int ); // Convert word-address to byte-address and fill.
			address++; // Auto-advance to next Flash word.
			pvb_sendchar('\r'); // Send OK back.

		// Write page.
		}  else if(val== 'm') {
			if( address >= (APP_END>>1) ) // Protect bootloader area.
			{
				pvb_sendchar('?');
			} else {
				_WAIT_FOR_SPM();
				_PAGE_WRITE( address << 1 ); // Convert word-address to byte-address and write.
			}

			pvb_sendchar('\r'); // Send OK back.

		// Read fuse bits.
		} else if(val=='F') {
			_WAIT_FOR_SPM();
			pvb_sendchar( _GET_LOW_FUSES() );

		// Read high fuse bits.
		} else if(val=='N') {
			_WAIT_FOR_SPM();
			pvb_sendchar( _GET_HIGH_FUSES() );

		// Read extended fuse bits.
		} else if(val=='Q') {
			_WAIT_FOR_SPM();
			pvb_sendchar( _GET_EXTENDED_FUSES() );

		// Enter and leave programming mode.
		}  else if((val=='P')||(val=='L')) {
			pvb_sendchar('\r'); // Nothing special to do, just answer OK.

		// Exit bootloader.
		} else if(val=='E') {
			_WAIT_FOR_SPM();
			_ENABLE_RWW_SECTION();
			sei();
			pvb_sendchar('\r');
			funcptr(); // Jump to Reset vector 0x0000 in Application Section.
			//asm("jmp 0000");

		// Get programmer type.
		} else if (val=='p') {
			pvb_sendchar('S'); // Answer 'SERIAL'.

		// Return supported device codes.
		} else if(val=='t') {

#if PARTCODE+0 > 0
			pvb_sendchar( PARTCODE ); // Supports only this device, of course.
#endif /* PARTCODE */
			pvb_sendchar( 0 ); // Send list terminator.

		// Set LED, clear LED and set device type.
		} else if((val=='x')||(val=='y')||(val=='T')) {
			pvb_recchar(); // Ignore the command and it's parameter.
			pvb_sendchar('\r'); // Send OK back.

		// Return programmer identifier.
		} else if(val=='S') {
			pvb_sendchar('A'); // Return 'AVRBOOT'.
			pvb_sendchar('V'); // Software identifier (aka programmer signature) is always 7 characters.
			pvb_sendchar('R');
			pvb_sendchar('B');
			pvb_sendchar('O');
			pvb_sendchar('O');
			pvb_sendchar('T');

		// Return software version.
		} else if(val=='V') {
			pvb_sendchar('1');
			pvb_sendchar('5');

		// Return signature bytes.
		} else if(val=='s') {
			pvb_sendchar( SIGNATURE_BYTE_3 );
			pvb_sendchar( SIGNATURE_BYTE_2 );
			pvb_sendchar( SIGNATURE_BYTE_1 );

		// If not ESC, then it is unrecognized...
		}  else if(val!=0x1b)  {
			// The last command to accept is ESC (synchronization).
			pvb_sendchar('?');
		}

	}
}
//------------------------------------------------------------------------------------
unsigned char BlockLoad(unsigned int size, unsigned char mem, ADDR_T *address)
{
    unsigned int data;
    ADDR_T tempaddress;

    // EEPROM memory type.
    if(mem=='E')
    {
        return '?'; // Report programming OK
    }

    // Flash memory type.
	else if(mem=='F')
    { // NOTE: For flash programming, 'address' is given in words.
        (*address) <<= 1; // Convert address to bytes temporarily.
        tempaddress = (*address);  // Store address in page.

        do
		{
            data = pvb_recchar();
            data |= ( pvb_recchar() << 8);
            _FILL_TEMP_WORD(*address,data);
            (*address)+=2; // Select next word in memory.
            size -= 2; // Reduce number of bytes to write by two.
        } while(size); // Loop until all bytes written.

	_PAGE_WRITE(tempaddress);
	_WAIT_FOR_SPM();
	_ENABLE_RWW_SECTION();

        (*address) >>= 1; // Convert address back to Flash words again.
        return '\r'; // Report programming OK
    }

    // Invalid memory type?
    else
    {
        return '?';
    }
}
//------------------------------------------------------------------------------------
void BlockRead(unsigned int size, unsigned char mem, ADDR_T *address)
{
    // EEPROM memory type.
    if (mem=='E') // Read EEPROM
    {
        do
        {
            EEARL = *address; // Setup EEPROM address
            EEARH = ((*address) >> 8);
            (*address)++; // Select next EEPROM byte
            EECR |= (1<<EERE); // Read EEPROM
            pvb_sendchar(EEDR); // Transmit EEPROM dat ato PC

            size--; // Decrease number of bytes to read
        } while (size); // Repeat until all block has been read
    }

    // Flash memory type.
	else if(mem=='F')
	{
        (*address) <<= 1; // Convert address to bytes temporarily.

        do
        {
        	pvb_sendchar( _LOAD_PROGRAM_MEMORY(*address) );
        	pvb_sendchar( _LOAD_PROGRAM_MEMORY((*address)+1) );
            (*address) += 2; // Select next word in memory.
            size -= 2; // Subtract two bytes from number of bytes to read
        } while (size); // Repeat until all block has been read

        (*address) >>= 1; // Convert address back to Flash words again.
    }
}
//------------------------------------------------------------------------------------
