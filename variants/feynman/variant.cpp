/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Arduino.h>
#include "udc.h" // udc_start() For starting ASF USB Stack 
#include "wdt.h" //watchdog ASF driver
#include "flexcom.h"

// ----------------------------------------------------------------------------
/*
 * USART objects
 */
RingBuffer rx_buffer2;
RingBuffer tx_buffer2;

USARTClass Serial1(FLEXCOM6, USART6, FLEXCOM6_IRQn, ID_FLEXCOM6, &rx_buffer2, &tx_buffer2);
void serialEvent1() __attribute__((weak));
void serialEvent1() { }

// IT handlers
void FLEXCOM6_Handler(void)
{
  Serial1.IrqHandler();
}

// ----------------------------------------------------------------------------

void serialEventRun(void)
{
  if (Serial1.available()) serialEvent1();
}

// ----------------------------------------------------------------------------

extern "C" void __libc_init_array(void);
//extern void UrgentInit();

// ConfigurePin() is carried over from dc42 CoreNG firmware. 
void ConfigurePin(const PinDescription& pinDesc)
{
	pio_configure(pinDesc.pPort, pinDesc.ulPinType, pinDesc.ulPin, pinDesc.ulPinConfiguration);
}

void init( void )
{
	SystemInit();

	// Set Systick to 1ms interval, common to all SAM3 variants
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		// Capture error
		while (true);
	}
	
	wdt_disable(WDT);

	//UrgentInit();			// initialise anything in the main application that can't wait

	// Initialize C library (I think this calls C++ constructors for static data too)
	__libc_init_array();
	
	// Enable parallel access on PIO output data registers
	PIOA->PIO_OWER = 0xFFFFFFFF;
	PIOB->PIO_OWER = 0xFFFFFFFF;

	// We no longer disable pullups on all pins here, better to leave them enabled until the port is initialised
/*
	// Initialize Serial port U(S)ART pins
	ConfigurePin(g_APinDescription[APINS_UART0]);
	setPullup(APIN_UART0_RXD, true); 							// Enable pullup for RX0
	ConfigurePin(g_APinDescription[APINS_UART1]);
	setPullup(APIN_UART1_RXD, true); 							// Enable pullup for RX1

	// No need to initialize the USB pins on the SAM4E because they are USB by default
*/
	// Initialize ADC
	analogInputInit();
/*
	// Initialize analogOutput module
	AnalogOutInit();

	// Initialize HSMCI pins
	ConfigurePin(g_APinDescription[APIN_HSMCI_CLOCK]);
	ConfigurePin(g_APinDescription[APINS_HSMCI_DATA]);
	*/
	
	// Configure some UART pins
	ConfigurePin(g_APinDescription[42]);
	ConfigurePin(g_APinDescription[43]);

	// Start the USB
	udc_start();
}