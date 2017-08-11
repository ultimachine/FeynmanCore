/*
  main.cpp - Main loop for Arduino sketches
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#define ARDUINO_MAIN
#include "Arduino.h"

/*
 * Cortex-M3 Systick IT handler
 */
/*
extern void SysTick_Handler( void )
{
  // Increment tick count each ms
  TimeTick_Increment() ;
}
*/

#include "sam.h"


volatile uint32_t g_ul_ms_ticks = 0;
void SysTick_Handler(void)
{
	g_ul_ms_ticks++;
}

void mdelay(uint32_t ul_dly_ticks)
{
	uint32_t ul_cur_ticks;

	ul_cur_ticks = g_ul_ms_ticks;
	while ((g_ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks);
}

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

void init() __attribute__((weak));
void init() { }

/*
 * \brief Main entry point of Arduino application
 */
int main( void )
{
	// Initialize watchdog
	//watchdogSetup();

	SystemInit();
	// Set Systick to 1ms interval, common to all SAM3 variants
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		// Capture error
		while (1);
	}

	init();

	initVariant();

	//delay(1);

#if defined(USBCON)
	//USBDevice.attach();
#endif

	setup();

	for (;;)
	{
		loop();
		//if (serialEventRun) serialEventRun();
	}

	return 0;
}
