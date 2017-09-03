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

#ifndef _VARIANT_FEYNMAN_
#define _VARIANT_FEYNMAN_

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		12000000

/** Master clock frequency */
#define VARIANT_MCK			F_CPU

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "Arduino.h"

#ifdef __cplusplus
#include "UARTClass.h"
#include "USARTClass.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/**
 * Libc porting layers
 */
#if defined (  __GNUC__  ) /* GCC CS3 */
#    include <syscalls.h> /** RedHat Newlib minimal stub */
#endif

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

//FIXMELATER
// Number of pins defined in PinDescription array
//#define APINS_COUNT				(79u)
#define NUM_DIGITAL_PINS		48
//#define NUM_ANALOG_INPUTS		(12u)

#define digitalPinToPort(P)        ( g_APinDescription[P].pPort )
#define digitalPinToBitMask(P)     ( g_APinDescription[P].ulPin )
#define portOutputRegister(port)   ( &(port->PIO_ODSR) )
#define portInputRegister(port)    ( &(port->PIO_PDSR) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

// Interrupts
#define digitalPinToInterrupt(p)  ((p) < NUM_DIGITAL_PINS ? (p) : -1)

/*
 * Analog pins
 */
static const uint8_t A0  = 17;
static const uint8_t A1  = 18;
static const uint8_t A2  = 19;
static const uint8_t A3  = 20;
static const uint8_t A4  = 32;
static const uint8_t A5  = 33;
static const uint8_t A6  = 34;
static const uint8_t A7  = 35;
#define ADC_RESOLUTION		12

/*
 * TC
 */
#define TC_INTERFACE        TC0
#define TC_INTERFACE_ID     ID_TC0
#define TC_FREQUENCY        100000
#define TC_MAX_DUTY_CYCLE   255
#define TC_MIN_DUTY_CYCLE   0
#define TC_RESOLUTION		8

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define SPI_INTERFACE        SPI0
#define SPI_INTERFACE_ID     ID_FLEXCOM0
#define SPI_CHANNELS_NUM 2
#define PIN_SPI_SS0          (25u)//
#define PIN_SPI_SS1          (26u)//
#define PIN_SPI_MOSI         (10u)//
#define PIN_SPI_MISO         (9u)//
#define PIN_SPI_SCK          (32u)//
#define BOARD_SPI_SS0        PIN_SPI_SS0
#define BOARD_SPI_SS1        PIN_SPI_SS1
#define BOARD_SPI_DEFAULT_SS BOARD_SPI_SS0

#define BOARD_PIN_TO_SPI_PIN(x) (x==BOARD_SPI_SS0 ? PIN_SPI_SS0 : -1)
#define BOARD_PIN_TO_SPI_CHANNEL(x) (x==BOARD_SPI_SS0 ? 0 : -1)
//#define BOARD_PIN_TO_SPI_PIN(x) \
//	(x==BOARD_SPI_SS0 ? PIN_SPI_SS0 : \
//	(x==BOARD_SPI_SS1 ? PIN_SPI_SS1 : \
//	(x==BOARD_SPI_SS2 ? PIN_SPI_SS2 : PIN_SPI_SS3 )))
//#define BOARD_PIN_TO_SPI_CHANNEL(x) \
//	(x==BOARD_SPI_SS0 ? 0 : \
//	(x==BOARD_SPI_SS1 ? 1 : \
//	(x==BOARD_SPI_SS2 ? 2 : 3)))

static const uint8_t SS   = BOARD_SPI_SS0;
static const uint8_t SS1  = BOARD_SPI_SS1;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (PIO_PA3_IDX)
#define PIN_WIRE_SCL         (PIO_PA4_IDX)
#define FLEXCOM_INTERFACE    FLEXCOM3
#define WIRE_INTERFACE       TWI3
#define WIRE_INTERFACE_ID    ID_FLEXCOM3
#define WIRE_ISR_HANDLER     FLEXCOM3_Handler
#define WIRE_ISR_ID          FLEXCOM3_IRQn

static const uint8_t SDA  = PIN_WIRE_SDA;
static const uint8_t SCL  = PIN_WIRE_SCL;

#ifdef __cplusplus
}
#endif


/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/
#ifdef __cplusplus

extern USARTClass Serial1;

#endif

#endif /* _VARIANT_ARDUINO_DUE_X_ */
