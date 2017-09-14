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

#include "Arduino.h"

#include <component/adc.h>
#include <adc2.h> // samg55 ASF sam/driver

#ifdef __cplusplus
extern "C" {
#endif

#define PIO_Configure pio_configure
#define PIO_GetOutputDataStatus pio_get_output_data_status
#define PIO_SetOutput pio_set_output
#define PIO_PullUp pio_pull_up
#define PIO_Set pio_set
#define PIO_Clear pio_clear
#define PIO_Get pio_get

#define TC_Start tc_start
#define TC_Stop tc_stop
#define TC_Configure tc_init
#define TC_SetRA tc_write_ra
#define TC_SetRC tc_write_rc
#define TC_ReadCV tc_read_cv
#define TC_GetStatus tc_get_status
#define TC_SetRB tc_write_rb

static int _readResolution = 10;
static int _writeResolution = 8;

void analogReadResolution(int res) {
	_readResolution = res;
}

void analogWriteResolution(int res) {
	_writeResolution = res;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
	if (from == to)
		return value;
	if (from > to)
		return value >> (from-to);
	else
		return value << (to-from);
}

eAnalogReference analog_reference = AR_DEFAULT;

void analogReference(eAnalogReference ulMode)
{
	analog_reference = ulMode;
}

/** The conversion data is done flag */
//volatile bool is_conversion_done = false;

/** The conversion data value */
//volatile uint32_t g_ul_value = 0;

/**
 * \brief ADC interrupt callback function.
 */
 /*
static void adc_end_conversion(void)
{
  g_ul_value = adc_channel_get_value(ADC, ADC_CHANNEL_1);
  is_conversion_done = true;
}
*/

void analogInputInit()
{
	// Initialize Analog Controller
	pmc_enable_periph_clk(ID_ADC);

	//adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
	//adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
	//adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
	//adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
	//adc_disable_all_channel(ADC);

//	ADC->ADC_MR |= ADC_MR_FREERUN_ON; // |
					 //ADC_MR_LOWRES_BITS_12;
	
//	ADC->ADC_EMR |= ADC_12_BITS;
	//adc_set_resolution(ADC, ADC_12_BITS);

/*
	adc_enable();
	#if SAMG55
	adc_select_clock_source_mck(ADC);
	#endif

	struct adc_config adc_cfg;

	adc_get_config_defaults(&adc_cfg);

	adc_init(ADC, &adc_cfg);
	adc_channel_enable(ADC, ADC_CHANNEL_0);

	adc_set_trigger(ADC, ADC_TRIG_TIO_CH_0);

	adc_set_callback(ADC, ADC_INTERRUPT_EOC_0, adc_end_conversion, 1);

	adc_set_resolution(ADC, ADC_12_BITS);
	adc_start_calibration(ADC);
	*/

	#if SAMG55
	adc_select_clock_source_mck(ADC);
	#endif

	struct adc_config adc_cfg;

	adc_get_config_defaults(&adc_cfg);

	adc_init(ADC, &adc_cfg);
}

#ifdef __SAMG55J19__
/**
 * \brief Enable the specified ADC channel.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param adc_ch ADC channel number.
 */
void adc_enable_channel(Adc *p_adc, const enum adc_channel_num adc_ch)
{
	p_adc->ADC_CHER = 1 << adc_ch;
}


/**
 * \brief Read the ADC channel status.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param adc_ch ADC channel number.
 *
 * \retval 1 if channel is enabled.
 * \retval 0 if channel is disabled.
 */
uint32_t adc_get_channel_status(const Adc *p_adc, const enum adc_channel_num adc_ch)
{
	return p_adc->ADC_CHSR & (1 << adc_ch);
}

/**
 * \brief Disable the specified ADC channel.
 *
 * \param p_adc Pointer to an ADC instance.
 * \param adc_ch ADC channel number.
 */
void adc_disable_channel(Adc *p_adc, const enum adc_channel_num adc_ch)
{
	p_adc->ADC_CHDR = 1 << adc_ch;
}

/**
 * \brief Start analog-to-digital conversion.
 *
 * \note If one of the hardware event is selected as ADC trigger,
 * this function can NOT start analog to digital conversion.
 *
 * \param p_adc Pointer to an ADC instance.
 */

void adc_start(Adc *p_adc)
{
	p_adc->ADC_CR = ADC_CR_START;
}

/**
 * \brief Get ADC interrupt and overrun error status.
 *
 * \param p_adc Pointer to an ADC instance.
 *
 * \return ADC status structure.
 */
uint32_t adc_get_status(const Adc *p_adc)
{
	return p_adc->ADC_ISR;
}
#endif //SAMG

uint32_t analogRead(uint32_t ulPin)
{
  uint32_t ulValue = 0;
  uint32_t ulChannel;

  if (ulPin < A0)
    ulPin += A0;

  ulChannel = g_APinDescription[ulPin].ulADCChannelNumber ;

#if defined __SAM3U4E__
	switch ( g_APinDescription[ulPin].ulAnalogChannel )
	{
		// Handling ADC 10 bits channels
		case ADC0 :
		case ADC1 :
		case ADC2 :
		case ADC3 :
		case ADC4 :
		case ADC5 :
		case ADC6 :
		case ADC7 :
			// Enable the corresponding channel
			adc_enable_channel( ADC, ulChannel );

			// Start the ADC
			adc_start( ADC );

			// Wait for end of conversion
			while ((adc_get_status(ADC) & ADC_SR_DRDY) != ADC_SR_DRDY)
				;

			// Read the value
			ulValue = adc_get_latest_value(ADC);
			ulValue = mapResolution(ulValue, 10, _readResolution);

			// Disable the corresponding channel
			adc_disable_channel( ADC, ulChannel );

			// Stop the ADC
			//      adc_stop( ADC ) ; // never do adc_stop() else we have to reconfigure the ADC each time
			break;

		// Handling ADC 12 bits channels
		case ADC8 :
		case ADC9 :
		case ADC10 :
		case ADC11 :
		case ADC12 :
		case ADC13 :
		case ADC14 :
		case ADC15 :
			// Enable the corresponding channel
			adc12b_enable_channel( ADC12B, ulChannel );

			// Start the ADC12B
			adc12b_start( ADC12B );

			// Wait for end of conversion
			while ((adc12b_get_status(ADC12B) & ADC12B_SR_DRDY) != ADC12B_SR_DRDY)
				;

			// Read the value
			ulValue = adc12b_get_latest_value(ADC12B) >> 2;
			ulValue = mapResolution(ulValue, 12, _readResolution);

			// Stop the ADC12B
			//      adc12_stop( ADC12B ) ; // never do adc12_stop() else we have to reconfigure the ADC12B each time

			// Disable the corresponding channel
			adc12b_disable_channel( ADC12B, ulChannel );
			break;
			

		// Compiler could yell because we don't handle DAC pins
		default :
			ulValue=0;
			break;
	}
#endif

#if defined __SAM3X8E__ || defined __SAM3X8H__ || defined __SAMG55J19__
	static uint32_t latestSelectedChannel = -1;
	switch ( g_APinDescription[ulPin].ulAnalogChannel )
	{
		// Handling ADC 12 bits channels
		case ADC0 :
		case ADC1 :
		case ADC2 :
		case ADC3 :
		case ADC4 :
		case ADC5 :
		case ADC6 :
		case ADC7 :
		//case ADC8 :
		//case ADC9 :
		//case ADC10 :
		//case ADC11 :

			// Enable the corresponding channel
			if (adc_get_channel_status(ADC, ulChannel) != 1) {
				adc_enable_channel( ADC, ulChannel );
				if ( latestSelectedChannel != (uint32_t)-1 && ulChannel != latestSelectedChannel)
					adc_disable_channel( ADC, latestSelectedChannel );
				latestSelectedChannel = ulChannel;
				g_pinStatus[ulPin] = (g_pinStatus[ulPin] & 0xF0) | PIN_STATUS_ANALOG;
			}

			// Trigger an ADC conversion
			adc_start( ADC );

			// Wait for end of conversion
			while ((adc_get_status(ADC) & ADC_ISR_DRDY) != ADC_ISR_DRDY)
				;

			// Read the value
			ulValue = adc_get_latest_value(ADC);
			ulValue = mapResolution(ulValue, ADC_RESOLUTION, _readResolution);

			break;

		// Compiler could yell because we don't handle DAC pins
		default :
			ulValue=0;
			break;
	}
#endif

	return ulValue;
}

static void TC_SetCMR_ChannelA(Tc *tc, uint32_t chan, uint32_t v)
{
	tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xFFF0FFFF) | v;
}

static void TC_SetCMR_ChannelB(Tc *tc, uint32_t chan, uint32_t v)
{
	tc->TC_CHANNEL[chan].TC_CMR = (tc->TC_CHANNEL[chan].TC_CMR & 0xF0FFFFFF) | v;
}

static uint8_t PWMEnabled = 0;
static uint8_t TCChanEnabled[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

void analogOutputInit(void) {
}

// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite(uint32_t ulPin, uint32_t ulValue) {
	uint32_t attr = g_APinDescription[ulPin].ulPinAttribute;
#if 0 //no DAC on samg55???
	if ((attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG) {
		EAnalogChannel channel = g_APinDescription[ulPin].ulADCChannelNumber;
		if (channel == DA0 || channel == DA1) {
			uint32_t chDACC = ((channel == DA0) ? 0 : 1);
			if (dacc_get_channel_status(DACC_INTERFACE) == 0) {
				/* Enable clock for DACC_INTERFACE */
				pmc_enable_periph_clk(DACC_INTERFACE_ID);

				/* Reset DACC registers */
				dacc_reset(DACC_INTERFACE);

				/* Half word transfer mode */
				dacc_set_transfer_mode(DACC_INTERFACE, 0);

				/* Power save:
				 * sleep mode  - 0 (disabled)
				 * fast wakeup - 0 (disabled)
				 */
				dacc_set_power_save(DACC_INTERFACE, 0, 0);
				/* Timing:
				 * refresh        - 0x08 (1024*8 dacc clocks)
				 * max speed mode -    0 (disabled)
				 * startup time   - 0x10 (1024 dacc clocks)
				 */
				dacc_set_timing(DACC_INTERFACE, 0x08, 0, 0x10);

				/* Set up analog current */
				dacc_set_analog_control(DACC_INTERFACE, DACC_ACR_IBCTLCH0(0x02) |
											DACC_ACR_IBCTLCH1(0x02) |
											DACC_ACR_IBCTLDACCORE(0x01));
			}

			/* Disable TAG and select output channel chDACC */
			dacc_set_channel_selection(DACC_INTERFACE, chDACC);

			if ((dacc_get_channel_status(DACC_INTERFACE) & (1 << chDACC)) == 0) {
				dacc_enable_channel(DACC_INTERFACE, chDACC);
			}

			// Write user value
			ulValue = mapResolution(ulValue, _writeResolution, DACC_RESOLUTION);
			dacc_write_conversion_data(DACC_INTERFACE, ulValue);
			while ((dacc_get_interrupt_status(DACC_INTERFACE) & DACC_ISR_EOC) == 0);
			return;
		}
	}

	if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM) {
		ulValue = mapResolution(ulValue, _writeResolution, PWM_RESOLUTION);

		if (!PWMEnabled) {
			// PWM Startup code
		    pmc_enable_periph_clk(PWM_INTERFACE_ID);
		    PWMC_ConfigureClocks(PWM_FREQUENCY * PWM_MAX_DUTY_CYCLE, 0, VARIANT_MCK);
			PWMEnabled = 1;
		}

		uint32_t chan = g_APinDescription[ulPin].ulPWMChannel;
		if ((g_pinStatus[ulPin] & 0xF) != PIN_STATUS_PWM) {
			// Setup PWM for this pin
			PIO_Configure(g_APinDescription[ulPin].pPort,
					g_APinDescription[ulPin].ulPinType,
					g_APinDescription[ulPin].ulPin,
					g_APinDescription[ulPin].ulPinConfiguration);
			PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, 0, 0);
			PWMC_SetPeriod(PWM_INTERFACE, chan, PWM_MAX_DUTY_CYCLE);
			PWMC_SetDutyCycle(PWM_INTERFACE, chan, ulValue);
			PWMC_EnableChannel(PWM_INTERFACE, chan);
			g_pinStatus[ulPin] = (g_pinStatus[ulPin] & 0xF0) | PIN_STATUS_PWM;
		}

		PWMC_SetDutyCycle(PWM_INTERFACE, chan, ulValue);
		return;
	}
#endif // no DAC. no PWM. only TIMER. all TIMER.
	if ((attr & PIN_ATTR_TIMER) == PIN_ATTR_TIMER) {
		// We use MCLK/2 as clock.
		const uint32_t TC = VARIANT_MCK / 2 / TC_FREQUENCY;

		// Map value to Timer ranges 0..255 => 0..TC
		ulValue = mapResolution(ulValue, _writeResolution, TC_RESOLUTION);
		ulValue = ulValue * TC;
		ulValue = ulValue / TC_MAX_DUTY_CYCLE;

		// Setup Timer for this pin
		ETCChannel channel = g_APinDescription[ulPin].ulTCChannel;
		static const uint32_t channelToChNo[] = { 0, 0, 1, 1, 2, 2, 0, 0, 1, 1, 2, 2, 0, 0, 1, 1, 2, 2 };
		static const uint32_t channelToAB[]   = { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 };
		static Tc *channelToTC[] = {
			TC0, TC0, TC0, TC0, TC0, TC0,
			TC1, TC1, TC1, TC1, TC1, TC1
			#ifdef TC2
			,TC2, TC2, TC2, TC2, TC2, TC2
			#endif
			};
		static const uint32_t channelToId[] = { 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8 };
		uint32_t chNo = channelToChNo[channel];
		uint32_t chA  = channelToAB[channel];
		Tc *chTC = channelToTC[channel];
		uint32_t interfaceID = channelToId[channel];

		if (!TCChanEnabled[interfaceID]) {
			pmc_enable_periph_clk(TC_INTERFACE_ID + interfaceID);
			TC_Configure(chTC, chNo,
				TC_CMR_TCCLKS_TIMER_CLOCK1 |
				TC_CMR_WAVE |         // Waveform mode
				TC_CMR_WAVSEL_UP_RC | // Counter running up and reset when equals to RC
				TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
				TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
				TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR);
			TC_SetRC(chTC, chNo, TC);
		}
		if (ulValue == 0) {
			if (chA)
				TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR);
			else
				TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR);
		} else {
			if (chA) {
				TC_SetRA(chTC, chNo, ulValue);
				TC_SetCMR_ChannelA(chTC, chNo, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
			} else {
				TC_SetRB(chTC, chNo, ulValue);
				TC_SetCMR_ChannelB(chTC, chNo, TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_SET);
			}
		}
		if ((g_pinStatus[ulPin] & 0xF) != PIN_STATUS_PWM) {
			PIO_Configure(g_APinDescription[ulPin].pPort,
					g_APinDescription[ulPin].ulPinType,
					g_APinDescription[ulPin].ulPin,
					g_APinDescription[ulPin].ulPinConfiguration);
			g_pinStatus[ulPin] = (g_pinStatus[ulPin] & 0xF0) | PIN_STATUS_PWM;
		}
		if (!TCChanEnabled[interfaceID]) {
			TC_Start(chTC, chNo);
			TCChanEnabled[interfaceID] = 1;
		}
		return;
	}

	// Defaults to digital write
	pinMode(ulPin, OUTPUT);
	ulValue = mapResolution(ulValue, _writeResolution, 8);
	if (ulValue < 128)
		digitalWrite(ulPin, LOW);
	else
		digitalWrite(ulPin, HIGH);
}

#ifdef __cplusplus
}
#endif
