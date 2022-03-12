/*********************************************/
/*  file:   ADC.c                            */
/*						                     */
/*  Diploma Thesis:                          */
/*   Battery Management System 2021/22       */
/* 						                     */
/*  brief:  Functions for analog measurement */
/* 						                     */
/*  Author: Simon Ball                       */
/*********************************************/

#include <avr/io.h>
#include <stdint.h>
#include <avr/eeprom.h>
#include "ADC.h"

// EEPROM
static uint16_t VOLT_K = 0;
static uint16_t VOLT_D = 0;
static uint16_t TEMP_D = 0;

static uint8_t state = ST_REGISTER;
static uint16_t adc_values[6];
static uint8_t adc_counter;
static uint16_t adc_value = 0;
static uint16_t sort; // sort algorithm

void ADC_init()
{
	// ADC5 : Temperature Sensor NTC
	ADCSRA |= (1 << ADEN);	// ADC enabled
	ADCSRA |= (1 << ADPS2); // Clock Prescaler of 16

	// Result is right adjusted

	// Single Conversion mode
	//  ADATE is not enabled, which means we drive the ADC in Single Conversion Mode.
	//  By setting ADSC (ADC Start Conversion) to a logic 1, the conversion is getting started.
	//  Once the conversion is done, ADSC is cleared and the ADIF flag will be set.
	//  When its completed the channel can safely be changed. The next conversion takes 25 clock cycles.
	//  ADIE is not set as ADIF gets set when the conversion is done
	//  ADIF must be written to 1 in order to clear it
}

int8_t measure_temperature(uint8_t conversions)
{
	int8_t temperature = -100;
	adc_value = 0;

	switch (state)
	{
	case ST_REGISTER:
		ADMUX = 0xC0;																  // Internal Reference Voltage 1.1V
		ADCSRB = 0x0F;																  // clearing REFS2, mux 5 bit set
		ADMUX |= (1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3) | (1 << MUX4); // Attaching Channel 11 to the ADC... Temperature
		state = ST_MEASURE;
		adc_counter = 0;
		ADC_START_CONVERSION();
		break;
	case ST_MEASURE:
		if (ADC_INTERRUPT)
		{
			ADC_CLEAR_INT();
			if (adc_counter < conversions)
			{
				adc_values[adc_counter] = 0;					 // current position in the array set to 0
				adc_values[adc_counter] |= ADCL;				 // save ADCL in the array
				adc_values[adc_counter] |= ((ADCH & 0x03) << 8); // save ADCH at the right position in the array
				adc_counter++;
				ADC_START_CONVERSION();
			}
			else
			{
				state = ST_FILTER;
			}
		}
		break;
	case ST_FILTER:
		#if ADC_FILTER == 1 // filters out the greatest and the smallest value measured for higher precision
		
			// shifting the greatest value to the right
			for (adc_counter = 0; adc_counter <= conversions; adc_counter++)
			{
				if (adc_values[adc_counter - 1] > adc_values[adc_counter])
				{
					sort = adc_values[adc_counter + 1];
					adc_values[adc_counter + 1] = adc_values[adc_counter];
					adc_values[adc_counter] = sort;
				}
			}

			// shifting the lowest value to the left
			for (adc_counter = conversions; adc_counter >= 0; adc_counter--)
			{
				if (adc_values[adc_counter] < adc_values[adc_counter - 1])
				{
					sort = adc_values[adc_counter - 1];
					adc_values[adc_counter - 1] = adc_values[adc_counter];
					adc_values[adc_counter] = sort;
				}
			}

			// Adding all measured values to variable, except the outer ones
			adc_value = 0; // Resetting variable
			for (adc_counter = 1; adc_counter < (conversions - 1); adc_counter++)
				adc_value += adc_values[adc_counter];
			adc_value /= (conversions - 2);
		#else
			// Adding all measured values to variable
			adc_value = 0; // Resetting variable
			for (adc_counter = 0; adc_counter < conversions; adc_counter++)
				adc_value += adc_values[adc_counter];
			adc_value /= (conversions);
		#endif
				temperature = adc_value - TEMP_D;
		state = ST_REGISTER;
		break;
	}
	return temperature;
}

uint16_t measure_voltage(uint8_t conversions)
{
	adc_value = 0;

	switch (state)
	{
	case ST_REGISTER:
		ADMUX = 0x86;			// Internal Reference Voltage 2.56V, ADC attached to Channel 6 aka PA7
		ADCSRB |= (1 << REFS2); // Internal Reference Voltage 2.56V
		state = ST_MEASURE;
		adc_counter = 0;
		ADC_START_CONVERSION();
		break;
	case ST_MEASURE:
		if (ADC_INTERRUPT)
		{
			ADC_CLEAR_INT();
			if (adc_counter < conversions)
			{
				adc_values[adc_counter] = 0;					 // current position in the array set to 0
				adc_values[adc_counter] |= ADCL;				 // save ADCL in the array
				adc_values[adc_counter] |= ((ADCH & 0x03) << 8); // save ADCH at the right position in the array
				adc_counter++;
				ADC_START_CONVERSION();
			}
			else
			{
				state = ST_FILTER;
			}
		}
		break;
	case ST_FILTER:
#if ADC_FILTER == 1 // filters out the greatest and the smallest value measured for higher precision
		// shifting the greatest value to the right
		for (adc_counter = 0; adc_counter <= conversions; adc_counter++)
		{
			if (adc_values[adc_counter + 1] < adc_values[adc_counter])
			{
				sort = adc_values[adc_counter + 1];
				adc_values[adc_counter + 1] = adc_values[adc_counter];
				adc_values[adc_counter] = sort;
			}
		}

		// shifting the lowest value to the left
		for (adc_counter = conversions; adc_counter > 0; adc_counter--)
		{
			if (adc_values[adc_counter] < adc_values[adc_counter - 1])
			{
				sort = adc_values[adc_counter - 1];
				adc_values[adc_counter - 1] = adc_values[adc_counter];
				adc_values[adc_counter] = sort;
			}
		}

		// Adding all measured values to variable, except the outer ones
		adc_value = 0; // Resetting variable
		for (adc_counter = 1; adc_counter < (conversions - 1); adc_counter++)
			adc_value += adc_values[adc_counter];
		adc_value /= (conversions - 2);
#else
		// Adding all measured values to variable
		adc_value = 0; // Resetting variable
		for (adc_counter = 0; adc_counter < conversions; adc_counter++)
			adc_value += adc_values[adc_counter];
		adc_value /= (conversions);
		// voltage = (float)adc_value / 400; //divided by 1024 aka 10-bit, multiplied by 2,56 aka internal reference voltage		
#endif
state = ST_REGISTER;
break;
	}
	return adc_value;
}