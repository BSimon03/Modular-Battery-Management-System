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

void ADC_setup()
{
	// ADC5 : Temperature Sensor NTC
	ADCSRA |= (1 << ADEN);				  // ADC enabled
	ADCSRA |= (1 << ADPS2) | (1 << ADIE); // Clock Prescaler of 16, ADC Interrupt enabled

	ADMUX |= (1 << REFS1);	// Internal Reference Voltage 2.56V
	ADCSRB |= (1 << REFS2); // Internal Reference Voltage 2.56V

	// Result is right adjusted

	// ADATE is not enabled, which means we drive the ADC in Single Conversion Mode.
	// By setting ADSC (ADC Start Conversion) to a logic 1, the conversion is getting started.
	// Once the conversion is done, ADSC is cleared and the ADIF flag will be set.
	// When its completed the channel can safely be changed. The next conversion takes 25 clock cycles.
}

void ADC_get_cal()
{
	VOLT_K = eeprom_read_word(EEPROM_k_ADR);
	VOLT_D = eeprom_read_word(EEPROM_d_ADR);
	TEMP_D = eeprom_read_word(EEPROM_temp_ADR);
}

int8_t measure_temperature(uint8_t reps)
{
	static uint8_t state_t = ST_REGISTER;
	static uint16_t adc_values_t[8];
	static int8_t temperature;
	temperature = -100;
	static uint8_t adc_counter_t;
	static uint16_t adc_value = 0;
	static uint16_t sort_t; //sort_t algorithm
	adc_value = 0;
	
	switch (state_t)
	{
	case ST_REGISTER:
		ADMUX |= (1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3) | (1 << MUX4) | (1 << MUX5); // Attaching Channel 11 to the ADC... Temperature
		state_t = ST_MEASURE;
		adc_counter_t = 0;
		break;
	case ST_MEASURE:
		if (ADC_INTERRUPT)
		{
			if (adc_counter_t < reps)
			{
				adc_values_t[adc_counter_t] = 0;					 // current position in the array set to 0
				adc_values_t[adc_counter_t] |= ADCL;				 // save ADCL in the array
				adc_values_t[adc_counter_t] |= ((ADCH & 0x03) << 8); // save ADCH at the right position in the array
				adc_counter_t++;
			}
			else
			{
				state_t = ST_FILTER;
			}
		}
		break;
	case ST_FILTER:
		if (ADC_FILTER) // filters out the greatest and the smallest value measured for higher precision
		{
			// shifting the greatest value to the right
			for (adc_counter_t = 0; adc_counter_t <= reps; adc_counter_t++)
			{
				if (adc_values_t[adc_counter_t - 1] > adc_values_t[adc_counter_t])
				{
					sort_t = adc_values_t[adc_counter_t + 1];
					adc_values_t[adc_counter_t + 1] = adc_values_t[adc_counter_t];
					adc_values_t[adc_counter_t] = sort_t;
				}
			}

			// shifting the lowest value to the left
			for (adc_counter_t = reps; adc_counter_t >= 0; adc_counter_t--)
			{
				if (adc_values_t[adc_counter_t] < adc_values_t[adc_counter_t - 1])
				{
					sort_t = adc_values_t[adc_counter_t - 1];
					adc_values_t[adc_counter_t - 1] = adc_values_t[adc_counter_t];
					adc_values_t[adc_counter_t] = sort_t;
				}
			}

			// Adding all measured values to variable, except the outer ones
			adc_value = 0; // Resetting variable
			for (adc_counter_t = 1; adc_counter_t < (reps - 1); adc_counter_t++)
				adc_value += adc_values_t[adc_counter_t];
			adc_value /= (reps - 2);
		}
		else
		{
			// Adding all measured values to variable
			adc_value = 0; // Resetting variable
			for (adc_counter_t = 0; adc_counter_t < reps; adc_counter_t++)
				adc_value += adc_values_t[adc_counter_t];
			adc_value /= (reps);
		}
		temperature = adc_value - TEMP_D;
		state_t = ST_REGISTER;
		break;
	}
	return temperature;
}

uint16_t measure_voltage(uint8_t reps)
{
	static uint8_t state_v = ST_REGISTER;
	static uint16_t adc_values_v[8];
	static uint8_t adc_counter_v;
	static uint16_t adc_value = 0;
	static uint16_t sort_v; //sort_v algorithm
	adc_value = 0;

	switch (state_v)
	{
	case ST_REGISTER:
		ADMUX &= ~(1 << MUX0) | (1 << MUX3) | (1 << MUX4) | (1 << MUX5); // Clearing all important bits of the ADMUX register
		state_v = ST_MEASURE;
		adc_counter_v = 0;
		break;
	case ST_MEASURE:
		if (ADC_INTERRUPT)
		{
			if (adc_counter_v < reps)
			{
				adc_values_v[adc_counter_v] = 0;					 // current position in the array set to 0
				adc_values_v[adc_counter_v] |= ADCL;				 // save ADCL in the array
				adc_values_v[adc_counter_v] |= ((ADCH & 0x03) << 8); // save ADCH at the right position in the array
				adc_counter_v++;
			}
			else
			{
				state_v = ST_FILTER;
			}
		}
		break;
	case ST_FILTER:
		if (ADC_FILTER) // filters out the greatest and the smallest value measured for higher precision
		{
			// shifting the greatest value to the right
			for (adc_counter_v = 0; adc_counter_v <= reps; adc_counter_v++)
			{
				if (adc_values_v[adc_counter_v + 1] < adc_values_v[adc_counter_v])
				{
					sort_v = adc_values_v[adc_counter_v + 1];
					adc_values_v[adc_counter_v + 1] = adc_values_v[adc_counter_v];
					adc_values_v[adc_counter_v] = sort_v;
				}
			}

			// shifting the lowest value to the left
			for (adc_counter_v = reps; adc_counter_v > 0; adc_counter_v--)
			{
				if (adc_values_v[adc_counter_v] < adc_values_v[adc_counter_v - 1])
				{
					sort_v = adc_values_v[adc_counter_v - 1];
					adc_values_v[adc_counter_v - 1] = adc_values_v[adc_counter_v];
					adc_values_v[adc_counter_v] = sort_v;
				}
			}

			// Adding all measured values to variable, except the outer ones
			adc_value = 0; // Resetting variable
			for (adc_counter_v = 1; adc_counter_v < (reps - 1); adc_counter_v++)
				adc_value += adc_values_v[adc_counter_v];
			adc_value /= (reps - 2);
		}
		else
		{
			// Adding all measured values to variable
			adc_value = 0; // Resetting variable
			for (adc_counter_v = 0; adc_counter_v < reps; adc_counter_v++)
				adc_value += adc_values_v[adc_counter_v];
			adc_value /= (reps);
		}
		// voltage = (float)adc_value / 400; //divided by 1024 aka 10-bit, multiplied by 2,56 aka internal reference voltage
		state_v = ST_REGISTER;
		break;
	}
	return adc_value;
}