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
float VOLT_K = 0;
int16_t VOLT_D = 0;
int16_t TEMP_D = 0;

uint8_t state = ST_REGISTER;
uint16_t adc_values[6];
uint8_t adc_counter;

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

	uint8_t status = eeprom_read_byte(EEPROM_STATUS_ADR);
	if (status & EEPROM_CALIBRATED)
	{
		uint16_t voltage_h = eeprom_read_word(EEPROM_4V_ADR);
		uint16_t voltage_l = eeprom_read_word(EEPROM_3V_ADR);
		int8_t temp_cal = (int8_t)eeprom_read_word(EEPROM_temp_ADR);
		VOLT_K = (CAL_VOLTAGE_H - CAL_VOLTAGE_L) / (voltage_h - voltage_l); // calculate voltage slope error
		VOLT_D = CAL_VOLTAGE_H - (voltage_h * VOLT_K);						// calculate voltage offset
		TEMP_D = temp_cal - CAL_TEMP;										// calculate temperature offset
	}
}

uint16_t ADC_measure(uint8_t conversions, uint8_t type)
{
	uint16_t adc_value = 0;
	uint16_t sort;

	switch (state)
	{
	case ST_REGISTER:
		if (type == 'v') // voltage measurement
		{
			ADMUX = 0x86;  // Internal Reference Voltage 2.56V, ADC attached to Channel 6 aka PA7
			ADCSRB = 0x01; // Internal Reference Voltage 2.56V
		}
		else // temperature measurement
		{
			ADMUX = 0xDF;  // Internal Reference Voltage 1.1V & Attaching Channel 11 to the ADC... Temperature
			ADCSRB = 0x0F; // clearing REFS2, mux 5 bit set
		}
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
				adc_values[adc_counter] = 0; // current position in the array set to 0
				adc_values[adc_counter] |= (ADCH << 8) | ADCL;
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
		if (ADC_FILTER) // filters out the greatest and the smallest value measured for higher precision
		{
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
			for (adc_counter = 1; adc_counter < (conversions - 1); adc_counter++)
			{
				adc_value += adc_values[adc_counter];
			}
			adc_value /= (conversions - 2);
		}
		else
		{
			// Adding all measured values to variable
			for (adc_counter = 0; adc_counter < conversions; adc_counter++)
			{
				adc_value += adc_values[adc_counter];
			}
			adc_value /= conversions;
			if (type == 'v')
			{
				adc_value = VOLT_K * adc_value + VOLT_D;
			}
			else
			{
				adc_value += TEMP_D;
			}
		}
		state = ST_REGISTER;
		break;
	}
	return adc_value;
}