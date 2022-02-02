/*************************/
/*  ADC.c                */
/*						 */
/*  Battery Management   */
/*      System           */
/* 						 */
/*  Functions for analog */
/*  digital conversions  */
/*************************/

#include "ADC.h"

uint16_t adc_value = 0;
uint16_t sort; //sort algorithm

void ADC_setup()
{
	//ADC5 : Temperature Sensor NTC 
	ADCSRA|=(1<<ADEN);					//ADC enabled, 
	ADCSRA|=(1<<ADPS2)|(1<<ADIE);		//Clock Prescaler of 16, ADC Interrupt enabled
	
	ADMUX|=(1<<REFS1);									//Internal Reference Voltage 2.56V
	ADCSRB|=(1<<REFS2);									//Internal Reference Voltage 2.56V

	//Result is right adjusted
	
	//ADATE is not enabled, which means we drive the ADC in Single Conversion Mode.
	//By setting ADSC (ADC Start Conversion) to a logic 1, the conversion is getting started.
	//Once the conversion is done, ADSC is cleared and the ADIF flag will be set.
	//When its completed the channel can safely be changed. The next conversion takes 25 clock cycles.
}

uint16_t measure_temperature(uint8_t reps, uint8_t filter, float temp_constant)
{
	uint16_t adc_values[reps];
    uint16_t temperature = 0;
	uint8_t adc_counter=0;
	ADMUX |= (1 << MUX0)|(1 << MUX1)|(1 << MUX2)|(1 << MUX3)|(1 << MUX4)|(1 << MUX5);	//Attaching Channel 11 to the ADC... Temperature
	
	for(adc_counter = 0; adc_counter <= reps; adc_counter++)
		{
			while(!ADC_INTERRUPT);
			adc_values[adc_counter] = 0;						//current position in the array set to 0
			adc_values[adc_counter] |= ADCL;					//save ADCL in the array
			adc_values[adc_counter] |= ((ADCH & 0x03) << 8);	//save ADCH at the right position in the array
		}
		if(filter)		//filters out the greatest and the smallest value measured for higher precision
		{
				//shifting the greatest value to the right
				for (adc_counter = 0; adc_counter <= reps; adc_counter++)
				{
					if (adc_values[adc_counter - 1] > adc_values[adc_counter])
					{
						sort = adc_values[adc_counter + 1];
						adc_values[adc_counter + 1] = adc_values[adc_counter];
						adc_values[adc_counter] = sort;
					}
				}

				//shifting the lowest value to the left
				for (adc_counter = reps; adc_counter >= 0; adc_counter--)
				{
					if (adc_values[adc_counter] < adc_values[adc_counter - 1])
					{
						sort = adc_values[adc_counter - 1];
						adc_values[adc_counter - 1] = adc_values[adc_counter];
						adc_values[adc_counter] = sort;
					}
				}

				//Adding all measured values to variable, except the outer ones
				adc_value = 0; //Resetting variable
				for (adc_counter = 1; adc_counter < (reps - 1); adc_counter++)
					adc_value += adc_values[adc_counter];
				adc_value /= (reps - 2);
		}
		else
		{
			//Adding all measured values to variable
				adc_value = 0; //Resetting variable
				for (adc_counter = 0; adc_counter < reps; adc_counter++)
					adc_value += adc_values[adc_counter];
				adc_value /= (reps);
		}
	temperature = (float)adc_value / temp_constant;
    return temperature;
}

float measure_voltage(uint8_t reps, uint8_t filter, float adc_offset, float adc_drift)
{
	uint16_t adc_values[reps];
    float voltage = 0;
	uint8_t adc_counter=0;
	ADMUX &= ~(1 << MUX0)|(1 << MUX3)|(1 << MUX4)|(1 << MUX5); //Clearing all important bits of the ADMUX register
		for(adc_counter = 0; adc_counter <= reps; adc_counter++)
		{
			while(!ADC_INTERRUPT);
			adc_values[adc_counter] = 0;						//current position in the array set to 0
			adc_values[adc_counter] |= ADCL;					//save ADCL in the array
			adc_values[adc_counter] |= ((ADCH & 0x03) << 8);	//save ADCH at the right position in the array
		}
		if(filter)		//filters out the greatest and the smallest value measured for higher precision
		{
				//shifting the greatest value to the right
				for (adc_counter = 0; adc_counter <= reps; adc_counter++)
				{
					if (adc_values[adc_counter - 1] > adc_values[adc_counter])
					{
						sort = adc_values[adc_counter + 1];
						adc_values[adc_counter + 1] = adc_values[adc_counter];
						adc_values[adc_counter] = sort;
					}
				}

				//shifting the lowest value to the left
				for (adc_counter = reps; adc_counter >= 0; adc_counter--)
				{
					if (adc_values[adc_counter] < adc_values[adc_counter - 1])
					{
						sort = adc_values[adc_counter - 1];
						adc_values[adc_counter - 1] = adc_values[adc_counter];
						adc_values[adc_counter] = sort;
					}
				}

				//Adding all measured values to variable, except the outer ones
				adc_value = 0; //Resetting variable
				for (adc_counter = 1; adc_counter < (reps - 1); adc_counter++)
					adc_value += adc_values[adc_counter];
				adc_value /= (reps - 2);
		}
		else
		{
			//Adding all measured values to variable
				adc_value = 0; //Resetting variable
				for (adc_counter = 0; adc_counter < reps; adc_counter++)
					adc_value += adc_values[adc_counter];
				adc_value /= (reps);
		}
	voltage = (float)adc_value / 400; //divided by 1024 aka 10-bit, multiplied by 2,56 aka internal reference voltage
    return voltage;
}