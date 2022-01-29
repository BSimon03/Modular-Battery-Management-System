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

int measure_temperature()
{
    int temperature = 0;

    return temperature;
}

float measure_voltage()
{
    float voltage = 0;

    return voltage;
}