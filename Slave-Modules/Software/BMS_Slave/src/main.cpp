/*************************/
/*  main.cpp             */
/*  Diplomarbeit BMS     */
/*                       */
/*  Attiny261A Slave     */
/*  V1.0                 */
/*                       */
/*  Author: Simon Ball   */
/*************************/

#include "init_ATtiny261A.h"

#define ADC_SAMPLES 6 		//Averaging x-2 samples

//Data received
uint8_t storedDATA = 0;

//Data to be sent
uint8_t information_string; //Bits[7:6] for indication and information, Bits[5:0] resistance of the battery cell
uint8_t value_string;		//Bits[7:0] capacitance of the battery cell

//Interrupt
uint8_t timer_counter = 0;
uint8_t adc_counter = ADC_SAMPLES;
uint32_t millis = 0; //32bit: Overflow after ~48 days

uint32_t t = 0; //millis compare value

//Status
uint8_t status = 0;
// 0  : Idle, waiting
// 1  : balance the cell to a start level: 3V
// 2  : charging until 4V with 1A
// 3  : charging until 4,2V with 500mA
// 4  : resistance measurement with 1 - 2A
// 5  : discharging until 3V with 1A
// 6  : charging until 3.7V
// 7  : Charging
// 8  : Discharging
// 9  : Error, temperature too high
// 10 : Error, low voltage
// 11 : Error, not suitable (res too high, cap too low)
// 12 : Error, timeout 10h
// 13 : Done with the measurement

uint8_t ADCstat = 0;
// 0  : Set up for Battery Temperature Measurement
// 1  : Set up for Battery Voltage Measurement

uint16_t adc_values[ADC_SAMPLES] = {0};

uint16_t adc_value = 0;
uint8_t battery_temperature = 0;
float battery_voltage = 0;

uint16_t sort; //sort algorithm

//Outcome of Measurements
uint8_t capacity = 0;
uint8_t resistance = 0;

int main(void)
{
	init_attiny261a(); //Initiating the MCU, Registers configurated

	sei(); //global interrupt enable

	while (1)
	{
		if (ADC_INTERRUPT) //ADC Interrupt ADIF is high
		{
			if (adc_counter >= ADC_SAMPLES)
			{
				//shifting the greatest value to the right
				for (adc_counter = 0; adc_counter <= ADC_SAMPLES; adc_counter++)
				{
					if (adc_values[adc_counter - 1] > adc_values[adc_counter])
					{
						sort = adc_values[adc_counter + 1];
						adc_values[adc_counter + 1] = adc_values[adc_counter];
						adc_values[adc_counter] = sort;
					}
				}

				//shifting the lowest value to the left
				for (adc_counter = ADC_SAMPLES; adc_counter >= 0; adc_counter--)
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
				for (adc_counter = 1; adc_counter < (ADC_SAMPLES - 1); adc_counter++)
					adc_value += adc_values[adc_counter];
				adc_value /= (ADC_SAMPLES - 2);
				adc_counter = 0;

				if (ADCstat) //Temperature
				{
					battery_temperature = (float)adc_value / TEMP_CONSTANT;
					ADMUX &= ~(1 << MUX2); //Clearing all important bits of the ADMUX register
					ADMUX |= (1 << MUX1);  //Attaching Channel 6 to the ADC... Battery
					ADCstat = 0;
				}
				else if (!ADCstat) //Battery
				{
					battery_voltage = (float)adc_value / 400; //divided by 1024 aka 10-bit, multiplied by 2,56 aka internal reference voltage
					ADMUX &= ~(1 << MUX1);					  //Clearing all important bits of the ADMUX register
					ADMUX |= (1 << MUX2);					  //Attaching Channel 5 to the ADC... Temperature
					ADCstat = 1;
				}
			}

			adc_values[adc_counter] = 0;
			adc_values[adc_counter] |= ADCL;
			adc_values[adc_counter] |= ((ADCH & 0x03) << 8);
			
			adc_counter++;
		}
		//Communication
		information_string = 0; //Reset string

		switch (status)
		{
		case 0: //Idle, waiting
			information_string |= idle;

			break;
		case 1: //balance the cell to a start level: 3V
			information_string |= processing;

			break;
		case 2: //charging until 4V with 1A
			information_string |= processing;

			break;
		case 3: //charging until 4,2V with 500mA
			information_string |= processing;

			break;
		case 4: //resistance measurement with 1 - 2A
			information_string |= processing;

			break;
		case 5: //discharging until 3V with 1A
			information_string |= processing;

			break;
		case 6: //charging until 3.7V
			information_string |= processing;

			break;
		case 7: //charging
			information_string |= processing;

			break;
		case 8: //Discharging
			information_string |= processing;

			break;
		case 9: //Error, temperature too high
			information_string |= error_high_temp;

			break;
		case 10: //Error, low voltage
			information_string |= error_low_volt;

			break;
		case 11: //Error, not suitable
			information_string |= error_not_suit;

			break;
		case 12: //Error, timeout 10h
			information_string |= error_timeout;

			break;
		case 13: //Done with the measurement
			information_string |= done;
			value_string = 0xFF;

			break;
		}
	}
}

ISR(INT0_vect) 		//Pin change interrupt set up for the chip-select pin
{
	if (!(PINB & 0x40))
	{	
		// If edge is falling, the command and index variables shall be initialized
		// and the 4-bit overflow counter of the USI communication shall be activated:
		USICR |= (1<<USIOIE);
		USISR |= (1<<USIOIF); // Clear Overflow bit
	}
	else
	{	
		// If edge is rising, turn the 4-bit overflow interrupt off:
		USICR &= ~(1<<USIOIE);
	}
}

ISR(USI_OVF_vect) 		// USI interrupt routine. Always executed when 4-bit overflows (after 16 clock edges = 8 clock cycles/ 8 bits)
{
	storedDATA = USIDR; 		// Read in from USIDR register
	switch (storedDATA)			// Switch-Case to respond according to the request from Master	
	{		
	case 0:					 	// If storedDATA is an empty string only the flag is being cleared
		USISR |= (1<<USIOIF); 		// Clear Overflow bit
		break;

	case request_info: 			// If the master requested information, the information string immediately gets sent back.
		USIDR |= information_string;
		USISR |= (1<<USIOIF);
		break;

	case request_secs: 			// If the master requested the second string, values are getting sent back.
		USIDR |= value_string;
		USISR |= (1<<USIOIF);
		break;
	}
}

ISR(TIMER1_COMPB_vect)	//Discharging OFF on compare match
{
	PORTA &= ~(1 << DISCHARGE);
}
ISR(TIMER1_OVF_vect)	//Charge or Discharge ON
{
	if (status == 2)
		PORTA |= (1 << DISCHARGE);
	timer_counter++;
	if (timer_counter == 10)
	{
		timer_counter = 0;
		millis++;
	}
}