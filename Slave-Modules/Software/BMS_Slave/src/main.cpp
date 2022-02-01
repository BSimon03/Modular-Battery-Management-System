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


uint8_t battery_temperature = 0;
float battery_voltage = 0;



//Outcome of Measurements
uint8_t capacity = 0;
uint8_t resistance = 0;

int main(void)
{
	init_attiny261a(); //Initiating the MCU, Registers configurated

	sei(); //global interrupt enable

	while (1)
	{
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