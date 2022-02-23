/*************************/
/*  main.cpp             */
/*  Diplomarbeit BMS     */
/*                       */
/*  Attiny261A Slave     */
/*  V1.0                 */
/*                       */
/*  Author: Simon Ball   */
/*************************/

#ifndef F_CPU
#define F_CPU 2000000L
#endif

#ifndef __AVR_ATtiny261A__
#define __AVR_ATtiny261A__
#endif

#define FILTER 1

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "../init_bms_slave.h"

#include "ADC.h"
#include "communication.h"
//#include "timer.h"
//#include "manch_m.h"
#include "status.h"


//Settings
#define ADC_SAMPLES 6 		//Averaging x-2 samples

//Tolerances read form eeprom
//only linear error correction... y=k*x+d
uint8_t adc_offset = 0;					//ADC offset
uint8_t adc_gain = 0;                   //ADC drift
uint8_t ntc_constant = 0;				//NTC Constant

//Data received
uint16_t storedDATA = 0;

//Interrupt
uint8_t secs = 0; //8bit: Overflow after 255 seconds, needs 1 second timer

uint32_t t = 0; //secs compare value

uint8_t ADCstat = 0;
// 0  : Set up for Battery Temperature Measurement
// 1  : Set up for Battery Voltage Measurement

//Measurements
uint16_t battery_temperature;
uint16_t battery_voltage;

void init_bms_slave(void);

int main(void)
{
	init_bms_slave(); //Initiating the MCU, Registers configurated

	
	while (1)
	{
		if((t+1)==secs)
		{
			if(ADCstat)
			{
				battery_temperature = measure_temperature(ADC_SAMPLES);
				ADCstat=0;
			}
			else
			{
				battery_voltage = measure_voltage(ADC_SAMPLES);
				ADCstat=1;
			}
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
	//PORTA &= ~(1 << DISCHARGE);
}
ISR(TIMER1_OVF_vect)	//Charge or Discharge ON
{
		secs++;
}

void init_bms_slave()					//Combining all setup functions
{
	stat_led_init();
	CLKPR |= CLK_PS_SETTING;
	ADC_setup();
	ADC_get_cal();
	sei(); //global interrupt enable
}