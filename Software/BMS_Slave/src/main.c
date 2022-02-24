/*********************************************/
/*  file:	main.c (V1.0)                    */
/*						                     */
/*  Diploma Thesis:                          */
/*   Battery Management System 2021/22       */
/* 						                     */
/*  brief:  Application for the Slaves       */
/* 						                     */
/*  Author: Simon Ball                       */
/*********************************************/


//--Define CPU frequency, if not already defined in the platform.ini or intellisense

#ifndef F_CPU
#define F_CPU 2000000L
#endif

//CPU frequency converted to prescaler bit settings.
#if F_CPU == 8000000L										//PS = 1
#define CLK_PS_SETTING (1<<CLKPCE)

#elif F_CPU == 4000000L										//PS = 2
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS0)

#elif F_CPU == 2000000L										//PS = 4
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS1)

#elif F_CPU == 1000000L										//PS = 8
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS1)|(1<<CLKPS0)

#else
#error Invalid prescaler setting.
#endif


//--Define Microcontroller, if not already defined in the platform.ini or intellisense

#ifndef __AVR_ATtiny261A__
#define __AVR_ATtiny261A__
#endif

//--Define main to be slave software

#ifndef BMS_SLAVE
#define BMS_SLAVE
#endif

//Pin definitions
	//PORTAS
	#define COMM_TOP PINA2
	#define STAT_R PINA5
	#define STAT_G PINA6
	#define ADC_VOLT PINA7       //ADC6 MUX5:0 000110       Connected to VCC through a 50:50 voltage divider
	
	//PORTB
	#define BALANCING PINB4
	#define DEBUG_PIN PINB5
	#define COMM_BOT PINB6

//Settings
#define MIN_VOLTAGE	3							//Minimum voltage for the battery
#define ADC_FILTER 1							//Enable ADC filtering
#define ADC_SAMPLES 6 		//Averaging x-2 samples

//Include Standard AVR libraries
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

//Include project specific source files
	//These are stored external... outside of the project folder, but will be compiled aswell
#include "ADC.h"
#include "communication.h"
#include "timer.h"
#include "manch_m.h"
#include "status.h"

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
int8_t battery_temperature;
uint16_t battery_voltage_raw;

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
				if(battery_temperature<=-100)
					ADCstat=0;
			}
			else
			{
				battery_voltage_raw = measure_voltage(ADC_SAMPLES);
				if(battery_voltage_raw)
				ADCstat=1;
			}
		}
	}
}

/*ISR(INT0_vect) 		//Pin change interrupt set up for the chip-select pin
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
}*/

/*ISR(TIMER1_COMPB_vect)	//Discharging OFF on compare match
{
	//PORTA &= ~(1 << DISCHARGE);
}
ISR(TIMER1_OVF_vect)	//Charge or Discharge ON
{
		secs++;
}*/

void init_bms_slave()					//Combining all setup functions
{
	stat_led_init();
	CLKPR |= CLK_PS_SETTING;
	ADC_setup();
	ADC_get_cal();
	sei(); //global interrupt enable
}