
//          B M S _ S L A V E _ C A L L I B R A T I O N . C         //

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

//--------------CPU-FREQUENCY----------------------//
//--Define CPU frequency, if not already defined in the platformio.ini or intellisense
#ifndef F_CPU
#define F_CPU 2000000L
#endif

// CPU frequency converted to prescaler bit settings.
#if F_CPU == 8000000L
#define CLK_PS_SETTING (1 << CLKPCE) // PS = 1; 8 MHz

#elif F_CPU == 4000000L
#define CLK_PS_SETTING (1 << CLKPCE) | (1 << CLKPS0) // PS = 2; 4 MHz

#elif F_CPU == 2000000L
#define CLK_PS_SETTING (1 << CLKPCE) | (1 << CLKPS1) // PS = 4; 2 MHz

#elif F_CPU == 1000000L
#define CLK_PS_SETTING (1 << CLKPCE) | (1 << CLKPS1) | (1 << CLKPS0) // PS = 8; 1MHz

#else
#error Invalid prescaler setting.
#endif

//--------------USED-HARDWARE----------------------//
//--Define Microcontroller, if not already defined in the platform.ini or intellisense
#ifndef __AVR_ATtiny261A__
#define __AVR_ATtiny261A__
#endif

#ifndef BMS_SLAVE
#define BMS_SLAVE
#endif

//--------------PIN/PORT-DEFINITIONS--------------------//
#define DEBUG_DDR DDRB
#define DEBUG_PORT PORTB
#define DEBUG_PIN PINB5 // PCINT13

//--------------SETTINGS---------------------------//
#define ADC_FILTER 1  // Enable ADC filtering  0:OFF  1:ON
#define ADC_SAMPLES 6 // Averaging samples

//--------------LIBRARY-INCLUDES-------------------//
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

//--------------SOURCE-FILES-----------------------//
// These are stored outside of the project folder, but will still be compiled
#include "ADC.h"
#include "timer.h"
#include "status.h"

uint8_t ADC_FLAG = 0;

uint16_t ADC_time = 0; // secs compare value

enum CAL{
	MEASURE_L,
	MEASURE_H,	
	MEASURE_TEMP,
	CALC_CAL
};

uint8_t ADCstat = MEASURE_L;
// 0  : Set up for Battery Temperature Measurement
// 1  : Set up for Battery Voltage Measurement

// Measurements
int8_t battery_temperature;
uint16_t battery_voltage_raw;
float battery_voltage_L = 0;
float battery_voltage_H = 0;

void PCINT_setup(void);
void init_bms_slave(void);

//--------------MAIN-------------------------------//
int main(void)
{
	init_bms_slave(); // Initiating the MCU, Registers configurated

	while (1)
	{
		//--------------ADC--------------------------------//
		ADC_time = timer_get_timer(TIMER_ADC);
		if (ADC_FLAG)
		{
			if (ADC_time >= 1)
			{
				timer_clear_timer(TIMER_ADC);
				timer_add_time();
				switch(ADCstat)
				{
					case MEASURE_L:
						battery_voltage_raw = measure_voltage(ADC_SAMPLES);
						if (battery_voltage_raw)			//make sure conversion is done
						{
							battery_voltage_L = (float)battery_voltage_raw / 400; //divided by 1024 aka 10-bit, multiplied by 2,56 aka internal reference voltage
							ADCstat = MEASURE_H;
							ADC_FLAG = 0;
						}
						break;
					case MEASURE_H:
						battery_voltage_raw = measure_voltage(ADC_SAMPLES);
						if (battery_voltage_raw)			//make sure conversion is done
						{
							battery_voltage_H = (float)battery_voltage_raw / 400; //divided by 1024 aka 10-bit, multiplied by 2,56 aka internal reference voltage
							ADCstat = MEASURE_TEMP;
							ADC_FLAG = 0;
						}
						break;
					case MEASURE_TEMP:
						battery_temperature = measure_temperature(ADC_SAMPLES);
						if (battery_temperature >= -100)	//make sure conversion is done
						{
							ADCstat = CALC_CAL;
							ADC_FLAG = 0;
						}
						break;
					case CALC_CAL:
						VOLT_K=(CAL_VOLTAGE_H-CAL_VOLTAGE_L)/(battery_voltage_H-battery_voltage_L);		//calculate voltage slope error
						VOLT_D=CAL_VOLTAGE_H-(battery_voltage_H*VOLT_K);								//calculate voltage offset
						TEMP_D=battery_temperature-CAL_TEMP;											//calculate temperature offset
						ADC_callibrate();
						break;
				}
			}
		}
	}
}

ISR(PCINT_vect) 		// Pin change interrupt set up for the chip-select pin
{
	if (!(PINB & (1 << DEBUG_PIN))) 	// falling edge
	{
		ADC_FLAG = 1;
	}
}

void PCINT_setup() 		// Pinchange interrupt
{
	DEBUG_DDR &= ~(1<<DEBUG_PIN);	// DEBUG PIN set as input
	DEBUG_PORT |= (1<<DEBUG_PIN);	//Internal Pullup
	GIMSK |= (1 << PCIE1);	  		// General Interrupt Mask Register / pin change interrupt enable
	PCMSK1 |= (1 << PCINT13); 		// PCINT13 enabled
}

void init_bms_slave() 	// Combining all setup functions
{
	CLKPR |= CLK_PS_SETTING; 		// Clock presescaler setting
	PCINT_setup();
	timer_init_timer();
	ADC_setup();
	stat_led_init(); 				// Status LED initialised
	sei();			 				// global interrupt enable
}