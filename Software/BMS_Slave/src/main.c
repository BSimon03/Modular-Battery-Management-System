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

	//CPU frequency converted to prescaler bit settings.
#if F_CPU == 8000000L
#define CLK_PS_SETTING (1<<CLKPCE)							//PS = 1; 8 MHz

#elif F_CPU == 4000000L
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS0)				//PS = 2; 4 MHz

#elif F_CPU == 2000000L
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS1)				//PS = 4; 2 MHz

#elif F_CPU == 1000000L
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS1)|(1<<CLKPS0)	//PS = 8; 1MHz

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

//--------------PIN-DEFINITIONS--------------------//
	//PORTA
	#define COMM_TOP PINA2
	
	//PORTB
	#define DEBUG_PIN PINB5
	#define COMM_BOT PINB6

//--------------SETTINGS---------------------------//
#define MIN_VOLTAGE	3		//Minimum voltage for the battery
#define ADC_FILTER 1		//Enable ADC filtering  0:OFF  1:ON
#define ADC_SAMPLES 6 		//Averaging samples

//--------------LIBRARY-INCLUDES-------------------//
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

//--------------SOURCE-FILES-----------------------//
	//These are stored outside of the project folder, but will still be compiled
#include "ADC.h"
#include "communication.h"
#include "timer.h"
//#include "manch_m.h"
#include "status.h"
#include "balancing.h"

//Data received
uint16_t storedDATA = 0;

uint16_t ADC_time = 0; //secs compare value

uint8_t ADCstat = 0;
// 0  : Set up for Battery Temperature Measurement
// 1  : Set up for Battery Voltage Measurement

//Measurements
int8_t battery_temperature;
uint16_t battery_voltage_raw;

void init_bms_slave(void);

//--------------MAIN-------------------------------//
int main(void)
{
	init_bms_slave(); //Initiating the MCU, Registers configurated
	
	while (1)
	{
		//--------------ADC--------------------------------//
		ADC_time=timer_get_timer(TIMER_ADC);
		if(ADC_time>=1)
		{
			timer_clear_timer(TIMER_ADC);
			timer_add_time();
			if(ADCstat)
			{
				battery_temperature = measure_temperature(ADC_SAMPLES);
				if(battery_temperature<=-100)
					ADCstat=0;
				stat_led_orange();
			}
			else
			{
				battery_voltage_raw = measure_voltage(ADC_SAMPLES);
				if(battery_voltage_raw)
					ADCstat=1;
				stat_led_off();
			}
		}
		//--------------COMMUNICATION----------------------//

		//--------------BALANCING--------------------------//
		start_balancing();
		stop_balancing();
	}
}

/*ISR(INT0_vect) 		//Pin change interrupt set up for the chip-select pin
{
	if (!(PINB & 0x40))
	{
	}
}*/

void init_bms_slave()		//Combining all setup functions
{
	CLKPR |= CLK_PS_SETTING;	//Clock presescaler setting
	timer_init_timer();
	ADC_setup();
	ADC_get_cal();
	stat_led_init();			//Status LED initialised
	balancing_setup();
	sei(); //global interrupt enable
}