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

//--------------PIN-DEFINITIONS--------------------//
// PORTA
#define COMM_TOP PINA2

// PORTB
#define DEBUG_DDR DDRB
#define DEBUG_PORT PORTB
#define DEBUG_PIN PINB5 // PCINT13
#define COMM_BOT PINB6

//--------------SETTINGS---------------------------//
#define MIN_VOLTAGE 3 // Minimum voltage for the battery
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
#include "communication.h"
#include "timer.h"
//#include "manch_m.h"
#include "status.h"
#include "balancing.h"

// Data received
uint16_t bot_data = 0; // data received from the lower slave
uint16_t top_data = 0; // data received from the upper slave

uint16_t ADC_time = 0; // secs compare value

enum ADC_STAT
{
	MEASURE_VOLT,
	MEASURE_TEMP
};

uint8_t ADCstat = MEASURE_VOLT;
// 0  : Set up for Battery Temperature Measurement
// 1  : Set up for Battery Voltage Measurement

// Measurements
int8_t battery_temperature;
uint16_t battery_voltage_raw;
float battery_voltage = 0;
float battery_voltage_L = 0;
float battery_voltage_H = 0;

float voltage_k = 0;
float voltage_d = 0;
float temperature_d = 0;

int8_t buffer_battery_temperature;
uint16_t buffer_battery_voltage_raw;

uint8_t eeprom_stat = 0;

void bms_slave_init(void);

//--------------MAIN-------------------------------//
int main(void)
{
	timer_clear_timer(TIMER_MANCH);
	timer_clear_timer(TIMER_ADC);
	timer_clear_timer(TIMER_COM);

	eeprom_stat = eeprom_read_byte(EEPROM_STATUS_ADR);
	if (!(eeprom_stat & EEPROM_CALLIBRATED))
	{
		while (!battery_voltage_raw)
		{
			battery_voltage_raw = measure_voltage(ADC_SAMPLES);
		}
		battery_voltage = (float)battery_voltage_raw / 400; // divided by 1024 aka 10-bit, multiplied by 2,56 aka internal reference voltage

		if ((battery_voltage <= 3.3) && (!(eeprom_stat & EEPROM_STATUS_L)))
		{
			battery_voltage_L = battery_voltage;
			if (!(eeprom_stat & EEPROM_STATUS_H))
			{
				eeprom_write_float(EEPROM_3V_ADR, battery_voltage_L);
				eeprom_write_byte(EEPROM_STATUS_ADR, EEPROM_STATUS_L);
			}
			else
			{
				eeprom_stat = EEPROM_VOLT_RDY;
			}
		}
		else if ((battery_voltage >= 3.7) && (!(eeprom_stat & EEPROM_STATUS_H)))
		{
			battery_voltage_H = battery_voltage;
			if (!(eeprom_stat & EEPROM_STATUS_L))
			{
				eeprom_write_float(EEPROM_4V_ADR, battery_voltage_H);
				eeprom_write_byte(EEPROM_STATUS_ADR, EEPROM_STATUS_H);
			}
			else
			{
				eeprom_stat = EEPROM_VOLT_RDY;
			}
		}
		else
		{
			stat_led_red();
		}

		if (eeprom_stat & EEPROM_VOLT_RDY)
		{
			voltage_k = (CAL_VOLTAGE_H - CAL_VOLTAGE_L) / (battery_voltage_H - battery_voltage_L); // calculate voltage slope error
			voltage_d = CAL_VOLTAGE_H - (battery_voltage_H * voltage_k);						   // calculate voltage offset
			
				while(battery_temperature == -100) // make sure conversion is done
				{
					battery_temperature = measure_temperature(ADC_SAMPLES);
				}
			temperature_d = battery_temperature - CAL_TEMP;										   // calculate temperature offset
			ADC_callibrate(voltage_k, voltage_d, temperature_d);
		}
	}

	bms_slave_init(); // Initiating the MCU, Registers configurated

	//--------------ENDLESS-LOOP-----------------------//
	while (1)
	{
		ADC_time = timer_get_timer(TIMER_ADC);

		if (ADC_time >= 1)
		{
			timer_clear_timer(TIMER_ADC);
			timer_add_time();
			switch (ADCstat)
			{
			case MEASURE_TEMP:
				buffer_battery_temperature = measure_temperature(ADC_SAMPLES);
				if (buffer_battery_temperature >= -100) // make sure conversion is done
				{
					battery_temperature = buffer_battery_temperature;
					ADCstat = MEASURE_VOLT;
				}
				break;
			case MEASURE_VOLT:
				buffer_battery_voltage_raw = measure_voltage(ADC_SAMPLES);
				if (buffer_battery_voltage_raw)
				{
					battery_voltage_raw = buffer_battery_voltage_raw;
					ADCstat = 1;
				}
				break;
			}
		}
		//--------------ADC--------------------------------//
		//--------------COMMUNICATION----------------------//
		if (bot_data & ADDRESS_MASK) // checking if the current slave is addressed
		{
			bot_data -= 1;				  // count down address by 1
			bot_data ^= PARITY_BIT_COM_A; // Toggle the parity bit
		}
		//--------------BALANCING--------------------------//
		if (bot_data == COM_BLC_A)
		{
			start_balancing();
			stat_led_orange();
		}
	}
}

void bms_slave_init() // Combining all init functions
{
	CLKPR |= CLK_PS_SETTING; // Clock presescaler setting
	timer_init_timer();
	timer_add_time();
	ADC_init();
	ADC_get_cal();
	stat_led_init(); // Status LED initialised
	balancing_init();
	sei(); // global interrupt enable
}