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
#define DEBUG_DDR DDRB
#define DEBUG_PORT PORTB
#define DEBUG_PIN PINB5 // PCINT13

//--------------SETTINGS---------------------------//
#define MIN_VOLTAGE 0x04B0 // Minimum voltage for the battery
#define ADC_FILTER 1	   // Enable ADC filtering  0:OFF  1:ON
#define ADC_SAMPLES 6	   // Averaging samples

//--------------BALANCING--------------------------//
#define BALANCING_DDR DDRB
#define BALANCING_PORT PORTB
#define BALANCING_PIN PINB4

#define START_BALANCING() BALANCING_PORT |= (1 << BALANCING_PIN)

#define STOP_BALANCING() BALANCING_PORT &= ~(1 << BALANCING_PIN)

//--------------LIBRARY-INCLUDES-------------------//
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>

//--------------SOURCE-FILES-----------------------//
// These are stored outside of the project folder, but will still be compiled
#include "ADC.h"
#include "communication.h"
#include "timer.h"
#include "manch_m.h"
#include "status.h"

enum ADC_STAT
{
	MEASURE_VOLT,
	MEASURE_TEMP
};

void bms_slave_init(void);

//--------------MAIN-------------------------------//
int main(void)
{
	// Data received
	uint16_t bot_received = 0; // data received from the lower slave
	uint8_t address_received = 0;
	uint16_t top_received = 0; // data received from the upper slave

	// Send data
	uint16_t bot_send = 0;
	uint16_t top_send = 0;

	// Timing
	uint16_t ADC_time = 0;	   // compare value
	uint16_t COMM_time = 0;	   // compare value
	uint16_t BALANCE_time = 0; // compare value

	uint8_t ADCstat = MEASURE_VOLT;
	// 0  : Set up for Battery Temperature Measurement
	// 1  : Set up for Battery Voltage Measurement

	// Measurements
	int8_t battery_temperature;
	int8_t buffer_battery_temperature;

	uint16_t battery_voltage;
	uint16_t buffer_battery_voltage;
	// battery_voltage = (float)battery_voltage / 400; // divided by 1024 aka 10-bit, multiplied by 2,56 aka internal reference voltage

	uint8_t eeprom_stat = 0;
	eeprom_stat = eeprom_read_byte(EEPROM_STATUS_ADR);
	//--------------CALIBRATION------------------------//
	if (!(eeprom_stat & EEPROM_CALIBRATED)) // if EEPROM not calibrated
	{
		while (!battery_voltage) // Measure SUPPLY voltage
		{
			battery_voltage = measure_voltage(ADC_SAMPLES);
		}
		while (battery_temperature == -100) // Measure ambient temperature
		{
			battery_temperature = measure_temperature(ADC_SAMPLES);
		}
		if (!(eeprom_stat & EEPROM_STATUS_TEMP)) // if neither 3V or 4V are measured
		{
			eeprom_write_word(EEPROM_3V_ADR, (uint16_t)battery_temperature);
		}
		if ((battery_voltage <= CAL_VOLTAGE_LB) && (!(eeprom_stat & EEPROM_STATUS_L))) // battery voltage smaller than lower max voltage and not calibrated yet
		{
			stat_led_green();
			eeprom_write_word(EEPROM_3V_ADR, battery_voltage);
			if (!(eeprom_stat & EEPROM_STATUS_H)) // high voltage not calibrated yet
				eeprom_write_byte(EEPROM_STATUS_ADR, EEPROM_STATUS_L);
			else
				eeprom_update_byte(EEPROM_STATUS_ADR, EEPROM_CALIBRATED);
		}
		else if ((battery_voltage >= CAL_VOLTAGE_HB) && (!(eeprom_stat & EEPROM_STATUS_H))) // battery voltage smaller than high min voltage and not calibrated yet
		{
			stat_led_orange();
			eeprom_write_word(EEPROM_4V_ADR, battery_voltage);
			if (!(eeprom_stat & EEPROM_STATUS_L)) // low voltage not calibrated yet
				eeprom_write_byte(EEPROM_STATUS_ADR, EEPROM_STATUS_H);
			else
				eeprom_update_byte(EEPROM_STATUS_ADR, EEPROM_CALIBRATED);
		}
		else
		{
			stat_led_red(); // battery voltage out of predefined borders
		}
	}

	bms_slave_init(); // Initiating the MCU, Registers configurated
	ADC_get_calibration();

	// clear timers after startup
	timer_clear_timer(TIMER_COMM);
	timer_clear_timer(TIMER_ADC);
	timer_clear_timer(TIMER_BALANCE);

	while (1)
	{
		//--------------ADC--------------------------------//
		timer_add_time(); // executed after max 32ms

		ADC_time = timer_get_timer(TIMER_ADC);
		COMM_time = timer_get_timer(TIMER_COMM);
		BALANCE_time = timer_get_timer(TIMER_BALANCE);

		if (ADC_time >= 1) // once every ms
		{
			timer_clear_timer(TIMER_ADC); // reset timer compare value

			switch (ADCstat)
			{
			case MEASURE_TEMP:
				buffer_battery_temperature = measure_temperature(ADC_SAMPLES);
				if (buffer_battery_temperature >= -100) // make sure conversion is done
				{
					battery_temperature = buffer_battery_temperature;
					ADCstat = MEASURE_VOLT;
					stat_led_green();
				}
				break;
			case MEASURE_VOLT:
				buffer_battery_voltage = measure_voltage(ADC_SAMPLES);
				if (buffer_battery_voltage) // make sure conversion is done
				{
					battery_voltage = buffer_battery_voltage;
					ADCstat = MEASURE_TEMP;
					stat_led_off();
				}
				break;
			}
		}
		//--------------PACKAGE-HANDLING-------------------//
		if ((!(bot_received & ADDRESS_MASK)) && (bot_received & COM_BLC_A)) // checking if the current slave is addressed and command is balancing
		{
			START_BALANCING();
			//stat_led_orange();
		}
		else if (bot_received & REQ_TEMP_G)
		{
			top_send = bot_received;
			bot_send = battery_temperature;
			bot_send |= (calc_parity(battery_temperature) << 14) | (1 << 15);
		}
		else if (bot_received & REQ_VOLT_G)
		{
			top_send = bot_received;
		}
		else
		{
			address_received = (uint8_t)bot_received & ADDRESS_MASK;
			top_send = calc_data_bal(address_received - 1);
		}
		//--------------COMMUNICATION----------------------//
		if (COMM_time >= 1)
		{
			timer_clear_timer(TIMER_COMM);
		}

		manch_init_receive();			  // init receive from bot device
		if (manch_receive(&bot_received)) // receive form bot
		{
			manch_init_send1();
			manch_send1(top_send);
		}

		manch_init_receive1();			   // init receive from top device
		if (manch_receive1(&top_received)) // receive form top
		{
			manch_init_send();
			manch_send(bot_send);
		}

		//--------------BALANCING-TIMING---------------------//
		if (BALANCE_time >= 10000) // Balancing Timeout of 10 000 ms
		{
			STOP_BALANCING();
			timer_clear_timer(TIMER_BALANCE);
		}
	}
}

void bms_slave_init() // Combining all init functions
{
	CLKPR |= CLK_PS_SETTING; // Clock presescaler setting
	timer_init_timer();
	timer_add_time();
	ADC_init();
	stat_led_init(); // Status LED initialised
	BALANCING_DDR |= (1 << BALANCING_PIN);
	sei(); // global interrupt enable
}