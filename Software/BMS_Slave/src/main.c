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

//--------------CPU-FREQUENCY--------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//--Define CPU frequency, if not already defined in the platformio.ini or intellisense
#ifndef F_CPU
#define F_CPU 2000000L
#endif

//--------------USED-HARDWARE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//--Define Microcontroller, if not already defined in the platform.ini or intellisense
#ifndef __AVR_ATtiny261A__
#define __AVR_ATtiny261A__
#endif

//--------------PIN-DEFINITIONS------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
#define DEBUG_DDR DDRB
#define DEBUG_PORT PORTB
#define DEBUG_PIN PINB5 // PCINT13

//--------------SETTINGS-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
#define ADC_SAMPLES_V 4 // Averaging samples, 6 is max

//--------------BALANCING------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
#define BALANCING_DDR DDRB
#define BALANCING_PORT PORTB
#define BALANCING_PIN PINB4

#define START_BALANCING() BALANCING_PORT |= (1 << BALANCING_PIN)

#define STOP_BALANCING() BALANCING_PORT &= ~(1 << BALANCING_PIN)

//--------------LIBRARY-INCLUDES-----------------------------------------------------------------------------------------------------------------------------------------------------------------------//
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <util/delay.h>

//--------------SOURCE-FILES---------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
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
void eeprom_callibrate(uint8_t eeprom_stat);

//--------------MAIN-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
int main(void)
{
	bms_slave_init(); // Initiating the MCU, Registers configurated
	// Data received
	uint16_t bot_received = 0; // data received from the lower slave
	uint8_t address_received = 0;
	uint16_t top_received = 0; // data received from the upper slave

	// Send data
	uint16_t bot_send = 0;
	uint16_t top_send = 0;

	// Timing
	uint16_t COMM_time = 0;	   // compare value
	uint16_t BALANCE_time = 0; // compare value

	uint8_t ADCstat = MEASURE_VOLT;
	// 0  : Set up for Battery Temperature Measurement
	// 1  : Set up for Battery Voltage Measurement

	// Measurements
	uint16_t volt_raw = 0;

	int8_t battery_temperature; // battery_temperature = adc_value - 273; // K to degree C
	uint16_t battery_voltage;	// battery_voltage = (float)adc_value / 200; // divided by 1024 aka 10-bit, multiplied by 2,56 aka internal reference voltage * 2 (voltage divider)

	_delay_ms(500);
	uint8_t eeprom_stat = eeprom_read_byte(EEPROM_STATUS_ADR);
	_delay_ms(500);
	//--------------CALIBRATION----------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
	if (eeprom_stat != EEPROM_CALIBRATED) // if EEPROM not calibrated
	{
		eeprom_update_byte(EEPROM_STATUS_ADR, 0x00);
		// TEMP CALLIBRATION
		while (battery_temperature < 0) // Measure ambient temperature
		{
			battery_temperature = measure_temperature();
		}
		eeprom_update_word(EEPROM_temp_ADR, (uint16_t)battery_temperature);

		// VOLT CALLIBRATION
		for (int i = 0; i < 10; i++)
			while (battery_voltage == 0) // Measure SUPPLY voltage
			{
				battery_voltage = measure_voltage(6);
			}

		// 3V detection
		if ((battery_voltage <= CAL_VOLT_LT) && (battery_voltage >= CAL_VOLT_LB) && (!(eeprom_stat & EEPROM_STATUS_L))) // battery voltage in low borders and not calibrated yet
		{
			eeprom_update_word(EEPROM_3V_ADR, battery_voltage);
			if (!(eeprom_stat & EEPROM_STATUS_H)) // high voltage not calibrated yet
			{
				eeprom_update_byte(EEPROM_STATUS_ADR, EEPROM_STATUS_L); // set low voltage callibrated
			}
			else
			{
				eeprom_update_byte(EEPROM_STATUS_ADR, EEPROM_CALIBRATED); // set all callibrated
			}
			while (1)
			{
				_delay_ms(250);
				stat_led_green();
				_delay_ms(250);
				stat_led_red();
			}
		}
		// 4V detection
		else if ((battery_voltage >= CAL_VOLT_HB) && (battery_voltage <= CAL_VOLT_HT) && (!(eeprom_stat & EEPROM_STATUS_H))) // battery voltage in high borders and not calibrated yet
		{
			eeprom_update_word(EEPROM_4V_ADR, battery_voltage);
			if (!(eeprom_stat & EEPROM_STATUS_L)) // low voltage not calibrated yet
			{
				eeprom_update_byte(EEPROM_STATUS_ADR, EEPROM_STATUS_H); // set high voltage callibrated
			}
			else
			{
				eeprom_update_byte(EEPROM_STATUS_ADR, EEPROM_CALIBRATED); // set all callibrated
			}
			while (1)
			{
				_delay_ms(250);
				stat_led_green();
				_delay_ms(250);
				stat_led_off();
			}
		}
		// battery voltage out of predefined borders
		else
		{
			while (1)
			{
				_delay_ms(200);
				stat_led_red();
				_delay_ms(200);
				stat_led_off();
			}
		}
	}
	int8_t TEMP_D = (int8_t)eeprom_read_word(EEPROM_temp_ADR) - CAL_TEMP; // calculate temperature offset

	uint16_t voltage_h = eeprom_read_word(EEPROM_4V_ADR);
	uint16_t voltage_l = eeprom_read_word(EEPROM_3V_ADR);

	uint16_t VOLT_K = (CAL_VOLT_H_EXT - CAL_VOLT_L_EXT) / (voltage_h - voltage_l); // Multiplication factor for slope error

	uint16_t VOLT_D = CAL_VOLT_H_EXT - (voltage_h * VOLT_K); // Value to subtract from measurement to kill offset VOLT_D is x64

	// clear timers after startup
	timer_clear_timer(TIMER_COMM);
	timer_clear_timer(TIMER_BALANCE);

	while (1)
	{
		//--------------ADC------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
		timer_add_time(); // executed after max 32ms
		stat_led_off();
		COMM_time = timer_get_timer(TIMER_COMM);
		BALANCE_time = timer_get_timer(TIMER_BALANCE);

		if (!ADCstat)
		{
			volt_raw = measure_voltage(ADC_SAMPLES_V);
			battery_voltage = volt_raw * VOLT_K + VOLT_D;
			if (volt_raw) // make sure conversion is done
			{
				battery_voltage = volt_raw * VOLT_K + VOLT_D;
				ADCstat = MEASURE_TEMP;
			}
		}
		else
		{
			battery_temperature = measure_temperature() - TEMP_D;
			if (battery_temperature > -100) // make sure conversion is done
			{
				battery_temperature = volt_raw + TEMP_D;
				ADCstat = MEASURE_VOLT;
			}
		}
		//--------------BOT-PACKAGE-HANDLING-------------------------------------------------------------------------------------------------------------------------------------------------------------------//
		manch_init_receive(); // init receive from bot device

		bot_send = 0; // reset bot_send... status
		top_send = 0; // reset top_send... status

		if (manch_receive(&bot_received)) // if received from bot
		{
			if (bot_received & REQ_VOLT_G) // BOT-X // checking if battery voltage is requested
			{
				top_send = bot_received; // handout global command to next module
				bot_send = battery_voltage;
				bot_send |= (calc_parity(battery_voltage) << 14) | (1 << 15); // add parity to data
			}
			else if (bot_received & REQ_TEMP_G) // BOT-X // checking if battery temperature is requested
			{
				top_send = bot_received; // handout global command to next module
				bot_send = battery_temperature;
				bot_send |= (calc_parity(battery_temperature) << 14) | (1 << 15); // add parity to data
			}
			else if ((!(bot_received & ADDRESS_MASK)) && (bot_received & COM_BLC_A)) // checking if the current slave is addressed and command is balancing
			{
				START_BALANCING();
				stat_led_orange();
			}
			else // BOT_TOP // current module not addressed... pass through addressed command
			{
				address_received = (uint8_t)bot_received & ADDRESS_MASK;
				top_send = calc_data_bal(address_received - 1);
			}
		}
		//--------------TOP-PACKAGE-HANDLING-------------------------------------------------------------------------------------------------------------------------------------------------------------------//
		manch_init_receive1();				   // init receive from top device
		if (manch_receive(&top_received) == 1) // TOP-BOT // if received from top
		{
			bot_send = top_received;
		}

		//--------------COMMUNICATION--------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
		if (COMM_time >= 1)
		{
			timer_clear_timer(TIMER_COMM);
		}

		if (top_send) // BOT-TOP // if package for top ready
		{
			manch_init_send1();
			manch_send1(top_send);
		}

		if (bot_send) // X-BOT // if package for bot ready
		{
			manch_init_send();
			manch_send(bot_send);
		}

		// package received from top gets immediately send further down, no collision is expected
		manch_init_receive1();					// init receive from top device
		if (manch_receive1(&top_received) == 1) // TOP-BOT // receive from top
		{
			manch_init_send();
			manch_send(top_received);
		}

		//--------------BALANCING-TIMING-------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
		if (BALANCE_time >= 10000) // Balancing Timeout of 10 000 ms
		{
			STOP_BALANCING();
			timer_clear_timer(TIMER_BALANCE);
		}
	}
}

void bms_slave_init() // Combining all init functions
{
	// CPU frequency settings.
#if F_CPU == 4000000L
	CLKPR = 0x80;
	CLKPR = 0x01;

#elif F_CPU == 2000000L
	CLKPR = 0x80;
	CLKPR = 0x02;

#elif F_CPU == 1000000L
	CLKPR = 0x80;
	CLKPR = 0x04;

#else
#error Invalid prescaler setting.
#endif
	timer_init_timer();
	timer_add_time();
	ADC_init();
	stat_led_init(); // Status LED initialised
	BALANCING_DDR |= (1 << BALANCING_PIN);
	sei(); // global interrupt enable
}