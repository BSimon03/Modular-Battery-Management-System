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

//--------------USED-HARDWARE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//--Define Microcontroller, if not already defined in the platform.ini or intellisense
#ifndef __AVR_ATtiny261A__
#define __AVR_ATtiny261A__
#endif

#ifndef BMS_SLAVE
#define BMS_SLAVE
#endif

//--------------PIN-DEFINITIONS------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
#define DEBUG_DDR DDRB
#define DEBUG_PORT PORTB
#define DEBUG_PIN PINB5 // PCINT13

//--------------SETTINGS-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
#define ADC_SAMPLES_V 4	   // Averaging samples, 6 is max

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
	uint16_t adc_raw = 0;

	int8_t battery_temperature; // battery_temperature = adc_value - 273; // K to degree C
	uint16_t battery_voltage;	// battery_voltage = (float)adc_value / 200; // divided by 1024 aka 10-bit, multiplied by 2,56 aka internal reference voltage * 2 (voltage divider)

	uint8_t eeprom_stat = eeprom_read_byte(EEPROM_STATUS_ADR);
	//--------------CALIBRATION----------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
	if (!eeprom_stat & EEPROM_CALIBRATED) // if EEPROM not calibrated
	{
		while (!battery_voltage) // Measure SUPPLY voltage
		{
			battery_voltage = measure_voltage(ADC_SAMPLES_V);
		}
		while (battery_temperature == -100) // Measure ambient temperature
		{
			battery_temperature = measure_temperature();
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
		while (1)
			;
	}
	uint16_t voltage_h = eeprom_read_word(EEPROM_4V_ADR);
	uint16_t voltage_l = eeprom_read_word(EEPROM_3V_ADR);
	int8_t temp_cal = (int8_t)eeprom_read_word(EEPROM_temp_ADR);
	float VOLT_K = (CAL_VOLTAGE_H - CAL_VOLTAGE_L) / (voltage_h - voltage_l); // calculate voltage slope error
	int8_t VOLT_D = CAL_VOLTAGE_H - (voltage_h * VOLT_K);					  // calculate voltage offset
	int8_t TEMP_D = temp_cal - CAL_TEMP;									  // calculate temperature offset

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

		if(!ADCstat)
		{
			adc_raw = VOLT_K * measure_voltage(ADC_SAMPLES_V);
			if (adc_raw) // make sure conversion is done
			{
				battery_voltage = adc_raw + VOLT_D;
				ADCstat = MEASURE_TEMP;
				stat_led_off();
			}
		}
		else
		{
			adc_raw = measure_temperature();
			if (adc_raw) // make sure conversion is done
			{
				battery_temperature = adc_raw + TEMP_D;
				ADCstat = MEASURE_VOLT;
				stat_led_green();
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
		manch_init_receive1();			  // init receive from top device
		if (manch_receive(&top_received)==1) // TOP-BOT // if received from top
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
		manch_init_receive1();			   // init receive from top device
		if (manch_receive1(&top_received)==1) // TOP-BOT // receive from top
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
	CLKPR |= CLK_PS_SETTING; // Clock presescaler setting
	timer_init_timer();
	timer_add_time();
	ADC_init();
	stat_led_init(); // Status LED initialised
	BALANCING_DDR |= (1 << BALANCING_PIN);
	sei(); // global interrupt enable
}