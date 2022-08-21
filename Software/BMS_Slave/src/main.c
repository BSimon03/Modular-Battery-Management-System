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
error! #define F_CPU 2000000L
#endif

//--------------USED-HARDWARE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//--Define Microcontroller, if not already defined in the platform.ini or intellisense
#ifndef __AVR_ATtiny261A__
error! #define __AVR_ATtiny261A__
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

enum COMM_STAT
{
	COMM_RECEIVE,
	COMM_GLOBAL,
	COMM_ADD,
	COMM_SEND
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

	uint8_t comm_stat_b = 0; // Status of the communication to the lower slave
	uint8_t comm_stat_t = 0; // Status of the communication to the upper slave

	// Communication Cycle
	uint8_t comm_stat = COMM_RECEIVE;

	// Timing
	uint16_t BALANCE_time = 0; // compare value

	uint8_t ADCstat = MEASURE_VOLT;
	// 0  : Set up for Battery Temperature Measurement
	// 1  : Set up for Battery Voltage Measurement

	// Measurements
	uint16_t volt_raw = 0;

	int8_t battery_temperature; // battery_temperature = adc_value - 273; // K to degree C
	uint16_t battery_voltage;	// battery_voltage = (float)adc_value / 200; // divided by 1024 aka 10-bit, multiplied by 2,56 aka internal reference voltage * 2 (voltage divider)

/*
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
*/	

	int8_t TEMP_D = (int8_t)eeprom_read_word(EEPROM_temp_ADR) - CAL_TEMP; // calculate temperature offset

	uint16_t voltage_h = eeprom_read_word(EEPROM_4V_ADR);
	uint16_t voltage_l = eeprom_read_word(EEPROM_3V_ADR);

	uint16_t VOLT_K = (CAL_VOLT_H_EXT - CAL_VOLT_L_EXT) / (voltage_h - voltage_l); // Multiplication factor for slope error

	uint16_t VOLT_D = CAL_VOLT_H_EXT - (voltage_h * VOLT_K); // Value to subtract from measurement to kill offset VOLT_D is x64

	// clear timers after startup
//	timer_clear_timer(TIMER_BALANCE);
unsigned char com_stat, state=0;
DDRA |= 0x80;
	while (1)
	{
		//--------------ADC------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//		timer_add_time(); // executed after max 32ms
//		stat_led_off();
//		_delay_ms(500);
//		stat_led_green();
//		manch_init_send();
//		manch_send();
//		_delay_ms(800);

		//============================Recieve Test==========================
		if(state==0)		//empfangen initialisieren
		{
			manch_init_receive();
			com_stat=0;
			state++;
		}
		
		else if(state==1)		//Daten Empfangen
		{
			com_stat=manch_receive();
			
			if (com_stat==0)		//während auf Daten gewartet wird LED orange blinken
			{
				stat_led_off();
			}
			
			else if(com_stat==1)			//wenn Daten erfolgreich Empfangen wurden LED grün
			{
				stat_led_green();
				_delay_ms(200);
				state=0;
			}
			else if (com_stat==2)		//wenn Fehler beim Empfangen LED rot
			{
				stat_led_red();
				_delay_ms(100);
				state=0;
			}
			else if (com_stat==3)		//wenn Fehler beim Empfangen LED rot
			{
				stat_led_red();
				_delay_ms(100);
				stat_led_off();
				_delay_ms(100);
				stat_led_red();
				_delay_ms(100);
				state=0;
			}
			else if (com_stat==4)		//wenn Fehler beim Empfangen LED rot
			{
				stat_led_red();
				_delay_ms(100);
				stat_led_off();
				_delay_ms(100);
				stat_led_red();
				_delay_ms(100);
				stat_led_off();
				_delay_ms(100);
				stat_led_red();
				_delay_ms(100);
				state=0;
			}
		}
/*		else if (state==2)	// antworten
		{
			stat_led_red();
			manch_init_send();
			manch_send();
			state = 3;
		}
		else if (state==3)	// warten, bis fertig gesendet
		{
			if (manch_res == 1)
			{
				state = 0;
			}
		}
*/
/*
		BALANCE_time = timer_get_timer(TIMER_BALANCE);

		if (!ADCstat)
		{
			volt_raw = measure_voltage(ADC_SAMPLES_V);
			if (volt_raw) // make sure conversion is done
			{
				battery_voltage = volt_raw * VOLT_K + VOLT_D;
				ADCstat = MEASURE_TEMP;
			}
		}
		else
		{
			battery_temperature = measure_temperature();
			if (battery_temperature > -100) // make sure conversion is done
			{
				battery_temperature = volt_raw - TEMP_D;
				ADCstat = MEASURE_VOLT;
			}
		}
		//--------------PACKAGE-HANDLING-------------------------------------------------------------------------------------------------------------------------------------------------------------------//

		// 0 is top, 1 is bottom

		switch (comm_stat)
		{
		case COMM_RECEIVE: // receiving data from bottom board
			comm_stat_b = manch_receive1(&bot_received);
//			comm_stat_t = manch_receive(&top_received);
			if (comm_stat_b == 1) // if data received from bot
			{
				if (bot_received & 0x40) // if command is Global
				{
					comm_stat = COMM_GLOBAL;
				}
				else
				{
					comm_stat = COMM_ADD;
				}
			}
			else if (comm_stat_b == 2) // if error
			{
				manch_init_receive1();
			}

			if (comm_stat_t == 1) // if data received from top
			{
				bot_send = top_received;
				top_send = 0x00;
				manch_init_send1();
				manch_send1(bot_send);
				manch_init_send();
				manch_send(top_send);
			}
			else if (comm_stat_t == 2) // if error
			{
				manch_init_receive();
			}

			break;
		case COMM_GLOBAL:					// handling global commands
			top_send = bot_received; 		// prepare global command for next module

			if (bot_received & REQ_VOLT_G) 	// checking if battery voltage is requested
			{
				bot_send = battery_voltage >> 6;
				bot_send |= (calc_parity(battery_voltage) << 14) | (1 << 15); // add parity to data
			}
			else if (bot_received & REQ_TEMP_G) // checking if battery temperature is requested
			{
				bot_send = (uint16_t)battery_temperature >> 6;
				bot_send |= (calc_parity(battery_temperature) << 14) | (1 << 15); // add parity to data
			}
			else if (*bot_received & COM_SLP_G) // sleep command received
			{
				// Standby mode ATtiny261A
				bot_send = 0x00;	// send nothing, high active bus
				set_sleep_mode(SLEEP_MODE_STANDBY);
				sleep_enable();
				sleep_mode();
				sleep_disable();
				// Power down mode ATtiny261A
				//set_sleep_mode(SLEEP_MODE_PWR_DOWN);
				//sleep_enable();
				//sleep_mode();
				//sleep_disable();
			}
			
			manch_init_send1();				// initialize send to bot module
			manch_send1(bot_send);			// send data to bot module
			manch_init_send();				// initialize send to top board
			manch_send(top_send);			// send global command to top board		
			break;
*/
/*
		case COMM_ADD:															  // handling addressed commands

			if ((!(bot_received & ADDRESS_MASK)) && (bot_received & COM_BLC_A)) // checking if the current slave is addressed and command is balancing
			{
				START_BALANCING();
				timer_clear_timer(TIMER_BALANCE);
				stat_led_orange();
			}
			else // current module not addressed... pass through addressed command
			{
				address_received = (uint8_t)bot_received & ADDRESS_MASK;
				top_send = bot_received & ~ADDRESS_MASK;		 // handout addressed command to next module
				top_send |= calc_data_bal(address_received - 1); // add address and parity to command
				manch_init_send();								 // initialize send to top board
				manch_send(top_send);							 // send command to next module
			}

			break;

		}
*/
		//--------------BALANCING-TIMING-------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
/*
		if (BALANCE_time >= 1000) // Balancing Timeout after 1 second
		{
			STOP_BALANCING();
			timer_clear_timer(TIMER_BALANCE);
		}
*/
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
//	timer_init_timer();
//	timer_add_time();
//	ADC_init();
	stat_led_init(); // Status LED initialised
	BALANCING_DDR |= (1 << BALANCING_PIN);
	sei(); // global interrupt enable
}
