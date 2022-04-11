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
#define MIN_VOLTAGE 0x04B0 // Minimum voltage for the battery: ADC value = voltage x 400
#define ADC_FILTER 1       // Enable ADC filtering  0:OFF  1:ON
#define ADC_SAMPLES 6      // Averaging samples

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
#include <util/delay.h>
//#include <avr/eeprom.h>
//#include <avr/sleep.h>

//--------------SOURCE-FILES---------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// These are stored outside of the project folder, but will still be compiled
//#include "ADC.h"
//#include "communication.h"
#include "timer.h"
//#include "manch_m.h"
//#include "status.h"

void bms_slave_init(void);

#define UPTODOWN

//--------------MAIN-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
int main(void)
{
  bms_slave_init();
#ifdef DOWNTOUP
  DDRA |= (1 << PINA2);
  DDRB &= ~(1 << PINB6);
  DDRA |= (1 << PINA5);
  PORTA |= (1 << PINA5);
#endif
#ifdef UPTODOWN
  DDRA &= ~(1 << PINA2);
  DDRB |= (1 << PINB6);
  DDRA |= (1 << PINA6);
  PORTA |= (1 << PINA6);
#endif
  while (1)
  {
    _delay_us(50);
#ifdef DOWNTOUP
      PORTA ^= (1 << PINA2);
#endif
#ifdef UPTODOWN
      PORTB ^= (1 << PINB6);
#endif
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
  BALANCING_DDR |= (1 << BALANCING_PIN);
  sei(); // global interrupt enable
}