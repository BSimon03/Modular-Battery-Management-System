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
  bms_slave_init();
  stat_led_red();
  float voltage;
  uint16_t adc;
  while (1)
  {
    adc=measure_voltage(6);
    if (adc)
    {
      voltage = (float)adc/200;
      if(voltage>4)
      {
        stat_led_green();
      }
      else if(voltage>3)
      {
        stat_led_orange();
      }
      else
      {
        stat_led_red();
      }
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

ISR(ADC_vect)
{
  stat_led_orange();
}