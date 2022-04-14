/*********************************************/
/*  file:	main.c (V1.0)                      */
/*						                               */
/*  Diploma Thesis:                          */
/*   Battery Management System 2021/22       */
/* 						                               */
/*  brief:  Application for the Slaves       */
/* 						                               */
/*  Author: Simon Ball                       */
/*********************************************/

/*
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

//--------------MAIN-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
int main(void)
{
  bms_slave_init(); // Initiating the MCU, Registers configurated

  uint8_t ADCstat = MEASURE_VOLT;
  // 0  : Set up for Battery Temperature Measurement
  // 1  : Set up for Battery Voltage Measurement

  // Measurements
  uint16_t adc_raw = 0;

  int8_t battery_temperature; // battery_temperature = adc_value - 273; // K to degree C
  uint16_t battery_voltage;   // battery_voltage = (float)adc_value / 200; // divided by 1024 aka 10-bit, multiplied by 2,56 aka internal reference voltage * 2 (voltage divider)
  _delay_ms(500);
  uint8_t eeprom_stat = eeprom_read_byte(EEPROM_STATUS_ADR);
  _delay_ms(500);
  //--------------CALIBRATION----------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
  if (eeprom_stat != EEPROM_CALIBRATED) // if EEPROM not calibrated
  {
    while (!battery_voltage) // Measure SUPPLY voltage
    {
      battery_voltage = measure_voltage(ADC_SAMPLES_V);
      eeprom_write_word(EEPROM_3V_ADR, (uint16_t)battery_temperature);
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
      eeprom_write_word(EEPROM_3V_ADR, battery_voltage);
      if (!(eeprom_stat & EEPROM_STATUS_H)) // high voltage not calibrated yet
      {
        eeprom_write_byte(EEPROM_STATUS_ADR, EEPROM_STATUS_L);
      }
      else
      {
        eeprom_update_byte(EEPROM_STATUS_ADR, EEPROM_CALIBRATED);
      }
      while (1)
      {
        _delay_ms(200);
        stat_led_green();
        _delay_ms(200);
        stat_led_red();
      }
    }
    else if ((battery_voltage >= CAL_VOLTAGE_HB) && (!(eeprom_stat & EEPROM_STATUS_H))) // battery voltage smaller than high min voltage and not calibrated yet
    {
      eeprom_write_word(EEPROM_4V_ADR, battery_voltage);
      if (!(eeprom_stat & EEPROM_STATUS_L)) // low voltage not calibrated yet
      {
        eeprom_write_byte(EEPROM_STATUS_ADR, EEPROM_STATUS_H);
      }
      else
      {
        eeprom_update_byte(EEPROM_STATUS_ADR, EEPROM_CALIBRATED);
      }
      while (1)
      {
        _delay_ms(200);
        stat_led_green();
        _delay_ms(200);
        stat_led_off();
      }
    }
    else
    {
      while (1)
      {
        _delay_ms(200);
        stat_led_red(); // battery voltage out of predefined borders
        _delay_ms(200);
        stat_led_off();
      }
    }
  }
  uint16_t voltage_h = eeprom_read_word(EEPROM_4V_ADR);
  uint16_t voltage_l = eeprom_read_word(EEPROM_3V_ADR);
  int8_t temp_cal = (int8_t)eeprom_read_word(EEPROM_temp_ADR);
  float VOLT_K = (CAL_VOLTAGE_H - CAL_VOLTAGE_L) / (voltage_h - voltage_l); // calculate voltage slope error
  int8_t VOLT_D = CAL_VOLTAGE_H - (voltage_h * VOLT_K);                     // calculate voltage offset
  int8_t TEMP_D = temp_cal - CAL_TEMP;                                      // calculate temperature offset

  while (1)
  {
    //--------------ADC------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
    stat_led_off();

    if (!ADCstat)
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
  ADC_init();
  stat_led_init(); // Status LED initialised
  BALANCING_DDR |= (1 << BALANCING_PIN);
  sei(); // global interrupt enable
}*/

//********************************************************************************************************************************************************************************************************************************

/* LEVEL SHIFTER TEST

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
} */

//********************************************************************************************************************************************************************************************************************************

/*EEPROM TEST

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
  uint16_t* EEP = 0x0000;
  eeprom_update_word(EEP, 0x0000);
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
        STOP_BALANCING();
      }
      else if(voltage>3)
      {
        stat_led_orange();
        START_BALANCING();
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
}*/

//********************************************************************************************************************************************************************************************************************************

///* BALANCING TEST

//--------------USED-HARDWARE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//--Define Microcontroller, if not already defined in the platform.ini or intellisense
#ifndef __AVR_ATtiny261A__
#define __AVR_ATtiny261A__
#endif

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

//--------------SOURCE-FILES---------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// These are stored outside of the project folder, but will still be compiled
#include "ADC.h"
#include "status.h"

void bms_slave_init(void);

//--------------MAIN-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
int main(void)
{
  bms_slave_init();
  uint16_t adc;
  while (1)
  {
    adc=measure_voltage(4);
    if (adc)
    {
      if(adc>800)
      {
        stat_led_green();
        STOP_BALANCING();
      }
      else if(adc>600)
      {
        stat_led_orange();
        START_BALANCING();
      }
      else
      {
        stat_led_red();
        STOP_BALANCING();
      }
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
  ADC_init();
  stat_led_init(); // Status LED initialised
  BALANCING_DDR |= (1 << BALANCING_PIN);
  sei(); // global interrupt enable
}

//********************************************************************************************************************************************************************************************************************************

/* ADC TEMP TEST

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
  int8_t temp;
  while (1)
  {
    temp = measure_temperature(6);
    if (temp > -100)
    {
      if (temp > 50)
      {
        stat_led_green();
      }
      else if (temp < 50)
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
}*/

//********************************************************************************************************************************************************************************************************************************

/* ADC VOLT TEST
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
}*/

//********************************************************************************************************************************************************************************************************************************

/* BLINK AND TIMER TEST
//--------------USED-HARDWARE--------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//--Define Microcontroller, if not already defined in the platform.ini or intellisense
#ifndef __AVR_ATtiny261A__
#define __AVR_ATtiny261A__
#endif

//--------------LIBRARY-INCLUDES-----------------------------------------------------------------------------------------------------------------------------------------------------------------------//
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>

//--------------SOURCE-FILES---------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// These are stored outside of the project folder, but will still be compiled
#include "timer.h"
//#include "status.h"

void bms_slave_init(void);

//--------------MAIN-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
int main(void)
{
  uint16_t t = 0;
  bms_slave_init();
  DDRA|=(1<<PINA6);
  while (1)
  {
    timer_add_time();
    t = timer_get_timer(TIMER_ADC);
    if (t >= 500)
    {
      timer_clear_timer(TIMER_ADC);
      PORTA^=(1<<PINA6);
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
  sei(); // global interrupt enable
}*/