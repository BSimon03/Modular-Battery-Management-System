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

//--------------LIBRARY-INCLUDES-----------------------------------------------------------------------------------------------------------------------------------------------------------------------//
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>

//--------------SOURCE-FILES---------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// These are stored outside of the project folder, but will still be compiled
#include "communication.h"
#include "timer.h"
#include "manch_m.h"
#include "status.h"

void bms_slave_init(void);

//--------------MAIN-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
int main(void)
{
  bms_slave_init(); // Initiating the MCU, Registers configurated

  // Send data
  uint16_t send = 0xAACC;

  // Timing
  uint16_t COMM_time = 0; // compare value

  // clear timers after startup
  timer_clear_timer(TIMER_COMM);

  while (1)
  {
    //--------------ADC------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
    timer_add_time(); // executed after max 32ms
    COMM_time = timer_get_timer(TIMER_COMM);
    if (COMM_time >= 1000)
    {
      timer_clear_timer(TIMER_COMM);
      manch_init_send1();
      manch_init_send();
      manch_send1(send);
      manch_send(send);
      stat_led_toggle_green();
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
  stat_led_init(); // Status LED initialised
  sei();           // global interrupt enable
}

//********************************************************************************************************************************************************************************************************************************

/* EEPROM CALLIBRATION TEST

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

  int8_t battery_temperature = -100;
  uint16_t battery_voltage = 0; // battery_voltage = (float)adc_value / 200; // divided by 1024 (10-bit), multiplied by 2,56 (internal reference voltage) * 2 (voltage divider)
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

  while (1)
  {
    if (!ADCstat)
    {
      battery_voltage = measure_voltage(ADC_SAMPLES_V);
      if (battery_voltage) // make sure conversion is done
      {
        if (battery_voltage > 800)
        {
          STOP_BALANCING();
        }
        else if (battery_voltage > 600)
        {
          START_BALANCING();
        }
        else
        {
          STOP_BALANCING();
        }
        ADCstat = MEASURE_TEMP;
      }
    }
    else
    {
      battery_temperature = measure_temperature();
      if (battery_temperature > -100) // make sure conversion is done
      {
        if (battery_temperature > 30)
      {
        stat_led_green();
      }
      else if (battery_temperature < 30 && battery_temperature > 0)
      {
        stat_led_orange();
      }
      else
      {
        stat_led_red();
      }
        ADCstat = MEASURE_VOLT;
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
  uint16_t* EEP_adr = 0x0000;
  uint16_t EEP_data = 0xAAAA;
  eeprom_update_word(EEP_adr, EEP_data);
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
}*/

//********************************************************************************************************************************************************************************************************************************

/* BALANCING TEST

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
}*/

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