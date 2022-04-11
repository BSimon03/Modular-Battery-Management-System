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
}