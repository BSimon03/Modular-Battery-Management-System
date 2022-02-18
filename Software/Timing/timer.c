//===============================================================
//
//   Projekt sax-extender master
//
//   rst, jan 2022
//
//================================================================


#include <avr/io.h>
#include "timer.h"

uint16_t timer_timer[TIMER_MAX_TIMER];
uint16_t timer_tick;  // zählt die ms (genauer: 1024µs)


void timer_init_timer()
// sartet timer 0
{
#if F_CPU == 2000000L
   TCCR0B = 0x04; // clk/256 => 8 takte je ms
#elif F_CPU == 1000000L
   TCCR0B = 0x04; // clk/256 => 4 takte je ms
#else
//noch nicht implementiert
#endif // F_CPU
}

void timer_add_time(void)
// liest timer aus und addiert die timer_tick
{
   uint8_t t;
   t=TCNT0;
#if F_CPU == 2000000L
   TCNT0 = t & 0x07; //stellen für die ganzen ms löschen, geht in simul nicht?
   timer_tick += (t >> 3); // ganze ms dazuaddieren.
#elif F_CPU == 1000000L
   TCNT0 = t & 0x03; //stellen für die ganzen ms löschen
   timer_tick += (t >> 2);
#else
   #error f_CPU not set or no function implemented
#endif // F_CPU
}

void timer_clear_timer(uint8_t nr)
// setzt den timer auf aktuellen wert
{
   timer_timer[nr] = timer_tick;
}

uint16_t timer_get_timer(uint8_t nr)
// liefert die zeit des timers seit dem letzten clear in ms zurück
{
   return (timer_tick - timer_timer[nr]);
}