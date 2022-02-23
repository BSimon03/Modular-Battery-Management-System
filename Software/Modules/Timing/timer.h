#ifndef TIMER_H
#define TIMER_H
//===============================================================
//
//   Projekt sax-extender master
//
//   rst, jan 2022
//
//================================================================
//  verwendet timer 0 und stellt funktionen fürs timing zur verfügung
// #define TIME_MAX_TIMER 2
enum TIMER_NR {
   TIMER_MANCH,
   TIMER_ADC,
// letzte eintrag, neue timer davor!
   TIMER_MAX_TIMER};

void timer_clear_timer(uint8_t nr);
// setzt den timer mit der nummer nr zurück
uint16_t timer_get_timer(uint8_t nr);
// liefert die zeit des timers mit der nummer nr seit dem letzten zuruecksetzen in ms zurück
void timer_init_timer(void);
// am beginn zum initialisieren 1 mal aufrufen
void timer_add_time(void);
// muss in der main-loop regelmässig aufgerufen werden.
// max. zeit zwischen 2 aufrufe: 32ms
// liest hw-timer aus und addiert die timer

#endif