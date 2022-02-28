/***************************/
/*        main.cpp         */
/*    Diploma Thesis BMS   */
/*						             */
/*    ATMEGA32u4 Master    */
/*          V1.0           */
/* 						             */
/* Author: Tristan Horvath */
/***************************/
#ifndef __AVR_ATmega32U4__
#define __AVR_ATmega32U4__
#endif
#ifndef F_CPU
#define F_CPU=2000000UL
#endif

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "../init_master.h"

#include "communication.h"
#include "timer.h"
#include "status.h"
#include "manch_m.h"


int main(void)
{
  timer_init_timer();
  init_master();
  uint8_t com_cnt=0;
  enum TIMER_NR {
   Wait,
   SEND_TEMP_REQ,
   SEND_VOL_REQ,
   RECIEVE_TEMP,
   RECIEVE_VOL};
  while(1)
  {
    timer_add_time();
    com_cnt=timer_get_timer(TIMER_COM);
    if(com_cnt>=REQUEST_TIME)
    {
      com_cnt=0;
      manch_init_send();
      manch_send(REQ_TEMP_G);
    }
  }
}