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
  timer_init_timer();   //initialize timers
  init_master();    //initialize master
  uint8_t com_cnt=0;
  enum COM_STATUS {
   WAIT,
   SEND_TEMP_REQ,
   SEND_VOL_REQ,
   RECIEVE_TEMP,
   RECIEVE_VOL};
   uint8_t com_status=WAIT;
   uint16_t data_temp;

  while(1)
  {
    timer_add_time();
    com_cnt+=timer_get_timer(TIMER_COM);    //add timer to counter
    if(com_cnt>=REQUEST_TIME)   //start requesting variables
    {
      com_cnt=0;
      switch(com_status)
      {
        case SEND_TEMP_REQ:   //send temperature request
          manch_init_send();
          manch_send(REQ_TEMP_G);
          com_status=RECIEVE_TEMP;
          break;
        case RECIEVE_TEMP:    //initialize recieving
          manch_init_receive();
          com_status=WAIT;    
        case WAIT:    //wait until done recieving
        if(manch_receive(data_temp)==1)   //if recieving was successfull
          {
            com_status=SEND_VOL_REQ;
            break;
          }
        else if(manch_receive(data_temp)==2)    //if an error ocurred, request again
        {
          com_status=RECIEVE_TEMP;
          break;
        }


      }
    }
  }
}