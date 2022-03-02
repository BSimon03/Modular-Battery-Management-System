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
  stat_ssr_init();
  stat_rel_init();
  stat_led_init();

  int16_t data_temp[SLAVE_COUNT]={0};
  float data_volt[SLAVE_COUNT]={0};
  uint16_t *data;
  uint8_t com_stat, com_err=0;
  uint8_t adr_high_volt=0,val_high_volt=0;

  while(1)
  {
    timer_add_time();

    //==========get temperature of battery cells==========
    do
    {
      manch_init_send();
      manch_send(REQ_TEMP_G);

      for(int i=0; i<SLAVE_COUNT; i++)    //ammount of slaves determines number of passes
      {
        manch_init_receive();   //initialize recieving
        do    //wait for recieve complete
        {
          com_stat=manch_receive(*data);    //get status from recieving
          if(com_stat==1)   //if recievig done
          {
            data_temp[i]=*data;   //copy data into array
          }
          else if(com_stat==2)  //if recieving error
          {
            com_err=1;    //set error flag
          } 
        }while (!com_stat);
      }
    }while (com_err);   //repeat if an error occurred while recieving

    //==========get voltage of battery cells==========
    do
    {
      manch_init_send();
      manch_send(REQ_VOLT_G);

      for(int i=0; i<SLAVE_COUNT; i++)    //ammount of slaves determines number of passes
      {
        manch_init_receive();   //initialize recieving
        do    //wait for recieve complete
        {
          com_stat=manch_receive(*data);    //get status from recieving
          if(com_stat==1)   //if recievig done
          {
            data_volt[i]=*data/400;   //copy data into array
          }
          else if(com_stat==2)  //if recieving error
          {
            com_err=1;    //set error flag
          } 
        }while (!com_stat);
      }
    }while (com_err);   //repeat if an error occurred while recieving

    //==========Check if there is as an overtemperature==========
    for (int i=0; i<SLAVE_COUNT; i++)   //check for every slave
    {
      if(data_temp[i]>=MAX_CELL_TEMP)   //if cell temperature is over limit
      {
        stat_rel_on();    //switch on status relay
        stat_led_red();   //turn status led red
      }
      else if(data_temp[i]<=MAX_CELL_TEMP)    //if cell temperature is under limit
      {
        stat_led_green();   //turn status led green
      }
    }

    //==========find highest voltage cell and send command to discharge that cell or while charging disable charging if voltage is over charge limit==========
    for (int i=0; i<SLAVE_COUNT; i++)   //find highest voltage cell
    {
      if(data_volt[i]>val_high_volt)    
      {
        val_high_volt=data_volt[i];
        adr_high_volt=i;
      }
    }
    stat_ssr_off();
    if(data_volt[adr_high_volt]>=EOC_VOLTAGE)   //if end-of-charge voltage is reached, switch on status SSR
    {
      stat_ssr_on();
    }
    adr_high_volt+=COM_BLC_A;   //calculate adress to send balancing command to
    manch_init_send();
    manch_send(adr_high_volt);    //send adressed balancing command
  }
}