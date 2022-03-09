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

//**********Pin definitions**********
//PORTB
#define CAN_SLEEP PINB1
#define ALT_SERIAL_RX PINB2
#define ALT_SERIAL_TX PINB3
#define Master_OUT_2 PINB4
#define MASTER_IN_1 PINB5
#define MASTER_OUT_1 PINB6
//PORTD
#define MASTER_IN_2 PIND0
#define STAT_R PIND6
#define STAT_G PIND7
//PORTF
#define IGNITION_DETECTION PINF4

#define IGNITION PINF&(1<<PINF4)        //check if ignition is on

//**********measurement**********
#define SLAVE_COUNT 13      //number of used slaves
#define MAX_CELL_TEMP 60        //maximum temperature a battery cell is allowed to have
#define EOC_VOLTAGE 4.2
#define MAX_VOLT_DIFF 0.1

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "communication.h"
#include "timer.h"
#include "status.h"
#include "manch_m.h"

void init_master();

int main(void)
{
  init_master();    //initialize master
  timer_init_timer();
  stat_ssr_init();
  stat_rel_init();
  stat_led_init();

  int16_t data_temp[SLAVE_COUNT]={0};
  float data_volt[SLAVE_COUNT]={0};
  uint16_t *data;
  uint8_t com_stat, com_err=0;
  uint8_t adr_high_volt=0,val_high_volt=0,val_low_volt=5;

  enum STATE {
    SEND_TEMP_R,
    INIT_RECIEVE_TEMP,
    RECIEVE_TEMP,
    SEND_VOLT_R,
    INIT_RECIEVE_VOLT,
    RECIEVE_VOLT,
    CHECK_TEMP,
    FIND_HIGHEST_VOLT,
    FIND_LOWEST_VOLT,
    CHECK_EOC_VOLT,
    SEND_BAL_COM
  };
  uint8_t state = SEND_TEMP_R;
  uint8_t recieve_cnt=0, pos_cnt=0;

  while(1)
  {
    timer_add_time();
    stat_ssr_off();

    switch(state)
    {
      //==========get temperature of battery cells==========
      case SEND_TEMP_R:
        manch_init_send();
        manch_send(REQ_TEMP_G); //send request for temp
        recieve_cnt=0;
        state=INIT_RECIEVE_TEMP;
        break;

      case INIT_RECIEVE_TEMP:
        manch_init_receive();   //initialize recieving
        state=RECIEVE_TEMP;
        break;
      
      case RECIEVE_TEMP:
        if(recieve_cnt<SLAVE_COUNT)   //if there is still data to be recieved
        {
          com_stat=manch_receive(data);    //get status from recieving
          if(com_stat==1)   //if recievig done
          {
            state=INIT_RECIEVE_TEMP;
            recieve_cnt++;
            if(calc_parity(*data))   //if parity wrong
            {
              com_err=1;    //set error flag
            }
            else if (!calc_parity(*data))    //if parity right
            {
              data_temp[recieve_cnt]=*data-0x8000;   //copy data into array and cut off startbit
            }
          }
          else if(com_stat==2)  //if recieving error
          {
            state=INIT_RECIEVE_TEMP;
            recieve_cnt++;
            com_err=1;    //set error flag
          }
          break;  
        }
        else if((recieve_cnt==SLAVE_COUNT)&&com_err) //if done recieving and an error occurred
        {
          state=SEND_TEMP_R;    //request temperature again
          break;
        }
        else if((recieve_cnt==SLAVE_COUNT)&&(!com_err))    //if recieving was successfull
        {
          state=SEND_VOLT_R;   //proceed to get voltages
          break;
        }

      //==========get voltage of battery cells==========
      case SEND_VOLT_R:
        manch_init_send();
        manch_send(REQ_VOLT_G); //send request for temp
        recieve_cnt=0;
        state=INIT_RECIEVE_VOLT;
        break;

      case INIT_RECIEVE_VOLT:
        manch_init_receive();   //initialize recieving
        state=RECIEVE_VOLT;
        break;
      
      case RECIEVE_VOLT:
        if(recieve_cnt<SLAVE_COUNT)   //if there is still data to be recieved
        {
          com_stat=manch_receive(data);    //get status from recieving
          if(com_stat==1)   //if recievig done
          {
            state=INIT_RECIEVE_VOLT;
            recieve_cnt++;
            if(calc_parity(*data))   //if parity wrong
            {
              com_err=1;    //set error flag
            }
            else if (!calc_parity(*data))    //if parity right
            {
              data_volt[recieve_cnt]=*data-0x8000;   //copy data into array and cut off startbit
            }
          }
          else if(com_stat==2)  //if recieving error
          {
            state=INIT_RECIEVE_VOLT;
            recieve_cnt++;
            com_err=1;    //set error flag
          }
          break;  
        }
        else if((recieve_cnt==SLAVE_COUNT)&&com_err) //if done recieving and an error occurred
        {
          state=SEND_VOLT_R;    //request voltage again
          break;
        }
        else if((recieve_cnt==SLAVE_COUNT)&&(!com_err))    //if recieving was successfull
        {
          state=SEND_VOLT_R;   //proceed to get voltages
          break;
        }

      //==========Check if there is as an overtemperature==========
      case CHECK_TEMP:
        if(pos_cnt<SLAVE_COUNT)
        {
          if(data_temp[pos_cnt]>=MAX_CELL_TEMP)   //if cell temperature is over limit
          {
            stat_rel_on();    //switch on status relay
            stat_led_red();   //turn status led red
            //eventual communication with Motor controller could be inserted here
          }
          else if(data_temp[pos_cnt]<=MAX_CELL_TEMP)    //if cell temperature is under limit
          {
            stat_led_green();   //turn status led green
          }
          pos_cnt++;
        }
        else
        {
          state=FIND_HIGHEST_VOLT;
          pos_cnt=0;
        }
        break;

      //==========find highest voltage cell and send command to discharge that cell or while charging disable charging if voltage is over charge limit==========  
      case FIND_HIGHEST_VOLT:
        if(pos_cnt<SLAVE_COUNT)
        {
          if(data_volt[pos_cnt]>val_high_volt)   
          {
            val_high_volt=data_volt[pos_cnt];
            adr_high_volt=pos_cnt;
          }
          pos_cnt++;
        }
        else
        {
          state=CHECK_EOC_VOLT;
          pos_cnt=0;
        }
        break;
      case FIND_LOWEST_VOLT:
        if(pos_cnt<SLAVE_COUNT)
        {
          if(data_volt[pos_cnt]<val_high_volt)   
          {
            val_low_volt=data_volt[pos_cnt];
          }
          pos_cnt++;
        }
        else
        {
          state=CHECK_EOC_VOLT;
          pos_cnt=0;
        }
        break;
        
      case CHECK_EOC_VOLT:
        if(data_volt[adr_high_volt]>=EOC_VOLTAGE)   //if end-of-charge voltage is reached, switch on status SSR
        {
          stat_ssr_on();
        }
        state=SEND_BAL_COM;
        break;

      case SEND_BAL_COM:
        if(IGNITION)    //if ignition is on
        {
          if ((val_high_volt-val_low_volt)>=MAX_VOLT_DIFF)
          {
            manch_init_send();
            manch_send(calc_data_bal(adr_high_volt));    //send adressed balancing command
          }
        }
        state=SEND_TEMP_R;
        break;
    }
  }
}

void init_master()
{
    DDRF&=~(1<<IGNITION_DETECTION);     //Set ignition detection Pin as input
    PORTF&=~(1<<IGNITION_DETECTION);        //enable pulldown resistor on ignition detection pin
    EIMSK|=(1<<INT0);       //Enable External Interrupt (INT0) for communication
    EICRA|=(1<<ISC01);      //INT0 triggers at falling edge
    EICRA&=~(1<<ISC00);

    PCICR|=(1<<PCIE0);      //Enable pin change interrupts
    PCMSK0=(1<<PCINT5);     //Enable pin change interrupt for communication
    sei();
}