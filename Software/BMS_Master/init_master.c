/***************************/
/*      init_master.c      */
/*    Master Initiation    */
/*						   */
/*    Battery Management   */
/*        System           */
/* 						   */
/* Author: Tristan Horvath */
/***************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include "init_master.h"

void CLK_setup()
{
    CLKPR=CLK_PS_SETTING;   //switch between prescalers according to the prescaler value set in init_master.h
}
void INT_setup()
{
    EIMSK|=(1<<INT0);       //Enable External Interrupt (INT0) for communication
    EICRA|=(1<<ISC01);      //INT0 triggers at falling edge
    EICRA&=~(1<<ISC00);

    PCICR|=(1<<PCIE0);      //Enable pin change interrupts
    PCMSK0=(1<<PCINT5);     //Enable pin change interrupt for communication
}
void init_master()
{
    DDRF&=~(1<<IGNITION_DETECTION);     //Set ignition detection Pin as input
    PORTF&=~(1<<IGNITION_DETECTION);        //enable pulldown resistor on ignition detection pin
    timer_init_timer();
    sei();
    CLK_setup();
    INT_setup();
}