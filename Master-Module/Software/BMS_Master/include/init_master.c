/***************************/
/*      init_master.c      */
/*    Master Initiation    */
/*						   */
/*    Battery Management   */
/*        System           */
/* 						   */
/* Author: Tristan Horvath */
/***************************/

#include "init_master.h"

void CLK_setup()
{
    CLKPR=CLK_PS_SETTING;   //switch between prescalers according to the prescaler value set in init_master.h
}
void INT_setup()
{
    EIMSK|=(1<<INT0);       //Enable INT0
    EICRA|=(1<<ISC01);      //INT0 triggers at falling edge
    EICRA&=~(1<<ISC00);

    PCICR|=(1<<PCIE0);      //Enable pin change interrupt
    PCMSK0=(1<<PCINT5);     //Enable PCINT5
}
void init_master()
{
    DDRD|=(1<<STAT_G)|(1<<STAT_R);
    DDRF|=(1<<STAT_RELAY)|(1<<STAT_SSR);
    DDRF&=~(1<<IGNITION_DETECTION);
    sei();
    CLK_setup();
    INT_setup();
}