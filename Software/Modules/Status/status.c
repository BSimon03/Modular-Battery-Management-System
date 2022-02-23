/***************************/
/*         status.c        */
/*          Status         */
/*						   */
/*    Battery Management   */
/*        System           */
/* 						   */
/* Author: Tristan Horvath */
/***************************/

#include <avr/io.h>
#include "status.h"

//Status LED
void stat_led_red(){
    LEDPORT|=(1<<LEDPINR);
    LEDPORT&=~(1<<LEDPING);
}
void stat_led_green(){
    LEDPORT|=(1<<LEDPING);
    LEDPORT&=~(1<<LEDPINR);
}
void stat_led_orange(){
    LEDPORT|=(1<<LEDPING);
    LEDPORT|=(1<<LEDPINR);
}
void stat_led_off(){
    LEDPORT==~(1<<LEDPING);
    LEDPORT&=~(1<<LEDPINR);
}

#ifdef __AVR_ATmega32U4__
//Status Relay
void stat_rel_on(){
    PORTF|=(1<<PINF6);
}
void stat_rel_off(){
    PORTF&=~(1<<PINF6);
}

//Status solid state relais
void stat_ssr_on(){
    PORTF|=(1<<PINF5);
}
void stat_ssr_off(){
    PORTF&=~(1<<PINF5);
}
#endif