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

//Status LED
void stat_led_red(){
    PORTD|=(1<<PIND6);
    PORTD&=~(1<<PIND7);
}
void stat_led_green(){
    PORTD|=(1<<PIND7);
    PORTD&=~(1<<PIND6);
}
void stat_led_orange(){
    PORTD|=(1<<PIND7);
    PORTD|=(1<<PIND6);
}
void stat_led_off(){
    PORTD==~(1<<PIND7);
    PORTD&=~(1<<PIND6);
}

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