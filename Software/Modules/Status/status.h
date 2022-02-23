#ifndef STATUS_H
#define STATUS_H
/***************************/
/*         status.h        */
/*          Status         */
/*						   */
/*    Battery Management   */
/*        System           */
/* 						   */
/* Author: Tristan Horvath */
/***************************/
#ifdef __AVR_ATtiny261A__
#define LEDPORT PORTA
#define LEDPINR PINA5
#define LEDPING PINA6
#elif __AVR_ATmega32U4__
#define LEDPORT PORTD
#define LEDPINR PIND6
#define LEDPING PIND7
#endif

//Status LED
void stat_led_red();
//make status LED light up RED
void stat_led_green();
//make status LED light up GREEN
void stat_led_orange();
//make status LED light up ORANGE
void stat_led_off();
//turn off status LED

#ifdef __AVR_ATmega32U4__
//Status Relay
void stat_rel_on();
//switch status relay on
void stat_rel_off();
//switch status relay off

//Status solid state relais
void stat_ssr_on();
//switch status solid state relais on
void stat_ssr_off();
//switch status solid state relais off
#endif
#endif  //STAUTS_H