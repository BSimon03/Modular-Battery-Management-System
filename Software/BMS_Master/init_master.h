#ifndef MASTER_INIT_H
#define MASTER_INIT_H
/***************************/
/*      init_master.h      */
/*    Master Initiation    */
/*						   */
/*    Battery Management   */
/*        System           */
/* 						   */
/* Author: Tristan Horvath */
/***************************/

//Pin definitions
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
#define STAT_SSR PINF5
#define STAT_RELAY PINF6

#define IGNITION PINF&(1<<PINF4)        //check if ignition is on

//Settings
#define CLK_PRESCALER_VALUE 1

#if CLK_PRESCALER_VALUE == 1
#define CLK_PS_SETTING (1<<CLKPCE)

#elif CLK_PRESCALER_VALUE == 2
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS0)

#elif CLK_PRESCALER_VALUE == 4
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS1)

#elif CLK_PRESCALER_VALUE == 8
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS1)|(1<<CLKPS0)

#elif CLK_PRESCALER_VALUE == 16
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS2)

#elif CLK_PRESCALER_VALUE == 32
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS2)|(1<<CLKPS0)

#elif CLK_PRESCALER_VALUE == 64
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS2)|(1<<CLKPS1)

#elif CLK_PRESCALER_VALUE == 128
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS2)|(1<<CLKPS1)|(1<<CLKPS0)

#elif CLK_PRESCALER_VALUE == 256
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS3)

#else
#error Invalid prescaler setting.

#endif

void CLK_setup();
void INT_setup();
void init_master();
#endif  //MASTER_INIT_H