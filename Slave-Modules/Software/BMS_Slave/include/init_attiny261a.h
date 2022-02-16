/*************************/
/*  init_attiny261a.h    */
/*  Slave Initiation     */
/*						 */
/*  Battery Management   */
/*      System           */
/* 						 */
/*  Author: Simon Ball   */
/*************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/eeprom.h>
#include "ADC.h"
#include "communication.h"

//Pin definitions
	//PORTA
	#define COMM_TOP PINA2
	#define STAT_R PINA5
	#define STAT_G PINA6
	#define ADC_VOLT PINA7       //ADC6 MUX5:0 000110       Connected to VCC through a 50:50 voltage divider
	
	//PORTB
	#define BALANCING PINB4
	#define DEBUG PINB5
	#define COMM_BOT PINB6

//Settings
#define MIN_VOLTAGE	3							//Minimum voltage for the battery
#define CLK_PRESCALER_VALUE 1  //Must be 1, 2, 4, 8, 16, 32, 64, 128 or 256

//Strings sent by the master -> Requests
const uint8_t request_info= 0xAA; 				//0b11001100		//Master requests information
const uint8_t request_secs= 0xF0; 				//0b11110000		//Master requests the second byte of information, after the finish is acknowledged

//Answer string templates
const uint8_t error_high_temp 	= 0b01100000;	//high temperature (>=60°C)		send temperature /2
const uint8_t error_low_volt 	= 0b00100000;	//low voltage (<=2,5V)			send voltage x 10

const uint8_t error_not_suit	= 0b01000000;	//not suitable
const uint8_t error_res_high 	= 0b01001000;	//res too high (res>=50mOhm)
const uint8_t error_cap_dif 	= 0b01010000;	//self discharge... charge_cap|discharge_cap+-10%
const uint8_t error_timeout 	= 0b01011000;	//timeout (time>=10h)

const uint8_t idle 				= 0b10100000;	//idle
const uint8_t processing 		= 0b10000000;	//measurement in progress		current progress in percent
const uint8_t done 				= 0b11000000;	//done with the measurement		resistance in mOhms

/* MCU CLOCK PRESCALER */

//Prescaler value converted to bit settings.

#if F_CPU == 8000000UL										//PS = 1
#define CLK_PS_SETTING (1<<CLKPCE)

#elif F_CPU == 4000000UL									//PS = 2
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS0)

#elif F_CPU == 2000000UL									//PS = 4
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS1)

#elif F_CPU == 1000000UL									//PS = 8
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS1)|(1<<CLKPS0)

#elif F_CPU == 500000UL										//PS = 16
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS2)

#elif F_CPU == 250000UL										//PS = 32
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS2)|(1<<CLKPS0)

#elif F_CPU == 125000UL										//PS = 64
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS2)|(1<<CLKPS1)

#elif F_CPU == 62500UL										//PS = 128
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS2)|(1<<CLKPS1)|(1<<CLKPS0)

#elif F_CPU == 31250UL										//PS = 256
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS3)

#else
#error Invalid prescaler setting.

#endif

void init_attiny261a(void){};