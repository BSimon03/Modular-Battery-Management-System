#ifndef INIT_BMS_SLAVE_H
#define INIT_BMS_SLAVE_H

/*************************/
/*  init_bms_slave.h     */
/*  Slave Initiation     */
/*						 */
/*  Battery Management   */
/*      System           */
/* 						 */
/*  Author: Simon Ball   */
/*************************/

#ifndef F_CPU
#define F_CPU 2000000L
#endif

#ifndef __AVR_ATtiny261A__
#define __AVR_ATtiny261A__
#endif

#ifndef BMS_SLAVE
#define BMS_SLAVE
#endif

//Pin definitions
	//PORTAS
	#define COMM_TOP PINA2
	#define STAT_R PINA5
	#define STAT_G PINA6
	#define ADC_VOLT PINA7       //ADC6 MUX5:0 000110       Connected to VCC through a 50:50 voltage divider
	
	//PORTB
	#define BALANCING PINB4
	#define DEBUG_PIN PINB5
	#define COMM_BOT PINB6

//Settings
#define MIN_VOLTAGE	3							//Minimum voltage for the battery
#define CLK_PRESCALER_VALUE 1  //Must be 1, 2, 4, 8, 16, 32, 64, 128 or 256

/* MCU CLOCK PRESCALER */

//Prescaler value converted to bit settings.

#if F_CPU == 8000000L										//PS = 1
#define CLK_PS_SETTING (1<<CLKPCE)

#elif F_CPU == 4000000L										//PS = 2
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS0)

#elif F_CPU == 2000000L										//PS = 4
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS1)

#elif F_CPU == 1000000L										//PS = 8
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS1)|(1<<CLKPS0)

#elif F_CPU == 500000L										//PS = 16
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS2)

#elif F_CPU == 250000L										//PS = 32
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS2)|(1<<CLKPS0)

#elif F_CPU == 125000L										//PS = 64
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS2)|(1<<CLKPS1)

#elif F_CPU == 62500L										//PS = 128
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS2)|(1<<CLKPS1)|(1<<CLKPS0)

#elif F_CPU == 31250L										//PS = 256
#define CLK_PS_SETTING (1<<CLKPCE)|(1<<CLKPS3)

#else
#error Invalid prescaler setting.

#endif
#endif //INIT_BMS_SLAVE_H