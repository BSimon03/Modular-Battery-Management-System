/*************************/
/*  init_attiny261a.c    */
/*  Slave Initiation     */
/*						 */
/*  Battery Management   */
/*      System           */
/* 						 */
/*  Author: Simon Ball   */
/*************************/

#include "init_attiny261a.h"

/*
void mcu_set_clock()		//set clock to 10MHz? supply voltage?
{
	//internal clock must not be set, its selected by default
	CLKPR|=(1<<CLKPCE); //Clock prescaler enabled
	CLKPR&=~(1<<CLKPS3)|(1<<CLKPS2)|(1<<CLKPS1)|(1<<CLKPS0); //no prescaler (all 0), but still avoids unintentional clock changes
}
*/

void mcu_set_clock()					//Function to easily switch between Clock prescalers using the previous define
{
	CLKPR = CLK_PRESCALER_VALUE;
}

void INT_setup()						//External Interrupt, any edge
{
	GIMSK |= (1<<INT0);                     //General Interrupt Mask Register / interrupt enable
	MCUCR|=(1<<ISC00);				//any edge
	MCUCR&=~(1<<ISC01);
}

void PWM_setup()	//Charge/Discharge
{
	//Timer 1: 10kHz Software-PWM
	//2 Channels can be controlled by changing OCR1x and used via the OVF Interrupt

	TCCR1A&=~(1<<COM1A1)|(1<<COM1A0)|(1<<COM1B1)|(1<<COM1B0);
	TCCR1A|=(1<<PWM1A)|(PWM1B);
	
	TCCR1B|=(1<<CS13)|(1<<CS11)|(1<<CS10);
	TCCR1B&=~(1<<CS12);
	
	TCCR1C&=~(1<<COM1D1)|(1<<COM1D0);
	
	TCCR1D&=~(1<<WGM11);
	TCCR1D|=(1<<WGM10);
	
	TIMSK|=(1<<OCIE1A)|(1<<OCIE1B);
	
	OCR1C=20;
	
	OCR1A=0;
	OCR1B=0;
}

void init_attiny261a()					//Combining all setup functions
{
	DDRA|=(1<<STAT_G)|(1<<STAT_R);
	mcu_set_clock();
	INT_setup();
	PWM_setup();
	ADC_setup();
}