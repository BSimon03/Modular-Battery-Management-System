/*************************/
/*  ADC.h                */
/*						 */
/*  Battery Management   */
/*      System           */
/* 						 */
/*  Functions for analog */
/*  digital conversions  */
/*************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>

#define ADC_INTERRUPT ADCSRA & 0b00010000

//ADC
#define TEMP		PINA6		//ADC5 MUX5:0 000101	//Connected to the NTC
#define BATT		PINA4		//ADC6 MUX3:0 000011	//Connected to VCC through a 50:50 voltage divider

//Tolerances
#define ADC_OFFSET		0.25					//ADC offset

//Functions

void ADC_setup(void){};

int measure_temperature(void);

float measure_voltage(void);