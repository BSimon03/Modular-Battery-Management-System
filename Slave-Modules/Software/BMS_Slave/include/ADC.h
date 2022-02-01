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

//ADC interrupt flag set?
#define ADC_INTERRUPT ADCSRA & 0b00010000

//Settings
#define ADC_SAMPLES 6 		//Averaging x-2 samples

//Tolerances
//only linear error correction... y=k*x+d
#define ADC_OFFSET		0.25					//ADC offset
#define ADC_DRIFT       0.25                    //ADC drift
#define TEMP_CONSTANT	3.6						//NTC Constant

//Function deklarations

void ADC_setup(void){};

uint16_t measure_temperature(uint8_t reps, uint8_t filter);

float measure_voltage(uint8_t reps, uint8_t filter);