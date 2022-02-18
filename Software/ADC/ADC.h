#ifndef ADC_H
#define ADC_H
/*************************/
/*  ADC.h                */
/*						 */
/*  Battery Management   */
/*      System           */
/* 						 */
/*  Functions for analog */
/*  digital conversions  */
/*************************/

//ADC interrupt flag set?
#define ADC_INTERRUPT ADCSRA & 0b00010000

//Function deklarations

void ADC_setup(void){};

uint16_t measure_temperature(uint8_t reps, uint8_t filter, float temp_constant){};

float measure_voltage(uint8_t reps, uint8_t filter, float adc_offset, float adc_drift){};
#endif