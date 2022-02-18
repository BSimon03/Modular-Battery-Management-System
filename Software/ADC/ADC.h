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

#define EEPROM_k_ADR (uint16_t *)0x01 		//Address of ADC slop coefficient
#define EEPROM_d_ADR (uint16_t *)0x03 		//Address of ADC offset
#define EEPROM_temp_ADR (uint16_t *)0x05 	//Address of ADC temperature offset

//ADC interrupt flag set?
#define ADC_INTERRUPT ADCSRA & 0b00010000

uint16_t VOLT_K = 0;
uint16_t VOLT_D = 0;
uint16_t TEMP_D = 0;

uint16_t adc_value = 0;
uint16_t sort; //sort algorithm
uint8_t adc_counter=0;

//Function deklarations

void ADC_setup(void);

void ADC_get_cal(void);

uint16_t measure_temperature(uint8_t , uint8_t){};

uint16_t measure_voltage(uint8_t, uint8_t){};
#endif