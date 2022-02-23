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
#define ADC_INTERRUPT ADCSRA&(1<<ADIF)

#ifndef FILTER
#define FILTER 0
#endif

enum STATES{
    ST_REG,
    ST_MSR,
    ST_FILTER
};

static uint16_t VOLT_K = 0;
static uint16_t VOLT_D = 0;
static uint16_t TEMP_D = 0;

static uint16_t adc_value = 0;
uint16_t sort; //sort algorithm

//Function deklarations

void ADC_setup(void);

void ADC_get_cal(void);

int8_t measure_temperature(uint8_t);

uint16_t measure_voltage(uint8_t);
#endif //ADC_H