/*********************************************/
/*  file:   ADC.h                            */
/*						                     */
/*  Diploma Thesis:                          */
/*   Battery Management System 2021/22       */
/* 						                     */
/*  brief:  Functions for analog measurement */
/* 						                     */
/*  Author: Simon Ball                       */
/*********************************************/

#ifndef ADC_H
#define ADC_H

#define EEPROM_STATUS_ADR (uint8_t *)0x00   //Address of the status byte
#define EEPROM_k_ADR (float *)0x01 		//Address of ADC slop coefficient
#define EEPROM_d_ADR (float *)0x03 		//Address of ADC offset

#define EEPROM_temp_ADR (uint8_t *)0x05 	//Address of ADC temperature offset
enum EEPROM_STATES{
    NOT_CALLIBRATED,
    CALLIBRATED
};

#define CAL_VOLTAGE_H 4
#define CAL_VOLTAGE_L 3
#define CAL_TEMP 22

//--------------MAKROS-----------------------------//
#define ADC_INTERRUPT ADCSRA&(1<<ADIF)      //ADC interrupt flag set?

//Setting a default value for the ADC_FILTER if nobody defines it in the main.c
#ifndef ADC_FILTER
#define ADC_FILTER 0
#endif

//Giving Names to Numbers
enum ADC_STATES{
    ST_REGISTER,
    ST_MEASURE,
    ST_FILTER
};

//--------------FUNCTION-DEKLARATIONS--------------//

//Function ADC_setup:
    //setting all important bits of the ADC register
void ADC_init(void);

//Function ADC_get_cal:
    //Retrieving calibration data from the eeprom
void ADC_get_cal(void);

//Function ADC_callibrate:
    //Set new callibration values
void ADC_callibrate(void);

//Function measure_temperature:
    //Returns the temperature in degree celsius after at least 3 calls
    //ADC filtering can be enabled by defining FILTER as 1
    //Filtering cuts lowest and highest value and averages the remaining ones.
    //It's recommended to make at least 6 measurements when using ADC filtering
int8_t measure_temperature(uint8_t);

//Function measure_voltage:
    //Returns the voltage raw value after at least 3 calls
    //ADC filtering can be enabled by defining FILTER as 1
    //Filtering cuts lowest and highest value and averages the remaining ones.
    //It's recommended to make at least 6 measurements when using ADC filtering
uint16_t measure_voltage(uint8_t);

#endif //ADC_H