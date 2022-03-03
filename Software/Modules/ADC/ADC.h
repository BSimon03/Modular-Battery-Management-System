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

#define EEPROM_3V_ADR (uint16_t *)0x01   // Address of 3v raw value
#define EEPROM_4V_ADR (uint16_t *)0x03   // Address of 4v raw value
#define EEPROM_temp_ADR (uint16_t *)0x05 // Address of ADC temperature offset
#define EEPROM_k_ADR (uint16_t *)0x07    // Address of slope error
#define EEPROM_d_ADR (uint16_t *)0x09    // Address of offset error

// Callibration
#define CAL_VOLTAGE_H 0x0640 // Callibration Voltage HIGH    -> 4V
#define CAL_VOLTAGE_HB 0x05C8//                              -> 3.7
#define CAL_VOLTAGE_L 0x04B0 // Callibration Voltage LOW     -> 3V
#define CAL_VOLTAGE_LB 0x0528//                              -> 3.3
#define CAL_TEMP 22          // Environment temperature during callibration

#define EEPROM_STATUS_ADR (uint8_t *)0x00 // Address of the status byte
// Byte pointers to each status bit in the eeprom status byte
#define EEPROM_CALLIBRATED 0x01
#define EEPROM_STATUS_L 0x02
#define EEPROM_STATUS_H 0x04
#define EEPROM_STATUS_TEMP 0x06

//--------------MAKROS-----------------------------//
#define ADC_INTERRUPT ADCSRA &(1 << ADIF) // ADC interrupt flag set?

// Setting a default value for the ADC_FILTER if nobody defines it in the main.c
#ifndef ADC_FILTER
#define ADC_FILTER 0
#endif

// Giving Names to Numbers
enum ADC_STATES
{
    ST_REGISTER,
    ST_MEASURE,
    ST_FILTER
};

//--------------FUNCTION-DEKLARATIONS--------------//

// Function ADC_setup:
// setting all important bits of the ADC register
void ADC_init(void);

// Function ADC_get_cal:
// Retrieving calibration data from the eeprom
void ADC_get_callibration(void);

// Function measure_temperature:
// Returns the temperature in degree celsius after at least 3 calls
// ADC filtering can be enabled by defining FILTER as 1
// Filtering cuts lowest and highest value and averages the remaining ones.
// It's recommended to make at least 6 measurements when using ADC filtering
int8_t measure_temperature(uint8_t);

// Function measure_voltage:
// Returns the voltage raw value after at least 3 calls
// ADC filtering can be enabled by defining FILTER as 1
// Filtering cuts lowest and highest value and averages the remaining ones.
// It's recommended to make at least 6 measurements when using ADC filtering
uint16_t measure_voltage(uint8_t);

#endif // ADC_H