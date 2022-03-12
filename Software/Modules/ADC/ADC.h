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

// Callibration
#define CAL_VOLTAGE_H 0x0640  // Callibration Voltage HIGH    -> 4V
#define CAL_VOLTAGE_HB 0x05C8 //                              -> 3.7
#define CAL_VOLTAGE_L 0x04B0  // Callibration Voltage LOW     -> 3V
#define CAL_VOLTAGE_LB 0x0528 //                              -> 3.3
#define CAL_TEMP 22           // Environment temperature during calibration

#define EEPROM_STATUS_ADR (uint8_t *)0x00 // Address of the status byte
// Byte pointers to each status bit in the eeprom status byte
#define EEPROM_CALIBRATED 0x01
#define EEPROM_STATUS_L 0x02
#define EEPROM_STATUS_H 0x04
#define EEPROM_STATUS_TEMP 0x06

//--------------MAKROS-----------------------------//
#define ADC_INTERRUPT ADCSRA &(1 << ADIF)            // ADC interrupt flag set?
#define ADC_START_CONVERSION() ADCSRA |= (1 << ADSC) // ADC start conversion
#define ADC_CLEAR_INT() ADCSRA |= (1 << ADIF)        // clear interrupt flag

//--------------FUNCTION-DEKLARATIONS--------------//

// Function ADC_setup:
// setting all important bits of the ADC register
void ADC_init(void);

void ADC_set_volt(void);

void ADC_set_temp(void);

// Function ADC_measure:
// Returns the temperature in degree celsius after at least 3 calls
// ADC filtering can be enabled by defining FILTER as 1
// Filtering cuts lowest and highest value and averages the remaining ones.
// It's recommended to make at least 6 measurements when using ADC filtering
uint16_t ADC_measure(uint8_t conversions);

#endif // ADC_H