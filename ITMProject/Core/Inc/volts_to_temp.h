/*
 * volts_to_temp.h
 *
 *  Created on: Nov 3, 2023
 *      Author: Jesse Haynie, Hayden Higgins
 *  Description
 */
#ifndef INC_VOLTS_TO_TEMP_H_
#define INC_VOLTS_TO_TEMP_H_

#include <stdint.h>
//#include <math.h>
#define VCC 3300
#define R_PULLUP 10000
#define ADC_VOLT_REF 4096
//#define CALIBRATION_RESISTANCE 10000
//#define CALIBRATION_TEMPERATURE 298.15
//#define THERMISTOR_BETA 3435

uint16_t volts_to_resistance(uint16_t adc_read);
//float volts_to_resistance(uint16_t adc_read);
//float resistance_to_temperature(float temperature);

#endif /* INC_VOLTS_TO_TEMP_H_ */
