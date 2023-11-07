/*
 * volts_to_temp.h
 *
 *  Created on: Nov 3, 2023
 *      Author: cengr-lab
 */
#include <stdint.h>

#ifndef INC_VOLTS_TO_TEMP_H_
#define INC_VOLTS_TO_TEMP_H_
#define VCC 3300
#define R_PULLUP 10000
#define ADC_VOLT_REF 4096

uint16_t volts_to_temp(uint16_t adc_read);

#endif /* INC_VOLTS_TO_TEMP_H_ */
