/*
 * volts_to_temp.c
 *
 *  Created on: Nov 3, 2023
 *      Author: cengr-lab
 */

#include "volts_to_temp.h"


uint16_t volts_to_temp(uint16_t adc_read)
{
	uint16_t volt_signal;
	uint16_t resistance;
	volt_signal = VCC*(adc_read*10000 / ADC_VOLT_REF)/10000;
	resistance = (volt_signal*R_PULLUP) / (VCC - volt_signal);
	return resistance;
}
