/*
 * volts_to_temp.c
 *
 *  Created on: Nov 3, 2023
 *      Author: Jesse Haynie, Hayden Higgins
 */

#include "volts_to_temp.h"

uint16_t volts_to_resistance(uint16_t adc_read)
{
	uint16_t volt_signal;
	uint16_t resistance;
	volt_signal = VCC * (adc_read * 10000 / ADC_VOLT_REF) / 10000;
	resistance = (volt_signal * R_PULLUP) / (VCC - volt_signal);
	return resistance;
}

//float volts_to_resistance(uint16_t adc_read)
//{
//	float volt_signal;
//	float resistance;
//	volt_signal = VCC * ((float)adc_read / ADC_VOLT_REF);
//	resistance = (volt_signal * R_PULLUP) / (VCC - volt_signal);
//	return resistance;
//}

//float resistance_to_temperature(float resistance)
//{
//	float temperature = 1.0/(log((float)resistance/CALIBRATION_RESISTANCE) / THERMISTOR_BETA + 1.0/CALIBRATION_TEMPERATURE);
//
//	return temperature;
//}
