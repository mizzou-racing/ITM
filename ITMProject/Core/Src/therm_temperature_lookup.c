/*
 * therm_temperature_lookup.c
 *
 *  Created on: Nov 7, 2023
 *      Author: Jesse Haynie
 */
#include "therm_temperature_lookup.h"

// Look up table in celsius
int16_t look_up_temperature[2][111] = {
    {-10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
     10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
     30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
     50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
     70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
     90, 91, 92, 93, 94, 95, 96, 97, 98, 99},
    {3369, 3339, 3308, 3277, 3244, 3212, 3178, 3144, 3109, 3074,
     3038, 3002, 2965, 2927, 2889, 2851, 2812, 2773, 2734, 2694,
     2654, 2614, 2574, 2533, 2493, 2452, 2411, 2370, 2330, 2289,
     2248, 2208, 2167, 2127, 2087, 2048, 2008, 1969, 1930, 1891,
     1853, 1816, 1778, 1741, 1705, 1669, 1633, 1598, 1563, 1529,
     1496, 1463, 1430, 1398, 1367, 1336, 1306, 1276, 1247, 1218,
     1190, 1163, 1136, 1109, 1083, 1058, 1033, 1009, 985, 962,
     939, 917, 895, 874, 854, 833, 814, 794, 776, 757, 739,
     722, 705, 688, 672, 656, 641, 625, 611, 596, 583, 569,
     556, 543, 530, 518, 506, 494, 483, 472, 461, 450, 440,
     430, 420, 411, 402, 393, 384, 375, 367}
};

// Binary search algoritm for look up table
int16_t binary_search(uint32_t adc_target)
{
	/**
	 * "A thermistor fault is triggered if the analog voltage measured from the battery thermistor is
	 * detected outside of the normal thermal operating range. This error can be triggered if the
	 * temperature of the thermistor rises above 85C or drops lower than -40C. A shorted or open
	 * wire can result in artificially high or low measurements that would result in this error code.
	 * Additionally the use of an incompatible thermistor can cause inaccurate readings and trigger
	 * this error code" Found here https://www.orionbms.com/faultcodes/DTC%20P0A9C%20-%20Battery%20Thermistor%20Fault.pdf
	 */
	if (adc_target >= 3369)
	{
		return -41;
	}
	else if (adc_target <= 494)
	{
		return 86;
	}

	uint16_t start = 0;
	uint16_t end = 110;
	uint16_t mid = start + (end - start) / 2;

	while(start <= end)
	{
		if(look_up_temperature[1][mid] == adc_target)
		{
			return look_up_temperature[0][mid];
		}
		else if(adc_target < look_up_temperature[1][mid])
		{
			start = mid + 1;
		}
		else
		{
			end = mid - 1;
		}
		mid = start + (end - start) / 2;
	}
	return look_up_temperature[0][mid];
}
