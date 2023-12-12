/*
 * adc.c
 *
 *  Created on: Dec 11, 2023
 *      Author: jvincent
 */


#include "adc.h"


adc_t adc_1 = {0};


void adc_init_adc_module( void )
{
	adc_1.reference_voltage = VREF;
	adc_1.output_code_range = OUTPUT_CODE_RANGE;
	adc_1.single_lsb = SINGLE_LSB;
}


float adc_calculate_voltage_from_output_code( uint32_t output_code )
{
	// It is not necessary to check that the output code is non-negative because it is an unsigned integer
	assert(output_code <= adc_1.output_code_range);
	float voltage = adc_1.single_lsb * (float) output_code;
	return voltage;
}
