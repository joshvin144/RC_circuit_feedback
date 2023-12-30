/*
 * adc.c
 *
 *  Created on: Dec 11, 2023
 *      Author: jvincent
 */


#include "adc.h"

// Structure to model the ADC transfer curve
adc_t adc_1 = {0};

// ADC Buffer
uint16_t adc1_buffer[ADC_BUFFER_LENGTH] = {0};
uint32_t adc1_buffer_idx = 0u;
bool adc1_buffer_is_full = false;


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
	float voltage = adc_1.single_lsb * ( float ) output_code;
	return voltage;
}


uint16_t poll_adc( ADC_HandleTypeDef* hadc )
{
	HAL_StatusTypeDef status = HAL_ADC_Start( hadc );
	assert( HAL_OK == status );
	status = HAL_ADC_PollForConversion( hadc, ADC_TIMEOUT_MS );
	assert( HAL_OK == status );
	uint16_t conversion_result = ( uint16_t ) HAL_ADC_GetValue( hadc );
	status = HAL_ADC_Stop( hadc );
	return conversion_result;
}


bool add_to_adc_buffer(uint16_t conversion_result)
{
	if ( !adc1_buffer_is_full )
	{
	    adc1_buffer[adc1_buffer_idx++] = conversion_result;
	}

	if ( ADC_BUFFER_LENGTH == adc1_buffer_idx )
	{
		adc1_buffer_is_full = true;
	}

	// Return whether the adc1 is full or not. If it is full, there is data to process.
	return adc1_buffer_is_full;
}


void process_adc_output_codes(float* dest, uint32_t dest_size)
{
	assert( NULL != dest );
	assert( ADC_BUFFER_LENGTH >= dest_size );

	for( uint32_t i = 0u; i < ADC_BUFFER_LENGTH; i++ )
	{
		dest[i] = adc_calculate_voltage_from_output_code( (uint32_t) adc1_buffer[i] );
	}

	// Reset adc1_buffer
	adc1_buffer_idx = 0u;
	adc1_buffer_is_full = false;
}

