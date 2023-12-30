/*
 * adc_user.h
 *
 *  Created on: Dec 11, 2023
 *      Author: jvincent
 */

#ifndef SRC_MODULES_ADC_ADC_USER_H_
#define SRC_MODULES_ADC_ADC_USER_H_


#include "stdint.h"
#include "stdbool.h"


#define ADC_BUFFER_LENGTH ( 256u )


/*
 * @description : Initializes the data structure in adc.c
 * @parameters : None
 */
void adc_init_adc_module( void );


/*
 * @description : Calculates the voltage from the ADC output code
 * @parameters : uint32_t output_code : The ADC output code between 0 and exp2( num_bits ), where num_bits is the number of ADC bits
 */
float adc_calculate_voltage_from_output_code( uint32_t output_code );


uint16_t poll_adc( ADC_HandleTypeDef* hadc );


bool add_to_adc_buffer( uint16_t conversion_result );


void process_adc_output_codes( float* dest, uint32_t dest_size );


#endif /* SRC_MODULES_ADC_ADC_USER_H_ */
