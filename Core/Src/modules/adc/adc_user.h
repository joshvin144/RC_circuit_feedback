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


#define ADC_BUFFER_LENGTH ( 4096u )


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


#endif /* SRC_MODULES_ADC_ADC_USER_H_ */
