/*
 * adc.h
 *
 *  Created on: Dec 11, 2023
 *      Author: jvincent
 */

#ifndef SRC_MODULES_ADC_ADC_H_
#define SRC_MODULES_ADC_ADC_H_


#include "stm32l4xx_hal.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>


// For polling the ADC
#define ADC_TIMEOUT_MS ( 1u )


// Sourced from "Figure 34. ADC accuracy characteristics" in the STM32L412KBU documentation
#define VDDA ( 3.6f )
#define VREF ( VDDA )
#define ADC_RESOLUTION ( 12u )
#define OUTPUT_CODE_RANGE ( ( uint32_t ) ( exp2( ADC_RESOLUTION ) ) )
#define SINGLE_LSB ( VREF/ ( float ) OUTPUT_CODE_RANGE )


// The length of the buffer in which to store ADC data
#define ADC_BUFFER_LENGTH ( 256u )


// Structure to store the ADC characteristics necessary to convert the output code to voltage
typedef struct{
	float reference_voltage;
	uint32_t output_code_range;
	float single_lsb;
} adc_t;



#endif /* SRC_MODULES_ADC_ADC_H_ */
