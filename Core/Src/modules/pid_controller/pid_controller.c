/*
 * pid_controller.c
 *
 *  Created on: Dec 29, 2023
 *      Author: jvincent
 */


#include "pid_controller.h"


// Error
float error = 0.0f;
float error_log[ERROR_LOG_LENGTH] = {0.0f};

// More error terms
float integral = 0.0f;
float derivative = 0.0f;
float composite = 0.0f;

// Derivative specific
float derivative_log[DERIVATIVE_LOG_LENGTH] = {0.0f};

// Weights
float k_integral = 3.0f;
float k_derivative = 3.0f;
float k_proportion = 1.0f;


void calculate_error_from_set_point( float* dest, uint32_t dest_size, float* src, uint32_t src_size, float set_point )
{
	assert( NULL != dest );
	assert( NULL != src );
	assert( dest_size == src_size );
	for(uint32_t idx = 0u; idx < src_size; idx++)
	{
		dest[idx] = set_point - src[idx];
	}
}


float calculate_composite( float* src, uint32_t src_size, float set_point )
{
	assert( NULL != src );
	assert( ERROR_LOG_LENGTH >= src_size );

    // Calculate the residual error term
	calculate_error_from_set_point( error_log, ERROR_LOG_LENGTH, src, src_size, set_point );

	// Compute the integral term
	integral = math_helpers_trapezoid_approximation( error_log, ERROR_LOG_LENGTH );

	// Compute the derivative term
	math_helpers_derivative( derivative_log, DERIVATIVE_LOG_LENGTH, src, src_size );

	// Assign intermediary variables
	error = error_log[ERROR_LOG_LENGTH - 1u];
	derivative = derivative_log[DERIVATIVE_LOG_LENGTH - 1u];
	composite = ( k_proportion * error ) + ( k_integral * integral ) - ( k_derivative * derivative );
	return composite;
}

