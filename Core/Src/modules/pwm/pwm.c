/*
 * pwm.c
 *
 *  Created on: Dec 11, 2023
 *      Author: jvincent
 */


#include "pwm.h"


/*
* Calculate Prescaler (PSC)
* F_PWM = ( F_CLK )/( ( ARR + 1 ) * ( PSC + 1 ) )
* ( ARR + 1 ) * ( PSC + 1 ) = ( F_CLK ) / ( F_PWM )
* PSC = ( ( F_CLK ) / ( ( F_PWM ) * ( ARR + 1 ) ) ) - 1
*/
uint32_t pwm_calculate_prescaler(float clock_frequency_Hz, float pulse_frequency_Hz, float auto_reload)
{
	return floor( ( clock_frequency_Hz ) / ( ( pulse_frequency_Hz ) * ( auto_reload + 1 ) ) ) - 1u;
}


/*
* Calculate CCRx
* DUTY_CYCLE = ( CCRx ) / ( ARRx )
* CCRx = ( DUTY_CYCLE ) * ( ARRx )
*/
uint32_t pwm_calculate_CCRx(float duty_cycle, float auto_reload)
{
	return floor( duty_cycle * auto_reload );
}
