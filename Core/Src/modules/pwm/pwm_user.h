/*
 * pwm_user.h
 *
 *  Created on: Dec 11, 2023
 *      Author: jvincent
 */

#ifndef SRC_MODULES_PWM_PWM_USER_H_
#define SRC_MODULES_PWM_PWM_USER_H_


#define PWM_CLOCK_FREQUENCY_HZ ( 4000000.0f )
#define PWM_PULSE_FREQUENCY_HZ ( 0.1f ) // Reset every 10 seconds
#define PWM_ARRX ( 65535.0f )
#define PWM_DUTY_CYCLE ( 0.50f ) // 50% Duty cycle


uint32_t pwm_calculate_prescaler(float clock_frequency_Hz, float pulse_frequency_Hz, float auto_reload);


uint32_t pwm_calculate_CCRx(float duty_cycle, float auto_reload);


#endif /* SRC_MODULES_PWM_PWM_USER_H_ */
