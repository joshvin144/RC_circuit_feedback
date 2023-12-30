/*
 * pid_controller_user.h
 *
 *  Created on: Dec 29, 2023
 *      Author: jvincent
 */

#ifndef SRC_MODULES_PID_CONTROLLER_PID_CONTROLLER_USER_H_
#define SRC_MODULES_PID_CONTROLLER_PID_CONTROLLER_USER_H_


#define MAX_ERROR ( 921.6f )


void calculate_error_from_set_point( float* dest, uint32_t dest_size, float* src, uint32_t src_size, float set_point );


float calculate_composite( float* src, uint32_t src_size, float set_point );


#endif /* SRC_MODULES_PID_CONTROLLER_PID_CONTROLLER_USER_H_ */
