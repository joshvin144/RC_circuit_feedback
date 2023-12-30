/*
 * pid_controller.h
 *
 *  Created on: Dec 29, 2023
 *      Author: jvincent
 */

#ifndef SRC_MODULES_PID_CONTROLLER_PID_CONTROLLER_H_
#define SRC_MODULES_PID_CONTROLLER_PID_CONTROLLER_H_

#include "stm32l4xx_hal.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

#include "math_helpers_user.h"


#define ERROR_LOG_LENGTH ( 256u )
#define DERIVATIVE_LOG_LENGTH ( 255u )
#define TOLERANCE ( 0.1f )

#endif /* SRC_MODULES_PID_CONTROLLER_PID_CONTROLLER_H_ */
