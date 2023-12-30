/*
 * messaging.h
 *
 *  Created on: Dec 29, 2023
 *      Author: jvincent
 */

#ifndef SRC_MODULES_MESSAGING_MESSAGING_H_
#define SRC_MODULES_MESSAGING_MESSAGING_H_


#include "stm32l4xx_hal.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#define DEFAULT_TIMEOUT_MS ( 10000u )
#define USART2_TIMEOUT_MS DEFAULT_TIMEOUT_MS

#define MESSAGE_LENGTH ( 6u )
#define MESSAGE_QUEUE_LENGTH ( 256u )


#endif /* SRC_MODULES_MESSAGING_MESSAGING_H_ */
