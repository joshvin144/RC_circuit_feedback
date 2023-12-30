/*
 * messaging_user.h
 *
 *  Created on: Dec 29, 2023
 *      Author: jvincent
 */

#ifndef SRC_MODULES_MESSAGING_MESSAGING_USER_H_
#define SRC_MODULES_MESSAGING_MESSAGING_USER_H_

#include "stm32l4xx_hal.h"

#include "stdint.h"
#include "stdbool.h"


void convert_data_to_messages_and_queue( float* src, uint32_t src_size );


void prepare_message(char* message, uint32_t message_length, float data);


void queue_message(char* message, uint32_t message_length);


HAL_StatusTypeDef dispatch_message_from_uart(UART_HandleTypeDef* huart, char* message, uint32_t message_length);


void empty_queue(UART_HandleTypeDef* huart);


#endif /* SRC_MODULES_MESSAGING_MESSAGING_USER_H_ */
