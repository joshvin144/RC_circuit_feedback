/*
 * messaging.c
 *
 *  Created on: Dec 29, 2023
 *      Author: jvincent
 */


#include "messaging.h"


static char message_queue[MESSAGE_QUEUE_LENGTH][MESSAGE_LENGTH] = {0};
static uint32_t message_queue_length = 0u;
static bool queue_is_full = false;


void prepare_message(char* message, uint32_t message_length, float data)
{
	// Only accept values below 10.0f
	float dividend = data / ( 10.0f );
	assert( dividend < 10.0f );

	// Copy the data
	int sprintf_return = sprintf(message, "%.3f,", data);
	assert(sprintf_return == message_length);
}


void queue_message(char* message, uint32_t message_length)
{
	/*
	 * TODO: Currently, there is a mismatch in timing between adding messages to the queue and emptying the queue
	 * If the queue is not emptied before adding more messages, a Hard Fault will occur.
	 * Therefore, queue_is_full was added.
	 * If the queue_is_full, another message cannot be added.
	 * queue_is_full will be reset the next time that the queue is emptied.
	 */
	if( !queue_is_full )
	{
		memcpy(message_queue[message_queue_length], message, message_length);
		message_queue_length++;
	}

	if( message_queue_length == ( MESSAGE_QUEUE_LENGTH - 1 ) )
	{
		queue_is_full = true;
	}
}


void convert_data_to_messages_and_queue( float* src, uint32_t src_size )
{
	assert( NULL != src );
	assert( MESSAGE_QUEUE_LENGTH >= src_size );

	char message[MESSAGE_LENGTH];
	for( uint32_t i = 0u; i < MESSAGE_QUEUE_LENGTH; i++ )
	{
		prepare_message(message, MESSAGE_LENGTH, src[i]);
		queue_message(message, MESSAGE_LENGTH);
	}
}


HAL_StatusTypeDef dispatch_message_from_uart(UART_HandleTypeDef* huart, char* message, uint32_t message_length)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_UART_Transmit(huart, (uint8_t*) message, message_length, USART2_TIMEOUT_MS);
	return status;
}


void empty_queue(UART_HandleTypeDef* huart)
{
	HAL_StatusTypeDef status = HAL_OK;
	for( uint32_t i = 0u; i < message_queue_length; i++ )
	{
		status = dispatch_message_from_uart(huart, message_queue[i], MESSAGE_LENGTH);
		assert( HAL_OK == status );
	}

	// Reset message queue
	message_queue_length = 0u;
	queue_is_full = false;
}

