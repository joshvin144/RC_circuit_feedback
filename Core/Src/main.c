/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pwm_user.h"
#include "adc_user.h"
#include "math_helpers_user.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TEST_MODE
#define USE_FULL_ASSERT

#define SAMPLE_PERIOD_MS ( 10u )
#define ADC_TIMEOUT_MS ( 1u )

#define DEFAULT_TIMEOUT_MS ( 10000u )
#define USART2_TIMEOUT_MS DEFAULT_TIMEOUT_MS

#define MESSAGE_LENGTH ( 6u )
#define MESSAGE_QUEUE_LENGTH ( 256u )

#define ERROR_LOG_LENGTH ( 256u )
#define DERIVATIVE_LOG_LENGTH ( 256u )

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static char message_queue[MESSAGE_QUEUE_LENGTH][MESSAGE_LENGTH] = {0};
static uint32_t message_queue_length = 0u;
static bool queue_is_full = false;
static bool queue_is_loaded = false;

// ADC Buffer
uint16_t adc1_buffer[ADC_BUFFER_LENGTH] = {0};
uint32_t adc1_buffer_idx = 0u;
bool adc1_buffer_is_full = false;

// Voltage values
float adc1_voltage[ADC_BUFFER_LENGTH] = {0.0f};

// If an RTOS was implemented, this may be replaced with a Semaphore
bool data_to_process = false;

float adjusted_duty_cycle = 0.0f;
// The voltage to achieve across the capacitor
float set_point_voltage = 3.6f;
float sample_voltage = 0.0f;

// Error
float error = 0.0f;
float error_log[ERROR_LOG_LENGTH] = {0.0f};
uint32_t error_log_idx = 0u;
float error_log_is_full = false;

// More error terms
float integral = 0.0f;
float derivative_log[DERIVATIVE_LOG_LENGTH] = {0.0f};
float derivative = 0.0f;
float composite = 0.0f;

// Weights
float k_integral = 100.0f;
float k_derivative = 100.0f;
float k_proportion = 100.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */


/*
 * HUART2
 */

static void prepare_message(char* message, uint32_t message_length, float data);
static void queue_message(char* message, uint32_t message_length);
static HAL_StatusTypeDef dispatch_message_from_huart2(char* message, uint32_t message_length);
static void empty_queue(void);


/*
 * ADC1
 */

static uint16_t poll_adc(void);
static void add_to_adc_buffer(uint16_t conversion_result);
static void process_adc_output_codes(void);
static void convert_data_to_messages_and_queue(void);

/*
 * Feedback
 */
static void add_to_error_log(float);
static float calculate_duty_cycle_from_error(float);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize data structures in the ADC module
  adc_init_adc_module();

  // Start PWM
  HAL_StatusTypeDef status = HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1 );
  assert( HAL_OK == status );

#ifdef TEST_MODE
  TIM1->CCR1 = pwm_calculate_CCRx( PWM_DUTY_CYCLE, PWM_ARRX );
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if( data_to_process )
	{
		// Sampling
		process_adc_output_codes();
		data_to_process = false;

#ifndef TEST_MODE
		// Feedback
		sample_voltage = adc1_voltage[ADC_BUFFER_LENGTH - 1u];
		error = ( set_point_voltage - sample_voltage );
		add_to_error_log( error );
		integral = math_helpers_trapezoid_approximation( error_log, error_log_idx );
		derivative = derivative_log[DERIVATIVE_LOG_LENGTH - 1u];
		composite = ( k_proportion * error ) + ( k_integral * integral ) - ( k_derivative * derivative );
		adjusted_duty_cycle = calculate_duty_cycle_from_error( composite );
		TIM1->CCR1 = pwm_calculate_CCRx( adjusted_duty_cycle, PWM_ARRX );
#endif

		// Data transfer
		convert_data_to_messages_and_queue();
		queue_is_loaded = true;
	}

	if( queue_is_loaded )
	{
		empty_queue();
		queue_is_loaded = false;
	}

	uint16_t conversion_result = poll_adc();
	add_to_adc_buffer( conversion_result );
	HAL_Delay( SAMPLE_PERIOD_MS );
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  uint32_t prescaler = pwm_calculate_prescaler( PWM_CLOCK_FREQUENCY_HZ, PWM_PULSE_FREQUENCY_HZ, PWM_ARRX );
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = prescaler;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */


/*
 * HUART2
 */


static void prepare_message(char* message, uint32_t message_length, float data)
{
	// Only accept values below 10.0f
	float dividend = data / ( 10.0f );
	assert( dividend < 10.0f );

	// Copy the data
	int sprintf_return = sprintf(message, "%.3f,", data);
	assert(sprintf_return == message_length);
}


static void queue_message(char* message, uint32_t message_length)
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


static HAL_StatusTypeDef dispatch_message_from_huart2(char* message, uint32_t message_length)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_UART_Transmit(&huart2, (uint8_t*) message, message_length, USART2_TIMEOUT_MS);
	return status;
}


static void empty_queue(void)
{
	HAL_StatusTypeDef status = HAL_OK;
	for( uint32_t i = 0u; i < message_queue_length; i++ )
	{
		status = dispatch_message_from_huart2(message_queue[i], MESSAGE_LENGTH);
		assert( HAL_OK == status );
	}

	// Reset message queue
	message_queue_length = 0u;
	queue_is_full = false;
}


/*
 * ADC1
 */


static uint16_t poll_adc(void)
{
	HAL_StatusTypeDef status = HAL_ADC_Start( &hadc1 );
	assert( HAL_OK == status );
	status = HAL_ADC_PollForConversion( &hadc1, ADC_TIMEOUT_MS );
	assert( HAL_OK == status );
	uint16_t conversion_result = ( uint16_t ) HAL_ADC_GetValue( &hadc1 );
	status = HAL_ADC_Stop( &hadc1 );
	return conversion_result;
}


static void add_to_adc_buffer(uint16_t conversion_result)
{
	if ( !adc1_buffer_is_full )
	{
	    adc1_buffer[adc1_buffer_idx++] = conversion_result;
	}

	if ( ADC_BUFFER_LENGTH == adc1_buffer_idx )
	{
		adc1_buffer_is_full = true;
		data_to_process = true;
	}
}


static void process_adc_output_codes(void)
{
	for( uint32_t i = 0u; i < ADC_BUFFER_LENGTH; i++ )
	{
		adc1_voltage[i] = adc_calculate_voltage_from_output_code( (uint32_t) adc1_buffer[i] );
	}

	// Reset adc1_buffer
	adc1_buffer_idx = 0u;
	adc1_buffer_is_full = false;
}


static void convert_data_to_messages_and_queue(void)
{
	char message[MESSAGE_LENGTH];
	for( uint32_t i = 0u; i < MESSAGE_QUEUE_LENGTH; i++ )
	{
		prepare_message(message, MESSAGE_LENGTH, adc1_voltage[i]);
		queue_message(message, MESSAGE_LENGTH);
	}
}


/*
 * Feedback
 */
static void add_to_error_log( float error )
{
	if( !error_log_is_full )
	{
	    error_log[error_log_idx++] = error;
	}

	if ( ERROR_LOG_LENGTH == error_log_idx )
	{
		error_log_is_full = true;
	}
}


static float calculate_duty_cycle_from_error( float error )
{
	assert( PWM_ARRX >= error );
	return error / PWM_ARRX;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while(true);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
