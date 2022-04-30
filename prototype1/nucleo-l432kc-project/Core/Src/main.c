/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint16_t timer_end;
uint16_t timer_val;
uint16_t time_diff;
uint16_t time_diff_arr[50];
int rand_num ;
int flag;
int state = 1;
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
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int counter = 0;
  int sum = 0;
  int average = 0;
  char buff [100] = {0};
  int once = 1;

  HAL_Delay(5000);
  uint16_t end_time, start_time = __HAL_TIM_GET_COUNTER(&htim16);
  while (1)
  {
	  flag = 0;
	  if (counter < 50 ){
		  rand_num = rand()%6;
		  timer_val = __HAL_TIM_GET_COUNTER(&htim16);
		  if (rand_num == 0)
			  HAL_GPIO_WritePin(GPIOA, LED0_Pin, 1);
		  else if (rand_num == 1)
			  HAL_GPIO_WritePin(GPIOA, LED1_Pin,1);
		  else if (rand_num == 2)
				  HAL_GPIO_WritePin(GPIOA, LED2_Pin, 1);
		  else if (rand_num == 3)
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
		  else if (rand_num == 4)
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
		  else if (rand_num == 5)
				  HAL_GPIO_WritePin(GPIOA, LED5_Pin, 1);

//		  HAL_Delay(5000);
		  state = 1;
		  flag = 0;
		  HAL_TIM_Base_Start_IT(&htim1);
		  while (state == 1){
			if (rand_num == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 1){
				HAL_GPIO_WritePin(GPIOA, LED0_Pin, 0);
				timer_end = __HAL_TIM_GET_COUNTER(&htim16);
				state = 0;
				flag = 1;
				HAL_TIM_Base_Stop_IT(&htim1);
				__HAL_TIM_SET_COUNTER(&htim1, 0);
			}
			else if (rand_num == 1 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 1){
				HAL_GPIO_WritePin(GPIOA, LED1_Pin,0);
				timer_end = __HAL_TIM_GET_COUNTER(&htim16);
				state = 0;
				flag = 1;
				HAL_TIM_Base_Stop_IT(&htim1);
				__HAL_TIM_SET_COUNTER(&htim1, 0);
			}
			else if (rand_num == 2 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1){
				HAL_GPIO_WritePin(GPIOA, LED2_Pin, 0);
				timer_end = __HAL_TIM_GET_COUNTER(&htim16);
				state = 0;
				flag = 1;
				HAL_TIM_Base_Stop_IT(&htim1);
				__HAL_TIM_SET_COUNTER(&htim1, 0);
			}
			else if (rand_num == 3 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 1){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
				timer_end = __HAL_TIM_GET_COUNTER(&htim16);
				state = 0;
				flag = 1;
				HAL_TIM_Base_Stop_IT(&htim1);
				__HAL_TIM_SET_COUNTER(&htim1, 0);
			}
			else if (rand_num == 4 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 1){
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
				timer_end = __HAL_TIM_GET_COUNTER(&htim16);
				state = 0;
				flag = 1;
				HAL_TIM_Base_Stop_IT(&htim1);
				__HAL_TIM_SET_COUNTER(&htim1, 0);
			}
			else if (rand_num == 5 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == 1){
				HAL_GPIO_WritePin(GPIOA, LED5_Pin, 0);
				timer_end = __HAL_TIM_GET_COUNTER(&htim16);
				state = 0;
				flag = 1;
				HAL_TIM_Base_Stop_IT(&htim1);
				__HAL_TIM_SET_COUNTER(&htim1, 0);
			}
		  }
		  HAL_TIM_Base_Stop_IT(&htim1);

		  HAL_GPIO_WritePin(GPIOA, LED0_Pin, 0);
		  HAL_GPIO_WritePin(GPIOA, LED1_Pin, 0);
		  HAL_GPIO_WritePin(GPIOA, LED2_Pin, 0);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
		  HAL_GPIO_WritePin(GPIOA, LED5_Pin, 0);

		  if (flag == 0) {
//			  time_diff = 5000;
			  timer_end = __HAL_TIM_GET_COUNTER(&htim16);
			  time_diff  = timer_end - timer_val;
		  }
		  else {
			  time_diff  = timer_end - timer_val;
		  }

		  time_diff_arr[counter] = time_diff;
		  counter++;

		  HAL_Delay(1000);

		  sprintf(buff, "%d: %u    \r\n", counter, time_diff);
		  HAL_UART_Transmit(&huart2, (uint8_t*) buff, sizeof(buff), 100);
	  }
	  else {
		  if (once == 1){
			  end_time = __HAL_TIM_GET_COUNTER(&htim16);
			  for (int i = 0 ; i <50 ; i++){
				 sum = sum + time_diff_arr[i];
			  }
			  average = sum / 50 ;
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);

			  sprintf(buff, "Done: Time = %u  \r\nAverage: %d    \r\n", end_time-start_time, average);
			  HAL_UART_Transmit(&huart2, (uint8_t*) buff, sizeof(buff), 100);

			  once = 0;
		  }
	  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 31999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 31;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED0_Pin|LED1_Pin|LED2_Pin|LED5_Pin
                          |GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED0_Pin LED1_Pin LED2_Pin LED5_Pin
                           PA9 PA10 */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|LED2_Pin|LED5_Pin
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB4 PB5
                           PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
////	flag = 1;
////	if (rand_num == 0 && GPIO_Pin == PB_0_Pin){
////		HAL_GPIO_WritePin(GPIOA, LED0_Pin, 0);
////		timer_end = __HAL_TIM_GET_COUNTER(&htim16);
////	}
////	else if (rand_num == 1 && GPIO_Pin == PB_1_Pin){
////		HAL_GPIO_WritePin(GPIOA, LED1_Pin,0);
////		timer_end = __HAL_TIM_GET_COUNTER(&htim16);
////	}
////	else if (rand_num == 2 && GPIO_Pin == PB_2_Pin){
////		HAL_GPIO_WritePin(GPIOA, LED2_Pin, 0);
////		timer_end = __HAL_TIM_GET_COUNTER(&htim16);
////	}
////	else if (rand_num == 3 && GPIO_Pin == PB_3_Pin){
////		HAL_GPIO_WritePin(GPIOA, LED3_Pin, 0);
////		timer_end = __HAL_TIM_GET_COUNTER(&htim16);
////	}
////	else if (rand_num == 4 && GPIO_Pin == PB_4_Pin){
////		HAL_GPIO_WritePin(GPIOA, LED4_Pin, 0);
////		timer_end = __HAL_TIM_GET_COUNTER(&htim16);
////	}
////	else if (rand_num == 5 && GPIO_Pin == PB_5_Pin){
////		HAL_GPIO_WritePin(GPIOA, LED5_Pin, 0);
////		timer_end = __HAL_TIM_GET_COUNTER(&htim16);
////	}
////	else {
////		timer_end = __HAL_TIM_GET_COUNTER(&htim16) + 5000;
////	}
//
//
//	if (rand_num == 0 && GPIO_Pin == PB_0_Pin && state == 1){
//		HAL_TIM_Base_Start_IT(&htim1);
//		state = 0;
//	}
//	else if (rand_num == 1 && GPIO_Pin == PB_1_Pin && state == 1){
//		HAL_TIM_Base_Start_IT(&htim1);
//		state = 0;
//	}
//	else if (rand_num == 2 && GPIO_Pin == PB_2_Pin && state == 1){
//		HAL_TIM_Base_Start_IT(&htim1);
//		state = 0;
//	}
//	else if (rand_num == 3 && GPIO_Pin == PB_3_Pin && state == 1){
//		HAL_TIM_Base_Start_IT(&htim1);
//		state = 0;
//	}
//	else if (rand_num == 4 && GPIO_Pin == PB_4_Pin && state == 1){
//		HAL_TIM_Base_Start_IT(&htim1);
//		state = 0;
//	}
//	else if (rand_num == 5 && GPIO_Pin == PB_5_Pin && state == 1){
//		HAL_TIM_Base_Start_IT(&htim1);
//		state = 0;
//	}
//
//}
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim){

//	if (HAL_GPIO_ReadPin())
//	flag = 1;
//	if (rand_num == 0 && HAL_GPIO_ReadPin(GPIOB, PB_0_Pin) == 0){
//		HAL_GPIO_WritePin(GPIOA, LED0_Pin, 0);
//		timer_end = __HAL_TIM_GET_COUNTER(&htim16);
//		state = 1;
//	}
//	else if (rand_num == 1 && HAL_GPIO_ReadPin(GPIOB, PB_1_Pin) == 0){
//		HAL_GPIO_WritePin(GPIOA, LED1_Pin,0);
//		timer_end = __HAL_TIM_GET_COUNTER(&htim16);
//		state = 1;
//	}
//	else if (rand_num == 2 && HAL_GPIO_ReadPin(GPIOB, PB_2_Pin) == 0){
//		HAL_GPIO_WritePin(GPIOA, LED2_Pin, 0);
//		timer_end = __HAL_TIM_GET_COUNTER(&htim16);
//		state = 1;
//	}
//	else if (rand_num == 3 && HAL_GPIO_ReadPin(GPIOB, PB_3_Pin) == 0){
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
//		timer_end = __HAL_TIM_GET_COUNTER(&htim16);
//		state = 1;
//	}
//	else if (rand_num == 4 && HAL_GPIO_ReadPin(GPIOB, PB_4_Pin) == 0){
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
//		timer_end = __HAL_TIM_GET_COUNTER(&htim16);
//		state = 1;
//	}
//	else if (rand_num == 5 && HAL_GPIO_ReadPin(GPIOB, PB_5_Pin) == 0){
//		HAL_GPIO_WritePin(GPIOA, LED5_Pin, 0);
//		timer_end = __HAL_TIM_GET_COUNTER(&htim16);
//		state = 1;
//	}
//	else {
//		timer_end = __HAL_TIM_GET_COUNTER(&htim16) + 5000;
//		state = 1;
//	}
	if (htim == &htim1){
		HAL_TIM_Base_Stop_IT(&htim1);
		state = 0;
	}
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

