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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "BTT.h"
#include "fatfs_sd.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
char buffer[100];
char fileNames [20][20] = {0};
int allFilesCount = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void transmit_uart (char * msg){
	uint8_t len = strlen(msg);
	HAL_UART_Transmit (&huart2, (uint8_t*) msg, len, 200);
}
void listDirectory (){

	DIR dir;
    char *path;
    UINT BytesWritten;
    char string[20];
    FRESULT res;

    path = ""; // where you want to list

    res = f_opendir(&dir, path);

#ifdef DBG
    if (res != FR_OK)
      printf("res = %d f_opendir\n", res);
#endif

    if (res == FR_OK)
    {
      while(1)
      {
        FILINFO fno;

        res = f_readdir(&dir, &fno);

#ifdef DBG
        if (res != FR_OK)
          printf("res = %d f_readdir\n", res);
#endif

        if ((res != FR_OK) || (fno.fname[0] == 0))
          break;

        int size = strlen(fno.fname);
        if (fno.fname[size-3] == 't' && fno.fname[size-2] == 'x' && fno.fname[size-1] == 't' && fno.fname[0] != '.'){
        	sprintf(string, "%s\r\n", fno.fname);
        	transmit_uart(string);
        	if (c<20){
				strcpy(fileNames[c],string);
				allFilesCount++;
        	}
        }
      }
    }
}
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
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(500);

  fres = f_mount(&fs, "", 1);
   if (fres == FR_OK){
	   transmit_uart("Micro SD card is mounted successfully!\r\n");
   } else if (fres != FR_OK){
	   transmit_uart("Error Micro SD card mount\r\n");
   }

//   fres = f_open(&fil, "/log-file.txt", FA_OPEN_APPEND | FA_WRITE | FA_READ);
//   if (fres == FR_OK){
//   	   transmit_uart("File opened for reading and checking free space\r\n");
//      } else if (fres != FR_OK){
//   	   transmit_uart("Error opening file for reading\r\n");
//      }
//
//   fres = f_getfree("", &fre_clust, &pfs);
//   totalSpace = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
//   freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);
//   char mSz[12];
//   sprintf(mSz, "%lu", freeSpace);
//   if (fres == FR_OK){
//	   transmit_uart("The free sapce is: ");
//	   transmit_uart(mSz);
//	   transmit_uart("\r\n");
//   } else if (fres != FR_OK){
//   	   transmit_uart("Error getting free space\r\n");
//   }
//
//   f_puts("This text is written in the file.\r\n", &fil);
//
//   fres = f_close(&fil);
//   if (fres == FR_OK){
//   	   transmit_uart("File closed\n");
//      } else if (fres != FR_OK){
//      	   transmit_uart("Error closing file\r\n");
//      }
//
//   fres = f_open(&fil, "log-file.txt", FA_READ);
//   if (fres == FR_OK){
//   	   transmit_uart("File opened\r\n");
//      } else if (fres != FR_OK){
//      	   transmit_uart("Error opening file\r\n");
//      }
//
//   while (f_gets(buffer, sizeof(buffer), &fil)){
//	   char mRd[100];
//	   sprintf(mRd, "%s", buffer);
//	   transmit_uart(mRd);
//   }
//
//   fres = f_close(&fil);
//   if (fres == FR_OK){
//      	   transmit_uart("File closed\r\n");
//         } else if (fres != FR_OK){
//         	   transmit_uart("Error closing file\r\n");
//         }

   listDirectory();


   fres = f_mount(NULL, "", 1);
   if (fres == FR_OK){
      	   transmit_uart("Micro SD unmounted\r\n");
         } else if (fres != FR_OK){
         	   transmit_uart("Error unmounting SD card\r\n");
         }

   transmit_uart(fileNames[0]);
   transmit_uart(fileNames[1]);
   transmit_uart(fileNames[2]);
//   f_opendir(dp, "/")
//f_readdir(dp, fno)
// https://community.st.com/s/question/0D53W00000wzjSmSAI/fatfs-show-all-files
//f_opendir(dp, path)
//f_open(fp, path, mode)

















   /* instantiate a new object */
     BTT* btt = btt_new_default();

     /* specify which functions should recieve notificaions */
     btt_set_beat_tracking_callback   (btt, beat_detected_callback , NULL);

     int buffer_size = 64;
     dft_sample_t dft_buffer[buffer_size];






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
	   int counterFiles = 0;
	   while(1){

		   // Show on LCD the file -> fileNames[counterFiles]

		   if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 1){
			   break;
			}
			else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 1){
				counterFiles = (counterFiles+1)%allFilesCount;
			}
	   }

	   fres = f_open(&fil, fileNames[counterFiles], FA_READ);
	   if (fres == FR_OK){
		   transmit_uart("File opened\r\n");
	   } else if (fres != FR_OK){
		   transmit_uart("Error opening file\r\n");
	   }

	  while (f_gets(buffer, sizeof(buffer), &fil)){
	  /*
	   * TASK1 1. Read from music file
	   * TASK1 2. Start beat analysis (callback releases binary semaph for training)
	   * TASK1 3. Play music file
	   * TASK1 4. Write to log file the metrics OR LCD
	   * TASK2 5. Training (light LED and check button pressed and calculate metrics accordingly)
	   *
	   */
		   char mRd[100];
		   sprintf(mRd, "%s", buffer);
		   transmit_uart(mRd);

		   /* Fill a buffer with your audio samples here then pass it to btt */
		   btt_process(btt, dft_buffer, buffer_size);


		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  fres = f_close(&fil);
	  if (fres == FR_OK){
		   transmit_uart("File closed\r\n");
	  } else if (fres != FR_OK){
		   transmit_uart("Error closing file\r\n");
	  }


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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

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

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
/*--------------------------------------------------------------------*/
void beat_detected_callback (void* SELF, unsigned long long sample_time)
{
  //called when beat was detected
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

