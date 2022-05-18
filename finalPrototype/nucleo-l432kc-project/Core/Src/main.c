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

#include "fatfs_sd.h"
//#include "DFPLAYER_MINI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN(a,b) (((a)<(b))? (a):(b))
#define BPS 1
#define DEBUG 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
char buffer[100];
char fileNames [20][20] = {0};
int allFilesCount = 0;

int *beats;
int appendBeat = 0;
int beatsSize = 1;

int counterSong = 0;
int counterBeat = 0;

char timeLCD [3] = {0}; //testing

volatile uint16_t timer_end;
uint16_t timer_val;
uint16_t time_diff;
uint16_t time_diff_arr[50];
int rand_num;
int flag;
int state = 1;
int detected = 0;
int counter = 0;
char buff [100] = {0};

int fileSelected = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void startTraining();
void training ();
void beat_detected_callback (void* SELF, unsigned long long sample_time);
void transmit_uart (char * msg, int x){
	if (x){
		uint8_t len = strlen(msg);
		HAL_UART_Transmit (&huart2, (uint8_t*) msg, len, 200);
	}
}
void listDirectory (){

	DIR dir;
    char *path;
    UINT BytesWritten;
    char string[20];
    FRESULT res;

    path = ""; // where you want to list

    res = f_opendir(&dir, path);

//    if (res != FR_OK)
//      printf("res = %d f_opendir\n", res);

    if (res == FR_OK)
    {
      while(1)
      {
        FILINFO fno;

        res = f_readdir(&dir, &fno);

//        if (res != FR_OK)
//          printf("res = %d f_readdir\n", res);

        if ((res != FR_OK) || (fno.fname[0] == 0))
          break;

        int size = strlen(fno.fname);
        sprintf(string, "%s\r\n", fno.fname);
                	transmit_uart(string, 0);
        if (fno.fname[size-3] == 'w' && fno.fname[size-2] == 'a' && fno.fname[size-1] == 'v' && fno.fname[0] != '.'){
//        	sprintf(string, "%s\r\n", fno.fname);
//        	transmit_uart(string);
        	if (allFilesCount<20){
				strcpy(fileNames[allFilesCount],string);
				allFilesCount++;
        	}
        }
      }
    }
}
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
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_DAC1_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(200);

  transmit_uart("Now playing       ", 1);
  HAL_Delay(2000);

  char fileStr [16] = {0};
  while(1){
	  memset(fileStr, 0, 16);
	  sprintf(fileStr, "00%d", fileSelected);
//	  transmit_uart(fileStr, 1);
	  HAL_UART_Transmit(&huart2, fileStr, 12, 1000);
	  HAL_Delay(500);
	   // Show on LCD the file -> fileNames[counterFiles]
	   if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1){
		   break;
		}
		else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 1){
			fileSelected=(fileSelected+1)%4;
		}
   }

  fileStr[3] = '.';
  fileStr[4] = 't';
  fileStr[5] = 'x';
  fileStr[6] = 't';
  transmit_uart(fileStr, DEBUG);

  HAL_Delay(500);

  fres = f_mount(&fs, "", 1);
   if (fres == FR_OK){
	   transmit_uart("Micro SD card is mounted\r\n", DEBUG);
   } else if (fres != FR_OK){
	   transmit_uart("Error Micro SD card mount\r\n", DEBUG);
   }


   listDirectory();
//   transmit_uart(fileNames[0]);
   fres = f_open(&fil, fileStr, FA_READ);
   if (fres == FR_OK){
	   transmit_uart("File opened\r\n", DEBUG);
   } else if (fres != FR_OK){
	   transmit_uart("Error opening file\r\n", DEBUG);
   }

   f_gets(buffer, sizeof(buffer), &fil);
   sscanf(buffer, "%d", &beatsSize);
   beats = (int*) calloc(beatsSize, sizeof(int));

   while (f_gets(buffer, sizeof(buffer), &fil)){
   	  char mRd[10];
   	  int number;
   	  sscanf(buffer, "%d", &number);
   	  beats[appendBeat] = number;
	  appendBeat++;
//   	  sprintf(mRd, "%d", number);
//   	  transmit_uart(mRd);
   }

   fres = f_close(&fil);
   if (fres == FR_OK){
   	  transmit_uart("File closed\r\n", DEBUG);
   } else if (fres != FR_OK){
   	  transmit_uart("Error closing file\r\n", DEBUG);
   }

   fres = f_mount(NULL, "", 1);
   if (fres == FR_OK){
	   transmit_uart("Micro SD unmounted\r\n", DEBUG);
   } else if (fres != FR_OK){
   	  transmit_uart("Error unmounting SD card\r\n", DEBUG);
   }

//   HAL_TIM_Base_Start(&htim2);
   HAL_TIM_Base_Start_IT(&htim2);
   HAL_TIM_Base_Start(&htim16);


   DF_Init(30);
   DF_PlayFromStart();
//   DF_Pause();
   int i;
   for (i=1; i<fileSelected; i++){
	   DF_Next();
	   DF_Next();
   }


   //     DF_PlayFrom(1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  	  Check_Key();

	  if (beats[counterBeat] == counterSong){
		  //beat detected (start training)
		  transmit_uart("Training: ", 1);
		  startTraining();
		  sprintf(timeLCD, "%d\n", counterSong);
		  transmit_uart(timeLCD, 1);
		  counterBeat++;
	  } else if (counterSong > beats[counterBeat]) counterBeat++;



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
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
//  htim2.Init.Period = 1000/BPS;
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000/BPS;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim16.Init.Prescaler = 31999;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED0_Pin|LED1_Pin|LED2_Pin|LCD7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pins : LED0_Pin LED1_Pin LED2_Pin LCD7_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|LED2_Pin|LCD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void startTraining(){

	rand_num = rand()%3;
	__HAL_TIM_SET_COUNTER(&htim16, 0);
	timer_val = __HAL_TIM_GET_COUNTER(&htim16);
	if (rand_num == 0)
		HAL_GPIO_WritePin(GPIOA, LED0_Pin, 1);
	else if (rand_num == 1)
		HAL_GPIO_WritePin(GPIOA, LED1_Pin,1);
	else if (rand_num == 2)
		HAL_GPIO_WritePin(GPIOA, LED2_Pin, 1);

	state = 1;
	flag = 0;
	while (state == 1){
		if (rand_num == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 1){
			HAL_GPIO_WritePin(GPIOA, LED0_Pin, 0);
			timer_end = __HAL_TIM_GET_COUNTER(&htim16);
			state = 0;
			flag = 1;
		}
		else if (rand_num == 1 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1){
			HAL_GPIO_WritePin(GPIOA, LED1_Pin,0);
			timer_end = __HAL_TIM_GET_COUNTER(&htim16);
			state = 0;
			flag = 1;
		}
		else if (rand_num == 2 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 1){
			HAL_GPIO_WritePin(GPIOA, LED2_Pin, 0);
			timer_end = __HAL_TIM_GET_COUNTER(&htim16);
			state = 0;
			flag = 1;
		}
	}

	HAL_GPIO_WritePin(GPIOA, LED0_Pin, 0);
	HAL_GPIO_WritePin(GPIOA, LED1_Pin, 0);
	HAL_GPIO_WritePin(GPIOA, LED2_Pin, 0);

	if (flag == 0) {
		timer_end = __HAL_TIM_GET_COUNTER(&htim16);
		time_diff  = timer_end - timer_val;
	}
	else {
		time_diff  = timer_end - timer_val;
	}

	time_diff_arr[counter] = time_diff;
	counter++;


	sprintf(buff, "%d: %u    \r\n", counter, time_diff);
	HAL_UART_Transmit(&huart2, (uint8_t*) buff, sizeof(buff), 100);
}

void training (){
	 flag = 0;
	 if (counter < 50 ){
		 rand_num = rand()%3;
		 __HAL_TIM_SET_COUNTER(&htim16, 0);
		 timer_val = __HAL_TIM_GET_COUNTER(&htim16);
		 if (rand_num == 0)
			  HAL_GPIO_WritePin(GPIOA, LED0_Pin, 1);
		 else if (rand_num == 1)
			  HAL_GPIO_WritePin(GPIOA, LED1_Pin,1);
		 else if (rand_num == 2)
			  HAL_GPIO_WritePin(GPIOA, LED2_Pin, 1);

	  state = 1;
	  flag = 0;
	  HAL_TIM_Base_Start_IT(&htim1);
	  while (state == 1){
		if (rand_num == 0 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 1){
			HAL_GPIO_WritePin(GPIOA, LED0_Pin, 0);
			timer_end = __HAL_TIM_GET_COUNTER(&htim16);
			state = 0;
			flag = 1;
			HAL_TIM_Base_Stop_IT(&htim1);
			__HAL_TIM_SET_COUNTER(&htim1, 0);
		}
		else if (rand_num == 1 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1){
			HAL_GPIO_WritePin(GPIOA, LED1_Pin,0);
			timer_end = __HAL_TIM_GET_COUNTER(&htim16);
			state = 0;
			flag = 1;
			HAL_TIM_Base_Stop_IT(&htim1);
			__HAL_TIM_SET_COUNTER(&htim1, 0);
		}
		else if (rand_num == 2 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 1){
			HAL_GPIO_WritePin(GPIOA, LED2_Pin, 0);
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

	  if (flag == 0) {
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
}
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim){

	if (htim == &htim2){
		counterSong++;
		state = 0;
		__HAL_TIM_SET_COUNTER(&htim2, 0);
	} else if (htim == &htim1){
		HAL_TIM_Base_Stop_IT(&htim1);
		state = 0;
	}
}
//void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim){
//
//
//	if (htim == &htim1){
//		HAL_TIM_Base_Stop_IT(&htim1);
//		state = 0;
//	}
//}
/*--------------------------------------------------------------------*/
void beat_detected_callback (void* SELF, unsigned long long sample_time)
{


  detected++;
//    printf(" count : %d , detected: %d \n", count , detected );

  /*MKAiff* beat_aiff = (MKAiff*) SELF;
  aiffSetPlayheadToSamples     (beat_aiff, sample_time);
  float click = 1;
  aiffAddFloatingPointSamplesAtPlayhead(beat_aiff, &click, 1, aiffFloatSampleType, aiffYes);*/
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

