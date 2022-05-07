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
#include "waveheader.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*funcP)(uint8_t channels, uint16_t numSamples, void *pIn, uint16_t *pOutput);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFSIZE 512
#define AUDIO_BUFFER_SIZE 512
#define MIN(a,b) (((a)<(b))? (a):(b))
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

static uint8_t fileBuffer[BUFSIZE];
static uint8_t dmaBuffer[2][BUFSIZE];
static uint8_t dmaBank = 0;

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
/* USER CODE BEGIN PFP */
void training ();
void beat_detected_callback (void* SELF, unsigned long long sample_time);
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
                	transmit_uart(string);
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
static void
setSampleRate(uint16_t freq)
{
  uint16_t period = (80000000 / freq) - 1;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = period;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim2);
}

static inline uint16_t
val2Dac8(int32_t v)
{
  uint16_t out = v << 3;
  return out;
}

static inline uint16_t
val2Dac16(int32_t v)
{
  v >>= 4;
  v += 2047;
  return v & 0xfff;
}

static void
prepareDACBuffer_8Bit(uint8_t channels, uint16_t numSamples, void *pIn, uint16_t *pOutput)
{
  uint8_t *pInput = (uint8_t *)pIn;

  for (int i=0; i<numSamples; i++) {
    int32_t val = 0;

    for(int j=0; j<channels; j++) {
      val += *pInput++;
    }
    val /= channels;
    *pOutput++ = val2Dac8(val);
  }
}

static void
prepareDACBuffer_16Bit(uint8_t channels, uint16_t numSamples, void *pIn, uint16_t *pOutput)
{
  int16_t *pInput = (int16_t *)pIn;

  for (int i=0; i<numSamples; i++) {
    int32_t val = 0;

    for(int j=0; j<channels; j++) {
      val += *pInput++;
    }
    val /= channels;
    *pOutput++ = val2Dac16(val);
  }
}



static uint8_t
isSupprtedWavFile(const struct Wav_Header *header)
{
  if (strncmp(header->riff, "RIFF", 4 ) != 0)
    return 0;

  if (header->vfmt != 1)
    return 0;

  if (strncmp(header->dataChunkHeader, "data", 4 ) != 0)
    return 0;

  return 1;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  /* USER CODE BEGIN 2 */

  // SETUP LCD
//   // Lcd_PortType ports[] = { D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port };
//   Lcd_PortType ports[] = { GPIOA, GPIOA, GPIOA, GPIOA };
//   // Lcd_PinType pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};
//   Lcd_PinType pins[] = {LCD4_Pin, LCD5_Pin, LCD6_Pin, LCD7_Pin};
//   Lcd_HandleTypeDef lcd;
//
//   // Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin, LCD_4_BIT_MODE);
//   lcd = Lcd_create(ports, pins, LCDRS_GPIO_Port, LCDRS_Pin, LCDE_GPIO_Port, LCDE_Pin, LCD_4_BIT_MODE);
//   Lcd_cursor(&lcd, 0,1);
//   Lcd_string(&lcd, "Welcome");

  HAL_Delay(500);

  fres = f_mount(&fs, "", 1);
   if (fres == FR_OK){
	   transmit_uart("Micro SD card is mounted successfully!\r\n");
   } else if (fres != FR_OK){
	   transmit_uart("Error Micro SD card mount\r\n");
   }

   listDirectory();


   transmit_uart(fileNames[0]);
//   transmit_uart(fileNames[1]);
//   transmit_uart(fileNames[2]);
//   f_opendir(dp, "/")
//f_readdir(dp, fno)
// https://community.st.com/s/question/0D53W00000wzjSmSAI/fatfs-show-all-files
//f_opendir(dp, path)
//f_open(fp, path, mode)


     // WAV Player
     HAL_TIM_Base_Start(&htim2);


  HAL_TIM_Base_Start(&htim16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int sum = 0;
  int average = 0;
  int once = 1;
  int count = 0;


//   Lcd_cursor(&lcd, 1,1);
//   Lcd_string(&lcd, "Let's Begin");
//
//   HAL_Delay(2000);
//
//   Lcd_cursor(&lcd, 0,1);
//   Lcd_string(&lcd, "Song name here");
//
//   Lcd_cursor(&lcd, 1,0);
//   Lcd_string(&lcd, "Btn2: play / Btn1: next");

  HAL_Delay(5000);
  uint16_t end_time, start_time = __HAL_TIM_GET_COUNTER(&htim16);

  //till now, you have all file names on mounted sd card stored in fileNames and their count in allFilesCount
  //	  /*
  //	   * TASK1 1. Read from music file
  //	   * TASK1 2. Start beat analysis (callback releases binary semaph for training)
  //	   * TASK1 3. Play music file
  //	   * TASK1 4. Training (light LED and check button pressed and calculate metrics accordingly)
  //	   */
  while (1)
  {
	  //make user choose a file from the list of files on sd card
	   int counterFiles = 0;
//	   while(1){
//
//		   // Show on LCD the file -> fileNames[counterFiles]
////		   Lcd_cursor(&lcd, 0,0);
////		   Lcd_int(&lcd, counterFiles);
////
////		   Lcd_cursor(&lcd, 0,2);
////		   Lcd_string(&lcd, fileNames[counterFiles]);
//
//		   if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1){
//			   break; //valid choice of song (read it and process it) -> counterFiles has the song index
//			}
//			else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 1){
//				counterFiles++;
//			}
//		   counterFiles = counterFiles%allFilesCount;
//	   }


	   //open file and read header
	   fres = f_open(&fil, fileNames[counterFiles], FA_READ);
	   if (fres == FR_OK){
		   transmit_uart("File opened\r\n");
	   } else if (fres != FR_OK){
		   break;
	   }

	   //pre-process the file and load beats using BTT
	   int* beats ;
	   beats = (int *)calloc (300 , sizeof(int) );

	   size_t bytesRead = 0;
	   dft_sample_t* bufferf = calloc(AUDIO_BUFFER_SIZE/8, sizeof(*bufferf));
	   char buffb [512] = {0};

	   BTT* btt = btt_new_default();
	   btt_set_beat_tracking_callback   (btt, beat_detected_callback , NULL);

	   char debug [100] = {0};
	   // read up to sizeof(buffer) bytes
	   while ((f_read(&fil, buffb, AUDIO_BUFFER_SIZE, &bytesRead)) != FR_OK){
		   if (bytesRead > 0){
		     count++;
		     for(int i=0; i<AUDIO_BUFFER_SIZE/8; i++)
			 {
			   memcpy (&bufferf[i], buffb+(i*8), sizeof (float));
//			   sprintf(debug, "%f\r\n", bufferf[i]);
//			   transmit_uart(debug);
			   bufferf[i] = bufferf[i] / (long double)0x7FFFFFFF;
			 }
		     btt_process(btt, bufferf, AUDIO_BUFFER_SIZE/8);
		     beats[detected] = count;
		   }
		   else {
			   break;
		   }
	   }

	   //read header
       struct Wav_Header header;
       int countHeader = 0;
       //rewind file pointer to the start of the file
	   fres = f_lseek(&fil, 0);
	   fres = f_read(&fil, &header, sizeof(struct Wav_Header), &countHeader);

	   if (fres != FR_OK)
	   {
		  //error reading header
		  transmit_uart("Error reading header\r\n");
		  break;
	   }
	   if (!isSupprtedWavFile(&header))
	   {
		   //error unsupported file
           transmit_uart("Error unsupported file\r\n");
           break;
	   }

	   //set sample rate
	   setSampleRate(header.sampleFreq);
	   const uint8_t channels = header.channels;
	   const uint8_t bytesPerSample = header.bitsPerSample / 8;

       funcP prepareData = (header.bitsPerSample == 8)? prepareDACBuffer_8Bit : prepareDACBuffer_16Bit;
       uint32_t bytes_last = header.dataChunkLength;


       //////////////////////////////BEAT


       int index = 0;
	   int beats_index = 0;
	   int beat = beats[0];

	   transmit_uart("Beginning playing song\r\n");

	   HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

	   while(0 < bytes_last) {

		   int blksize = (header.bitsPerSample == 8)? MIN(bytes_last, BUFSIZE / 2) : MIN(bytes_last, BUFSIZE);

		   UINT bytes_read;
		   FRESULT res;

		   res = f_read(&fil, fileBuffer, blksize, &bytes_read);
		   if (res != FR_OK || bytes_read == 0)
			   break;

		   uint16_t numSamples = bytes_read / bytesPerSample / channels;
		   int16_t     *pInput = (int16_t *)fileBuffer;
		   uint16_t   *pOutput = (uint16_t *)dmaBuffer[dmaBank];

		   prepareData(channels, numSamples, pInput, pOutput);

//		   HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dmaBuffer[dmaBank], numSamples, DAC_ALIGN_12B_R);
//		   for (int j=0; j<numSamples; j++){
//			   HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, *dmaBuffer[j]);
//			   HAL_Delay(0.001);
//		   }
//		   HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, *dmaBuffer[dmaBank]);
//		   DAC1->DHR12R1 = *dmaBuffer[dmaBank];
		   if (beat == index ){
			   //get rand(), pick a random LED
//			   training();
			   beats_index++;
			   beats_index = beats_index % 300;
			   beat = beats[beats_index];
		   }
//		   dmaBank = !dmaBank;
		   bytes_last -= blksize;
		   index++;
	   }
	   transmit_uart("Done\r\n");

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  fres = f_close(&fil);
//	  if (fres == FR_OK){
//		   transmit_uart("File closed\r\n");
//	  } else if (fres != FR_OK){
//		   transmit_uart("Error closing file\r\n");
//	  }


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

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  HAL_GPIO_WritePin(GPIOA, LED0_Pin|LED1_Pin|LED2_Pin|LCD4_Pin
                          |LCD5_Pin|LCD6_Pin|LCD7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pins : LED0_Pin LED1_Pin LED2_Pin LCD4_Pin
                           LCD5_Pin LCD6_Pin LCD7_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|LED2_Pin|LCD4_Pin
                          |LCD5_Pin|LCD6_Pin|LCD7_Pin;
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


	if (htim == &htim1){
		HAL_TIM_Base_Stop_IT(&htim1);
		state = 0;
	}
}
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

