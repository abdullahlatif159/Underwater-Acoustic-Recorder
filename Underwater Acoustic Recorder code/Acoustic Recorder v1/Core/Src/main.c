/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>
#include <time.h>
#include <arm_math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//struct for .WAV header
typedef struct {
    char    ChunkID[4];
    int32_t ChunkSize;
    char    Format[4];

    char    Subchunk1ID[4];
    int32_t Subchunk1Size;
    int16_t AudioFormat;
    int16_t NumChannels;
    int32_t SampleRate;
    int32_t ByteRate;
    int16_t BlockAlign;
    int16_t BitsPerSample;

    char    Subchunk2ID[4];
    int32_t Subchunk2Size;
} header;

//state machine states
typedef enum {
    PRE_ANALYSIS_RECORDING,
    MAIN_RECORDING,
    IDLE
} application_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//.WAV file defines, currently 16-bit/192kHz
#define Sub_chunk1Size 16
#define Audio_Format 1
#define Num_Channels 1
#define Sample_Rate 192000 //32.5cycles
#define Bitsper_Sample 16
#define duration 5
#define CHUNK_SIZE_SECONDS 0.20 //must be able to divide duration
#define CHUNK_SIZE_SAMPLES (CHUNK_SIZE_SECONDS * Sample_Rate)

//used for sine wave generation to test .WAV setup.
#define amplitude 0.5
#define frequency 415.30

#define PRE_ANALYSIS_BUF_LEN 8192 //4096  //closest power of 2 to 0.1 seconds of data at 192kHz
#define FFT_SIZE PRE_ANALYSIS_BUF_LEN

//adjust below parameters to alter pre analysis parameters
#define FREQUENCY_THRESHOLD 18000 //should add 4,449.4 to wanted value to account for error.
#define AMPLITUDE_THRESHOLD 0 //ambient room noise ranges from 0-10.
//26449.40 - 22000 = 4,449.4 error

#define ADC_BUF_LEN 32768
#define ADC_BUF_LEN_HALF ADC_BUF_LEN/2


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_HS;

/* USER CODE BEGIN PV */
header header_actual; //declare .WAV header globally

uint16_t adc_buf[ADC_BUF_LEN];
uint16_t adc_half_buf[ADC_BUF_LEN_HALF];
uint16_t adc_full_buf[ADC_BUF_LEN_HALF];

volatile uint8_t dma_half_transfer_complete = 0;
volatile uint8_t dma_transfer_complete = 0;

uint16_t pre_analysis_buf[PRE_ANALYSIS_BUF_LEN];
volatile uint8_t pre_analysis_complete = 0; //flag to indicate completion of pre-analysis recording
volatile application_state_t app_state = PRE_ANALYSIS_RECORDING;
q15_t  fft_input[FFT_SIZE]; //buffer for FFT input
q15_t  fft_output[2 * FFT_SIZE]; //should be (x*2)+1

uint32_t totalDataBytesWritten = 0;
UINT bw; //bytes written

RTC_TimeTypeDef gTime;
RTC_DateTypeDef gDate;

FIL AudioFile;
FIL TextFile;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_OTG_HS_PCD_Init(void);
/* USER CODE BEGIN PFP */
void sd_setup(void);
void sd_write(short *data, int num_samples);
void initializeHeader(header *header_actual);
void generateSineWave(short *data, int num_samples);
void writeToFile(const char *filename, header header_actual, short *data);
void finalizeRecording();
void Get_RTC_Time(void);
void SetRTCToCompileTime(RTC_HandleTypeDef *hrtc);
void sd_audio_file();
void sd_data_file();
int detectSignificantFrequency(float32_t *fftMagnitude, uint32_t fftSize, float sampleRate);
void performFFT();
q15_t unsigned_to_q15(uint16_t val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FRESULT res;
uint32_t byteswritten, bytesread;
uint8_t wtext[] = "This is a test for my 3rd Year Project!"; /* File write buffer */
uint8_t rtext[_MAX_SS];/* File read buffer */

arm_rfft_instance_q15 S;

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SDMMC1_SD_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_HS_PCD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  //sets rtc to complie time, easier than changing code each build
  SetRTCToCompileTime(&hrtc);
  //used for debugging check time is correct.
  Get_RTC_Time();
  sd_setup();
  sd_audio_file();
  sd_data_file();

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)pre_analysis_buf, PRE_ANALYSIS_BUF_LEN);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (app_state == PRE_ANALYSIS_RECORDING) {
	        if (pre_analysis_complete) {

	        	//calculating DC offset from audio
	        	float dc_offset = 0;
	        	for (int i = 0; i < PRE_ANALYSIS_BUF_LEN; i++) {
	        	    dc_offset += pre_analysis_buf[i];
	        	}
	        	dc_offset /= PRE_ANALYSIS_BUF_LEN;

	            //converts the ADC values to q15 and store them in fft_input for use in q15 fft.
	            for (int i = 0; i < PRE_ANALYSIS_BUF_LEN; i++) {
	                fft_input[i] = unsigned_to_q15(pre_analysis_buf[i]- dc_offset); //removing  DC offset from audio
	            }

	            //perform FFT on the pre-analysis data
	            performFFT();

	            //analyze the FFT output to detect significant frequencies as defined in #DEFINE
	            if (detectSignificantFrequency(fft_output, FFT_SIZE, Sample_Rate)) {
	                app_state = MAIN_RECORDING; //starts the main recording if significant frequency detected
		            pre_analysis_complete = 0;

	            } else {
	                //app_state = IDLE; //IDLE state here if needed, current code should loop
	                app_state = PRE_ANALYSIS_RECORDING;

	            }

	        }
	    } else if (app_state == MAIN_RECORDING) {

	    	//stop pre analysis ADC DMA setup and restarts for main recording. Output buffer is different
	    	HAL_ADC_Stop_DMA(&hadc1);
	    	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

	    		header header_actual;
	    		initializeHeader(&header_actual);
	        	      //calculating DC offset from audio - i should turn this into a function
	    			  float remaining_duration = duration;
	    		      float dc_offset = 0;

	    		      for (int i = 0; i < ADC_BUF_LEN; i++) {
	    		                 dc_offset += adc_buf[i];
	    		             }
	    		             dc_offset /= ADC_BUF_LEN;
	    		      //main recording loop, writes data to SD card in segments and checks whether duration has elapsed.
	    			  while (remaining_duration > 0) {
	    				  //allocate memory for the chunk
	    				  int chunk_samples = remaining_duration >= CHUNK_SIZE_SECONDS ? CHUNK_SIZE_SAMPLES : (int)(remaining_duration * Sample_Rate);
	    				  short *data = malloc(chunk_samples * Num_Channels * sizeof(short)); //malloc use I dont think does anything here

	    				  while (!dma_half_transfer_complete) {

	    				     }

	    				  dma_half_transfer_complete = 0;

	    		            for (int i = 0; i < ADC_BUF_LEN_HALF; i++) {
	    		                adc_half_buf[i] = (short)((adc_buf[i] - dc_offset) * 40);//removing  DC offset from audio
	    		            }
	    				  //write first DMA segment chunk to SD card
	    				  sd_write((short*)adc_half_buf, ADC_BUF_LEN_HALF);

	    				  while (!dma_transfer_complete) {

	    				     }

	    				  dma_transfer_complete = 0;
	    		            for (int i = 0; i < ADC_BUF_LEN_HALF; i++) {
	    		                adc_full_buf[i] = (short)((adc_buf[i + ADC_BUF_LEN_HALF] - dc_offset) * 40);  // Example scaling and offset adjust
	    		            }

		    			  //write second DMA segment chunk to SD card
	    				  sd_write((short*)adc_full_buf, ADC_BUF_LEN_HALF);

	    				  //update remaining duration
	    				  remaining_duration -= CHUNK_SIZE_SECONDS;

	    				  //free memory allocated for the chunk
	    				  free(data);
	    			          }
	    			  //closes files onn SD cards.
	    			  finalizeRecording();

	    			  printf("Write operation complete\r\n");

	    			  app_state = PRE_ANALYSIS_RECORDING;
			          pre_analysis_complete = 0; // Reset flag
				      HAL_ADC_Stop_DMA(&hadc1);
			          HAL_ADC_Start_DMA(&hadc1, (uint32_t*)pre_analysis_buf, PRE_ANALYSIS_BUF_LEN);


	    } else if (app_state == IDLE) {
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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB2;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
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
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00707CBB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 1;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_7B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
  hpcd_USB_OTG_HS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_HS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_HS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_EMBEDDED_PHY;
  hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_HS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin : PG9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//called when first half of buffer is filled
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	if (app_state == PRE_ANALYSIS_RECORDING) {
	        //copy the first half of the ADC data to the first half of the pre_analysis_buf
	        memcpy(pre_analysis_buf, adc_buf, (PRE_ANALYSIS_BUF_LEN / 2) * sizeof(uint16_t));
	        //don't set the pre_analysis_complete flag here as it is only half done
	} else if (app_state == MAIN_RECORDING) {
    for (int i = 0; i < 16384; i++) {
    	adc_half_buf[i] = (adc_buf[i]*40); //transferring data from DMA to a separate array
    }
	}
	dma_half_transfer_complete = 1;
}

//called when second half of buffer is filled
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	 if (app_state == PRE_ANALYSIS_RECORDING) {
	        //copy the data to pre_analysis_buf instead of adc_full_buf
	        memcpy(pre_analysis_buf + (PRE_ANALYSIS_BUF_LEN / 2), adc_buf + (ADC_BUF_LEN / 2), (PRE_ANALYSIS_BUF_LEN / 2) * sizeof(uint16_t));
	        pre_analysis_complete = 1; //indicate that pre-analysis recording is complete
	        //printf("Pre-analysis complete.\r\n"); // Debugging statement
	} else if (app_state == MAIN_RECORDING) {
    for (int i = 0; i < 16384; i++) {
    	adc_full_buf[i] = (adc_buf[i + 16384]*40); //the *40 is used to determine gain applied with DSP. Simply increase or decrease as needed.
    }
	}
	dma_transfer_complete = 1;
}


void sd_setup()
{


	if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK)
	{
	  	Error_Handler();
	}

	else if(f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, rtext, sizeof(rtext)) != FR_OK)
	{
	  	Error_Handler();
	}


}

void sd_audio_file(){

    char filename[40]; //buffer to accommodate date and time //FatFS will need to allow longfilenames
    //print time and date
    snprintf(filename, sizeof(filename), "%02d-%02d-%4d_%02d-%02d-%02d.wav", gDate.Date, gDate.Month, 2000 + gDate.Year,gTime.Hours, gTime.Minutes, gTime.Seconds);

	if(f_open(&AudioFile, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
	{
		Error_Handler();
	}
    initializeHeader(&header_actual);
    //write to the .WAV file
    f_write(&AudioFile, &header_actual, sizeof(header), &byteswritten);

}

void sd_data_file(){
	const char* TextHolder = "This is where environmental data would be stored! e.g Temp,Tilt,CO2"; //if sensors are implemented add the values here.

    char filename[40];
    snprintf(filename, sizeof(filename), "%02d-%02d-%4d_%02d-%02d-%02d.txt", gDate.Date, gDate.Month, 2000 + gDate.Year,gTime.Hours, gTime.Minutes, gTime.Seconds);

	if(f_open(&TextFile, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
	{
		Error_Handler();
	}
    //write to the .txt file
	f_write(&TextFile, TextHolder, strlen(TextHolder), &bw);

}


void sd_write(short *data, int num_samples) {
    uint32_t bytesToWrite = num_samples * Num_Channels * sizeof(short);
    f_write(&AudioFile, data, bytesToWrite, &byteswritten);
    totalDataBytesWritten += byteswritten; //updates the total bytes written here
}

void sd_close()
{
	f_close(&AudioFile);
	f_close(&TextFile);
	f_mount(&SDFatFS, (TCHAR const*)NULL, 0);

}
void initializeHeader(header *header_actual) {
    header_actual->ChunkID[0] = 'R';
    header_actual->ChunkID[1] = 'I';
    header_actual->ChunkID[2] = 'F';
    header_actual->ChunkID[3] = 'F';

    header_actual->Format[0] = 'W';
    header_actual->Format[1] = 'A';
    header_actual->Format[2] = 'V';
    header_actual->Format[3] = 'E';

    header_actual->Subchunk1ID[0] = 'f';
    header_actual->Subchunk1ID[1] = 'm';
    header_actual->Subchunk1ID[2] = 't';
    header_actual->Subchunk1ID[3] = ' ';

    header_actual->Subchunk1Size = Sub_chunk1Size;
    header_actual->AudioFormat = Audio_Format;
    header_actual->NumChannels = Num_Channels;
    header_actual->SampleRate = Sample_Rate;
    header_actual->ByteRate = Sample_Rate * Num_Channels * Bitsper_Sample/8;
    header_actual->BlockAlign = Num_Channels * Bitsper_Sample/8;
    header_actual->BitsPerSample = Bitsper_Sample;

    header_actual->Subchunk2ID[0] = 'd';
    header_actual->Subchunk2ID[1] = 'a';
    header_actual->Subchunk2ID[2] = 't';
    header_actual->Subchunk2ID[3] = 'a';

    header_actual->ChunkSize = 0; //placeholder, to be updated
    header_actual->Subchunk2Size = 0; //placeholder, to be updated
}

void generateSineWave(short *data, int num_samples) {
    for (int i = 0; i < num_samples; i++) {
        float phase = 2.0 * M_PI * frequency * i / Sample_Rate;
        data[i] = amplitude * sin(phase) * 32767;
    }
}

void finalizeRecording() {
    // calculate actual sizes based on recorded data length
    header_actual.Subchunk2Size = totalDataBytesWritten; //total bytes of audio data written
    header_actual.ChunkSize = 36 + header_actual.Subchunk2Size;

    //seeks the beginning of the file to update the header
    f_lseek(&AudioFile, 0);
    f_write(&AudioFile, &header_actual, sizeof(header), &byteswritten);

    //close the file
    sd_close();
}

void Get_RTC_Time(void) {


    //get the RTC current Time
    HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
    //get the RTC current Date
    HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN); //this line is needed to unlock the time read

    printf("Current time is %02d:%02d:%02d\r\n", gTime.Hours, gTime.Minutes, gTime.Seconds);
    printf("Current date is %02d-%02d-%2d\r\n", gDate.Date, gDate.Month, 2000 + gDate.Year);
}


void SetRTCToCompileTime(RTC_HandleTypeDef *hrtc) {
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    struct tm compileTime;

    //parse the __DATE__ and __TIME__ macros to struct tm
    strptime(__DATE__ " " __TIME__, "%b %d %Y %H:%M:%S", &compileTime);

    //populate the RTC structures
    sTime.Hours = compileTime.tm_hour;
    sTime.Minutes = compileTime.tm_min;
    sTime.Seconds = compileTime.tm_sec;
    sDate.Year = compileTime.tm_year - 100; // tm_year is years since 1900
    sDate.Month = compileTime.tm_mon + 1; // tm_mon is months since January [0-11]
    sDate.Date = compileTime.tm_mday;
    sDate.WeekDay = compileTime.tm_wday + 1; // tm_wday is days since Sunday [0-6], RTC_WeekDay is [1-7]

    //set the RTC time and date
    if (HAL_RTC_SetTime(hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_RTC_SetDate(hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }
}

void performFFT() {
    //initialise the FFT instance
	arm_rfft_init_q15(&S, FFT_SIZE, 0, 1);

    for (int i = 0; i < FFT_SIZE; i++) {
        fft_input[i] = (float)pre_analysis_buf[i];
        //here you would apply a window here if needed
    }

    //perform the FFT
    arm_rfft_q15(&S, fft_input, fft_output);

    //compute magnitude of FFT output (complex numbers) for analysis
    //skipping every other value as they represent complex parts in the output array
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float real = fft_output[2*i];
        float imag = fft_output[2*i + 1];
        float magnitude = sqrtf(real * real + imag * imag);
        float frequency_bin = ((float)i * Sample_Rate) / FFT_SIZE;
        printf("Frequency: %.2f Hz, Intensity: %.2f\n", frequency_bin, magnitude);
        HAL_Delay(0.1);
    }
}

int detectSignificantFrequency(float32_t *fftMagnitude, uint32_t fftSize, float sampleRate) {
    uint32_t startIndex = FREQUENCY_THRESHOLD / (sampleRate / fftSize);

    for (uint32_t i = startIndex; i < fftSize / 2; i++) { //only need to check up to Nyquist frequency
        if (fftMagnitude[i] > AMPLITUDE_THRESHOLD) {
            return 1; //frequency above threshold found
        }
    }
    return 0; //no significant frequency found
}

q15_t unsigned_to_q15(uint16_t val) {
    int32_t temp = val - 32768;  //center around 0
    return (q15_t)temp;          //cast and return
}

int __io_putchar(int ch)
{
    ITM_SendChar(ch);
    return (ch);
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
