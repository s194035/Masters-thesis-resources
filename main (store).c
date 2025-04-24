/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 32
#define HALF_BUFFER_SIZE (BUFFER_SIZE / 2)
#define FILTER_ORDER 0
#define FILTER_LENGTH (FILTER_ORDER+1)
#define FIR_LENGTH (HALF_BUFFER_SIZE + FILTER_LENGTH - 1)
#define FLASH_ARR_SIZE 2000
#define FLASH_WRITE_SECTOR 0x08140000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t c[1] = {0};
const uint32_t MAX_SAMPLES = 2000;
const uint32_t UART_TRANSMIT_CYCLES = MAX_SAMPLES/BUFFER_SIZE;
volatile uint8_t filter_flag = 0;
volatile uint16_t adc_data[BUFFER_SIZE] = {0};
volatile uint32_t timer2_counter = 0;
volatile float32_t flash_arr[FLASH_ARR_SIZE] = {0};
volatile uint32_t flash_arr_index = 0;
float32_t uart_buffer[BUFFER_SIZE] = {0};
//float32_t fir_coeffs[FILTER_LENGTH] = {1,0,0,0,0,0,-1}; //6th order comb filter
float32_t fir_coeffs[FILTER_LENGTH] = {1}; //1st order all-pass filter
float32_t fir_state[FIR_LENGTH] = {0};
float32_t fir_in0[HALF_BUFFER_SIZE] = {0};
float32_t fir_in1[HALF_BUFFER_SIZE] = {0};
float32_t fir_out[HALF_BUFFER_SIZE] = {0};
arm_fir_instance_f32 fir0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){
	filter_flag = (1 << 0);
	for(int i = 0; i < HALF_BUFFER_SIZE; i++){
		fir_in0[i] = (float32_t)adc_data[i];
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	filter_flag = (1 << 1);
	for(int i = HALF_BUFFER_SIZE; i < BUFFER_SIZE; i++){
		fir_in1[i-HALF_BUFFER_SIZE] = (float32_t)adc_data[i];
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	timer2_counter++;
}

//ChatGPT generated function
void reverse_array(float32_t arr[], int size) {
	int temp;
	for (int i = 0; i < size / 2; i++) {
		// Swap elements
		temp = arr[i];
		arr[i] = arr[size - 1 - i];
		arr[size - 1 - i] = temp;
	}
}


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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  reverse_array(fir_coeffs, FILTER_LENGTH);
  arm_fir_init_f32(&fir0, FILTER_LENGTH, fir_coeffs, fir_state, HALF_BUFFER_SIZE);
  while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)){} //Wait until both pins are pulled up to high level

  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) && !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)){ // Pin4 high and Pin6 low means sampling mode
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET); //Red LED is on
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET); //Green LED is off
	  HAL_ADC_Start_DMA(&hadc1, (uint8_t*)adc_data, BUFFER_SIZE);
	  HAL_TIM_Base_Start_IT(&htim2);
	  while(timer2_counter <= MAX_SAMPLES){ //Wait for the ADC to record for the given duration
		  switch(filter_flag){
			  case 1:
				filter_flag = 0;
				arm_fir_f32(&fir0, fir_in0, fir_out, HALF_BUFFER_SIZE);
				for(int i = 0; i < HALF_BUFFER_SIZE; i++){
					flash_arr[flash_arr_index] = fir_out[i];
					flash_arr_index++;
				}
				break;
			  case 2:
				filter_flag = 0;
				arm_fir_f32(&fir0, fir_in1, fir_out, HALF_BUFFER_SIZE);
				for(int i = 0; i < HALF_BUFFER_SIZE; i++){
					flash_arr[flash_arr_index] = fir_out[i];
					flash_arr_index++;
				}
				break;
		  }
	  }
	  HAL_TIM_Base_Stop_IT(&htim2);
	  HAL_ADC_Stop_DMA(&hadc1);
	  //Sector 18 in flash seems to be empty. It stretches from 0x08140000 - 0x0815FFFF
	  HAL_FLASH_Unlock();
	  FLASH_Erase_Sector(FLASH_SECTOR_18, VOLTAGE_RANGE_3);
	  for(int i = 0; i < MAX_SAMPLES; i++){
		  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_WRITE_SECTOR + i*4, *(uint32_t*)&flash_arr[i]);
	  }
	  HAL_FLASH_Lock();
	  for(int i = 0; i < MAX_SAMPLES; i++){
		  flash_arr[i] = 0; //Reset array for testing.
	  }
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET); //Red LED is off
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET); //Green LED is on
  }
  else if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)){ //Pin4 low and Pin6 high means read mode
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET); //Red LED is on
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET); //Green LED is off
	  for(int i = 0; i<MAX_SAMPLES; i++){
		  flash_arr[i] = *(float32_t*)(FLASH_WRITE_SECTOR + i*sizeof(float32_t));
	  }
	  //Ready to receive prompt from computer
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET); //Red LED is on
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET); //Green LED is on
	  HAL_UART_Receive(&huart1, c, sizeof(char), HAL_MAX_DELAY); //Wait to receive start from python
	  for(int i = 0; i < UART_TRANSMIT_CYCLES; i++){
		  for(int j = 0; j < BUFFER_SIZE; j++){
			  uart_buffer[j] = flash_arr[i*BUFFER_SIZE + j];
		  }
		  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, sizeof(float32_t)*BUFFER_SIZE,HAL_MAX_DELAY);
	  }
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET); //Red LED is on
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET); //Green LED is off
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 1599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
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
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
