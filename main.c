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
#include <stdbool.h>
#include "arm_math.h"
#include "fir_coeffs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//General defines
#define SAMPLING_FREQUENCY 1000
#define BUFFER_SIZE 32
#define HALF_BUFFER_SIZE (BUFFER_SIZE / 2)
#define DECIMATION_FACTOR 4
#define HALF_BUFFER_SIZE_DECIMATED (HALF_BUFFER_SIZE/DECIMATION_FACTOR)
#define ADC_CONVERSION_FACTOR 0.00073260073
#define TIMER_PERIOD 1/(SAMPLING_FREQUENCY)*1000

//IIR defines
#define IIR_FILTER_ORDER 6
#define IIR_NUM_STAGES (IIR_FILTER_ORDER / 2)

//FIR defines
#define FIR_FILTER_ORDER 1000
#define FIR_FILTER_LENGTH (FIR_FILTER_ORDER+1)
#define FIR_LENGTH (HALF_BUFFER_SIZE_DECIMATED + FIR_FILTER_LENGTH - 1)

//Pan-thompkins defines
#define WINDOW_LENGTH 38
#define PAN_THOMPKINS_FIR_FILTER_LENGTH 5
#define PAN_THOMPKINS_FIR_LENGTH (HALF_BUFFER_SIZE_DECIMATED + PAN_THOMPKINS_FIR_FILTER_LENGTH - 1)
#define PAN_THOMPKINS_NUM_STAGES 4
#define REFACTORY_PERIOD 200
#define PEAK_STORAGE 1000
#define R_PEAKS 16 //Should be 8, but testing other values

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
uint32_t MAX_SAMPLES = 10000; //Initial value has no effect
volatile uint16_t adc_data[BUFFER_SIZE] = {0};
volatile float32_t uart_data0[HALF_BUFFER_SIZE] = {0};
volatile float32_t uart_data1[HALF_BUFFER_SIZE] = {0};
volatile float32_t uart_data0_downsampled[HALF_BUFFER_SIZE_DECIMATED] = {0};
volatile float32_t pan_thompkins0[HALF_BUFFER_SIZE_DECIMATED] = {0};
volatile float32_t pan_thompkins1[HALF_BUFFER_SIZE_DECIMATED] = {0};
volatile uint8_t transmit_flag = 0;
volatile uint32_t timer2_counter = 0;
volatile float32_t uart_ma_hr_data[HALF_BUFFER_SIZE_DECIMATED + 1] = {0};

//Moving average
float32_t ma_output[HALF_BUFFER_SIZE_DECIMATED];

//Peak detection
uint32_t rpeak_index[PEAK_STORAGE] = {0};
float32_t r_peak_interval[R_PEAKS] = {0};
uint8_t r_index = 0;
float32_t r_peak_interval_average = 0;
bool peak_detected = false;
float32_t beats_per_minute = 0;
float32_t THRESHOLD1I = 0;

//FIR filtering stuff
float32_t fir_state[FIR_LENGTH] = {0};
float32_t fir_in0[HALF_BUFFER_SIZE_DECIMATED] = {0};
arm_fir_instance_f32 fir0;



//Every coefficient has been normalized to a0. The array stores b10, b11, b12, a11, a12, ...
//Coefficients in SOS form. Remember, MATLAB puts a minus sign in front of a coefficients. Just negate them.
float32_t iir_coeffs[5*IIR_NUM_STAGES] = { 1.000000000000000, -2.000000000000000, 1.000000000000000,   1.9878958, -0.9879351,
									   1.000000000000000, -2.000000000000000, 1.000000000000000,   1.9911143, -0.9911535,
									   1.000000000000000, -1.999969278284963, 0.999980865437720,   1.9967135, -0.9967529};

float32_t iir_state0[2*IIR_NUM_STAGES] = {0}; //For DF1 we need 4*numstages, but for DF2T we need 2*numstages


float32_t iir_butterworth_coeffs[5*IIR_NUM_STAGES] = {0.0010516, 0.0021033, 0.0010516, 0.8402869, -0.1883452,
												  	  1.0000000, 2.0000000, 1.0000000, 0.9428090, -0.3333333,
													  1.0000000, 2.0000000, 1.0000000, 1.1954340, -0.6905989};
float32_t iir_butterworth_state[2*IIR_NUM_STAGES] = {0};
float32_t iir_in0[HALF_BUFFER_SIZE] = {0};
float32_t iir_in1[HALF_BUFFER_SIZE] = {0};
arm_biquad_cascade_df2T_instance_f32 iir0;

float32_t iir_pan_thompkins_bp[5*PAN_THOMPKINS_NUM_STAGES] = {
										1.8321602e-04, 3.6643204e-04, 1.8321602e-04, 1.6776208, -0.7465797,
										1.0000000,	   2.0000000,	  1.0000000,     1.8128196,	-0.8389382,
										1.0000000,	  -2.0000000,	  1.0000000,	 1.7450038,	-0.8690063,
										1.0000000,	  -2.0000000,	  1.0000000,	 1.9344329,	-0.9507427};
float32_t iir_pan_thompkins_bp_state[2*4] = {0};
arm_biquad_cascade_df2T_instance_f32 iir_pan_thompkins;

//float32_t fir_derivative_coeffs[5] = {-31.250000000000000,	-62.500000000000000,	0,	62.500000000000000,	31.250000000000000};
float32_t fir_derivative_coeffs[5] = {-1,	-2,	0,	2,	1};
float32_t fir_derivative_state[PAN_THOMPKINS_FIR_LENGTH] = {0};
arm_fir_instance_f32 fir_pan_thompkins;


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
	transmit_flag = (1 << 0);
	for(int i = 0; i < HALF_BUFFER_SIZE; i++){
		iir_in0[i] = (float32_t)adc_data[i] * ADC_CONVERSION_FACTOR;
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	transmit_flag = (1 << 1);
	for(int i = HALF_BUFFER_SIZE; i < BUFFER_SIZE; i++){
		iir_in1[i-HALF_BUFFER_SIZE] = (float32_t)adc_data[i] * ADC_CONVERSION_FACTOR;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	timer2_counter++;
}
//Reverse array in place
void reverse_array(float32_t arr[], int size) {
	float32_t temp;
	int last_index = size - 1;
	for (int i = 0; i < size / 2; i++) {
		temp = arr[i];
		arr[i] = arr[last_index - i];
		arr[last_index - i] = temp;
	}
}


void decimate(float32_t source[], float32_t dest[], int size){
	for(int i = 0; i < size/DECIMATION_FACTOR; i++){
		dest[i] = source[i*DECIMATION_FACTOR];
	}
}

void square(float32_t source[], int size){
	for(int i = 0; i < size; i++){
		source[i] = (source[i] * source[i]);
	}
}

//We process HALF_BUFFER_SIZE_DECIMATED samples at a time, hence the need for the for loop.
void moving_average(float32_t* input_samples, int size){
	static uint32_t ma_counter = 0;
	static uint16_t ma_index = 0;
	static float32_t ma = 0;
	static float32_t ma_samples[WINDOW_LENGTH] = {0};
	for(int i = 0; i < size; i++){
		if(ma_counter < WINDOW_LENGTH){
			ma = (float32_t) (input_samples[i] + ma_counter*ma)/(ma_counter + 1);
		}
		else{
			ma = ma + (input_samples[i] - (float32_t)ma_samples[ma_index])/WINDOW_LENGTH;
		}
		ma_output[i] = ma; //Store the output in a buffer to be sent via uart.
		ma_samples[ma_index] = input_samples[i]; //Store the input sample
		ma_counter++;
		ma_index = ma_counter % WINDOW_LENGTH; //Update index as to know which samples is the oldest.
	}
}

bool peak_detection(float32_t* input_samples, int size){
	static float32_t middle = 0;
	static float32_t left = 0;
	static uint32_t last_peak_index = 0;
	static uint8_t j = 0;
	static float32_t SPKI = 0;
	static float32_t NPKI = 0;

	bool peak_detected = false;
	for(int i = 0; i < size; i++){
		if(middle >= left && middle >= input_samples[i]){ //Middle sample is a peak
			float32_t peak = middle;
			uint32_t index_diff = timer2_counter - last_peak_index;
			float32_t time_diff = (float32_t)index_diff * (float32_t)TIMER_PERIOD;
			if(time_diff >= REFACTORY_PERIOD){
				if(peak > THRESHOLD1I){
					peak_detected = true;
					rpeak_index[j] = timer2_counter;
					j++;
					last_peak_index = timer2_counter;
					SPKI = (float32_t)0.125 * peak + (float32_t)0.875*SPKI;
					r_peak_interval[r_index] = time_diff;
					r_index = (r_index + 1) % R_PEAKS;
					float32_t sum = 0;
					for(int i = 0; i < R_PEAKS; i++){
						sum += r_peak_interval[i];
					}
					r_peak_interval_average = (float32_t)sum/R_PEAKS;
					beats_per_minute = (float32_t) (60/(r_peak_interval_average/1000));
				}
				else{
					NPKI = (float32_t)0.125 * peak + (float32_t)0.875 * NPKI;
				}
				THRESHOLD1I = NPKI + (float32_t)0.75*(SPKI - NPKI); //Try to tune the multiplication value
			}
		}
		left = middle;
		middle = input_samples[i];
	}
	return peak_detected;
}

void collect(float32_t* arr, int size, float32_t val){
	for(int i = 0; i < size; i++){
		uart_ma_hr_data[i] = arr[i];
	}
	uart_ma_hr_data[size] = val; //Remove division, just testing
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

  //Regular signal processing
  reverse_array(fir_coeffs, FIR_FILTER_LENGTH);
  arm_biquad_cascade_df2T_init_f32(&iir0, IIR_NUM_STAGES, iir_coeffs, iir_state0); //Remember to change this
  arm_fir_init_f32(&fir0, FIR_FILTER_LENGTH, fir_coeffs, fir_state, HALF_BUFFER_SIZE_DECIMATED);

  //Pan-thompkins
  reverse_array(fir_derivative_coeffs, HALF_BUFFER_SIZE_DECIMATED + 5 - 1);
  arm_biquad_cascade_df2T_init_f32(&iir_pan_thompkins,PAN_THOMPKINS_NUM_STAGES, iir_pan_thompkins_bp, iir_pan_thompkins_bp_state);
  arm_fir_init_f32(&fir_pan_thompkins, PAN_THOMPKINS_FIR_FILTER_LENGTH, fir_derivative_coeffs, fir_derivative_state, HALF_BUFFER_SIZE_DECIMATED);

  HAL_UART_Receive(&huart1, c, sizeof(char), HAL_MAX_DELAY); //Wait to receive start from python
  switch(c[0]){
  	  case 1: //Are we in "writing to file" mode?
  		  MAX_SAMPLES = 20000; //Remember to change this
  		  break;
  	  case 2: //Or are we in "real-time plotting" mode?
  		  MAX_SAMPLES = 50000;
  		  break;
  }
  HAL_ADC_Start_DMA(&hadc1, (uint8_t*)adc_data, BUFFER_SIZE);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(transmit_flag){
	  	  case 1: //Transmit buffer0
	  		  transmit_flag = 0;
	  		  arm_biquad_cascade_df2T_f32(&iir0, iir_in0, uart_data0, HALF_BUFFER_SIZE);
	  		  HAL_UART_Transmit_IT(&huart1, uart_data0, sizeof(uart_data0));

	  		  //FIR and decimate scheme
	  		  //arm_biquad_cascade_df2T_f32(&iir0, iir_in0, uart_data0, HALF_BUFFER_SIZE);
	  		  //decimate(uart_data0, fir_in0, HALF_BUFFER_SIZE);
	  		  //arm_fir_f32(&fir0, fir_in0, uart_data0_downsampled, HALF_BUFFER_SIZE_DECIMATED);
	  		  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)uart_data0_downsampled, sizeof(float32_t)*HALF_BUFFER_SIZE_DECIMATED);
	  		  /*
	  		  //Pan.thompkins
	  		  arm_biquad_cascade_df2T_f32(&iir_pan_thompkins, uart_data0_downsampled, pan_thompkins0, HALF_BUFFER_SIZE_DECIMATED); //Bandpass filtering
	  		  arm_fir_f32(&fir_pan_thompkins, pan_thompkins0, pan_thompkins1, HALF_BUFFER_SIZE_DECIMATED); //Derivative
	  		  square(pan_thompkins1, HALF_BUFFER_SIZE_DECIMATED);
	  		  moving_average(pan_thompkins1, HALF_BUFFER_SIZE_DECIMATED);
	  		  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)ma_output, sizeof(ma_output));
	  		  if(timer2_counter > 5000){
	  			peak_detected = peak_detection(ma_output, HALF_BUFFER_SIZE_DECIMATED);
	  		  }
	  		  collect(ma_output, HALF_BUFFER_SIZE_DECIMATED, beats_per_minute);
	  		  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)ma_output, sizeof(ma_output));
	  		  HAL_UART_Transmit_IT(&huart1, (uint8_t*)uart_ma_hr_data, sizeof(uart_ma_hr_data));
	  		  */
	  		  break;
	  	  case 2: //Transmit buffer1
	  		  transmit_flag = 0;
	  		  arm_biquad_cascade_df2T_f32(&iir0, iir_in1, uart_data1, HALF_BUFFER_SIZE);
	  		  HAL_UART_Transmit_IT(&huart1, uart_data1, sizeof(uart_data1));

	  		  //Fir and decimate scheme
	  		  //arm_biquad_cascade_df2T_f32(&iir0, iir_in1, uart_data0, HALF_BUFFER_SIZE);
	  		  //decimate(uart_data0, fir_in0, HALF_BUFFER_SIZE);
	  		  //arm_fir_f32(&fir0, fir_in0, uart_data0_downsampled, HALF_BUFFER_SIZE_DECIMATED);
	  		  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)uart_data0_downsampled, sizeof(float32_t)*HALF_BUFFER_SIZE_DECIMATED);
	  		  /*
	  		  //Pan.thompkins
	  		  arm_biquad_cascade_df2T_f32(&iir_pan_thompkins, uart_data0_downsampled, pan_thompkins0, HALF_BUFFER_SIZE_DECIMATED); //Bandpass filtering
	  		  arm_fir_f32(&fir_pan_thompkins, pan_thompkins0, pan_thompkins1, HALF_BUFFER_SIZE_DECIMATED); //Derivative
	  		  square(pan_thompkins1, HALF_BUFFER_SIZE_DECIMATED);
	  		  moving_average(pan_thompkins1, HALF_BUFFER_SIZE_DECIMATED);
	  		  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)ma_output, sizeof(ma_output));
	  		  if(timer2_counter > 5000){
	  			peak_detected = peak_detection(ma_output, HALF_BUFFER_SIZE_DECIMATED);
	  		  }
	  		  collect(ma_output, HALF_BUFFER_SIZE_DECIMATED, beats_per_minute);
	  		  //HAL_UART_Transmit_IT(&huart1, (uint8_t*)ma_output, sizeof(ma_output));
	  		  HAL_UART_Transmit_IT(&huart1, (uint8_t*)uart_ma_hr_data, sizeof(uart_ma_hr_data));
	  		  */
	  		  break;
	  	  default:
	  }
	  if(peak_detected){
		  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
	  }

	  if(timer2_counter >= MAX_SAMPLES){
		  HAL_TIM_Base_Stop_IT(&htim2);
		  HAL_ADC_Stop_DMA(&hadc1);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
		  while(1){}
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
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PG10 PG11 PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;
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
