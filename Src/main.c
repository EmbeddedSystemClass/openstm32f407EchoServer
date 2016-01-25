/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */
#include "battery-socket.h"
#include "arm_math.h"
#include "arm_common_tables.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
VoltageStruct voltageStruct;

static DAC_ChannelConfTypeDef sConfig;

const uint8_t aEscalator8bit[6] = {0x0, 0x33, 0x66, 0x99, 0xCC, 0xFF};

/* Variable used to get converted value */
__IO uint16_t uhADCxConvertedValue[VOLTAGE_BUFFER_LENGTH] = {0};
__IO uint16_t uhDACxConvertedValue = 0;
uint32_t sampleCounter = 0;
uint32_t sampleMultiplier = 0;
uint32_t bufferFirst[ETHERNET_BUFFER_LENGTH] = {0};
uint32_t bufferLast[ETHERNET_BUFFER_LENGTH] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static void ToggleLed4(void const * argument);
static void BatteryVoltageMonitor(void const * argument);
static void BatteryVoltageController(void const * argument);
static void Error_Handler(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle);
void HAL_ADC_ConvCpltCallback2(ADC_HandleTypeDef* AdcHandle);
void SET_DAC();

static void DAC_Ch1_TriangleConfig(void);
static void DAC_Ch1_EscalatorConfig(void);
void InitializeVoltageStruct();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  voltageStruct.packetHeader = 0x0000AA55;
  voltageStruct.bufferFirstHalf = bufferFirst;
  voltageStruct.bufferLastHalf = bufferLast;
  voltageStruct.bufferLength = ETHERNET_BUFFER_LENGTH;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */

  /*##-2- Enable TIM peripheral counter ######################################*/
  HAL_TIM_Base_Start(&htim6);

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  /*##-3- Start the conversion process and enable interrupt ##################*/
  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&uhADCxConvertedValue, VOLTAGE_BUFFER_LENGTH) != HAL_OK)
  {
    /* Start Conversation Error */
    Error_Handler();
  }

  /*##-2- Enable DAC Channel1 and associated DMA #############################*/
  if(HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)&uhDACxConvertedValue, 1, DAC_ALIGN_12B_R) != HAL_OK)
  {
    /* Start DMA Error */
    Error_Handler();
  }



  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

//  osThreadDef(LED4, ToggleLed4, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
//  osThreadCreate(osThread(LED4), NULL);

//  osThreadDef(BATTERYADC1, BatteryVoltageMonitor, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
//  osThreadCreate(osThread(BATTERYADC1), NULL);

//  osThreadDef(BATTERYDAC1, BatteryVoltageController, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
//  osThreadCreate(osThread(BATTERYDAC1), NULL);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* DAC init function */
void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  HAL_DAC_Init(&hdac);

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 25000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0x7FF;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//uint32_t dacCounter = 0;
//float32_t dacPeriod= 0x7FF / 50000000.;
//float32_t dacResolution = 2^12;
/////**
////  * @brief  Conversion complete callback in non blocking mode for Channel1
////  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
////  *         the configuration information for the specified DAC.
////  * @retval None
////  */
//void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
//{
//  float32_t theta = dacCounter * 2. * 3.14 * dacPeriod;
//  uint32_t sinTheta = dacResolution * (1.001 + arm_sin_f32(theta)) / 2.;
//
////  uhDACxConvertedValue = (uint32_t)sinTheta;
//  dacCounter += 1;
////  HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 2048);
//}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	broadcastVoltageAll();
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}


static void BatteryVoltageMonitor(void const * argument)
{

  /*##-3- Start the conversion process and enable interrupt ##################*/
  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&uhADCxConvertedValue, VOLTAGE_BUFFER_LENGTH) != HAL_OK)
  {
    /* Start Conversation Error */
    Error_Handler();
  }

  /* Infinite loop */
  while (1)
  {
	  osDelay(1);
  }

}


static void BatteryVoltageController(void const * argument)
{
//      DAC_Ch1_TriangleConfig();
//      DAC_Ch1_EscalatorConfig();

  /* Infinite loop */
  while (1)
  {
	  osDelay(1);
  }

}


/**
  * @brief  Conversion complete callback in non blocking mode for Channel1
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
//void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
//{
//	q15_t phase = 10 * HAL_GetTick() / 1000;
//	waveformValue = arm_sin_q15(phase);
//	HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, waveformValue);
//	HAL_DAC_Start(hdac, DAC_CHANNEL_1);

//	waveformValue = HAL_GetTick() % 4095;
//	if(HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t*)&waveformValue, sizeof(uint16_t), DAC_ALIGN_12B_R) != HAL_OK)
//	{
//	/* Start Error */
//	Error_Handler();
//	}
//}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED5 on */
//  BSP_LED_On(LED5);
  while(1)
  {
  }
}


static void DAC_Ch1_EscalatorConfig(void)
{
  /*##-1- Initialize the DAC peripheral ######################################*/
  if(HAL_DAC_Init(&hdac) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-1- DAC channel1 Configuration #########################################*/
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

  if(HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Channel configuration Error */
    Error_Handler();
  }

  /*##-2- Enable DAC Channel1 and associated DMA #############################*/
  if(HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)aEscalator8bit, 6, DAC_ALIGN_8B_R) != HAL_OK)
  {
    /* Start DMA Error */
    Error_Handler();
  }
}


/**
  * @brief  DAC Channel1 Triangle Configuration
  * @param  None
  * @retval None
  */
static void DAC_Ch1_TriangleConfig(void)
{
  /*##-1- Initialize the DAC peripheral ######################################*/
  if(HAL_DAC_Init(&hdac) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- DAC channel2 Configuration #########################################*/
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

  if(HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Channel configuration Error */
    Error_Handler();
  }

  /*##-3- DAC channel2 Triangle Wave generation configuration ################*/
  if(HAL_DACEx_TriangleWaveGenerate(&hdac, DAC_CHANNEL_1, DAC_TRIANGLEAMPLITUDE_1023) != HAL_OK)
  {
    /* Triangle wave generation Error */
    Error_Handler();
  }

  /*##-4- Enable DAC Channel1 ################################################*/
  if(HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /*##-5- Set DAC channel1 DHR12RD register ################################################*/
  if(HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x100) != HAL_OK)
  {
    /* Setting value Error */
    Error_Handler();
  }
}


//uint32_t * getVoltagePacket()
//{
//	int i;
//	for(i = 0; i < VOLTAGE_BUFFER_LENGTH; i++)
//	{
//		voltagePacket[i] = voltageStruct.buffer[i];
//		voltageStruct.buffer[i] = 0;
//	}
//	voltageStruct.packetCount = 0;
//	return voltagePacket;
//}


#define PERIOD 2520
uint32_t averagingBuffer = 0;
uint32_t tick = 0;
/**
  * @brief  Conversion complete callback in non blocking mode for Channel1
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac){
	SET_DAC();
}


void SET_DAC()
{
    float32_t phase = ((float32_t)tick * (VOLTAGE_BUFFER_LENGTH / 2)) * 2. * 3.14 /((float32_t)PERIOD);
    float32_t sin = arm_sin_f32(phase * 10.0);
    float32_t cos = arm_cos_f32(phase);

    uint16_t waveformValue = 4095. * (2. +  .3 * sin + cos) / 4.;

    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, waveformValue);
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
}


int i, firstHalfBufferIsActive = 1;
//int16_t waveformValue[6] = {0};
/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	int j, startIndex, bufferAverage = 0;

	startIndex = VOLTAGE_BUFFER_LENGTH / 2;
	for (j=startIndex; j<VOLTAGE_BUFFER_LENGTH; j++){
		bufferAverage += uhADCxConvertedValue[j];
	}

	if(firstHalfBufferIsActive){
		voltageStruct.bufferFirstHalf[i] = voltageStruct.packetHeader;
		voltageStruct.bufferFirstHalf[i + 1] = tick * (VOLTAGE_BUFFER_LENGTH / 2);
		voltageStruct.bufferFirstHalf[i + 2] = bufferAverage / (VOLTAGE_BUFFER_LENGTH / 2);
	}else{
		voltageStruct.bufferLastHalf[i] = voltageStruct.packetHeader;
		voltageStruct.bufferLastHalf[i + 1] = tick * (VOLTAGE_BUFFER_LENGTH / 2);
		voltageStruct.bufferLastHalf[i + 2] = bufferAverage / (VOLTAGE_BUFFER_LENGTH / 2);
	}
	tick += 1;

	if(i<ETHERNET_BUFFER_LENGTH - 3){
		i += 3;
	}else{
		i = 0;
		firstHalfBufferIsActive = !firstHalfBufferIsActive;
	}
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	int j, endIndex, bufferAverage = 0;

	endIndex = VOLTAGE_BUFFER_LENGTH / 2;
	for (j=0; j<endIndex; j++){
		bufferAverage += uhADCxConvertedValue[j];
	}

	if(firstHalfBufferIsActive){
		voltageStruct.bufferFirstHalf[i] = voltageStruct.packetHeader;
		voltageStruct.bufferFirstHalf[i + 1] = tick * (VOLTAGE_BUFFER_LENGTH / 2);
		voltageStruct.bufferFirstHalf[i + 2] = bufferAverage / (VOLTAGE_BUFFER_LENGTH / 2);
	}else{
		voltageStruct.bufferLastHalf[i] = voltageStruct.packetHeader;
		voltageStruct.bufferLastHalf[i + 1] = tick * (VOLTAGE_BUFFER_LENGTH / 2);
		voltageStruct.bufferLastHalf[i + 2] = bufferAverage / (VOLTAGE_BUFFER_LENGTH / 2);
	}
	tick += 1;

	if(i<ETHERNET_BUFFER_LENGTH - 3){
		i += 3;
	}else{
		i = 0;
		firstHalfBufferIsActive = !firstHalfBufferIsActive;
	}
}


//void HAL_ADC_ConvCpltCallback2(ADC_HandleTypeDef* AdcHandle)
//{
//	averagingBuffer += uhADCxConvertedValue;
//    if(sampleCounter >= VOLTAGE_SUBSAMPLE)
//    {
//    	uint32_t newHeaderLocation = voltageStruct.packetCount * voltageStruct.packetLength;
//    	if(newHeaderLocation + voltageStruct.packetLength <= voltageStruct.bufferLength)
//    	{
//			voltageStruct.buffer[newHeaderLocation] = voltageStruct.packetHeader;
//			voltageStruct.buffer[newHeaderLocation + 1] = sampleMultiplier - VOLTAGE_SUBSAMPLE / 2;
//			voltageStruct.buffer[newHeaderLocation + 2] = averagingBuffer / VOLTAGE_SUBSAMPLE;
//			voltageStruct.packetCount += 1;
//    	}
//
//    	sampleMultiplier += 1;
//
//    	sampleCounter = 0;
//    	averagingBuffer = 0;
//
//    } else
//	{
//		sampleCounter += 1;
//	}
//
//    if(tick >= PERIOD)
//    {
//    	tick = 0;
//    }else
//    {
//    	tick += 1;
//    }
//
//    float32_t phase = ((float32_t)tick) * 2. * 3.14 /((float32_t)PERIOD);
//    float32_t sin = arm_sin_f32(phase * 10.0);
//    float32_t cos = arm_cos_f32(phase);
//
//    uint16_t waveformValue = 4095. * (2. +  .3 * sin + cos) / 4.;
//
//    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, waveformValue);
//    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
//
//}
//

/**
  * @brief  Error DAC callback for Channel1.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac)
{
	Error_Handler();
}

/**
  * @brief  DMA underrun DAC callback for channel1.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac)
{
	Error_Handler();
}


/**
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* USER CODE BEGIN 5 */

  voltage_server_socket();

  /* USER CODE END 5 */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
