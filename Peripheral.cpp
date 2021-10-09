/*
  Peripheral control library.
  2021-09-12  T. Nakagawa
*/

#include "Peripheral.h"
#include <stm32_def.h>
#include <stm32f1xx_hal.h>

static ADC_HandleTypeDef hadc1;
static DMA_HandleTypeDef hdma_adc1;
static TIM_HandleTypeDef htim2;
static DMA_HandleTypeDef hdma_tim2_ch2_ch4;
static UART_HandleTypeDef huart3;
static DMA_HandleTypeDef hdma_usart3_tx;
static void (*adc_callback)();

static void hal_ADC_MspInit(ADC_HandleTypeDef* hadc) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1) {
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);
  }
}

static void hal_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
  if(htim_base->Instance==TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
  
    hdma_tim2_ch2_ch4.Instance = DMA1_Channel7;
    hdma_tim2_ch2_ch4.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch2_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch2_ch4.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch2_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim2_ch2_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim2_ch2_ch4.Init.Mode = DMA_CIRCULAR;
    hdma_tim2_ch2_ch4.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_tim2_ch2_ch4) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC2],hdma_tim2_ch2_ch4);
    __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC4],hdma_tim2_ch2_ch4);
  }
}

static void hal_TIM_MspPostInit(TIM_HandleTypeDef* htim) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim->Instance==TIM2) {
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void hal_UART_MspInit(UART_HandleTypeDef* huart) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==USART3) {
    __HAL_RCC_USART3_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hdma_usart3_tx.Instance = DMA1_Channel2;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

    __HAL_LINKDMA(huart,hdmatx,hdma_usart3_tx);
  }
}

extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL14;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
}

static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig;

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) _Error_Handler(__FILE__, __LINE__);
}

static void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

  hal_TIM_MspPostInit(&htim2);
}

static void MX_USART3_UART_Init(void) {
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 500000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) _Error_Handler(__FILE__, __LINE__);
}

static void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);	// ADC
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);	// PWM
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);	// USART
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

extern "C" void DMA1_Channel1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_adc1);
}

extern "C" void DMA1_Channel7_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_tim2_ch2_ch4);
}

extern "C" void DMA1_Channel2_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  huart3.gState = HAL_UART_STATE_READY;
}

extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  (void)hadc;
  adc_callback();
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  (void)hadc;
  adc_callback();
}

void initPeripherals() {
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  hal_ADC_MspInit(&hadc1);
  hal_TIM_Base_MspInit(&htim2);
  hal_UART_MspInit(&huart3);
  MX_USART3_UART_Init();	// USART is not activated without this second call.
}

void startPeripherals(int adc_buf_size, int16_t *adc_buf, void (*adc_handler)(), int pwm_buf_size, int16_t *pwm_buf) {
  adc_callback = adc_handler;
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_buf, adc_buf_size);
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t *)pwm_buf, pwm_buf_size);
}

void sendSerial(uint8_t *buf, int size) {
  HAL_UART_Transmit_DMA(&huart3, buf, size);
}
