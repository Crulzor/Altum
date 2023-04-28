#include "Initializer.h"
//#include "../stm32g4xx_hal_msp.c"
Initializer::Initializer(UART_HandleTypeDef* huart1, UART_HandleTypeDef* huart2){

	this->_huart1 = huart1;
	this->_huart2 = huart2;
	this->initClock();
	this->initGPIO();
}





void Initializer::init_Configs(void){


	this->initDMA();
	this->initTIM_1();
	this->initTIM_2();
	this->initTIM_3();
	this->initTIM_4();
	this->initTIM_8();
	this->initTIM_16();
	this->initTIM_20();
	this->initUSART_1();
	this->initUSART_2();
	this->initI2C2();


    MX_USB_Device_Init();


}


void Initializer::initTIM_1(void){

	  __HAL_RCC_TIM1_CLK_ENABLE();

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_OC_InitTypeDef sConfigOC = {0};
	  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	  /* USER CODE BEGIN TIM1_Init 1 */

	  /* USER CODE END TIM1_Init 1 */
	  htim1.Instance = TIM1;
	  htim1.Init.Prescaler = 8;
	  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim1.Init.Period = 1080;
	  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim1.Init.RepetitionCounter = 0;
	  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM1 init");
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM1 init");

	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	  {
	    this->Error_Handler();
	  }
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = 0;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM1 init");

	  }
	  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	  sBreakDeadTimeConfig.DeadTime = 0;
	  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	  sBreakDeadTimeConfig.BreakFilter = 0;
	  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	  sBreakDeadTimeConfig.Break2Filter = 0;
	  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM1 init");

	  }
	  /* USER CODE BEGIN TIM1_Init 2 */

	  /* USER CODE END TIM1_Init 2 */
	  HAL_TIM_MspPostInit(&htim1);


	  //uitleg van chatGPT over verschil met normale PWM start functie:

	  /* function, HAL_TIMEx_PWMN_Start(), is used for timers that support complementary PWM outputs.
	   * Complementary outputs are two output signals that are 180 degrees out of phase with each other.
	   * These signals are used to drive a full-bridge inverter, which is a common configuration for driving
	   * motor loads. In this case, the function starts the	   *  timer channel's complementary output
	   *  (marked as "PWMN" in the function name) for the specified timer. */


		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);


}

void Initializer::initTIM_2(void){

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM2_Init 1 */

	  /* USER CODE END TIM2_Init 1 */
	  htim2.Instance = TIM2;
	  htim2.Init.Prescaler = 0;
	  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim2.Init.Period = 4.294967295E9;
	  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM2 init");

	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM2 init");

	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM2 init");

	  }


}


void Initializer::initTIM_3(void){

	  __HAL_RCC_TIM3_CLK_ENABLE();  // Enable the TIM3 clock


	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_OC_InitTypeDef sConfigOC = {0};

	  /* USER CODE BEGIN TIM3_Init 1 */

	  /* USER CODE END TIM3_Init 1 */
	  htim3.Instance = TIM3;
	  htim3.Init.Prescaler = 0;
	  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim3.Init.Period = 1000;
	  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM3 init");

	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM3 init");

	  }
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = 0;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM3 init");

	  }
	  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM3 init");

	  }
	  /* USER CODE BEGIN TIM3_Init 2 */

	  /* USER CODE END TIM3_Init 2 */
	  HAL_TIM_MspPostInit(&htim3);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);



}

void Initializer::initTIM_4(void){

	  __HAL_RCC_TIM4_CLK_ENABLE();  // Enable the TIM4 clock

	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_OC_InitTypeDef sConfigOC = {0};

	  /* USER CODE BEGIN TIM4_Init 1 */

	  /* USER CODE END TIM4_Init 1 */
	  htim4.Instance = TIM4;
	  htim4.Init.Prescaler = 2;
	  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim4.Init.Period = 1000;
	  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM4 init");


	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM4 init");


	  }
	  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM4 init");


	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM4 init");


	  }
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = 0;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM4 init");


	  }
	  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM4 init");


	  }
	  /* USER CODE BEGIN TIM4_Init 2 */

	  /* USER CODE END TIM4_Init 2 */
	  HAL_TIM_MspPostInit(&htim4);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);


}

void Initializer::initTIM_8(void){

	  __HAL_RCC_TIM8_CLK_ENABLE();  // Enable the TIM16 clock


	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_OC_InitTypeDef sConfigOC = {0};
	  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	  /* USER CODE BEGIN TIM8_Init 1 */

	  /* USER CODE END TIM8_Init 1 */
	  htim8.Instance = TIM8;
	  htim8.Init.Prescaler = 2;
	  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim8.Init.Period = 1000;
	  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim8.Init.RepetitionCounter = 0;
	  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
	  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
	  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	  sBreakDeadTimeConfig.Break2Filter = 0;
	  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN TIM8_Init 2 */

	  /* USER CODE END TIM8_Init 2 */
	  HAL_TIM_MspPostInit(&htim8);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_4);




}

void Initializer::initTIM_16(void){

	  __HAL_RCC_TIM16_CLK_ENABLE();  // Enable the TIM16 clock
	  TIM_OC_InitTypeDef sConfigOC = {0};
	  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	  /* USER CODE BEGIN TIM16_Init 1 */

	  /* USER CODE END TIM16_Init 1 */
	  htim16.Instance = TIM16;
	  htim16.Init.Prescaler = 128;
	  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim16.Init.Period = 1000;
	  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim16.Init.RepetitionCounter = 0;
	  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM16 init");


	  }
	  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM16 init");

	  }
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = 0;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	  {
	   this->Error_Handler();
	    printf("problem with TIM16 init");


	  }
	  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	  sBreakDeadTimeConfig.DeadTime = 0;
	  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	  sBreakDeadTimeConfig.BreakFilter = 0;
	  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM16 init");


	  }
	  /* USER CODE BEGIN TIM16_Init 2 */

	  /* USER CODE END TIM16_Init 2 */
	  HAL_TIM_MspPostInit(&htim16);
	  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	  //set to zero...
	  __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0);






}

void Initializer::initTIM_20(void){

	  TIM_HandleTypeDef htim20;
	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM20_Init 1 */

	  /* USER CODE END TIM20_Init 1 */
	  htim20.Instance = TIM20;
	  htim20.Init.Prescaler = 0;
	  htim20.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim20.Init.Period = 65535;
	  htim20.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim20.Init.RepetitionCounter = 0;
	  htim20.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim20) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM16 init");

	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim20, &sClockSourceConfig) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM16 init");

	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim20, &sMasterConfig) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with TIM16 init");

	  }



}

void Initializer::initUSART_1(void){

	  _huart1->Instance = USART1;
	  _huart1->Init.BaudRate = 57600;
	  _huart1->Init.WordLength = UART_WORDLENGTH_8B;
	  _huart1->Init.StopBits = UART_STOPBITS_1;
	  _huart1->Init.Parity = UART_PARITY_NONE;
	  _huart1->Init.Mode = UART_MODE_TX_RX;
	  _huart1->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  _huart1->Init.OverSampling = UART_OVERSAMPLING_16;
	  _huart1->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  _huart1->Init.ClockPrescaler = UART_PRESCALER_DIV1;
	  _huart1->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  if (HAL_UART_Init(_huart1) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with UART1");
	  }
	  if (HAL_UARTEx_SetTxFifoThreshold(_huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with UART1");

	  }
	  if (HAL_UARTEx_SetRxFifoThreshold(_huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with UART1");

	  }
	  if (HAL_UARTEx_DisableFifoMode(_huart1) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with UART1");

	  }



}

void Initializer::initUSART_2(void){

	   _huart2->Instance = USART2;
	   _huart2->Init.BaudRate = 100000;
	   _huart2->Init.WordLength = UART_WORDLENGTH_9B;
	   _huart2->Init.StopBits = UART_STOPBITS_1;
	   _huart2->Init.Parity = UART_PARITY_EVEN;
	   _huart2->Init.Mode = UART_MODE_TX_RX;
	   _huart2->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	   _huart2->Init.OverSampling = UART_OVERSAMPLING_16;
	   _huart2->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	   _huart2->Init.ClockPrescaler = UART_PRESCALER_DIV1;
	   _huart2->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
	   _huart2->AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
	   if (HAL_UART_Init(_huart2) != HAL_OK)
	   {
	     Error_Handler();
		    printf("problem with UART2");

	   }
	   if (HAL_UARTEx_SetTxFifoThreshold(_huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	   {
	     Error_Handler();
		    printf("problem with UART2");

	   }
	   if (HAL_UARTEx_SetRxFifoThreshold(_huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	   {
	     Error_Handler();
		    printf("problem with UART2");

	   }
	   if (HAL_UARTEx_DisableFifoMode(_huart2) != HAL_OK)
	   {
	     Error_Handler();
		    printf("problem with UART2");

	   }
	   /* USER CODE BEGIN USART2_Init 2 */

	   /* USER CODE END USART2_Init 2 */



}

void Initializer::initI2C2(void){

	  hi2c_ptr->Instance = I2C2;
	  hi2c_ptr->Init.Timing = 0x20A0C4DF;
	  hi2c_ptr->Init.OwnAddress1 = 0;
	  hi2c_ptr->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c_ptr->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c_ptr->Init.OwnAddress2 = 0;
	  hi2c_ptr->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	  hi2c_ptr->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c_ptr->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  if (HAL_I2C_Init(hi2c_ptr) != HAL_OK)
	  {
	    Error_Handler();
	    printf("problem with I2C2");
	  }

	  /** Configure Analogue filter
	  */
	  if (HAL_I2CEx_ConfigAnalogFilter(hi2c_ptr, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with I2C2");

	  }

	  /** Configure Digital filter
	  */
	  if (HAL_I2CEx_ConfigDigitalFilter(hi2c_ptr, 0) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with I2C2");

	  }
	  /* USER CODE BEGIN I2C2_Init 2 */

	  /* USER CODE END I2C2_Init 2 */
	  __HAL_RCC_I2C2_CLK_ENABLE();


}




void Initializer::initDMA(void){


	  /* DMA controller clock enable */
	  __HAL_RCC_DMAMUX1_CLK_ENABLE();
	  __HAL_RCC_DMA1_CLK_ENABLE();
	  __HAL_RCC_DMA2_CLK_ENABLE();

	  /* DMA interrupt init */
	  /* DMA1_Channel1_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	  /* DMA1_Channel2_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	  /* DMA1_Channel3_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	  /* DMA1_Channel4_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	  /* DMA2_Channel1_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

}

void Initializer::initGPIO(void){

	 GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOF_CLK_ENABLE();
	  __HAL_RCC_GPIOG_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOC, gled_pc14_Pin|PC15_RTS_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(PF1_CTS_GPIO_Port, PF1_CTS_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(PG10_NRST_GPIO_Port, PG10_NRST_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	  /*Configure GPIO pins : gled_pc14_Pin PC15_RTS_Pin */
	  GPIO_InitStruct.Pin = gled_pc14_Pin|PC15_RTS_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin : PF1_CTS_Pin */
	  GPIO_InitStruct.Pin = PF1_CTS_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(PF1_CTS_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : PG10_NRST_Pin */
	  GPIO_InitStruct.Pin = PG10_NRST_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(PG10_NRST_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : PA4 */
	  GPIO_InitStruct.Pin = GPIO_PIN_4;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}


void Initializer::initClock(){


	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	  RCC_OscInitStruct.PLL.PLLN = 16;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with initClock function");
	  }

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	  {
	    this->Error_Handler();
	    printf("problem with initClock function");

	  }


}

TIM_HandleTypeDef* Initializer::get_LED_Timer(void){

	return &htim16;

}

TIM_HandleTypeDef* Initializer::get_selector_Timer(void){

	return &htim8;
}

TIM_HandleTypeDef* Initializer::get_pushMotor_Timer(void){

	return &htim4;
}


TIM_HandleTypeDef* Initializer::get_fluidMotor_Timer(void){

	return &htim3;

}

TIM_HandleTypeDef* Initializer::get_cleanerMotor_Timer(void){

	return &htim1;

}

I2C_HandleTypeDef* Initializer::get_i2c(void){

	return &hi2c2;
}



void Initializer::Error_Handler(void){

	//separate error handler for the Initializer
	//adapted the handler to show the error on the status led then exit and try again.

	for (uint8_t i = 0; i < 30; i++){		/* Toggle LED signal for error */
		HAL_GPIO_TogglePin(gled_pc14_GPIO_Port, gled_pc14_Pin); //signal led
		HAL_Delay(50);
		printf("Initialization error handler !! \r\n");

	}

}


