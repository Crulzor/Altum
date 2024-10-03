/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <string>
#include "stm32g4xx_it.h"


#include "../../Mavlink_v2/common/mavlink.h"
#include "custom_classes/Initializer.h"
#include "custom_classes/components.h"
#include "custom_classes/debugger.h"
#include "custom_classes/handlers.h"
#include "custom_classes/HerelinkController/HerelinkController.h"

//UART HANDLES IN MAIN FOR NOW, DMA HANDLES are created in stm32g4xx_hal_msp.c file
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

void testFunction(void);


int main(void){
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes
	 * the Flash interface and the Systick. */
	HAL_Init();
	HAL_Delay(100);


	//Initialize clock/DMA/... configurations and components.
	Initializer init(&huart1, &huart2);

	init.init_Configs();
	Components components;
	components.init_Components();
	//HAL_GPIO_TogglePin(PA4_GPIO_GPIO_Port, PA4_GPIO_Pin);
	HAL_Delay(500);

	//HerelinkController object contains all sbus, altimeter & mavlink functionality. ctrl + click to expand
	HerelinkController controller(&huart2, &huart1, &init, &components);
	Debugger debugger(&controller);

	HAL_Delay(500);

	printf("\r\n sanity check \r \n");
	//set camera pin to block 5v.

	uint32_t timeout_start = HAL_GetTick();	//get the current time
	uint32_t timeout_duration = 13000;	//timeout in milliseconds
	//HAL_GPIO_TogglePin(PA4_GPIO_GPIO_Port, PA4_GPIO_Pin);
    GPIO_PinState PA4_pin_state = HAL_GPIO_ReadPin(PA4_GPIO_GPIO_Port, PA4_GPIO_Pin);

    if(PA4_pin_state == GPIO_PIN_RESET)
    {
    	HAL_GPIO_TogglePin(PA4_GPIO_GPIO_Port, PA4_GPIO_Pin);
    }

	/* Main loop */
	while (1){

			controller.update();

			/*
			all the printf functions in the debugger-class can cause problems while restarting the Herelink controller
			so leave it commented when not debugging
			*/
			debugger.displayDebugInfo();
			HAL_GPIO_TogglePin(gled_pc14_GPIO_Port, gled_pc14_Pin); //signal led


			if(HAL_GetTick() - timeout_start == timeout_duration)
			{
				HAL_GPIO_TogglePin(PA4_GPIO_GPIO_Port, PA4_GPIO_Pin);
				HAL_Delay(500);
				HAL_GPIO_TogglePin(PA4_GPIO_GPIO_Port, PA4_GPIO_Pin);
				HAL_Delay(500);
				HAL_GPIO_TogglePin(PA4_GPIO_GPIO_Port, PA4_GPIO_Pin);
				HAL_Delay(500);
				HAL_GPIO_TogglePin(PA4_GPIO_GPIO_Port, PA4_GPIO_Pin);



			}
			PA4_pin_state = HAL_GPIO_ReadPin(PA4_GPIO_GPIO_Port, PA4_GPIO_Pin);

	}



// END OF MAIN
}

void testFunction(void)
{



}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}


void Error_Handler(void){


	for (uint8_t i = 0; i < 30; i++){		/* Toggle LED signal for error */
		HAL_GPIO_TogglePin(gled_pc14_GPIO_Port, gled_pc14_Pin); //signal led
		printf("MESSAGE FROM MAIN ERROR HANDLER \r\n");
		HAL_Delay(1000);

	}

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
