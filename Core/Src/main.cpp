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


//#include "custom_classes/USBDevice.h"
#include "../../Mavlink_v2/common/mavlink.h"
#include "custom_classes/altimeter.h"
#include "custom_classes/Initializer.h"
#include "custom_classes/components.h"
#include "custom_classes/SBUS.h"
#include "custom_classes/MavlinkControl.h"
#include "custom_classes/Convertor.h"
#include "custom_classes/debugger.h"
#include "custom_classes/handlers.h"

//UART HANDLES IN MAIN FOR NOW, DMA HANDLES are created in stm32g4xx_hal_msp.c file
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;



int main(void){
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes
	 * the Flash interface and the Systick. */
	HAL_Init();
	//Initialize clock/DMA/... configurations and components.
	HAL_Delay(100);
	Initializer init(&huart1, &huart2);
	init.init_Configs();

	Components components;
	components.init_Components();

	HAL_Delay(100);
	//SBUS, Convertor, Debugger Objects
	SBUS sbus(&huart2);
	Altimeter altimeter(&hi2c2);
	MavlinkControl mavlink(&huart1, &altimeter);

	Convertor convertor(&sbus, &init, &components, &altimeter);
	Debugger debugger(&sbus, &mavlink, &convertor, &altimeter);


	HAL_Delay(5000);

	printf("\r\n sanity check \r \n");


    altimeter.init_altimeter();

	/* Main loop */
	while (1){


		sbus.update();
		convertor.process();
		mavlink.update_TX();
		altimeter.read_altitude();

		//debugger.displayDebugInfo();
		//debugger.displayMavlink_header();
		//debugger.displaySBUS_channels();
		//debugger.displayMavlink_RAW();


		HAL_GPIO_TogglePin(gled_pc14_GPIO_Port, gled_pc14_Pin);





	}



// END OF MAIN
}




void Error_Handler(void){


	for (uint8_t i = 0; i < 30; i++){		/* Toggle LED signal for error */
		HAL_GPIO_TogglePin(gled_pc14_GPIO_Port, gled_pc14_Pin); //signal led
		printf("MESSAGE FROM MAIN ERROR HANDLER \r\n");
		HAL_Delay(100);
	}

}


//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size){
//
//	if(huart->Instance == USART1){
//
//		printf("testing callback \r\n");
//	}
//
//}


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
