#pragma once

#include "../../Inc/main.h"

class Initializer{



	public:

		//Constructor, currently takes in uart and dma pointers to avoid any scope issues.

		Initializer(UART_HandleTypeDef *huart1, UART_HandleTypeDef *huart2);

		//In initConfigs() worden alle functies hieronder aangeroepen. Dit best gewoon in de constructor steken denk ik.
		void init_Configs();
		void Error_Handler(void);

		void initClock(void);
		void initGPIO(void);
		void initDMA(void);
		void initTIM_1(void);
		void initTIM_2(void);
		void initTIM_3(void);
		void initTIM_4(void);
		void initTIM_8(void);
		void initTIM_16(void);
		void initTIM_20(void);
		void initUSART_1(void);
		void initUSART_2(void);
		void initI2C2(void);
		void HAL_UART_MspInit(void);


		//Getters for handlers because the handlers themselves should stay private
		TIM_HandleTypeDef* get_LED_Timer(void);
		TIM_HandleTypeDef* get_selector_Timer(void);
		TIM_HandleTypeDef* get_pushMotor_Timer(void);
		TIM_HandleTypeDef* get_fluidMotor_Timer(void);
		TIM_HandleTypeDef* get_cleanerMotor_Timer(void);
		UART_HandleTypeDef* get_huart_1(void);
		UART_HandleTypeDef* get_huart_2(void);




	private:

		UART_HandleTypeDef*  _huart1;
		UART_HandleTypeDef* _huart2;

		//Timer for cleaner motor (the big one that scrubs)
		TIM_HandleTypeDef htim1;
		//Timer for push motor
		TIM_HandleTypeDef htim4;
		//Timer for Led
		TIM_HandleTypeDef htim16;
		//Timer for selector actuator
		TIM_HandleTypeDef htim8;
		//Timer for fluid actuator
		TIM_HandleTypeDef htim3;

		//not sure if this is even used... just keep it here for now.
		TIM_HandleTypeDef htim2;

		//getters and setters for now


};
