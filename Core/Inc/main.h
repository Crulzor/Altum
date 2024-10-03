/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_device.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LINEAR_FIN1_Pin GPIO_PIN_13
#define LINEAR_FIN1_GPIO_Port GPIOC
#define gled_pc14_Pin GPIO_PIN_14
#define gled_pc14_GPIO_Port GPIOC
#define PC15_RTS_Pin GPIO_PIN_15
#define PC15_RTS_GPIO_Port GPIOC
#define MOTOR_TIM1_3N_Pin GPIO_PIN_0
#define MOTOR_TIM1_3N_GPIO_Port GPIOF
#define PF1_CTS_Pin GPIO_PIN_1
#define PF1_CTS_GPIO_Port GPIOF
#define PG10_NRST_Pin GPIO_PIN_10
#define PG10_NRST_GPIO_Port GPIOG
#define ADC_BATT_Pin GPIO_PIN_0
#define ADC_BATT_GPIO_Port GPIOA
#define SHUNT_MOTOR_Pin GPIO_PIN_1
#define SHUNT_MOTOR_GPIO_Port GPIOA
#define PA4_GPIO_Pin GPIO_PIN_4
#define PA4_GPIO_GPIO_Port GPIOA
#define led_tim16_1_Pin GPIO_PIN_6
#define led_tim16_1_GPIO_Port GPIOA
#define SHUNT_LIN3_Pin GPIO_PIN_0
#define SHUNT_LIN3_GPIO_Port GPIOB
#define ADC_LIN3_Pin GPIO_PIN_1
#define ADC_LIN3_GPIO_Port GPIOB
#define ADC_LIN2_Pin GPIO_PIN_11
#define ADC_LIN2_GPIO_Port GPIOB
#define ADC_LIN1_Pin GPIO_PIN_12
#define ADC_LIN1_GPIO_Port GPIOB
#define SHUNT_LIN2_Pin GPIO_PIN_13
#define SHUNT_LIN2_GPIO_Port GPIOB
#define SHUNT_LIN1_Pin GPIO_PIN_14
#define SHUNT_LIN1_GPIO_Port GPIOB
#define LINEAR_RIN3_Pin GPIO_PIN_4
#define LINEAR_RIN3_GPIO_Port GPIOB
#define LINEAR_FIN3_Pin GPIO_PIN_5
#define LINEAR_FIN3_GPIO_Port GPIOB
#define LINEAR_RIN2_Pin GPIO_PIN_7
#define LINEAR_RIN2_GPIO_Port GPIOB
#define LINEAR_FIN2_TIM4CH3_Pin GPIO_PIN_8
#define LINEAR_FIN2_TIM4CH3_GPIO_Port GPIOB
#define LINEAR_RIN1_TIM8CH3_Pin GPIO_PIN_9
#define LINEAR_RIN1_TIM8CH3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define RX_BUFFER_SIZE 20

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
