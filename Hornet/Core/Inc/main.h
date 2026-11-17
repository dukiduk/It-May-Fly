/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define CONTROL_DT 0.0005
#define Voltage_ADC_Pin GPIO_PIN_3
#define Voltage_ADC_GPIO_Port GPIOA
#define Current_ADC_Pin GPIO_PIN_4
#define Current_ADC_GPIO_Port GPIOA
#define GNSS_RESET_Pin GPIO_PIN_13
#define GNSS_RESET_GPIO_Port GPIOD
#define GNSS_CS_Pin GPIO_PIN_14
#define GNSS_CS_GPIO_Port GPIOD
#define GNSS_TIMEPULSE_Pin GPIO_PIN_15
#define GNSS_TIMEPULSE_GPIO_Port GPIOD
#define GNSS_TIMEPULSE_EXTI_IRQn EXTI15_10_IRQn
#define MAG_INT_Pin GPIO_PIN_4
#define MAG_INT_GPIO_Port GPIOG
#define MAG_INT_EXTI_IRQn EXTI4_IRQn
#define MAG_CS_Pin GPIO_PIN_5
#define MAG_CS_GPIO_Port GPIOG
#define BARO_CS_Pin GPIO_PIN_6
#define BARO_CS_GPIO_Port GPIOG
#define IMU_INT_Pin GPIO_PIN_7
#define IMU_INT_GPIO_Port GPIOG
#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn
#define IMU_CS_Pin GPIO_PIN_8
#define IMU_CS_GPIO_Port GPIOG
#define SERVO_TIM_CH1_Pin GPIO_PIN_6
#define SERVO_TIM_CH1_GPIO_Port GPIOC
#define SERVO_TIM_CH2_Pin GPIO_PIN_7
#define SERVO_TIM_CH2_GPIO_Port GPIOC
#define SERVO_TIM_CH3_Pin GPIO_PIN_8
#define SERVO_TIM_CH3_GPIO_Port GPIOC
#define SERVO_TIM_CH4_Pin GPIO_PIN_9
#define SERVO_TIM_CH4_GPIO_Port GPIOC
#define ESC_TIM_CH1_Pin GPIO_PIN_8
#define ESC_TIM_CH1_GPIO_Port GPIOA
#define COMMS_BUSY_Pin GPIO_PIN_11
#define COMMS_BUSY_GPIO_Port GPIOG
#define COMMS_RESET_Pin GPIO_PIN_12
#define COMMS_RESET_GPIO_Port GPIOG
#define COMMS_INT_Pin GPIO_PIN_13
#define COMMS_INT_GPIO_Port GPIOG
#define COMMS_INT_EXTI_IRQn EXTI15_10_IRQn
#define COMMS_CS_Pin GPIO_PIN_14
#define COMMS_CS_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
