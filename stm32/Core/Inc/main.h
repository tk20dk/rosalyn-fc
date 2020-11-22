/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f7xx_hal.h"

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
#define BEEPER_Pin GPIO_PIN_13
#define BEEPER_GPIO_Port GPIOC
#define BARO_NSS_Pin GPIO_PIN_15
#define BARO_NSS_GPIO_Port GPIOC
#define VBAT_ADC_Pin GPIO_PIN_1
#define VBAT_ADC_GPIO_Port GPIOC
#define CURRENT_ADC_Pin GPIO_PIN_2
#define CURRENT_ADC_GPIO_Port GPIOC
#define RSSI_ADC_Pin GPIO_PIN_3
#define RSSI_ADC_GPIO_Port GPIOC
#define GYRO1_NSS_Pin GPIO_PIN_4
#define GYRO1_NSS_GPIO_Port GPIOA
#define GYRO1_EXTI_Pin GPIO_PIN_4
#define GYRO1_EXTI_GPIO_Port GPIOC
#define M3_Pin GPIO_PIN_0
#define M3_GPIO_Port GPIOB
#define M4_Pin GPIO_PIN_1
#define M4_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_2
#define LED0_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define M2_Pin GPIO_PIN_7
#define M2_GPIO_Port GPIOC
#define FLASH_CS_Pin GPIO_PIN_8
#define FLASH_CS_GPIO_Port GPIOC
#define M1_Pin GPIO_PIN_9
#define M1_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOA
#define PPM_Pin GPIO_PIN_15
#define PPM_GPIO_Port GPIOA
#define M5_Pin GPIO_PIN_6
#define M5_GPIO_Port GPIOB
#define M6_Pin GPIO_PIN_8
#define M6_GPIO_Port GPIOB
#define CAMERA_CONTROL_Pin GPIO_PIN_9
#define CAMERA_CONTROL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
