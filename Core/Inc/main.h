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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define T9C1_friwheel_Pin GPIO_PIN_5
#define T9C1_friwheel_GPIO_Port GPIOE
#define T2C3_reload_Pin GPIO_PIN_10
#define T2C3_reload_GPIO_Port GPIOB
#define T2C4_reload_Pin GPIO_PIN_11
#define T2C4_reload_GPIO_Port GPIOB
#define T4C1_upload_Pin GPIO_PIN_12
#define T4C1_upload_GPIO_Port GPIOD
#define T4C3_maxload_Pin GPIO_PIN_14
#define T4C3_maxload_GPIO_Port GPIOD
#define T4C4_Catch_Pin GPIO_PIN_15
#define T4C4_Catch_GPIO_Port GPIOD
#define T8C1_friwheel_Pin GPIO_PIN_6
#define T8C1_friwheel_GPIO_Port GPIOC
#define T8C2_friwheel_Pin GPIO_PIN_7
#define T8C2_friwheel_GPIO_Port GPIOC
#define T1C1_friwheel_Pin GPIO_PIN_8
#define T1C1_friwheel_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
