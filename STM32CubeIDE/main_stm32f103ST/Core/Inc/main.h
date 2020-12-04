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
#include "stm32f1xx_hal.h"

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
extern void DefaultTask(void const * argument);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LINE_TB66_AIN2_Pin GPIO_PIN_0
#define LINE_TB66_AIN2_GPIO_Port GPIOA
#define LINE_TB66_AIN1_Pin GPIO_PIN_1
#define LINE_TB66_AIN1_GPIO_Port GPIOA
#define LINE_TB66_STBY_Pin GPIO_PIN_2
#define LINE_TB66_STBY_GPIO_Port GPIOA
#define LINE_TB66_BIN2_Pin GPIO_PIN_4
#define LINE_TB66_BIN2_GPIO_Port GPIOA
#define LINE_TB66_BIN1_Pin GPIO_PIN_5
#define LINE_TB66_BIN1_GPIO_Port GPIOA
#define LINE_TB66_PWMB_Pin GPIO_PIN_6
#define LINE_TB66_PWMB_GPIO_Port GPIOA
#define LINE_TB66_PWMA_Pin GPIO_PIN_7
#define LINE_TB66_PWMA_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
void main_cpp(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
