/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BTN_DOWN_Pin GPIO_PIN_14
#define BTN_DOWN_GPIO_Port GPIOC
#define BTN_DOWN_EXTI_IRQn EXTI15_10_IRQn
#define DHT11_DAT_Pin GPIO_PIN_15
#define DHT11_DAT_GPIO_Port GPIOC
#define BTN_UP_Pin GPIO_PIN_0
#define BTN_UP_GPIO_Port GPIOA
#define RELAY_Pin GPIO_PIN_0
#define RELAY_GPIO_Port GPIOB
#define DS1623_RST_N_Pin GPIO_PIN_1
#define DS1623_RST_N_GPIO_Port GPIOB
#define DS1623_CLK_Pin GPIO_PIN_10
#define DS1623_CLK_GPIO_Port GPIOB
#define DS1623_DQ_Pin GPIO_PIN_11
#define DS1623_DQ_GPIO_Port GPIOB
#define LC12S_SET_Pin GPIO_PIN_8
#define LC12S_SET_GPIO_Port GPIOA
#define TM1637_CLK_Pin GPIO_PIN_3
#define TM1637_CLK_GPIO_Port GPIOB
#define TM1637_IO_Pin GPIO_PIN_4
#define TM1637_IO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
