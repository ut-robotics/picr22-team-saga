/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usbd_customhid.h"
#include "dshot.h"
#include "usb_device.h"
#include "utils.h"
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
#define M1_DIR_1_Pin GPIO_PIN_0
#define M1_DIR_1_GPIO_Port GPIOF
#define M1_DIR_2_Pin GPIO_PIN_1
#define M1_DIR_2_GPIO_Port GPIOF
#define M1_ENCA_Pin GPIO_PIN_0
#define M1_ENCA_GPIO_Port GPIOA
#define M1_ENCB_Pin GPIO_PIN_1
#define M1_ENCB_GPIO_Port GPIOA
#define Thrower_PWM_Pin GPIO_PIN_2
#define Thrower_PWM_GPIO_Port GPIOA
#define M2_ENCB_Pin GPIO_PIN_4
#define M2_ENCB_GPIO_Port GPIOA
#define M2_ENCA_Pin GPIO_PIN_6
#define M2_ENCA_GPIO_Port GPIOA
#define M3_ENCA_Pin GPIO_PIN_8
#define M3_ENCA_GPIO_Port GPIOA
#define M3_ENCB_Pin GPIO_PIN_9
#define M3_ENCB_GPIO_Port GPIOA
#define nSleep_Pin GPIO_PIN_10
#define nSleep_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define Servo_PWM_Pin GPIO_PIN_15
#define Servo_PWM_GPIO_Port GPIOA
#define M3_DIR_Pin GPIO_PIN_3
#define M3_DIR_GPIO_Port GPIOB
#define M3_PWM_Pin GPIO_PIN_4
#define M3_PWM_GPIO_Port GPIOB
#define M2_DIR_Pin GPIO_PIN_5
#define M2_DIR_GPIO_Port GPIOB
#define M2_PWM_Pin GPIO_PIN_6
#define M2_PWM_GPIO_Port GPIOB
#define M1_DIR_Pin GPIO_PIN_7
#define M1_DIR_GPIO_Port GPIOB
#define M1_PWM_Pin GPIO_PIN_8
#define M1_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */