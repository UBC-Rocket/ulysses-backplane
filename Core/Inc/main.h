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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32g0xx_it.h"
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

/* USER CODE BEGIN Private defines */
/* ========================= Pin Definitions ========================= */

#define Overcurrent_Alert_Pin        GPIO_PIN_4
#define Overcurrent_Alert_GPIO_Port  GPIOA

#define Status_Pin                   GPIO_PIN_5
#define Status_GPIO_Port             GPIOA

#define HighSide_Switching_Pin       GPIO_PIN_6
#define HighSide_Switching_GPIO_Port GPIOA

/* --- Camera control --- */
#define Camera_1_Pin                 GPIO_PIN_14
#define Camera_1_GPIO_Port           GPIOB

#define Camera_2_Pin                 GPIO_PIN_15
#define Camera_2_GPIO_Port           GPIOB

/* --- Hotplug inputs --- */
#define HOTPLUG1_Pin                 GPIO_PIN_2
#define HOTPLUG1_GPIO_Port           GPIOD

#define HOTPLUG2_Pin                 GPIO_PIN_1
#define HOTPLUG2_GPIO_Port           GPIOD

#define HOTPLUG3_Pin                 GPIO_PIN_0
#define HOTPLUG3_GPIO_Port           GPIOD

#define HOTPLUG4_Pin                 GPIO_PIN_12
#define HOTPLUG4_GPIO_Port           GPIOB

/* High-side switching: ACTIVE-LOW (LOW = ON, HIGH = OFF) */
#define TURN_ON_POWER() \
    HAL_GPIO_WritePin(HighSide_Switching_GPIO_Port, \
                      HighSide_Switching_Pin, \
                      GPIO_PIN_RESET)

#define TURN_OFF_POWER() \
    HAL_GPIO_WritePin(HighSide_Switching_GPIO_Port, \
                      HighSide_Switching_Pin, \
                      GPIO_PIN_SET)

/* Status LED: ACTIVE-HIGH (HIGH = ON, LOW = OFF) */
#define TURN_ON_STATUS_LED() \
    HAL_GPIO_WritePin(Status_GPIO_Port, \
                      Status_Pin, \
                      GPIO_PIN_SET)

#define TURN_OFF_STATUS_LED() \
    HAL_GPIO_WritePin(Status_GPIO_Port, \
                      Status_Pin, \
                      GPIO_PIN_RESET)

/* Camera control: assumed ACTIVE-HIGH (change if your logic is inverted) */
#define TURN_ON_CAMERA_1() \
    HAL_GPIO_WritePin(Camera_1_GPIO_Port, \
                      Camera_1_Pin, \
                      GPIO_PIN_SET)

#define TURN_OFF_CAMERA_1() \
    HAL_GPIO_WritePin(Camera_1_GPIO_Port, \
                      Camera_1_Pin, \
                      GPIO_PIN_RESET)

#define TURN_ON_CAMERA_2() \
    HAL_GPIO_WritePin(Camera_2_GPIO_Port, \
                      Camera_2_Pin, \
                      GPIO_PIN_SET)

#define TURN_OFF_CAMERA_2() \
    HAL_GPIO_WritePin(Camera_2_GPIO_Port, \
                      Camera_2_Pin, \
                      GPIO_PIN_RESET)

/* Overcurrent alert read helper */
#define READ_OVERCURRENT_ALERT() \
    HAL_GPIO_ReadPin(Overcurrent_Alert_GPIO_Port, \
                     Overcurrent_Alert_Pin)

/* Hotplug read helpers */
#define READ_HOTPLUG1() HAL_GPIO_ReadPin(HOTPLUG1_GPIO_Port, HOTPLUG1_Pin)
#define READ_HOTPLUG2() HAL_GPIO_ReadPin(HOTPLUG2_GPIO_Port, HOTPLUG2_Pin)
#define READ_HOTPLUG3() HAL_GPIO_ReadPin(HOTPLUG3_GPIO_Port, HOTPLUG3_Pin)
#define READ_HOTPLUG4() HAL_GPIO_ReadPin(HOTPLUG4_GPIO_Port, HOTPLUG4_Pin)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
