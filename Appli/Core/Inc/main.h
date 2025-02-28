/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7rsxx_hal.h"

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
#define RF_SET_Pin GPIO_PIN_13
#define RF_SET_GPIO_Port GPIOC
#define SDMMC_CD_Pin GPIO_PIN_5
#define SDMMC_CD_GPIO_Port GPIOE
#define LTDC_DISP_Pin GPIO_PIN_6
#define LTDC_DISP_GPIO_Port GPIOF
#define ILLUM_Pin GPIO_PIN_0
#define ILLUM_GPIO_Port GPIOC
#define LSO1_Pin GPIO_PIN_2
#define LSO1_GPIO_Port GPIOC
#define LSO2_Pin GPIO_PIN_3
#define LSO2_GPIO_Port GPIOC
#define HC_EN_Pin GPIO_PIN_4
#define HC_EN_GPIO_Port GPIOC
#define HC_State_Pin GPIO_PIN_5
#define HC_State_GPIO_Port GPIOC
#define DebugG_Pin GPIO_PIN_0
#define DebugG_GPIO_Port GPIOB
#define Buzz_Pin GPIO_PIN_7
#define Buzz_GPIO_Port GPIOE
#define DebugR_Pin GPIO_PIN_6
#define DebugR_GPIO_Port GPIOC
#define DebugB_Pin GPIO_PIN_7
#define DebugB_GPIO_Port GPIOC
#define CAN1_STBY_Pin GPIO_PIN_12
#define CAN1_STBY_GPIO_Port GPIOE
#define LCD_RST_Pin GPIO_PIN_13
#define LCD_RST_GPIO_Port GPIOE
#define LCD_INT_Pin GPIO_PIN_14
#define LCD_INT_GPIO_Port GPIOE
#define BL_CTRL_Pin GPIO_PIN_5
#define BL_CTRL_GPIO_Port GPIOD
#define RF_CS_Pin GPIO_PIN_1
#define RF_CS_GPIO_Port GPIOE
#define CAN2_STBY_Pin GPIO_PIN_2
#define CAN2_STBY_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
