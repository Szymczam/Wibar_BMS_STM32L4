/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern void MX_FREERTOS_Init(void);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EXT_INT_GPIO_EXTI13_Pin GPIO_PIN_13
#define EXT_INT_GPIO_EXTI13_GPIO_Port GPIOC
#define RTC_I2C3_SCL_Pin GPIO_PIN_0
#define RTC_I2C3_SCL_GPIO_Port GPIOC
#define RTC_I2C3_SDA_Pin GPIO_PIN_1
#define RTC_I2C3_SDA_GPIO_Port GPIOC
#define RTC_GPIO_Input_Pin GPIO_PIN_2
#define RTC_GPIO_Input_GPIO_Port GPIOC
#define BMS_EOC_GPIO_Input_Pin GPIO_PIN_4
#define BMS_EOC_GPIO_Input_GPIO_Port GPIOA
#define BMS_SD_GPIO_Input_Pin GPIO_PIN_5
#define BMS_SD_GPIO_Input_GPIO_Port GPIOA
#define BMS_EOC_PSD_Input_Pin GPIO_PIN_6
#define BMS_EOC_PSD_Input_GPIO_Port GPIOA
#define BMS_INT_GPIO_Input_Pin GPIO_PIN_7
#define BMS_INT_GPIO_Input_GPIO_Port GPIOA
#define BMS_I2C2_SCL_Pin GPIO_PIN_10
#define BMS_I2C2_SCL_GPIO_Port GPIOB
#define BMS_I2C2_SDA_Pin GPIO_PIN_11
#define BMS_I2C2_SDA_GPIO_Port GPIOB
#define LED_ERROR_Pin GPIO_PIN_12
#define LED_ERROR_GPIO_Port GPIOB
#define LED_OK_Pin GPIO_PIN_13
#define LED_OK_GPIO_Port GPIOB
#define LED_CHARGE_Pin GPIO_PIN_14
#define LED_CHARGE_GPIO_Port GPIOB
#define LED_NEED_Pin GPIO_PIN_15
#define LED_NEED_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOC
#define LED3_TIM_CH2_Pin GPIO_PIN_7
#define LED3_TIM_CH2_GPIO_Port GPIOC
#define AUX2_GPIO_Output_Pin GPIO_PIN_5
#define AUX2_GPIO_Output_GPIO_Port GPIOB
#define AUX1_GPIO_Output_Pin GPIO_PIN_9
#define AUX1_GPIO_Output_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
