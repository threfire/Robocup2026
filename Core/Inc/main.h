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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "ZDT_X42_V2.h"
#include "tim.h"
#include "adc.h"
#include "useradc.h"
extern uint8_t IsLight;
extern uint8_t IsFollowline;
extern uint64_t Tick_Time;
extern uint8_t QAtx_buff[];
extern uint8_t QArxbuff[100];
extern uint8_t HWT101rxbuff[100];
extern uint8_t uart5buff[100];
extern uint8_t qanum;
extern uint8_t rx_data[2];
extern uint8_t Is_vel_mode;
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
int abs_int(int int1);
float abs_float(float value);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_15
#define KEY2_GPIO_Port GPIOD
#define KEY3_Pin GPIO_PIN_2
#define KEY3_GPIO_Port GPIOG
#define KEY4_Pin GPIO_PIN_3
#define KEY4_GPIO_Port GPIOG
#define KEY5_Pin GPIO_PIN_4
#define KEY5_GPIO_Port GPIOG
#define KEY1_Pin GPIO_PIN_15
#define KEY1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
