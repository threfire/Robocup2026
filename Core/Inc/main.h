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
#include "Emm_V5.h"
#include "tim.h"
#include "adc.h"
#include "useradc.h"

#define APP_QA_TX_BUF_LEN 9U
#define APP_QA_RX_BUF_LEN 100U
#define APP_COLOR_RX_BUF_LEN 2U
#define APP_GY33_CMD_LEN 3U
#define ODOM_TEST_TARGET_X_MM 200.0f
#define ODOM_TEST_TARGET_Y_MM 200.0f
#define ODOM_TEST_HOME_X_MM 0.0f
#define ODOM_TEST_HOME_Y_MM 0.0f
#define ODOM_TEST_REACHED_M 0.003f
#define OPEN_LOOP_TEST_TIME_MS 500U
#define OPEN_LOOP_STOP_TIME_MS 200U
#define CONTROL_DT_S 0.001f
#define TIM6_TICK_MS 1U
#define MOTOR_CMD_DIVIDER 2U
#define STATUS_TICK_DIVIDER 1000U
#define TASK2_ZERO_SETTLE_MS 100U
#define ODOMETER_ZERO_FINAL_WAIT_MS 8000U

typedef struct {
	uint8_t is_light;
	uint8_t is_followline;
	uint8_t qa_tx_buf[APP_QA_TX_BUF_LEN];
	uint8_t qa_rx_buf[APP_QA_RX_BUF_LEN];
	uint8_t qa_num;
	uint8_t color_rx_data[APP_COLOR_RX_BUF_LEN];
	uint32_t camera_time_ms;
	uint8_t camera_timeout_enable;
	uint8_t debug_enable;
	uint8_t start_flag;
	uint8_t task2_start_pending;
	uint32_t task2_zero_confirm_time_ms;
	uint16_t led_count;
	uint32_t run_time_ms;
	uint8_t gy33_white_balance_cmd[APP_GY33_CMD_LEN];
	uint8_t gy33_read_cmd[APP_GY33_CMD_LEN];
	uint8_t gy33_baud_9600_cmd[APP_GY33_CMD_LEN];
	uint8_t gy33_range_24ms_cmd[APP_GY33_CMD_LEN];
	uint8_t chassis_velocity_mode;
} AppContext_t;

extern AppContext_t app_ctx;

#define IsLight app_ctx.is_light
#define IsFollowline app_ctx.is_followline
#define QAtx_buff app_ctx.qa_tx_buf
#define QArxbuff app_ctx.qa_rx_buf
#define qanum app_ctx.qa_num
#define rx_data app_ctx.color_rx_data
#define camtim app_ctx.camera_time_ms
#define camtim_button app_ctx.camera_timeout_enable
#define debug app_ctx.debug_enable
#define Start_flag app_ctx.start_flag
#define task2_start_pending app_ctx.task2_start_pending
#define task2_zero_confirm_time_ms app_ctx.task2_zero_confirm_time_ms
#define led_count app_ctx.led_count
#define timcount app_ctx.run_time_ms
#define gy33sendbuffer1 app_ctx.gy33_white_balance_cmd
#define gy33sendbuffer2 app_ctx.gy33_read_cmd
#define gy33baudbuffer app_ctx.gy33_baud_9600_cmd
#define gy33rangebuffer app_ctx.gy33_range_24ms_cmd
#define Is_vel_mode app_ctx.chassis_velocity_mode
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
void Robot_RealtimeControl(float dt);
void Robot_ServiceTick(void);
extern volatile uint32_t robot_control_current_us;
extern volatile uint32_t robot_control_max_us;
extern volatile uint32_t robot_control_overrun_count;
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
