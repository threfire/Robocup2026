/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
#include "key.h"
#include "lcd_spi_154.h"
#include "control.h"
#include "woniudonghua.h"
#include "usart_manager.h"
#include "userkey.h"
#include "servo.h"
#include "odometer.h"
#include "graysensor.h"
#include "mxcam.h"
#include "QAma.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
AppContext_t app_ctx = {
        .qa_tx_buf = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD},
        .gy33_white_balance_cmd = {0xA5, 0xbb, 0x60},
        .gy33_read_cmd = {0xA5, 0xCC, 0x71},
        .chassis_velocity_mode = 1,
				.debug_enable = 0,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void Robot_ServiceTick(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  MX_TIM13_Init();
  MX_TIM11_Init();
  MX_TIM14_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
	/* 板载 LCD 用于启动阶段状态提示。 */
	SPI_LCD_Init();
	LCD_Clear();
	LCD_DisplayText(48, 0, "Initting...");

	/* UART4 接入里程计，使用 DMA + IDLE 中断获得完整帧。 */
	Usart4_DMA_IDLE_Init();
	HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxCmd, CMD_LEN);

	/* CAN1 用于 X42 电机和其他 CAN 设备通信。 */
	USER_CAN1_Filter_Init();
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) !=
	        HAL_OK) {
		Error_Handler();
	}
	task1_state.active = 0;
	task1_state.completed = 0;
	task2_state.active = 0;
	task2_state.completed = 0;
	/* 麦轮几何参数和速度上限在启动时写入控制模块。 */
	ChassisConfig_t config = {.wheel_radius = 0.077f,
	        .wheel_dist_x = 0.0759f,
	        .wheel_dist_y = 0.085f,
	        .max_rpm = CHASSIS_SPEED_MAX_RPM};
	Mecanum_Init(config);
	/* 复位后立即下发四轮 0 RPM，清除电机保留的上一条速度指令。 */
	Mecanum_SendVelocity();
	/* 等待里程计上电后进入稳定输出，再初始化零点状态。 */
	HAL_Delay(2000);
	odometer_init();
	/* 控制环模块初始化：视觉、位置、位姿协同和姿态环。 */
	CamStabilization_Init();
	PositionControl_Init();
	PoseCoordination_Init();
	AttitudeStabilization_Init();
	memset(rx_data, 0, sizeof(rx_data));
	/* USART3 颜色传感器按字节中断接收。 */
	HAL_UART_Receive_IT(&huart3, rx_data, 1);
	CamStabilization_Enable(0);

	LCD_DisplayText(48, 0, "Init Success!");
	HAL_Delay(100);
	/* 已完成 2 s 里程计稳定等待，补足 8 s 后发送最后一条初始化指令。 */
	HAL_Delay(ODOMETER_ZERO_FINAL_WAIT_MS);
	(void)odometer_request_zero();

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void Robot_HandleDebugModeTransition(void) {
	static uint8_t initialized = 0U;
	static uint8_t previous_debug;

	if (!initialized) {
		previous_debug = debug;
		initialized = 1U;
		return;
	}
	if (previous_debug == debug) {
		return;
	}

	previous_debug = debug;
	Start_flag = 0U;
	task1_start_pending = 0U;
	task1_zero_confirm_time_ms = 0U;
	task1_state.active = 0U;
	task2_state.active = 0U;
	PositionControl_Enable(0U);
	EnableAttitudeStabilization(0U);
	CamStabilization_Enable(0U);
	stop();
	Mecanum_SendVelocity();
}

void Robot_RealtimeControl(float dt) {
	if (dt > 0.0f) {
		/* FreeRTOS 固定周期任务调用本函数，dt 为控制周期秒数。 */
		Robot_HandleDebugModeTransition();
		led_count++;
		timcount++;
		if (camtim_button) {
			camtim += TIM6_TICK_MS;
		}
		UpdatePoseCoordination(dt);
		if (is_position_control_enabled) {
			UpdatePositionControl(dt);
		}
		UpdateAttitudeStabilization(dt);
		Process_usart1_packet();
		if (cam_stabilization.is_enabled) {

			UpdateCamStabilization(Camera_err.x, Camera_err.y, dt);
		}
		TargetVelocity_t current_target = target_velocity;
		if (!Start_flag) {
			current_target.vx = 0.0f;
			current_target.vy = 0.0f;
			current_target.omega = 0.0f;
		}

		/* 速度目标统一进入麦轮逆解，再由 X42 速度模式发送。 */
		Mecanum_Kinematics(current_target.vx, current_target.vy,
		        current_target.omega);
		if (Is_vel_mode) {
			if ((led_count % MOTOR_CMD_DIVIDER) == 0U) {
				Mecanum_SendVelocity();
			}
		}
		if (led_count >= STATUS_TICK_DIVIDER) {
			/* 状态周期任务：发送视觉模式并翻转 LED 表示系统运行。 */
			HAL_UART_Transmit_IT(
			        &huart1, &cammode, sizeof(cammode));
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			led_count = 0;
		}
		/* 串口 DMA 周期性重启，维持 MaixCam 接收链路。 */
		DMA_resend();
		if (IsLight)
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 200);
		else
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
	}
}
void Robot_ServiceTick(void) {
	/* 低频服务任务处理 LCD、按键、舵机和任务状态机。 */
	Robot_HandleDebugModeTransition();
	LCD_DisplayNumber(0, 20, timcount, 6);

	Key_Tick();
	if (Key_Num == 1) {
		Key_Num = 0;
		Start_flag = 0U;
		task1_start_pending = odometer_request_zero();
		task1_zero_confirm_time_ms = 0U;
	}
	if ((task1_start_pending == 1U) && odometer_is_zeroed()) {
		task1_start_pending = 2U;
		task1_zero_confirm_time_ms = HAL_GetTick();
	}
	if ((task1_start_pending == 2U) &&
	        ((HAL_GetTick() - task1_zero_confirm_time_ms) >=
	                TASK1_ZERO_SETTLE_MS)) {
		task1_start_pending = 0U;
		Start_flag = 1U;
		servo_init();
		start_task1();
	}
	run_task1();
}

int abs_int(int int1) {
	if (int1 > 0)
		return int1;
	return (-int1);
}

float abs_float(float value) {
	if (value > 0) {
		return value;
	}
	return (-value);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM12 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM12) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return
	 * state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line
	   number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
	   file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
