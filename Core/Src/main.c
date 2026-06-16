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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
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
#include "hwt_imu.h"
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
extern TaskState_t task1_state; //声明任务一结构体
uint8_t QAtx_buff[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};//二维码扫一次指令（没用到）
uint8_t QArxbuff[100];//二维码接收buffer
uint8_t uart5buff[100];//UART5接收buffer：HWT906
uint8_t qanum = 0;//定义并初始化二维码接收变量
uint8_t rx_data[2];//颜色传感器接收buffer
uint32_t camtim = 0;//摄像头超时计时（好像没用到）
uint8_t camtim_button = 0;//摄像头超时计时开关
uint8_t debug = 0;//调试
uint8_t Start_flag = 0;//软件开始开关
uint8_t IsLight = 0;//开灯（初始化为关灯）
uint16_t led_count;//led翻转计时
uint32_t timcount;//系统运行计时
uint8_t gy33sendbuffer1[] = {0xA5,0xbb,0x60};//白平衡校准
uint8_t gy33sendbuffer2[] = {0xA5,0xCC,0x71};//gy33亮度调节指令
uint8_t Is_vel_mode = 1;//是否速度模式（初始化为速度模式）
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Main_Tasktick(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ODOM_TEST_TARGET_X_MM 200.0f
#define ODOM_TEST_TARGET_Y_MM 200.0f
#define ODOM_TEST_HOME_X_MM   0.0f
#define ODOM_TEST_HOME_Y_MM   0.0f
#define ODOM_TEST_REACHED_M   0.003f
#define OPEN_LOOP_TEST_TIME_MS 500U
#define OPEN_LOOP_STOP_TIME_MS 200U

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
	SPI_LCD_Init();
	LCD_Clear();
	LCD_DisplayText(48, 0,"Initting...");
/*************************************************************初 始 化 开 始*********************************************************************/
/*************************************************************初 始 化 开 始*********************************************************************/
/******************UART4新OPS里程计DMA空闲中断接收初始化********************/
	Usart4_DMA_IDLE_Init();
	HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxCmd, CMD_LEN); 
/******************can初始化********************/
	USER_CAN1_Filter_Init();												// 初始化CAN滤波器
	if(HAL_CAN_Start(&hcan1) != HAL_OK) { Error_Handler(); }				// 启动CAN控制器
	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) { Error_Handler(); }	// 使能CAN控制器接收中断
//******************任务初始化********************//
	task1_state.active = 0; 
	task1_state.completed = 0;
//******************运动解析参数配置********************//	
	ChassisConfig_t config = {
        .wheel_radius = 0.077f,		// 轮子半径（米）
        .wheel_dist_x = 0.0759f,	// X轴轮距（米）
        .wheel_dist_y = 0.085f,		// Y轴轮距（米）
        .max_rpm = 300.0f			// 电机最大转速（RPM）
    };
    Mecanum_Init(config);
	HAL_Delay(2000);
//******************传感器和pid初始化********************//
	hwt_imu_init();					// 初始化HWT906解析器
	CamStabilization_Init();		// 初始化视觉pid控制器
	PositionControl_Init(); 		// 初始化位置pid控制器
    AttitudeStabilization_Init();	//初始化角度pid控制器
//	servo_init();					//舵机初始化
	AdcGet_Init();					//红外读取初始化		
	memset(rx_data, 0, sizeof(rx_data));		//初始化颜色传感器接收变量内存
	HAL_UART_Receive_IT(&huart3, rx_data, 1);	//开启颜色传感器参数接收
	CamStabilization_Enable(0);					//摄像头矫正关
	EnableAttitudeStabilization(0);				//姿态矫正关
/*************************************************************初 始 化 结 束*********************************************************************/
/*************************************************************初 始 化 结 束*********************************************************************/
	LCD_DisplayText(48, 0,"Init Success!");
	HAL_Delay(100);
	if(debug){		//调试标志
	//	HAL_UART_Transmit_IT(&huart3,gy33sendbuffer1 ,sizeof (gy33sendbuffer1 ));
	//	HAL_UART_Transmit_IT(&huart3,gy33sendbuffer1 ,sizeof (gy33sendbuffer2 ));
		HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
		Start_flag = 1;
		EnableAttitudeStabilization(1);
		SetTargetYaw(0.0f);
		pospid_reinit(LOWSPEED);
		while (!move_to_target(300.0f, 300.0f)) {
			HAL_Delay(5);
		}
		HAL_Delay(3000);
//		while (!move_to_target(ODOM_TEST_HOME_X_MM, ODOM_TEST_HOME_Y_MM)) {
//			HAL_Delay(5);
//		}
		EnableAttitudeStabilization(0);
		stop();
		{
		motor1.speed_rpm= 0;motor2.speed_rpm = 0;motor3.speed_rpm= 0;motor4.speed_rpm = 0;}
		HAL_Delay (100);
		Start_flag = 0;
//		HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
//		HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
//		Servo_Action_Group(3);
//		HAL_Delay(1000);
//		Servo_Action_Group(4);
//		PositionControl_Enable(0);
//		CamStabilization_Enable(1);	
	}
//******************等待按钮按下启动********************//
	while(1){
//		Get_Materialcolor(1);		//测试颜色阈值，平时注释
		Key_Tick();
		if (Key_Num == 1) {
			Start_flag = 1;
			Key_Num = 0;
			Update_zero();					//初始清零ops9（不能放前面因为第一次启动ops9自己没初始化完成）
			HAL_Delay(100);					//等待100msops9数据清零稳定
		}
		if(Start_flag){
			servo_init();
			if(!debug && !is_task1_active()){			//启动任务1
			start_task1();}
			break ;
		}
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Main_Tasktick();		//主循环任务（在下面定义）

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
//****************************************中断回调服务函数*****************************//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6) // 检查是否是 TIM6 触发中断
    {
		led_count++;
		timcount ++;
		if(camtim_button){camtim +=5;}//摄像头超时计时，移动精度高用不到
		if (is_position_control_enabled) {
			UpdatePositionControl(0.005f);//更新位置环（200hz）
		}
		Process_usart1_packet();//解析maxcam数据200hz
		if (cam_stabilization.is_enabled) {
			
            UpdateCamStabilization(Camera_err.x, Camera_err.y, 0.005f);//更新视觉环
        }
		UpdateAttitudeStabilization(0.005f);//更新角度环
		// 添加临界区保护
        __disable_irq();
        TargetVelocity_t current_target = target_velocity;
        __enable_irq();
        
        // 实时执行运动学解算
        Mecanum_Kinematics(
            current_target.vx,
            current_target.vy,
            current_target.omega
        );
		if(Is_vel_mode){//发送运动指令100hz
			if(led_count %2 ==0){
				if(Start_flag){
					ZDT_X42_V2_Velocity_Control(1, motor1.direction, 230, (float)abs_int(motor1.speed_rpm), 0);
					ZDT_X42_V2_Velocity_Control(2, motor2.direction, 230, (float)abs_int(motor2.speed_rpm), 0);
					ZDT_X42_V2_Velocity_Control(3, motor3.direction, 230, (float)abs_int(motor3.speed_rpm), 0);
					ZDT_X42_V2_Velocity_Control(4, motor4.direction, 230, (float)abs_int(motor4.speed_rpm), 0);
				}
			}	
		}
		if(led_count == 100)//系统正常运行提示灯
		{
			HAL_UART_Transmit_IT(&huart1,&cammode,sizeof (cammode));
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			led_count = 0;
		}
		// 其他任务
		DMA_resend();//串口1dma重新开启的调用
		if(IsLight)__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,200);else __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,0);//牢灯
    }
}
//*************************主循环中执行函数************************************************//
void Main_Tasktick(void)
{
	LCD_DisplayNumber(0,20,timcount,6);//显示系统计时

    run_task1();

    __WFI(); // 进入低功耗模式等待中断
}


//*************************************取绝对值函数******************************//
// int型 取绝对值函数
int abs_int(int int1)
{
    if (int1 > 0)
        return int1;
    return (-int1);
}

float abs_float(float value)
{
    if (value > 0)
    {
        return value;
    }
    return (-value);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
