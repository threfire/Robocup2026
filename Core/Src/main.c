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
extern TaskState_t task1_state; //澹版槑浠诲姟涓€缁撴瀯浣?
uint8_t QAtx_buff[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};//浜岀淮鐮佹壂涓€娆℃寚浠わ紙娌＄敤鍒帮級
uint8_t QArxbuff[100];//浜岀淮鐮佹帴鏀禸uffer
uint8_t uart5buff[100];//UART5鎺ユ敹buffer锛欻WT906
uint8_t qanum = 0;//瀹氫箟骞跺垵濮嬪寲浜岀淮鐮佹帴鏀跺彉閲?
uint8_t rx_data[2];//棰滆壊浼犳劅鍣ㄦ帴鏀禸uffer
uint32_t camtim = 0;//鎽勫儚澶磋秴鏃惰鏃讹紙濂藉儚娌＄敤鍒帮級
uint8_t camtim_button = 0;//鎽勫儚澶磋秴鏃惰鏃跺紑鍏?
uint8_t debug = 0;//璋冭瘯
uint8_t Start_flag = 0;//杞欢寮€濮嬪紑鍏?
uint8_t IsLight = 0;//寮€鐏紙鍒濆鍖栦负鍏崇伅锛?
uint16_t led_count;//led缈昏浆璁℃椂
uint32_t timcount;//绯荤粺杩愯璁℃椂
uint8_t gy33sendbuffer1[] = {0xA5,0xbb,0x60};//鐧藉钩琛℃牎鍑?
uint8_t gy33sendbuffer2[] = {0xA5,0xCC,0x71};//gy33浜害璋冭妭鎸囦护
uint8_t Is_vel_mode = 1;//鏄惁閫熷害妯″紡锛堝垵濮嬪寲涓洪€熷害妯″紡锛?
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
#define CONTROL_DT_S          0.001f
#define TIM6_TICK_MS          1U
#define MOTOR_CMD_DIVIDER     2U
#define STATUS_TICK_DIVIDER   1000U

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
/*************************************************************鍒?濮?鍖?寮€ 濮?********************************************************************/
/*************************************************************鍒?濮?鍖?寮€ 濮?********************************************************************/
/******************UART4鏂癘PS閲岀▼璁MA绌洪棽涓柇鎺ユ敹鍒濆鍖?*******************/
	Usart4_DMA_IDLE_Init();
	HAL_UART_Receive_DMA(&huart4, (uint8_t *)rxCmd, CMD_LEN); 
/******************can鍒濆鍖?*******************/
	USER_CAN1_Filter_Init();												// 鍒濆鍖朇AN婊ゆ尝鍣?
	if(HAL_CAN_Start(&hcan1) != HAL_OK) { Error_Handler(); }				// 鍚姩CAN鎺у埗鍣?
	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) { Error_Handler(); }	// 浣胯兘CAN鎺у埗鍣ㄦ帴鏀朵腑鏂?
//******************浠诲姟鍒濆鍖?*******************//
	task1_state.active = 0; 
	task1_state.completed = 0;
//******************杩愬姩瑙ｆ瀽鍙傛暟閰嶇疆********************//	
	ChassisConfig_t config = {
        .wheel_radius = 0.077f,		// 杞瓙鍗婂緞锛堢背锛?
        .wheel_dist_x = 0.0759f,	// X杞磋疆璺濓紙绫筹級
        .wheel_dist_y = 0.085f,		// Y杞磋疆璺濓紙绫筹級
        .max_rpm = 300.0f			// 鐢垫満鏈€澶ц浆閫燂紙RPM锛?
    };
    Mecanum_Init(config);
	HAL_Delay(2000);
//******************浼犳劅鍣ㄥ拰pid鍒濆鍖?*******************//
	hwt_imu_init();					// 鍒濆鍖朒WT906瑙ｆ瀽鍣?
	CamStabilization_Init();		// 鍒濆鍖栬瑙塸id鎺у埗鍣?
	PositionControl_Init(); 		// 鍒濆鍖栦綅缃畃id鎺у埗鍣?
    PoseCoordination_Init();
    AttitudeStabilization_Init();
	memset(rx_data, 0, sizeof(rx_data));		//鍒濆鍖栭鑹蹭紶鎰熷櫒鎺ユ敹鍙橀噺鍐呭瓨
	HAL_UART_Receive_IT(&huart3, rx_data, 1);	//寮€鍚鑹蹭紶鎰熷櫒鍙傛暟鎺ユ敹
	CamStabilization_Enable(0);					//鎽勫儚澶寸煫姝ｅ叧
/*************************************************************鍒?濮?鍖?缁?鏉?********************************************************************/
/*************************************************************鍒?濮?鍖?缁?鏉?********************************************************************/
	LCD_DisplayText(48, 0,"Init Success!");
	HAL_Delay(100);
	if(debug){		//璋冭瘯鏍囧織
	//	HAL_UART_Transmit_IT(&huart3,gy33sendbuffer1 ,sizeof (gy33sendbuffer1 ));
	//	HAL_UART_Transmit_IT(&huart3,gy33sendbuffer1 ,sizeof (gy33sendbuffer2 ));
		HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
		Start_flag = 1;
		POS_move();
//		HAL_Delay(3000);
//		while (!move_to_target(ODOM_TEST_HOME_X_MM, ODOM_TEST_HOME_Y_MM)) {
//			HAL_Delay(5);
//		}
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
//******************绛夊緟鎸夐挳鎸変笅鍚姩********************//
	while(1){
//		Get_Materialcolor(1);		//娴嬭瘯棰滆壊闃堝€硷紝骞虫椂娉ㄩ噴
		Key_Tick();
		if (Key_Num == 1) {
			Start_flag = 1;
			Key_Num = 0;
			Update_zero();					//鍒濆娓呴浂ops9锛堜笉鑳芥斁鍓嶉潰鍥犱负绗竴娆″惎鍔╫ps9鑷繁娌″垵濮嬪寲瀹屾垚锛?
			HAL_Delay(100);					//绛夊緟100msops9鏁版嵁娓呴浂绋冲畾
		}
		if(Start_flag){
			servo_init();
			if(!debug && !is_task1_active()){			//鍚姩浠诲姟1
			start_task1();}
			break ;
		}
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Main_Tasktick();		//涓诲惊鐜换鍔★紙鍦ㄤ笅闈㈠畾涔夛級

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
//****************************************涓柇鍥炶皟鏈嶅姟鍑芥暟*****************************//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6) // 妫€鏌ユ槸鍚︽槸 TIM6 瑙﹀彂涓柇
    {
		led_count++;
		timcount ++;
		if(camtim_button){camtim += TIM6_TICK_MS;}//鎽勫儚澶磋秴鏃惰鏃讹紝绉诲姩绮惧害楂樼敤涓嶅埌
        UpdatePoseCoordination(CONTROL_DT_S);//鏇存柊浣嶅Э鍗忚皟锛?kHz
		if (is_position_control_enabled) {
			UpdatePositionControl(CONTROL_DT_S);//鏇存柊浣嶇疆鐜紝1kHz
		}
		UpdateAttitudeStabilization(CONTROL_DT_S);
		Process_usart1_packet();//瑙ｆ瀽maxcam鏁版嵁锛?kHz璋冪敤
		if (cam_stabilization.is_enabled) {
			
            UpdateCamStabilization(Camera_err.x, Camera_err.y, CONTROL_DT_S);//鏇存柊瑙嗚鐜紝1kHz
        }
		// 娣诲姞涓寸晫鍖轰繚鎶?
        __disable_irq();
        TargetVelocity_t current_target = target_velocity;
        __enable_irq();
        
        // 瀹炴椂鎵ц杩愬姩瀛﹁В绠?
        Mecanum_Kinematics(
            current_target.vx,
            current_target.vy,
            current_target.omega
        );
		if(Is_vel_mode){//鍙戦€佽繍鍔ㄦ寚浠わ紝200Hz
			if((led_count % MOTOR_CMD_DIVIDER) == 0U){
				if(Start_flag){
					ZDT_X42_V2_Velocity_Control(1, motor1.direction, 230, (float)abs_int(motor1.speed_rpm), 0);
					ZDT_X42_V2_Velocity_Control(2, motor2.direction, 230, (float)abs_int(motor2.speed_rpm), 0);
					ZDT_X42_V2_Velocity_Control(3, motor3.direction, 230, (float)abs_int(motor3.speed_rpm), 0);
					ZDT_X42_V2_Velocity_Control(4, motor4.direction, 230, (float)abs_int(motor4.speed_rpm), 0);
				}
			}	
		}
		if(led_count >= STATUS_TICK_DIVIDER)//绯荤粺姝ｅ父杩愯鎻愮ず鐏?
		{
			HAL_UART_Transmit_IT(&huart1,&cammode,sizeof (cammode));
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			led_count = 0;
		}
		// 鍏朵粬浠诲姟
		DMA_resend();//涓插彛1dma閲嶆柊寮€鍚殑璋冪敤
		if(IsLight)__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,200);else __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,0);//鐗㈢伅
    }
}
//*************************涓诲惊鐜腑鎵ц鍑芥暟************************************************//
void Main_Tasktick(void)
{
	LCD_DisplayNumber(0,20,timcount,6);//鏄剧ず绯荤粺璁℃椂

    run_task1();

    __WFI(); // 杩涘叆浣庡姛鑰楁ā寮忕瓑寰呬腑鏂?
}


//*************************************鍙栫粷瀵瑰€煎嚱鏁?*****************************//
// int鍨?鍙栫粷瀵瑰€煎嚱鏁?
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
