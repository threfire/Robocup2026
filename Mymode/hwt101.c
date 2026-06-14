#include "hwt101.h"
#include "usart.h"


#define ACC_UPDATE     0x01  // 加速度数据更新标志
#define GYRO_UPDATE    0x02  // 陀螺仪数据更新标志
#define ANGLE_UPDATE   0x04  // 角度数据更新标志
#define MAG_UPDATE     0x08  // 磁力计数据更新标志
#define READ_UPDATE    0x80  // 读取更新标志

/**
 * @brief 解析结果变量，如需跨文件调用，需要外部声明
 */
float pos_x=0;
float pos_y=0;
float zangle=0;
float xangle=0;
float yangle=0;
float w_z=0;


static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;  // 数据更新标志和命令变量
float absangle = 0;
// 存储传感器数据的数组
//static float fAcc[3];    // 加速度数据(x,y,z)
float fGyro[3];   // 角SU度数据
float fAngle[3];  // 角度数据(roll,pitch,yaw)

static uint8_t gIMU_StartFlag = 0;  // IMU开始上报数据标志位


uint8_t IsIMU_Start(void)	// 获取IMU是否已开始工作
{
	if(fAngle[2] == 0)
		{
		return gIMU_StartFlag;}
	else 
		{
		return !gIMU_StartFlag;}
}

/**
  * @brief    解析维特101角度
  * @param    
  * @param    
  * @param    
  * @param    
  * @param    
  * @retval   
  */
void ParseHWT101Yaw(void)
{
  // 检查数据头 (0x55, 0x53)
  if(HWT101rxbuff[11] != 0x55 || HWT101rxbuff[12] != 0x53) {
        return;  // 不满足包头直接 return
    }
	  

  // 提取原始数据字节
	
  uint8_t WzL = HWT101rxbuff[6];
  uint8_t WzH = HWT101rxbuff[7];
	
  uint8_t YawL = HWT101rxbuff[17];
  uint8_t YawH = HWT101rxbuff[18];
  uint8_t VL   = HWT101rxbuff[19];
  uint8_t VH   = HWT101rxbuff[20];
  uint8_t SUM  = HWT101rxbuff[21];

  // 计算校验和
  uint8_t calcSum = 0x55 + 0x53 + YawH + YawL + VL + VH;
  
  // 校验和验证
  if(calcSum != SUM)
    return ;  // 校验失败直接返回

  // 组合16位有符号整数（高位在前）
  int16_t yaw_raw = (int16_t)((YawH << 8) | YawL);
  int16_t Wz = (int16_t)((WzH << 8) | WzL);
  //Z=((WzH<<8)|WzL)/32768*2000°/s
  fGyro[2] = (float)Wz/32768.0f*2000;
  // 转换为角度值（-180° ~ +180°）
  fAngle[2] = (float)yaw_raw / 32768.0f * 180.0f;
  
}
/**
  * @brief    获取Yaw角度
  * @param    
  * @param    
  * @param    
  * @param    
  * @param    
  * @retval   
  */
float GetYaw(void)	// 获取Yaw角度(偏航角)
{
	return zangle;
}

void HWT_Init(void)
{
	fAngle[2] = 0;
	fGyro[2] = 0;
	uint8_t unlock_register[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
	uint8_t reset_z_axis[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
	uint8_t save_settings[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
	uint8_t restart_device[] = {0xFF, 0xAA, 0x00, 0xFF, 0x00};
	uint8_t count = 0;

	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)unlock_register, sizeof(unlock_register));
	HAL_Delay(10);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)reset_z_axis, sizeof(reset_z_axis));
	HAL_Delay(10);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)save_settings, sizeof(save_settings));
	HAL_Delay(10);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)restart_device, sizeof(restart_device));
	HAL_Delay(10);
	HAL_Delay (1000);
	if(count < 100 && IsIMU_Start())
	{
		HWT_Init();
	}
	
	absangle = GetYaw ();
}


/**
 * @brief 数据解析函数  如更换MCU平台或更换软件库，只需将串口接收到的值传入该函数即可解析
 * @param  rec 串口接收到的字节数据
 */
void Data_Analyse(uint8_t rec)
{
	static uint8_t ch;
	static union
	{
		uint8_t date[24];
		float ActVal[6];
	}posture;
	static uint8_t count=0;
	static uint8_t i=0;

	ch=rec;
	switch(count)
	{
		case 0:
			if(ch==0x0d)
				count++;
			else
				count=0;
			break;
		case 1:
			if(ch==0x0a)
			{
				i=0;
				count++;
			}
			else if(ch==0x0d);
			else
				count=0;
			break;
		case 2:
			posture.date[i]=ch;
			i++;
			if(i>=24)
			{
				i=0;
				count++;
			}
			break;
		case 3:
			if(ch==0x0a)
				count++;
			else
				count=0;
			break;
		case 4:
			if(ch==0x0d)
			{
				zangle=posture.ActVal[0];
				xangle=posture.ActVal[1];
				yangle=posture.ActVal[2];
				pos_x=posture.ActVal[3];
				pos_y=posture.ActVal[4];
				w_z=posture.ActVal[5];
			}
			count=0;
			break;
		default:
			count=0;
		break;
	}
}

void Update_zero(void){
    int i = 0;
    char Update_zero[4] = "ACT0";
    for(i = 0; i < 4; i++){
        HAL_UART_Transmit(&huart4, (uint8_t*)&Update_zero[i], 1, HAL_MAX_DELAY);
    }
}
