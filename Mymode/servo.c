#include "stm32f4xx.h"                  // Device header
#include "servo.h"


servo_t duoji_doing[DJ_NUM];
uint8_t startServo = 0;


// 脉宽范围定义
#define SERVO_MIN_PULSE  500   // 最小脉冲宽度 (μs)
#define SERVO_MAX_PULSE  2500  // 最大脉冲宽度 (μs)
#define SERVO_MID_PULSE  1500  // 中位脉冲宽度 (μs)

// 设置脉冲宽度 (直接设置占空比)
void Servo_SetPulseWidth(uint16_t pulse_us)
{
    // 安全限制
    if(pulse_us < SERVO_MIN_PULSE) pulse_us = SERVO_MIN_PULSE;
    if(pulse_us > SERVO_MAX_PULSE) pulse_us = SERVO_MAX_PULSE;
    
//	HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
    // 直接设置比较值 (1计数=1μs)
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, pulse_us);
//	HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);
}

// 设置角度 (0-270度)
void Servo_SetAngle(uint16_t angle)
{
    // 角度限制
    if(angle > 270) angle = 270;
    
    // 角度转脉冲宽度: 500 + (angle/270)*2000
    uint32_t pulse_width = SERVO_MIN_PULSE + 
                          (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 270;
    
    Servo_SetPulseWidth(pulse_width);
}
/* 舵机初始化 */
void servo_init(void)
{
	
	HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);		//0	PF7
  HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);		//1 PF8
	HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);		//2 PF9
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);		//4	PE11
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//打开led灯pwm通道
	
//    for (i = 0; i < DJ_NUM; i++)
//    {
//        duoji_doing[i].aim = 500;
//        duoji_doing[i].cur = 500;
//        duoji_doing[i].inc = 0;
//        duoji_doing[i].time = 5000;

//    }
////	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, duoji_doing[0].aim);
////	__HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, duoji_doing[1].aim);
////	__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, duoji_doing[2].aim);
//	
//	duoji_doing_set(0,800,5000);		//倒转臂
//	duoji_doing_set(1,575,5000);
//	duoji_doing_set(2,550,5000);
}

/***********************************************
    功能介绍：	设置舵机控制参数函数
    函数参数：	index 舵机编号 aim 执行目标 time 执行时间(如果aim 执行目标==0，视为舵机停止)
    返回值：		无
 ***********************************************/
void duoji_doing_set(uint8_t index, int aim, int time)
{
    /* 限制输入值大小 */
    if (index >= DJ_NUM)
        return;

    if (aim == 0)
    {
        duoji_doing[index].inc = 0;
        duoji_doing[index].aim = duoji_doing[index].cur;
        return;
    }

    if (aim > 2490)
        aim = 2490;
    else if (aim < 200)
        aim = 200;

    if (time > 10000)
        time = 10000;

    if (duoji_doing[index].cur == aim)
    {
        aim = aim + 0.0077;
    }

    if (time < 20) /* 执行时间太短，舵机直接以最快速度运动 */
    {
        duoji_doing[index].aim = aim;
        duoji_doing[index].cur = aim;
        duoji_doing[index].inc = 0;
    }
    else
    {
        duoji_doing[index].aim = aim;
        duoji_doing[index].time = time;
        duoji_doing[index].inc = (duoji_doing[index].aim - duoji_doing[index].cur) / (duoji_doing[index].time / 20.000);
    }
}

/* 设置舵机每次增加的偏移量 */
void servo_inc_offset(uint8_t index)
{
    int aim_temp;

    if (duoji_doing[index].inc != 0)
    {

        aim_temp = duoji_doing[index].aim;

        if (aim_temp > 2490)
        {
            aim_temp = 2490;
        }
        else if (aim_temp < 500)
        {
            aim_temp = 500;
        }

        if (abs_float(aim_temp - duoji_doing[index].cur) <= abs_float(duoji_doing[index].inc + duoji_doing[index].inc))
        {
            duoji_doing[index].cur = aim_temp;
            duoji_doing[index].inc = 0;
        }
        else
        {
            duoji_doing[index].cur += duoji_doing[index].inc;
        }
    }
}

void servo_tick(void)
{
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, duoji_doing[0].aim);
	__HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, duoji_doing[1].aim);
	__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, duoji_doing[2].aim);
}
void jiazhuadongzuo(void)
{
	duoji_doing_set(0,490,5000);		//夹
	//非阻塞延时
	duoji_doing_set(0,700,5000);		//松
}
void wuliaopanxuanzhuan(void)
{
	duoji_doing_set(2,1000,5000);
}

void Servo_Action_Group(uint8_t num){
	if(num == 1){
		__HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, 1100);
		__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 2260);
//		__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, 575);
	}
	if(num == 2){
		__HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, 1100);
		__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1820);
//		__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, 770);
	}
	if(num == 3){//举起来（冠军台）
		__HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, 1460);
		__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1680);
	}
	if(num == 4){//放平（冠军台）
		uint16_t pwm13last = 1460;
		uint16_t pwm14last = 1680;

		for(uint8_t i = 1;i<=100;i++){
			__HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, pwm13last-i*(pwm13last-1360)/100);
			__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, pwm14last-i*(pwm14last-1500)/100);
			HAL_Delay(2);
		}
	}
	if(num == 5){//举起来（亚军台）
		__HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, 1390);
		__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1760);
	}
	if(num == 6){//放平（亚军台）
		uint16_t pwm13last = 1390;
		uint16_t pwm14last = 1760;

		for(uint8_t i = 1;i<=100;i++){
			__HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, pwm13last-i*(pwm13last-1260)/100);
			__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, pwm14last-i*(pwm14last-1700)/100);
			HAL_Delay(2);
		}
	}
	if(num == 7){//放平（季军台）
		uint16_t pwm13last = 1460;
		uint16_t pwm14last = 1580;

		for(uint8_t i = 1;i<=100;i++){
			__HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, pwm13last-i*(pwm13last-1100)/100);
			__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, pwm14last-i*(pwm14last-1820)/100);
			HAL_Delay(1);
		}
	}
	else
		return;
	}
