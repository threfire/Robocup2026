#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

#define DJ_NUM 3 /* 舵机数量 */
extern uint8_t startServo;
typedef struct
{
    // uint8_t valid; // 有效 TODO
    uint16_t aim;  // 执行目标
    uint16_t time; // 执行时间
    float cur;     // 当前值
    float inc;     // 增量
} servo_t;

/**
 * @brief 舵机时间轨迹状态。
 */
typedef struct
{
    TIM_HandleTypeDef *htim;  /* PWM 定时器句柄 */
    uint32_t channel;         /* PWM 通道 */
    uint16_t start_pwm;       /* 起始 PWM */
    uint16_t target_pwm;      /* 终点 PWM */
    uint32_t start_time_ms;   /* 轨迹开始时刻，单位 ms */
    uint32_t duration_ms;     /* 轨迹时长，单位 ms */
    uint8_t active;           /* 轨迹运行标志 */
} servo_smooth_move_t;

extern servo_t duoji_doing[DJ_NUM];

//相关函数声明*******//
void Servo_SetPulseWidth(uint16_t pulse_us);
void servo_init(void);
void duoji_doing_set(uint8_t index, int aim, int time); /* 设置舵机参数 */
void servo_inc_offset(uint8_t index);                  /* 设置舵机每次增加的偏移量 */
void servo_tick(void);
void jiazhuadongzuo(void);
void wuliaopanxuanzhuan(void);
void Servo_Action_Group(uint8_t num);

/**
 * @brief 启动舵机缓出轨迹。
 *
 * PWM 按时间从起始值平滑收敛至终点值，末段速度连续减小。
 *
 * @param move 舵机轨迹状态。
 * @param htim PWM 定时器句柄。
 * @param channel PWM 通道。
 * @param start_pwm 起始 PWM。
 * @param target_pwm 终点 PWM。
 * @param duration_ms 运动时长，单位 ms。
 */
void servo_smooth_move_start(servo_smooth_move_t *move,
        TIM_HandleTypeDef *htim, uint32_t channel, uint16_t start_pwm,
        uint16_t target_pwm, uint32_t duration_ms);

/**
 * @brief 更新舵机缓出轨迹。
 *
 * 在任务循环中周期调用，函数根据经过时间输出当前 PWM。
 *
 * @param move 舵机轨迹状态。
 */
void servo_smooth_move_update(servo_smooth_move_t *move);
#endif
