#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

/**
 * @brief 舵机平滑运动状态。
 */
typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t start_pwm;
    uint16_t target_pwm;
    uint32_t start_time_ms;
    uint32_t duration_ms;
    uint8_t active;
} servo_smooth_move_t;

void servo_init(void);
void Servo_Action_Group(uint8_t num);

/**
 * @brief 启动舵机平滑运动。
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
 * @brief 更新舵机平滑运动轨迹。
 *
 * @param move 舵机轨迹状态。
 */
void servo_smooth_move_update(servo_smooth_move_t *move);

#endif
