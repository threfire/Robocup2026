/**
 ******************************************************************************
 * @file    pid.c
 * @brief   PID 控制器实现
 ******************************************************************************
 */

#include "pid.h"
#include <math.h>

/** 将 PID 输出限制在对称范围内。 */
static float pid_limit(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

/** 初始化标准 PID 参数和历史状态。 */
void pid_init(PID_Controller *pid, float kp, float ki, float kd, float max_output)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->max_output = max_output;
}

/** 初始化视觉 PID 的二次比例项。 */
void pid_init_cam(PID_Controller *pid, float kp, float kp2, float ki, float kd,
        float max_output)
{
    pid_init(pid, kp, ki, kd, max_output);
    pid->Kp2 = kp2;
}

/** 计算标准 PID，并在输出饱和时撤销本周期积分。 */
float pid_calc(PID_Controller *pid, float error, float dt)
{
    float output;

    if (pid == 0 || dt <= 0.0f) {
        return 0.0f;
    }
    pid->integral += error * dt;
    output = pid->Kp * error + pid->Ki * pid->integral +
            pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    if (fabsf(output) > pid->max_output) {
        pid->integral -= error * dt;
    }
    return pid_limit(output, pid->max_output);
}

/** 计算带二次比例项的视觉 PID。 */
float pid_calc_cam(PID_Controller *pid, float error, float dt)
{
    float output;

    if (pid == 0 || dt <= 0.0f) {
        return 0.0f;
    }
    pid->integral += error * dt;
    output = pid->Kp * error + pid->Kp2 * error * fabsf(error) +
            pid->Ki * pid->integral + pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;
    if (fabsf(output) > pid->max_output) {
        pid->integral -= error * dt;
    }
    return pid_limit(output, pid->max_output);
}

/** 角度误差已在上层归一化，复用标准 PID 计算。 */
float pid_calc_angle(PID_Controller *pid, float error, float dt)
{
    return pid_calc(pid, error, dt);
}
