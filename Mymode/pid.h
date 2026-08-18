/**
 ******************************************************************************
 * @file    pid.h
 * @brief   PID 控制器接口
 ******************************************************************************
 */

#ifndef PID_H
#define PID_H

/** PID 控制器运行状态。 */
typedef struct {
    float Kp;
    float Kp2;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float fuzzy_rate;
    float max_output;
} PID_Controller;

/** 初始化标准 PID 参数。 */
void pid_init(PID_Controller *pid, float kp, float ki, float kd, float max_output);
/** 初始化带二次比例项的视觉 PID 参数。 */
void pid_init_cam(PID_Controller *pid, float kp, float kp2, float ki, float kd,
        float max_output);
/** 计算标准 PID 输出。 */
float pid_calc(PID_Controller *pid, float error, float dt);
/** 计算视觉 PID 输出。 */
float pid_calc_cam(PID_Controller *pid, float error, float dt);
/** 计算角度 PID 输出。 */
float pid_calc_angle(PID_Controller *pid, float error, float dt);

#endif /* PID_H */
