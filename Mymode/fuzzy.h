/**
 ******************************************************************************
 * @file    fuzzy.h
 * @brief   模糊自适应 PID 接口
 ******************************************************************************
 */

#ifndef FUZZY_H
#define FUZZY_H

#include <stdint.h>
#include "pid.h"

/** 按误差和误差变化率缩放 PID 比例项与微分项。 */
float fuzzy_pid_calc(PID_Controller *pid, float error, float dt,
        float error_full_scale, float rate_full_scale, float kp_min_scale,
        float kp_max_scale, float kd_min_scale, float kd_max_scale,
        uint8_t angle_mode);

#endif /* FUZZY_H */
