/**
 ******************************************************************************
 * @file    fuzzy.c
 * @brief   模糊自适应 PID 实现
 ******************************************************************************
 */

#include "fuzzy.h"
#include <math.h>
#include "control.h"

/** 将误差映射为 0 到 1 的隶属度。 */
static float fuzzy_level(float value, float full_scale)
{
    float level = value / full_scale;

    if (level < 0.0f) {
        return 0.0f;
    }
    if (level > 1.0f) {
        return 1.0f;
    }
    return level;
}

/** 按当前误差缩放比例项和微分项，计算后恢复基准参数。 */
float fuzzy_pid_calc(PID_Controller *pid, float error, float dt,
        float error_full_scale, float rate_full_scale, float kp_min_scale,
        float kp_max_scale, float kd_min_scale, float kd_max_scale,
        uint8_t angle_mode)
{
#if FUZZY_PID_ENABLE
    float base_kp = pid->Kp;
    float base_kd = pid->Kd;
    float error_level = fuzzy_level(fabsf(error), error_full_scale);
    float raw_rate = (error - pid->prev_error) / dt;
    float rate_alpha = angle_mode ? ATTITUDE_FUZZY_RATE_FILTER_ALPHA :
            POSITION_FUZZY_RATE_FILTER_ALPHA;
    pid->fuzzy_rate += rate_alpha * (raw_rate - pid->fuzzy_rate);
    float rate_level = fuzzy_level(fabsf(pid->fuzzy_rate), rate_full_scale);
    float kp_scale = kp_min_scale + (kp_max_scale - kp_min_scale) * error_level;
    float kd_level = fmaxf(error_level, rate_level);
    float kd_scale = kd_min_scale + (kd_max_scale - kd_min_scale) * kd_level;
    float output;

    pid->Kp = base_kp * kp_scale;
    pid->Kd = base_kd * kd_scale;
    pid->prev_error = error - pid->fuzzy_rate * dt;
    output = angle_mode ? pid_calc_angle(pid, error, dt) : pid_calc(pid, error, dt);
    pid->Kp = base_kp;
    pid->Kd = base_kd;
    return output;
#else
    return angle_mode ? pid_calc_angle(pid, error, dt) : pid_calc(pid, error, dt);
#endif
}
