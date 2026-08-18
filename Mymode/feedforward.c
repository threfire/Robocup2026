/**
 ******************************************************************************
 * @file    feedforward.c
 * @brief   平移加速度与质心补偿前馈实现
 ******************************************************************************
 */

#include "feedforward.h"

#include <stddef.h>

#include "control.h"

/**
 * @brief 计算局部速度前馈。
 *
 * @param desired_accel_x_mps2 规划局部 X 加速度，单位 m/s²。
 * @param desired_accel_y_mps2 规划局部 Y 加速度，单位 m/s²。
 * @param yaw_rate_rad_s 实际偏航角速度，单位 rad/s。
 * @param yaw_accel_rad_s2 实际偏航角加速度，单位 rad/s²。
 * @param accel_gain_s 规划加速度前馈系数，单位 s。
 * @param com_gain_s 质心偏置补偿系数，单位 s。
 * @param accel_vx_mps 平移 X 前馈输出，单位 m/s。
 * @param accel_vy_mps 平移 Y 前馈输出，单位 m/s。
 * @param com_vx_mps 质心 X 补偿输出，单位 m/s。
 * @param com_vy_mps 质心 Y 补偿输出，单位 m/s。
 */
void feedforward_linear(float desired_accel_x_mps2, float desired_accel_y_mps2,
        float yaw_rate_rad_s, float yaw_accel_rad_s2, float accel_gain_s,
        float com_gain_s, float *accel_vx_mps, float *accel_vy_mps,
        float *com_vx_mps, float *com_vy_mps)
{
    if ((accel_vx_mps == NULL) || (accel_vy_mps == NULL) ||
            (com_vx_mps == NULL) || (com_vy_mps == NULL)) {
        return;
    }

    *accel_vx_mps = desired_accel_x_mps2 * accel_gain_s;
    *accel_vy_mps = desired_accel_y_mps2 * accel_gain_s;
    *com_vx_mps = (-yaw_accel_rad_s2 * CHASSIS_CENTER_OF_MASS_Y_M -
            yaw_rate_rad_s * yaw_rate_rad_s * CHASSIS_CENTER_OF_MASS_X_M) *
            com_gain_s;
    *com_vy_mps = (yaw_accel_rad_s2 * CHASSIS_CENTER_OF_MASS_X_M -
            yaw_rate_rad_s * yaw_rate_rad_s * CHASSIS_CENTER_OF_MASS_Y_M) *
            com_gain_s;
}
