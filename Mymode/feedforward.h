/**
 ******************************************************************************
 * @file    feedforward.h
 * @brief   平移加速度与质心补偿前馈接口
 ******************************************************************************
 */

#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

/**
 * @brief 计算局部速度前馈。
 *
 * 规划加速度生成平移前馈，偏航运动根据质心偏置生成附加补偿。
 */
void feedforward_linear(float desired_accel_x_mps2, float desired_accel_y_mps2,
        float yaw_rate_rad_s, float yaw_accel_rad_s2, float accel_gain_s,
        float com_gain_s, float *accel_vx_mps, float *accel_vy_mps,
        float *com_vx_mps, float *com_vy_mps);

#endif /* FEEDFORWARD_H */
