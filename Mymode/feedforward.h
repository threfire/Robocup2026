/**
 ******************************************************************************
 * @file    feedforward.h
 * @brief   平移与偏航前馈接口
 ******************************************************************************
 */

#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

/** 清除加速度和偏航前馈历史量。 */
void feedforward_reset(void);
/** 用里程计速度更新平移加速度反馈。 */
void feedforward_accel_update(float dt);
/** 获取世界坐标系 X 轴加速度。 */
float feedforward_accel_x(void);
/** 获取世界坐标系 Y 轴加速度。 */
float feedforward_accel_y(void);
/** 根据规划角加速度和底盘惯量生成角速度前馈。 */
float feedforward_yaw(float desired_accel_rad_s2, float dt, float wheel_dist_x_m,
        float wheel_dist_y_m, float k_omega_m);

#endif /* FEEDFORWARD_H */
