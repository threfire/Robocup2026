/**
 ******************************************************************************
 * @file    feedforward.c
 * @brief   平移加速度与偏航动力学前馈实现
 ******************************************************************************
 */

#include "feedforward.h"
#include "control.h"
#include "odometer.h"

static float last_vel_x_mps;
static float last_vel_y_mps;
static float accel_x_mps2;
static float accel_y_mps2;
static float filtered_yaw_accel_rad_s2;
static uint8_t accel_valid;

/** 将前馈输出限制在指定范围。 */
static float feedforward_limit(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/** 清除平移和偏航前馈历史状态。 */
void feedforward_reset(void)
{
    accel_x_mps2 = 0.0f;
    accel_y_mps2 = 0.0f;
    accel_valid = 0U;
    filtered_yaw_accel_rad_s2 = 0.0f;
}

/** 对里程计速度微分并低通滤波，获得平移加速度。 */
void feedforward_accel_update(float dt)
{
    float vel_x_mps = odometer_ctx.motion.vel_x_mps;
    float vel_y_mps = odometer_ctx.motion.vel_y_mps;
    float measured_accel_x_mps2;
    float measured_accel_y_mps2;

    if (!accel_valid) {
        last_vel_x_mps = vel_x_mps;
        last_vel_y_mps = vel_y_mps;
        accel_valid = 1U;
        return;
    }
    measured_accel_x_mps2 = (vel_x_mps - last_vel_x_mps) / dt;
    measured_accel_y_mps2 = (vel_y_mps - last_vel_y_mps) / dt;
    accel_x_mps2 += POSITION_ACCEL_FEEDBACK_FILTER_ALPHA *
            (measured_accel_x_mps2 - accel_x_mps2);
    accel_y_mps2 += POSITION_ACCEL_FEEDBACK_FILTER_ALPHA *
            (measured_accel_y_mps2 - accel_y_mps2);
    last_vel_x_mps = vel_x_mps;
    last_vel_y_mps = vel_y_mps;
}

/** 获取世界坐标系 X 轴加速度。 */
float feedforward_accel_x(void)
{
    return accel_x_mps2;
}

/** 获取世界坐标系 Y 轴加速度。 */
float feedforward_accel_y(void)
{
    return accel_y_mps2;
}

/** 根据偏航角加速度误差计算角速度前馈。 */
float feedforward_yaw(float desired_accel_rad_s2, float dt, float wheel_dist_x_m,
        float wheel_dist_y_m, float k_omega_m)
{
    float actual_accel_rad_s2;
    float inertia_kg_m2;
    float torque_nm;
    float torque_to_omega;

    (void)dt;
    desired_accel_rad_s2 = feedforward_limit(desired_accel_rad_s2,
            -ATTITUDE_FF_MAX_ACCEL_RAD_S2, ATTITUDE_FF_MAX_ACCEL_RAD_S2);
    filtered_yaw_accel_rad_s2 += ATTITUDE_FF_ACCEL_FILTER_ALPHA *
            (desired_accel_rad_s2 - filtered_yaw_accel_rad_s2);
    desired_accel_rad_s2 = filtered_yaw_accel_rad_s2;
    actual_accel_rad_s2 = odometer_ctx.motion.yaw_accel_dps2 * 0.017453292519943295f;
    inertia_kg_m2 = CHASSIS_MASS_KG * (wheel_dist_x_m * wheel_dist_x_m +
            wheel_dist_y_m * wheel_dist_y_m) / 3.0f + CHASSIS_MASS_KG *
            (CHASSIS_CENTER_OF_MASS_X_M * CHASSIS_CENTER_OF_MASS_X_M +
             CHASSIS_CENTER_OF_MASS_Y_M * CHASSIS_CENTER_OF_MASS_Y_M);
    torque_nm = inertia_kg_m2 * (ATTITUDE_ACCEL_FEEDFORWARD_GAIN *
            desired_accel_rad_s2 +
            ATTITUDE_ACCEL_FEEDBACK_GAIN *
            (desired_accel_rad_s2 - actual_accel_rad_s2));
    torque_to_omega = POSITION_FORCE_TO_SPEED_FF_MPS_PER_N /
            (k_omega_m * k_omega_m);
    return feedforward_limit(torque_nm * torque_to_omega,
            -ATTITUDE_FF_MAX_OMEGA_RAD_S, ATTITUDE_FF_MAX_OMEGA_RAD_S);
}
