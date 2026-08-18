/**
 ******************************************************************************
 * @file    motion_plan.c
 * @brief   平移加加速度轨迹规划器实现。
 ******************************************************************************
 */

#include "motion_plan.h"

#include <math.h>

#include "control.h"

static float plan_speed_mps;
static float plan_accel_mps2;
static float plan_cruise_speed_mps;
static float plan_distance_m;
static float plan_progress_m;
static uint8_t plan_finished;

/** 将数值限制在指定范围。 */
static float motion_plan_limit(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/** 初始化轨迹规划器。 */
void motion_plan_init(float cruise_speed_mps)
{
    plan_cruise_speed_mps = cruise_speed_mps;
    motion_plan_reset();
}

/** 清空轨迹状态。 */
void motion_plan_reset(void)
{
    plan_speed_mps = 0.0f;
    plan_accel_mps2 = 0.0f;
    plan_distance_m = 0.0f;
    plan_progress_m = 0.0f;
    plan_finished = 1U;
}

/** 更新巡航速度。 */
void motion_plan_set_cruise(float cruise_speed_mps)
{
    plan_cruise_speed_mps = cruise_speed_mps;
}

/** 启动长度固定的加加速度轨迹。 */
void motion_plan_start(float distance_m)
{
    motion_plan_reset();
    plan_distance_m = distance_m;
    plan_finished = (distance_m <= POSITION_TRAJECTORY_NEAR_TARGET_DISTANCE_M);
}

/** 按轨迹剩余长度更新加加速度受限的参考位置。 */
void motion_plan_update(float dt)
{
    float remaining_distance_m;
    float cruise_speed;
    float brake_speed;
    float target_speed;
    float target_accel;
    float accel_delta;
    float next_speed_mps;
    float next_progress_m;

    if ((dt <= 0.0f) || (plan_finished != 0U))
    {
        return;
    }

    remaining_distance_m = plan_distance_m - plan_progress_m;
    cruise_speed = motion_plan_limit(plan_cruise_speed_mps, 0.0f,
            POSITION_MAX_LINEAR_SPEED_MPS);
    brake_speed = sqrtf(2.0f * POSITION_TRAJECTORY_MAX_DECEL_MPS2 *
            remaining_distance_m);
    target_speed = motion_plan_limit(cruise_speed, 0.0f, brake_speed);
    if (plan_speed_mps > target_speed)
    {
        target_accel = (target_speed - plan_speed_mps) / dt;
        target_accel = motion_plan_limit(target_accel,
                -POSITION_TRAJECTORY_MAX_DECEL_MPS2, 0.0f);
    }
    else if (plan_speed_mps < target_speed)
    {
        target_accel = POSITION_TRAJECTORY_MAX_ACCEL_MPS2;
    }
    else
    {
        target_accel = 0.0f;
    }

    accel_delta = target_accel - plan_accel_mps2;
    accel_delta = motion_plan_limit(accel_delta,
            -POSITION_TRAJECTORY_MAX_JERK_MPS3 * dt,
            POSITION_TRAJECTORY_MAX_JERK_MPS3 * dt);
    plan_accel_mps2 += accel_delta;
    plan_accel_mps2 = motion_plan_limit(plan_accel_mps2,
            -POSITION_TRAJECTORY_MAX_DECEL_MPS2,
            POSITION_TRAJECTORY_MAX_ACCEL_MPS2);
    next_speed_mps = plan_speed_mps + plan_accel_mps2 * dt;
    next_speed_mps = motion_plan_limit(next_speed_mps, 0.0f, cruise_speed);
    next_progress_m = plan_progress_m +
            (plan_speed_mps + next_speed_mps) * 0.5f * dt;
    if (next_progress_m >= plan_distance_m)
    {
        plan_progress_m = plan_distance_m;
        plan_speed_mps = 0.0f;
        plan_accel_mps2 = 0.0f;
        plan_finished = 1U;
        return;
    }

    plan_progress_m = next_progress_m;
    plan_speed_mps = next_speed_mps;
}

/** 获取当前轨迹进度。 */
float motion_plan_progress(void)
{
    return plan_progress_m;
}

/** 获取当前轨迹剩余长度。 */
float motion_plan_remaining(void)
{
    return plan_distance_m - plan_progress_m;
}

/** 获取当前轨迹速度。 */
float motion_plan_speed(void)
{
    return plan_speed_mps;
}

/** 获取当前轨迹加速度。 */
float motion_plan_accel(void)
{
    return plan_accel_mps2;
}

/** 获取当前巡航速度。 */
float motion_plan_cruise(void)
{
    return plan_cruise_speed_mps;
}

/** 获取轨迹完成状态。 */
uint8_t motion_plan_finished(void)
{
    return plan_finished;
}
