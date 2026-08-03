/**
 ******************************************************************************
 * @file    motion_plan.c
 * @brief   平移速度规划器实现
 ******************************************************************************
 */

#include "motion_plan.h"
#include <math.h>
#include "control.h"

static float plan_speed_mps;
static float plan_accel_mps2;
static float plan_cruise_speed_mps;

/** 将数值限制在指定范围。 */
static float motion_plan_limit(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/** 初始化速度规划器。 */
void motion_plan_init(float cruise_speed_mps)
{
    plan_cruise_speed_mps = cruise_speed_mps;
    motion_plan_reset();
}

/** 清空速度与加速度历史量。 */
void motion_plan_reset(void)
{
    plan_speed_mps = 0.0f;
    plan_accel_mps2 = 0.0f;
}

/** 更新巡航速度。 */
void motion_plan_set_cruise(float cruise_speed_mps)
{
    plan_cruise_speed_mps = cruise_speed_mps;
}

/** 依据剩余距离执行带加加速度限制的减速规划。 */
float motion_plan_update(float distance_error_m, float dt)
{
    float cruise_speed = motion_plan_limit(plan_cruise_speed_mps, 0.0f,
            POSITION_MAX_LINEAR_SPEED_MPS);
    float brake_speed = sqrtf(2.0f * POSITION_COMMAND_BRAKE_DECEL_MPS2 *
            distance_error_m);
    float target_speed = motion_plan_limit(cruise_speed, 0.0f, brake_speed);
    float target_accel;
    float accel_delta;

    if (plan_speed_mps > target_speed) {
        target_accel = -POSITION_PLANNER_MAX_DECEL_MPS2;
    } else if (plan_speed_mps < target_speed) {
        target_accel = POSITION_PLANNER_MAX_ACCEL_MPS2;
    } else {
        target_accel = 0.0f;
    }

    accel_delta = target_accel - plan_accel_mps2;
    accel_delta = motion_plan_limit(accel_delta,
            -POSITION_PLANNER_MAX_JERK_MPS3 * dt,
            POSITION_PLANNER_MAX_JERK_MPS3 * dt);
    plan_accel_mps2 += accel_delta;
    plan_accel_mps2 = motion_plan_limit(plan_accel_mps2,
            -POSITION_PLANNER_MAX_DECEL_MPS2, POSITION_PLANNER_MAX_ACCEL_MPS2);
    plan_speed_mps += plan_accel_mps2 * dt;
    plan_speed_mps = motion_plan_limit(plan_speed_mps, 0.0f, cruise_speed);

    if (plan_speed_mps > target_speed) {
        plan_speed_mps = target_speed;
        plan_accel_mps2 = 0.0f;
    }

    if (plan_speed_mps < POSITION_PLANNER_MIN_SPEED_MPS &&
            target_speed > POSITION_PLANNER_MIN_SPEED_MPS) {
        plan_speed_mps = POSITION_PLANNER_MIN_SPEED_MPS;
    }
    return plan_speed_mps;
}

/** 获取当前规划速度。 */
float motion_plan_speed(void)
{
    return plan_speed_mps;
}

/** 获取当前规划加速度。 */
float motion_plan_accel(void)
{
    return plan_accel_mps2;
}

/** 获取当前巡航速度。 */
float motion_plan_cruise(void)
{
    return plan_cruise_speed_mps;
}
