/**
 ******************************************************************************
 * @file    motion_plan.h
 * @brief   平移速度规划器接口
 ******************************************************************************
 */

#ifndef MOTION_PLAN_H
#define MOTION_PLAN_H

/** 初始化巡航速度并清空规划状态。 */
void motion_plan_init(float cruise_speed_mps);
/** 清空速度与加速度状态。 */
void motion_plan_reset(void);
/** 设置巡航速度上限。 */
void motion_plan_set_cruise(float cruise_speed_mps);
/** 按剩余距离更新速度、加速度和加加速度约束。 */
float motion_plan_update(float distance_error_m, float dt);
/** 获取当前规划速度。 */
float motion_plan_speed(void);
/** 获取当前规划加速度。 */
float motion_plan_accel(void);
/** 获取当前巡航速度。 */
float motion_plan_cruise(void);

#endif /* MOTION_PLAN_H */
