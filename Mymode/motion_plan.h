/**
 ******************************************************************************
 * @file    motion_plan.h
 * @brief   平移加加速度轨迹规划器接口。
 ******************************************************************************
 */

#ifndef MOTION_PLAN_H
#define MOTION_PLAN_H

#include <stdint.h>

/** 初始化巡航速度并清空轨迹状态。 */
void motion_plan_init(float cruise_speed_mps);
/** 清空轨迹位置、速度与加速度状态。 */
void motion_plan_reset(void);
/** 设置巡航速度上限。 */
void motion_plan_set_cruise(float cruise_speed_mps);
/** 启动长度固定的平移轨迹，长度单位为 m。 */
void motion_plan_start(float distance_m);
/** 按加加速度约束更新轨迹位置、速度和加速度。 */
void motion_plan_update(float dt);
/** 获取当前轨迹进度，单位为 m。 */
float motion_plan_progress(void);
/** 获取当前轨迹剩余长度，单位为 m。 */
float motion_plan_remaining(void);
/** 获取当前轨迹速度。 */
float motion_plan_speed(void);
/** 获取当前轨迹加速度。 */
float motion_plan_accel(void);
/** 获取当前巡航速度。 */
float motion_plan_cruise(void);
/** 获取轨迹是否已到达终点。 */
uint8_t motion_plan_finished(void);

#endif /* MOTION_PLAN_H */
