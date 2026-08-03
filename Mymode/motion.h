/**
 ******************************************************************************
 * @file    motion.h
 * @brief   底盘运动控制接口
 ******************************************************************************
 */

#ifndef MOTION_H
#define MOTION_H

/** 设置底盘局部坐标系速度目标。 */
void motion_set_velocity(float vx, float vy, float omega);
/** 设置底盘局部坐标系平移速度目标。 */
void motion_set_linear_velocity(float vx, float vy);
/** 清空底盘速度、规划器和控制器历史状态。 */
void motion_reset(void);

#endif /* MOTION_H */
