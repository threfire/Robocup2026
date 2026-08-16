/**
 ******************************************************************************
 * @file    motion.h
 * @brief   底盘运动控制接口
 ******************************************************************************
 */

#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

/**
 * @brief 连续路径上的世界系坐标点。
 */
typedef struct
{
    float x_mm; /* 世界系 X 坐标，单位 mm */
    float y_mm; /* 世界系 Y 坐标，单位 mm */
} MotionPathPoint_t;

/**
 * @brief 大圆收拢路径几何参数。
 */
typedef struct
{
    float center_x_mm; /* 大圆世界系圆心 X 坐标，单位 mm */
    float center_y_mm; /* 大圆世界系圆心 Y 坐标，单位 mm */
    float radius_mm; /* 大圆半径，单位 mm */
    float slot_x_mm; /* 槽口捕获中心在车体系的 X 坐标，单位 mm */
    float slot_y_mm; /* 槽口捕获中心在车体系的 Y 坐标，单位 mm */
    const MotionPathPoint_t *material_points; /* 物料世界系坐标数组 */
    uint8_t point_count; /* 物料坐标数量 */
    int8_t direction; /* 绕圆方向，1 为逆时针，-1 为顺时针 */
    float cruise_speed_mps; /* 路径巡航速度，单位 m/s */
} MotionCirclePath_t;

/** 设置底盘局部坐标系速度目标。 */
void motion_set_velocity(float vx, float vy, float omega);
/** 设置底盘局部坐标系平移速度目标。 */
void motion_set_linear_velocity(float vx, float vy);
/** 清空底盘速度、规划器和控制器历史状态。 */
void motion_reset(void);

/**
 * @brief 启动槽口经过物料坐标的大圆连续闭环路径。
 *
 * 函数根据槽口车体系坐标和圆切线姿态反解底盘中心路径。
 *
 * @param path 大圆收拢路径参数。
 * @return 1 表示路径已启动，0 表示参数无效。
 */
uint8_t motion_circle_path_start(const MotionCirclePath_t *path);

/**
 * @brief 更新连续路径参考量。
 *
 * @param dt 控制周期，单位 s。
 */
void motion_path_update(float dt);

/**
 * @brief 获取连续路径完成状态。
 *
 * @return 1 表示终点位置和姿态均已到达，0 表示路径仍在运行。
 */
uint8_t motion_path_finished(void);

/**
 * @brief 清除连续路径状态。
 */
void motion_path_stop(void);

#endif /* MOTION_H */
