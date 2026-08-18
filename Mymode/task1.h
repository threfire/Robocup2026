/**
 ******************************************************************************
 * @file    task1.h
 * @brief   任务一状态机接口
 ******************************************************************************
 */

#ifndef TASK1_H
#define TASK1_H

#include <stdint.h>

struct task1_arc_control_s;
typedef struct task1_arc_control_s task1_arc_control_t;
extern task1_arc_control_t task1_arc_control;

/** 初始化任务一。 */
void start_task1(void);
/** 推进任务一步。 */
void run_task1(void);
/** 在 1 ms 实时任务中更新圆弧径向 PID。 */
void task1_arc_control_update(float dt);
/** 动态设置圆弧闭环基础角速度，单位 rad/s。 */
void task1_arc_set_omega(float base_omega_rad_s);
/** 获取任务一激活状态。 */
uint8_t is_task1_active(void);

#endif /* TASK1_H */
