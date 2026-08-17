/**
 ******************************************************************************
 * @file    task1.h
 * @brief   任务一状态机接口
 ******************************************************************************
 */

#ifndef TASK1_H
#define TASK1_H

#include <stdint.h>

/** 圆弧径向 PID 比例系数，输入单位 m。 */
#define TASK1_ARC_PID_KP 20.0f
/** 圆弧径向 PID 积分系数。 */
#define TASK1_ARC_PID_KI 0.1f
/** 圆弧径向 PID 微分系数。 */
#define TASK1_ARC_PID_KD 0.5f
/** 圆弧 PID 积分限幅，单位 m·s。 */
#define TASK1_ARC_PID_INTEGRAL_LIMIT 0.50f
/** 圆弧 PID 角速度输出限幅，单位 rad/s。 */
#define TASK1_ARC_PID_MAX_OMEGA_RAD_S 3.50f
/** step 2 圆弧基础角速度，单位 rad/s。 */
#define TASK1_MATERIAL_ARC_BASE_OMEGA_RAD_S (-2.0f)
/** step 28 圆弧基础角速度，单位 rad/s。 */
#define TASK1_TROPHY_ARC_BASE_OMEGA_RAD_S 2.0f
/** 槽口中心相对车体中心的车体系 X 坐标，单位 mm。 */
#define TASK1_ARC_SLOT_X_MM (-308.58f)
/** 槽口中心相对车体中心的车体系 Y 坐标，单位 mm。 */
#define TASK1_ARC_SLOT_Y_MM 0.0f

struct task1_arc_control_s;
typedef struct task1_arc_control_s task1_arc_control_t;
extern task1_arc_control_t task1_arc_control;

/** 初始化任务一。 */
void start_task1(void);
/** 推进任务一步。 */
void run_task1(void);
/** 在 1 ms 实时任务中更新圆弧径向 PID。 */
void task1_arc_control_update(float dt);
/** 获取任务一激活状态。 */
uint8_t is_task1_active(void);

#endif /* TASK1_H */
