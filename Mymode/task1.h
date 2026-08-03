/**
 ******************************************************************************
 * @file    task1.h
 * @brief   任务一状态机接口
 ******************************************************************************
 */

#ifndef TASK1_H
#define TASK1_H

#include <stdint.h>

/** 初始化任务一。 */
void start_task1(void);
/** 推进任务一步。 */
void run_task1(void);
/** 获取任务一激活状态。 */
uint8_t is_task1_active(void);

#endif /* TASK1_H */
