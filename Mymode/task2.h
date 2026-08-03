/**
 ******************************************************************************
 * @file    task2.h
 * @brief   任务二状态机接口
 ******************************************************************************
 */

#ifndef TASK2_H
#define TASK2_H

#include <stdint.h>

/** 初始化任务二。 */
void start_task2(void);
/** 推进任务二。 */
void run_task2(void);
/** 获取任务二激活状态。 */
uint8_t is_task2_active(void);

#endif /* TASK2_H */
