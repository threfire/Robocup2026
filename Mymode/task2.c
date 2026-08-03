/**
 ******************************************************************************
 * @file    task2.c
 * @brief   任务二状态机
 ******************************************************************************
 */

#include "task2.h"
#include "control.h"
#include "vision.h"
#include "motion.h"

/** 初始化任务二的恒速调试状态。 */
void start_task2(void)
{
    task2_state.active = 1U;
    task2_state.current_step = 1U;
    task2_state.completed = 0U;
    task2_state.step_start = HAL_GetTick();
    tasknum = 2U;
    motion_reset();
    PoseCoordination_Enable(0U);
    PositionControl_Enable(0U);
    EnableAttitudeStabilization(0U);
    vision_enable(0U);
    SetTargetVelocity(TASK2_TEST_SPEED_MPS, 0.0f, 0.0f);
}

/** 执行任务二的恒定横移命令。 */
void run_task2(void)
{
    if (!task2_state.active)
    {
        return;
    }

    SetTargetVelocity(0.0f, 0.5f, 0.0f);
}

/** 返回任务二激活状态。 */
uint8_t is_task2_active(void)
{
    return task2_state.active;
}
