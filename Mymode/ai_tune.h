/**
 ******************************************************************************
 * @file    ai_tune.h
 * @brief   坐标闭环在线调参串口接口。
 ******************************************************************************
 */

#ifndef ROBOCUP_AI_TUNE_H
#define ROBOCUP_AI_TUNE_H

#include <stdint.h>

/** USART5 上位机在线调试模式开关：0 为比赛模式，1 为调试模式。 */
#define AI_TUNE_MODE_ENABLE 0U

typedef struct {
    uint32_t version;                 /* 上位机参数版本号 */
    float kp;                         /* X/Y 位置环比例系数 */
    float kd;                         /* X/Y 位置环微分系数 */
    float accel_feedback_gain;        /* 加速度误差反馈系数 */
    float force_to_speed_ff;          /* 力到速度前馈换算系数，单位 m/(s·N) */
    float static_friction_speed_mps;  /* 静摩擦补偿速度，单位 m/s */
} ai_tune_param_t;

typedef struct {
    uint32_t time_ms;                 /* 控制时间戳，单位 ms */
    float target_x_m;                 /* 目标世界坐标 X，单位 m */
    float target_y_m;                 /* 目标世界坐标 Y，单位 m */
    float position_x_m;               /* 实际世界坐标 X，单位 m */
    float position_y_m;               /* 实际世界坐标 Y，单位 m */
    float target_yaw_deg;             /* 目标偏航角，单位 deg */
    float yaw_deg;                    /* 实际偏航角，单位 deg */
    float error_x_local_m;            /* 车体系 X 误差，单位 m */
    float error_y_local_m;            /* 车体系 Y 误差，单位 m */
    float velocity_x_mps;             /* 里程计世界系 X 速度，单位 m/s */
    float velocity_y_mps;             /* 里程计世界系 Y 速度，单位 m/s */
    float plan_accel_mps2;            /* 规划加速度，单位 m/s² */
    float accel_x_mps2;               /* 实测车体系 X 加速度，单位 m/s² */
    float accel_y_mps2;               /* 实测车体系 Y 加速度，单位 m/s² */
    float feedforward_vx_mps;         /* X 速度前馈，单位 m/s */
    float feedforward_vy_mps;         /* Y 速度前馈，单位 m/s */
    float feedback_vx_mps;            /* X PID 速度，单位 m/s */
    float feedback_vy_mps;            /* Y PID 速度，单位 m/s */
    float command_vx_mps;             /* 最终 X 速度指令，单位 m/s */
    float command_vy_mps;             /* 最终 Y 速度指令，单位 m/s */
    int16_t motor_rpm[4];             /* 四轮速度指令，单位 RPM */
    uint8_t position_reached;         /* 到点标志 */
} ai_tune_telemetry_t;

/** @brief 初始化 UART5 调参收发链路。 */
void ai_tune_init(void);
/** @brief 处理 UART5 空闲中断中的 DMA 接收数据。 */
void ai_tune_uart_idle_irq(void);
/** @brief 轮询 UART5 IDLE 标志，作为中断接收路径的运行时补偿。*/
void ai_tune_poll(void);
/** @brief 在新坐标运动段起点应用待生效参数。 */
uint8_t ai_tune_apply_pending(void);
/** @brief 标记新坐标运动段开始，复位保护计时状态。 */
void ai_tune_segment_start(void);
/** @brief 获取当前活动参数。 */
const ai_tune_param_t *ai_tune_get_active(void);
/** @brief 查询在线调参模式状态。 */
uint8_t ai_tune_is_enabled(void);
/** @brief 进入在线调参待命状态。 */
void ai_tune_arm(void);
/** @brief 更新在线调参保护状态。 */
uint8_t ai_tune_guard_update(float distance_error_m, float linear_speed_mps,
        float dt);
/** @brief 查询当前运动段的保护状态。 */
uint8_t ai_tune_is_guarded(void);
/** @brief 发送一帧坐标闭环遥测数据。 */
void ai_tune_send_telemetry(const ai_tune_telemetry_t *telemetry);
/** @brief 读取上位机请求的固定测试目标。 */
uint8_t ai_tune_take_start(float *target_x_mm, float *target_y_mm,
        float *max_speed_mps);

#endif /* ROBOCUP_AI_TUNE_H */
