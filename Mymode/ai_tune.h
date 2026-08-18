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
#define AI_TUNE_MODE_ENABLE 1U

typedef struct {
    uint32_t version;                 /* 上位机参数版本号 */
    float position_kp;                /* X/Y 位置环比例系数 */
    float position_ki;                /* X/Y 位置环积分系数 */
    float position_kd;                /* X/Y 位置环微分系数 */
    float accel_ff_gain_s;            /* 规划加速度前馈系数，单位 s */
    float com_ff_gain_s;              /* 质心补偿系数，单位 s */
    float yaw_kp;                     /* 偏航环比例系数 */
    float yaw_ki;                     /* 偏航环积分系数 */
    float yaw_kd;                     /* 偏航环微分系数 */
    float yaw_rate_ff_gain;           /* 偏航规划角速度前馈系数 */
} ai_tune_param_t;

typedef struct {
    uint32_t time_ms;                 /* 控制时间戳，单位 ms */
    float target_x_m;                 /* 目标世界坐标 X，单位 m */
    float target_y_m;                 /* 目标世界坐标 Y，单位 m */
    float trajectory_x_m;             /* 轨迹参考世界坐标 X，单位 m */
    float trajectory_y_m;             /* 轨迹参考世界坐标 Y，单位 m */
    float position_x_m;               /* 实际世界坐标 X，单位 m */
    float position_y_m;               /* 实际世界坐标 Y，单位 m */
    float target_yaw_deg;             /* 目标偏航角，单位 deg */
    float yaw_deg;                    /* 实际偏航角，单位 deg */
    float error_x_local_m;            /* 车体系 X 误差，单位 m */
    float error_y_local_m;            /* 车体系 Y 误差，单位 m */
    float velocity_x_mps;             /* 里程计世界系 X 速度，单位 m/s */
    float velocity_y_mps;             /* 里程计世界系 Y 速度，单位 m/s */
    float plan_vx_mps;                /* 规划局部 X 速度，单位 m/s */
    float plan_vy_mps;                /* 规划局部 Y 速度，单位 m/s */
    float plan_accel_x_mps2;          /* 规划局部 X 加速度，单位 m/s² */
    float plan_accel_y_mps2;          /* 规划局部 Y 加速度，单位 m/s² */
    float accel_ff_vx_mps;            /* 加速度前馈 X 输出，单位 m/s */
    float accel_ff_vy_mps;            /* 加速度前馈 Y 输出，单位 m/s */
    float com_ff_vx_mps;              /* 质心补偿 X 输出，单位 m/s */
    float com_ff_vy_mps;              /* 质心补偿 Y 输出，单位 m/s */
    float pid_p_vx_mps;               /* X 位置 PID 比例输出，单位 m/s */
    float pid_p_vy_mps;               /* Y 位置 PID 比例输出，单位 m/s */
    float pid_i_vx_mps;               /* X 位置 PID 积分输出，单位 m/s */
    float pid_i_vy_mps;               /* Y 位置 PID 积分输出，单位 m/s */
    float pid_d_vx_mps;               /* X 位置 PID 微分输出，单位 m/s */
    float pid_d_vy_mps;               /* Y 位置 PID 微分输出，单位 m/s */
    float pid_vx_mps;                 /* X 位置 PID 合量，单位 m/s */
    float pid_vy_mps;                 /* Y 位置 PID 合量，单位 m/s */
    float velocity_feedback_vx_mps;   /* 车体系 X 速度反馈输出，单位 m/s */
    float velocity_feedback_vy_mps;   /* 车体系 Y 速度反馈输出，单位 m/s */
    float yaw_plan_rate_rad_s;        /* 偏航规划角速度，单位 rad/s */
    float yaw_p_rad_s;                /* 偏航 PID 比例输出，单位 rad/s */
    float yaw_i_rad_s;                /* 偏航 PID 积分输出，单位 rad/s */
    float yaw_d_rad_s;                /* 偏航 PID 微分输出，单位 rad/s */
    float yaw_pid_rad_s;              /* 偏航 PID 合量，单位 rad/s */
    float command_vx_mps;             /* 最终 X 速度指令，单位 m/s */
    float command_vy_mps;             /* 最终 Y 速度指令，单位 m/s */
    float command_omega_rad_s;        /* 最终偏航角速度指令，单位 rad/s */
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
