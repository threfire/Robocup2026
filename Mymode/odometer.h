/**
 ******************************************************************************
 * @file    odometer.h
 * @brief   里程计位姿接入模块
 *
 * @note    UART4 接收里程计世界系位姿帧，模块内部维护零点和最新位姿。
 ******************************************************************************
 */

#ifndef ROBOCUP_MYMODE_ODOMETER_H
#define ROBOCUP_MYMODE_ODOMETER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ODOMETER_FRAME_LEN 28U    /* 里程计固定帧长度，单位 byte */
#define ODOMETER_HEAD0 0xA5U      /* 帧头第 1 字节 */
#define ODOMETER_HEAD1 0x5AU      /* 帧头第 2 字节 */
#define ODOMETER_X_OFFSET 2U      /* 世界系 x 坐标偏移，单位 mm */
#define ODOMETER_Y_OFFSET 6U      /* 世界系 y 坐标偏移，单位 mm */
#define ODOMETER_YAW_OFFSET 10U   /* 世界系 yaw 角偏移，单位 deg */
#define ODOMETER_GYRO_OFFSET 14U  /* yaw 角速度偏移，单位 deg/s */
#define ODOMETER_VEL_X_OFFSET 18U /* 世界系 X 速度偏移，单位 mm/s */
#define ODOMETER_VEL_Y_OFFSET 22U /* 世界系 Y 速度偏移，单位 mm/s */
#define ODOMETER_CRC_OFFSET 26U   /* CRC16 低字节偏移 */
#define ODOMETER_CRC_INIT 0xFFFFU /* CRC16 初值 */
#define ODOMETER_CRC_POLY 0xA001U /* CRC16 多项式 */
#define ODOMETER_CTRL_FRAME_LEN 6U
#define ODOMETER_CTRL_POSE_ZERO 0xC3U
#define ODOMETER_REMOTE_ZERO_TOLERANCE_MM 5.0f
#define ODOMETER_REMOTE_ZERO_TOLERANCE_DEG 2.0f
//#define ODOMETER_SENSOR_OFFSET_X_MM -27.14f
#define ODOMETER_SENSOR_OFFSET_X_MM 7.44f
#define ODOMETER_SENSOR_OFFSET_Y_MM 0.0f
#define ODOMETER_OBSERVER_ALPHA 0.20f /* 位置残差校正比例 */
#define ODOMETER_OBSERVER_BETA 0.02f  /* 速度残差校正比例 */
#define ODOMETER_OBSERVER_GAMMA 0.001f /* 加速度残差校正比例 */
#define ODOMETER_OBSERVER_YAW_ACCEL_FILTER 0.15f /* 角加速度低通系数 */
#define ODOMETER_OBSERVER_MAX_DT_MS 50U /* 观测器有效采样周期上限 */

typedef struct {
	float pos_x_mm;       /* 世界系 x 坐标，单位 mm */
	float pos_y_mm;       /* 世界系 y 坐标，单位 mm */
	float yaw_deg;        /* 世界系 yaw 角，单位 deg */
	float yaw_gyro_dps;   /* yaw 角速度，单位 deg/s */
	uint8_t data_updated; /* 里程计数据更新标志 */
} odometer_pose_t;

typedef struct {
	float vel_x_mps;       /* 世界系 x 速度，单位 m/s */
	float vel_y_mps;       /* 世界系 y 速度，单位 m/s */
	float accel_x_mps2;    /* 世界系 x 加速度，单位 m/s^2 */
	float accel_y_mps2;    /* 世界系 y 加速度，单位 m/s^2 */
	float yaw_rate_dps;    /* 偏航角速度，单位 deg/s */
	float yaw_accel_dps2;  /* 偏航角加速度，单位 deg/s^2 */
	uint8_t valid;         /* 观测器已接收连续位姿数据 */
} odometer_motion_t;

typedef struct {
	/* 同一个功能的公开状态统一放在模块上下文里，调用方通过 odometer_ctx 读取。 */
	odometer_pose_t pose;     /* 当前位姿状态 */
	odometer_motion_t motion; /* 坐标观测得到的运动状态 */
} odometer_context_t;

extern odometer_context_t odometer_ctx;

/**
 * @brief 初始化里程计状态。
 */
void odometer_init(void);

/**
 * @brief 解析里程计串口数据。
 *
 * @param data UART4 DMA 接收缓冲区。
 * @param len 接收数据长度，单位 byte。
 */
void odometer_rx_parse(const uint8_t *data, uint16_t len);

/**
 * @brief 获取当前 yaw 角。
 *
 * @return yaw 角，单位 deg。
 */
float odometer_get_yaw_deg(void);

/**
 * @brief 向外部里程计发送坐标归零命令。
 */
uint8_t odometer_request_zero(void);

/**
 * @brief 获取外部里程计归零确认状态。
 */
uint8_t odometer_is_zeroed(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBOCUP_MYMODE_ODOMETER_H */
