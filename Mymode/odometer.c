/**
 ******************************************************************************
 * @file    odometer.c
 * @brief   里程计位姿接入模块实现
 *
 * @note    输入帧格式来自里程计 README，坐标为世界系坐标。
 ******************************************************************************
 */

#include "odometer.h"
#include "main.h"
#include "usart.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

odometer_context_t odometer_ctx;

/* 里程计原始世界系位姿，手动清零时用当前原始值作为新零点。 */
static float s_observer_pos_x_m;
static float s_observer_pos_y_m;
static float s_observer_last_gyro_dps;
static uint32_t s_observer_last_time_ms;
static uint8_t s_observer_initialized;
static uint8_t s_remote_zero_pending;

/**
 * @brief 使用当前测量值重置运动状态观测器。
 */
static void odometer_observer_reset(
        float pos_x_m, float pos_y_m, float gyro_dps) {
	s_observer_pos_x_m = pos_x_m;
	s_observer_pos_y_m = pos_y_m;
	s_observer_last_gyro_dps = gyro_dps;
	s_observer_last_time_ms = HAL_GetTick();
	s_observer_initialized = 1U;
	memset(&odometer_ctx.motion, 0, sizeof(odometer_ctx.motion));
	odometer_ctx.motion.yaw_rate_dps = gyro_dps;
}

/**
 * @brief 用里程计坐标和陀螺仪角速度更新 α-β-γ 观测状态。
 */
static void odometer_observer_update(
        float pos_x_m, float pos_y_m, float gyro_dps) {
	uint32_t now_ms = HAL_GetTick();
	uint32_t elapsed_ms;
	float dt;
	float predicted_x_m;
	float predicted_y_m;
	float residual_x_m;
	float residual_y_m;
	float gyro_accel_dps2;

	if (!s_observer_initialized) {
		odometer_observer_reset(pos_x_m, pos_y_m, gyro_dps);
		return;
	}

	elapsed_ms = now_ms - s_observer_last_time_ms;
	if ((elapsed_ms == 0U) || (elapsed_ms > ODOMETER_OBSERVER_MAX_DT_MS)) {
		odometer_observer_reset(pos_x_m, pos_y_m, gyro_dps);
		return;
	}

	dt = (float)elapsed_ms * 0.001f;
	predicted_x_m = s_observer_pos_x_m +
	                odometer_ctx.motion.vel_x_mps * dt +
	                0.5f * odometer_ctx.motion.accel_x_mps2 * dt * dt;
	predicted_y_m = s_observer_pos_y_m +
	                odometer_ctx.motion.vel_y_mps * dt +
	                0.5f * odometer_ctx.motion.accel_y_mps2 * dt * dt;
	residual_x_m = pos_x_m - predicted_x_m;
	residual_y_m = pos_y_m - predicted_y_m;

	s_observer_pos_x_m = predicted_x_m + ODOMETER_OBSERVER_ALPHA * residual_x_m;
	s_observer_pos_y_m = predicted_y_m + ODOMETER_OBSERVER_ALPHA * residual_y_m;
	odometer_ctx.motion.vel_x_mps +=
	        odometer_ctx.motion.accel_x_mps2 * dt +
	        ODOMETER_OBSERVER_BETA * residual_x_m / dt;
	odometer_ctx.motion.vel_y_mps +=
	        odometer_ctx.motion.accel_y_mps2 * dt +
	        ODOMETER_OBSERVER_BETA * residual_y_m / dt;
	odometer_ctx.motion.accel_x_mps2 +=
	        2.0f * ODOMETER_OBSERVER_GAMMA * residual_x_m / (dt * dt);
	odometer_ctx.motion.accel_y_mps2 +=
	        2.0f * ODOMETER_OBSERVER_GAMMA * residual_y_m / (dt * dt);
	gyro_accel_dps2 = (gyro_dps - s_observer_last_gyro_dps) / dt;
	odometer_ctx.motion.yaw_accel_dps2 +=
	        ODOMETER_OBSERVER_YAW_ACCEL_FILTER *
	        (gyro_accel_dps2 - odometer_ctx.motion.yaw_accel_dps2);
	odometer_ctx.motion.yaw_rate_dps = gyro_dps;
	odometer_ctx.motion.valid = 1U;
	s_observer_last_gyro_dps = gyro_dps;
	s_observer_last_time_ms = now_ms;
}

/**
 * @brief 计算里程计帧 CRC16。
 *
 * 协议使用 Modbus CRC16，小端放在帧尾，计算范围为帧头到 gyro 字段。
 */
static uint16_t odometer_crc16(const uint8_t *data, uint16_t length) {
	uint16_t crc = ODOMETER_CRC_INIT;
	uint16_t i;
	uint8_t bit;

	for (i = 0U; i < length; i++) {
		crc ^= data[i];
		for (bit = 0U; bit < 8U; bit++) {
			if ((crc & 0x0001U) != 0U) {
				crc = (uint16_t)((crc >> 1) ^
				                 ODOMETER_CRC_POLY);
			} else {
				crc >>= 1;
			}
		}
	}

	return crc;
}

/**
 * @brief 读取小端 float。
 *
 * 里程计协议以 4 字节 IEEE754 float 传输 x、y、yaw 和 yaw 角速度。
 */
static float odometer_read_float_le(const uint8_t *data) {
	float value;

	memcpy(&value, data, sizeof(value));
	return value;
}

/**
 * @brief 更新零点后的位姿状态。
 *
 * 第一次收到有效帧时自动记录世界系零点，之后输出相对启动零点的位姿。
 */
static void odometer_update_pose(
        float x_mm, float y_mm, float yaw_deg, float gyro_dps) {
	odometer_ctx.pose.pos_x_mm = x_mm;
	odometer_ctx.pose.pos_y_mm = y_mm;
	odometer_ctx.pose.yaw_deg = yaw_deg;
	odometer_ctx.pose.yaw_gyro_dps = gyro_dps;
	odometer_ctx.pose.data_updated = 1U;
	if (s_remote_zero_pending &&
	        (fabsf(x_mm) <= ODOMETER_REMOTE_ZERO_TOLERANCE_MM) &&
	        (fabsf(y_mm) <= ODOMETER_REMOTE_ZERO_TOLERANCE_MM) &&
	        (fabsf(yaw_deg) <= ODOMETER_REMOTE_ZERO_TOLERANCE_DEG)) {
		s_remote_zero_pending = 0U;
		odometer_observer_reset(x_mm * 0.001f, y_mm * 0.001f, gyro_dps);
		return;
	}
	odometer_observer_update(odometer_ctx.pose.pos_x_mm * 0.001f,
	        odometer_ctx.pose.pos_y_mm * 0.001f, gyro_dps);
}

void odometer_init(void) {
	memset(&odometer_ctx, 0, sizeof(odometer_ctx));

	s_observer_initialized = 0U;
	s_remote_zero_pending = 0U;
}

void odometer_rx_parse(const uint8_t *data, uint16_t len) {
	uint16_t frame_offset;
	uint16_t rx_crc;
	uint16_t calc_crc;
	float x_mm;
	float y_mm;
	float yaw_deg;
	float gyro_dps;

	if ((data == NULL) || (len < ODOMETER_FRAME_LEN)) {
		return;
	}

	/* UART4 使用 DMA 空闲中断收包，缓冲区内可能带有帧前杂字节，需要扫描帧头。 */
	for (frame_offset = 0U;
	        frame_offset <= (uint16_t)(len - ODOMETER_FRAME_LEN);
	        frame_offset++) {
		if ((data[frame_offset] == ODOMETER_HEAD0) &&
		        (data[frame_offset + 1U] == ODOMETER_HEAD1)) {
			rx_crc = (uint16_t)data[frame_offset +
			                        ODOMETER_CRC_OFFSET] |
			         ((uint16_t)data[frame_offset +
			                         ODOMETER_CRC_OFFSET + 1U]
			                 << 8);
			calc_crc = odometer_crc16(
			        &data[frame_offset], ODOMETER_FRAME_LEN - 2U);
			if (rx_crc == calc_crc) {
				/* 坐标和姿态由里程计内部融合完成，控制层直接使用世界系位姿。 */
				x_mm = odometer_read_float_le(
				        &data[frame_offset +
				                ODOMETER_X_OFFSET]);
				y_mm = odometer_read_float_le(
				        &data[frame_offset +
				                ODOMETER_Y_OFFSET]);
				yaw_deg = odometer_read_float_le(
				        &data[frame_offset +
				                ODOMETER_YAW_OFFSET]);
				gyro_dps = odometer_read_float_le(
				        &data[frame_offset +
				                ODOMETER_GYRO_OFFSET]);

				odometer_update_pose(
				        x_mm, y_mm, yaw_deg, gyro_dps);
				return;
			}
		}
	}
}

float odometer_get_yaw_deg(void) {
	return odometer_ctx.pose.yaw_deg;
}

/**
 * @brief 发送外部里程计坐标归零控制帧。
 */
uint8_t odometer_request_zero(void) {
	uint8_t frame[ODOMETER_CTRL_FRAME_LEN] = {
	        ODOMETER_HEAD0, ODOMETER_HEAD1, ODOMETER_CTRL_POSE_ZERO, 0U, 0U, 0U};
	uint16_t crc = odometer_crc16(frame, 4U);

	frame[4] = (uint8_t)(crc & 0xFFU);
	frame[5] = (uint8_t)(crc >> 8);
	if (HAL_UART_Transmit(&huart4, frame, sizeof(frame), 10U) != HAL_OK) {
		return 0U;
	}

	s_remote_zero_pending = 1U;
	odometer_ctx.pose.data_updated = 0U;
	return 1U;
}

/**
 * @brief 判断外部里程计归零后的位姿帧是否已到达。
 */
uint8_t odometer_is_zeroed(void) {
	return (s_remote_zero_pending == 0U) && odometer_ctx.pose.data_updated;
}
