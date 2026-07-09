/**
 ******************************************************************************
 * @file    odometer.c
 * @brief   里程计位姿接入模块实现
 *
 * @note    输入帧格式来自里程计 README，坐标为世界系坐标。
 ******************************************************************************
 */

#include "odometer.h"

#include <stddef.h>
#include <string.h>

odometer_context_t odometer_ctx;

/* 里程计原始世界系位姿，手动清零时用当前原始值作为新零点。 */
static float s_odometer_raw_x_mm;
static float s_odometer_raw_y_mm;
static float s_odometer_raw_yaw_deg;
static float s_odometer_zero_x_mm;
static float s_odometer_zero_y_mm;
static float s_odometer_zero_yaw_deg;
static uint8_t s_odometer_world_inited;

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
	s_odometer_raw_x_mm = x_mm;
	s_odometer_raw_y_mm = y_mm;
	s_odometer_raw_yaw_deg = yaw_deg;

	if (s_odometer_world_inited == 0U) {
		s_odometer_zero_x_mm = x_mm;
		s_odometer_zero_y_mm = y_mm;
		s_odometer_zero_yaw_deg = yaw_deg;
		s_odometer_world_inited = 1U;
	}

	odometer_ctx.pose.pos_x_mm = x_mm - s_odometer_zero_x_mm;
	odometer_ctx.pose.pos_y_mm = y_mm - s_odometer_zero_y_mm;
	odometer_ctx.pose.yaw_deg = yaw_deg - s_odometer_zero_yaw_deg;
	odometer_ctx.pose.yaw_gyro_dps = gyro_dps;
	odometer_ctx.pose.data_updated = 1U;
}

void odometer_init(void) {
	memset(&odometer_ctx, 0, sizeof(odometer_ctx));

	s_odometer_raw_x_mm = 0.0f;
	s_odometer_raw_y_mm = 0.0f;
	s_odometer_raw_yaw_deg = 0.0f;
	s_odometer_zero_x_mm = 0.0f;
	s_odometer_zero_y_mm = 0.0f;
	s_odometer_zero_yaw_deg = 0.0f;
	s_odometer_world_inited = 0U;
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

void odometer_update_zero(void) {
	/* 按键启动任务时重置零点，避免开机后里程计自身初始化漂移进入任务坐标。 */
	s_odometer_zero_x_mm = s_odometer_raw_x_mm;
	s_odometer_zero_y_mm = s_odometer_raw_y_mm;
	s_odometer_zero_yaw_deg = s_odometer_raw_yaw_deg;

	odometer_ctx.pose.pos_x_mm = 0.0f;
	odometer_ctx.pose.pos_y_mm = 0.0f;
	odometer_ctx.pose.yaw_deg = 0.0f;
	odometer_ctx.pose.data_updated = 0U;
	s_odometer_world_inited = 1U;
}
