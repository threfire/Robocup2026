/**
 ******************************************************************************
 * @file    vision.h
 * @brief   MaixCam 视觉校正接口
 ******************************************************************************
 */

#ifndef VISION_H
#define VISION_H

#include <stdint.h>

/** 初始化视觉控制器和误差窗口。 */
void vision_init(void);
/** 设置视觉校正使能状态。 */
void vision_enable(uint8_t enable);
/** 更新视觉误差对应的底盘校正量。 */
void vision_update(float dt);
/** 等待目标点的视觉坐标帧。 */
uint8_t vision_wait_for_uart(uint8_t point_index);
/** 驱动底盘完成视觉居中。 */
uint8_t vision_center(int tolerance_x, int tolerance_y, int max_fluctuate_error);
/** 解析 USART1 接收的 MaixCam 坐标帧。 */
void vision_process_usart1_packet(void);

#endif /* VISION_H */
