/**
 ******************************************************************************
 * @file    vision.c
 * @brief   MaixCam 视觉接收、校正与到位判定
 ******************************************************************************
 */

#include "vision.h"
#include <string.h>
#include "main.h"
#include "usart.h"
#include "control.h"

typedef struct {
    int16_t x;
    int16_t y;
} VisionError_t;

typedef struct {
    int errors_x[ERROR_QUEUE_SIZE];
    int errors_y[ERROR_QUEUE_SIZE];
    uint8_t index;
    uint8_t count;
    uint8_t is_full;
} VisionErrorQueue_t;

static VisionError_t vision_error;
static VisionErrorQueue_t vision_error_queue;
static uint32_t vision_stabilize_start_time;


/** 清空视觉误差滑动窗口。 */
static void vision_error_queue_init(void)
{
    vision_error_queue.index = 0U;
    vision_error_queue.count = 0U;
    vision_error_queue.is_full = 0U;
    memset(vision_error_queue.errors_x, 0, sizeof(vision_error_queue.errors_x));
    memset(vision_error_queue.errors_y, 0, sizeof(vision_error_queue.errors_y));
}


/** 将一帧图像误差写入滑动窗口。 */
static void vision_error_queue_push(int error_x, int error_y)
{
    vision_error_queue.errors_x[vision_error_queue.index] = error_x;
    vision_error_queue.errors_y[vision_error_queue.index] = error_y;
    vision_error_queue.index = (vision_error_queue.index + 1U) % ERROR_QUEUE_SIZE;

    if (!vision_error_queue.is_full) {
        vision_error_queue.count++;
        if (vision_error_queue.count >= ERROR_QUEUE_SIZE) {
            vision_error_queue.is_full = 1U;
        }
    }
}


static void vision_error_queue_get_average(float *avg_x, float *avg_y)
{
    int sum_x = 0;
    int sum_y = 0;
    uint8_t valid_count = vision_error_queue.is_full ? ERROR_QUEUE_SIZE :
            vision_error_queue.count;

    if (valid_count == 0U) {
        *avg_x = 0.0f;
        *avg_y = 0.0f;
        return;
    }

    for (uint8_t i = 0U; i < valid_count; i++) {
        sum_x += abs_int(vision_error_queue.errors_x[i]);
        sum_y += abs_int(vision_error_queue.errors_y[i]);
    }

    *avg_x = (float)sum_x / valid_count;
    *avg_y = (float)sum_y / valid_count;
}


static void vision_error_queue_get_fluctuate(int *fluct_x, int *fluct_y)
{
    int max_x = -CAM_ERROR_ABS_LIMIT;
    int min_x = CAM_ERROR_ABS_LIMIT;
    int max_y = -CAM_ERROR_ABS_LIMIT;
    int min_y = CAM_ERROR_ABS_LIMIT;
    uint8_t valid_count = vision_error_queue.is_full ? ERROR_QUEUE_SIZE :
            vision_error_queue.count;

    if (valid_count == 0U) {
        *fluct_x = 0;
        *fluct_y = 0;
        return;
    }

    for (uint8_t i = 0U; i < valid_count; i++) {
        if (vision_error_queue.errors_x[i] > max_x)
            max_x = vision_error_queue.errors_x[i];
        if (vision_error_queue.errors_x[i] < min_x)
            min_x = vision_error_queue.errors_x[i];
        if (vision_error_queue.errors_y[i] > max_y)
            max_y = vision_error_queue.errors_y[i];
        if (vision_error_queue.errors_y[i] < min_y)
            min_y = vision_error_queue.errors_y[i];
    }

    *fluct_x = max_x - min_x;
    *fluct_y = max_y - min_y;
}


static uint8_t vision_is_reached(int tolerance_x, int tolerance_y,
        int max_fluctuate_error)
{
    float avg_error_x;
    float avg_error_y;
    int fluct_x;
    int fluct_y;

    if (!Is_coordgot) {
        return 0U;
    }

    vision_error_queue_push(vision_error.x, vision_error.y);
    vision_error_queue_get_average(&avg_error_x, &avg_error_y);
    vision_error_queue_get_fluctuate(&fluct_x, &fluct_y);

    return (uint8_t)(vision_error_queue.is_full &&
            avg_error_x <= tolerance_x && avg_error_y <= tolerance_y &&
            fluct_x <= max_fluctuate_error && fluct_y <= max_fluctuate_error);
}

/** 初始化视觉 PID 和运行状态。 */
void vision_init(void)
{
    pid_init_cam(&cam_stabilization.pid_x, VISION_PID_KP, VISION_PID_KP2,
            VISION_PID_KI, VISION_PID_KD, VISION_PID_MAX_OUTPUT);
    pid_init_cam(&cam_stabilization.pid_y, VISION_PID_KP, VISION_PID_KP2,
            VISION_PID_KI, VISION_PID_KD, VISION_PID_MAX_OUTPUT);
    cam_stabilization.is_enabled = 0U;
    cam_correction_x = 0.0f;
    cam_correction_y = 0.0f;
    lookmode_centrl = 0U;
    is_cam_stabilizing = 0U;
    Is_coordgot = 0;
}

/** 设置视觉校正使能状态。 */
void vision_enable(uint8_t enable)
{
    cam_stabilization.is_enabled = enable;
    cam_stabilization.pid_x.integral = 0.0f;
    cam_stabilization.pid_x.prev_error = 0.0f;
    cam_stabilization.pid_y.integral = 0.0f;
    cam_stabilization.pid_y.prev_error = 0.0f;
    cam_correction_x = 0.0f;
    cam_correction_y = 0.0f;
    IsLight = enable ? 1U : 0U;
}

/** 将图像误差计算为底盘校正量。 */
void vision_update(float dt)
{
    float actual_x_err;
    float actual_y_err;

    if (!cam_stabilization.is_enabled || dt <= 0.0f) {
        cam_correction_x = 0.0f;
        cam_correction_y = 0.0f;
        return;
    }

    actual_x_err = abs_float(vision_error.x) > CAM_DEAD_ZONE ? vision_error.x : 0.0f;
    actual_y_err = abs_float(vision_error.y) > CAM_DEAD_ZONE ? vision_error.y : 0.0f;
    cam_correction_x = pid_calc_cam(&cam_stabilization.pid_x, actual_x_err, dt);
    cam_correction_y = pid_calc_cam(&cam_stabilization.pid_y, actual_y_err, dt);
}

/** 等待视觉坐标帧或超时。 */
uint8_t vision_wait_for_uart(uint8_t point_index)
{
    uint32_t start_time = HAL_GetTick();

    (void)point_index;
    Is_coordgot = 0;
    while (!Is_coordgot) {
        if (HAL_GetTick() - start_time > UART_WAIT_TIMEOUT_MS) {
            return 0U;
        }
        HAL_Delay(1U);
    }

    return 1U;
}

/** 执行视觉居中过程。 */
uint8_t vision_center(int tolerance_x, int tolerance_y, int max_fluctuate_error)
{
    if (!is_cam_stabilizing) {
        vision_error_queue_init();
        PositionControl_Enable(0U);
        vision_enable(1U);
        is_cam_stabilizing = 1U;
        iscamreached_safecount = 0U;
        vision_stabilize_start_time = HAL_GetTick();
        IsLight = 1U;
        return 0U;
    }

    if (HAL_GetTick() - vision_stabilize_start_time > CAM_STABILIZE_TIMEOUT_MS) {
        vision_enable(0U);
        is_cam_stabilizing = 0U;
        IsLight = 0U;
        return 1U;
    }

    if (vision_is_reached(tolerance_x, tolerance_y, max_fluctuate_error)) {
        iscamreached_safecount++;
        if (iscamreached_safecount > 10U) {
            vision_enable(0U);
            is_cam_stabilizing = 0U;
            IsLight = 0U;
            return 1U;
        }
    }

    return 0U;
}

/** 解析 USART1 接收的 MaixCam 坐标帧。 */
void vision_process_usart1_packet(void)
{
    uint8_t camera_packet[9] = {0};

    if (!recv_end_flag || rx_len < sizeof(camera_packet)) {
        return;
    }

    for (uint8_t i = 0U; i < sizeof(camera_packet); i++) {
        camera_packet[i] = rx_buffer[i];
    }

    if (camera_packet[0] == 0xB3U && camera_packet[1] == 0xB3U &&
            camera_packet[2] == 0x02U) {
        int16_t x_coord = (int16_t)(camera_packet[3] + camera_packet[4]);
        int16_t y_coord = (int16_t)(camera_packet[5] + camera_packet[6]);

        vision_error.x = 180 - x_coord;
        if (task1_state.current_step > 30U) {
            vision_error.y = 130 - y_coord;
        } else {
            vision_error.y = 125 - y_coord;
        }
        Is_coordgot = 1;
    }
}
