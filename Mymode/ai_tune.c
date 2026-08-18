/**
 ******************************************************************************
 * @file    ai_tune.c
 * @brief   坐标闭环在线调参串口实现。
 ******************************************************************************
 */

#include "ai_tune.h"

#include <string.h>

#include "control.h"
#include "usart.h"

#if (AI_TUNE_MODE_ENABLE != 0U)
#define AI_TUNE_FRAME_HEAD0 0xC3U
#define AI_TUNE_FRAME_HEAD1 0x3CU
#define AI_TUNE_CMD_PARAM 0x01U
#define AI_TUNE_CMD_COMMIT 0x02U
#define AI_TUNE_CMD_START 0x03U
#define AI_TUNE_TELEMETRY 0x81U
#define AI_TUNE_ACK 0x82U
#define AI_TUNE_RX_BUFFER_SIZE 64U
#define AI_TUNE_TX_BUFFER_SIZE 64U
#define AI_TUNE_TELEMETRY_PERIOD_MS 5U
#define AI_TUNE_PARAM_PAYLOAD_SIZE 40U
#define AI_TUNE_START_PAYLOAD_SIZE 12U
#endif
#define AI_TUNE_INCREASE_GUARD_MS 150U
#define AI_TUNE_SATURATION_GUARD_MS 100U
#define AI_TUNE_TIMEOUT_MS 8000U
#define AI_TUNE_DISTANCE_GUARD_M 0.50f
#define AI_TUNE_DISTANCE_INCREASE_M 0.001f
#define AI_TUNE_SPEED_GUARD_MPS 6.99f

static const ai_tune_param_t ai_tune_default_param = {
        .version = 0U,
        .position_kp = POSITION_PID_DEFAULT_KP,
        .position_ki = 0.0f,
        .position_kd = POSITION_PID_DEFAULT_KD,
        .accel_ff_gain_s = POSITION_ACCEL_FEEDFORWARD_GAIN,
        .com_ff_gain_s = POSITION_COM_FEEDFORWARD_GAIN,
        .yaw_kp = ATTITUDE_PID_DEFAULT_KP,
        .yaw_ki = ATTITUDE_PID_DEFAULT_KI,
        .yaw_kd = ATTITUDE_PID_DEFAULT_KD,
        .yaw_rate_ff_gain = ATTITUDE_RATE_FEEDFORWARD_GAIN,
};

static ai_tune_param_t ai_tune_active_param;
static ai_tune_param_t ai_tune_pending_param;
static ai_tune_param_t ai_tune_verified_param;
static uint8_t ai_tune_pending_valid;
static uint8_t ai_tune_enabled;
static uint8_t ai_tune_guarded;
static uint8_t ai_tune_segment_active;
static uint8_t ai_tune_start_pending;
static float ai_tune_start_x_mm;
static float ai_tune_start_y_mm;
static float ai_tune_start_speed_mps;
static float ai_tune_last_distance_m;
static float ai_tune_min_distance_m;
static float ai_tune_increase_time_s;
static float ai_tune_saturation_time_s;
static uint32_t ai_tune_segment_start_ms;
#if (AI_TUNE_MODE_ENABLE != 0U)
static uint32_t ai_tune_last_telemetry_ms;
static uint16_t ai_tune_tx_sequence;
static uint8_t ai_tune_telemetry_frame_type;
static volatile uint8_t ai_tune_ack_pending;
static volatile uint8_t ai_tune_ack_command;
static volatile uint8_t ai_tune_ack_status;
static volatile uint16_t ai_tune_ack_sequence;
static volatile uint32_t ai_tune_ack_version;
static uint8_t ai_tune_rx_buffer[AI_TUNE_RX_BUFFER_SIZE];
static uint8_t ai_tune_tx_buffer[AI_TUNE_TX_BUFFER_SIZE];
#endif

#if (AI_TUNE_MODE_ENABLE != 0U)
static uint16_t ai_tune_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < length; index++) {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1U) ^ 0xA001U;
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

static uint32_t ai_tune_read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
            ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static float ai_tune_read_float(const uint8_t *data)
{
    union {
        uint32_t u32;
        float value;
    } converter;

    converter.u32 = ai_tune_read_u32(data);
    return converter.value;
}

static void ai_tune_write_u16(uint8_t *data, uint16_t *index, uint16_t value)
{
    data[(*index)++] = (uint8_t)value;
    data[(*index)++] = (uint8_t)(value >> 8U);
}

static void ai_tune_write_u32(uint8_t *data, uint16_t *index, uint32_t value)
{
    data[(*index)++] = (uint8_t)value;
    data[(*index)++] = (uint8_t)(value >> 8U);
    data[(*index)++] = (uint8_t)(value >> 16U);
    data[(*index)++] = (uint8_t)(value >> 24U);
}

static void ai_tune_write_scaled_i16(uint8_t *data, uint16_t *index,
        float value, float scale)
{
    float scaled_value = value * scale;

    if (scaled_value > 32767.0f) {
        scaled_value = 32767.0f;
    }
    if (scaled_value < -32768.0f) {
        scaled_value = -32768.0f;
    }
    ai_tune_write_u16(data, index, (uint16_t)(int16_t)scaled_value);
}

static void ai_tune_queue_ack(uint8_t command, uint8_t status,
        uint16_t sequence, uint32_t version)
{
    ai_tune_ack_command = command;
    ai_tune_ack_status = status;
    ai_tune_ack_sequence = sequence;
    ai_tune_ack_version = version;
    ai_tune_ack_pending = 1U;
}

static uint8_t ai_tune_param_valid(const ai_tune_param_t *param)
{
    if ((param->position_kp != param->position_kp) ||
            (param->position_ki != param->position_ki) ||
            (param->position_kd != param->position_kd) ||
            (param->accel_ff_gain_s != param->accel_ff_gain_s) ||
            (param->com_ff_gain_s != param->com_ff_gain_s) ||
            (param->yaw_kp != param->yaw_kp) ||
            (param->yaw_ki != param->yaw_ki) ||
            (param->yaw_kd != param->yaw_kd) ||
            (param->yaw_rate_ff_gain != param->yaw_rate_ff_gain)) {
        return 0U;
    }
    if ((param->position_kp < 0.0f) || (param->position_kp > 30.0f) ||
            (param->position_ki < 0.0f) || (param->position_ki > 10.0f) ||
            (param->position_kd < 0.0f) || (param->position_kd > 10.0f) ||
            (param->accel_ff_gain_s < 0.0f) ||
            (param->accel_ff_gain_s > 1.0f) ||
            (param->com_ff_gain_s < 0.0f) ||
            (param->com_ff_gain_s > 1.0f) ||
            (param->yaw_kp < 0.0f) || (param->yaw_kp > 30.0f) ||
            (param->yaw_ki < 0.0f) || (param->yaw_ki > 10.0f) ||
            (param->yaw_kd < 0.0f) || (param->yaw_kd > 10.0f) ||
            (param->yaw_rate_ff_gain < 0.0f) ||
            (param->yaw_rate_ff_gain > 2.0f)) {
        return 0U;
    }
    return 1U;
}

static void ai_tune_parse_frame(const uint8_t *frame, uint8_t payload_length)
{
    const uint8_t *payload = &frame[6];
    uint16_t sequence = (uint16_t)frame[4] |
            ((uint16_t)frame[5] << 8U);

    if ((frame[2] == AI_TUNE_CMD_PARAM) &&
            (payload_length == AI_TUNE_PARAM_PAYLOAD_SIZE)) {
        ai_tune_param_t param;

        param.version = ai_tune_read_u32(&payload[0]);
        param.position_kp = ai_tune_read_float(&payload[4]);
        param.position_ki = ai_tune_read_float(&payload[8]);
        param.position_kd = ai_tune_read_float(&payload[12]);
        param.accel_ff_gain_s = ai_tune_read_float(&payload[16]);
        param.com_ff_gain_s = ai_tune_read_float(&payload[20]);
        param.yaw_kp = ai_tune_read_float(&payload[24]);
        param.yaw_ki = ai_tune_read_float(&payload[28]);
        param.yaw_kd = ai_tune_read_float(&payload[32]);
        param.yaw_rate_ff_gain = ai_tune_read_float(&payload[36]);
        if (ai_tune_param_valid(&param)) {
            ai_tune_pending_param = param;
            ai_tune_pending_valid = 1U;
            ai_tune_queue_ack(AI_TUNE_CMD_PARAM, 0U, sequence,
                    ai_tune_active_param.version);
        } else {
            ai_tune_queue_ack(AI_TUNE_CMD_PARAM, 1U, sequence,
                    ai_tune_active_param.version);
        }
        return;
    }
    if ((frame[2] == AI_TUNE_CMD_COMMIT) && (payload_length == 4U)) {
        if (ai_tune_read_u32(payload) == ai_tune_active_param.version) {
            ai_tune_verified_param = ai_tune_active_param;
            ai_tune_queue_ack(AI_TUNE_CMD_COMMIT, 0U, sequence,
                    ai_tune_active_param.version);
        } else {
            ai_tune_queue_ack(AI_TUNE_CMD_COMMIT, 1U, sequence,
                    ai_tune_active_param.version);
        }
        return;
    }
    if ((frame[2] == AI_TUNE_CMD_START) &&
            (payload_length == AI_TUNE_START_PAYLOAD_SIZE)) {
        float target_x_mm = ai_tune_read_float(&payload[0]);
        float target_y_mm = ai_tune_read_float(&payload[4]);
        float max_speed_mps = ai_tune_read_float(&payload[8]);

        if ((target_x_mm >= -3000.0f) && (target_x_mm <= 3000.0f) &&
                (target_y_mm >= -3000.0f) && (target_y_mm <= 3000.0f) &&
                (max_speed_mps >= 0.20f) && (max_speed_mps <= 7.0f)) {
            ai_tune_start_x_mm = target_x_mm;
            ai_tune_start_y_mm = target_y_mm;
            ai_tune_start_speed_mps = max_speed_mps;
            ai_tune_start_pending = 1U;
            ai_tune_queue_ack(AI_TUNE_CMD_START, 0U, sequence,
                    ai_tune_active_param.version);
        } else {
            ai_tune_queue_ack(AI_TUNE_CMD_START, 1U, sequence,
                    ai_tune_active_param.version);
        }
    }
}

static void ai_tune_parse_rx(uint16_t length)
{
    uint16_t index = 0U;

    while ((index + 8U) <= length) {
        uint8_t payload_length;
        uint16_t frame_length;
        uint16_t received_crc;
        uint16_t calculated_crc;

        if ((ai_tune_rx_buffer[index] != AI_TUNE_FRAME_HEAD0) ||
                (ai_tune_rx_buffer[index + 1U] != AI_TUNE_FRAME_HEAD1)) {
            index++;
            continue;
        }
        payload_length = ai_tune_rx_buffer[index + 3U];
        frame_length = (uint16_t)payload_length + 8U;
        if ((index + frame_length) > length) {
            break;
        }
        received_crc = (uint16_t)ai_tune_rx_buffer[index + frame_length - 2U] |
                ((uint16_t)ai_tune_rx_buffer[index + frame_length - 1U] << 8U);
        calculated_crc = ai_tune_crc16(&ai_tune_rx_buffer[index + 2U],
                (uint16_t)payload_length + 4U);
        if (received_crc == calculated_crc) {
            ai_tune_parse_frame(&ai_tune_rx_buffer[index], payload_length);
        }
        index += frame_length;
    }
}
#endif

void ai_tune_init(void)
{
    ai_tune_active_param = ai_tune_default_param;
    ai_tune_verified_param = ai_tune_default_param;
    ai_tune_pending_valid = 0U;
    ai_tune_enabled = 0U;
    ai_tune_guarded = 0U;
    ai_tune_segment_active = 0U;
    ai_tune_start_pending = 0U;
#if (AI_TUNE_MODE_ENABLE != 0U)
    ai_tune_tx_sequence = 0U;
    ai_tune_telemetry_frame_type = 0U;
    ai_tune_ack_pending = 0U;
    ai_tune_last_telemetry_ms = HAL_GetTick();
    memset(ai_tune_rx_buffer, 0, sizeof(ai_tune_rx_buffer));
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart5, ai_tune_rx_buffer, sizeof(ai_tune_rx_buffer));
#endif
}

void ai_tune_uart_idle_irq(void)
{
#if (AI_TUNE_MODE_ENABLE != 0U)
    uint16_t received_length;

    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE) == RESET) {
        return;
    }
    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    received_length = AI_TUNE_RX_BUFFER_SIZE -
            __HAL_DMA_GET_COUNTER(huart5.hdmarx);
    HAL_UART_AbortReceive(&huart5);
    if (received_length > 0U) {
        ai_tune_parse_rx(received_length);
    }
    HAL_UART_Receive_DMA(&huart5, ai_tune_rx_buffer, sizeof(ai_tune_rx_buffer));
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
#endif
}

void ai_tune_poll(void)
{
#if (AI_TUNE_MODE_ENABLE != 0U)
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE) != RESET) {
        ai_tune_uart_idle_irq();
    }
#endif
}

uint8_t ai_tune_apply_pending(void)
{
    if (ai_tune_pending_valid == 0U) {
        return 0U;
    }
    ai_tune_active_param = ai_tune_pending_param;
    ai_tune_pending_valid = 0U;
    ai_tune_enabled = 1U;
    return 1U;
}

void ai_tune_segment_start(void)
{
    ai_tune_guarded = 0U;
    ai_tune_segment_active = 1U;
    ai_tune_last_distance_m = 0.0f;
    ai_tune_min_distance_m = 1000.0f;
    ai_tune_increase_time_s = 0.0f;
    ai_tune_saturation_time_s = 0.0f;
    ai_tune_segment_start_ms = HAL_GetTick();
}

const ai_tune_param_t *ai_tune_get_active(void)
{
    return &ai_tune_active_param;
}

uint8_t ai_tune_is_enabled(void)
{
    return ai_tune_enabled;
}

void ai_tune_arm(void)
{
    ai_tune_enabled = 1U;
}

uint8_t ai_tune_guard_update(float distance_error_m, float linear_speed_mps,
        float dt)
{
    if ((ai_tune_enabled == 0U) || (ai_tune_segment_active == 0U) ||
            (dt <= 0.0f)) {
        return 0U;
    }
    if (distance_error_m < ai_tune_min_distance_m) {
        ai_tune_min_distance_m = distance_error_m;
    }
    if (distance_error_m > (ai_tune_last_distance_m +
            AI_TUNE_DISTANCE_INCREASE_M)) {
        ai_tune_increase_time_s += dt;
    } else {
        ai_tune_increase_time_s = 0.0f;
    }
    if (linear_speed_mps >= AI_TUNE_SPEED_GUARD_MPS) {
        ai_tune_saturation_time_s += dt;
    } else {
        ai_tune_saturation_time_s = 0.0f;
    }
    ai_tune_last_distance_m = distance_error_m;
    if ((distance_error_m > (ai_tune_min_distance_m +
            AI_TUNE_DISTANCE_GUARD_M)) ||
            (ai_tune_increase_time_s >=
            ((float)AI_TUNE_INCREASE_GUARD_MS * 0.001f)) ||
            (ai_tune_saturation_time_s >=
            ((float)AI_TUNE_SATURATION_GUARD_MS * 0.001f)) ||
            ((HAL_GetTick() - ai_tune_segment_start_ms) >= AI_TUNE_TIMEOUT_MS)) {
        ai_tune_active_param = ai_tune_verified_param;
        ai_tune_guarded = 1U;
        ai_tune_segment_active = 0U;
        return 1U;
    }
    return 0U;
}

uint8_t ai_tune_is_guarded(void)
{
    return ai_tune_guarded;
}

void ai_tune_send_telemetry(const ai_tune_telemetry_t *telemetry)
{
#if (AI_TUNE_MODE_ENABLE != 0U)
    uint16_t index = 0U;
    uint16_t crc;
    uint32_t now_ms;

    if (telemetry == NULL) {
        return;
    }
    now_ms = HAL_GetTick();
    if (huart5.gState != HAL_UART_STATE_READY) {
        return;
    }
    if (ai_tune_ack_pending != 0U) {
        index = 0U;
        ai_tune_tx_buffer[index++] = AI_TUNE_FRAME_HEAD0;
        ai_tune_tx_buffer[index++] = AI_TUNE_FRAME_HEAD1;
        ai_tune_tx_buffer[index++] = AI_TUNE_ACK;
        ai_tune_tx_buffer[index++] = 6U;
        ai_tune_write_u16(ai_tune_tx_buffer, &index,
                ai_tune_ack_sequence);
        ai_tune_tx_buffer[index++] = ai_tune_ack_command;
        ai_tune_tx_buffer[index++] = ai_tune_ack_status;
        ai_tune_write_u32(ai_tune_tx_buffer, &index, ai_tune_ack_version);
        crc = ai_tune_crc16(&ai_tune_tx_buffer[2], index - 2U);
        ai_tune_write_u16(ai_tune_tx_buffer, &index, crc);
        ai_tune_ack_pending = 0U;
        HAL_UART_Transmit_DMA(&huart5, ai_tune_tx_buffer, index);
        return;
    }
    if ((now_ms - ai_tune_last_telemetry_ms) < AI_TUNE_TELEMETRY_PERIOD_MS) {
        return;
    }
    ai_tune_last_telemetry_ms = now_ms;
    ai_tune_tx_buffer[index++] = AI_TUNE_FRAME_HEAD0;
    ai_tune_tx_buffer[index++] = AI_TUNE_FRAME_HEAD1;
    ai_tune_tx_buffer[index++] = AI_TUNE_TELEMETRY;
    ai_tune_tx_buffer[index++] = 0U;
    ai_tune_write_u16(ai_tune_tx_buffer, &index, ai_tune_tx_sequence++);
    ai_tune_tx_buffer[index++] = ai_tune_telemetry_frame_type;
    ai_tune_write_u32(ai_tune_tx_buffer, &index, telemetry->time_ms);
    ai_tune_write_u32(ai_tune_tx_buffer, &index, ai_tune_active_param.version);

    if (ai_tune_telemetry_frame_type == 0U) {
        ai_tune_write_u32(ai_tune_tx_buffer, &index,
                (uint32_t)((int32_t)(telemetry->target_x_m * 1000.0f)));
        ai_tune_write_u32(ai_tune_tx_buffer, &index,
                (uint32_t)((int32_t)(telemetry->target_y_m * 1000.0f)));
        ai_tune_write_u32(ai_tune_tx_buffer, &index,
                (uint32_t)((int32_t)(telemetry->trajectory_x_m * 1000.0f)));
        ai_tune_write_u32(ai_tune_tx_buffer, &index,
                (uint32_t)((int32_t)(telemetry->trajectory_y_m * 1000.0f)));
        ai_tune_write_u32(ai_tune_tx_buffer, &index,
                (uint32_t)((int32_t)(telemetry->position_x_m * 1000.0f)));
        ai_tune_write_u32(ai_tune_tx_buffer, &index,
                (uint32_t)((int32_t)(telemetry->position_y_m * 1000.0f)));
        ai_tune_write_u32(ai_tune_tx_buffer, &index,
                (uint32_t)((int32_t)(telemetry->target_yaw_deg * 1000.0f)));
        ai_tune_write_u32(ai_tune_tx_buffer, &index,
                (uint32_t)((int32_t)(telemetry->yaw_deg * 1000.0f)));
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index,
                telemetry->error_x_local_m, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index,
                telemetry->error_y_local_m, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index,
                telemetry->velocity_x_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index,
                telemetry->velocity_y_mps, 1000.0f);
        ai_tune_tx_buffer[index++] = telemetry->position_reached;
        ai_tune_tx_buffer[index++] = ai_tune_guarded;
    } else if (ai_tune_telemetry_frame_type == 1U) {
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->plan_vx_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->plan_vy_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->plan_accel_x_mps2, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->plan_accel_y_mps2, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->accel_ff_vx_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->accel_ff_vy_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->com_ff_vx_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->com_ff_vy_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->pid_p_vx_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->pid_p_vy_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->pid_i_vx_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->pid_i_vy_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->pid_d_vx_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->pid_d_vy_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->pid_vx_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->pid_vy_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index,
                telemetry->velocity_feedback_vx_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index,
                telemetry->velocity_feedback_vy_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->command_vx_mps, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index, telemetry->command_vy_mps, 1000.0f);
    } else {
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index,
                telemetry->yaw_plan_rate_rad_s, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index,
                telemetry->yaw_p_rad_s, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index,
                telemetry->yaw_i_rad_s, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index,
                telemetry->yaw_d_rad_s, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index,
                telemetry->yaw_pid_rad_s, 1000.0f);
        ai_tune_write_scaled_i16(ai_tune_tx_buffer, &index,
                telemetry->command_omega_rad_s, 1000.0f);
        ai_tune_write_u16(ai_tune_tx_buffer, &index,
                (uint16_t)telemetry->motor_rpm[0]);
        ai_tune_write_u16(ai_tune_tx_buffer, &index,
                (uint16_t)telemetry->motor_rpm[1]);
        ai_tune_write_u16(ai_tune_tx_buffer, &index,
                (uint16_t)telemetry->motor_rpm[2]);
        ai_tune_write_u16(ai_tune_tx_buffer, &index,
                (uint16_t)telemetry->motor_rpm[3]);
        ai_tune_tx_buffer[index++] = telemetry->position_reached;
        ai_tune_tx_buffer[index++] = ai_tune_guarded;
    }
    ai_tune_tx_buffer[3] = (uint8_t)(index - 6U);
    ai_tune_telemetry_frame_type = (uint8_t)((ai_tune_telemetry_frame_type + 1U) % 3U);
    crc = ai_tune_crc16(&ai_tune_tx_buffer[2], index - 2U);
    ai_tune_write_u16(ai_tune_tx_buffer, &index, crc);
    HAL_UART_Transmit_DMA(&huart5, ai_tune_tx_buffer, index);
#else
    (void)telemetry;
#endif
}

uint8_t ai_tune_take_start(float *target_x_mm, float *target_y_mm,
        float *max_speed_mps)
{
    if ((target_x_mm == NULL) || (target_y_mm == NULL) ||
            (max_speed_mps == NULL) || (ai_tune_start_pending == 0U)) {
        return 0U;
    }
    *target_x_mm = ai_tune_start_x_mm;
    *target_y_mm = ai_tune_start_y_mm;
    *max_speed_mps = ai_tune_start_speed_mps;
    ai_tune_start_pending = 0U;
    return 1U;
}
