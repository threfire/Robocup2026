#include "servo.h"

void servo_init(void)
{
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
}

void servo_smooth_move_start(servo_smooth_move_t *move,
        TIM_HandleTypeDef *htim, uint32_t channel, uint16_t start_pwm,
        uint16_t target_pwm, uint32_t duration_ms)
{
    if (move == NULL || htim == NULL)
    {
        return;
    }

    move->htim = htim;
    move->channel = channel;
    move->start_pwm = start_pwm;
    move->target_pwm = target_pwm;
    move->start_time_ms = HAL_GetTick();
    move->duration_ms = duration_ms;
    move->active = (duration_ms > 0U);

    __HAL_TIM_SetCompare(htim, channel, start_pwm);

    if (!move->active)
    {
        __HAL_TIM_SetCompare(htim, channel, target_pwm);
    }
}

void servo_smooth_move_update(servo_smooth_move_t *move)
{
    uint32_t elapsed_ms;
    float progress;
    float curve;
    uint16_t pwm;

    if (move == NULL || move->htim == NULL)
    {
        return;
    }

    if (!move->active)
    {
        __HAL_TIM_SetCompare(move->htim, move->channel, move->target_pwm);
        return;
    }

    elapsed_ms = HAL_GetTick() - move->start_time_ms;
    if (elapsed_ms >= move->duration_ms)
    {
        __HAL_TIM_SetCompare(move->htim, move->channel, move->target_pwm);
        move->active = 0U;
        return;
    }

    progress = (float)elapsed_ms / (float)move->duration_ms;
    /* 三次缓出曲线让 PWM 变化速率随时间逐步减小。 */
    curve = 1.0f - (1.0f - progress) * (1.0f - progress) *
            (1.0f - progress);
    pwm = (uint16_t)((float)move->start_pwm +
            ((float)move->target_pwm - (float)move->start_pwm) * curve);
    __HAL_TIM_SetCompare(move->htim, move->channel, pwm);
}

void Servo_Action_Group(uint8_t num)
{
    if (num == 1U)
    {
        __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, 1070U);
        __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 2460U);
    }
    else if (num == 2U)
    {
        __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, 1070U);
        __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1800U);
    }
    else if (num == 3U)
    {
        __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, 1460U);
        __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1880U);
    }
    else if (num == 4U)
    {
        uint16_t pwm13_last = 1460U;
        uint16_t pwm14_last = 1880U;

        for (uint16_t i = 1U; i <= 200U; i++)
        {
            __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1,
                    pwm13_last - i * (pwm13_last - 1380U) / 200U);
            __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1,
                    pwm14_last - i * (pwm14_last - 1520U) / 200U);
            HAL_Delay(2U);
        }
    }
    else if (num == 5U)
    {
        __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, 1390U);
        __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1860U);
    }
    else if (num == 6U)
    {
        uint16_t pwm13_last = 1390U;
        uint16_t pwm14_last = 1860U;

        for (uint16_t i = 1U; i <= 200U; i++)
        {
            __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1,
                    pwm13_last - i * (pwm13_last - 1260U) / 200U);
            __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1,
                    pwm14_last - i * (pwm14_last - 1700U) / 200U);
            HAL_Delay(2U);
        }
    }
    else if (num == 7U)
    {
        uint16_t pwm13_last = 1460U;
        uint16_t pwm14_last = 1680U;

        for (uint16_t i = 1U; i <= 200U; i++)
        {
            __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1,
                    pwm13_last - i * (pwm13_last - 1070U) / 200U);
            __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1,
                    pwm14_last - i * (pwm14_last - 1800U) / 200U);
            HAL_Delay(3U);
        }
    }
}
