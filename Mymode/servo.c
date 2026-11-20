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
	
		if (num == 0U)
		{
      __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 2200);
		}
		else if (num == 1U)
    {
        __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, 1220);
        __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 2200);
    }
    else if (num == 2U)/* 放地上 */
    {
        __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, 1200);
        __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1790);
    }
    else if (num == 3U) /* 举起来（冠军台） */
    {
        __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, 1460);
        __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1700);
    }
    else if (num == 4U) /* 放平（冠军台） */
    {
        uint16_t pwm13_last = 1460;
        uint16_t pwm14_last = 1700;

        for (uint16_t i = 1U; i <= 100U; i++)
        {
            __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1,
                    pwm13_last - i * (pwm13_last - 1470) / 100);
            __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1,
                    pwm14_last - i * (pwm14_last - 1540) / 100);
            HAL_Delay(2U);
        }
    }
    else if (num == 5U) /* 举起来（亚军台） */
    {
        __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1, 1460);
        __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1720);
    }
    else if (num == 6U) /* 放平（亚军台） */
    {
        uint16_t pwm13_last = 1460;
        uint16_t pwm14_last = 1720;

        for (uint16_t i = 1U; i <= 200U; i++)
        {
            __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1,
                    pwm13_last - i * (pwm13_last - 1390U) / 200);
            __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1,
                    pwm14_last - i * (pwm14_last - 1590U) / 200);
            HAL_Delay(2U);
        }
    }
    else if (num == 7U) /* 放平（季军台） */
    {
        uint16_t pwm13_last = 1390;
        uint16_t pwm14_last = 1700;

        for (uint16_t i = 1U; i <= 150U; i++)
        {
            __HAL_TIM_SetCompare(&htim13, TIM_CHANNEL_1,
                    pwm13_last - i * (pwm13_last - 1200) / 150);
            __HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1,
                    pwm14_last - i * (pwm14_last - 1790) / 150);
            HAL_Delay(3U);
        }
    }
}
