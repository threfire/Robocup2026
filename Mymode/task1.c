/**
 ******************************************************************************
 * @file    task1.c
 * @brief   任务一状态机
 ******************************************************************************
 */

#include "task1.h"
#include "control.h"
#include "motion.h"
#include "servo.h"
#include "vision.h"
#include "QAma.h"
#include <string.h>

/** 任务一路径点，单位 mm。 */
static const Point2D_t task1_targets[] = {
        {-650.0f, 255.25f, 0.0f},
        {-416.8f, 941.44f, -54.0f},
        {-211.12f, 660.19f, -54.0f},
        {354.75f, 898.77f, -68.0f},
        {471.22f, 615.22f, -68.0f},
        {974.56f, 686.34f, -68.0f},
        {650.0f, 250.25f, 0.0f},
        {1021.66f + 75.0f, 348.24f, -145.0f},
        {270.0f-25.0f, 1710.0f + 20.0f, -90.0f},
        {0.0f-20.0f, 1695.0f + 25.0f, -90.0f},
        {-270.0f-20.0f, 1715.0f + 25.0f, -90.0f},
        {0.0f, -20.0f, -90.0f},

};

/** 伸缩环 PWM 位置表。 */
const int Telescopic_ring[] = {320, 750, 750};
static uint8_t task1_vision_timeout;
static uint8_t task1_fourth_color_state;
static servo_smooth_move_t task1_qrcode_servo_move;
static uint8_t task1_qrcode_servo_move_started;

/** 切换物料转盘到指定槽位。 */
void Move_to_turnplate(uint8_t num) {
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Material[num - 1]);
}


/** 按照二维码结果选择物料转盘槽位。 */
static uint8_t task1_select_material_turnplate(uint8_t material_index) {
	uint8_t i;

	if (tasknum == 1U) {
		if (qanum == 0U || qanum > 16U || material_index == 0U ||
		        material_index > 5U || !is_hasgetcolor) {
			return 0U;
		}

		for (i = 0U; i < 5U; i++) {
			if (task1_scheme[qanum - 1U][material_index - 1U] ==
			        detected_colors[i]) {
				Move_to_turnplate(i + 1U);
				return (uint8_t)(i + 1U);
			}
		}
		return 0U;
	}

	if (tasknum == 2U && qanum > 0U && qanum <= 6U &&
	        material_index > 0U && material_index <= 3U) {
		for (i = 0U; i < 3U; i++) {
			if (task2_scheme[qanum - 1U][i] == material_index) {
				uint8_t target_slot = (uint8_t)(i + 3U);
				Move_to_turnplate(target_slot);
				return target_slot;
			}
		}
	}

	return 0U;
}
static void task1_next_step(void);
static void task1_finish(void);
/** 下发任务一路径点的位姿目标。 */
static uint8_t task1_move_target(
        uint8_t target_index, float y_offset_mm, float yaw_deg);
static void task1_go_first_qrcode(void);
static void task1_prepare_material_turnplate(void);
static void task1_go_first_material_point(void);
/** 前往物料点并处理颜色识别。 */
static void task1_go_material_point(
        uint8_t target_index, uint8_t material_index);
static void task1_wait_vision(uint8_t point_index);
static void task1_relative_action(uint8_t servo_after, uint32_t delay_after_ms);
static void task1_go_second_qrcode(void);
static void task1_go_trophy_transition(void);
static void task1_pick_trophies(void);
static void task1_go_second_podium(void);
static void task1_go_champion_podium(void);
static void task1_go_third_podium(void);
/** 前往奖台并在接近时预置舵机。 */
static void task1_go_podium_point(uint8_t target_index,
        uint8_t arrive_servo, uint8_t prepare_servo,
        float prepare_tolerance_m);
static void task1_go_home(void);

/** 初始化任务一状态机和舵机缓动状态。 */
void start_task1(void) {

	task1_state.active = 1;
	task1_state.current_step = 1;
	task1_state.completed = 0;
	task1_state.step_start = HAL_GetTick();
	task1_vision_timeout = 0U;
	task1_fourth_color_state = 0U;
	PositionControl_SetMaxSpeed(1.80f);
	memset(&task1_qrcode_servo_move, 0, sizeof(task1_qrcode_servo_move));
	task1_qrcode_servo_move_started = 0U;
	is_hasgetcolor = 0U;
}

/** 按当前步骤执行任务一流程。 */
void run_task1(void) {
	Update_Materialcolor();

	if (!task1_state.active) {
		return;
	}

	switch (task1_state.current_step) {
	case 1: task1_go_first_qrcode(); break;
	case 2: task1_prepare_material_turnplate(); break;
	case 11: task1_go_first_material_point(); break;
	case 12: task1_wait_vision(1); break;
	case 13: task1_relative_action(0, 0U); break;
	case 14: task1_go_material_point(2, 2); break;
	case 15: task1_wait_vision(2); break;
	case 16: task1_relative_action(0, 0U); break;
	case 17: task1_go_material_point(3, 3); break;
	case 18: task1_wait_vision(3); break;
	case 19: task1_relative_action(0, 0U); break;
	case 20: task1_go_material_point(4, 4); break;
	case 21: task1_wait_vision(4); break;
	case 22: task1_relative_action(0, 0U); break;
	case 23: task1_go_material_point(5, 5); break;
	case 24: task1_wait_vision(5); break;
	case 25: task1_relative_action(0, 0U); break;
	case 26: task1_go_second_qrcode(); break;
	case 27: task1_go_trophy_transition(); break;
	case 28: task1_pick_trophies(); break;
	case 31: task1_go_second_podium(); break;
	case 32: task1_wait_vision(8); break;
	case 33: task1_relative_action(3, 100U); break;
	case 34: task1_go_champion_podium(); break;
	case 35: task1_wait_vision(9); break;
	case 36: task1_relative_action(3, 0U); break;
	case 37: task1_go_third_podium(); break;
	case 38: task1_wait_vision(10); break;
	case 39: task1_relative_action(0, 0U); break;
	case 40: task1_go_home(); break;
	default: task1_finish(); break;
	}
}
/** 返回任务一运行状态。 */
uint8_t is_task1_active(void) {
	return task1_state.active;
}






/** 切换到下一任务步骤并记录进入时间。 */
static void task1_next_step(void) {
	task1_state.current_step++;
	task1_state.step_start = HAL_GetTick();
}


/** 结束任务一。 */
static void task1_finish(void) {
	task1_state.active = 0;
	task1_state.completed = 1;
}


static uint8_t task1_move_target(
        uint8_t target_index, float y_offset_mm, float yaw_deg) {
	return move_to_pose_target(task1_targets[target_index].x,
	        task1_targets[target_index].y + y_offset_mm, yaw_deg);
}


/** 接近二维码点时平滑下放舵机。 */
static void task1_qrcode_servo_slowdown(void) {
	if (!task1_qrcode_servo_move_started && IsPositionReached(0.60f)) {
		servo_smooth_move_start(&task1_qrcode_servo_move, &htim14,
		        TIM_CHANNEL_1, 2460U, 1800U, 600);
		task1_qrcode_servo_move_started = 1U;
	}

	servo_smooth_move_update(&task1_qrcode_servo_move);
}


/** 前往第一个二维码点。 */
static void task1_go_first_qrcode(void) {

	EnableAttitudeStabilization(1);
	SetTargetYaw(0.0f);
	Move_to_turnplate(1);
	if (!task1_qrcode_servo_move_started) {
		Servo_Action_Group(1);
	}
	servo_init();
	tasknum = 1;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[1]);
	if (task1_move_target(0, 0.0f, 0.0f)) {
		PositionControl_SetMaxSpeed(1.95f);
		Servo_Action_Group(2);
		if (qanum == 0U) {
			return;
		}

		task1_next_step();
		HAL_Delay (50);
		return;
	}

	task1_qrcode_servo_slowdown();
}


/** 开环转动并识别物料颜色。 */
static void task1_prepare_material_turnplate(void) {
	Servo_Action_Group(2);
	PositionControl_Enable(0);
	EnableAttitudeStabilization(0);

	SetTargetVelocity(-0.8f, 0.0f, -3.121f);
	HAL_Delay(300);
	Move_to_turnplate(2);
	HAL_Delay(600);

	SetTargetVelocity(-0.8f, 0.0f, -1.55f);
	HAL_Delay(200);
	Move_to_turnplate(3);
	HAL_Delay(550);

	Move_to_turnplate(4);
	SetTargetVelocity(-0.8f, 0.0f, -1.924f);
	HAL_Delay(600);
	Get_Materialcolor(0U);
	HAL_Delay(300);
	Move_to_turnplate(5);
	SetTargetVelocity(-0.8f, 0.0f, -1.730);
	HAL_Delay(300);
	Get_Materialcolor(1U);
	HAL_Delay(100);


	Move_to_turnplate(6);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[1]);
	motion_reset();
	HAL_Delay(300);
	task1_state.current_step = 11;
	task1_state.step_start = HAL_GetTick();
}


/** 前往第一个物料点。 */
static void task1_go_first_material_point(void) {
	EnableAttitudeStabilization(1);
	Get_Materialcolor(2U);
	is_hasgetcolor = 1U;
	task1_go_material_point(1, 1);
	HAL_Delay (100);
}


static void task1_go_material_point(
        uint8_t target_index, uint8_t material_index) {
	if (target_index == 1U) {
		if ((task1_fourth_color_state == 0U) &&
		        IsPositionReached(0.8f)) {
			Move_to_turnplate(7U);
			task1_fourth_color_state = 1U;
		}

		if ((task1_fourth_color_state == 1U) &&
		        IsPositionReached(0.2f)) {
			Get_Materialcolor(3U);
			Get_Materialcolor(4U);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[0]);
			__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, TROPHY_PLACE_PWM);
			task1_fourth_color_state = 2U;
		}
	}

	if (task1_move_target(target_index, 0.0f,
	        task1_targets[target_index].yaw_deg)) {
		if ((target_index == 1U) && (task1_fourth_color_state != 2U)) {
			return;
		}

		task1_next_step();
	}
}


/** 等待视觉结果或视觉超时。 */
static void task1_wait_vision(uint8_t point_index) {
	task1_vision_timeout = (uint8_t)!vision_wait_for_uart(point_index);
	task1_next_step();
}


/** 执行视觉校正后的取放动作。 */
static void task1_relative_action(uint8_t servo_after, uint32_t delay_after_ms) {
	uint8_t skip_pos_move =
	        (task1_state.current_step == 13U ||
	         task1_state.current_step == 19U);

	if (task1_vision_timeout) {
		task1_vision_timeout = 0U;
	} else if (!vision_center(TOLERANCE, TOLERANCE,
	        max_min_error)) {
		return;
	}

	if ((task1_state.current_step == 33U) ||
	        (task1_state.current_step == 36U) ||
	        (task1_state.current_step == 39U)) {
		static const uint8_t podium_table_index[3] = {2U, 0U, 1U};
		uint8_t podium_order =
		        (uint8_t)((task1_state.current_step - 33U) / 3U);
		uint8_t material_index = 0U;
		if (qanum > 0U && qanum <= 6U && podium_order < 3U) {
			material_index = (uint8_t)task2_scheme[qanum - 1U]
			        [podium_table_index[podium_order]];
		}
		uint8_t target_slot =
		        task1_select_material_turnplate(material_index);
		if (target_slot != 0U) {
			uint8_t slot_distance = (uint8_t)(6U - target_slot);
			HAL_Delay((uint32_t)slot_distance * 300U);
		}
	}

	if (task1_state.current_step >= 13U &&
	        task1_state.current_step <= 25U) {
		uint8_t material_index =
		        (uint8_t)(5U - (task1_state.current_step - 13U) / 3U);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[1]);
		__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1,
		        control_ctx.material_pwm[material_index - 1U]);
		HAL_Delay(1900- (material_index- 1) * 300);
	}

	if (!skip_pos_move) {
		POS_move();
	}
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[0]);

	if (task1_state.current_step != 25U){
		__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, TROPHY_PLACE_PWM);
	}else{
		Move_to_turnplate(5);
	}

	if (servo_after != 0U) {
		Servo_Action_Group(servo_after);
	}
	if (delay_after_ms > 0U) {
		HAL_Delay(delay_after_ms);
	}
	task1_next_step();
}


/** 前往第二个二维码点。 */
static void task1_go_second_qrcode(void) {
	tasknum = 2;
	if (task1_move_target(6, 0.0f, 0.0f)) {
		Move_to_turnplate(5);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[1]);
		task1_next_step();
	}
}


/** 前往奖杯取放过渡点。 */
static void task1_go_trophy_transition(void) {
	if (task1_move_target(7U, 0.0f, task1_targets[7U].yaw_deg)) {
		task1_next_step();
	}
}


/** 执行奖杯连续拾取动作。 */
static void task1_pick_trophies(void) {
	PositionControl_Enable(0);
	EnableAttitudeStabilization(0);

	SetTargetVelocity(-0.8f, 0.0f, 1.85f);
	HAL_Delay(300);
	Move_to_turnplate(4);
	HAL_Delay(300);
	SetTargetVelocity(-0.8f, 0.0f, 1.64f);
	HAL_Delay(400);

	Move_to_turnplate(3);
		HAL_Delay(500);

	HAL_Delay(250);

	motion_reset();
	HAL_Delay(100);
	task1_state.current_step = 31;
	task1_state.step_start = HAL_GetTick();
}


/** 前往亚军台。 */
static void task1_go_second_podium(void) {
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[0]);
	task1_go_podium_point(8, 6, 5, 0.6f);
}


/** 前往冠军台。 */
static void task1_go_champion_podium(void) {
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[0]);
	task1_go_podium_point(9, 4, 0, 0.0f);
}


/** 前往季军台。 */
static void task1_go_third_podium(void) {
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[0]);
	task1_go_podium_point(10, 7, 0, 0.0f);
}


static void task1_go_podium_point(uint8_t target_index,
        uint8_t arrive_servo, uint8_t prepare_servo,
        float prepare_tolerance_m) {
	if (task1_move_target(target_index, 0.0f, -90.0f)) {
		Servo_Action_Group(arrive_servo);
		task1_next_step();
		return;
	}

	if (prepare_servo != 0U && IsPositionReached(prepare_tolerance_m)) {
		Servo_Action_Group(prepare_servo);
		__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, TROPHY_PLACE_PWM);
	}
}


/** 返回起始点并结束任务。 */
static void task1_go_home(void) {
	raceh = POSITION_FINE_REACHED_TOLERANCE_M;
	if (task1_move_target(11, 0.0f, -90.0f)) {
		task1_finish();
		return;
	}
	PositionControl_SetMaxSpeed(IsPositionReached(TASK1_HOME_APPROACH_DISTANCE_M) ?
	        TASK1_HOME_APPROACH_SPEED_MPS : TASK1_HOME_CRUISE_SPEED_MPS);
}
