/**
 ******************************************************************************
 * @file    task1.c
 * @brief   任务一状态机
 ******************************************************************************
 */

#include "task1.h"
#include "control.h"
#include "motion.h"
#include "odometer.h"
#include "servo.h"
#include "vision.h"
#include "QAma.h"
#include "cmsis_os.h"
#include <math.h>
#include <string.h>

/** 圆弧径向循迹配置。 */
typedef struct {
	float center_x_mm;
	float center_y_mm;
	float radius_mm;
	int8_t direction;
	float base_omega_rad_s;
} task1_arc_config_t;

/** 圆弧径向循迹运行状态。 */
struct task1_arc_control_s {
	PID_Controller pid;
	const task1_arc_config_t *config;
	volatile uint8_t active;
	float slot_x_mm;
	float slot_y_mm;
	float actual_radius_mm;
	float radial_error_m;
	float pid_output_rad_s;
	float omega_command_rad_s;
};

/** step 2 五圆柱目标圆弧。 */
static const task1_arc_config_t task1_material_arc = {
	.center_x_mm = TASK1_MATERIAL_ARC_CENTER_X_MM,
	.center_y_mm = TASK1_MATERIAL_ARC_CENTER_Y_MM,
	.radius_mm = TASK1_MATERIAL_ARC_RADIUS_MM,
	.direction = TASK1_MATERIAL_ARC_DIRECTION,
	.base_omega_rad_s = TASK1_MATERIAL_ARC_BASE_OMEGA_RAD_S,
};

/** step 28 奖杯目标圆弧。 */
static const task1_arc_config_t task1_trophy_arc = {
	.center_x_mm = TASK1_TROPHY_ARC_CENTER_X_MM,
	.center_y_mm = TASK1_TROPHY_ARC_CENTER_Y_MM,
	.radius_mm = TASK1_TROPHY_ARC_RADIUS_MM,
	.direction = TASK1_TROPHY_ARC_DIRECTION,
	.base_omega_rad_s = TASK1_TROPHY_ARC_BASE_OMEGA_RAD_S,
};

/** 任务一路径点，单位 mm。 */
static const Point2D_t task1_targets[] = {
        {-650.0f, 255.25f, 0.0f},
        {-449.36f, 985.96f, -54.0f},
        {-243.68f, 704.72f, -54.0f},
        {336.69f, 949.89f, -68.0f},
        {450.51f, 668.22f, -68.0f},
        {953.85f, 737.46f, -68.0f},
        {650.0f, 250.25f, 0.0f},
        {1021.66f + 75.0f, 348.24f, -145.0f},
        {270.0f-25.0f, 1816.29f, -90.0f},
        {0.0f-20.0f, 1816.29f, -90.0f},
        {-270.0f-20.0f, 1816.29f, -90.0f},
        {0.0f, 0.0f, -90.0f},

};

/** 伸缩环 PWM 位置表。 */
const int Telescopic_ring[] = {1500, 1800, 1340};
static uint8_t task1_vision_timeout;
static servo_smooth_move_t task1_qrcode_servo_move;
static uint8_t task1_qrcode_servo_move_started;
/** 到达二维码点后的等待起始时间，单位 ms。 */
static uint32_t task1_qrcode_wait_start_ms;
static uint8_t task1_turnplate_last_slot = 0xFFU;
task1_arc_control_t task1_arc_control;

/** 启动指定目标圆弧的径向 PID。 */
static void task1_arc_control_start(const task1_arc_config_t *config) {
	task1_arc_control.active = 0U;
	task1_arc_control.config = config;
	task1_arc_control.slot_x_mm = 0.0f;
	task1_arc_control.slot_y_mm = 0.0f;
	task1_arc_control.actual_radius_mm = 0.0f;
	task1_arc_control.radial_error_m = 0.0f;
	task1_arc_control.pid_output_rad_s = 0.0f;
	task1_arc_control.omega_command_rad_s = 0.0f;
	pid_init(&task1_arc_control.pid, TASK1_ARC_PID_KP,
	        TASK1_ARC_PID_KI, TASK1_ARC_PID_KD,
	        TASK1_ARC_PID_MAX_OMEGA_RAD_S);
	__DMB();
	task1_arc_control.active = 1U;
}

/** 停止圆弧径向 PID。 */
static void task1_arc_control_stop(void) {
	task1_arc_control.active = 0U;
	task1_arc_control.pid_output_rad_s = 0.0f;
	task1_arc_control.omega_command_rad_s = 0.0f;
	__DMB();
}

/** 在 1 ms 实时任务中更新圆弧径向 PID。 */
void task1_arc_control_update(float dt) {
	const task1_arc_config_t *config;
	float delta_x_mm;
	float delta_y_mm;
	float yaw_rad;
	float slot_x_mm;
	float slot_y_mm;
	float actual_radius_mm;
	float radial_error_m;
	float omega_rad_s;

	if ((task1_arc_control.active == 0U) || (dt <= 0.0f)) {
		return;
	}

	config = task1_arc_control.config;
	yaw_rad = odometer_ctx.pose.yaw_deg * 0.017453292519943295f;
	slot_x_mm = odometer_ctx.pose.pos_x_mm +
	        cosf(yaw_rad) * TASK1_ARC_SLOT_X_MM -
	        sinf(yaw_rad) * TASK1_ARC_SLOT_Y_MM;
	slot_y_mm = odometer_ctx.pose.pos_y_mm +
	        sinf(yaw_rad) * TASK1_ARC_SLOT_X_MM +
	        cosf(yaw_rad) * TASK1_ARC_SLOT_Y_MM;
	task1_arc_control.slot_x_mm = slot_x_mm;
	task1_arc_control.slot_y_mm = slot_y_mm;
	delta_x_mm = slot_x_mm - config->center_x_mm;
	delta_y_mm = slot_y_mm - config->center_y_mm;
	actual_radius_mm = sqrtf(delta_x_mm * delta_x_mm +
	        delta_y_mm * delta_y_mm);
	task1_arc_control.actual_radius_mm = actual_radius_mm;
	/* 方向符号将实际半径相对目标半径的误差映射到旋转方向。 */
	radial_error_m = (float)config->direction *
	        (actual_radius_mm - config->radius_mm) * 0.001f;
	task1_arc_control.radial_error_m = radial_error_m;
	task1_arc_control.pid_output_rad_s = pid_calc(
	        &task1_arc_control.pid, radial_error_m, dt);
	omega_rad_s = config->base_omega_rad_s +
	        task1_arc_control.pid_output_rad_s;
	task1_arc_control.omega_command_rad_s = omega_rad_s;

	if (task1_arc_control.pid.integral > TASK1_ARC_PID_INTEGRAL_LIMIT) {
		task1_arc_control.pid.integral = TASK1_ARC_PID_INTEGRAL_LIMIT;
	} else if (task1_arc_control.pid.integral <
	        -TASK1_ARC_PID_INTEGRAL_LIMIT) {
		task1_arc_control.pid.integral = -TASK1_ARC_PID_INTEGRAL_LIMIT;
	}

	if (task1_arc_control.active != 0U) {
		SetTargetVelocity(TASK1_ARC_SPEED_MPS, 0.0f, omega_rad_s);
	}
}

/** 切换物料转盘到指定槽位。 */
void Move_to_turnplate(uint8_t num) {
	if (num > TURNPLATE_SLOT_COUNT ||
	        task1_turnplate_last_slot == num) {
		return;
	}

	Emm_V5_Pos_Control(TURNPLATE_MOTOR_ADDR, TURNPLATE_DIRECTION,
	        TURNPLATE_SPEED_RPM, TURNPLATE_ACCEL,
	        TurnplatePosition[num], 1U, 0U);
	task1_turnplate_last_slot = num;
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
	task1_turnplate_last_slot = 0xFFU;

	task1_state.active = 1;
	task1_state.current_step = 1;
	task1_state.completed = 0;
	task1_state.step_start = HAL_GetTick();
	task1_vision_timeout = 0U;
	PositionControl_SetMaxSpeed(TASK1_START_CRUISE_SPEED_MPS);
	memset(&task1_qrcode_servo_move, 0, sizeof(task1_qrcode_servo_move));
	task1_qrcode_servo_move_started = 0U;
	task1_qrcode_wait_start_ms = 0U;
	qanum = 0U;
	task1_arc_control_stop();
	motion_path_stop();
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
	case 33: task1_relative_action(2, 100U); break;
	case 34: task1_go_champion_podium(); break;
	case 35: task1_wait_vision(9); break;
	case 36: task1_relative_action(2, 0U); break;
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
		        TIM_CHANNEL_1, 2460U, 1840U, 600);
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
		Servo_Action_Group(2);
	}
	servo_init();
	tasknum = 1;
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[1]);
	if (task1_qrcode_wait_start_ms == 0U) {
		if (!task1_move_target(0, 0.0f, 0.0f)) {
			return;
		}

		PositionControl_SetMaxSpeed(POSITION_TRAJECTORY_CRUISE_SPEED_MPS);
		task1_qrcode_wait_start_ms = HAL_GetTick();
	}

	if (((qanum == 0U) || (qanum > 16U)) &&
	        ((HAL_GetTick() - task1_qrcode_wait_start_ms) <
	                TASK1_QRCODE_TIMEOUT_MS)) {
		return;
	}

	task1_qrcode_wait_start_ms = 0U;
	Servo_Action_Group(2);
	task1_next_step();
	HAL_Delay(50U);
}


/** 开环转动并识别物料颜色。 */
static void task1_prepare_material_turnplate(void) {
	Servo_Action_Group(2);
	PositionControl_Enable(0U);
	EnableAttitudeStabilization(0U);
	task1_arc_control_start(&task1_material_arc);

	osDelay(300U);
	Move_to_turnplate(2U);
	
	osDelay(550);
	Move_to_turnplate(3U);
	
	
	osDelay(550);
	Get_Materialcolor(0U);
	Move_to_turnplate(4U);
	

	osDelay(550);
	Get_Materialcolor(1U);
	Move_to_turnplate(5U);
	
	osDelay(550);
	Get_Materialcolor(2U);
	Move_to_turnplate(6U);
	
	task1_arc_control_stop();
	motion_reset();
	osDelay(300U);
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[2]);
	Get_Materialcolor(3U);
	Get_Materialcolor(4U);
	Move_to_turnplate(0U);
	task1_state.current_step = 11U;
	task1_state.step_start = HAL_GetTick();
}


/** 前往第一个物料点。 */
static void task1_go_first_material_point(void) {
	EnableAttitudeStabilization(1);
	Get_Materialcolor(2U);
	is_hasgetcolor = 1U;
	task1_go_material_point(1, 1);
}


static void task1_go_material_point(
        uint8_t target_index, uint8_t material_index) {

	if (task1_move_target(target_index, 0.0f,
	        task1_targets[target_index].yaw_deg)) {
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
		__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[1]);
		Move_to_turnplate(material_index);
		HAL_Delay(1900- (material_index- 1) * 300);
	}

	if (!skip_pos_move) {
		POS_move();
	}
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[2]);

	Move_to_turnplate(0);

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
	if (tasknum != 2U) {
		tasknum = 2U;
		qanum = 0U;
		task1_qrcode_wait_start_ms = 0U;
	}

	if (task1_qrcode_wait_start_ms == 0U) {
		if (!task1_move_target(6, 0.0f, 0.0f)) {
			return;
		}

		task1_qrcode_wait_start_ms = HAL_GetTick();
	}

	if (((qanum == 0U) || (qanum > 6U)) &&
	        ((HAL_GetTick() - task1_qrcode_wait_start_ms) <
	                TASK1_QRCODE_TIMEOUT_MS)) {
		return;
	}

	task1_qrcode_wait_start_ms = 0U;
	Move_to_turnplate(5U);
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[1]);
	task1_next_step();
}


/** 前往奖杯取放过渡点。 */
static void task1_go_trophy_transition(void) {
	if (task1_move_target(7U, 0.0f, task1_targets[7U].yaw_deg)) {
		task1_next_step();
	}
}


/** 开环转动并拾取奖杯。 */
static void task1_pick_trophies(void) {
	PositionControl_Enable(0U);
	EnableAttitudeStabilization(0U);
	task1_arc_control_start(&task1_trophy_arc);

	osDelay(400U);
	Move_to_turnplate(4U);
	osDelay(600U);

	Move_to_turnplate(3U);
	osDelay(600U);

	task1_arc_control_stop();
	motion_reset();
	task1_state.current_step = 31U;
	task1_state.step_start = HAL_GetTick();
}


/** 前往亚军台。 */
static void task1_go_second_podium(void) {
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[2]);
	task1_go_podium_point(8, 2, 2, 0.6f);
}


/** 前往冠军台。 */
static void task1_go_champion_podium(void) {
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[2]);
	task1_go_podium_point(9, 2, 2, 0.0f);
}


/** 前往季军台。 */
static void task1_go_third_podium(void) {
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[2]);
	task1_go_podium_point(10, 2, 2, 0.0f);
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
		Move_to_turnplate(6U);
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
