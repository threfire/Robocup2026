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
	float linear_speed_mps;
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
	volatile float base_omega_rad_s;
	float omega_command_rad_s;
};

/** step 2 五圆柱目标圆弧。 */
static const task1_arc_config_t task1_material_arc = {
	.center_x_mm = TASK1_MATERIAL_ARC_CENTER_X_MM,
	.center_y_mm = TASK1_MATERIAL_ARC_CENTER_Y_MM,
	.radius_mm = TASK1_MATERIAL_ARC_RADIUS_MM,
	.direction = TASK1_MATERIAL_ARC_DIRECTION,
	.linear_speed_mps = TASK1_MATERIAL_ARC_SPEED_MPS,
	.base_omega_rad_s = TASK1_MATERIAL_ARC_BASE_OMEGA_RAD_S,
};

/** step 28 奖杯目标圆弧。 */
static const task1_arc_config_t task1_trophy_arc = {
	.center_x_mm = TASK1_TROPHY_ARC_CENTER_X_MM,
	.center_y_mm = TASK1_TROPHY_ARC_CENTER_Y_MM,
	.radius_mm = TASK1_TROPHY_ARC_RADIUS_MM,
	.direction = TASK1_TROPHY_ARC_DIRECTION,
	.linear_speed_mps = TASK1_TROPHY_ARC_SPEED_MPS,
	.base_omega_rad_s = TASK1_TROPHY_ARC_BASE_OMEGA_RAD_S,
};

/** 任务一路径点，单位 mm。 */
static const Point2D_t task1_targets[] = {
        {-650.0f, 255.25f, 0.0f},
        {-449.36f - 10.0f, 985.96f+ 5.0f, -54.0f},
        {-243.68f - 10.0f, 704.72f+ 5.0f, -54.0f},
        {336.69f - 10.0f, 949.89f+ 5.0f, -68.0f},
        {450.51f - 10.0f, 668.22f+ 5.0f, -68.0f},
        {953.85f - 5.0f, 737.46f+ 5.0f, -68.0f},
        {630.0f, 300.25f, 0.0f},
        {1021.66f + 75.0f, 348.24f, -144.0f},
        {270.0f-25.0f, 1816.29f- 40.0f, -90.0f},
        {0.0f-25.0f, 1816.29f - 38.0f, -90.0f},
        {-270.0f-25.0f, 1816.29f - 10.0f, -90.0f},
        {0.0f, 0.0f, -90.0f},
        {0.0f, 200.25f, 0.0f},

};

/** 伸缩环 PWM 位置表。 */
const int Telescopic_ring[] = {1500, 1890, 1340, 1800, 2000};
static uint8_t task1_vision_timeout;
static servo_smooth_move_t task1_qrcode_servo_move;
static uint8_t task1_qrcode_servo_group1_set;
static uint8_t task1_qrcode_servo_group2_set;
/** 到达二维码点后的等待起始时间，单位 ms。 */
static uint32_t task1_qrcode_wait_start_ms;
static uint8_t task1_turnplate_last_slot = 0xFFU;
static int32_t task1_turnplate_absolute_pulses;
static uint8_t task1_podium_prepare_done;
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
	task1_arc_control.base_omega_rad_s = config->base_omega_rad_s;
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
	float base_omega_rad_s;
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
	base_omega_rad_s = task1_arc_control.base_omega_rad_s;
	omega_rad_s = base_omega_rad_s +
	        task1_arc_control.pid_output_rad_s;
	task1_arc_control.omega_command_rad_s = omega_rad_s;

	if (task1_arc_control.pid.integral > TASK1_ARC_PID_INTEGRAL_LIMIT) {
		task1_arc_control.pid.integral = TASK1_ARC_PID_INTEGRAL_LIMIT;
	} else if (task1_arc_control.pid.integral <
	        -TASK1_ARC_PID_INTEGRAL_LIMIT) {
		task1_arc_control.pid.integral = -TASK1_ARC_PID_INTEGRAL_LIMIT;
	}

	if (task1_arc_control.active != 0U) {
		SetTargetVelocity(config->linear_speed_mps, 0.0f, omega_rad_s);
	}
}

/** 动态设置圆弧闭环基础角速度。 */
void task1_arc_set_omega(float base_omega_rad_s) {
	task1_arc_control.base_omega_rad_s = base_omega_rad_s;
	__DMB();
}

/** 同步转盘当前槽位，不下发电机命令。 */
void turnplate_sync_position(uint8_t num) {
	if (num >= TURNPLATE_POSITION_COUNT) {
		return;
	}

	task1_turnplate_absolute_pulses = (int32_t)
	        (TurnplatePosition[num] % TURNPLATE_PULSES_PER_REV);
	task1_turnplate_last_slot = num;
}

/**
 * @brief 按单圈最短路径切换物料转盘槽位。
 *
 * 位置表只保存 0～3200 的正脉冲位置。函数分别计算顺时针和逆时针
 * 所需的正脉冲数，选择较短方向后通过 EMM 的 CW/CCW 方向位下发。
 */
void Move_to_turnplate(uint8_t num) {
	uint32_t target_pulses;
	uint32_t current_pulses;
	uint32_t cw_pulses;
	uint32_t ccw_pulses;
	uint32_t absolute_pulses;
	int32_t target_absolute_pulses;
	uint8_t direction;

	if (num >= TURNPLATE_POSITION_COUNT ||
	        task1_turnplate_last_slot == num) {
		return;
	}

	target_pulses = TurnplatePosition[num] % TURNPLATE_PULSES_PER_REV;
	current_pulses = (uint32_t)((task1_turnplate_absolute_pulses %
	        (int32_t)TURNPLATE_PULSES_PER_REV +
	        (int32_t)TURNPLATE_PULSES_PER_REV) %
	        (int32_t)TURNPLATE_PULSES_PER_REV);
	/* 两个方向均使用单圈取模，结果始终为正脉冲数。 */
	cw_pulses = (target_pulses + TURNPLATE_PULSES_PER_REV -
	        current_pulses) % TURNPLATE_PULSES_PER_REV;
	if (cw_pulses == 0U) {
		task1_turnplate_last_slot = num;
		return;
	}
	ccw_pulses = TURNPLATE_PULSES_PER_REV - cw_pulses;

	/* 最短弧生成单圈等价的绝对目标，方向位表示绝对位置的正负。 */
	if (cw_pulses <= ccw_pulses) {
		target_absolute_pulses = task1_turnplate_absolute_pulses +
		        (int32_t)cw_pulses;
	} else {
		target_absolute_pulses = task1_turnplate_absolute_pulses -
		        (int32_t)ccw_pulses;
	}
	direction = (target_absolute_pulses >= 0) ?
	        TURNPLATE_CW : TURNPLATE_CCW;
	absolute_pulses = (target_absolute_pulses >= 0) ?
	        (uint32_t)target_absolute_pulses :
	        (uint32_t)(-target_absolute_pulses);

	Emm_V5_Pos_Control(TURNPLATE_MOTOR_ADDR, direction,
	        TURNPLATE_SPEED_RPM, TURNPLATE_ACCEL,
	        absolute_pulses, 1U, 0U);
	task1_turnplate_absolute_pulses = target_absolute_pulses;
	task1_turnplate_last_slot = num;
}


/** 按二维码目标值查找对应物料槽位并转动转盘。 */
static uint8_t task1_select_material_turnplate(uint8_t material_index) {
	uint8_t i;

	if (tasknum == 1U) {
		if (qanum == 0U || qanum > 16U ||
		        material_index == 0U || material_index > 5U) {
			return 0U;
		}

		/* 二维码表给出 A～E 点的目标颜色，识别数组下标对应转盘槽位。 */
		for (i = 0U; i < 5U; i++) {
			if (task1_scheme[qanum - 1U][material_index - 1U] ==
			        detected_colors[i]) {
				Move_to_turnplate(i + 1U);
				return (uint8_t)(i + 1U);
			}
		}
	}

	if (tasknum == 2U && qanum > 0U && qanum <= 6U) {
		if (material_index == 0U || material_index > 3U) {
			return 0U;
		}
		for (i = 0U; i < 3U; i++) {
			if (task2_scheme[qanum - 1U][i] == material_index) {
				Move_to_turnplate(i + 1U);
				return (uint8_t)(i + 1U);
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
/** 前往奖台并按任务时序执行提前舵机动作。 */
static void task1_go_podium_point(uint8_t target_index,
        uint8_t arrive_servo, uint8_t prepare_servo);
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
	task1_qrcode_servo_group1_set = 0U;
	task1_qrcode_servo_group2_set = 0U;
	task1_qrcode_wait_start_ms = 0U;
	task1_podium_prepare_done = 0U;
	servo_init();
	Servo_Action_Group(0U);
	qanum = 0U;
	task1_arc_control_stop();
	motion_path_stop();
	is_hasgetcolor = 0U;
}

/** 按当前步骤执行任务一流程。 */
void run_task1(void) {
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
	task1_podium_prepare_done = 0U;
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


/** 接近二维码点时按距离切换舵机动作。 */
static void task1_qrcode_servo_slowdown(void) {
	if (!task1_qrcode_servo_group1_set && IsPositionReached(0.4f)) {
		Servo_Action_Group(1);
		task1_qrcode_servo_group1_set = 1U;
	}

	if (task1_qrcode_servo_group1_set &&
	        !task1_qrcode_servo_group2_set && IsPositionReached(0.35f)) {
		servo_smooth_move_start(&task1_qrcode_servo_move, &htim14,
		        TIM_CHANNEL_1, 2200U, 1790U, 1200U);
		task1_qrcode_servo_group2_set = 1U;
	}

	servo_smooth_move_update(&task1_qrcode_servo_move);
}


/** 前往第一个二维码点。 */
static void task1_go_first_qrcode(void) {
	uint8_t target_reached;

	EnableAttitudeStabilization(1);
	SetTargetYaw(0.0f);
	Move_to_turnplate(1);
	tasknum = 1;
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[1]);

	if (task1_qrcode_wait_start_ms == 0U) {
		target_reached = task1_move_target(0, 0.0f, 0.0f);
		task1_qrcode_servo_slowdown();
		if (!target_reached) {
			return;
		}

		PositionControl_SetMaxSpeed(POSITION_TRAJECTORY_CRUISE_SPEED_MPS);
		task1_qrcode_wait_start_ms = HAL_GetTick();
	} else {
		servo_smooth_move_update(&task1_qrcode_servo_move);
	}

	if (task1_qrcode_servo_move.active) {
		return;
	}

	if (((qanum == 0U) || (qanum > 16U)) &&
	        ((HAL_GetTick() - task1_qrcode_wait_start_ms) <
	                TASK1_QRCODE_TIMEOUT_MS)) {
		return;
	}
	task1_qrcode_wait_start_ms = 0U;
	task1_next_step();
//	task1_state.current_step = 26;
	HAL_Delay(50U);
}


/** 开环转动并识别物料颜色。 */
static void task1_prepare_material_turnplate(void) {
	PositionControl_Enable(0U);
	EnableAttitudeStabilization(0U);
	task1_arc_control_start(&task1_material_arc);
	Servo_Action_Group(2);
	osDelay(300U);
	Move_to_turnplate(2U);
	
	osDelay(650);
	Move_to_turnplate(3U);
	
	task1_arc_set_omega(-1.60);
	
	osDelay(650);
	Get_Materialcolor(0U);
//	osDelay(300);
	Move_to_turnplate(4U);
	
	task1_arc_set_omega(-1.65);

	osDelay(650);
	Get_Materialcolor(1U);
//	osDelay(250);
	Move_to_turnplate(5U);
	
	osDelay(650);
	Get_Materialcolor(2U);
//	osDelay(150);
	Move_to_turnplate(6U);
	osDelay(400U);
		task1_arc_control_stop();
	motion_reset();
	
	osDelay(250U);
	
	
	Get_Materialcolor(3U);
	Get_Materialcolor(4U);
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[1]);
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[2]);
	task1_state.current_step = 11U;
	task1_state.step_start = HAL_GetTick();
}


/** 前往第一个物料点。 */
static void task1_go_first_material_point(void) {
	EnableAttitudeStabilization(1);
	Move_to_turnplate(7U);
	/* 五个槽位颜色已在转盘扫描阶段记录，此处直接使用扫描结果。 */
	is_hasgetcolor = 1U;
	task1_go_material_point(1, 1);
}


static void task1_go_material_point(
        uint8_t target_index, uint8_t material_index) {
	(void)material_index;
	PositionControl_SetMaxSpeed(TASK1_five_CRUISE_SPEED_MPS);
	if (task1_move_target(target_index, 0.0f,
	        task1_targets[target_index].yaw_deg)) {
		
		Move_to_turnplate(0U);
		osDelay (500);
		__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[2]);
		osDelay (200);
		task1_next_step();
	}
}


/** 等待视觉结果或视觉超时。 */
static void task1_wait_vision(uint8_t point_index) {
	/* 奖台视觉识别从 8 号点开始，等待坐标期间保持补光。 */
	if (point_index >= 8U) {
		IsLight = 1U;
	}
	task1_vision_timeout = (uint8_t)!vision_wait_for_uart(point_index);
	if (task1_vision_timeout && point_index >= 8U) {
		IsLight = 0U;
	}
	task1_next_step();
}


/**
 * @brief 执行视觉校正完成后的取放动作。
 *
 * 视觉坐标接收成功时持续调用居中控制，居中完成后再执行转盘、伸缩环和舵机动作。
 * 视觉坐标接收超时时跳过居中等待，使任务状态机能够继续运行。
 *
 * @param servo_after 取放完成后执行的舵机动作组编号，0 表示不执行动作组。
 * @param delay_after_ms 舵机动作组下发后的等待时间，单位 ms。
 */
static void task1_relative_action(uint8_t servo_after, uint32_t delay_after_ms) {
	/* 接收超时只在本次动作中生效，清零后直接进入机械取放流程。 */
	if (task1_vision_timeout) {
		task1_vision_timeout = 0U;
	} else if (!vision_center(TOLERANCE, TOLERANCE,
	        max_min_error)) {
		/* 居中控制尚未完成，保持当前状态机步骤，等待下一周期继续校正。 */
		return;
	}

	/* 视觉流程结束后，同一周期伸出机构并将目标物料转到取放位置。 */
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[4]);
	if (task1_state.current_step >= 13U &&
	        task1_state.current_step <= 25U) {
		uint8_t material_index =
		        (uint8_t)((task1_state.current_step - 10U) / 3U);
		uint8_t target_slot =
		        task1_select_material_turnplate(material_index);
		if (target_slot != 0U) {
			uint8_t slot_distance =
			        (target_slot <= TURNPLATE_SLOT_COUNT / 2U) ?
			        target_slot : TURNPLATE_SLOT_COUNT - target_slot;
			HAL_Delay((uint32_t)slot_distance * 600U);
		}
	}

	/* 奖台放置步骤根据二维码方案确定本次奖杯对应的转盘槽位。 */
	if ((task1_state.current_step == 33U) ||
	        (task1_state.current_step == 36U) ||
	        (task1_state.current_step == 39U)) {
		/* 三个奖台依次放置 B、A、C 奖杯。 */
		static const uint8_t podium_material_index[3] = {2U, 1U, 3U};
		uint8_t podium_order =
		        (uint8_t)((task1_state.current_step - 33U) / 3U);
		uint8_t material_index = 0U;
		if (podium_order < 3U) {
			material_index = podium_material_index[podium_order];
		}
		/* 物料编号映射为实际槽位，并立即下发转盘位置控制命令。 */
		uint8_t target_slot =
		        task1_select_material_turnplate(material_index);
		if (target_slot != 0U) {
			uint8_t slot_distance = (uint8_t)(6U - target_slot);
			/* 按跨越槽位数等待电机运动，为后续取放动作保留机械到位时间。 */
			HAL_Delay((uint32_t)slot_distance * 600U);
		}
	}

	/* 物料区取放前先收回伸缩环，避免伸出状态下执行竖直取放。 */
	if (task1_state.current_step >= 13U &&
	        task1_state.current_step <= 25U) {
		__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[1]);
		/* 等待伸缩环离开物料，防止紧接着的升降动作产生干涉。 */
		HAL_Delay(150U);
	}
	/* 伸缩环移动到取放位置，随后执行机构完成一次竖直取放。 */
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[3]);
	POS_move();
//	/* E 点结束后直接转向第二二维码所需槽位，其余步骤回到零号槽位。 */
//	if (task1_state.current_step != 25U) {
//		Move_to_turnplate(0U);
//	}

	/* 非零动作组用于奖台放置后的舵机姿态切换。 */
	if (servo_after != 0U) {
		Servo_Action_Group(servo_after);
	}
	/* 等待舵机到达目标姿态，再允许状态机进入下一步骤。 */
	if (delay_after_ms > 0U) {
		HAL_Delay(delay_after_ms);
	}
	/* 所有取放指令均已下发，推进到下一任务步骤。 */
	task1_next_step();
}


/** 前往第二个二维码点。 */
static void task1_go_second_qrcode(void) {
	Move_to_turnplate(3U);
	
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
	Move_to_turnplate(3U);
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

	osDelay(700U);
	Move_to_turnplate(2U);
	osDelay(900U);

	Move_to_turnplate(1U);
	osDelay(1200U);
	
	Move_to_turnplate(0U);
	osDelay(200);
	task1_arc_control_stop();
	motion_reset();
	task1_state.current_step = 31U;
	task1_state.step_start = HAL_GetTick();
	task1_podium_prepare_done = 0U;
}


/** 前往亚军台。 */
static void task1_go_second_podium(void) {
	PositionControl_SetMaxSpeed(0.8f);
	task1_go_podium_point(8, 6, 5);
}


/** 前往冠军台。 */
static void task1_go_champion_podium(void) {
	PositionControl_SetMaxSpeed(0.6f);
	task1_go_podium_point(9, 4, 3);
}


/** 前往季军台。 */
static void task1_go_third_podium(void) {
	task1_go_podium_point(10, 7, 5);
}


void task1_podium_prepare_update(void) {
	if (task1_state.active && task1_state.current_step == 31U &&
	        !task1_podium_prepare_done &&
	        (HAL_GetTick() - task1_state.step_start >= 500U)) {
		Servo_Action_Group(5U);
		Move_to_turnplate(6U);
		task1_podium_prepare_done = 1U;
	}
}


static void task1_go_podium_point(uint8_t target_index,
        uint8_t arrive_servo, uint8_t prepare_servo) {
	/* 亚军台由 1 ms 实时任务延时触发，其余奖台立即执行提前动作。 */
	if (!task1_podium_prepare_done && target_index != 8U) {
		Servo_Action_Group(prepare_servo);
		Move_to_turnplate(6U);
		task1_podium_prepare_done = 1U;
	}

	if (task1_move_target(target_index, 0.0f, -90.0f)) {
		Servo_Action_Group(arrive_servo);
		__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Telescopic_ring[2]);
		task1_next_step();
		return;
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
