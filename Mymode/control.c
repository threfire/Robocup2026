#include "control.h"
#include <math.h>
#include <string.h>
#include "lcd_spi_154.h"
#include "odometer.h"
#include "servo.h"
#include "graysensor.h"
#include "userkey.h"
#include "mxcam.h"
#include "QAma.h"
#include "ai_tune.h"

/* 控制模块运行状态统一收敛到 control_ctx，旧任务代码通过宏访问字段。 */
ControlContext_t control_ctx = {
        .race_tolerance_m = POSITION_REACHED_TOLERANCE_M,
        .material_pwm = { 2340, 2340-290*1, 2340-290*2, 2340-290*3, 2340-290*4 +10, 2340-290*5, 2340-290*6},
        .task_num = 1,
        .cam_mode = 2,
        .look_mode_control = 1,
};

/* 任务 1 路径点，单位 mm，坐标系与里程计输出一致。 */
static const Point2D_t task1_targets[] = {
        {-650.0f, 255.25f, 0.0f},         /* 索引 0：点 1，二维码 1。 */
        {-416.8f, 941.44f, -54.0f},       /* 索引 1：点 2，物料 a。 */
        {-211.12f, 660.19f, -54.0f},      /* 索引 2：点 3，物料 b。 */
        {354.75f, 898.77f, -68.0f},       /* 索引 3：点 4，物料 c。 */
        {471.22f, 615.22f, -68.0f},       /* 索引 4：点 5，物料 d。 */
        {974.56f, 686.34f, -68.0f},       /* 索引 5：点 6，物料 e。 */
        {650.0f, 250.25f, 0.0f},          /* 索引 6：点 7，二维码 2。 */
        {1021.66f + 75.0f, 348.24f, -145.0f},      /* 索引 7：点 8，二维码 2 至奖杯开环动作过渡位姿。 */
        {270.0f-25.0f, 1710.0f + 20.0f, -90.0f},        /* 索引 8：点 9，亚军奖台。 */
        {0.0f-20.0f, 1695.0f + 25.0f, -90.0f},          /* 索引 9：点 10，冠军奖台。 */
        {-270.0f-20.0f, 1715.0f + 25.0f, -90.0f},         /* 索引 10：点 11，季军奖台。 */
        {0.0f, -20.0f, -90.0f},             /* 索引 11：起始点。 */

};

const int Telescopic_ring[] = {320, 750, 750};
static void MotionState_Reset(void);
static uint8_t task1_vision_timeout;
static uint32_t cam_stabilize_start_time;
static uint8_t task1_fourth_color_state;
static servo_smooth_move_t task1_qrcode_servo_move;
static uint8_t task1_qrcode_servo_move_started;

void Setmaixcam_mode(uint8_t ch) {
	cammode = ch;
}
void Move_to_turnplate(uint8_t num) {
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Material[num - 1]);
}

/**
 * @brief 根据二维码任务顺序选择对应颜色的转盘位置。
 *
 * 任务一由二维码颜色序列匹配物料扫描结果，任务二由奖杯序号匹配二维码序列。
 *
 * @param material_index 当前需要的物料或奖杯序号，从 1 开始。
 */
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
void Pos_moveconctrl(uint8_t dir, uint16_t vel, uint16_t clk) {
	/* EMM 位置模式用于局部前后微动，clk 单位为脉冲。 */
	if (dir == 1) {
		Emm_V5_Pos_Control(1, 1, vel, CHASSIS_POS_FORWARD_ACCEL,
		        clk, 0, 1);

		Emm_V5_Pos_Control(2, 1, vel, CHASSIS_POS_FORWARD_ACCEL,
		        clk, 0, 1);

		Emm_V5_Pos_Control(3, 0, vel, CHASSIS_POS_FORWARD_ACCEL,
		        clk, 0, 1);

		Emm_V5_Pos_Control(4, 0, vel, CHASSIS_POS_FORWARD_ACCEL,
		        clk, 0, 1);
		HAL_Delay(5);
		Emm_V5_Synchronous_motion(0);
	}
	if (dir == 0) {
		Emm_V5_Pos_Control(1, 0, vel, CHASSIS_POS_BACKWARD_ACCEL,
		        clk, 0, 1);

		Emm_V5_Pos_Control(2, 0, vel, CHASSIS_POS_BACKWARD_ACCEL,
		        clk, 0, 1);

		Emm_V5_Pos_Control(3, 1, vel, CHASSIS_POS_BACKWARD_ACCEL,
		        clk, 0, 1);

		Emm_V5_Pos_Control(4, 1, vel, CHASSIS_POS_BACKWARD_ACCEL,
		        clk, 0, 1);
		HAL_Delay(5);
		Emm_V5_Synchronous_motion(0);
	}
}
void POS_move(void) {
	/* EMM 位置模式后退固定脉冲，动作结束后恢复实时速度链路。 */
	PositionControl_Enable(0U);
	PoseCoordination_Enable(0U);
	EnableAttitudeStabilization(0U);
	stop();
	Mecanum_SendVelocity();
	HAL_Delay(5U);
	Is_vel_mode = 0;
	Pos_moveconctrl(0U, TASK1_EMM_BACKWARD_SPEED_RPM,
	        TASK1_EMM_BACKWARD_PULSES);
	HAL_Delay(TASK1_EMM_BACKWARD_WAIT_MS);
	MotionState_Reset();
	Is_vel_mode = 1;
}
static float position_plan_speed_mps = 0.0f;
static float position_plan_accel_mps2 = 0.0f;
static float position_plan_cruise_speed_mps =
        POSITION_PLANNER_CRUISE_SPEED_MPS;
static float position_feedback_last_vel_x_mps;
static float position_feedback_last_vel_y_mps;
static float position_feedback_accel_x_mps2;
static float position_feedback_accel_y_mps2;
static uint8_t position_feedback_accel_valid;
static ChassisConfig_t chassis_config;
static float k_omega;
static float attitude_last_pid_output;
static uint8_t attitude_pid_output_valid;

static float NormalizeYawError(float error) {
	while (error > 180.0f)
		error -= 360.0f;
	while (error < -180.0f)
		error += 360.0f;
	return error;
}

static float ClampFloat(float value, float min_value, float max_value) {
	if (value < min_value)
		return min_value;
	if (value > max_value)
		return max_value;
	return value;
}

static void PositionPlanner_Reset(void) {
	position_plan_speed_mps = 0.0f;
	position_plan_accel_mps2 = 0.0f;
	position_feedback_accel_valid = 0U;
	position_feedback_accel_x_mps2 = 0.0f;
	position_feedback_accel_y_mps2 = 0.0f;
}

static void PositionAccelerationFeedback_Update(float dt) {
	float vel_x_mps = odometer_ctx.motion.vel_x_mps;
	float vel_y_mps = odometer_ctx.motion.vel_y_mps;
	float accel_x_mps2;
	float accel_y_mps2;

	if (!position_feedback_accel_valid) {
		position_feedback_last_vel_x_mps = vel_x_mps;
		position_feedback_last_vel_y_mps = vel_y_mps;
		position_feedback_accel_valid = 1U;
		return;
	}

	accel_x_mps2 = (vel_x_mps - position_feedback_last_vel_x_mps) / dt;
	accel_y_mps2 = (vel_y_mps - position_feedback_last_vel_y_mps) / dt;
	position_feedback_accel_x_mps2 += POSITION_ACCEL_FEEDBACK_FILTER_ALPHA *
	        (accel_x_mps2 - position_feedback_accel_x_mps2);
	position_feedback_accel_y_mps2 += POSITION_ACCEL_FEEDBACK_FILTER_ALPHA *
	        (accel_y_mps2 - position_feedback_accel_y_mps2);
	position_feedback_last_vel_x_mps = vel_x_mps;
	position_feedback_last_vel_y_mps = vel_y_mps;
}

/**
 * @brief 计算偏航加速度前馈。
 *
 * 车体转动惯量由质量和轮距计算，角加速度使用里程计滤波后的陀螺仪微分值。
 *
 * @param pid_output 姿态 PID 输出，单位 rad/s。
 * @param dt 控制周期，单位 s。
 * @return 偏航前馈角速度，单位 rad/s。
 */
static float AttitudeFeedforward_Update(float pid_output, float dt) {
	float desired_accel_rad_s2;
	float actual_accel_rad_s2;
	float yaw_inertia_kg_m2;
	float torque_nm;
	float torque_to_omega;

	if (!attitude_pid_output_valid) {
		attitude_last_pid_output = pid_output;
		attitude_pid_output_valid = 1U;
		return 0.0f;
	}

	desired_accel_rad_s2 =
	        (pid_output - attitude_last_pid_output) / dt;
	desired_accel_rad_s2 = ClampFloat(desired_accel_rad_s2,
	        -ATTITUDE_FF_MAX_ACCEL_RAD_S2, ATTITUDE_FF_MAX_ACCEL_RAD_S2);
	actual_accel_rad_s2 = odometer_ctx.motion.yaw_accel_dps2 *
	        0.017453292519943295f;
	yaw_inertia_kg_m2 = CHASSIS_MASS_KG *
	        (chassis_config.wheel_dist_x * chassis_config.wheel_dist_x +
	         chassis_config.wheel_dist_y * chassis_config.wheel_dist_y) /
	        3.0f + CHASSIS_MASS_KG *
	        (CHASSIS_CENTER_OF_MASS_X_M * CHASSIS_CENTER_OF_MASS_X_M +
	         CHASSIS_CENTER_OF_MASS_Y_M * CHASSIS_CENTER_OF_MASS_Y_M);
	torque_nm = yaw_inertia_kg_m2 * (desired_accel_rad_s2 +
	        ATTITUDE_ACCEL_FEEDBACK_GAIN *
	                (desired_accel_rad_s2 - actual_accel_rad_s2));
	torque_to_omega = POSITION_FORCE_TO_SPEED_FF_MPS_PER_N /
	        (k_omega * k_omega);
	attitude_last_pid_output = pid_output;

	return ClampFloat(torque_nm * torque_to_omega,
	        -ATTITUDE_FF_MAX_OMEGA_RAD_S, ATTITUDE_FF_MAX_OMEGA_RAD_S);
}

static float PositionSpeedPlan_Update(float distance_error_m, float dt) {
	/* 距离误差生成前馈速度，减速距离约束保证接近目标点时速度可收敛。 */
	float cruise_speed = ClampFloat(position_plan_cruise_speed_mps, 0.0f,
	        position_control.pid_x.max_output);
	float brake_speed = sqrtf(2.0f * POSITION_COMMAND_BRAKE_DECEL_MPS2 *
	        distance_error_m);
	float target_speed = ClampFloat(cruise_speed, 0.0f, brake_speed);
	float target_accel = 0.0f;
	float accel_delta;

	if (distance_error_m <= raceh) {
		PositionPlanner_Reset();
		return 0.0f;
	}

	if (distance_error_m <= POSITION_PLANNER_NEAR_TARGET_DISTANCE_M) {
		PositionPlanner_Reset();
		return 0.0f;
	}

	/* 加速度变化率限幅用于抑制电流模式下的突变力矩。 */
	if (position_plan_speed_mps > target_speed) {
		target_accel = -POSITION_PLANNER_MAX_DECEL_MPS2;
	} else if (position_plan_speed_mps < target_speed) {
		target_accel = POSITION_PLANNER_MAX_ACCEL_MPS2;
	}

	accel_delta = target_accel - position_plan_accel_mps2;
	accel_delta =
	        ClampFloat(accel_delta, -POSITION_PLANNER_MAX_JERK_MPS3 * dt,
	                POSITION_PLANNER_MAX_JERK_MPS3 * dt);
	position_plan_accel_mps2 += accel_delta;
	position_plan_accel_mps2 = ClampFloat(position_plan_accel_mps2,
	        -POSITION_PLANNER_MAX_DECEL_MPS2,
	        POSITION_PLANNER_MAX_ACCEL_MPS2);

	position_plan_speed_mps += position_plan_accel_mps2 * dt;
	position_plan_speed_mps =
	        ClampFloat(position_plan_speed_mps, 0.0f, cruise_speed);

	if (position_plan_speed_mps > target_speed) {
		position_plan_speed_mps = target_speed;
		position_plan_accel_mps2 = 0.0f;
	}

	if (position_plan_speed_mps < POSITION_PLANNER_MIN_SPEED_MPS &&
	        target_speed > POSITION_PLANNER_MIN_SPEED_MPS) {
		position_plan_speed_mps = POSITION_PLANNER_MIN_SPEED_MPS;
	}

	return position_plan_speed_mps;
}

void PositionControl_Init(void) {
	/* 位置环只控制 x/y 平移，yaw 由姿态环单独闭环。 */
	PID_Init(&position_control.pid_x, POSITION_PID_DEFAULT_KP, 0.00f,
	        POSITION_PID_DEFAULT_KD, HIGHESTSPEED);

	PID_Init(&position_control.pid_y, POSITION_PID_DEFAULT_KP, 0.00f,
	        POSITION_PID_DEFAULT_KD, HIGHESTSPEED);

	position_control.is_enabled = 0;
	position_control.target_x = 0.0f;
	position_control.target_y = 0.0f;
	position_control.last_update_time = HAL_GetTick();
	position_plan_cruise_speed_mps = POSITION_PLANNER_CRUISE_SPEED_MPS;

	is_position_control_enabled = 0;
}

void PositionControl_Enable(uint8_t enable) {
	position_control.is_enabled = enable;
	is_position_control_enabled = enable;

	if (!enable) {
		/* 关闭位置环时清积分并停止底盘，避免下一次目标继承旧误差。 */
		position_control.pid_x.integral = 0.0f;
		position_control.pid_x.prev_error = 0.0f;
		position_control.pid_y.integral = 0.0f;
		position_control.pid_y.prev_error = 0.0f;
		PositionPlanner_Reset();

		stop();
	} else {
		position_control.last_update_time = HAL_GetTick();
	}
}

void PositionControl_SetMaxSpeed(float max_speed_mps) {
	max_speed_mps = ClampFloat(max_speed_mps, POSITION_PLANNER_MIN_SPEED_MPS,
	        POSITION_MAX_LINEAR_SPEED_MPS);
	position_control.pid_x.max_output = max_speed_mps;
	position_control.pid_y.max_output = max_speed_mps;
	position_plan_cruise_speed_mps = max_speed_mps;
}

void SetTargetPosition(float x, float y) {
	if (ai_tune_apply_pending()) {
		const ai_tune_param_t *param = ai_tune_get_active();

		position_control.pid_x.Kp = param->kp;
		position_control.pid_x.Kd = param->kd;
		position_control.pid_y.Kp = param->kp;
		position_control.pid_y.Kd = param->kd;
	}
	ai_tune_segment_start();
	/* 目标点单位 mm，来自任务路径表；控制计算内部统一换算为 m。 */
	Is_vel_mode = 1;
	position_control.target_x = x;
	position_control.target_y = y;
	target_velocity.omega = 0.0f;
	position_control.pid_x.integral = 0.0f;
	position_control.pid_x.prev_error = 0.0f;
	position_control.pid_y.integral = 0.0f;
	position_control.pid_y.prev_error = 0.0f;
	PositionPlanner_Reset();
}

uint8_t IsPositionReached(float tolerance_xy) {
	/* 里程计输出为世界系 mm，位置到达判断使用世界系欧氏距离。 */
	float current_x_m = odometer_ctx.pose.pos_x_mm * 0.001f;
	float current_y_m = odometer_ctx.pose.pos_y_mm * 0.001f;

	float error_x = position_control.target_x * 0.001f - current_x_m;
	float error_y = position_control.target_y * 0.001f - current_y_m;

	float distance_error = sqrtf(error_x * error_x + error_y * error_y);

	if (distance_error <= tolerance_xy) {
		return 1;
	}

	return 0;
}

static uint8_t IsPositionControlFinished(float tolerance_xy) {
	return IsPositionReached(tolerance_xy) &&
	       (position_plan_speed_mps <= POSITION_PLANNER_STOP_SPEED_MPS);
}

void UpdatePositionControl(float dt) {
	if (!position_control.is_enabled || dt <= 0) {
		return;
	}

	/* 当前 yaw 将世界系位置误差旋转到底盘局部坐标系。 */
	float current_yaw_rad = odometer_get_yaw_deg() * 3.1415926f / 180.0f;

	float current_x_global = odometer_ctx.pose.pos_x_mm * 0.001f;
	float current_y_global = odometer_ctx.pose.pos_y_mm * 0.001f;
	float target_x_global = position_control.target_x * 0.001f;
	float target_y_global = position_control.target_y * 0.001f;

	float error_x_global = target_x_global - current_x_global;
	float error_y_global = target_y_global - current_y_global;
	float distance_error = sqrtf(error_x_global * error_x_global +
	                             error_y_global * error_y_global);
	control_ctx.error_x_global_m = error_x_global;
	control_ctx.error_y_global_m = error_y_global;

	/* 世界系误差旋转到车体系，x 正向前，y 正向右。 */
	float error_x_local = error_x_global * cosf(current_yaw_rad) +
	                      error_y_global * sinf(current_yaw_rad);
	float error_y_local = -error_x_global * sinf(current_yaw_rad) +
	                      error_y_global * cosf(current_yaw_rad);
	control_ctx.error_x_local_m = error_x_local;
	control_ctx.error_y_local_m = error_y_local;

	/* 规划速度沿目标方向进入速度命令，加速度前馈补偿动态跟踪误差。 */
	float plan_speed = PositionSpeedPlan_Update(distance_error, dt);
	PositionAccelerationFeedback_Update(dt);
	const ai_tune_param_t *param = ai_tune_get_active();
	float plan_vx = 0.0f;
	float plan_vy = 0.0f;
	float feedforward_vx = 0.0f;
	float feedforward_vy = 0.0f;
	float accel_x_local = position_feedback_accel_x_mps2 *
	                              cosf(current_yaw_rad) +
	                      position_feedback_accel_y_mps2 *
	                              sinf(current_yaw_rad);
	float accel_y_local = -position_feedback_accel_x_mps2 *
	                              sinf(current_yaw_rad) +
	                      position_feedback_accel_y_mps2 *
	                              cosf(current_yaw_rad);
	float yaw_rate_rad_s = odometer_ctx.motion.yaw_rate_dps *
	        0.017453292519943295f;
	float yaw_accel_rad_s2 = odometer_ctx.motion.yaw_accel_dps2 *
	        0.017453292519943295f;
	/* 质心偏置在转动时产生切向和向心加速度，叠加到车体中心的平移期望加速度。 */
	float com_accel_x_local = -yaw_accel_rad_s2 * CHASSIS_CENTER_OF_MASS_Y_M -
	        yaw_rate_rad_s * yaw_rate_rad_s * CHASSIS_CENTER_OF_MASS_X_M;
	float com_accel_y_local = yaw_accel_rad_s2 * CHASSIS_CENTER_OF_MASS_X_M -
	        yaw_rate_rad_s * yaw_rate_rad_s * CHASSIS_CENTER_OF_MASS_Y_M;
	if (distance_error > raceh) {
		plan_vx = error_x_local * plan_speed / distance_error;
		plan_vy = error_y_local * plan_speed / distance_error;
		float desired_accel_x_local = error_x_local *
		        position_plan_accel_mps2 / distance_error;
		float desired_accel_y_local = error_y_local *
		        position_plan_accel_mps2 / distance_error;
		float force_x_n = CHASSIS_MASS_KG *
		        (desired_accel_x_local + com_accel_x_local +
		                param->accel_feedback_gain *
		                (desired_accel_x_local - accel_x_local));
		float force_y_n = CHASSIS_MASS_KG *
		        (desired_accel_y_local + com_accel_y_local +
		                param->accel_feedback_gain *
		                (desired_accel_y_local - accel_y_local));
		feedforward_vx = force_x_n * param->force_to_speed_ff;
		feedforward_vy = force_y_n * param->force_to_speed_ff;
	}
	float feedback_vx =
	        PID_Update(&position_control.pid_x, error_x_local, dt);
	float feedback_vy =
	        PID_Update(&position_control.pid_y, error_y_local, dt);
	float feedback_speed =
	        sqrtf(feedback_vx * feedback_vx + feedback_vy * feedback_vy);

	if (feedback_speed > POSITION_FEEDBACK_MAX_SPEED_MPS) {
		/* 反馈速度限幅使规划速度主导长距离运动，减少近点抖动。 */
		float feedback_ratio =
		        POSITION_FEEDBACK_MAX_SPEED_MPS / feedback_speed;
		feedback_vx *= feedback_ratio;
		feedback_vy *= feedback_ratio;
	}
	if ((distance_error > raceh) &&
	        (distance_error <= POSITION_STATIC_FRICTION_ENABLE_DISTANCE_M) &&
	        (feedback_speed < param->static_friction_speed_mps)) {
		/* 近点误差沿目标方向叠加最小有效速度，克服底盘静摩擦。 */
		float static_friction_scale =
		        param->static_friction_speed_mps / distance_error;
		feedforward_vx += error_x_local * static_friction_scale;
		feedforward_vy += error_y_local * static_friction_scale;
	}

	float vx_local = plan_vx + feedforward_vx + feedback_vx;
	float vy_local = plan_vy + feedforward_vy + feedback_vy;
	float linear_speed = sqrtf(vx_local * vx_local + vy_local * vy_local);
	float command_speed_limit = sqrtf(2.0f *
	        POSITION_COMMAND_BRAKE_DECEL_MPS2 * distance_error);
	command_speed_limit = ClampFloat(command_speed_limit, 0.0f,
	        position_plan_cruise_speed_mps);
	if (linear_speed > command_speed_limit) {
		/* 制动包络约束规划、前馈和反馈速度总和。 */
		float ratio = command_speed_limit / linear_speed;
		vx_local *= ratio;
		vy_local *= ratio;
		linear_speed = command_speed_limit;
	}

	vx_local *= pose_coordination.sync_scale;
	vy_local *= pose_coordination.sync_scale;
	if (ai_tune_guard_update(distance_error, linear_speed, dt)) {
		vx_local = 0.0f;
		vy_local = 0.0f;
	}
	{
		ai_tune_telemetry_t telemetry = {
		        .time_ms = HAL_GetTick(),
		        .target_x_m = target_x_global,
		        .target_y_m = target_y_global,
		        .position_x_m = current_x_global,
		        .position_y_m = current_y_global,
		        .error_x_local_m = error_x_local,
		        .error_y_local_m = error_y_local,
		        .velocity_x_mps = odometer_ctx.motion.vel_x_mps,
		        .velocity_y_mps = odometer_ctx.motion.vel_y_mps,
		        .plan_accel_mps2 = position_plan_accel_mps2,
		        .accel_x_mps2 = accel_x_local,
		        .accel_y_mps2 = accel_y_local,
		        .feedforward_vx_mps = feedforward_vx,
		        .feedforward_vy_mps = feedforward_vy,
		        .feedback_vx_mps = feedback_vx,
		        .feedback_vy_mps = feedback_vy,
		        .command_vx_mps = vx_local,
		        .command_vy_mps = vy_local,
		        .motor_rpm = {motor1.speed_rpm, motor2.speed_rpm,
		                motor3.speed_rpm, motor4.speed_rpm},
		        .position_reached = IsPositionControlFinished(raceh),
		};

		ai_tune_send_telemetry(&telemetry);
	}
	/* 世界系误差经 yaw 旋转后，局部 vx/vy 按同名轴传入麦轮逆解。 */
	SetTargetLinearVelocity(vx_local, vy_local);
	if (ai_tune_is_guarded()) {
		PositionControl_Enable(0U);
		return;
	}

	if (IsPositionControlFinished(raceh)) {
		testflag = 1;
		PositionPlanner_Reset();
	}
}

void Example_PositionControl(void) {
	SetTargetPosition(1.0f, 0.5f);

	PositionControl_Enable(1);

	while (!IsPositionReached(0.02f)) {
		HAL_Delay(10);
	}
}

float GetAttitudeCorrection(void) {
	return attitude_correction;
}
void AttitudeStabilization_Init(void) {
	/* 姿态环使用里程计融合后的世界系 yaw，输出叠加到麦轮 omega。 */
	PID_Init(&attitude_control.yaw_pid, ATTITUDE_PID_DEFAULT_KP,
	        ATTITUDE_PID_DEFAULT_KI, ATTITUDE_PID_DEFAULT_KD,
	        ATTITUDE_PID_MAX_OUTPUT);
	attitude_pid_output_valid = 0U;

	isPosture_correction = 0;

	attitude_control.target_yaw = 0.0f;
	attitude_control.is_enabled = 0;
	attitude_control.last_update_time = HAL_GetTick();
}

void SetTargetYaw(float target) {
	/* 目标 yaw 变化时清空积分项，避免角度切换瞬间冲击。 */
	if (fabsf(NormalizeYawError(attitude_control.target_yaw - target)) <
	        0.001f) {
		return;
	}
	attitude_control.target_yaw = target;

	attitude_control.yaw_pid.integral = 0.0f;
	attitude_control.yaw_pid.prev_error = 0.0f;
	attitude_pid_output_valid = 0U;
}
uint8_t IsYawReached(float target_yaw, float tolerance_degrees) {
	float current_yaw = odometer_get_yaw_deg();
	float error = target_yaw - current_yaw;

	/* yaw 误差归一化到 [-180, 180]，跨零点时保持最短旋转方向。 */
	while (error > 180.0f)
		error -= 360.0f;
	while (error < -180.0f)
		error += 360.0f;

	return (fabsf(error) <= tolerance_degrees);
}
void EnableAttitudeStabilization(uint8_t enable) {
	if (attitude_control.is_enabled == enable) {
		return;
	}
	attitude_control.is_enabled = enable;

	if (enable) {
		/* 启用姿态环时先锁住当前 yaw，再由任务层设置新目标。 */
		attitude_control.target_yaw = odometer_get_yaw_deg();
		attitude_control.last_update_time = HAL_GetTick();

		attitude_control.yaw_pid.integral = 0.0f;
		attitude_control.yaw_pid.prev_error = 0.0f;
		attitude_pid_output_valid = 0U;
	}
}

void UpdateAttitudeStabilization(float dt) {
	if (!attitude_control.is_enabled || dt <= 0) {
		attitude_correction = 0.0f;
		attitude_pid_output_valid = 0U;
		return;
	}

	float current_yaw = odometer_get_yaw_deg();

	float error = attitude_control.target_yaw - current_yaw;

	/* 角度环绕处理和死区共同限制小误差下的来回修正。 */
	while (error > 180.0f)
		error -= 360.0f;
	while (error < -180.0f)
		error += 360.0f;
	control_ctx.yaw_error_deg = error;

	if (abs_float(error) < DEAD_ZONE) {
		error = 0.0f;
		attitude_control.yaw_pid.integral = 0.0f;
		attitude_correction = 0.0f;
		attitude_pid_output_valid = 0U;
		return;
	}

	float pid_output = PID_Update_angle(&attitude_control.yaw_pid, error, dt);
	attitude_correction = ClampFloat(pid_output +
	        AttitudeFeedforward_Update(pid_output, dt),
	        -ATTITUDE_PID_MAX_OUTPUT, ATTITUDE_PID_MAX_OUTPUT);
	attitude_correction *= pose_coordination.sync_scale;
}

void PoseCoordination_Init(void) {
	/* 位姿协同用于位置环和姿态环同时收敛，接近目标时同步降速。 */
	pose_coordination.is_enabled = 0;
	pose_coordination.sync_scale = 1.0f;
	pose_coordination.slow_distance_m = POSE_SYNC_SLOW_DISTANCE_M;
	pose_coordination.slow_yaw_deg = POSE_SYNC_SLOW_YAW_DEG;
	pose_coordination.min_scale = POSE_SYNC_MIN_SCALE;
}

void PoseCoordination_Enable(uint8_t enable) {
	pose_coordination.is_enabled = enable;
	pose_coordination.sync_scale = 1.0f;
}

void SetTargetPose(float x, float y, float yaw) {
	/* 任务层调用统一位姿目标：先给 x/y，再开启姿态目标。 */
	SetTargetPosition(x, y);
	PoseCoordination_Enable(1);
	PositionControl_Enable(1);
	EnableAttitudeStabilization(1);
	SetTargetYaw(yaw);
}

uint8_t IsPoseReached(float tolerance_xy, float tolerance_yaw_degrees) {
	return IsPositionReached(tolerance_xy) &&
	       IsYawReached(attitude_control.target_yaw, tolerance_yaw_degrees);
}

void UpdatePoseCoordination(float dt) {
	float distance_error_m;
	float yaw_error_deg;
	float pos_scale;
	float yaw_scale;

	if (!pose_coordination.is_enabled || dt <= 0.0f) {
		pose_coordination.sync_scale = 1.0f;
		return;
	}

	distance_error_m =
	        sqrtf(powf(position_control.target_x * 0.001f -
	                              odometer_ctx.pose.pos_x_mm * 0.001f,
	                      2.0f) +
	                powf(position_control.target_y * 0.001f -
	                                odometer_ctx.pose.pos_y_mm * 0.001f,
	                        2.0f));
	yaw_error_deg = NormalizeYawError(
	        attitude_control.target_yaw - odometer_get_yaw_deg());

	/* 位置误差和角度误差分别换算为缩放系数，取较小值保证两环同步到点。 */
	pos_scale = distance_error_m / pose_coordination.slow_distance_m;
	yaw_scale = fabsf(yaw_error_deg) / pose_coordination.slow_yaw_deg;
	pos_scale = ClampFloat(pos_scale, pose_coordination.min_scale, 1.0f);
	yaw_scale = ClampFloat(yaw_scale, pose_coordination.min_scale, 1.0f);
	pose_coordination.sync_scale =
	        ClampFloat((pos_scale < yaw_scale) ? pos_scale : yaw_scale,
	                pose_coordination.min_scale, 1.0f);
}

uint8_t move_to_pose_target(float x, float y, float yaw) {
	static uint8_t pose_target_set = 0;

	if (!pose_target_set) {
		/* 非阻塞位姿目标只在首次进入时下发，后续循环等待到达。 */
		SetTargetPose(x, y, yaw);
		pose_target_set = 1;
		return 0;
	}

	if (IsPoseReached(raceh, POSE_YAW_TOLERANCE_DEG)) {
		stop();
		PoseCoordination_Enable(0);
		PositionControl_Enable(0);
		EnableAttitudeStabilization(0);
		pose_target_set = 0;
		return 1;
	}

	return 0;
}
uint8_t RotateToYaw(float target_yaw, float tolerance) {

	if (!is_rotating) {
		SetTargetYaw(target_yaw);
		is_rotating = 1;
		return 0;
	}

	if (IsYawReached(target_yaw, tolerance)) {
		is_rotating = 0;
		return 1;
	}

	return 0;
}
void CamStabilization_Init(void) {
	/* 视觉稳定环用图像误差生成局部平移校正量。 */
	PID_Init_cam(
	        &cam_stabilization.pid_x, 0.15f, 0.0000f, 0.0f, 0.001f, 20.0f);

	PID_Init_cam(
	        &cam_stabilization.pid_y, 0.15f, 0.0000f, 0.0f, 0.001f, 20.0f);

	cam_stabilization.is_enabled = 0;
	cam_correction_x = 0.0f;
	cam_correction_y = 0.0f;
	lookmode_centrl = 0;
	is_cam_stabilizing = 0;
	Is_coordgot = 0;
}

void CamStabilization_Enable(uint8_t enable) {
	cam_stabilization.is_enabled = enable;

	/* 每次切入视觉环清除上一段误差历史，首个校正点从新帧坐标起算。 */
	cam_stabilization.pid_x.integral = 0.0f;
	cam_stabilization.pid_x.prev_error = 0.0f;
	cam_stabilization.pid_y.integral = 0.0f;
	cam_stabilization.pid_y.prev_error = 0.0f;
	cam_correction_x = 0.0f;
	cam_correction_y = 0.0f;
	IsLight = enable ? 1U : 0U;
}

void UpdateCamStabilization(float x_err, float y_err, float dt) {
	if (!cam_stabilization.is_enabled || dt <= 0) {
		cam_correction_x = 0.0f;
		cam_correction_y = 0.0f;
		return;
	}

	float actual_x_err = (abs_float(x_err) > CAM_DEAD_ZONE) ? x_err : 0.0f;

	float actual_y_err = (abs_float(y_err) > CAM_DEAD_ZONE) ? y_err : 0.0f;

	/* 相机误差直接进入 PID，输出在运动学解算中叠加到底盘局部速度。 */
	cam_correction_x =
	        PID_Update_cam(&cam_stabilization.pid_x, actual_x_err, dt);

	cam_correction_y =
	        PID_Update_cam(&cam_stabilization.pid_y, actual_y_err, dt);
}

static ErrorQueue_t error_queue = {0};

void ErrorQueue_Init(void) {
	error_queue.index = 0;
	error_queue.count = 0;
	error_queue.is_full = 0;
	memset(error_queue.errors_x, 0, sizeof(error_queue.errors_x));
	memset(error_queue.errors_y, 0, sizeof(error_queue.errors_y));
}

void ErrorQueue_Push(int error_x, int error_y) {
	error_queue.errors_x[error_queue.index] = error_x;
	error_queue.errors_y[error_queue.index] = error_y;

	error_queue.index = (error_queue.index + 1) % ERROR_QUEUE_SIZE;

	if (!error_queue.is_full) {
		error_queue.count++;
		if (error_queue.count >= ERROR_QUEUE_SIZE) {
			error_queue.is_full = 1;
		}
	}
}

void ErrorQueue_GetAverage(float *avg_x, float *avg_y) {
	int sum_x = 0, sum_y = 0;
	uint8_t valid_count =
	        error_queue.is_full ? ERROR_QUEUE_SIZE : error_queue.count;

	if (valid_count == 0) {
		*avg_x = 0;
		*avg_y = 0;
		return;
	}

	for (int i = 0; i < valid_count; i++) {
		sum_x += abs_int(error_queue.errors_x[i]);
		sum_y += abs_int(error_queue.errors_y[i]);
	}

	*avg_x = (float)sum_x / valid_count;
	*avg_y = (float)sum_y / valid_count;
}
void ErrorQueue_GetFluctuate(int *fluct_x, int *fluct_y) {
	int max_x = -CAM_ERROR_ABS_LIMIT, min_x = CAM_ERROR_ABS_LIMIT;
	int max_y = -CAM_ERROR_ABS_LIMIT, min_y = CAM_ERROR_ABS_LIMIT;
	uint8_t valid_count =
	        error_queue.is_full ? ERROR_QUEUE_SIZE : error_queue.count;

	if (valid_count == 0) {
		*fluct_x = 0;
		*fluct_y = 0;
		return;
	}

	for (int i = 0; i < valid_count; i++) {
		if (error_queue.errors_x[i] > max_x)
			max_x = error_queue.errors_x[i];
		if (error_queue.errors_x[i] < min_x)
			min_x = error_queue.errors_x[i];
		if (error_queue.errors_y[i] > max_y)
			max_y = error_queue.errors_y[i];
		if (error_queue.errors_y[i] < min_y)
			min_y = error_queue.errors_y[i];
	}

	*fluct_x = max_x - min_x;
	*fluct_y = max_y - min_y;
}

uint8_t IsCamReached(
        int tolerance_x, int tolerance_y, int MAX_Fluctuate_error) {
	if (!Is_coordgot) {
		return 0;
	}

	int error_x = Camera_err.x;
	int error_y = Camera_err.y;

	ErrorQueue_Push(error_x, error_y);

	float avg_error_x, avg_error_y;
	int F_x, F_y;
	ErrorQueue_GetAverage(&avg_error_x, &avg_error_y);
	ErrorQueue_GetFluctuate(&F_x, &F_y);
	if (error_queue.is_full && avg_error_x <= tolerance_x &&
	        avg_error_y <= tolerance_y && F_x <= MAX_Fluctuate_error &&
	        F_y <= MAX_Fluctuate_error) {
		return 1;
	}

	return 0;
}

uint8_t Wait_for_uart_with_search(uint8_t point_index) {
	(void)point_index;
	uint32_t start_time = HAL_GetTick();
	Is_coordgot = 0;

	while (!Is_coordgot) {
		if (HAL_GetTick() - start_time > UART_WAIT_TIMEOUT_MS) {
			return 0;
		}
		HAL_Delay(1);
	}

	return 1;
}
uint8_t CamStabilizationToCenter(
        int tolerance_x, int tolerance_y, int MAX_Fluctuate_error) {

	if (!is_cam_stabilizing) {
		ErrorQueue_Init();
		PositionControl_Enable(0);
		CamStabilization_Enable(1);
		is_cam_stabilizing = 1;
		iscamreached_safecount = 0U;
		cam_stabilize_start_time = HAL_GetTick();
		IsLight = 1;
		return 0;
	}

	if (HAL_GetTick() - cam_stabilize_start_time > CAM_STABILIZE_TIMEOUT_MS) {
		CamStabilization_Enable(0);
		is_cam_stabilizing = 0;
		IsLight = 0;
		return 1;
	}

	if (IsCamReached(tolerance_x, tolerance_y, MAX_Fluctuate_error)) {
		iscamreached_safecount++;
		if (iscamreached_safecount > 10) {
			CamStabilization_Enable(0);
			is_cam_stabilizing = 0;
			IsLight = 0;
			return 1;
		}
	}

	return 0;
}
void SetTargetVelocity(float vx, float vy, float omega) {
	/* 实时任务只写目标速度，实际电机输出在固定周期里统一发送。 */
	target_velocity.vx = vx;
	target_velocity.vy = vy;
	target_velocity.omega = omega;
}

void SetTargetLinearVelocity(float vx, float vy) {
	target_velocity.vx = vx;
	target_velocity.vy = vy;
	target_velocity.omega = 0.0f;
}
void Mecanum_Init(ChassisConfig_t config) {
	/* 麦轮几何参数在启动时配置，后续运动学解算只使用静态配置。 */
	chassis_config = config;
	if (chassis_config.max_rpm <= 0.0f) {
		chassis_config.max_rpm = CHASSIS_SPEED_MAX_RPM;
	}
	k_omega = (config.wheel_dist_x + config.wheel_dist_y) / 2.0f;
	memset(control_ctx.motor, 0, sizeof(control_ctx.motor));
}

void Mecanum_Kinematics(float vx, float vy, float omega) {
	float actual_omega = omega;

	if (attitude_control.is_enabled) {
		/* 姿态环输出叠加到角速度，限制最大 omega 保护底盘转向。 */
		actual_omega += attitude_correction;

		if (actual_omega > ATTITUDE_MAX_OMEGA)
			actual_omega = ATTITUDE_MAX_OMEGA;
		if (actual_omega < -ATTITUDE_MAX_OMEGA)
			actual_omega = -ATTITUDE_MAX_OMEGA;
	}
	if (cam_stabilization.is_enabled) {
		/* 视觉校正量叠加到局部平移速度，用于对准目标物。 */
		vy += cam_correction_x * 0.01f;
		vx += cam_correction_y * 0.01f;
	}
	/* 局部坐标定义为 vx 正向前、vy 正向右；当前安装使 vx/vy 均需反向。 */
	vx = -vx;
	vy = -vy;

	/* 麦轮逆解：局部 vx/vy/omega 映射到四轮等效线速度。 */
	float v4 = vx - vy - k_omega * actual_omega;
	float v2 = vx + vy + k_omega * actual_omega;
	float v3 = vx + vy - k_omega * actual_omega;
	float v1 = vx - vy + k_omega * actual_omega;

	/* 等效轮速转换为电机速度命令。 */
	const float linear_to_rpm = 60.0f /
	        (2.0f * 3.1415926f * chassis_config.wheel_radius) *
	        CHASSIS_WHEEL_SPEED_SCALE;
	float rpm4 = v4 * linear_to_rpm;
	float rpm2 = v2 * linear_to_rpm;
	float rpm3 = v3 * linear_to_rpm;
	float rpm1 = v1 * linear_to_rpm;
	float max_abs_rpm = fabsf(rpm4);
	float scale = 1.0f;

	if (fabsf(rpm2) > max_abs_rpm)
		max_abs_rpm = fabsf(rpm2);
	if (fabsf(rpm3) > max_abs_rpm)
		max_abs_rpm = fabsf(rpm3);
	if (fabsf(rpm1) > max_abs_rpm)
		max_abs_rpm = fabsf(rpm1);
	if (max_abs_rpm > chassis_config.max_rpm) {
		scale = chassis_config.max_rpm / max_abs_rpm;
		rpm4 *= scale;
		rpm2 *= scale;
		rpm3 *= scale;
		rpm1 *= scale;
	}

	/* 实际轮位：左前4、左后3、右前1、右后2。 */
	motor4.speed_rpm = (int16_t)rpm4;
	motor3.speed_rpm = (int16_t)rpm3;
	motor1.speed_rpm = (int16_t)rpm2;
	motor2.speed_rpm = (int16_t)rpm1;
	motor4.direction = (motor4.speed_rpm >= 0) ? 0U : 1U;
	motor2.direction = (motor2.speed_rpm >= 0) ? 1U : 0U;
	motor3.direction = (motor3.speed_rpm >= 0) ? 0U : 1U;
	motor1.direction = (motor1.speed_rpm >= 0) ? 1U : 0U;
}

void Mecanum_SendVelocity(void) {
	/* 速度命令持续发送，未启动时四轮速度为 0 RPM。 */
	Emm_V5_Vel_Control(1, motor1.direction,
	        (uint16_t)abs_int(motor1.speed_rpm), CHASSIS_VELOCITY_ACCEL, 0);
	Emm_V5_Vel_Control(2, motor2.direction,
	        (uint16_t)abs_int(motor2.speed_rpm), CHASSIS_VELOCITY_ACCEL, 0);
	Emm_V5_Vel_Control(3, motor3.direction,
	        (uint16_t)abs_int(motor3.speed_rpm), CHASSIS_VELOCITY_ACCEL, 0);
	Emm_V5_Vel_Control(4, motor4.direction,
	        (uint16_t)abs_int(motor4.speed_rpm), CHASSIS_VELOCITY_ACCEL, 0);
}

void stop(void) {
	SetTargetVelocity(0.0f, 0.0f, 0.0f);
}

static void MotionState_Reset(void) {
	/* 任务切段时清除速度、规划器和 PID 历史状态，保证下一段从零状态进入。 */
	stop();
	PositionPlanner_Reset();
	target_velocity.vx = 0.0f;
	target_velocity.vy = 0.0f;
	target_velocity.omega = 0.0f;
	attitude_correction = 0.0f;
	pose_coordination.sync_scale = 1.0f;
	position_control.pid_x.integral = 0.0f;
	position_control.pid_x.prev_error = 0.0f;
	position_control.pid_y.integral = 0.0f;
	position_control.pid_y.prev_error = 0.0f;
	attitude_control.yaw_pid.integral = 0.0f;
	attitude_control.yaw_pid.prev_error = 0.0f;
	motor1.speed_rpm = 0;
	motor2.speed_rpm = 0;
	motor3.speed_rpm = 0;
	motor4.speed_rpm = 0;
	motor1.direction = 0;
	motor2.direction = 0;
	motor3.direction = 0;
	motor4.direction = 0;
}

void go(void) {
	SetTargetVelocity(0.5f, 0.0f, 0.0f);
}

void back(void) {
	SetTargetVelocity(-0.5f, 0.0f, 0.0f);
}

void left(void) {
	SetTargetVelocity(0.0f, -0.5f, 0.0f);
}

void right(void) {
	SetTargetVelocity(0.0f, 0.5f, 0.0f);
}
void right_back(void) {
	SetTargetVelocity(-0.5f, -0.5f, 0.0f);
}

void turn_left(void) {
	SetTargetVelocity(0.0f, 0.0f, 4.5f);
}

void turn_right(void) {
	SetTargetVelocity(0.0f, 0.0f, -4.5f);
}

void PID_Init(
        PID_Controller *pid, float Kp, float Ki, float Kd, float max_output) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
	pid->max_output = max_output;
}
void PID_Init_cam(PID_Controller *pid, float Kp, float Kp2, float Ki, float Kd,
        float max_output) {
	pid->Kp = Kp;
	pid->Kp2 = Kp2;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;
	pid->max_output = max_output;
}

float PID_Update(PID_Controller *pid, float error, float dt) {
	float proportional = pid->Kp * error;

	pid->integral += error * dt;
	float integral = pid->Ki * pid->integral;

	float derivative = pid->Kd * (error - pid->prev_error) / dt;
	pid->prev_error = error;

	float output = proportional + integral + derivative;

	if (output > pid->max_output) {
		output = pid->max_output;
		pid->integral -= error * dt;
	} else if (output < -pid->max_output) {
		output = -pid->max_output;
		pid->integral -= error * dt;
	}

	return output;
}
float PID_Update_cam(PID_Controller *pid, float error, float dt) {
	float proportional = pid->Kp * error;
	float proportional2 = pid->Kp2 * error * fabsf(error);

	pid->integral += error * dt;
	float integral = pid->Ki * pid->integral;

	float derivative = pid->Kd * (error - pid->prev_error) / dt;
	pid->prev_error = error;

	float output = proportional + proportional2 + integral + derivative;

	if (output > pid->max_output) {
		output = pid->max_output;
		pid->integral -= error * dt;
	} else if (output < -pid->max_output) {
		output = -pid->max_output;
		pid->integral -= error * dt;
	}

	return output;
}
float PID_Update_angle(PID_Controller *pid, float error, float dt) {
	float proportional = pid->Kp * error;

	pid->integral += error * dt;
	float integral = pid->Ki * pid->integral;

	float derivative = pid->Kd * (error - pid->prev_error) / dt;
	pid->prev_error = error;

	float G = odometer_ctx.pose.yaw_gyro_dps * g;
	float output = proportional + integral + derivative;

	if (output > pid->max_output) {
		output = pid->max_output;
		pid->integral -= error * dt;
	} else if (output < -pid->max_output) {
		output = -pid->max_output;
		pid->integral -= error * dt;
	}

	return output;
}
static PID_Controller line_follow_pid;
void Line_Follow_Enable(uint8_t enable) {
	is_line_follow_enabled = enable;

	if (!enable) {
		line_follow_pid.integral = 0.0f;
		line_follow_pid.prev_error = 0.0f;
		stop();
	}
}
void Line_Follow_Control(uint8_t *sensor_values, float base_speed, float dt) {
	if (!is_line_follow_enabled) {
		line_follow_pid.integral = 0.0f;
		line_follow_pid.prev_error = 0.0f;

		return;
	}
	float error = 0.0f;

	if (is_task1_active()) {
		if (sensor_values[2] == 0 || sensor_values[3] == 0) {
			error = 0.0f;
			line_follow_pid.integral = 0.0f;
			line_follow_pid.prev_error = 0.0f;
		} else {

			float total_weight = 0.0f;
			const float weights[8] = {-6.f, -3.0f, -1.0f, 1.0f,
			        1.0f, 2.0f, 4.0f, 6.0f};

			for (int i = 0; i < 8; i++) {
				if (sensor_values[i] == 0) {
					error += weights[i];
					total_weight += abs_float(weights[i]);
				}
			}

			if (total_weight > 0) {
				error /= total_weight;
			} else {
				stop();
				return;
			}
		}
	} else {
		if (sensor_values[2] == 0 || sensor_values[3] == 0) {
			error = 0.0f;
			line_follow_pid.integral = 0.0f;
		} else {
			float total_weight = 0.0f;
			const float weights[8] = {-4.f, -1.0f, -1.0f, 1.0f,
			        10.0f, 10.0f, 11.0f, 12.0f};

			for (int i = 0; i < 8; i++) {
				if (sensor_values[i] == 0) {
					error += weights[i];
					total_weight += abs_float(weights[i]);
				}
			}

			if (total_weight > 0) {
				error /= total_weight;
			} else {
				stop();
				return;
			}
		}
	}

	float omega = PID_Update(&line_follow_pid, error, dt);

	if (task1_state.active) {
		if (!IsRun)
			SetTargetVelocity(0, 0.0f, -omega);
		else
			SetTargetVelocity(base_speed, 0.0f, -0.68f - omega);
	} else {
		if (!IsRun)
			SetTargetVelocity(0, 0.0f, -omega);
		else
			SetTargetVelocity(base_speed, 0.0f, 0.695f - omega);
	}
}
void Follow_line_task(void) {
	static uint32_t last_time = 0;
	uint32_t current_time = HAL_GetTick();
	float dt = (current_time - last_time) / 1000.0f;
	last_time = current_time;

	if (dt <= 0)
		dt = 0.01f;
	if (is_task1_active()) {
		Line_Follow_Control(Anolog, 0.2376f, dt);
	} else {
		Line_Follow_Control(Anolog, 0.229f, dt);
	}
}

uint8_t move_to_target(float x, float y) {

	if (!is_position_control_enabled && !target_set) {
		/* 非阻塞位置目标下发一次，循环任务等待位置环自动关闭。 */
		SetTargetPosition(x, y);
		PositionControl_Enable(1);
		target_set = 1;
		return 0;
	}

	if (!is_position_control_enabled && target_set) {
		target_set = 0;
		return 1;
	}

	return 0;
}
/***********Task1***************/

static void task1_next_step(void);
static void task1_finish(void);
static uint8_t task1_move_target(
        uint8_t target_index, float y_offset_mm, float yaw_deg);
static void task1_go_first_qrcode(void);
static void task1_prepare_material_turnplate(void);
static void task1_go_first_material_point(void);
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
static void task1_go_podium_point(uint8_t target_index,
        uint8_t arrive_servo, uint8_t prepare_servo,
        float prepare_tolerance_m);
static void task1_go_home(void);

void start_task1(void) {
	/* 任务状态机从步骤 1 开始，step_start 记录当前步骤进入时间。 */
	task1_state.active = 1;
	task1_state.current_step = 1;
	task1_state.completed = 0;
	task1_state.step_start = HAL_GetTick();
	task1_vision_timeout = 0U;
	task1_fourth_color_state = 0U;
	memset(&task1_qrcode_servo_move, 0, sizeof(task1_qrcode_servo_move));
	task1_qrcode_servo_move_started = 0U;
	is_hasgetcolor = 0U;
}

void run_task1(void) {
	Update_Materialcolor();

	if (!task1_state.active) {
		return;
	}

	switch (task1_state.current_step) {
	case 1: task1_go_first_qrcode(); break;                   /* 起步途经二维码点 1。 */
	case 2: task1_prepare_material_turnplate(); break;         /* 准备物料转盘。 */
	case 11: task1_go_first_material_point(); break;          /* a 点取第 1 个物料。 */
	case 12: task1_wait_vision(1); break;
	case 13: task1_relative_action(0, 0U); break;
	case 14: task1_go_material_point(2, 2); break;            /* b 点取第 2 个物料。 */
	case 15: task1_wait_vision(2); break;
	case 16: task1_relative_action(0, 0U); break;
	case 17: task1_go_material_point(3, 3); break;            /* c 点取第 3 个物料。 */
	case 18: task1_wait_vision(3); break;
	case 19: task1_relative_action(0, 0U); break;
	case 20: task1_go_material_point(4, 4); break;            /* d 点取第 4 个物料。 */
	case 21: task1_wait_vision(4); break;
	case 22: task1_relative_action(0, 0U); break;
	case 23: task1_go_material_point(5, 5); break;            /* e 点取第 5 个物料。 */
	case 24: task1_wait_vision(5); break;
	case 25: task1_relative_action(0, 0U); break;
	case 126: task1_go_second_qrcode(); break;                 /* 途经二维码点 2。 */
	case 27: task1_go_trophy_transition(); break;              /* 前往奖杯开环动作过渡点。 */
	case 28: task1_pick_trophies(); break;                    /* 连续拾取三个奖杯。 */
	case 31: task1_go_second_podium(); break;                 /* 亚军台。 */
	case 32: task1_wait_vision(8); break;
	case 33: task1_relative_action(3, 100U); break;
	case 34: task1_go_champion_podium(); break;              /* 冠军台。 */
	case 35: task1_wait_vision(9); break;
	case 36: task1_relative_action(3, 0U); break;
	case 37: task1_go_third_podium(); break;                 /* 季军台。 */
	case 38: task1_wait_vision(10); break;
	case 39: task1_relative_action(0, 0U); break;
	case 40: task1_go_home(); break;                          /* 直接回起始点。 */
	default: task1_finish(); break;
	}
}
uint8_t is_task1_active(void) {
	return task1_state.active;
}

void start_task2(void) {
	/* 任务二用于底盘 X 轴恒速调试。 */
	task2_state.active = 1U;
	task2_state.current_step = 1U;
	task2_state.completed = 0U;
	task2_state.step_start = HAL_GetTick();
	tasknum = 2U;
	MotionState_Reset();
	PoseCoordination_Enable(0U);
	PositionControl_Enable(0U);
	EnableAttitudeStabilization(0U);
	CamStabilization_Enable(0U);
	SetTargetVelocity(TASK2_TEST_SPEED_MPS, 0.0f, 0.0f);
}

void run_task2(void) {
	if (!task2_state.active) {
		return;
	}

	SetTargetVelocity(0.0f, 0.5f, 0.0f);
}

uint8_t is_task2_active(void) {
	return task2_state.active;
}

/**
 * @brief 进入任务 1 的下一个顺序步骤。
 *
 * current_step 是任务 1 的主线步骤号，step_start 记录每次进新步骤的时间。
 */
static void task1_next_step(void) {
	task1_state.current_step++;
	task1_state.step_start = HAL_GetTick();
}

/**
 * @brief 标记任务 1 已完成。
 */
static void task1_finish(void) {
	task1_state.active = 0;
	task1_state.completed = 1;
}

/**
 * @brief 按任务 1 路径点移动。
 *
 * @param target_index task1_targets 下标。
 * @param y_offset_mm Y 方向附加偏移，单位 mm。
 * @param yaw_deg 目标 yaw，单位 deg。
 * @return 1 表示到达目标点。
 */
static uint8_t task1_move_target(
        uint8_t target_index, float y_offset_mm, float yaw_deg) {
	return move_to_pose_target(task1_targets[target_index].x,
	        task1_targets[target_index].y + y_offset_mm, yaw_deg);
}

/**
 * @brief 二维码接近阶段平滑下压舵机。
 *
 * 剩余距离进入 0.50 m 时启动 500 ms 轨迹，PWM 由 2460 平滑减小至 1820。
 */
static void task1_qrcode_servo_slowdown(void) {
	if (!task1_qrcode_servo_move_started && IsPositionReached(0.60f)) {
		servo_smooth_move_start(&task1_qrcode_servo_move, &htim14,
		        TIM_CHANNEL_1, 2460U, 1820U, 500);
		task1_qrcode_servo_move_started = 1U;
	}

	servo_smooth_move_update(&task1_qrcode_servo_move);
}

/**
 * @brief 起步到第一个二维码点。
 *
 * 接近二维码点时启动 500 ms 的舵机缓出轨迹。
 */
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

/**
 * @brief 按原有运动节奏准备转盘五个物料位。
 *
 * 转盘到位后读取 RGB，保存四个实测颜色并在后续推导第五种颜色。
 */
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
	HAL_Delay(600);
	Get_Materialcolor(1U);
	HAL_Delay(100);
	

	Move_to_turnplate(6);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[1]);
	MotionState_Reset();
	HAL_Delay(300);
	task1_state.current_step = 11;
	task1_state.step_start = HAL_GetTick();
}

/**
 * @brief 进入物块段第一个取放点。
 *
 * 这里额外开启姿态稳定。
 */
static void task1_go_first_material_point(void) {
	EnableAttitudeStabilization(1);
	Get_Materialcolor(2U);
	is_hasgetcolor = 1U;
	task1_go_material_point(1, 1);
	HAL_Delay (100);
}

/**
 * @brief 移动到物块区取放点并切换对应物料位。
 *
 * @param target_index task1_targets 下标。
 * @param material_index 放置序号，从 1 开始，转盘按 5 到 1 倒序切换。
 */
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

/**
 * @brief 等待视觉搜索结束并进入下一步。
 *
 * 当前策略是收到坐标或超时都继续，视觉等待语义集中在这里方便后续改策略。
 */
static void task1_wait_vision(uint8_t point_index) {
	task1_vision_timeout = (uint8_t)!Wait_for_uart_with_search(point_index);
	task1_next_step();
}

/**
 * @brief 执行一次相对前后取放动作。
 *
 * @param servo_after 相对动作完成后的舵机动作组，0 表示不执行。
 * @param delay_after_ms 舵机动作后的等待时间，单位 ms。
 */
static void task1_relative_action(uint8_t servo_after, uint32_t delay_after_ms) {
	uint8_t skip_pos_move =
	        (task1_state.current_step == 13U ||
	         task1_state.current_step == 19U);

	if (task1_vision_timeout) {
		task1_vision_timeout = 0U;
	} else if (!CamStabilizationToCenter(TOLERANCE, TOLERANCE,
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

/**
 * @brief 前往第二个二维码点。
 *
 * 到点后将转盘准备到第 3 个奖杯位。
 */
static void task1_go_second_qrcode(void) {
	tasknum = 2;
	if (task1_move_target(6, 0.0f, 0.0f)) {
		Move_to_turnplate(5);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[1]);
		task1_next_step();
	}
}

/**
 * @brief 前往奖杯连续拾取动作的过渡位姿。
 */
static void task1_go_trophy_transition(void) {
	if (task1_move_target(7U, 0.0f, task1_targets[7U].yaw_deg)) {
		task1_next_step();
	}
}

/**
 * @brief 按定时速度连续拾取三个奖杯。
 *
 * 奖杯依次进入转盘 3、2、1 号位，底盘旋转方向与物料拾取段相反。
 */
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

	MotionState_Reset();
	HAL_Delay(100);
	task1_state.current_step = 31;
	task1_state.step_start = HAL_GetTick();
}

/**
 * @brief 前往亚军台。
 *
 * 接近途中先展开舵机 5，到点后执行舵机 6 。
 */
static void task1_go_second_podium(void) {
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[0]);
	task1_go_podium_point(8, 6, 5, 0.6f);
}

/**
 * @brief 前往冠军台，到点后执行舵机 4 。
 */
static void task1_go_champion_podium(void) {
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[0]);
	task1_go_podium_point(9, 4, 0, 0.0f);
}

/**
 * @brief 前往季军台，到点后执行舵机 7 。
 */
static void task1_go_third_podium(void) {
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Telescopic_ring[0]);
	task1_go_podium_point(10, 7, 0, 0.0f);
}

/**
 * @brief 移动到奖台点。
 *
 * @param target_index task1_targets 下标。
 * @param arrive_servo 到点后的舵机动作组。
 * @param turnplate_num 到点后切换的奖杯转盘位。
 * @param prepare_servo 接近途中提前执行的舵机动作组，0 表示不执行。
 * @param prepare_tolerance_m 提前动作触发容差，单位 m。
 */
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

/**
 * @brief 直接返回起始点并恢复精细到达容差。
 */
static void task1_go_home(void) {
	raceh = POSITION_FINE_REACHED_TOLERANCE_M;
	if (task1_move_target(11, 0.0f, -90.0f)) {
		task1_finish();
	}
}
/***********Task1***************/
