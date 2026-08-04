/**
 ******************************************************************************
 * @file    motion.c
 * @brief   底盘位置、姿态与麦轮运动控制
 ******************************************************************************
 */

#include "motion.h"
#include "control.h"
#include "motion_plan.h"
#include "feedforward.h"
#include "fuzzy.h"
#include "odometer.h"
#include "ai_tune.h"
#include <math.h>
#include <string.h>

static void yaw_plan_reset(float current_yaw_deg);
static float yaw_plan_update(float target_yaw_deg, float dt);
static float NormalizeYawError(float error);
static float ClampFloat(float value, float min_value, float max_value);
static void motion_observer_reset(void);
static void motion_observer_update(float dt);
static float mpc_linear_speed_limit(float distance_m, float speed_mps,
		float cruise_speed_mps);
static float mpc_yaw_rate_limit(float angle_error_rad, float yaw_rate_rad_s);

/** 设置底盘局部坐标系速度目标。 */
void motion_set_velocity(float vx, float vy, float omega)
{
    target_velocity.vx = vx;
    target_velocity.vy = vy;
    target_velocity.omega = omega;
}

void motion_set_linear_velocity(float vx, float vy)
{
    motion_set_velocity(vx, vy, 0.0f);
}

/** 清空速度、规划器、PID 和四轮输出状态。 */
void motion_reset(void) {

	stop();
	motion_plan_reset();
	feedforward_reset();
	target_velocity.vx = 0.0f;
	target_velocity.vy = 0.0f;
	target_velocity.omega = 0.0f;
	attitude_correction = 0.0f;
	pose_coordination.sync_scale = 1.0f;
	position_control.pid_x.integral = 0.0f;
	position_control.pid_x.prev_error = 0.0f;
	position_control.pid_x.fuzzy_rate = 0.0f;
	position_control.pid_y.integral = 0.0f;
	position_control.pid_y.prev_error = 0.0f;
	position_control.pid_y.fuzzy_rate = 0.0f;
	attitude_control.yaw_pid.integral = 0.0f;
	attitude_control.yaw_pid.prev_error = 0.0f;
	attitude_control.yaw_pid.fuzzy_rate = 0.0f;
	yaw_plan_reset(odometer_get_yaw_deg());
	motion_observer_reset();
	motor1.speed_rpm = 0;
	motor2.speed_rpm = 0;
	motor3.speed_rpm = 0;
	motor4.speed_rpm = 0;
	motor1.direction = 0;
	motor2.direction = 0;
	motor3.direction = 0;
	motor4.direction = 0;
}

static ChassisConfig_t chassis_config;
static float k_omega;
static float yaw_plan_angle_deg;
static float yaw_plan_rate_rad_s;

typedef struct
{
	float velocity_x_mps;
	float velocity_y_mps;
	float accel_x_mps2;
	float accel_y_mps2;
	float yaw_rate_rad_s;
	float yaw_accel_rad_s2;
	float last_velocity_x_mps;
	float last_velocity_y_mps;
	float last_yaw_rate_rad_s;
	uint32_t last_update_ms;
	uint8_t initialized;
} MotionObserverState;

static MotionObserverState motion_observer;

static void yaw_plan_reset(float current_yaw_deg)
{
	yaw_plan_angle_deg = current_yaw_deg;
	yaw_plan_rate_rad_s = 0.0f;
}

static float yaw_plan_update(float target_yaw_deg, float dt)
{
	const float deg_to_rad = 0.017453292519943295f;
	float error_rad;
	float brake_rate_rad_s;
	float target_rate_rad_s;
	float rate_delta_rad_s;

	if (dt <= 0.0f) {
		return yaw_plan_angle_deg;
	}
	error_rad = NormalizeYawError(target_yaw_deg - yaw_plan_angle_deg) *
			deg_to_rad;
	brake_rate_rad_s = sqrtf(2.0f * ATTITUDE_PLANNER_MAX_DECEL_RAD_S2 *
			fabsf(error_rad));
	brake_rate_rad_s = ClampFloat(brake_rate_rad_s, 0.0f,
			ATTITUDE_PID_MAX_OUTPUT);
	target_rate_rad_s = error_rad >= 0.0f ? brake_rate_rad_s :
			-brake_rate_rad_s;
	rate_delta_rad_s = target_rate_rad_s - yaw_plan_rate_rad_s;
	rate_delta_rad_s = ClampFloat(rate_delta_rad_s,
			-ATTITUDE_PLANNER_MAX_DECEL_RAD_S2 * dt,
			ATTITUDE_PLANNER_MAX_ACCEL_RAD_S2 * dt);
	yaw_plan_rate_rad_s += rate_delta_rad_s;
	yaw_plan_angle_deg += yaw_plan_rate_rad_s * dt / deg_to_rad;
	return yaw_plan_angle_deg;
}

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

static void motion_observer_reset(void)
{
	memset(&motion_observer, 0, sizeof(motion_observer));
	motion_observer.velocity_x_mps = odometer_ctx.motion.vel_x_mps;
	motion_observer.velocity_y_mps = odometer_ctx.motion.vel_y_mps;
	motion_observer.yaw_rate_rad_s = odometer_ctx.motion.yaw_rate_dps *
	        0.017453292519943295f;
	motion_observer.last_velocity_x_mps = motion_observer.velocity_x_mps;
	motion_observer.last_velocity_y_mps = motion_observer.velocity_y_mps;
	motion_observer.last_yaw_rate_rad_s = motion_observer.yaw_rate_rad_s;
	motion_observer.last_update_ms = HAL_GetTick();
	motion_observer.initialized = 1U;
}

static void motion_observer_update(float dt)
{
	uint32_t now_ms = HAL_GetTick();
	uint32_t elapsed_ms;
	float sample_dt;
	float raw_accel_x;
	float raw_accel_y;
	float raw_yaw_accel;
	float alpha = ClampFloat(MOTION_OBSERVER_ALPHA, 0.0f, 1.0f);
	float raw_vx = odometer_ctx.motion.vel_x_mps;
	float raw_vy = odometer_ctx.motion.vel_y_mps;
	float raw_yaw_rate = odometer_ctx.motion.yaw_rate_dps *
	        0.017453292519943295f;

	if (!motion_observer.initialized) {
		motion_observer_reset();
		return;
	}
	elapsed_ms = now_ms - motion_observer.last_update_ms;
	if (elapsed_ms == 0U) {
		return;
	}
	if ((elapsed_ms > 100U) || (dt <= 0.0f)) {
		motion_observer_reset();
		return;
	}
	sample_dt = (float)elapsed_ms * 0.001f;
	motion_observer.velocity_x_mps += alpha *
	        (raw_vx - motion_observer.velocity_x_mps);
	motion_observer.velocity_y_mps += alpha *
	        (raw_vy - motion_observer.velocity_y_mps);
	motion_observer.yaw_rate_rad_s += alpha *
	        (raw_yaw_rate - motion_observer.yaw_rate_rad_s);
	raw_accel_x = (motion_observer.velocity_x_mps -
	        motion_observer.last_velocity_x_mps) / sample_dt;
	raw_accel_y = (motion_observer.velocity_y_mps -
	        motion_observer.last_velocity_y_mps) / sample_dt;
	raw_yaw_accel = (motion_observer.yaw_rate_rad_s -
	        motion_observer.last_yaw_rate_rad_s) / sample_dt;
	motion_observer.accel_x_mps2 += alpha *
	        (raw_accel_x - motion_observer.accel_x_mps2);
	motion_observer.accel_y_mps2 += alpha *
	        (raw_accel_y - motion_observer.accel_y_mps2);
	motion_observer.yaw_accel_rad_s2 += alpha *
	        (raw_yaw_accel - motion_observer.yaw_accel_rad_s2);
	motion_observer.last_velocity_x_mps = motion_observer.velocity_x_mps;
	motion_observer.last_velocity_y_mps = motion_observer.velocity_y_mps;
	motion_observer.last_yaw_rate_rad_s = motion_observer.yaw_rate_rad_s;
	motion_observer.last_update_ms = now_ms;
}

static float mpc_linear_speed_limit(float distance_m, float speed_mps,
		float cruise_speed_mps)
{
	float best_speed = 0.0f;
	float best_cost = 1.0e30f;
	uint8_t candidate_index;

	if (distance_m <= 0.0f) {
		return 0.0f;
	}
	cruise_speed_mps = ClampFloat(cruise_speed_mps, 0.0f,
	        POSITION_MAX_LINEAR_SPEED_MPS);
	for (candidate_index = 0U; candidate_index < 9U; candidate_index++) {
		float target_speed = cruise_speed_mps *
		        (float)candidate_index * 0.125f;
		float predicted_distance = distance_m;
		float predicted_speed = ClampFloat(speed_mps, 0.0f,
		        POSITION_MAX_LINEAR_SPEED_MPS);
		float cost = 0.0f;
		uint8_t step;

		for (step = 0U; step < MOTION_MPC_HORIZON_STEPS; step++) {
			float speed_delta = ClampFloat(target_speed - predicted_speed,
			        -MOTION_MPC_MAX_ACCEL_MPS2 * MOTION_MPC_STEP_S,
			        MOTION_MPC_MAX_ACCEL_MPS2 * MOTION_MPC_STEP_S);
			predicted_speed = ClampFloat(predicted_speed + speed_delta,
			        0.0f, POSITION_MAX_LINEAR_SPEED_MPS);
			predicted_distance -= predicted_speed * MOTION_MPC_STEP_S;
			cost += 0.02f * speed_delta * speed_delta;
			if (predicted_distance < 0.0f) {
				cost += MOTION_MPC_OVERSHOOT_WEIGHT *
				        predicted_distance * predicted_distance;
			}
		}
		cost += MOTION_MPC_TERMINAL_DISTANCE_WEIGHT *
		        predicted_distance * predicted_distance +
		        MOTION_MPC_TERMINAL_SPEED_WEIGHT * predicted_speed *
		        predicted_speed;
		if (distance_m > MOTION_MPC_CRUISE_PREFERENCE_DISTANCE_M) {
			float cruise_speed_error = cruise_speed_mps - target_speed;
			cost += MOTION_MPC_CRUISE_PREFERENCE_GAIN *
			        cruise_speed_error * cruise_speed_error;
		}
		if ((distance_m > MOTION_MPC_LOW_SPEED_PREFERENCE_DISTANCE_M) &&
		        (target_speed < MOTION_MPC_LOW_SPEED_PREFERENCE_SPEED_MPS)) {
			float low_speed_error =
			        MOTION_MPC_LOW_SPEED_PREFERENCE_SPEED_MPS - target_speed;
			cost += MOTION_MPC_LOW_SPEED_PREFERENCE_GAIN *
			        low_speed_error * low_speed_error;
		}
		if (cost < best_cost) {
			best_cost = cost;
			best_speed = target_speed;
		}
	}
	if ((distance_m > POSITION_PLANNER_NEAR_TARGET_DISTANCE_M) &&
	        (best_speed < POSITION_PLANNER_MIN_SPEED_MPS)) {
		best_speed = POSITION_PLANNER_MIN_SPEED_MPS;
	}
	return best_speed;
}

static float mpc_yaw_rate_limit(float angle_error_rad, float yaw_rate_rad_s)
{
	float sign = angle_error_rad >= 0.0f ? 1.0f : -1.0f;
	float best_magnitude = 0.0f;
	float best_cost = 1.0e30f;
	float braking_limit;
	uint8_t candidate_index;

	if (fabsf(angle_error_rad) < 0.0001f) {
		return ClampFloat(fabsf(yaw_rate_rad_s), 0.25f,
		        ATTITUDE_PID_MAX_OUTPUT);
	}
	for (candidate_index = 0U; candidate_index < 5U; candidate_index++) {
		float target_rate = sign * ATTITUDE_PID_MAX_OUTPUT *
		        (float)candidate_index * 0.25f;
		float predicted_error = angle_error_rad;
		float predicted_rate = yaw_rate_rad_s;
		float cost = 0.0f;
		uint8_t step;

		for (step = 0U; step < MOTION_MPC_HORIZON_STEPS; step++) {
			float rate_delta = ClampFloat(target_rate - predicted_rate,
			        -MOTION_MPC_MAX_ANG_ACCEL_RAD_S2 * MOTION_MPC_STEP_S,
			        MOTION_MPC_MAX_ANG_ACCEL_RAD_S2 * MOTION_MPC_STEP_S);
			predicted_rate += rate_delta;
			predicted_error -= predicted_rate * MOTION_MPC_STEP_S;
			cost += 0.02f * rate_delta * rate_delta;
			if (predicted_error * sign < 0.0f) {
				cost += 120.0f * predicted_error * predicted_error;
			}
		}
		cost += 160.0f * predicted_error * predicted_error +
		        3.0f * predicted_rate * predicted_rate;
		if (cost < best_cost) {
			best_cost = cost;
			best_magnitude = fabsf(target_rate);
		}
	}
	braking_limit = sqrtf(2.0f * ATTITUDE_PLANNER_MAX_DECEL_RAD_S2 *
	        fabsf(angle_error_rad));
	best_magnitude = fmaxf(best_magnitude, braking_limit);
	best_magnitude = fmaxf(best_magnitude, fabsf(yaw_rate_rad_s));
	return ClampFloat(best_magnitude, 0.25f, ATTITUDE_PID_MAX_OUTPUT);
}

static void PositionPlanner_Reset(void) {
	motion_plan_reset();
	feedforward_reset();
}

static float PositionSpeedPlan_Update(float distance_error_m, float dt) {
	if (distance_error_m <= raceh ||
	        distance_error_m <= POSITION_PLANNER_NEAR_TARGET_DISTANCE_M) {
		PositionPlanner_Reset();
		return 0.0f;
	}
	return motion_plan_update(distance_error_m, dt);
}

/** 位置控制模块。 */
void PositionControl_Init(void) {

	pid_init(&position_control.pid_x, POSITION_PID_DEFAULT_KP, 0.00f,
	        POSITION_PID_DEFAULT_KD, HIGHESTSPEED);

	pid_init(&position_control.pid_y, POSITION_PID_DEFAULT_KP, 0.00f,
	        POSITION_PID_DEFAULT_KD, HIGHESTSPEED);

	position_control.is_enabled = 0;
	position_control.target_x = 0.0f;
	position_control.target_y = 0.0f;
	position_control.last_update_time = HAL_GetTick();
	motion_plan_init(POSITION_PLANNER_CRUISE_SPEED_MPS);

	is_position_control_enabled = 0;
}

void PositionControl_Enable(uint8_t enable) {
	position_control.is_enabled = enable;
	is_position_control_enabled = enable;

	if (!enable) {

		position_control.pid_x.integral = 0.0f;
		position_control.pid_x.prev_error = 0.0f;
		position_control.pid_x.fuzzy_rate = 0.0f;
		position_control.pid_y.integral = 0.0f;
		position_control.pid_y.prev_error = 0.0f;
		position_control.pid_y.fuzzy_rate = 0.0f;
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
	motion_plan_set_cruise(max_speed_mps);
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

	Is_vel_mode = 1;
	position_control.target_x = x;
	position_control.target_y = y;
	target_velocity.omega = 0.0f;
	position_control.pid_x.integral = 0.0f;
	position_control.pid_x.prev_error = 0.0f;
	position_control.pid_x.fuzzy_rate = 0.0f;
	position_control.pid_y.integral = 0.0f;
	position_control.pid_y.prev_error = 0.0f;
	position_control.pid_y.fuzzy_rate = 0.0f;
	PositionPlanner_Reset();
}

uint8_t IsPositionReached(float tolerance_xy) {

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
	       (motion_plan_speed() <= POSITION_PLANNER_STOP_SPEED_MPS);
}

void UpdatePositionControl(float dt) {
	if (!position_control.is_enabled || dt <= 0) {
		return;
	}
	motion_observer_update(dt);


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


	float error_x_local = error_x_global * cosf(current_yaw_rad) +
	                      error_y_global * sinf(current_yaw_rad);
	float error_y_local = -error_x_global * sinf(current_yaw_rad) +
	                      error_y_global * cosf(current_yaw_rad);
	control_ctx.error_x_local_m = error_x_local;
	control_ctx.error_y_local_m = error_y_local;


	float plan_speed = PositionSpeedPlan_Update(distance_error, dt);
	float observed_speed = 0.0f;
	float mpc_speed_limit;
	if (distance_error > 0.001f) {
		observed_speed = (error_x_global * motion_observer.velocity_x_mps +
		        error_y_global * motion_observer.velocity_y_mps) /
		        distance_error;
	}
	mpc_speed_limit = mpc_linear_speed_limit(distance_error, observed_speed,
	        motion_plan_cruise());
	feedforward_accel_update(dt);
	const ai_tune_param_t *param = ai_tune_get_active();
	float plan_vx = 0.0f;
	float plan_vy = 0.0f;
	float feedforward_vx = 0.0f;
	float feedforward_vy = 0.0f;
	float accel_x_local = feedforward_accel_x() *
	                              cosf(current_yaw_rad) +
	                      feedforward_accel_y() *
	                              sinf(current_yaw_rad);
	float accel_y_local = -feedforward_accel_x() *
	                              sinf(current_yaw_rad) +
	                      feedforward_accel_y() *
	                              cosf(current_yaw_rad);
	float yaw_rate_rad_s = odometer_ctx.motion.yaw_rate_dps *
	        0.017453292519943295f;
	float yaw_accel_rad_s2 = odometer_ctx.motion.yaw_accel_dps2 *
	        0.017453292519943295f;

	float com_accel_x_local = -yaw_accel_rad_s2 * CHASSIS_CENTER_OF_MASS_Y_M -
	        yaw_rate_rad_s * yaw_rate_rad_s * CHASSIS_CENTER_OF_MASS_X_M;
	float com_accel_y_local = yaw_accel_rad_s2 * CHASSIS_CENTER_OF_MASS_X_M -
	        yaw_rate_rad_s * yaw_rate_rad_s * CHASSIS_CENTER_OF_MASS_Y_M;
	if (distance_error > raceh) {
		plan_vx = error_x_local * plan_speed / distance_error;
		plan_vy = error_y_local * plan_speed / distance_error;
		float desired_accel_x_local = error_x_local *
		        motion_plan_accel() / distance_error;
		float desired_accel_y_local = error_y_local *
		        motion_plan_accel() / distance_error;
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
	float feedback_vx_global =
	        fuzzy_pid_calc(&position_control.pid_x, error_x_global, dt,
				POSITION_FUZZY_ERROR_FULL_M, POSITION_FUZZY_RATE_FULL_MPS,
				POSITION_FUZZY_KP_MIN_SCALE, POSITION_FUZZY_KP_MAX_SCALE,
				POSITION_FUZZY_KD_MIN_SCALE, POSITION_FUZZY_KD_MAX_SCALE, 0U);
	float feedback_vy_global =
	        fuzzy_pid_calc(&position_control.pid_y, error_y_global, dt,
				POSITION_FUZZY_ERROR_FULL_M, POSITION_FUZZY_RATE_FULL_MPS,
				POSITION_FUZZY_KP_MIN_SCALE, POSITION_FUZZY_KP_MAX_SCALE,
				POSITION_FUZZY_KD_MIN_SCALE, POSITION_FUZZY_KD_MAX_SCALE, 0U);
	float feedback_vx = feedback_vx_global * cosf(current_yaw_rad) +
	        feedback_vy_global * sinf(current_yaw_rad);
	float feedback_vy = -feedback_vx_global * sinf(current_yaw_rad) +
	        feedback_vy_global * cosf(current_yaw_rad);
	float feedback_speed =
	        sqrtf(feedback_vx * feedback_vx + feedback_vy * feedback_vy);

	if (feedback_speed > POSITION_FEEDBACK_MAX_SPEED_MPS) {

		float feedback_ratio =
		        POSITION_FEEDBACK_MAX_SPEED_MPS / feedback_speed;
		feedback_vx *= feedback_ratio;
		feedback_vy *= feedback_ratio;
	}
	if ((distance_error > raceh) &&
	        (distance_error <= POSITION_STATIC_FRICTION_ENABLE_DISTANCE_M) &&
	        (feedback_speed < param->static_friction_speed_mps)) {

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
	        motion_plan_cruise());
	command_speed_limit = ClampFloat(command_speed_limit, 0.0f,
	        mpc_speed_limit);
	if (linear_speed > command_speed_limit) {

		float ratio = command_speed_limit / linear_speed;
		vx_local *= ratio;
		vy_local *= ratio;
		linear_speed = command_speed_limit;
	}
	if (distance_error <= raceh) {
		vx_local = 0.0f;
		vy_local = 0.0f;
		linear_speed = 0.0f;
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
		        .target_yaw_deg = attitude_control.target_yaw,
		        .yaw_deg = odometer_get_yaw_deg(),
		        .error_x_local_m = error_x_local,
		        .error_y_local_m = error_y_local,
		        .velocity_x_mps = odometer_ctx.motion.vel_x_mps,
		        .velocity_y_mps = odometer_ctx.motion.vel_y_mps,
		        .plan_accel_mps2 = motion_plan_accel(),
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

/** 姿态控制模块。 */
float GetAttitudeCorrection(void) {
	return attitude_correction;
}
void AttitudeStabilization_Init(void) {

	pid_init(&attitude_control.yaw_pid, ATTITUDE_PID_DEFAULT_KP,
	        ATTITUDE_PID_DEFAULT_KI, ATTITUDE_PID_DEFAULT_KD,
	        ATTITUDE_PID_MAX_OUTPUT);

	isPosture_correction = 0;

	attitude_control.target_yaw = 0.0f;
	attitude_control.is_enabled = 0;
	attitude_control.last_update_time = HAL_GetTick();
	yaw_plan_reset(odometer_get_yaw_deg());
}

void SetTargetYaw(float target) {

	if (fabsf(NormalizeYawError(attitude_control.target_yaw - target)) <
	        0.001f) {
		return;
	}
	attitude_control.target_yaw = target;

	attitude_control.yaw_pid.integral = 0.0f;
	attitude_control.yaw_pid.prev_error = 0.0f;
	attitude_control.yaw_pid.fuzzy_rate = 0.0f;
	yaw_plan_reset(odometer_get_yaw_deg());
}
uint8_t IsYawReached(float target_yaw, float tolerance_degrees) {
	float current_yaw = odometer_get_yaw_deg();
	float error = target_yaw - current_yaw;


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

		attitude_control.target_yaw = odometer_get_yaw_deg();
		attitude_control.last_update_time = HAL_GetTick();

		attitude_control.yaw_pid.integral = 0.0f;
		attitude_control.yaw_pid.prev_error = 0.0f;
		attitude_control.yaw_pid.fuzzy_rate = 0.0f;
		yaw_plan_reset(attitude_control.target_yaw);
	}
}

void UpdateAttitudeStabilization(float dt) {
	if (!attitude_control.is_enabled || dt <= 0) {
		attitude_correction = 0.0f;
		return;
	}
	motion_observer_update(dt);

	float current_yaw = odometer_get_yaw_deg();

	float planned_yaw = yaw_plan_update(attitude_control.target_yaw, dt);
	float error = planned_yaw - current_yaw;
	float error_rad;
	float yaw_rate_rad_s;
	float target_error_deg;
	float mpc_rate_limit;
	float omega_command;


	while (error > 180.0f)
		error -= 360.0f;
	while (error < -180.0f)
		error += 360.0f;
	yaw_rate_rad_s = motion_observer.yaw_rate_rad_s;
	target_error_deg = NormalizeYawError(attitude_control.target_yaw -
	        current_yaw);
	control_ctx.yaw_error_deg = target_error_deg;
	if ((fabsf(target_error_deg) <= POSE_YAW_TOLERANCE_DEG) &&
	        (fabsf(yaw_rate_rad_s) < 0.05f)) {
		yaw_plan_reset(current_yaw);
		attitude_control.yaw_pid.integral = 0.0f;
		attitude_correction = 0.0f;
		control_ctx.yaw_error_deg = target_error_deg;
		return;
	}
	if ((fabsf(target_error_deg) < DEAD_ZONE) &&
	        (fabsf(yaw_rate_rad_s) < 0.03f)) {
		attitude_control.yaw_pid.integral = 0.0f;
		attitude_correction = 0.0f;
		return;
	}

	error_rad = error * 0.017453292519943295f;
	mpc_rate_limit = mpc_yaw_rate_limit(target_error_deg *
	        0.017453292519943295f, yaw_rate_rad_s);
	omega_command = yaw_plan_rate_rad_s +
			attitude_control.yaw_pid.Kp * error_rad -
			attitude_control.yaw_pid.Kd * yaw_rate_rad_s;
	attitude_correction = ClampFloat(omega_command,
			-ATTITUDE_PID_MAX_OUTPUT, ATTITUDE_PID_MAX_OUTPUT);
	if (fabsf(attitude_correction) > mpc_rate_limit) {
		attitude_correction = (attitude_correction >= 0.0f) ?
				mpc_rate_limit : -mpc_rate_limit;
	}
}

/** 位姿协调模块。 */
void PoseCoordination_Init(void) {

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
	float yaw_rate_scale;

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


	pos_scale = distance_error_m / pose_coordination.slow_distance_m;
	yaw_scale = fabsf(yaw_error_deg) / pose_coordination.slow_yaw_deg;
	pos_scale = ClampFloat(pos_scale, pose_coordination.min_scale, 1.0f);
	yaw_scale = ClampFloat(yaw_scale, pose_coordination.min_scale, 1.0f);
	pose_coordination.sync_scale =
	        ClampFloat((pos_scale < yaw_scale) ? pos_scale : yaw_scale,
	                pose_coordination.min_scale, 1.0f);
	yaw_rate_scale = 1.0f - fabsf(motion_observer.yaw_rate_rad_s) /
	        POSE_SYNC_YAW_RATE_FULL_RAD_S;
	yaw_rate_scale = ClampFloat(yaw_rate_scale,
	        POSE_SYNC_YAW_RATE_MIN_SCALE, 1.0f);
	pose_coordination.sync_scale *= yaw_rate_scale;
}

uint8_t move_to_pose_target(float x, float y, float yaw) {
	static uint8_t pose_target_set = 0;

	if (!pose_target_set) {

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
void SetTargetVelocity(float vx, float vy, float omega) {
	motion_set_velocity(vx, vy, omega);
}

void SetTargetLinearVelocity(float vx, float vy) {
	motion_set_linear_velocity(vx, vy);
}
/** 麦轮逆解与电机速度输出模块。 */
void Mecanum_Init(ChassisConfig_t config) {

	chassis_config = config;
	if (chassis_config.max_rpm <= 0.0f) {
		chassis_config.max_rpm = CHASSIS_OPEN_LOOP_MAX_RPM;
	}
	k_omega = (config.wheel_dist_x + config.wheel_dist_y) / 2.0f;
	memset(control_ctx.motor, 0, sizeof(control_ctx.motor));
}

void Mecanum_Kinematics(float vx, float vy, float omega) {
	float actual_omega = omega;
	float max_rpm;

	if (attitude_control.is_enabled) {

		actual_omega += attitude_correction;

		if (actual_omega > ATTITUDE_MAX_OMEGA)
			actual_omega = ATTITUDE_MAX_OMEGA;
		if (actual_omega < -ATTITUDE_MAX_OMEGA)
			actual_omega = -ATTITUDE_MAX_OMEGA;
	}
	if (cam_stabilization.is_enabled) {

		vy += cam_correction_x * 0.01f;
		vx += cam_correction_y * 0.01f;
	}

	vx = -vx;
	vy = -vy;


	const float linear_to_rpm = 60.0f /
	        (2.0f * 3.1415926f * chassis_config.wheel_radius) *
	        CHASSIS_WHEEL_SPEED_SCALE;
	float translation_speed[4] = {vx - vy, vx + vy, vx + vy, vx - vy};
	float rotation_speed[4] = {-k_omega * actual_omega,
	        k_omega * actual_omega, -k_omega * actual_omega,
	        k_omega * actual_omega};
	float translation_scale = 1.0f;
	uint8_t wheel_index;

	max_rpm = ((is_position_control_enabled != 0U) ||
	        (attitude_control.is_enabled != 0U)) ?
	        CHASSIS_CLOSED_LOOP_MAX_RPM : chassis_config.max_rpm;
	for (wheel_index = 0U; wheel_index < 4U; wheel_index++) {
		if (translation_speed[wheel_index] > 0.0f) {
			float scale = (max_rpm / linear_to_rpm -
			        rotation_speed[wheel_index]) /
			        translation_speed[wheel_index];
			if (scale < translation_scale) {
				translation_scale = scale;
			}
		} else if (translation_speed[wheel_index] < 0.0f) {
			float scale = (-max_rpm / linear_to_rpm -
			        rotation_speed[wheel_index]) /
			        translation_speed[wheel_index];
			if (scale < translation_scale) {
				translation_scale = scale;
			}
		}
	}
	translation_scale = ClampFloat(translation_scale, 0.0f, 1.0f);
	float rpm4 = (translation_speed[0] * translation_scale +
	        rotation_speed[0]) * linear_to_rpm;
	float rpm2 = (translation_speed[1] * translation_scale +
	        rotation_speed[1]) * linear_to_rpm;
	float rpm3 = (translation_speed[2] * translation_scale +
	        rotation_speed[2]) * linear_to_rpm;
	float rpm1 = (translation_speed[3] * translation_scale +
	        rotation_speed[3]) * linear_to_rpm;


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
