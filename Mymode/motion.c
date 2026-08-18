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

/** 连续路径最大坐标点数量。 */
#define MOTION_PATH_MAX_POINTS 25U

/** 单段曲线长度估算采样数。 */
#define MOTION_PATH_LENGTH_SAMPLES 16U

/** 连续路径参考点沿圆弧的前视距离，单位 m。 */
#define MOTION_PATH_LOOKAHEAD_M 0.030f

/** 连续路径终点槽口位置容差，单位 m。 */
#define MOTION_PATH_TRACK_TOLERANCE_M 0.003f

/** 入弧姿态渐进对齐对应的圆心角，单位 rad。 */
#define MOTION_PATH_ENTRY_YAW_BLEND_RAD 0.515f

/**
 * @brief 连续路径运行状态。
 */
typedef struct
{
	float segment_length_m[MOTION_PATH_MAX_POINTS - 1U];
	float segment_u;
	float total_length_m;
	float projected_progress_m;
	float remaining_distance_m;
	float tangent_x;
	float tangent_y;
	float curvature_m_inv;
	float yaw_rate_rad_s;
	float yaw_rate_m_inv;
	float circle_angle_rad[MOTION_PATH_MAX_POINTS];
	float circle_radius_mm[MOTION_PATH_MAX_POINTS];
	float circle_yaw_rad[MOTION_PATH_MAX_POINTS];
	float circle_center_x_mm;
	float circle_center_y_mm;
	float circle_base_radius_mm;
	float circle_slot_x_mm;
	float circle_slot_y_mm;
	uint8_t point_count;
	uint8_t segment_index;
	uint8_t active;
	uint8_t reference_complete;
	uint8_t completed;
} MotionPathState_t;

/** 连续路径运行状态。 */
static MotionPathState_t motion_path;

/**
 * @brief 点到点二维 MPC 参考状态。
 */
typedef struct
{
	float start_position_x_m;
	float start_position_y_m;
	float position_x_m;
	float position_y_m;
	float velocity_x_mps;
	float velocity_y_mps;
	float filtered_velocity_x_mps;
	float filtered_velocity_y_mps;
	float filtered_position_x_m;
	float filtered_position_y_m;
	float accel_x_mps2;
	float accel_y_mps2;
	float update_accumulator_s;
	uint8_t initialized;
	uint8_t reference_x_complete;
	uint8_t reference_y_complete;
	uint8_t reference_complete;
} PositionMpcState_t;

static PositionMpcState_t position_mpc;
static uint8_t position_arrival_latched;

/** 二维 MPC 非均匀加速度候选，兼顾大范围响应与小误差收敛。 */
static const float position_mpc_accel_candidate_mps2[] = {
	-12.0f, -6.0f, -2.0f, -POSITION_MPC_MIN_ACCEL_MPS2, 0.0f,
	POSITION_MPC_MIN_ACCEL_MPS2, 2.0f, 6.0f, 12.0f
};
#define POSITION_MPC_ACCEL_CANDIDATE_COUNT 9U

/**
 * @brief 根据大圆和槽口几何计算底盘中心曲线。
 *
 * 物料径向偏差通过三次缓入缓出函数连接，槽口在每个路径点与物料中心重合。
 *
 * @param segment_index 曲线段索引。
 * @param u 曲线段参数，范围 0 至 1。
 * @param x_mm 底盘中心 X 坐标输出，单位 mm。
 * @param y_mm 底盘中心 Y 坐标输出，单位 mm。
 * @param dx_du X 坐标一阶导数输出。
 * @param dy_du Y 坐标一阶导数输出。
 * @param d2x_du2 X 坐标二阶导数输出。
 * @param d2y_du2 Y 坐标二阶导数输出。
 */
static void motion_circle_eval(uint8_t segment_index, float u,
		float *x_mm, float *y_mm, float *dx_du, float *dy_du,
		float *d2x_du2, float *d2y_du2)
{
	float start_angle_rad = motion_path.circle_angle_rad[segment_index];
	float angle_delta_rad = motion_path.circle_angle_rad[segment_index + 1U] -
	        start_angle_rad;
	float angle_rad = start_angle_rad + angle_delta_rad * u;
	float start_offset_mm = motion_path.circle_radius_mm[segment_index] -
	        motion_path.circle_base_radius_mm;
	float end_offset_mm = motion_path.circle_radius_mm[segment_index + 1U] -
	        motion_path.circle_base_radius_mm;
	float offset_delta_mm = end_offset_mm - start_offset_mm;
	float blend = 3.0f * u * u - 2.0f * u * u * u;
	float radius_mm = motion_path.circle_base_radius_mm + start_offset_mm +
	        offset_delta_mm * blend;
	float radius_du_mm = offset_delta_mm * (6.0f * u - 6.0f * u * u);
	float radius_d2u_mm = offset_delta_mm * (6.0f - 12.0f * u);
	float angle_cos = cosf(angle_rad);
	float angle_sin = sinf(angle_rad);
	float radial_accel_mm = radius_d2u_mm -
	        radius_mm * angle_delta_rad * angle_delta_rad;
	float tangent_accel_mm = 2.0f * radius_du_mm * angle_delta_rad;
	float start_yaw_rad = motion_path.circle_yaw_rad[segment_index];
	float yaw_delta_rad = motion_path.circle_yaw_rad[segment_index + 1U] -
	        start_yaw_rad;
	float slot_yaw_rad = start_yaw_rad + yaw_delta_rad * u;
	float yaw_cos = cosf(slot_yaw_rad);
	float yaw_sin = sinf(slot_yaw_rad);
	float slot_world_x_mm = yaw_cos * motion_path.circle_slot_x_mm -
	        yaw_sin * motion_path.circle_slot_y_mm;
	float slot_world_y_mm = yaw_sin * motion_path.circle_slot_x_mm +
	        yaw_cos * motion_path.circle_slot_y_mm;
	float slot_angle_accel = yaw_delta_rad * yaw_delta_rad;

	/* 槽口目标曲线反解底盘中心，同时对旋转偏置求一、二阶导数。 */
	*x_mm = motion_path.circle_center_x_mm + radius_mm * angle_cos -
	        slot_world_x_mm;
	*y_mm = motion_path.circle_center_y_mm + radius_mm * angle_sin -
	        slot_world_y_mm;
	*dx_du = radius_du_mm * angle_cos -
	        radius_mm * angle_delta_rad * angle_sin +
	        slot_world_y_mm * yaw_delta_rad;
	*dy_du = radius_du_mm * angle_sin +
	        radius_mm * angle_delta_rad * angle_cos -
	        slot_world_x_mm * yaw_delta_rad;
	*d2x_du2 = radial_accel_mm * angle_cos -
	        tangent_accel_mm * angle_sin +
	        slot_world_x_mm * slot_angle_accel;
	*d2y_du2 = radial_accel_mm * angle_sin +
	        tangent_accel_mm * angle_cos +
	        slot_world_y_mm * slot_angle_accel;
}

/**
 * @brief 计算当前大圆路径的位置、一阶导数和二阶导数。
 *
 * @param segment_index 曲线段索引。
 * @param u 曲线段参数，范围 0 至 1。
 * @param x_mm X 坐标输出，单位 mm。
 * @param y_mm Y 坐标输出，单位 mm。
 * @param dx_du X 坐标一阶导数输出。
 * @param dy_du Y 坐标一阶导数输出。
 * @param d2x_du2 X 坐标二阶导数输出。
 * @param d2y_du2 Y 坐标二阶导数输出。
 */
static void motion_path_eval(uint8_t segment_index, float u,
		float *x_mm, float *y_mm, float *dx_du, float *dy_du,
		float *d2x_du2, float *d2y_du2)
{
	motion_circle_eval(segment_index, u, x_mm, y_mm, dx_du, dy_du,
	        d2x_du2, d2y_du2);
}

/**
 * @brief 估算一段三次曲线的弧长。
 *
 * @param segment_index 曲线段索引。
 * @return 曲线段长度，单位 m。
 */
static float motion_path_segment_length(uint8_t segment_index)
{
	float last_x_mm;
	float last_y_mm;
	float dx_du;
	float dy_du;
	float d2x_du2;
	float d2y_du2;
	float length_mm = 0.0f;
	uint8_t sample_index;

	motion_path_eval(segment_index, 0.0f, &last_x_mm, &last_y_mm,
	        &dx_du, &dy_du, &d2x_du2, &d2y_du2);
	for (sample_index = 1U; sample_index <= MOTION_PATH_LENGTH_SAMPLES;
	        sample_index++) {
		float x_mm;
		float y_mm;
		float u = (float)sample_index /
		        (float)MOTION_PATH_LENGTH_SAMPLES;

		motion_path_eval(segment_index, u, &x_mm, &y_mm,
		        &dx_du, &dy_du, &d2x_du2, &d2y_du2);
		length_mm += sqrtf((x_mm - last_x_mm) * (x_mm - last_x_mm) +
		        (y_mm - last_y_mm) * (y_mm - last_y_mm));
		last_x_mm = x_mm;
		last_y_mm = y_mm;
	}

	return length_mm * 0.001f;
}

/**
 * @brief 更新当前位置参考、切线和曲率。
 */
static void motion_path_reference_update(void)
{
	float x_mm;
	float y_mm;
	float dx_du;
	float dy_du;
	float d2x_du2;
	float d2y_du2;
	float derivative_norm;
	float curvature_denominator;

	motion_path_eval(motion_path.segment_index, motion_path.segment_u,
	        &x_mm, &y_mm, &dx_du, &dy_du, &d2x_du2, &d2y_du2);
	derivative_norm = sqrtf(dx_du * dx_du + dy_du * dy_du);
	if (derivative_norm < 0.001f) {
		return;
	}

	position_control.target_x = x_mm;
	position_control.target_y = y_mm;
	motion_path.tangent_x = dx_du / derivative_norm;
	motion_path.tangent_y = dy_du / derivative_norm;
	curvature_denominator = derivative_norm * derivative_norm *
	        derivative_norm;
	/* 导数单位为 mm，曲率乘 1000 后转换为 1/m。 */
	motion_path.curvature_m_inv = 1000.0f *
	        (dx_du * d2y_du2 - dy_du * d2x_du2) /
	        curvature_denominator;
	{
		float start_yaw_rad = motion_path.circle_yaw_rad[
		        motion_path.segment_index];
		float end_yaw_rad = motion_path.circle_yaw_rad[
		        motion_path.segment_index + 1U];
		float yaw_rad = start_yaw_rad +
		        (end_yaw_rad - start_yaw_rad) * motion_path.segment_u;

		attitude_control.target_yaw = yaw_rad * 57.29577951308232f;
		motion_path.yaw_rate_m_inv = (end_yaw_rad - start_yaw_rad) /
		        motion_path.segment_length_m[motion_path.segment_index];
	}
}

/**
 * @brief 将槽口极角投影为路径弧长进度。
 *
 * @param angle_rad 槽口相对圆心的极角，单位 rad。
 * @return 路径投影进度，单位 m。
 */
static float motion_path_project_progress(float angle_rad)
{
	float reference_angle_rad = motion_path.circle_angle_rad[
	        motion_path.segment_index] +
	        (motion_path.circle_angle_rad[motion_path.segment_index + 1U] -
	                motion_path.circle_angle_rad[motion_path.segment_index]) *
	                motion_path.segment_u;
	float path_angle_delta_rad = motion_path.circle_angle_rad[
	        motion_path.point_count - 1U] - motion_path.circle_angle_rad[0U];
	float progress_m = 0.0f;
	uint8_t segment_index;

	/* 将传感器极角展开到当前参考角附近，保持跨越正负 180° 时连续。 */
	while ((angle_rad - reference_angle_rad) > 3.1415926535897932f) {
		angle_rad -= 6.2831853071795865f;
	}
	while ((angle_rad - reference_angle_rad) <= -3.1415926535897932f) {
		angle_rad += 6.2831853071795865f;
	}

	if (((path_angle_delta_rad > 0.0f) &&
	        (angle_rad <= motion_path.circle_angle_rad[0U])) ||
	        ((path_angle_delta_rad < 0.0f) &&
	                (angle_rad >= motion_path.circle_angle_rad[0U]))) {
		return 0.0f;
	}
	if (((path_angle_delta_rad > 0.0f) &&
	        (angle_rad >= motion_path.circle_angle_rad[
	                motion_path.point_count - 1U])) ||
	        ((path_angle_delta_rad < 0.0f) &&
	                (angle_rad <= motion_path.circle_angle_rad[
	                        motion_path.point_count - 1U]))) {
		return motion_path.total_length_m;
	}

	for (segment_index = 0U;
	        segment_index < (uint8_t)(motion_path.point_count - 1U);
	        segment_index++) {
		float start_angle_rad = motion_path.circle_angle_rad[segment_index];
		float end_angle_rad = motion_path.circle_angle_rad[segment_index + 1U];
		float angle_delta_rad = end_angle_rad - start_angle_rad;

		if (((angle_delta_rad > 0.0f) && (angle_rad <= end_angle_rad)) ||
		        ((angle_delta_rad < 0.0f) && (angle_rad >= end_angle_rad))) {
			float segment_u = (angle_rad - start_angle_rad) /
			        angle_delta_rad;

			return progress_m + motion_path.segment_length_m[segment_index] *
			        ClampFloat(segment_u, 0.0f, 1.0f);
		}
		progress_m += motion_path.segment_length_m[segment_index];
	}

	return motion_path.total_length_m;
}

/**
 * @brief 根据弧长进度更新连续路径参考点。
 *
 * @param progress_m 目标弧长进度，单位 m。
 */
static void motion_path_set_progress(float progress_m)
{
	uint8_t reference_complete =
	        (progress_m >= motion_path.total_length_m);
	uint8_t segment_index;

	progress_m = ClampFloat(progress_m, 0.0f, motion_path.total_length_m);
	for (segment_index = 0U;
	        segment_index < (uint8_t)(motion_path.point_count - 1U);
	        segment_index++) {
		float segment_length_m = motion_path.segment_length_m[segment_index];

		if ((progress_m <= segment_length_m) ||
		        (segment_index >= (uint8_t)(motion_path.point_count - 2U))) {
			motion_path.segment_index = segment_index;
			motion_path.segment_u = ClampFloat(progress_m / segment_length_m,
			        0.0f, 1.0f);
			break;
		}
		progress_m -= segment_length_m;
	}

	motion_path.reference_complete = reference_complete;
	motion_path_reference_update();
}

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

	motion_path_stop();
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

/**
 * @brief 完成路径弧长计算并启动底盘闭环。
 *
 * @param cruise_speed_mps 巡航速度，单位 m/s。
 * @return 1 表示启动成功，0 表示路径无效。
 */
static uint8_t motion_path_begin(float cruise_speed_mps)
{
	float total_length_m = 0.0f;
	float dx_du;
	float dy_du;
	float d2x_du2;
	float d2y_du2;
	float start_x_mm;
	float start_y_mm;
	float start_yaw_deg;
	uint8_t segment_index;

	for (segment_index = 0U;
	        segment_index < (uint8_t)(motion_path.point_count - 1U);
	        segment_index++) {
		motion_path.segment_length_m[segment_index] =
		        motion_path_segment_length(segment_index);
		if (motion_path.segment_length_m[segment_index] <= 0.001f) {
			motion_path_stop();
			return 0U;
		}
		total_length_m += motion_path.segment_length_m[segment_index];
	}
	if (total_length_m <= 0.01f) {
		motion_path_stop();
		return 0U;
	}

	motion_path.segment_index = 0U;
	motion_path.segment_u = 0.0f;
	motion_path.total_length_m = total_length_m;
	motion_path.projected_progress_m = 0.0f;
	motion_path.remaining_distance_m = total_length_m;
	motion_path.active = 1U;
	motion_path.reference_complete = 0U;
	motion_path.completed = 0U;
	motion_path_reference_update();
	start_yaw_deg = attitude_control.target_yaw;
	motion_path_eval(0U, 0.0f, &start_x_mm, &start_y_mm,
	        &dx_du, &dy_du, &d2x_du2, &d2y_du2);
	SetTargetPosition(start_x_mm, start_y_mm);
	PositionControl_SetMaxSpeed(cruise_speed_mps);
	PoseCoordination_Enable(0U);
	PositionControl_Enable(1U);
	EnableAttitudeStabilization(1U);
	SetTargetYaw(start_yaw_deg);
	return 1U;
}

uint8_t motion_circle_path_start(const MotionCirclePath_t *path)
{
	float previous_angle_rad = 0.0f;
	float entry_yaw_rad;
	float entry_yaw_offset_rad;
	uint8_t point_index;

	if ((path == NULL) || (path->material_points == NULL) ||
	        (path->point_count < 2U) ||
	        (path->point_count > MOTION_PATH_MAX_POINTS) ||
	        (path->radius_mm <= 0.0f) ||
	        ((path->direction != 1) && (path->direction != -1)) ||
	        (path->cruise_speed_mps <= 0.0f)) {
		return 0U;
	}

	motion_path_stop();
	for (point_index = 0U; point_index < path->point_count; point_index++) {
		float radius_x_mm = path->material_points[point_index].x_mm -
		        path->center_x_mm;
		float radius_y_mm = path->material_points[point_index].y_mm -
		        path->center_y_mm;
		float angle_rad = atan2f(radius_y_mm, radius_x_mm);
		float point_radius_mm = sqrtf(radius_x_mm * radius_x_mm +
		        radius_y_mm * radius_y_mm);

		if (point_radius_mm < 1.0f) {
			motion_path_stop();
			return 0U;
		}
		if (point_index > 0U) {
			if (path->direction > 0) {
				while (angle_rad <= previous_angle_rad) {
					angle_rad += 6.2831853071795865f;
				}
			} else {
				while (angle_rad >= previous_angle_rad) {
					angle_rad -= 6.2831853071795865f;
				}
			}
		}

		motion_path.circle_angle_rad[point_index] = angle_rad;
		motion_path.circle_radius_mm[point_index] = point_radius_mm;
		motion_path.circle_yaw_rad[point_index] = angle_rad -
		        (float)path->direction * 1.5707963267948966f;
		previous_angle_rad = angle_rad;
	}
	/* 入弧姿态按固定圆心角渐进对齐，使转向过程不受锚点密度影响。 */
	entry_yaw_rad = odometer_get_yaw_deg() * 0.017453292519943295f;
	while ((entry_yaw_rad - motion_path.circle_yaw_rad[0U]) >
	        3.1415926535897932f) {
		entry_yaw_rad -= 6.2831853071795865f;
	}
	while ((entry_yaw_rad - motion_path.circle_yaw_rad[0U]) <=
	        -3.1415926535897932f) {
		entry_yaw_rad += 6.2831853071795865f;
	}
	entry_yaw_offset_rad = entry_yaw_rad - motion_path.circle_yaw_rad[0U];
	for (point_index = 0U; point_index < path->point_count; point_index++) {
		float angle_progress_rad = fabsf(motion_path.circle_angle_rad[point_index] -
		        motion_path.circle_angle_rad[0U]);
		float entry_blend = ClampFloat(angle_progress_rad /
		        MOTION_PATH_ENTRY_YAW_BLEND_RAD, 0.0f, 1.0f);

		motion_path.circle_yaw_rad[point_index] += entry_yaw_offset_rad *
		        (1.0f - entry_blend);
	}

	motion_path.point_count = path->point_count;
	motion_path.circle_center_x_mm = path->center_x_mm;
	motion_path.circle_center_y_mm = path->center_y_mm;
	motion_path.circle_base_radius_mm = path->radius_mm;
	motion_path.circle_slot_x_mm = path->slot_x_mm;
	motion_path.circle_slot_y_mm = path->slot_y_mm;
	return motion_path_begin(path->cruise_speed_mps);
}

void motion_path_update(float dt)
{
	float target_yaw_rad;
	float current_yaw_rad;
	float target_slot_x_mm;
	float target_slot_y_mm;
	float current_slot_x_mm;
	float current_slot_y_mm;
	float slot_error_x_m;
	float slot_error_y_m;
	float slot_error_m;
	float current_slot_angle_rad;
	float projected_progress_m;
	float reference_progress_m;

	if (!motion_path.active || (dt <= 0.0f)) {
		return;
	}

	/* 实际槽口投影确定沿弧进度，固定前视距离生成连续底盘参考点。 */
	current_yaw_rad = odometer_get_yaw_deg() * 0.017453292519943295f;
	current_slot_x_mm = odometer_ctx.pose.pos_x_mm +
	        cosf(current_yaw_rad) * motion_path.circle_slot_x_mm -
	        sinf(current_yaw_rad) * motion_path.circle_slot_y_mm;
	current_slot_y_mm = odometer_ctx.pose.pos_y_mm +
	        sinf(current_yaw_rad) * motion_path.circle_slot_x_mm +
	        cosf(current_yaw_rad) * motion_path.circle_slot_y_mm;
	current_slot_angle_rad = atan2f(
	        current_slot_y_mm - motion_path.circle_center_y_mm,
	        current_slot_x_mm - motion_path.circle_center_x_mm);
	projected_progress_m = motion_path_project_progress(current_slot_angle_rad);
	if (projected_progress_m > motion_path.projected_progress_m) {
		motion_path.projected_progress_m = projected_progress_m;
	}
	reference_progress_m = motion_path.projected_progress_m +
	        MOTION_PATH_LOOKAHEAD_M;
	motion_path_set_progress(reference_progress_m);
	motion_path.remaining_distance_m = motion_path.total_length_m -
	        motion_path.projected_progress_m;
	motion_path.yaw_rate_rad_s = motion_path.reference_complete ? 0.0f :
	        motion_plan_speed() * motion_path.yaw_rate_m_inv;

	target_yaw_rad = attitude_control.target_yaw * 0.017453292519943295f;
	target_slot_x_mm = position_control.target_x +
	        cosf(target_yaw_rad) * motion_path.circle_slot_x_mm -
	        sinf(target_yaw_rad) * motion_path.circle_slot_y_mm;
	target_slot_y_mm = position_control.target_y +
	        sinf(target_yaw_rad) * motion_path.circle_slot_x_mm +
	        cosf(target_yaw_rad) * motion_path.circle_slot_y_mm;
	slot_error_x_m = (target_slot_x_mm - current_slot_x_mm) * 0.001f;
	slot_error_y_m = (target_slot_y_mm - current_slot_y_mm) * 0.001f;
	slot_error_m = sqrtf(slot_error_x_m * slot_error_x_m +
	        slot_error_y_m * slot_error_y_m);
	/* 按实测 yaw 反算底盘中心，使位置环误差直接等于槽口轨迹误差。 */
	position_control.target_x = odometer_ctx.pose.pos_x_mm +
	        slot_error_x_m * 1000.0f;
	position_control.target_y = odometer_ctx.pose.pos_y_mm +
	        slot_error_y_m * 1000.0f;

	if (motion_path.reference_complete &&
	        (slot_error_m <= MOTION_PATH_TRACK_TOLERANCE_M)) {
		motion_path.active = 0U;
		motion_path.completed = 1U;
		motion_path.yaw_rate_rad_s = 0.0f;
		PositionControl_Enable(0U);
		EnableAttitudeStabilization(0U);
		stop();
	}
}

uint8_t motion_path_finished(void)
{
	return motion_path.completed;
}

void motion_path_stop(void)
{
	motion_path.point_count = 0U;
	motion_path.segment_index = 0U;
	motion_path.segment_u = 0.0f;
	motion_path.total_length_m = 0.0f;
	motion_path.projected_progress_m = 0.0f;
	motion_path.remaining_distance_m = 0.0f;
	motion_path.tangent_x = 0.0f;
	motion_path.tangent_y = 0.0f;
	motion_path.curvature_m_inv = 0.0f;
	motion_path.yaw_rate_rad_s = 0.0f;
	motion_path.yaw_rate_m_inv = 0.0f;
	motion_path.circle_center_x_mm = 0.0f;
	motion_path.circle_center_y_mm = 0.0f;
	motion_path.circle_base_radius_mm = 0.0f;
	motion_path.circle_slot_x_mm = 0.0f;
	motion_path.circle_slot_y_mm = 0.0f;
	motion_path.active = 0U;
	motion_path.reference_complete = 0U;
	motion_path.completed = 0U;
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

/**
 * @brief 限制二维速度矢量模长。
 */
static void position_mpc_limit_speed(float *velocity_x_mps,
		float *velocity_y_mps, float max_speed_mps)
{
	float speed_mps = sqrtf(*velocity_x_mps * *velocity_x_mps +
	        *velocity_y_mps * *velocity_y_mps);

	if (speed_mps > max_speed_mps) {
		float scale = max_speed_mps / speed_mps;

		*velocity_x_mps *= scale;
		*velocity_y_mps *= scale;
	}
}

/**
 * @brief 限制二维加速度矢量模长。
 */
static void position_mpc_limit_accel(float *accel_x_mps2,
		float *accel_y_mps2)
{
	float accel_mps2 = sqrtf(*accel_x_mps2 * *accel_x_mps2 +
	        *accel_y_mps2 * *accel_y_mps2);

	if (accel_mps2 > MOTION_MPC_MAX_ACCEL_MPS2) {
		float scale = MOTION_MPC_MAX_ACCEL_MPS2 / accel_mps2;

		*accel_x_mps2 *= scale;
		*accel_y_mps2 *= scale;
	}
}

/**
 * @brief 限制二维加速度单周期变化量。
 */
static void position_mpc_limit_accel_change(float *delta_accel_x_mps2,
		float *delta_accel_y_mps2)
{
	float delta_accel_mps2 = sqrtf(*delta_accel_x_mps2 *
	        *delta_accel_x_mps2 + *delta_accel_y_mps2 *
	        *delta_accel_y_mps2);
	float max_delta_accel_mps2 = POSITION_MPC_MAX_JERK_MPS3 *
	        POSITION_MPC_PERIOD_S;

	if (delta_accel_mps2 > max_delta_accel_mps2) {
		float scale = max_delta_accel_mps2 / delta_accel_mps2;

		*delta_accel_x_mps2 *= scale;
		*delta_accel_y_mps2 *= scale;
	}
}

/**
 * @brief 将单轴 MPC 参考位置限制在本次运动目标点。
 *
 * @return 参考位置到达目标点时返回 1，否则返回 0。
 */
static uint8_t position_mpc_clamp_target(float start_position_m,
		float target_position_m, float *reference_position_m)
{
	if (((target_position_m >= start_position_m) &&
	        (*reference_position_m >= target_position_m)) ||
	        ((target_position_m < start_position_m) &&
	        (*reference_position_m <= target_position_m))) {
		*reference_position_m = target_position_m;
		return 1U;
	}
	return 0U;
}

/**
 * @brief 对二维 MPC 规划速度执行一阶低通。
 */
static void position_mpc_filter_velocity(void)
{
	position_mpc.filtered_velocity_x_mps +=
	        POSITION_MPC_VELOCITY_FILTER_ALPHA *
	        (position_mpc.velocity_x_mps - position_mpc.filtered_velocity_x_mps);
	position_mpc.filtered_velocity_y_mps +=
	        POSITION_MPC_VELOCITY_FILTER_ALPHA *
	        (position_mpc.velocity_y_mps - position_mpc.filtered_velocity_y_mps);
	position_mpc.filtered_position_x_m +=
	        POSITION_MPC_VELOCITY_FILTER_ALPHA *
	        (position_mpc.position_x_m - position_mpc.filtered_position_x_m);
	position_mpc.filtered_position_y_m +=
	        POSITION_MPC_VELOCITY_FILTER_ALPHA *
	        (position_mpc.position_y_m - position_mpc.filtered_position_y_m);
}

/**
 * @brief 以当前实测状态重置二维 MPC 参考。
 */
static void position_mpc_reset(float position_x_m, float position_y_m)
{
	memset(&position_mpc, 0, sizeof(position_mpc));
	position_mpc.start_position_x_m = position_x_m;
	position_mpc.start_position_y_m = position_y_m;
	position_mpc.position_x_m = position_x_m;
	position_mpc.position_y_m = position_y_m;
	position_mpc.velocity_x_mps = odometer_ctx.motion.vel_x_mps;
	position_mpc.velocity_y_mps = odometer_ctx.motion.vel_y_mps;
	position_mpc.filtered_velocity_x_mps = position_mpc.velocity_x_mps;
	position_mpc.filtered_velocity_y_mps = position_mpc.velocity_y_mps;
	position_mpc.filtered_position_x_m = position_x_m;
	position_mpc.filtered_position_y_m = position_y_m;
	position_mpc.update_accumulator_s = POSITION_MPC_PERIOD_S;
	position_mpc.initialized = 1U;
}

/**
 * @brief 按固定周期求解二维位置 MPC 并更新位置、速度、加速度参考。
 */
static void position_mpc_update(float current_x_m, float current_y_m,
		float target_x_m, float target_y_m, float dt)
{
	float current_velocity_x_mps = odometer_ctx.motion.vel_x_mps;
	float current_velocity_y_mps = odometer_ctx.motion.vel_y_mps;
	float max_speed_mps = motion_plan_cruise();

	if (position_mpc.initialized == 0U) {
		position_mpc_reset(current_x_m, current_y_m);
	}
	position_mpc.update_accumulator_s += dt;
	if (position_mpc.update_accumulator_s < POSITION_MPC_PERIOD_S) {
		return;
	}
	position_mpc.update_accumulator_s -= POSITION_MPC_PERIOD_S;
	if (position_arrival_latched != 0U) {
		position_mpc.position_x_m = current_x_m;
		position_mpc.position_y_m = current_y_m;
		position_mpc.velocity_x_mps = 0.0f;
		position_mpc.velocity_y_mps = 0.0f;
		position_mpc.filtered_velocity_x_mps = 0.0f;
		position_mpc.filtered_velocity_y_mps = 0.0f;
		position_mpc.filtered_position_x_m = current_x_m;
		position_mpc.filtered_position_y_m = current_y_m;
		position_mpc.accel_x_mps2 = 0.0f;
		position_mpc.accel_y_mps2 = 0.0f;
		return;
	}
	if (position_mpc.reference_complete != 0U) {
		position_mpc.position_x_m = target_x_m;
		position_mpc.position_y_m = target_y_m;
		position_mpc.velocity_x_mps = 0.0f;
		position_mpc.velocity_y_mps = 0.0f;
		position_mpc.filtered_velocity_x_mps = 0.0f;
		position_mpc.filtered_velocity_y_mps = 0.0f;
		position_mpc.filtered_position_x_m = target_x_m;
		position_mpc.filtered_position_y_m = target_y_m;
		position_mpc.accel_x_mps2 = 0.0f;
		position_mpc.accel_y_mps2 = 0.0f;
		return;
	}
	{
		float best_cost = 1.0e30f;
		float best_target_accel_x_mps2 = 0.0f;
		float best_target_accel_y_mps2 = 0.0f;
		float target_error_x_m = target_x_m - current_x_m;
		float target_error_y_m = target_y_m - current_y_m;
		uint8_t candidate_x;
		uint8_t candidate_y;

		for (candidate_x = 0U;
		        candidate_x < POSITION_MPC_ACCEL_CANDIDATE_COUNT;
		        candidate_x++) {
			for (candidate_y = 0U;
			        candidate_y < POSITION_MPC_ACCEL_CANDIDATE_COUNT;
			        candidate_y++) {
				float target_accel_x_mps2 =
				        position_mpc_accel_candidate_mps2[candidate_x];
				float target_accel_y_mps2 =
				        position_mpc_accel_candidate_mps2[candidate_y];
				float predicted_x_m = current_x_m;
				float predicted_y_m = current_y_m;
				float predicted_vx_mps = current_velocity_x_mps;
				float predicted_vy_mps = current_velocity_y_mps;
				float predicted_accel_x_mps2 = position_mpc.accel_x_mps2;
				float predicted_accel_y_mps2 = position_mpc.accel_y_mps2;
				float cost = 0.0f;
				uint8_t step;

				position_mpc_limit_accel(&target_accel_x_mps2,
				        &target_accel_y_mps2);
				for (step = 0U; step < POSITION_MPC_HORIZON_STEPS; step++) {
					float delta_accel_x_mps2 = target_accel_x_mps2 -
					        predicted_accel_x_mps2;
					float delta_accel_y_mps2 = target_accel_y_mps2 -
					        predicted_accel_y_mps2;
					float error_x_m;
					float error_y_m;

					position_mpc_limit_accel_change(&delta_accel_x_mps2,
					        &delta_accel_y_mps2);
					predicted_accel_x_mps2 += delta_accel_x_mps2;
					predicted_accel_y_mps2 += delta_accel_y_mps2;
					predicted_x_m += predicted_vx_mps * POSITION_MPC_PERIOD_S +
					        0.5f * predicted_accel_x_mps2 * POSITION_MPC_PERIOD_S *
					        POSITION_MPC_PERIOD_S;
					predicted_y_m += predicted_vy_mps * POSITION_MPC_PERIOD_S +
					        0.5f * predicted_accel_y_mps2 * POSITION_MPC_PERIOD_S *
					        POSITION_MPC_PERIOD_S;
					predicted_vx_mps += predicted_accel_x_mps2 *
					        POSITION_MPC_PERIOD_S;
					predicted_vy_mps += predicted_accel_y_mps2 *
					        POSITION_MPC_PERIOD_S;
					position_mpc_limit_speed(&predicted_vx_mps,
					        &predicted_vy_mps, max_speed_mps);
					error_x_m = target_x_m - predicted_x_m;
					error_y_m = target_y_m - predicted_y_m;
					cost += POSITION_MPC_STAGE_POSITION_WEIGHT *
					        (error_x_m * error_x_m + error_y_m * error_y_m);
					if ((error_x_m * target_error_x_m +
					        error_y_m * target_error_y_m) <= 0.0f) {
						cost += POSITION_MPC_OVERSHOOT_WEIGHT;
					}
					cost += POSITION_MPC_ACCEL_CHANGE_WEIGHT *
					        (delta_accel_x_mps2 * delta_accel_x_mps2 +
					        delta_accel_y_mps2 * delta_accel_y_mps2);
				}
				{
					float error_x_m = target_x_m - predicted_x_m;
					float error_y_m = target_y_m - predicted_y_m;

					cost += POSITION_MPC_TERMINAL_POSITION_WEIGHT *
					        (error_x_m * error_x_m + error_y_m * error_y_m);
				}
				cost += POSITION_MPC_TERMINAL_SPEED_WEIGHT *
				        (predicted_vx_mps * predicted_vx_mps +
				        predicted_vy_mps * predicted_vy_mps) +
				        POSITION_MPC_ACCEL_WEIGHT *
				        (target_accel_x_mps2 * target_accel_x_mps2 +
				        target_accel_y_mps2 * target_accel_y_mps2);
				if (cost < best_cost) {
					best_cost = cost;
					best_target_accel_x_mps2 = target_accel_x_mps2;
					best_target_accel_y_mps2 = target_accel_y_mps2;
				}
			}
		}
		{
			float best_accel_mps2 = sqrtf(best_target_accel_x_mps2 *
			        best_target_accel_x_mps2 + best_target_accel_y_mps2 *
			        best_target_accel_y_mps2);
			float target_distance_m = sqrtf(target_error_x_m * target_error_x_m +
			        target_error_y_m * target_error_y_m);
			float delta_accel_x_mps2;
			float delta_accel_y_mps2;

			if ((best_accel_mps2 < POSITION_MPC_MIN_ACCEL_MPS2) &&
			        (target_distance_m > raceh)) {
				best_target_accel_x_mps2 = POSITION_MPC_MIN_ACCEL_MPS2 *
				        target_error_x_m / target_distance_m;
				best_target_accel_y_mps2 = POSITION_MPC_MIN_ACCEL_MPS2 *
				        target_error_y_m / target_distance_m;
			}
			delta_accel_x_mps2 = best_target_accel_x_mps2 -
			        position_mpc.accel_x_mps2;
			delta_accel_y_mps2 = best_target_accel_y_mps2 -
			        position_mpc.accel_y_mps2;
			position_mpc_limit_accel_change(&delta_accel_x_mps2,
			        &delta_accel_y_mps2);
			position_mpc.accel_x_mps2 += delta_accel_x_mps2;
			position_mpc.accel_y_mps2 += delta_accel_y_mps2;
		}
	}
	{
		position_mpc.position_x_m = current_x_m + current_velocity_x_mps *
		        POSITION_MPC_PERIOD_S + 0.5f * position_mpc.accel_x_mps2 *
		        POSITION_MPC_PERIOD_S * POSITION_MPC_PERIOD_S;
		position_mpc.position_y_m = current_y_m + current_velocity_y_mps *
		        POSITION_MPC_PERIOD_S + 0.5f * position_mpc.accel_y_mps2 *
		        POSITION_MPC_PERIOD_S * POSITION_MPC_PERIOD_S;
		position_mpc.velocity_x_mps = current_velocity_x_mps +
		        position_mpc.accel_x_mps2 *
		        POSITION_MPC_PERIOD_S;
		position_mpc.velocity_y_mps = current_velocity_y_mps +
		        position_mpc.accel_y_mps2 *
		        POSITION_MPC_PERIOD_S;
		position_mpc_limit_speed(&position_mpc.velocity_x_mps,
		        &position_mpc.velocity_y_mps, max_speed_mps);
		if ((position_mpc.reference_x_complete != 0U) ||
		        position_mpc_clamp_target(position_mpc.start_position_x_m,
		        target_x_m, &position_mpc.position_x_m)) {
			position_mpc.position_x_m = target_x_m;
			position_mpc.velocity_x_mps = 0.0f;
			position_mpc.filtered_velocity_x_mps = 0.0f;
			position_mpc.filtered_position_x_m = target_x_m;
			position_mpc.accel_x_mps2 = 0.0f;
			position_mpc.reference_x_complete = 1U;
		}
		if ((position_mpc.reference_y_complete != 0U) ||
		        position_mpc_clamp_target(position_mpc.start_position_y_m,
		        target_y_m, &position_mpc.position_y_m)) {
			position_mpc.position_y_m = target_y_m;
			position_mpc.velocity_y_mps = 0.0f;
			position_mpc.filtered_velocity_y_mps = 0.0f;
			position_mpc.filtered_position_y_m = target_y_m;
			position_mpc.accel_y_mps2 = 0.0f;
			position_mpc.reference_y_complete = 1U;
		}
		if ((position_mpc.reference_x_complete != 0U) &&
		        (position_mpc.reference_y_complete != 0U)) {
			position_mpc.reference_complete = 1U;
			return;
		}
	}
	position_mpc_filter_velocity();
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

	pid_init(&position_control.pid_x, POSITION_PID_DEFAULT_KP,
	        POSITION_PID_DEFAULT_KI,
	        POSITION_PID_DEFAULT_KD, HIGHESTSPEED);

	pid_init(&position_control.pid_y, POSITION_PID_DEFAULT_KP,
	        POSITION_PID_DEFAULT_KI,
	        POSITION_PID_DEFAULT_KD, HIGHESTSPEED);
	pid_set_integral_limit(&position_control.pid_x,
	        POSITION_PID_INTEGRAL_LIMIT_M_S);
	pid_set_integral_limit(&position_control.pid_y,
	        POSITION_PID_INTEGRAL_LIMIT_M_S);

	position_control.is_enabled = 0;
	position_control.target_x = 0.0f;
	position_control.target_y = 0.0f;
	position_control.last_update_time = HAL_GetTick();
	motion_plan_init(POSITION_PLANNER_CRUISE_SPEED_MPS);
	memset(&position_mpc, 0, sizeof(position_mpc));
	position_arrival_latched = 0U;

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
	position_arrival_latched = 0U;
	position_mpc_reset(odometer_ctx.pose.pos_x_mm * 0.001f,
	        odometer_ctx.pose.pos_y_mm * 0.001f);
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

uint8_t IsPositionControlFinished(float tolerance_xy) {
	(void)tolerance_xy;
	return position_arrival_latched;
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
	uint8_t path_enabled = motion_path.active;
	float position_tolerance_m = path_enabled ?
	        MOTION_PATH_TRACK_TOLERANCE_M : raceh;
	float track_error_x_global;
	float track_error_y_global;

	float error_x_global = target_x_global - current_x_global;
	float error_y_global = target_y_global - current_y_global;
	float distance_error = sqrtf(error_x_global * error_x_global +
	                             error_y_global * error_y_global);
	if ((distance_error <= position_tolerance_m) &&
	        ((path_enabled == 0U) || motion_path.reference_complete) &&
	        (position_arrival_latched == 0U)) {
		position_arrival_latched = 1U;
		position_control.pid_x.integral = 0.0f;
		position_control.pid_y.integral = 0.0f;
		Emm_V5_Stop_Now(0U, 0U);
	}
	if (path_enabled == 0U) {
		position_mpc_update(current_x_global, current_y_global,
		        target_x_global, target_y_global, dt);
	}
	float planner_distance_m = path_enabled ?
	        motion_path.remaining_distance_m + distance_error : distance_error;
	control_ctx.error_x_global_m = error_x_global;
	control_ctx.error_y_global_m = error_y_global;


	float error_x_local = error_x_global * cosf(current_yaw_rad) +
	                      error_y_global * sinf(current_yaw_rad);
	float error_y_local = -error_x_global * sinf(current_yaw_rad) +
	                      error_y_global * cosf(current_yaw_rad);
	control_ctx.error_x_local_m = error_x_local;
	control_ctx.error_y_local_m = error_y_local;

	float plan_speed = 0.0f;
	if (path_enabled != 0U) {
		plan_speed = PositionSpeedPlan_Update(planner_distance_m, dt);
	}
	float actual_speed_mps = sqrtf(
	        odometer_ctx.motion.vel_x_mps * odometer_ctx.motion.vel_x_mps +
	        odometer_ctx.motion.vel_y_mps * odometer_ctx.motion.vel_y_mps);
	feedforward_accel_update(dt);
	const ai_tune_param_t *param = ai_tune_get_active();
	float plan_vx = 0.0f;
	float plan_vy = 0.0f;
	float feedforward_vx = 0.0f;
	float feedforward_vy = 0.0f;
	float velocity_feedback_vx;
	float velocity_feedback_vy;
	float velocity_feedback_scale;
	/* 里程计速度为世界系，转换到车体系后与局部目标速度闭环。 */
	float velocity_x_local = motion_observer.velocity_x_mps *
	        cosf(current_yaw_rad) + motion_observer.velocity_y_mps *
	        sinf(current_yaw_rad);
	float velocity_y_local = -motion_observer.velocity_x_mps *
	        sinf(current_yaw_rad) + motion_observer.velocity_y_mps *
	        cosf(current_yaw_rad);
	float target_vx_local;
	float target_vy_local;
	float desired_accel_x_local = 0.0f;
	float desired_accel_y_local = 0.0f;
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
	if (path_enabled) {
		float tangent_x_local = motion_path.tangent_x * cosf(current_yaw_rad) +
		        motion_path.tangent_y * sinf(current_yaw_rad);
		float tangent_y_local = -motion_path.tangent_x * sinf(current_yaw_rad) +
		        motion_path.tangent_y * cosf(current_yaw_rad);
		float normal_accel_mps2 = plan_speed * plan_speed *
		        motion_path.curvature_m_inv;
		float desired_accel_x_global = motion_path.tangent_x *
		        motion_plan_accel() - motion_path.tangent_y * normal_accel_mps2;
		float desired_accel_y_global = motion_path.tangent_y *
		        motion_plan_accel() + motion_path.tangent_x * normal_accel_mps2;

		plan_vx = tangent_x_local * plan_speed;
		plan_vy = tangent_y_local * plan_speed;
		desired_accel_x_local = desired_accel_x_global * cosf(current_yaw_rad) +
		        desired_accel_y_global * sinf(current_yaw_rad);
		desired_accel_y_local = -desired_accel_x_global * sinf(current_yaw_rad) +
		        desired_accel_y_global * cosf(current_yaw_rad);
	} else {
		plan_vx = position_mpc.filtered_velocity_x_mps * cosf(current_yaw_rad) +
		        position_mpc.filtered_velocity_y_mps * sinf(current_yaw_rad);
		plan_vy = -position_mpc.filtered_velocity_x_mps * sinf(current_yaw_rad) +
		        position_mpc.filtered_velocity_y_mps * cosf(current_yaw_rad);
		desired_accel_x_local = position_mpc.accel_x_mps2 *
		        cosf(current_yaw_rad) + position_mpc.accel_y_mps2 *
		        sinf(current_yaw_rad);
		desired_accel_y_local = -position_mpc.accel_x_mps2 *
		        sinf(current_yaw_rad) + position_mpc.accel_y_mps2 *
		        cosf(current_yaw_rad);
	}
	if (path_enabled || (distance_error > position_tolerance_m)) {
		float force_x_n = CHASSIS_MASS_KG *
		        (desired_accel_x_local + com_accel_x_local);
		float force_y_n = CHASSIS_MASS_KG *
		        (desired_accel_y_local + com_accel_y_local);
		feedforward_vx = force_x_n * param->force_to_speed_ff;
		feedforward_vy = force_y_n * param->force_to_speed_ff;
	}
	if (path_enabled != 0U) {
		track_error_x_global = error_x_global;
		track_error_y_global = error_y_global;
	} else {
		track_error_x_global = position_mpc.filtered_position_x_m -
		        current_x_global;
		track_error_y_global = position_mpc.filtered_position_y_m -
		        current_y_global;
	}
	float feedback_vx_global =
	        fuzzy_pid_calc(&position_control.pid_x, track_error_x_global, dt,
				POSITION_FUZZY_ERROR_FULL_M, POSITION_FUZZY_RATE_FULL_MPS,
				POSITION_FUZZY_KP_MIN_SCALE, POSITION_FUZZY_KP_MAX_SCALE,
				POSITION_FUZZY_KD_MIN_SCALE, POSITION_FUZZY_KD_MAX_SCALE, 0U);
	float feedback_vy_global =
	        fuzzy_pid_calc(&position_control.pid_y, track_error_y_global, dt,
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
	if ((distance_error > position_tolerance_m) &&
	        (distance_error <= POSITION_STATIC_FRICTION_ENABLE_DISTANCE_M) &&
	        (actual_speed_mps <= POSITION_STATIC_FRICTION_ENABLE_SPEED_MPS) &&
	        (feedback_speed < param->static_friction_speed_mps)) {

		float static_friction_scale = param->static_friction_speed_mps /
		        distance_error;
		feedforward_vx += error_x_local * static_friction_scale;
		feedforward_vy += error_y_local * static_friction_scale;
	}

	target_vx_local = plan_vx + feedback_vx;
	target_vy_local = plan_vy + feedback_vy;
	velocity_feedback_scale = ClampFloat(distance_error /
	        POSITION_VELOCITY_FEEDBACK_FADE_DISTANCE_M, 0.0f, 1.0f);
	velocity_feedback_vx = velocity_feedback_scale *
	        POSITION_VELOCITY_FEEDBACK_GAIN *
	        (target_vx_local - velocity_x_local);
	velocity_feedback_vy = velocity_feedback_scale *
	        POSITION_VELOCITY_FEEDBACK_GAIN *
	        (target_vy_local - velocity_y_local);
	float vx_local = target_vx_local + feedforward_vx +
	        velocity_feedback_vx;
	float vy_local = target_vy_local + feedforward_vy +
	        velocity_feedback_vy;
	float linear_speed = sqrtf(vx_local * vx_local + vy_local * vy_local);
	float command_speed_limit = sqrtf(2.0f *
	        POSITION_COMMAND_BRAKE_DECEL_MPS2 * planner_distance_m);
	command_speed_limit = ClampFloat(command_speed_limit, 0.0f,
	        motion_plan_cruise());
	if (linear_speed > command_speed_limit) {

		float ratio = command_speed_limit / linear_speed;
		vx_local *= ratio;
		vy_local *= ratio;
		linear_speed = command_speed_limit;
	}
	if (position_arrival_latched != 0U) {
		vx_local = 0.0f;
		vy_local = 0.0f;
		linear_speed = 0.0f;
	}

	vx_local *= pose_coordination.sync_scale;
	vy_local *= pose_coordination.sync_scale;
	if (ai_tune_guard_update(planner_distance_m, linear_speed, dt)) {
		vx_local = 0.0f;
		vy_local = 0.0f;
	}
#if (AI_TUNE_MODE_ENABLE != 0U)
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
		        .plan_accel_mps2 = sqrtf(position_mpc.accel_x_mps2 *
		                position_mpc.accel_x_mps2 + position_mpc.accel_y_mps2 *
		                position_mpc.accel_y_mps2),
		        .accel_x_mps2 = accel_x_local,
		        .accel_y_mps2 = accel_y_local,
		        .plan_vx_mps = plan_vx,
		        .plan_vy_mps = plan_vy,
		        .feedforward_vx_mps = feedforward_vx,
		        .feedforward_vy_mps = feedforward_vy,
		        .feedback_vx_mps = feedback_vx,
		        .feedback_vy_mps = feedback_vy,
		        .velocity_feedback_vx_mps = velocity_feedback_vx,
		        .velocity_feedback_vy_mps = velocity_feedback_vy,
		        .reference_x_m = path_enabled ? target_x_global :
		                position_mpc.position_x_m,
		        .reference_y_m = path_enabled ? target_y_global :
		                position_mpc.position_y_m,
		        .command_vx_mps = vx_local,
		        .command_vy_mps = vy_local,
		        .motor_rpm = {motor1.speed_rpm, motor2.speed_rpm,
		                motor3.speed_rpm, motor4.speed_rpm},
		        .position_reached = IsPositionControlFinished(raceh),
		};

		ai_tune_send_telemetry(&telemetry);
	}
#endif

	SetTargetLinearVelocity(vx_local, vy_local);
	if (ai_tune_is_guarded()) {
		PositionControl_Enable(0U);
		return;
	}

	if (IsPositionControlFinished(position_tolerance_m)) {
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

	float planned_yaw = motion_path.active ? attitude_control.target_yaw :
	        yaw_plan_update(attitude_control.target_yaw, dt);
	float desired_yaw_rate_rad_s = motion_path.active ?
	        motion_path.yaw_rate_rad_s : yaw_plan_rate_rad_s;
	float error = planned_yaw - current_yaw;
	float error_rad;
	float yaw_rate_rad_s;
	float target_error_deg;
	float omega_command;


	while (error > 180.0f)
		error -= 360.0f;
	while (error < -180.0f)
		error += 360.0f;
	yaw_rate_rad_s = motion_observer.yaw_rate_rad_s;
	target_error_deg = NormalizeYawError(attitude_control.target_yaw -
	        current_yaw);
	control_ctx.yaw_error_deg = target_error_deg;
	if ((!motion_path.active || motion_path.reference_complete) &&
	        (fabsf(target_error_deg) <= POSE_YAW_TOLERANCE_DEG) &&
	        (fabsf(yaw_rate_rad_s) < 0.05f)) {
		yaw_plan_reset(current_yaw);
		attitude_control.yaw_pid.integral = 0.0f;
		attitude_correction = 0.0f;
		control_ctx.yaw_error_deg = target_error_deg;
		return;
	}
	if ((!motion_path.active || motion_path.reference_complete) &&
	        (fabsf(target_error_deg) < DEAD_ZONE) &&
	        (fabsf(yaw_rate_rad_s) < 0.03f)) {
		attitude_control.yaw_pid.integral = 0.0f;
		attitude_correction = 0.0f;
		return;
	}

	error_rad = error * 0.017453292519943295f;
	omega_command = desired_yaw_rate_rad_s +
			attitude_control.yaw_pid.Kp * error_rad -
			attitude_control.yaw_pid.Kd * yaw_rate_rad_s;
	attitude_correction = ClampFloat(omega_command,
			-ATTITUDE_PID_MAX_OUTPUT, ATTITUDE_PID_MAX_OUTPUT);
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
	return IsPositionControlFinished(tolerance_xy) &&
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


	uint8_t is_closed_loop_enabled = (is_position_control_enabled != 0U) ||
	        (attitude_control.is_enabled != 0U);
	float wheel_speed_scale = (is_closed_loop_enabled != 0U) ?
	        CHASSIS_CLOSED_LOOP_WHEEL_SPEED_SCALE :
	        CHASSIS_OPEN_LOOP_WHEEL_SPEED_SCALE;
	const float linear_to_rpm = 60.0f /
	        (2.0f * 3.1415926f * chassis_config.wheel_radius) *
	        wheel_speed_scale;
	float translation_speed[4] = {vx - vy, vx + vy, vx + vy, vx - vy};
	float rotation_speed[4] = {-k_omega * actual_omega,
	        k_omega * actual_omega, -k_omega * actual_omega,
	        k_omega * actual_omega};
	float translation_scale = 1.0f;
	uint8_t wheel_index;

	max_rpm = (is_closed_loop_enabled != 0U) ?
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
	const uint8_t is_closed_loop_enabled =
	        (is_position_control_enabled != 0U) ||
	        (attitude_control.is_enabled != 0U);
	const uint8_t velocity_accel = (is_closed_loop_enabled != 0U) ?
	        CHASSIS_CLOSED_LOOP_VELOCITY_ACCEL :
	        CHASSIS_OPEN_LOOP_VELOCITY_ACCEL;

	Emm_V5_Vel_Control(1, motor1.direction,
	        (uint16_t)abs_int(motor1.speed_rpm), velocity_accel, 0);
	Emm_V5_Vel_Control(2, motor2.direction,
	        (uint16_t)abs_int(motor2.speed_rpm), velocity_accel, 0);
	Emm_V5_Vel_Control(3, motor3.direction,
	        (uint16_t)abs_int(motor3.speed_rpm), velocity_accel, 0);
	Emm_V5_Vel_Control(4, motor4.direction,
	        (uint16_t)abs_int(motor4.speed_rpm), velocity_accel, 0);
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
