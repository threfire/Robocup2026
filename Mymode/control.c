#include "control.h"
#include <math.h>
#include <string.h>
#include "lcd_spi_154.h"
#include "odometer.h"
#include "servo.h"
#include "graysensor.h"
#include "userkey.h"
#include "QAma.h"
#include "mxcam.h"

/* 控制模块运行状态统一收敛到 control_ctx，旧任务代码通过宏访问字段。 */
ControlContext_t control_ctx = {
        .race_tolerance_m = POSITION_REACHED_TOLERANCE_M,
        .material_pwm = {575, 970, 1365, 1785, 2165},
        .task_num = 1,
        .cam_mode = 2,
        .look_mode_control = 1,
};

/*
 * 任务 1 路径点，单位 mm，坐标系与里程计输出一致。
 * 0: 扫二维码 1，1-5: 物块区，6-11: 中间点，12-14: 奖杯区，
 * 15-17: 奖台，18: 起始点，19-20: 过渡点。
 */
static const Point2D_t task1_targets[] = {
        {250.25f, 650.0f}, {258.25f, 1058.24f}, {604.89f, 1399.38f},
        {1075.0f, 1524.0f}, {1545.10f, 1399.38f}, {1891.74f, 1058.24f},
        {835.14f, 631.60f}, {553.90f, 425.91f}, {836.02f, -220.80f},
        {554.98f, -334.63f}, {623.60f, -837.96f}, {345.80f, -646.16f},
        {409.58f, -1467.21f}, {860.0f, -1564.0f}, {1315.42f, -1442.21f},
        {1710.0f, -270.0f}, {1695.0f, 0.0f}, {1715.0f, 270.0f}, {0.0f, 0.0f},
        {150.00f, -1467.21f}, {1000.f, 0.0f}, {156.27f, 682.54f}

};
void Setmaixcam_mode(uint8_t ch) {
	cammode = ch;
}
void Move_to_turnplate(uint8_t num) {
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Material[num - 1]);
}
void Find_corresponding_material(uint8_t index) {
	uint8_t i;
	if (tasknum == 1) {
		for (i = 0; i < 5; i++) {
			if (task1_scheme[qanum - 1][index - 1] ==
			        detected_colors[i]) {
				Move_to_turnplate(i + 1);
			}
		}
	}
	if (tasknum == 2) {
		for (i = 0; i < 3; i++) {
			if (index == task2_scheme[qanum - 1][i]) {
				Move_to_turnplate(i + 1);
			}
		}
	}
}
void Pos_moveconctrl(uint8_t dir, uint16_t vel, uint16_t clk) {
	/* X42 位置模式用于局部前后微动，clk 按 3200 脉冲每圈换算为角度。 */
	float position_deg = (float)clk * ZDT_X42_POS_DEG_PER_CLK;

	if (dir == 1) {
		ZDT_X42_V2_Traj_Position_Control(1, 1, CHASSIS_POS_ACCEL,
		        CHASSIS_POS_DECEL, (float)vel, position_deg, 0, 1);

		ZDT_X42_V2_Traj_Position_Control(2, 1, CHASSIS_POS_ACCEL,
		        CHASSIS_POS_DECEL, (float)vel, position_deg, 0, 1);

		ZDT_X42_V2_Traj_Position_Control(3, 0, CHASSIS_POS_ACCEL,
		        CHASSIS_POS_DECEL, (float)vel, position_deg, 0, 1);

		ZDT_X42_V2_Traj_Position_Control(4, 0, CHASSIS_POS_ACCEL,
		        CHASSIS_POS_DECEL, (float)vel, position_deg, 0, 1);
		HAL_Delay(5);
		ZDT_X42_V2_Synchronous_motion(0);
	}
	if (dir == 0) {
		ZDT_X42_V2_Traj_Position_Control(1, 0, CHASSIS_POS_ACCEL,
		        CHASSIS_POS_DECEL, (float)vel, position_deg, 0, 1);

		ZDT_X42_V2_Traj_Position_Control(2, 0, CHASSIS_POS_ACCEL,
		        CHASSIS_POS_DECEL, (float)vel, position_deg, 0, 1);

		ZDT_X42_V2_Traj_Position_Control(3, 1, CHASSIS_POS_ACCEL,
		        CHASSIS_POS_DECEL, (float)vel, position_deg, 0, 1);

		ZDT_X42_V2_Traj_Position_Control(4, 1, CHASSIS_POS_ACCEL,
		        CHASSIS_POS_DECEL, (float)vel, position_deg, 0, 1);
		HAL_Delay(5);
		ZDT_X42_V2_Synchronous_motion(0);
	}
}
void POS_move(void) {
	/* 位置微动期间暂停实时速度链路，动作结束后恢复底盘实时控制。 */
	Is_vel_mode = 0;
	Pos_moveconctrl(1, 50, 1058);
	HAL_Delay(1800);
	Pos_moveconctrl(0, 50, 1442);
	HAL_Delay(1800);
	Is_vel_mode = 1;
}
static float position_plan_speed_mps = 0.0f;
static float position_plan_accel_mps2 = 0.0f;

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
}

static void MotionState_Reset(void);

static float PositionSpeedPlan_Update(float distance_error_m, float dt) {
	/* 距离误差生成前馈速度，减速距离约束保证接近目标点时速度可收敛。 */
	float cruise_speed = ClampFloat(POSITION_PLANNER_CRUISE_SPEED_MPS, 0.0f,
	        position_control.pid_x.max_output);
	float brake_speed = sqrtf(
	        2.0f * POSITION_PLANNER_MAX_DECEL_MPS2 * distance_error_m);
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

	if (position_plan_speed_mps > target_speed &&
	        position_plan_accel_mps2 > 0.0f) {
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
	        POSITION_PID_DEFAULT_KD, HIGHSPEED);

	PID_Init(&position_control.pid_y, POSITION_PID_DEFAULT_KP, 0.00f,
	        POSITION_PID_DEFAULT_KD, HIGHSPEED);

	position_control.is_enabled = 0;
	position_control.target_x = 0.0f;
	position_control.target_y = 0.0f;
	position_control.last_update_time = HAL_GetTick();

	is_position_control_enabled = 0;
}

void pospid_reinit(float remax_output, float re_kp) {
	position_control.pid_x.max_output = remax_output;
	position_control.pid_y.max_output = remax_output;
	position_control.pid_x.Kp = re_kp;
	position_control.pid_y.Kp = re_kp;
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

void SetTargetPosition(float x, float y) {
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

	/* 局部坐标误差用于麦轮底盘速度指令，正负号与底盘安装方向保持一致。 */
	float error_x_local = error_x_global * cosf(current_yaw_rad) +
	                      error_y_global * sinf(current_yaw_rad);
	float error_y_local = -error_x_global * sinf(current_yaw_rad) +
	                      error_y_global * cosf(current_yaw_rad);

	float direction_x = 0.0f;
	float direction_y = 0.0f;
	float error_norm = sqrtf(
	        error_x_local * error_x_local + error_y_local * error_y_local);
	float planned_speed = PositionSpeedPlan_Update(distance_error, dt);

	if (error_norm > 0.001f) {
		/* 单位方向向量叠加前馈速度，PID 只补偿剩余误差。 */
		direction_x = error_x_local / error_norm;
		direction_y = error_y_local / error_norm;
	}

	float feedforward_vx = direction_x * planned_speed;
	float feedforward_vy = direction_y * planned_speed;
	float feedback_vx =
	        PID_Update(&position_control.pid_x, error_x_local, dt);
	float feedback_vy =
	        PID_Update(&position_control.pid_y, error_y_local, dt);
	float feedback_speed =
	        sqrtf(feedback_vx * feedback_vx + feedback_vy * feedback_vy);

	if (feedback_speed > POSITION_FEEDBACK_MAX_SPEED_MPS) {
		/* 反馈速度限幅保留前馈速度主导性，减少近点抖动。 */
		float feedback_ratio =
		        POSITION_FEEDBACK_MAX_SPEED_MPS / feedback_speed;
		feedback_vx *= feedback_ratio;
		feedback_vy *= feedback_ratio;
	}

	float vx_local = feedforward_vx + feedback_vx;
	float vy_local = feedforward_vy + feedback_vy;
	float linear_speed = sqrtf(vx_local * vx_local + vy_local * vy_local);
	if (linear_speed > POSITION_MAX_LINEAR_SPEED_MPS) {
		/* 最终平移速度按比例缩放，保留运动方向。 */
		float ratio = POSITION_MAX_LINEAR_SPEED_MPS / linear_speed;
		vx_local *= ratio;
		vy_local *= ratio;
	}

	vx_local *= pose_coordination.sync_scale;
	vy_local *= pose_coordination.sync_scale;
	/* SetTargetLinearVelocity 的参数顺序对应当前底盘局部轴定义。 */
	SetTargetLinearVelocity(vy_local, vx_local);

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
	PID_Init(&attitude_control.yaw_pid, 0.26f, 0.0f, 0.0051f, 6.0f);

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
	}
}

void UpdateAttitudeStabilization(float dt) {
	if (!attitude_control.is_enabled || dt <= 0) {
		attitude_correction = 0.0f;
		return;
	}

	float current_yaw = odometer_get_yaw_deg();

	float error = attitude_control.target_yaw - current_yaw;

	/* 角度环绕处理和死区共同限制小误差下的来回修正。 */
	while (error > 180.0f)
		error -= 360.0f;
	while (error < -180.0f)
		error += 360.0f;

	if (abs_float(error) < DEAD_ZONE) {
		error = 0.0f;
		attitude_control.yaw_pid.integral = 0.0f;
		attitude_correction = 0.0f;
		return;
	}

	attitude_correction =
	        PID_Update_angle(&attitude_control.yaw_pid, error, dt);
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
	        &cam_stabilization.pid_x, 0.26f, 0.0000f, 0.0f, 0.002f, 20.0f);

	PID_Init_cam(
	        &cam_stabilization.pid_y, 0.26f, 0.0000f, 0.0f, 0.002f, 20.0f);

	cam_stabilization.is_enabled = 0;
	cam_correction_x = 0.0f;
	cam_correction_y = 0.0f;
	lookmode_centrl = 0;
	is_cam_stabilizing = 0;
	Is_coordgot = 0;
}

void CamStabilization_Enable(uint8_t enable) {
	cam_stabilization.is_enabled = enable;

	IsLight = 1;
	if (!enable) {
		cam_stabilization.pid_x.integral = 0.0f;
		cam_stabilization.pid_x.prev_error = 0.0f;
		cam_stabilization.pid_y.integral = 0.0f;
		cam_stabilization.pid_y.prev_error = 0.0f;
		cam_correction_x = 0.0f;
		cam_correction_y = 0.0f;
		IsLight = 0;
	}
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
		IsLight = 1;
		return 0;
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
static ChassisConfig_t chassis_config;
static float k_omega;

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
	if (chassis_config.max_wheel_mps <= 0.0f) {
		chassis_config.max_wheel_mps = 1.0f;
	}
	if (chassis_config.max_current_ma == 0U) {
		chassis_config.max_current_ma = CHASSIS_CURRENT_MAX_MA;
	}
	k_omega = (config.wheel_dist_x + config.wheel_dist_y) / 2.0f;
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

	/* 麦轮逆解：局部 vx/vy/omega 映射到四轮等效线速度。 */
	float v4 = vx - vy - k_omega * actual_omega;
	float v2 = vx + vy + k_omega * actual_omega;
	float v3 = vx + vy - k_omega * actual_omega;
	float v1 = vx - vy + k_omega * actual_omega;

	float current_scale;

	float max_abs_v = fabsf(v4);
	float scale = 1.0f;

	/* 四轮速度统一缩放，保持运动方向和角速度比例。 */
	if (fabsf(v2) > max_abs_v)
		max_abs_v = fabsf(v2);
	if (fabsf(v3) > max_abs_v)
		max_abs_v = fabsf(v3);
	if (fabsf(v1) > max_abs_v)
		max_abs_v = fabsf(v1);

	if (max_abs_v > chassis_config.max_wheel_mps) {
		scale = chassis_config.max_wheel_mps / max_abs_v;
		v4 *= scale;
		v2 *= scale;
		v3 *= scale;
		v1 *= scale;
	}

	current_scale = (float)chassis_config.max_current_ma /
	                chassis_config.max_wheel_mps;
	/* X42 力矩电流模式：等效轮速按最大电流线性换算为电流命令。 */
	motor4.current_ma = (int16_t)(v4 * current_scale);
	motor2.current_ma = (int16_t)(v2 * current_scale);
	motor3.current_ma = (int16_t)(v3 * current_scale);
	motor1.current_ma = (int16_t)(v1 * current_scale);

	motor4.direction = (motor4.current_ma >= 0) ? 0 : 1;
	motor2.direction = (motor2.current_ma >= 0) ? 1 : 0;
	motor3.direction = (motor3.current_ma >= 0) ? 0 : 1;
	motor1.direction = (motor1.current_ma >= 0) ? 1 : 0;
}

static uint16_t Motor_CurrentAbs(int16_t current_ma) {
	/* 固件接口方向和电流幅值分开发送，电流幅值需要限幅和最小有效值。 */
	uint16_t current_abs;

	if (current_ma < 0) {
		current_abs = (uint16_t)(-current_ma);
	} else {
		current_abs = (uint16_t)current_ma;
	}

	if (current_abs > chassis_config.max_current_ma) {
		current_abs = chassis_config.max_current_ma;
	}
	if ((current_abs > 0U) && (current_abs < CHASSIS_CURRENT_MIN_MA)) {
		current_abs = CHASSIS_CURRENT_MIN_MA;
	}

	return current_abs;
}

void Mecanum_SendTorqueCurrent(void) {
	/* 四个 X42 电机使用力矩电流模式，方向位由运动学符号生成。 */
	ZDT_X42_V2_Torque_Control(1, motor1.direction, CHASSIS_TORQUE_RAMP_MA_S,
	        Motor_CurrentAbs(motor1.current_ma), 0);
	ZDT_X42_V2_Torque_Control(2, motor2.direction, CHASSIS_TORQUE_RAMP_MA_S,
	        Motor_CurrentAbs(motor2.current_ma), 0);
	ZDT_X42_V2_Torque_Control(3, motor3.direction, CHASSIS_TORQUE_RAMP_MA_S,
	        Motor_CurrentAbs(motor3.current_ma), 0);
	ZDT_X42_V2_Torque_Control(4, motor4.direction, CHASSIS_TORQUE_RAMP_MA_S,
	        Motor_CurrentAbs(motor4.current_ma), 0);
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
	motor1.current_ma = 0;
	motor2.current_ma = 0;
	motor3.current_ma = 0;
	motor4.current_ma = 0;
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
uint8_t task1_relative_forward_back(void) {
	/* 任务 1 的局部前后微动：记录当前点，先前进固定距离，再回到基准点。 */
	static uint8_t relative_stage = 0;
	static uint8_t yaw_hold_set = 0;
	static uint32_t switch_pause_start = 0;
	static float base_x_mm = 0.0f;
	static float base_y_mm = 0.0f;

	if (relative_stage == 0) {
		if (!yaw_hold_set) {
			/* 微动前锁定 yaw，降低相对位移过程中的方向漂移。 */
			PoseCoordination_Enable(0);
			stop();
			attitude_correction = 0.0f;
			attitude_control.yaw_pid.integral = 0.0f;
			attitude_control.yaw_pid.prev_error = 0.0f;
			EnableAttitudeStabilization(1);
			SetTargetYaw(-90.0f);
			base_x_mm = odometer_ctx.pose.pos_x_mm;
			base_y_mm = odometer_ctx.pose.pos_y_mm;
			yaw_hold_set = 1;
		}
		pospid_reinit(
		        TASK1_RELATIVE_SPEED, POSITION_PID_TASK1_RELATIVE_KP);
		if (move_to_pose_target(base_x_mm + TASK1_RELATIVE_X_MOVE_MM,
		            base_y_mm, -90.0f)) {
			relative_stage = 1;
			switch_pause_start = HAL_GetTick();
			stop();
			PositionPlanner_Reset();
		}
		return 0;
	}

	if (relative_stage == 1) {
		/* 前进和返回之间停顿一小段时间，让底盘电流和里程计状态稳定。 */
		stop();
		if ((HAL_GetTick() - switch_pause_start) <
		        TASK1_RELATIVE_SWITCH_PAUSE_MS) {
			PositionPlanner_Reset();
			return 0;
		}
		relative_stage = 2;
		PositionPlanner_Reset();
		return 0;
	}

	if (relative_stage == 2) {
		if (move_to_pose_target(base_x_mm, base_y_mm, -90.0f)) {
			relative_stage = 0;
			yaw_hold_set = 0;
			switch_pause_start = 0;
			base_x_mm = 0.0f;
			base_y_mm = 0.0f;
			PositionPlanner_Reset();
			pospid_reinit(MIDSPEED, POSITION_PID_DEFAULT_KP);
			return 1;
		}
		return 0;
	}

	relative_stage = 0;
	yaw_hold_set = 0;
	switch_pause_start = 0;
	base_x_mm = 0.0f;
	base_y_mm = 0.0f;
	return 0;
}
/***********Task1***************/

void start_task1(void) {
	/* 任务状态机从步骤 1 开始，step_start 记录当前步骤进入时间。 */
	task1_state.active = 1;
	task1_state.current_step = 1;
	task1_state.completed = 0;
	task1_state.step_start = HAL_GetTick();
}

void run_task1(void) {
	if (!task1_state.active)
		return;
	uint32_t current_time = HAL_GetTick();
	switch (task1_state.current_step) {
	case 1:
		/* 起步到二维码点：保持 yaw 为 0，转盘先转到第一个物料位。 */
		EnableAttitudeStabilization(1);
		SetTargetYaw(0.0f);
		Move_to_turnplate(1);
		Servo_Action_Group(1);
		tasknum = 1;
		pospid_reinit(LOWSPEED, POSITION_PID_DEFAULT_KP);
		if (move_to_pose_target(
		            task1_targets[0].x, task1_targets[0].y, 0.0f)) {
			Servo_Action_Group(2);
			task1_state.current_step++;
		}
		if (IsPositionReached(0.50f)) {
			__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 2060);
		}
		if (IsPositionReached(0.45f)) {
			__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1980);
		}
		if (IsPositionReached(0.40f)) {
			__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1930);
		}
		if (IsPositionReached(0.35f)) {
			__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1900);
		}
		if (IsPositionReached(0.30f)) {
			__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1880);
		}
		if (IsPositionReached(0.25f)) {
			__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1860);
		}
		if (IsPositionReached(0.20f)) {
			__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1840);
		}
		if (IsPositionReached(0.15f)) {
			__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1820);
		}
		if (IsPositionReached(0.10f)) {
			__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1810);
		}
		if (IsPositionReached(0.04f)) {
			__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1800);
		}
		break;
	case 2:
		/* 转盘依次采集物料颜色，扫描结束后进入中间点搬运阶段。 */
		Servo_Action_Group(2);
		PositionControl_Enable(0);
		EnableAttitudeStabilization(0);
		SetTargetVelocity(1.0f, 0.0f, -2.57f);
		HAL_Delay(500);
		Move_to_turnplate(2);
		HAL_Delay(1000);
		Get_Materialcolor(0);
		Move_to_turnplate(3);
		HAL_Delay(450);
		SetTargetVelocity(1.0f, 0.0f, -2.38f);
		HAL_Delay(500);
		Get_Materialcolor(1);
		Move_to_turnplate(4);
		HAL_Delay(500);
		SetTargetVelocity(1.0f, 0.0f, -2.24f);
		HAL_Delay(550);
		Get_Materialcolor(2);
		Move_to_turnplate(5);
		HAL_Delay(1000);
		Get_Materialcolor(3);
		__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Material[3] + 200);
		MotionState_Reset();
		HAL_Delay(300);
		task1_state.current_step = 11;
		break;

	case 11:
		/* 进入物块搬运序列：到达第一个取放点并转到对应物料位。 */
		EnableAttitudeStabilization(1);
		pospid_reinit(LOWSPEED, POSITION_PID_DEFAULT_KP);
		Get_Materialcolor(4);
		if (move_to_pose_target(task1_targets[6].x, task1_targets[6].y,
		            -90.0f) /* 到达 a 点 */) {
			Find_corresponding_material(1);
			task1_state.current_step++;
		}
		break;
	case 12:
		/* 等待视觉串口搜索结果，当前策略允许超时后继续执行下一步。 */
		if (Wait_for_uart_with_search(6)) {
			task1_state.current_step++;
		} else {
			task1_state.current_step++;
		}
		break;
	case 13:
		/* 在 a 点执行一次相对前后动作完成取放。 */
		if (task1_relative_forward_back()) {
			task1_state.current_step++;
		}
		break;
	case 14:
		/* 到达 b 点并切换到第二个物料位。 */
		pospid_reinit(MIDSPEED, POSITION_PID_DEFAULT_KP);
		if (move_to_pose_target(
		            task1_targets[7].x, task1_targets[7].y, -90.0f)) {
			Find_corresponding_material(2);
			task1_state.current_step++;
		}
		break;
	case 15:
		if (Wait_for_uart_with_search(7)) {
			task1_state.current_step++;
		} else {
			task1_state.current_step++;
		}
		break;
	case 16:
		if (task1_relative_forward_back()) {
			task1_state.current_step++;
		}
		break;
	case 17:
		/* c 点、d 点、e 点沿用到点、视觉确认、相对取放流程。 */
		if (move_to_pose_target(
		            task1_targets[8].x, task1_targets[8].y, -90.0f)) {
			Find_corresponding_material(3);
			task1_state.current_step++;
		}
		break;
	case 18:
		if (Wait_for_uart_with_search(8)) {
			task1_state.current_step++;
		} else {
			task1_state.current_step++;
		}
		break;
	case 19:
		if (task1_relative_forward_back()) {
			task1_state.current_step++;
		}
		break;
	case 20:
		if (move_to_pose_target(
		            task1_targets[9].x, task1_targets[9].y, -90.0f)) {
			Find_corresponding_material(4);
			task1_state.current_step++;
		}
		break;
	case 21:
		if (Wait_for_uart_with_search(9)) {
			task1_state.current_step++;
		} else {
			task1_state.current_step++;
		}
		break;
	case 22:
		if (task1_relative_forward_back()) {
			task1_state.current_step++;
		}
		break;
	case 23:
		if (move_to_pose_target(
		            task1_targets[10].x, task1_targets[10].y, -90.0f)) {
			Find_corresponding_material(5);
			task1_state.current_step++;
		}
		break;
	case 24:
		if (Wait_for_uart_with_search(10)) {
			task1_state.current_step++;
		} else {
			task1_state.current_step++;
		}
		break;
	case 25:
		if (task1_relative_forward_back()) {
			task1_state.current_step++;
		}
		break;
	case 26:
		/* 第一段取放结束后切换任务编号，前往第二个二维码点。 */
		tasknum = 2;
		if (move_to_pose_target(
		            task1_targets[11].x, task1_targets[11].y, 0.0f)) {
			Move_to_turnplate(3);
			task1_state.current_step++;
		}
		break;
	case 27:
		if (move_to_pose_target(
		            task1_targets[19].x, task1_targets[19].y, -90.0f)) {
			task1_state.current_step++;
		}
		break;
	case 28:
		/* 低速进入奖杯区，按三个奖杯点依次靠近并切换转盘位置。 */
		pospid_reinit(LOWESTSPEED, POSITION_PID_DEFAULT_KP);
		if (move_to_pose_target(
		            task1_targets[12].x, task1_targets[12].y, -90.0f)) {
			task1_state.current_step++;
			Move_to_turnplate(2);
		}
		break;
	case 29:
		if (move_to_pose_target(
		            task1_targets[13].x, task1_targets[13].y, -90.0f)) {
			task1_state.current_step++;
			Move_to_turnplate(1);
		}
		break;
	case 30:
		if (move_to_pose_target(
		            task1_targets[14].x, task1_targets[14].y, -90.0f)) {
			__HAL_TIM_SetCompare(
			        &htim11, TIM_CHANNEL_1, Material[1] + 200);
			task1_state.current_step++;
		}
		break;
	case 31:
		/* 进入奖台区，接近目标时提前展开舵机动作。 */
		pospid_reinit(HIGHSPEED, POSITION_PID_DEFAULT_KP);
		if (move_to_pose_target(task1_targets[15].x,
		            task1_targets[15].y + 15.0f, -90.0f)) {
			Servo_Action_Group(6);
			Find_corresponding_material(2);
			task1_state.current_step++;
			break;
		}
		if (IsPositionReached(0.6f)) {
			Servo_Action_Group(5);
		}
		break;
	case 32:
		if (Wait_for_uart_with_search(15)) {
			task1_state.current_step++;
		} else {
			task1_state.current_step++;
		}
		break;
	case 33:
		/* 奖台放置动作：完成视觉确认后执行相对前后取放和舵机复位。 */
		if (task1_relative_forward_back()) {
			Servo_Action_Group(3);
			HAL_Delay(100);
			task1_state.current_step++;
		}
		break;
	case 34:
		if (move_to_pose_target(task1_targets[16].x,
		            task1_targets[16].y + 15.0f, -90.0f)) {
			Servo_Action_Group(4);
			Find_corresponding_material(1);
			task1_state.current_step++;
		}
		break;
	case 35:
		if (Wait_for_uart_with_search(16)) {
			task1_state.current_step++;
		} else {
			task1_state.current_step++;
		}
		break;
	case 36:
		if (task1_relative_forward_back()) {
			Servo_Action_Group(3);
			task1_state.current_step++;
		}
		break;
	case 37:
		if (move_to_pose_target(task1_targets[17].x,
		            task1_targets[17].y + 25.0f, -90.0f)) {
			Servo_Action_Group(7);
			Find_corresponding_material(3);
			task1_state.current_step++;
		}
		break;
	case 38:
		if (Wait_for_uart_with_search(17)) {
			task1_state.current_step++;
		} else {
			task1_state.current_step++;
		}
		break;
	case 39:
		if (task1_relative_forward_back()) {
			task1_state.current_step++;
		}
		break;
	case 40:
		/* 回家前先经过过渡点，使用较大到达容差提高返航通过性。 */
		pospid_reinit(HIGHESTSPEED, POSITION_PID_DEFAULT_KP);
		raceh = POSITION_RETURN_REACHED_TOLERANCE_M;
		if (move_to_pose_target(
		            task1_targets[20].x, task1_targets[20].y, -90.0f)) {
			task1_state.current_step = 41;
		}
		break;
	case 41:
		/* 最后回到起始点，恢复精细到达容差。 */
		raceh = POSITION_FINE_REACHED_TOLERANCE_M;
		if (move_to_pose_target(
		            task1_targets[18].x, task1_targets[18].y, -90.0f)) {
			task1_state.current_step++;
		}
		break;
	default:
		/* 超出任务步骤后标记任务完成。 */
		task1_state.active = 0;
		task1_state.completed = 1;
		break;
	}
}
uint8_t is_task1_active(void) {
	return task1_state.active;
}
/***********Task1***************/
