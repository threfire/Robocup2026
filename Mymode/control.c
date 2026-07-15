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
        {-650.0f, 250.25f}, {-1058.24f, 258.25f},
        {-1399.38f, 604.89f}, {-1524.0f, 1075.0f},
        {-1399.38f, 1545.10f}, {-1058.24f, 1891.74f},
        {-631.60f, 835.14f}, {-425.91f, 553.90f},
        {220.80f, 836.02f}, {334.63f, 554.98f}, {837.96f, 623.60f},
        {646.16f, 345.80f}, {1467.21f, 409.58f}, {1564.0f, 860.0f},
        {1442.21f, 1315.42f}, {270.0f, 1710.0f}, {0.0f, 1695.0f},
        {-270.0f, 1715.0f}, {0.0f, 0.0f}, {1467.21f, 150.00f},
        {0.0f, 1000.0f}, {-682.54f, 156.27f}

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

	/* 轨迹规划器保留加速度状态，速度前馈当前设为零。 */
	(void)PositionSpeedPlan_Update(distance_error, dt);
	float feedforward_vx = 0.0f;
	float feedforward_vy = 0.0f;
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
	/* 局部坐标定义为 vx 正向前、vy 正向左；X42 安装使 vx 需反向。 */
	vx = -vx;

	/* 麦轮逆解：局部 vx/vy/omega 映射到四轮等效线速度。 */
	float v4 = vx - vy - k_omega * actual_omega;
	float v2 = vx + vy + k_omega * actual_omega;
	float v3 = vx + vy - k_omega * actual_omega;
	float v1 = vx - vy + k_omega * actual_omega;

	/* 等效轮速转换为电机速度命令。 */
	const float linear_to_rpm = 60.0f /
	        (2.0f * 3.1415926f * chassis_config.wheel_radius);
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

	motor4.speed_rpm = (int16_t)rpm4;
	motor2.speed_rpm = (int16_t)rpm2;
	motor3.speed_rpm = (int16_t)rpm3;
	motor1.speed_rpm = (int16_t)rpm1;
	motor4.direction = (motor4.speed_rpm >= 0) ? 0U : 1U;
	motor2.direction = (motor2.speed_rpm >= 0) ? 1U : 0U;
	motor3.direction = (motor3.speed_rpm >= 0) ? 0U : 1U;
	motor1.direction = (motor1.speed_rpm >= 0) ? 1U : 0U;
}

void Mecanum_SendVelocity(void) {
	/* 速度命令持续发送，未启动时四轮速度为 0 RPM。 */
	ZDT_X42_V2_Velocity_Control(1, motor1.direction, CHASSIS_VELOCITY_RAMP,
	        (float)abs_int(motor1.speed_rpm), 0);
	ZDT_X42_V2_Velocity_Control(2, motor2.direction, CHASSIS_VELOCITY_RAMP,
	        (float)abs_int(motor2.speed_rpm), 0);
	ZDT_X42_V2_Velocity_Control(3, motor3.direction, CHASSIS_VELOCITY_RAMP,
	        (float)abs_int(motor3.speed_rpm), 0);
	ZDT_X42_V2_Velocity_Control(4, motor4.direction, CHASSIS_VELOCITY_RAMP,
	        (float)abs_int(motor4.speed_rpm), 0);
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

static void task1_next_step(void);
static void task1_finish(void);
static uint8_t task1_move_target(
        uint8_t target_index, float y_offset_mm, float yaw_deg);
static void task1_go_first_qrcode(void);
static void task1_scan_material_colors(void);
static void task1_go_first_material_point(void);
static void task1_go_material_point(
        uint8_t target_index, uint8_t material_index, float speed_mps);
static void task1_wait_vision(uint8_t point_index);
static void task1_relative_action(uint8_t servo_after, uint32_t delay_after_ms);
static void task1_go_second_qrcode(void);
static void task1_go_trophy_point(uint8_t target_index, uint8_t turnplate_num);
static void task1_go_third_trophy(void);
static void task1_go_first_trophy(void);
static void task1_go_second_podium(void);
static void task1_go_champion_podium(void);
static void task1_go_third_podium(void);
static void task1_go_podium_point(uint8_t target_index, float y_offset_mm,
        uint8_t arrive_servo, uint8_t material_index, uint8_t prepare_servo,
        float prepare_tolerance_m);
static void task1_go_return_transition(void);
static void task1_go_home(void);

void start_task1(void) {
	/* 任务状态机从步骤 1 开始，step_start 记录当前步骤进入时间。 */
	task1_state.active = 1;
	task1_state.current_step = 1;
	task1_state.completed = 0;
	task1_state.step_start = HAL_GetTick();
}

void run_task1(void) {
	if (!task1_state.active) {
		return;
	}

	switch (task1_state.current_step) {
	case 1: task1_go_first_qrcode(); break;                   /* 起步到二维码点 1。 */
	case 2: task1_scan_material_colors(); break;              /* 转盘采集物料颜色。 */
	case 11: task1_go_first_material_point(); break;          /* a 点取第 1 个物料。 */
	case 12: task1_wait_vision(6); break;
	case 13: task1_relative_action(0, 0U); break;
	case 14: task1_go_material_point(7, 2, MIDSPEED); break;  /* b 点取第 2 个物料。 */
	case 15: task1_wait_vision(7); break;
	case 16: task1_relative_action(0, 0U); break;
	case 17: task1_go_material_point(8, 3, 0.0f); break;      /* c 点取第 3 个物料。 */
	case 18: task1_wait_vision(8); break;
	case 19: task1_relative_action(0, 0U); break;
	case 20: task1_go_material_point(9, 4, 0.0f); break;      /* d 点取第 4 个物料。 */
	case 21: task1_wait_vision(9); break;
	case 22: task1_relative_action(0, 0U); break;
	case 23: task1_go_material_point(10, 5, 0.0f); break;     /* e 点取第 5 个物料。 */
	case 24: task1_wait_vision(10); break;
	case 25: task1_relative_action(0, 0U); break;
	case 26: task1_go_second_qrcode(); break;                 /* 前往二维码点 2。 */
	case 27: task1_go_trophy_point(19, 0); break;             /* 奖杯区过渡点。 */
	case 28: task1_go_third_trophy(); break;                  /* 奖杯 3。 */
	case 29: task1_go_trophy_point(13, 1); break;             /* 奖杯 2。 */
	case 30: task1_go_first_trophy(); break;                  /* 奖杯 1。 */
	case 31: task1_go_second_podium(); break;                 /* 亚军台。 */
	case 32: task1_wait_vision(15); break;
	case 33: task1_relative_action(3, 100U); break;
	case 34: task1_go_champion_podium(); break;              /* 冠军台。 */
	case 35: task1_wait_vision(16); break;
	case 36: task1_relative_action(3, 0U); break;
	case 37: task1_go_third_podium(); break;                 /* 季军台。 */
	case 38: task1_wait_vision(17); break;
	case 39: task1_relative_action(0, 0U); break;
	case 40: task1_go_return_transition(); break;             /* 返航过渡点。 */
	case 41: task1_go_home(); break;                          /* 回起始点。 */
	default: task1_finish(); break;
	}
}
uint8_t is_task1_active(void) {
	return task1_state.active;
}

void start_task2(void) {
	/* 任务二按顺序测试局部 vx 前进、局部 vy 左移和 -90° 航向闭环。 */
	task2_state.active = 1U;
	task2_state.current_step = 1U;
	task2_state.completed = 0U;
	task2_state.step_start = HAL_GetTick();
	tasknum = 2U;
	MotionState_Reset();
	PoseCoordination_Enable(0U);
	PositionControl_Enable(0U);
	EnableAttitudeStabilization(0U);
	Is_vel_mode = 1U;
	SetTargetVelocity(TASK2_OPEN_LOOP_SPEED_MPS, 0.0f, 0.0f);
}

void run_task2(void) {
	if (!task2_state.active) {
		return;
	}

	switch (task2_state.current_step) {
	case 1U:
		if ((HAL_GetTick() - task2_state.step_start) >=
		        TASK2_OPEN_LOOP_TIME_MS) {
			stop();
			task2_state.current_step = 2U;
			task2_state.step_start = HAL_GetTick();
			SetTargetVelocity(0.0f, TASK2_OPEN_LOOP_SPEED_MPS, 0.0f);
		}
		break;

	case 2U:
		if ((HAL_GetTick() - task2_state.step_start) >=
		        TASK2_OPEN_LOOP_TIME_MS) {
			stop();
			EnableAttitudeStabilization(1U);
			SetTargetYaw(TASK2_TEST_YAW_DEG);
			task2_state.current_step = 3U;
			task2_state.step_start = HAL_GetTick();
		}
		break;

	case 3U:
		if (IsYawReached(TASK2_TEST_YAW_DEG, POSE_YAW_TOLERANCE_DEG)) {
			stop();
			EnableAttitudeStabilization(0U);
			task2_state.active = 0U;
			task2_state.completed = 1U;
		}
		break;

	default:
		stop();
		task2_state.active = 0U;
		task2_state.completed = 1U;
		break;
	}
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
 * @brief 起步到第一个二维码点。
 *
 * 这里保留接近二维码点时的舵机分段下压，现场调动作只改本函数。
 */
static void task1_go_first_qrcode(void) {
	EnableAttitudeStabilization(1);
	SetTargetYaw(0.0f);
	Move_to_turnplate(1);
	Servo_Action_Group(1);
	tasknum = 1;
	pospid_reinit(LOWSPEED, POSITION_PID_DEFAULT_KP);

	if (task1_move_target(0, 0.0f, 0.0f)) {
		Servo_Action_Group(2);
		task1_next_step();
	}

	/* 接近二维码点时逐段下压，减小末端一次动作带来的识别抖动。 */
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
}

/**
 * @brief 扫描转盘五个物料颜色。
 *
 * 转盘角度、底盘速度和延时都集中在这里，调颜色采集节奏时不用展开主流程。
 */
static void task1_scan_material_colors(void) {
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
	task1_state.step_start = HAL_GetTick();
}

/**
 * @brief 进入物块段第一个取放点。
 *
 * 这里额外开启姿态稳定，并补采第 5 个颜色点。
 */
static void task1_go_first_material_point(void) {
	EnableAttitudeStabilization(1);
	Get_Materialcolor(4);
	task1_go_material_point(6, 1, LOWSPEED);
}

/**
 * @brief 移动到物块区取放点并切换对应物料位。
 *
 * @param target_index task1_targets 下标。
 * @param material_index 物料序号，从 1 开始。
 * @param speed_mps 位置环速度，单位 m/s；传入 0 时沿用当前速度。
 */
static void task1_go_material_point(
        uint8_t target_index, uint8_t material_index, float speed_mps) {
	if (speed_mps > 0.0f) {
		pospid_reinit(speed_mps, POSITION_PID_DEFAULT_KP);
	}

	if (task1_move_target(target_index, 0.0f, -90.0f)) {
		Find_corresponding_material(material_index);
		task1_next_step();
	}
}

/**
 * @brief 等待视觉搜索结束并进入下一步。
 *
 * 当前策略是收到坐标或超时都继续，视觉等待语义集中在这里方便后续改策略。
 */
static void task1_wait_vision(uint8_t point_index) {
	(void)Wait_for_uart_with_search(point_index);
	task1_next_step();
}

/**
 * @brief 执行一次相对前后取放动作。
 *
 * @param servo_after 相对动作完成后的舵机动作组，0 表示不执行。
 * @param delay_after_ms 舵机动作后的等待时间，单位 ms。
 */
static void task1_relative_action(uint8_t servo_after, uint32_t delay_after_ms) {
	if (task1_relative_forward_back()) {
		if (servo_after != 0U) {
			Servo_Action_Group(servo_after);
		}
		if (delay_after_ms > 0U) {
			HAL_Delay(delay_after_ms);
		}
		task1_next_step();
	}
}

/**
 * @brief 前往第二个二维码点。
 *
 * 到点后将转盘准备到第 3 个物料位。
 */
static void task1_go_second_qrcode(void) {
	tasknum = 2;
	if (task1_move_target(11, 0.0f, 0.0f)) {
		Move_to_turnplate(3);
		task1_next_step();
	}
}

/**
 * @brief 移动到奖杯点并在到点后切换转盘。
 *
 * @param target_index task1_targets 下标。
 * @param turnplate_num 到点后切换的转盘物料位，0 表示不切换。
 */
static void task1_go_trophy_point(uint8_t target_index, uint8_t turnplate_num) {
	if (task1_move_target(target_index, 0.0f, -90.0f)) {
		task1_next_step();
		if (turnplate_num != 0U) {
			Move_to_turnplate(turnplate_num);
		}
	}
}

/**
 * @brief 低速进入奖杯 3，并准备奖杯 2。
 */
static void task1_go_third_trophy(void) {
	pospid_reinit(LOWESTSPEED, POSITION_PID_DEFAULT_KP);
	task1_go_trophy_point(12, 2);
}

/**
 * @brief 进入奖杯 1，并给转盘最后补偿量。
 */
static void task1_go_first_trophy(void) {
	if (task1_move_target(14, 0.0f, -90.0f)) {
		__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Material[1] + 200);
		task1_next_step();
	}
}

/**
 * @brief 前往亚军台。
 *
 * 接近途中先展开舵机 5，到点后执行舵机 6 并切第 2 个物料。
 */
static void task1_go_second_podium(void) {
	pospid_reinit(HIGHSPEED, POSITION_PID_DEFAULT_KP);
	task1_go_podium_point(15, 15.0f, 6, 2, 5, 0.6f);
}

/**
 * @brief 前往冠军台，到点后执行舵机 4 并切第 1 个物料。
 */
static void task1_go_champion_podium(void) {
	task1_go_podium_point(16, 15.0f, 4, 1, 0, 0.0f);
}

/**
 * @brief 前往季军台，到点后执行舵机 7 并切第 3 个物料。
 */
static void task1_go_third_podium(void) {
	task1_go_podium_point(17, 25.0f, 7, 3, 0, 0.0f);
}

/**
 * @brief 移动到奖台点。
 *
 * @param target_index task1_targets 下标。
 * @param y_offset_mm Y 方向附加偏移，单位 mm。
 * @param arrive_servo 到点后的舵机动作组。
 * @param material_index 到点后切换的目标物料序号。
 * @param prepare_servo 接近途中提前执行的舵机动作组，0 表示不执行。
 * @param prepare_tolerance_m 提前动作触发容差，单位 m。
 */
static void task1_go_podium_point(uint8_t target_index, float y_offset_mm,
        uint8_t arrive_servo, uint8_t material_index, uint8_t prepare_servo,
        float prepare_tolerance_m) {
	if (task1_move_target(target_index, y_offset_mm, -90.0f)) {
		Servo_Action_Group(arrive_servo);
		Find_corresponding_material(material_index);
		task1_next_step();
		return;
	}

	if (prepare_servo != 0U && IsPositionReached(prepare_tolerance_m)) {
		Servo_Action_Group(prepare_servo);
	}
}

/**
 * @brief 返回起点前的过渡点。
 */
static void task1_go_return_transition(void) {
	pospid_reinit(HIGHESTSPEED, POSITION_PID_DEFAULT_KP);
	raceh = POSITION_RETURN_REACHED_TOLERANCE_M;
	if (task1_move_target(20, 0.0f, -90.0f)) {
		task1_state.current_step = 41;
		task1_state.step_start = HAL_GetTick();
	}
}

/**
 * @brief 返回起始点并恢复精细到达容差。
 */
static void task1_go_home(void) {
	raceh = POSITION_FINE_REACHED_TOLERANCE_M;
	if (task1_move_target(18, 0.0f, -90.0f)) {
		task1_next_step();
	}
}
/***********Task1***************/
