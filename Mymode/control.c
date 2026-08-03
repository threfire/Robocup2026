#include "control.h"
#include <math.h>
#include <string.h>
#include "lcd_spi_154.h"
#include "odometer.h"
#include "servo.h"
#include "graysensor.h"
#include "userkey.h"
#include "vision.h"
#include "QAma.h"
#include "ai_tune.h"
#include "fuzzy.h"
#include "motion_plan.h"
#include "feedforward.h"
#include "motion.h"

/* 控制模块运行状态统一收敛到 control_ctx，旧任务代码通过宏访问字段。 */
ControlContext_t control_ctx = {
        .race_tolerance_m = POSITION_REACHED_TOLERANCE_M,
        .material_pwm = { 2340, 2340-290*1, 2340-290*2, 2340-290*3, 2340-290*4 +10, 2340-290*5, 2340-290*6},
        .task_num = 1,
        .cam_mode = 2,
        .look_mode_control = 1,
};

/* 任务 1 路径点，单位 mm，坐标系与里程计输出一致。 */
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
	motion_reset();
	Is_vel_mode = 1;
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

	float omega = pid_calc(&line_follow_pid, error, dt);

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
