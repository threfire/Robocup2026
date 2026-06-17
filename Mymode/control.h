#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

#define g 0.06f
#define yuanhuan 0x03
#define shizi 	 0x02
extern uint8_t cammode;
#define lookingangle 700
extern uint16_t Material[5];
void Setmaixcam_mode(uint8_t ch);

//********************************角度pid部分************************************//
#define DEAD_ZONE 0.1f  // 姿态死区，单位度
float GetAttitudeCorrection(void);  // 获取姿态校正量
extern float attitude_correction;   // 姿态校正量（外部可访问）
extern uint8_t isPosture_correction;	// 开启姿态校正标志位（外部可访问）
void AttitudeStabilization_Init(void);
void SetTargetYaw(float target);
uint8_t IsYawReached(float target_yaw, float tolerance_degrees);
void UpdateAttitudeStabilization(float dt);
void EnableAttitudeStabilization(uint8_t enable);
//********************************角度pid部分************************************//

//********************************循迹pid部分************************************//
typedef struct {
    float Kp;       // 比例系数
	float Kp2;
    float Ki;       // 积分系数
    float Kd;       // 微分系数
    float integral; // 积分项
    float prev_error; // 上一次误差
    float max_output; // 最大输出限制
} PID_Controller;

void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd, float max_output);
void PID_Init_cam(PID_Controller* pid, float Kp,float Kp2, float Ki, float Kd, float max_output);
float PID_Update(PID_Controller* pid, float error, float dt);
float PID_Update_cam(PID_Controller* pid, float error, float dt);
float PID_Update_angle(PID_Controller* pid, float error, float dt);
void Line_Follow_Init(void);
void Follow_line_task(void);
//********************************循迹pid部分************************************//

//********************************位置控制pid部分************************************//
#define HIGHESTSPEED 6.3f                           // 最高位置控制速度，单位m/s
#define HIGHSPEED 3.80f                             // 高速位置控制速度，单位m/s
#define MIDSPEED 2.0f                               // 中速位置控制速度，单位m/s
#define LOWSPEED  1.5f                              // 低速位置控制速度，单位m/s
#define LOWESTSPEED  0.55f                          // 最低位置控制速度，单位m/s
#define TASK1_RELATIVE_SPEED 0.75f                  // Task1相对位移动作速度，单位m/s
#define TASK1_RELATIVE_SWITCH_PAUSE_MS 20U          // Task1相对位移切换停顿时间，单位ms
#define POSITION_PID_DEFAULT_KP 7.0f                // 位置PID默认比例系数
#define POSITION_PID_DEFAULT_KD 0.05f               // 位置PID默认微分系数
#define POSITION_PID_TASK1_RELATIVE_KP 7.0f         // Task1相对位移使用的位置PID比例系数
#define POSITION_PLANNER_MIN_SPEED_MPS 0.20f        // 位置规划最小速度，单位m/s
#define POSITION_PLANNER_CRUISE_SPEED_MPS 0.90f     // 位置规划巡航速度，单位m/s
#define POSITION_PLANNER_MAX_ACCEL_MPS2 5.0f        // 位置规划最大加速度，单位m/s^2
#define POSITION_PLANNER_MAX_DECEL_MPS2 11.0f       // 位置规划最大减速度，单位m/s^2
#define POSITION_PLANNER_MAX_JERK_MPS3 18.0f        // 位置规划最大加加速度，单位m/s^3
#define POSITION_PLANNER_STOP_SPEED_MPS 0.04f       // 位置规划停止速度阈值，单位m/s
#define POSITION_PLANNER_NEAR_TARGET_DISTANCE_M 0.04f // 位置规划近目标距离阈值，单位m
#define POSITION_FEEDBACK_MAX_SPEED_MPS 0.35f       // 位置反馈速度上限，单位m/s
#define POSITION_REACHED_TOLERANCE_M 0.003f         // 默认位置到位容差，单位m
#define POSITION_FINE_REACHED_TOLERANCE_M 0.002f    // 精定位位置到位容差，单位m
#define POSITION_RETURN_REACHED_TOLERANCE_M 0.200f  // 回家段粗定位位置到位容差，单位m
#define POSE_SYNC_SLOW_DISTANCE_M 0.25f             // 姿态协同减速距离，单位m
#define POSE_SYNC_SLOW_YAW_DEG    34.0f             // 姿态协同减速角度，单位deg
#define POSE_SYNC_MIN_SCALE       0.60f             // 姿态协同最小缩放比例
#define POSE_YAW_TOLERANCE_DEG    2.0f              // 姿态到位角度容差，单位deg
#define POSE_YAW_DEAD_ZONE_DEG    0.1f              // 姿态死区，单位deg

#define MAKEUP_FOR_ANGLE 2.0f

typedef struct {
    PID_Controller pid_x;      // X轴位置PID
    PID_Controller pid_y;      // Y轴位置PID
    uint8_t is_enabled;        // 位置控制使能标志
    float target_x;            // 目标X坐标
    float target_y;            // 目标Y坐标
    uint32_t last_update_time; // 最后更新时间
} PositionController_t;

typedef struct {
    uint8_t is_enabled;       // 位姿协调使能
    float sync_scale;         // 平移和旋转共用减速比例
    float slow_distance_m;    // 平移减速距离，单位m
    float slow_yaw_deg;       // 旋转减速角度，单位deg
    float min_scale;          // 最小输出比例
    PID_Controller yaw_pid;   // yaw轴PID
    float target_yaw;         // 目标yaw角，单位deg
    uint8_t has_yaw_target;   // yaw目标有效标志
} PoseCoordination_t;

extern PositionController_t position_control;
extern uint8_t is_position_control_enabled;
extern PoseCoordination_t pose_coordination;

void PositionControl_Init(void);
void PositionControl_Enable(uint8_t enable);
void SetTargetPosition(float x, float y);
void UpdatePositionControl(float dt);
uint8_t IsPositionReached(float tolerance_xy);
void PoseCoordination_Init(void);
void PoseCoordination_Enable(uint8_t enable);
void SetTargetPose(float x, float y, float yaw);
void UpdatePoseCoordination(float dt);
uint8_t IsPoseReached(float tolerance_xy, float tolerance_yaw_degrees);
uint8_t move_to_pose_target(float x, float y, float yaw);
uint8_t move_to_target(float x, float y);
//********************************位置控制pid部分************************************//

//********************************摄像头校正pid部分************************************//
#define TOLERANCE 3
#define max_min_error 3
#define ERROR_QUEUE_SIZE 10		  // 定义误差队列长度
#define UART_WAIT_TIMEOUT_MS 500  // 超时时间，单位ms
#define CAM_STABILIZE_TIMEOUT_MS 1200  // 视觉校正等待超时时间，单位ms

typedef struct {
    int errors_x[ERROR_QUEUE_SIZE];
    int errors_y[ERROR_QUEUE_SIZE];
    uint8_t index;
    uint8_t count;
    uint8_t is_full;
} ErrorQueue_t;

typedef struct {
    PID_Controller pid_x;  // X轴PID
    PID_Controller pid_y;  // Y轴PID
    uint8_t is_enabled;    // 使能标志
} CamStabilization_t;

extern CamStabilization_t cam_stabilization;
extern float cam_correction_x; // X轴校正量
extern float cam_correction_y; // Y轴校正量

void CamStabilization_Init(void);
void CamStabilization_Enable(uint8_t enable);
void UpdateCamStabilization(float x_err, float y_err, float dt);
//********************************摄像头校正pid部分************************************//

//********************************圆弧跟踪部分************************************//
typedef struct {
    float center_x;        // 圆心x坐标，单位mm
    float center_y;        // 圆心y坐标，单位mm
    float radius;          // 圆弧半径，单位mm
    float start_angle;     // 起始角度，单位rad
    float end_angle;       // 结束角度，单位rad
    uint8_t direction;     // 方向：0逆时针，1顺时针
    float current_angle;   // 当前目标角度
    uint32_t start_time;   // 开始时间
    float total_time;      // 总运动时间
} ArcTrajectory_t;

typedef struct {
    PID_Controller angle_pid;    // 角度跟踪PID
    PID_Controller radius_pid;   // 半径跟踪PID
    uint8_t is_enabled;
} ArcTracking_t;

void ArcTracking_Init(void);
uint8_t MoveAlongArc(float center_x, float center_y, float radius,
                    float start_angle, float end_angle, uint8_t direction, float speed);
uint8_t IsArcMotionCompleted(void);
void ArcTracking_Enable(uint8_t enable);
void UpdateArcTracking(float dt);
//********************************圆弧跟踪部分************************************//

//********************************任务调度部分************************************//
typedef struct {
    uint8_t active;         // 任务是否激活
    uint8_t current_step;   // 当前执行步骤
    uint32_t step_start;    // 当前步骤开始时间
	uint8_t completed;      // 任务结束时置1
} TaskState_t;

void start_task1(void);
void run_task1(void);
uint8_t is_task1_active(void);
//********************************任务调度部分************************************//

typedef struct {
    float x;
    float y;
} Point2D_t;

typedef struct {
    float vx;
    float vy;
    float omega;
} TargetVelocity_t;

extern TargetVelocity_t target_velocity; // 全局目标速度

typedef struct {
    int16_t speed_rpm;  // 转速范围：-32768~32767 RPM
	uint8_t direction;  // 方向标识：0停止，1正转，2反转
} Motor_t;

typedef struct {
    float wheel_radius;  // 轮子半径，单位m
    float wheel_dist_x;  // X轴轮距，单位m
    float wheel_dist_y;  // Y轴轮距，单位m
    float max_rpm;       // 电机最大转速，单位RPM
} ChassisConfig_t;

extern Motor_t motor1, motor2, motor3, motor4;
void SetTargetVelocity(float vx, float vy, float omega);
void SetTargetLinearVelocity(float vx, float vy);
extern uint8_t task1state;

void Mecanum_Init(ChassisConfig_t config);
void Mecanum_Kinematics(float vx, float vy, float omega);

void stop(void);
void go(void);
void back(void);
void left(void);
void right(void);
void turn_left(void);
void turn_right(void);
void shengjiang(void);
void fangzhi(void);
void jiaozhun(void);
void POS_move(void);
uint8_t task1_relative_forward_back(void);

void task1(void);
void Motor_tick(void);
#endif
