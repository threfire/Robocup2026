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
#define DEAD_ZONE 0.1f  // 死区大小(度)
float GetAttitudeCorrection(void);  // 获取姿态校正量
extern float attitude_correction;   // 姿态校正量（外部可访问）
extern uint8_t isPosture_correction;	// 开启姿态校正标志位（外部可访问）
void AttitudeStabilization_Init(void);
void SetTargetYaw(float target);
void UpdateAttitudeStabilization(float dt);
void EnableAttitudeStabilization(uint8_t enable);
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
void PID_Init_cam(PID_Controller* pid, float Kp,float Kp2, float Ki, float Kd, float max_output) ;
float PID_Update(PID_Controller* pid, float error, float dt);
float PID_Update_cam(PID_Controller* pid, float error, float dt);
float PID_Update_angle(PID_Controller* pid, float error, float dt);
void Line_Follow_Init(void);
void Follow_line_task(void);
//********************************循迹pid部分************************************//
//********************************位置控制pid部分************************************//
#define HIGHESTSPEED 7.0f
#define HIGHSPEED 5.4f
#define MIDSPEED 3.0f
#define LOWSPEED  1.2f
#define LOWESTSPEED  1.0f
#define POSE_SYNC_SLOW_DISTANCE_M 0.35f
#define POSE_SYNC_SLOW_YAW_DEG    35.0f
#define POSE_SYNC_MIN_SCALE       0.15f
#define POSE_YAW_TOLERANCE_DEG    2.0f

#define MAKEUP_FOR_ANGLE 2.0f
// 位置控制PID结构体
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
} PoseCoordination_t;
// 声明全局位置控制变量
extern PositionController_t position_control;
extern uint8_t is_position_control_enabled;
extern PoseCoordination_t pose_coordination;

// 位置控制函数声明
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
//********************************摄像头校正pid部分************************************//
#define TOLERANCE 3
#define max_min_error 3
#define ERROR_QUEUE_SIZE 10		  // 定义误差队列长度
#define UART_WAIT_TIMEOUT_MS 500  // 超时时间，根据需要调整

typedef struct {				// 定义误差队列结构体
    int errors_x[ERROR_QUEUE_SIZE];
    int errors_y[ERROR_QUEUE_SIZE];
    uint8_t index;
    uint8_t count;
    uint8_t is_full;
} ErrorQueue_t;

// 摄像头PID结构体
typedef struct {
    PID_Controller pid_x;  // X轴PID
    PID_Controller pid_y;  // Y轴PID
    uint8_t is_enabled;    // 使能标志
} CamStabilization_t;

// 声明全局摄像头校正变量
extern CamStabilization_t cam_stabilization;
extern float cam_correction_x; // X轴校正量
extern float cam_correction_y; // Y轴校正量

// 声明函数
void CamStabilization_Init(void);
void CamStabilization_Enable(uint8_t enable);
void UpdateCamStabilization(float x_err, float y_err, float dt);
//********************************摄像头校正pid部分************************************//
// 在 control.h 中添加
typedef struct {
    float center_x;        // 圆心x坐标（毫米）
    float center_y;        // 圆心y坐标（毫米）
    float radius;          // 圆弧半径（毫米）
    float start_angle;     // 起始角度（弧度）
    float end_angle;       // 结束角度（弧度）
    uint8_t direction;     // 方向: 0-逆时针, 1-顺时针
    float current_angle;   // 当前目标角度
    uint32_t start_time;   // 开始时间
    float total_time;      // 总运动时间
} ArcTrajectory_t;

// 圆弧跟踪PID结构体
typedef struct {
    PID_Controller angle_pid;    // 角度跟踪PID
    PID_Controller radius_pid;   // 半径跟踪PID
    uint8_t is_enabled;
} ArcTracking_t;

// 函数声明
void ArcTracking_Init(void);
uint8_t MoveAlongArc(float center_x, float center_y, float radius, 
                    float start_angle, float end_angle, uint8_t direction, float speed);
uint8_t IsArcMotionCompleted(void);
void ArcTracking_Enable(uint8_t enable);
void UpdateArcTracking(float dt);
//********************************任务调度部分************************************//
// 任务状态结构体
typedef struct {
    uint8_t active;         // 任务是否激活
    uint8_t current_step;   // 当前执行步骤
    uint32_t step_start;    // 当前步骤开始时间
	uint8_t completed;      // 任务是否完成，任务结束时置1
} TaskState_t;
// 任务调度函数1
void start_task1(void);
void run_task1(void);
uint8_t is_task1_active(void);

//********************************任务调度部分************************************//
typedef struct {
    float x;
    float y;
} Point2D_t;

// 全局目标速度结构体
typedef struct {
    float vx;
    float vy;
    float omega;
} TargetVelocity_t;

extern TargetVelocity_t target_velocity; // 全局目标速度
// 电机结构体（速度和方向）
typedef struct {
    int16_t speed_rpm;      // 转速范围：-32768~32767 RPM（有符号数表示方向）
	uint8_t direction;  // 方向标识：0=停止，1=正转，2=反转（可自定义）
} Motor_t;

// 底盘参数配置
typedef struct {
    float wheel_radius;  // 轮子半径（米）
    float wheel_dist_x;  // X轴轮距（米）
    float wheel_dist_y;  // Y轴轮距（米）
    float max_rpm;       // 电机最大转速（RPM）
} ChassisConfig_t;


// 获取电机控制数据（外部调用）
extern Motor_t motor1, motor2, motor3, motor4;
void SetTargetVelocity(float vx, float vy, float omega);
void SetTargetLinearVelocity(float vx, float vy);
extern uint8_t task1state;

// 初始化底盘
void Mecanum_Init(ChassisConfig_t config);

// 运动解算函数
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

void task1(void);
void Motor_tick(void);
#endif
