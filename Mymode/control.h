#ifndef CONTROL_H
#define CONTROL_H /* 控制模块头文件保护宏 */

#include "main.h"
#include "pid.h"

/* 转盘和视觉识别参数 */
#define MATERIAL_RING_MODE 0x03U       /* 转盘环形识别模式编号 */
#define MATERIAL_CROSS_MODE 0x02U      /* 转盘十字识别模式编号 */
#define MATERIAL_LOOKING_ANGLE 700U    /* 视觉搜索角度命令 */

/* 姿态控制参数 */
#define ATTITUDE_DEAD_ZONE_DEG 0.05f    /* 姿态闭环死区，单位 deg */
#define ATTITUDE_PID_DEFAULT_KP 15.50f /* 姿态角度环比例系数，单位 rad/s/rad */
#define ATTITUDE_PID_DEFAULT_KI 0.05f   /* 姿态角度环积分系数 */
#define ATTITUDE_PID_DEFAULT_KD 1.05f /* 姿态角度环角速度阻尼系数，单位 s */
#define ATTITUDE_PID_MAX_OUTPUT 9.00f   /* 姿态角度环输出上限，单位 rad/s */
#define ATTITUDE_ACCEL_FEEDFORWARD_GAIN 1.0f /* 偏航 J·α 加速度前馈系数 */
#define ATTITUDE_ACCEL_FEEDBACK_GAIN 0.15f /* 偏航加速度反馈增益 */
#define ATTITUDE_FF_MAX_ACCEL_RAD_S2 50.0f /* 偏航前馈最大角加速度，单位 rad/s² */
#define ATTITUDE_FF_MAX_OMEGA_RAD_S 3.8f /* 偏航前馈最大角速度，单位 rad/s */
#define ATTITUDE_PLANNER_MAX_ACCEL_RAD_S2 65.0f /* 偏航规划最大角加速度，单位 rad/s² */
#define ATTITUDE_PLANNER_MAX_DECEL_RAD_S2 50.0f /* 偏航规划最大角减速度，单位 rad/s² */
#define ATTITUDE_PLANNER_MAX_JERK_RAD_S3 120.0f /* 偏航规划最大角加加速度，单位 rad/s³ */
#define FUZZY_PID_ENABLE 1U /* 模糊自适应 PID 使能标志 */
#define ATTITUDE_FUZZY_ERROR_FULL_DEG 15.0f /* 偏航误差达到最高隶属度的阈值，单位 deg */
#define ATTITUDE_FUZZY_RATE_FULL_DPS 60.0f /* 偏航误差变化率达到最高隶属度的阈值，单位 deg/s */
#define ATTITUDE_FUZZY_KP_MIN_SCALE 0.90f /* 偏航小误差时 Kp 缩放系数 */
#define ATTITUDE_FUZZY_KP_MAX_SCALE 1.15f /* 偏航大误差时 Kp 缩放系数 */
#define ATTITUDE_FUZZY_KD_MIN_SCALE 0.90f /* 偏航平稳时 Kd 缩放系数 */
#define ATTITUDE_FUZZY_KD_MAX_SCALE 1.20f /* 偏航快速变化时 Kd 缩放系数 */
#define ATTITUDE_FUZZY_RATE_FILTER_ALPHA 0.20f /* 偏航误差变化率滤波系数 */
#define ATTITUDE_FF_ACCEL_FILTER_ALPHA 0.20f /* 偏航期望角加速度滤波系数 */

/* 位置控制速度参数，单位 m/s */
#define POSITION_SPEED_HIGHEST_MPS 9.3f /* 最高位置控制速度，单位 m/s */
#define POSITION_SPEED_HIGH_MPS 3.80f   /* 高速位置控制速度，单位 m/s */
#define POSITION_SPEED_MID_MPS 2.0f     /* 中速位置控制速度，单位 m/s */
#define POSITION_SPEED_LOW_MPS 1.5f     /* 低速位置控制速度，单位 m/s */
#define POSITION_SPEED_LOWEST_MPS 0.65f /* 最低位置控制速度，单位 m/s */
#define TASK1_EMM_BACKWARD_SPEED_RPM 200U /* 任务一 EMM 后退转速，单位 RPM */
#define TASK1_EMM_BACKWARD_PULSES 1442U  /* 任务一 EMM 后退脉冲数 */
#define TASK1_EMM_BACKWARD_WAIT_MS 400U  /* 任务一 EMM 后退等待时间，单位 ms */
#define TASK1_HOME_CRUISE_SPEED_MPS 2.20f /* 回家巡航速度，单位 m/s */
#define TASK1_HOME_APPROACH_SPEED_MPS 1.50f /* 回家接近速度，单位 m/s */
#define TASK1_HOME_APPROACH_DISTANCE_M 0.50f /* 回家降速距离，单位 m */
#define TASK2_TEST_SPEED_MPS 0.5f        /* 任务二测试速度，单位 m/s */
/* 位置控制 PID 和轨迹规划参数 */
#define POSITION_PID_DEFAULT_KP 10.50f /* 位置 PID 比例系数 */
#define POSITION_PID_DEFAULT_KD 1.4555565f /* 位置 PID 微分系数 */
#define POSITION_FUZZY_ERROR_FULL_M 0.30f /* 位置误差达到最高隶属度的阈值，单位 m */
#define POSITION_FUZZY_RATE_FULL_MPS 1.00f /* 位置误差变化率达到最高隶属度的阈值，单位 m/s */
#define POSITION_FUZZY_KP_MIN_SCALE 0.90f /* 位置小误差时 Kp 缩放系数 */
#define POSITION_FUZZY_KP_MAX_SCALE 1.50f /* 位置大误差时 Kp 缩放系数 */
#define POSITION_FUZZY_KD_MIN_SCALE 0.80f /* 位置平稳时 Kd 缩放系数 */
#define POSITION_FUZZY_KD_MAX_SCALE 1.50f /* 位置快速变化时 Kd 缩放系数 */
#define POSITION_FUZZY_RATE_FILTER_ALPHA 0.35f /* 位置误差变化率滤波系数 */
#define POSITION_PLANNER_MIN_SPEED_MPS 0.40f /* 速度规划器最小速度，单位 m/s */
#define POSITION_PLANNER_CRUISE_SPEED_MPS 2.0f /* 速度规划器巡航速度，单位 m/s */
#define POSITION_PLANNER_MAX_ACCEL_MPS2 12.0f /* 速度规划器最大加速度，单位 m/s² */
#define POSITION_PLANNER_MAX_DECEL_MPS2 18.0f /* 速度规划器最大减速度，单位 m/s² */
#define POSITION_PLANNER_MAX_JERK_MPS3 3000.0f /* 速度规划器最大加加速度，单位 m/s³ */
#define POSITION_COMMAND_BRAKE_DECEL_MPS2 5.00f /* 总速度命令制动包络，单位 m/s² */
#define MOTION_OBSERVER_ALPHA 0.25f /* 车体状态观测器低通系数 */
#define MOTION_MPC_HORIZON_STEPS 50U /* MPC 预测步数 */
#define MOTION_MPC_STEP_S 0.01f /* MPC 预测步长，单位 s */
#define MOTION_MPC_MAX_ACCEL_MPS2 12.0f /* MPC 线加速度约束，单位 m/s² */
#define MOTION_MPC_MAX_ANG_ACCEL_RAD_S2 20.0f /* MPC 角加速度约束，单位 rad/s² */
#define MOTION_MPC_TERMINAL_DISTANCE_WEIGHT 40.0f /* 终端距离代价权重 */
#define MOTION_MPC_TERMINAL_SPEED_WEIGHT 0.5f /* 终端速度代价权重 */
#define MOTION_MPC_OVERSHOOT_WEIGHT 200.0f /* 预测越过目标代价权重 */
#define MOTION_MPC_CRUISE_PREFERENCE_DISTANCE_M 0.25f /* 巡航候选偏好距离 */
#define MOTION_MPC_CRUISE_PREFERENCE_GAIN 300.0f /* 巡航候选偏好增益 */
#define MOTION_MPC_LOW_SPEED_PREFERENCE_DISTANCE_M 0.08f /* 低速候选偏好启用距离，单位 m */
#define MOTION_MPC_LOW_SPEED_PREFERENCE_SPEED_MPS 0.4f /* 低速候选偏好速度基准，单位 m/s */
#define MOTION_MPC_LOW_SPEED_PREFERENCE_GAIN 8.0f /* 低速候选代价增益 */
#define POSITION_PLANNER_STOP_SPEED_MPS 0.04f /* 判定停止的规划速度，单位 m/s */
#define POSITION_PLANNER_NEAR_TARGET_DISTANCE_M 0.005f /* 近目标距离阈值，单位 m */
#define POSITION_FEEDBACK_MAX_SPEED_MPS 0.70f /* 速度反馈参与上限，单位 m/s */
#define POSITION_STATIC_FRICTION_ENABLE_DISTANCE_M 0.030f /* 静摩擦补偿启用距离，单位 m */
#define POSITION_STATIC_FRICTION_SPEED_MPS 0.0700433f /* 静摩擦补偿速度，单位 m/s */
#define CHASSIS_MASS_KG 5.888f /* 底盘质量，单位 kg */
#define CHASSIS_CENTER_OF_MASS_X_M - 0.050f /* 质心相对车体中心的前向偏移，单位 m */
#define CHASSIS_CENTER_OF_MASS_Y_M 0.0f /* 质心相对车体中心的右向偏移，单位 m */
#define POSITION_ACCEL_FEEDBACK_GAIN 0.274512f /* 位置环加速度反馈增益 */
#define POSITION_FORCE_TO_SPEED_FF_MPS_PER_N 0.0103671f /* 作用力前馈转速度系数，单位 m/(s·N) */
#define POSITION_ACCEL_FEEDBACK_FILTER_ALPHA 0.15f /* 加速度反馈低通滤波系数 */

#define POSITION_REACHED_TOLERANCE_M 0.003f /* 常规到点容差，单位 m */
#define POSITION_FINE_REACHED_TOLERANCE_M 0.003f /* 精确到点容差，单位 m */
#define POSITION_RETURN_REACHED_TOLERANCE_M 0.200f /* 回程到点容差，单位 m */

/* 姿态协同参数 */
#define POSE_SYNC_SLOW_DISTANCE_M 0.25f /* 位姿协同降速距离，单位 m */
#define POSE_SYNC_SLOW_YAW_DEG 34.0f    /* 位姿协同降速偏航误差，单位 deg */
#define POSE_SYNC_MIN_SCALE 1.00f       /* 位姿协同最小速度缩放系数 */
#define POSE_SYNC_YAW_RATE_FULL_RAD_S 3.0f /* 偏航角速度满量程，单位 rad/s */
#define POSE_SYNC_YAW_RATE_MIN_SCALE 0.50f /* 偏航运动时平移最低缩放系数 */
#define POSE_YAW_TOLERANCE_DEG 1.0f     /* 位姿到点偏航容差，单位 deg */
#define POSE_YAW_DEAD_ZONE_DEG 0.05f     /* 位姿偏航死区，单位 deg */
#define MAKEUP_FOR_ANGLE 2.0f           /* 偏航角补偿量，单位 deg */

/* 视觉稳定参数 */
#define CAM_REACHED_TOLERANCE 3         /* 视觉到位像素容差 */
#define CAM_MAX_MIN_ERROR 3             /* 视觉误差波动上限 */
#define ERROR_QUEUE_SIZE 10U            /* 视觉误差队列长度 */
#define UART_WAIT_TIMEOUT_MS 800U       /* 串口等待超时，单位 ms */
#define CAM_STABILIZE_TIMEOUT_MS 1200U  /* 视觉稳定超时，单位 ms */

/* EMM 速度与位置模式参数。 */
/* EMM 输出轴相对轮速模型存在约 1.8 倍的速度换算比例。 */
#define CHASSIS_OPEN_LOOP_WHEEL_SPEED_SCALE 1.80f   /* 开环 EMM 转速到轮速换算系数 */
#define CHASSIS_CLOSED_LOOP_WHEEL_SPEED_SCALE 2.00f /* 闭环 EMM 转速到轮速换算系数 */
#define CHASSIS_OPEN_LOOP_MAX_RPM 700.0f    /* 开环速度模式最大转速，单位 RPM */
#define CHASSIS_CLOSED_LOOP_MAX_RPM 1000.0f /* 闭环控制最大转速，单位 RPM */
/* 速度斜坡由位置规划器生成，EMM 采用 0 以避免内部 RPM 斜坡叠加延迟。 */
#define CHASSIS_OPEN_LOOP_VELOCITY_ACCEL 0U  /* 开环 EMM 速度模式内部加速度 */
#define CHASSIS_CLOSED_LOOP_VELOCITY_ACCEL 0U /* 闭环 EMM 速度模式内部加速度 */
#define CHASSIS_POS_FORWARD_ACCEL 100U  /* EMM 前进位置模式加速度 */
#define CHASSIS_POS_BACKWARD_ACCEL 100U  /* EMM 后退位置模式加速度 */
#define POSITION_MAX_LINEAR_SPEED_MPS 7.0f /* 位置控制最大线速度，单位 m/s */
#define ATTITUDE_MAX_OMEGA 20.0f        /* 姿态控制最大角速度，单位 rad/s */
#define CAM_DEAD_ZONE 0.0f              /* 视觉控制死区 */
#define CAM_ERROR_ABS_LIMIT 480         /* 视觉误差绝对值上限 */
/* 转盘 ZDT 电机 5 的绝对位置参数，位置单位为驱动器脉冲。 */
#define TURNPLATE_MOTOR_ADDR 5U
#define TURNPLATE_DIRECTION 0U
#define TURNPLATE_SPEED_RPM 300U
#define TURNPLATE_ACCEL 230U
#define TURNPLATE_SLOT_COUNT 6U
/* 转盘单圈零点保存与上电回零配置。 */
#define TURNPLATE_ORIGIN_SAVE_ON_STARTUP 0U /* 首次标定后改为 0U */
#define TURNPLATE_ORIGIN_MODE 0U

/* 兼容旧任务代码的速度宏 */
#define HIGHESTSPEED POSITION_SPEED_HIGHEST_MPS /* 兼容旧代码的最高速度 */
#define HIGHSPEED POSITION_SPEED_HIGH_MPS       /* 兼容旧代码的高速速度 */
#define MIDSPEED POSITION_SPEED_MID_MPS         /* 兼容旧代码的中速速度 */
#define LOWSPEED POSITION_SPEED_LOW_MPS         /* 兼容旧代码的低速速度 */
#define LOWESTSPEED POSITION_SPEED_LOWEST_MPS   /* 兼容旧代码的最低速度 */
#define DEAD_ZONE ATTITUDE_DEAD_ZONE_DEG        /* 兼容旧代码的姿态死区 */
#define TOLERANCE CAM_REACHED_TOLERANCE         /* 兼容旧代码的视觉容差 */
#define max_min_error CAM_MAX_MIN_ERROR          /* 兼容旧代码的视觉误差上限 */
#define g 0.06f                                  /* 旧姿态补偿比例系数 */
#define yuanhuan MATERIAL_RING_MODE              /* 兼容旧代码的环形识别模式 */
#define shizi MATERIAL_CROSS_MODE                /* 兼容旧代码的十字识别模式 */
#define lookingangle MATERIAL_LOOKING_ANGLE      /* 兼容旧代码的视觉搜索角度 */

typedef struct {
	PID_Controller pid_x;   /* X 轴位置 PID */
	PID_Controller pid_y;   /* Y 轴位置 PID */
	uint8_t is_enabled;     /* 位置环使能标志 */
	float target_x;         /* 目标 X 坐标，单位 mm */
	float target_y;         /* 目标 Y 坐标，单位 mm */
	uint32_t last_update_time; /* 最后更新时间，单位 ms */
} PositionController_t;

typedef struct {
	uint8_t is_enabled;      /* 位姿协同使能标志 */
	float sync_scale;        /* 位置环和姿态环同步缩放系数 */
	float slow_distance_m;   /* 进入降速区的位置误差，单位 m */
	float slow_yaw_deg;      /* 进入降速区的 yaw 误差，单位 deg */
	float min_scale;         /* 最小同步缩放系数 */
	PID_Controller yaw_pid;  /* 预留 yaw PID */
	float target_yaw;        /* 协同目标 yaw，单位 deg */
	uint8_t has_yaw_target;  /* yaw 目标有效标志 */
} PoseCoordination_t;

typedef struct {
	PID_Controller yaw_pid;  /* 偏航角 PID 控制器 */
	float target_yaw;        /* 目标偏航角，单位 deg */
	uint8_t is_enabled;      /* 姿态环使能标志 */
	uint32_t last_update_time; /* 最后更新时间，单位 ms */
} AttitudeStabilization_t;

typedef struct {
	PID_Controller pid_x; /* 视觉 X 轴 PID */
	PID_Controller pid_y; /* 视觉 Y 轴 PID */
	uint8_t is_enabled;   /* 视觉稳定使能标志 */
} CamStabilization_t;

typedef struct {
	float center_x;      /* 圆心 X 坐标，单位 mm */
	float center_y;      /* 圆心 Y 坐标，单位 mm */
	float radius;        /* 圆弧半径，单位 mm */
	float start_angle;   /* 起始角度，单位 rad */
	float end_angle;     /* 结束角度，单位 rad */
	uint8_t direction;   /* 方向：0 逆时针，1 顺时针 */
	float current_angle; /* 当前目标角度，单位 rad */
	uint32_t start_time; /* 圆弧运动开始时间，单位 ms */
	float total_time;    /* 圆弧运动总时间，单位 s */
} ArcTrajectory_t;

typedef struct {
	PID_Controller angle_pid;  /* 圆弧角度跟踪 PID */
	PID_Controller radius_pid; /* 圆弧半径跟踪 PID */
	uint8_t is_enabled;        /* 圆弧跟踪使能标志 */
} ArcTracking_t;

typedef struct {
	uint8_t active;       /* 任务激活标志 */
	uint8_t current_step; /* 当前任务步骤 */
	uint32_t step_start;  /* 当前步骤开始时间，单位 ms */
	uint8_t completed;    /* 任务完成标志 */
} TaskState_t;

typedef struct {
	float x; /* 世界系 X 坐标，单位 mm */
	float y; /* 世界系 Y 坐标，单位 mm */
	float yaw_deg; /* 到达目标点时的偏航角，单位 deg */
} Point2D_t;

typedef struct {
	float vx;    /* 底盘局部 X 速度，单位 m/s */
	float vy;    /* 底盘局部 Y 速度，单位 m/s */
	float omega; /* 底盘角速度，单位 rad/s */
} TargetVelocity_t;

typedef struct {
	int16_t speed_rpm; /* X42 速度命令，单位 RPM */
	uint8_t direction; /* X42 方向位 */
} Motor_t;

typedef struct {
	float wheel_radius;      /* 轮子半径，单位 m */
	float wheel_dist_x;      /* X 轴轮距，单位 m */
	float wheel_dist_y;      /* Y 轴轮距，单位 m */
	float max_rpm;           /* 最大电机转速，单位 RPM */
} ChassisConfig_t;

typedef struct {
	float race_tolerance_m;             /* 当前任务到点容差，单位 m */
	uint32_t material_position_pulses[TURNPLATE_SLOT_COUNT + 1U]; /* 初始点及转盘绝对脉冲位置 */
	TaskState_t task1;                  /* 任务 1 状态机 */
	TaskState_t task2;                  /* 任务 2 状态机 */
	uint8_t task_num;                   /* 当前任务编号 */
	uint8_t cam_mode;                   /* MaixCam 工作模式 */
	uint8_t look_mode_control;          /* 视觉搜索模式控制标志 */
	PositionController_t position;      /* 位置控制状态 */
	uint8_t position_enabled;           /* 位置控制外部使能镜像 */
	PoseCoordination_t pose;            /* 位姿协同状态 */
	AttitudeStabilization_t attitude;   /* 姿态控制状态 */
	float attitude_correction;          /* 姿态环角速度修正量 */
	uint8_t posture_correction_enabled; /* 姿态校正任务标志 */
	uint8_t position_test_flag;         /* 位置环测试到点标志 */
	uint8_t rotating;                   /* 非阻塞转向状态 */
	CamStabilization_t cam;             /* 视觉稳定控制状态 */
	float cam_correction_x;             /* 视觉 X 修正量 */
	float cam_correction_y;             /* 视觉 Y 修正量 */
	uint8_t cam_stabilizing;            /* 视觉稳定过程标志 */
	int coord_ready;                    /* 视觉坐标有效标志 */
	uint8_t cam_reached_count;          /* 视觉连续到达计数 */
	TargetVelocity_t target_velocity;   /* 当前底盘目标速度 */
	float error_x_global_m;             /* 世界系 X 坐标误差，单位 m */
	float error_y_global_m;             /* 世界系 Y 坐标误差，单位 m */
	float error_x_local_m;              /* 车体系 X 坐标误差，单位 m */
	float error_y_local_m;              /* 车体系 Y 坐标误差，单位 m */
	float yaw_error_deg;                /* 当前航向误差，单位 deg */
	Motor_t motor[4];                   /* 四个 X42 电机输出状态 */
	uint8_t run_enabled;                /* 底盘运行使能 */
	uint8_t line_follow_enabled;        /* 循迹控制使能 */
	uint8_t target_set;                 /* 非阻塞目标已下发标志 */
	uint8_t color_ready;                /* 颜色传感器数据有效标志 */
} ControlContext_t;

extern ControlContext_t control_ctx;
extern const int Telescopic_ring[];
#define raceh control_ctx.race_tolerance_m /* 当前任务到点容差 */
#define TurnplatePosition control_ctx.material_position_pulses /* 转盘绝对位置表 */
#define task1_state control_ctx.task1 /* 任务一状态机 */
#define task2_state control_ctx.task2 /* 任务二状态机 */
#define tasknum control_ctx.task_num /* 当前任务编号 */
#define cammode control_ctx.cam_mode /* MaixCam 工作模式 */
#define lookmode_centrl control_ctx.look_mode_control /* 视觉搜索模式控制标志 */
#define position_control control_ctx.position /* 位置控制状态 */
#define is_position_control_enabled control_ctx.position_enabled /* 位置控制外部使能 */
#define pose_coordination control_ctx.pose /* 位姿协同状态 */
#define attitude_control control_ctx.attitude /* 姿态控制状态 */
#define attitude_correction control_ctx.attitude_correction /* 姿态角速度修正量 */
#define isPosture_correction control_ctx.posture_correction_enabled /* 姿态校正使能 */
#define testflag control_ctx.position_test_flag /* 位置测试到点标志 */
#define is_rotating control_ctx.rotating /* 非阻塞转向状态 */
#define cam_stabilization control_ctx.cam /* 视觉稳定控制状态 */
#define cam_correction_x control_ctx.cam_correction_x /* 视觉 X 轴修正量 */
#define cam_correction_y control_ctx.cam_correction_y /* 视觉 Y 轴修正量 */
#define is_cam_stabilizing control_ctx.cam_stabilizing /* 视觉稳定过程标志 */
#define Is_coordgot control_ctx.coord_ready /* 视觉坐标有效标志 */
#define iscamreached_safecount control_ctx.cam_reached_count /* 视觉连续到点计数 */
#define target_velocity control_ctx.target_velocity /* 底盘目标速度 */
#define motor1 control_ctx.motor[0] /* 1 号 X42 电机状态 */
#define motor2 control_ctx.motor[1] /* 2 号 X42 电机状态 */
#define motor3 control_ctx.motor[2] /* 3 号 X42 电机状态 */
#define motor4 control_ctx.motor[3] /* 4 号 X42 电机状态 */
#define IsRun control_ctx.run_enabled /* 底盘运行使能 */
#define is_line_follow_enabled control_ctx.line_follow_enabled /* 循迹控制使能 */
#define target_set control_ctx.target_set /* 非阻塞目标已下发标志 */
#define is_hasgetcolor control_ctx.color_ready /* 颜色传感器数据有效标志 */

float GetAttitudeCorrection(void);
void Line_Follow_Init(void);
void Follow_line_task(void);
void AttitudeStabilization_Init(void);
void SetTargetYaw(float target);
uint8_t IsYawReached(float target_yaw, float tolerance_degrees);
void UpdateAttitudeStabilization(float dt);
void EnableAttitudeStabilization(uint8_t enable);
void PositionControl_Init(void);
void PositionControl_Enable(uint8_t enable);
void PositionControl_SetMaxSpeed(float max_speed_mps);
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
void ArcTracking_Init(void);
uint8_t MoveAlongArc(float center_x, float center_y, float radius,
        float start_angle, float end_angle, uint8_t direction, float speed);
uint8_t IsArcMotionCompleted(void);
void ArcTracking_Enable(uint8_t enable);
void UpdateArcTracking(float dt);
void start_task1(void);
void run_task1(void);
uint8_t is_task1_active(void);
void start_task2(void);
void run_task2(void);
uint8_t is_task2_active(void);
void SetTargetVelocity(float vx, float vy, float omega);
void SetTargetLinearVelocity(float vx, float vy);
void Mecanum_Init(ChassisConfig_t config);
void Mecanum_Kinematics(float vx, float vy, float omega);
void Mecanum_SendVelocity(void);
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
void task1(void);
void Motor_tick(void);

#endif
