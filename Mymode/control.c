#include "control.h"
#include <math.h>
#include <string.h>
#include "lcd_spi_154.h"
#include "my_delay.h"
#include "hwt_imu.h"
#include "servo.h"
#include "graysensor.h"
#include "userkey.h"
#include "QAma.h"
#include "mxcam.h"


float raceh = 0.002f;  // 位置环默认容差，单位：m，2mm
uint16_t Material[5] = {575,970,1365,1785,2165};//转盘坐标
// 任务1的目标点坐标数组
static const Point2D_t task1_targets[] = {
    {250.25f, 650.0f},     // 索引0		点1扫二维码1
    {258.25f, 1058.24f},   // 索引1		点2物块1
    {604.89f, 1399.38f},   // 索引2		点3物块2
    {1075.0f, 1524.0f},    // 索引3		点4物块3
    {1545.10f, 1399.38f},  // 索引4		点5物块4
    {1891.74f, 1058.24f},  // 索引5		点6物块5
	{835.14f, 631.60f},     // 索引6		点7  a
	{553.90f, 425.91f},	   // 索引7		点8  b
	{836.02f, -220.80f},   // 索引8		点9  c
	{554.98f, -334.63f},   // 索引9		点10 d
	{623.60f, -837.96f},    // 索引10	点11 e
	{345.80f, -646.16f},   // 索引11	点12扫二维码2
	{409.58f, -1467.21f},  // 索引12	点13 奖杯3
	{860.0f, -1564.0f},    // 索引13	点12 奖杯2
	{1315.42f, -1442.21f}, // 索引14	点12 奖杯1
	{1710.0f, -270.0f},    // 索引15	点16 亚军奖台
	{1695.0f, 0.0f},       // 索引16	点17 冠军奖台
	{1715.0f, 270.0f},     // 索引17	点18 季军奖台
	{0.0f, 0.0f},          // 索引18	点19 起始点
	{150.00f, -1467.21f},	// 索引19	点20 奖杯3前面过度点
	{1000.f, 0.0f},		// 索引20	点21 回家前过度点
	{156.27f, 682.54f}		// 索引21

};
// X42位置接口使用角度单位，原位置入口参数按3200脉冲/圈换算。
#define ZDT_X42_POS_DEG_PER_CLK (360.0f / 3200.0f)
// 任务状态变量
TaskState_t task1_state = {0};

uint8_t tasknum = 1;
uint8_t cammode = 2;
uint8_t lookmode_centrl = 1;
void Setmaixcam_mode(uint8_t ch)
{
	cammode = ch;
}
void Move_to_turnplate(uint8_t num){
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Material[num-1]);
}
void Find_corresponding_material(uint8_t index)
{
	uint8_t i;
	if(tasknum == 1){
		for(i= 0;i<5;i++){
			if (task1_scheme[qanum-1][index-1] == detected_colors[i]){
//				duoji_doing_set(1,Material[i],2000);
				Move_to_turnplate(i+1);}
		}
	}
	if(tasknum == 2){
		for(i= 0;i<3;i++){
			if (index == task2_scheme[qanum-1][i]){
//				duoji_doing_set(1,Material[i],2000);
				Move_to_turnplate(i+1);}
		}
	}
}

void Pos_moveconctrl(uint8_t dir, uint16_t vel, uint16_t clk){
	float position_deg = (float)clk * ZDT_X42_POS_DEG_PER_CLK;

	if(dir == 1){
		ZDT_X42_V2_Traj_Position_Control(1,1,2,2,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(2,1,2,2,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(3,0,2,2,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(4,0,2,2,(float)vel,position_deg,0,1);
		HAL_Delay(5);
		ZDT_X42_V2_Synchronous_motion(0); 								 // 广播地址0触发

	}
	if(dir == 0){
		ZDT_X42_V2_Traj_Position_Control(1,0,1,1,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(2,0,1,1,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(3,1,1,1,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(4,1,1,1,(float)vel,position_deg,0,1);
		HAL_Delay(5);
		ZDT_X42_V2_Synchronous_motion(0); 								 // 广播地址0触发

	}
}
void POS_move(void){
	Is_vel_mode = 0;//关闭速度模式
	Pos_moveconctrl(1,200,1058);
	HAL_Delay(1800);
	Pos_moveconctrl(0,200,1442);
	HAL_Delay(1800);
	Is_vel_mode = 1;//打开速度模式
}
//void POS_move2(void){
//	Is_vel_mode = 0;//关闭速度模式
//	Pos_moveconctrl(1,200,457);
//	HAL_Delay(2000);
//	Pos_moveconctrl(0,200,457);
//	HAL_Delay(2000);
//	Is_vel_mode = 1;//打开速度模式
//}
//void POS_move3(void){
//	Is_vel_mode = 0;//关闭速度模式
//	Pos_moveconctrl(1,200,457);
//	HAL_Delay(2000);
//	Pos_moveconctrl(0,200,457);
//	HAL_Delay(2000);
//	Is_vel_mode = 1;//打开速度模式
//}
	//********************************位置控制pid****************************************//
// 全局位置控制实例
static PositionController_t position_control;
uint8_t is_position_control_enabled = 0;

// 位置控制初始化
void PositionControl_Init(void) {
    // 初始化X轴位置PID - 参数需要根据实际调试
    PID_Init(&position_control.pid_x, 6.0f, 0.00f, 0.005f, HIGHSPEED); // Kp, Ki, Kd, max_output

    // 初始化Y轴位置PID
    PID_Init(&position_control.pid_y, 6.0f, 0.00f, 0.005f, HIGHSPEED);

    position_control.is_enabled = 0;
    position_control.target_x = 0.0f;
    position_control.target_y = 0.0f;
    position_control.last_update_time = HAL_GetTick();

    is_position_control_enabled = 0;
}

void pospid_reinit(float remax_output){
	position_control.pid_x.max_output = remax_output;
	position_control.pid_y.max_output = remax_output;
}
// 启用/禁用位置控制
void PositionControl_Enable(uint8_t enable) {
    position_control.is_enabled = enable;
    is_position_control_enabled = enable;

    if (!enable) {
        // 禁用时重置PID状态
        position_control.pid_x.integral = 0.0f;
        position_control.pid_x.prev_error = 0.0f;
        position_control.pid_y.integral = 0.0f;
        position_control.pid_y.prev_error = 0.0f;

        // 停止小车
        stop();
    } else {
        // 启用时记录当前时间
        position_control.last_update_time = HAL_GetTick();
    }
}

// 设置目标位置（只控制x,y位置，角度单独控制）
void SetTargetPosition(float x, float y) {
    Is_vel_mode = 1;
	position_control.target_x = x;
    position_control.target_y = y;

    // 重置PID积分项，避免突变带来的积分饱和
    position_control.pid_x.integral = 0.0f;
    position_control.pid_x.prev_error = 0.0f;
    position_control.pid_y.integral = 0.0f;
    position_control.pid_y.prev_error = 0.0f;
}

// 检查是否到达目标位置（只检查x,y位置）
uint8_t IsPositionReached(float tolerance_xy) {
    if (!position_control.is_enabled) {
        return 0;
    }

    // 单位转换
    float current_x_m = robot_pose_state.pos_x_mm * 0.001f;
    float current_y_m = robot_pose_state.pos_y_mm * 0.001f;

    // 计算位置误差（米）
    float error_x = position_control.target_x * 0.001f - current_x_m;
    float error_y = position_control.target_y * 0.001f - current_y_m;

    // 计算距离误差（米）
    float distance_error = sqrtf(error_x * error_x + error_y * error_y);

    // 检查是否在容差范围内
    if (distance_error <= tolerance_xy) {
        return 1;
    }

    return 0;
}
uint8_t testflag = 0;
// 更新位置控制（只控制x,y位置）
void UpdatePositionControl(float dt) {
    if (!position_control.is_enabled || dt <= 0) {
        return;
    }

    // 获取当前偏航角（弧度）
    float current_yaw_rad = GetYaw() * 3.1415926f / 180.0f;

    // 获取当前位置（全局坐标系）
    float current_x_global = robot_pose_state.pos_x_mm * 0.001f;
    float current_y_global = robot_pose_state.pos_y_mm * 0.001f;
    float target_x_global = position_control.target_x * 0.001f;
    float target_y_global = position_control.target_y * 0.001f;

    // 计算全局坐标系下的误差
    float error_x_global = target_x_global - current_x_global;
    float error_y_global = target_y_global - current_y_global;

    // 将误差从全局坐标系转换到小车局部坐标系
    float error_x_local = error_x_global * cosf(current_yaw_rad) + error_y_global * sinf(current_yaw_rad);
    float error_y_local = -error_x_global * sinf(current_yaw_rad) + error_y_global * cosf(current_yaw_rad);

    // 使用局部坐标系的误差进行PID控制
	float vx_local = PID_Update(&position_control.pid_x, error_x_local, dt);
	float vy_local = PID_Update(&position_control.pid_y, error_y_local, dt);

    // 限制最大速度
    #define MAX_LINEAR_SPEED 7.0f
    float linear_speed = sqrtf(vx_local * vx_local + vy_local * vy_local);
    if (linear_speed > MAX_LINEAR_SPEED) {
        float ratio = MAX_LINEAR_SPEED / linear_speed;
        vx_local *= ratio;
        vy_local *= ratio;
    }

    // 设置目标速度（使用局部坐标系的速度）
    SetTargetLinearVelocity(vy_local, vx_local);  // 位置环X轴方向与底盘横向速度方向一致

    // 检查是否到达目标位置（使用全局坐标系）
    if (IsPositionReached(raceh)) {
        stop();
        testflag = 1;
        PositionControl_Enable(0);
    }
}

void Example_PositionControl(void) {
    // 设置目标位置：x=1.0m, y=0.5m
    SetTargetPosition(1.0f, 0.5f);

    // 启用位置控制
    PositionControl_Enable(1);

    // 等待到达目标位置
    while (!IsPositionReached(0.02f)) {
        HAL_Delay(10);
    }

    // 到达后执行其他任务...
}

//********************************位置控制pid****************************************//
//********************************角度pid****************************************//

// 姿态稳定控制结构体
typedef struct {
    PID_Controller yaw_pid;      // 偏航角PID控制器
    float target_yaw;             // 目标偏航角(度)
    uint8_t is_enabled;           // 控制使能标志
    uint32_t last_update_time;    // 最后更新时间
} AttitudeStabilization_t;

// 全局姿态控制实例
AttitudeStabilization_t attitude_control;
float attitude_correction = 0.0f; // 姿态校正量
uint8_t isPosture_correction = 0;
// 获取姿态校正量
float GetAttitudeCorrection(void) {
    return attitude_correction;
}
// 姿态稳定初始化
void AttitudeStabilization_Init(void) {
    // 初始化偏航角PID控制器
    // 参数需要根据实际机器人动态调整
    PID_Init(&attitude_control.yaw_pid, 0.24f, 0.0f, 0.0001f, 10.0f); // Kp, Ki, Kd, max_output

    isPosture_correction=0;//初始不开启姿态矫正，需要用时置1

    attitude_control.target_yaw = 0.0f;
    attitude_control.is_enabled = 0;
    attitude_control.last_update_time = HAL_GetTick();
}

// 设置目标偏航角
void SetTargetYaw(float target) {
     // 确保目标角度在[0, 360)范围内
    attitude_control.target_yaw = target;

    // 重置PID积分项，避免突变
    attitude_control.yaw_pid.integral = 0.0f;
    attitude_control.yaw_pid.prev_error = 0.0f;
}
// 检查是否到达目标角度
uint8_t IsYawReached(float target_yaw, float tolerance_degrees) {
    float current_yaw = GetYaw();
    float error = target_yaw - current_yaw;

    // 将误差归一化到[-180, 180)范围
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // 检查是否在容差范围内
    return (fabsf(error) <= tolerance_degrees);
}
// 启用/禁用姿态稳定
void EnableAttitudeStabilization(uint8_t enable) {
    attitude_control.is_enabled = enable;

    if (enable) {
        // 启用时使用当前角度作为目标
        attitude_control.target_yaw = GetYaw();//GetYaw();
        attitude_control.last_update_time = HAL_GetTick();

        // 重置PID
        attitude_control.yaw_pid.integral = 0.0f;
        attitude_control.yaw_pid.prev_error = 0.0f;
    }
}


// 更新姿态稳定控制
void UpdateAttitudeStabilization(float dt) {
    if (!attitude_control.is_enabled || dt <= 0) {
        attitude_correction = 0.0f; // 禁用时校正量为0
        return;
    }

    // 获取当前偏航角
    float current_yaw = GetYaw();

    // 计算角度误差（考虑角度环绕）
    float error = attitude_control.target_yaw - current_yaw;

    // 将误差归一化到[-180, 180)范围
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // 死区处理
    if (abs_float(error) < DEAD_ZONE) {
        error = 0.0f;
        attitude_control.yaw_pid.integral = 0.0f; // 重置积分项防止积分饱和
		attitude_correction = 0.0f;
        return;
    }

    // 使用PID计算旋转速度控制量
    attitude_correction = PID_Update_angle(&attitude_control.yaw_pid, error, dt);

//	if (IsYawReached(attitude_control.target_yaw,0.5f)) { // 检查是否到达目标角度
//        attitude_correction = 0.0f;
//    }
}
uint8_t is_rotating = 0;
// 旋转到指定角度（非阻塞）
uint8_t RotateToYaw(float target_yaw, float tolerance) {


    if (!is_rotating) {
        // 开始旋转
        SetTargetYaw(target_yaw);
        is_rotating = 1;
        return 0;
    }

    // 检查是否到达目标角度
    if (IsYawReached(target_yaw, tolerance)) {
//        EnableAttitudeStabilization(0);
        is_rotating = 0;
        return 1; // 旋转完成
    }

    return 0; // 旋转中
}
//*******************************角度pid***************************************//
//********************************摄像头校正pid************************************//
// 全局摄像头校正变量
CamStabilization_t cam_stabilization = {0};
float cam_correction_x = 0.0f;
float cam_correction_y = 0.0f;
uint8_t is_cam_stabilizing = 0;
int Is_coordgot =0;
// 初始化摄像头PID
void CamStabilization_Init(void) {
    // 初始化X轴PID: kp=0.8, ki=0.05, kd=0.2, 输出限幅±100
    PID_Init_cam(&cam_stabilization.pid_x, 0.26f,0.0000f, 0.0f, 0.002f, 20.0f);

    // 初始化Y轴PID: kp=0.7, ki=0.03, kd=0.15, 输出限幅±100,
    PID_Init_cam(&cam_stabilization.pid_y, 0.26f,0.0000f,0.0f, 0.002f, 20.0f);

    cam_stabilization.is_enabled = 0;
    cam_correction_x = 0.0f;
    cam_correction_y = 0.0f;
	lookmode_centrl = 0;
	is_cam_stabilizing = 0;
	Is_coordgot =0;
}

// 启用/禁用摄像头校正
void CamStabilization_Enable(uint8_t enable) {
    cam_stabilization.is_enabled = enable;

    IsLight = 1;
    if (!enable) {
        // 禁用时重置状态
        cam_stabilization.pid_x.integral = 0.0f;
        cam_stabilization.pid_x.prev_error = 0.0f;
        cam_stabilization.pid_y.integral = 0.0f;
        cam_stabilization.pid_y.prev_error = 0.0f;
        cam_correction_x = 0.0f;
        cam_correction_y = 0.0f;
		IsLight = 0;
    }
}

// 更新摄像头校正
void UpdateCamStabilization(float x_err, float y_err, float dt) {
    if (!cam_stabilization.is_enabled || dt <= 0) {
        cam_correction_x = 0.0f;
        cam_correction_y = 0.0f;
        return;
    }

    // 定义死区阈值（2像素）
    #define CAM_DEAD_ZONE 0.0f

    // X轴死区处理
    float actual_x_err = (abs_float (x_err) > CAM_DEAD_ZONE) ? x_err : 0.0f;

    // Y轴死区处理
    float actual_y_err = (abs_float(y_err) > CAM_DEAD_ZONE) ? y_err : 0.0f;

    // 计算X轴校正
    cam_correction_x = PID_Update_cam(&cam_stabilization.pid_x, actual_x_err, dt);

    // 计算Y轴校正
    cam_correction_y = PID_Update_cam(&cam_stabilization.pid_y, actual_y_err, dt);
}

static ErrorQueue_t error_queue = {0};

// 初始化误差队列
void ErrorQueue_Init(void) {
    error_queue.index = 0;
    error_queue.count = 0;
    error_queue.is_full = 0;
    memset(error_queue.errors_x, 0, sizeof(error_queue.errors_x));
    memset(error_queue.errors_y, 0, sizeof(error_queue.errors_y));
}

// 向误差队列添加数据
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

// 计算误差队列的均值
void ErrorQueue_GetAverage(float *avg_x, float *avg_y) {
    int sum_x = 0, sum_y = 0;
    uint8_t valid_count = error_queue.is_full ? ERROR_QUEUE_SIZE : error_queue.count;

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
#define INT_MAX 480       //最大偏差绝对值
// 计算误差队列中x/y方向的最大波动幅度（max - min）
void ErrorQueue_GetFluctuate(int *fluct_x, int *fluct_y) {
    int max_x = -INT_MAX, min_x = INT_MAX;
    int max_y = -INT_MAX, min_y = INT_MAX;
    uint8_t valid_count = error_queue.is_full ? ERROR_QUEUE_SIZE : error_queue.count;

    // 若队列空，波动幅度为0
    if (valid_count == 0) {
        *fluct_x = 0;
        *fluct_y = 0;
        return;
    }

    // 遍历有效数据，找max和min
    for (int i = 0; i < valid_count; i++) {
        // x方向
        if (error_queue.errors_x[i] > max_x) max_x = error_queue.errors_x[i];
        if (error_queue.errors_x[i] < min_x) min_x = error_queue.errors_x[i];
        // y方向
        if (error_queue.errors_y[i] > max_y) max_y = error_queue.errors_y[i];
        if (error_queue.errors_y[i] < min_y) min_y = error_queue.errors_y[i];
    }

    // 波动幅度 = 最大值 - 最小值
    *fluct_x = max_x - min_x;
    *fluct_y = max_y - min_y;
}

uint8_t IsCamReached(int tolerance_x, int tolerance_y,int MAX_Fluctuate_error) {
    int error_x = Camera_err.x;
    int error_y = Camera_err.y;

    // 将当前误差加入队列
    ErrorQueue_Push(error_x, error_y);

    // 计算队列中所有误差的均值
    float avg_error_x, avg_error_y;
	  int F_x,F_y;
    ErrorQueue_GetAverage(&avg_error_x, &avg_error_y);
    ErrorQueue_GetFluctuate(&F_x,&F_y);
    // 检查队列是否已满且均值在容差范围内
    if (error_queue.is_full && avg_error_x <= tolerance_x && avg_error_y <= tolerance_y&&F_x<=MAX_Fluctuate_error&&F_y<=MAX_Fluctuate_error) {
        return 1;
    }

    return 0;
}

// 简化的搜索等待函数 - 只在四个方向移动一次，看到目标立即返回
uint8_t Wait_for_uart_with_search(uint8_t point_index) {
    uint32_t start_time = HAL_GetTick();
    Is_coordgot = 0;

    // 第一步：先正常等待一小段时间（比如500ms）
    while (!Is_coordgot) {
        if (HAL_GetTick() - start_time > UART_WAIT_TIMEOUT_MS) {
            break;  // 短暂等待后进入搜索模式
        }
        HAL_Delay(1);
    }

    if (Is_coordgot) {
        return 1;  // 在等待期间就收到了数据
    }

    // 第二步：开始搜索移动（四个方向各移动10mm）
    float base_x = task1_targets[point_index].x;
    float base_y = task1_targets[point_index].y;

    // 定义四个搜索方向：前、后、左、右
    float search_points[4][2] = {
        {base_x + 0.01f, base_y},  // 前
        {base_x - 0.01f, base_y},  // 后
        {base_x, base_y + 0.01f},  // 左
        {base_x, base_y - 0.01f}   // 右
    };

    // 依次移动到四个搜索点
    for (int i = 0; i < 4; i++) {
        // 移动到搜索点
        SetTargetPosition(search_points[i][0], search_points[i][1]);
        PositionControl_Enable(1);

        // 等待移动完成，期间如果收到数据立即返回
        while (!IsPositionReached(0.002f)) {
            if (Is_coordgot) {
                PositionControl_Enable(0);  // 立即停止移动
                HAL_Delay(5);
                return 1;  // 搜索成功，收到数据
            }
            HAL_Delay(10);
        }

        // 检查是否在移动完成后立即收到数据
        if (Is_coordgot) {
            HAL_Delay(5);
            return 1;
        }
    }

    // 四个方向都移动完毕，仍未收到数据
    // 回到原点
    SetTargetPosition(base_x, base_y);
    PositionControl_Enable(1);

    // 等待回到原点
    while (!IsPositionReached(0.002f)) {
        if (Is_coordgot) {
            PositionControl_Enable(0);
            HAL_Delay(5);
            return 1;
        }
        HAL_Delay(10);
    }

    PositionControl_Enable(0);
    return 0;  // 搜索失败
}
extern uint32_t camtim;
extern uint8_t camtim_button;
uint8_t iscamreached_safecount = 0;
// 摄像头校正到中心（非阻塞）
uint8_t CamStabilizationToCenter(int tolerance_x, int tolerance_y,int MAX_Fluctuate_error) {

    if (!is_cam_stabilizing) {
        // 开始校正
        ErrorQueue_Init();				// 开始校正时清空队列
        PositionControl_Enable(0);		//先关闭坐标校正，确保坐标校正和摄像头校正互斥
		CamStabilization_Enable(1);
        is_cam_stabilizing = 1;
		IsLight = 1;
        return 0;
    }

    // 检查是否到达目标
    if (IsCamReached(tolerance_x, tolerance_y,MAX_Fluctuate_error)) {
		iscamreached_safecount ++;
		if(iscamreached_safecount>10){
			CamStabilization_Enable(0);
			is_cam_stabilizing = 0;
			IsLight = 0;
			return 1; // 校正完成
		}
    }

    return 0; // 校正中
}
//********************************摄像头校正pid************************************//
// 全局目标速度变量
TargetVelocity_t target_velocity = {0};

// 定义4个电机的全局变量
Motor_t motor1 = {0, 0};
Motor_t motor2 = {0, 0};
Motor_t motor3 = {0, 0};
Motor_t motor4 = {0, 0};
Motor_t motor5 = {0, 0};
// 底盘参数
static ChassisConfig_t chassis_config;
static float k_omega;  // 旋转系数 k = (Lx + Ly)/2

void SetTargetVelocity(float vx, float vy, float omega) {
    target_velocity.vx = vx;
    target_velocity.vy = vy;
    target_velocity.omega = omega;
}

void SetTargetLinearVelocity(float vx, float vy) {
    target_velocity.vx = vx;
    target_velocity.vy = vy;
}
// 初始化底盘参数
void Mecanum_Init(ChassisConfig_t config) {
    chassis_config = config;
    k_omega = (config.wheel_dist_x + config.wheel_dist_y) / 2.0f;
}

// 运动解算核心函数（不带角度补偿）
void Mecanum_Kinematics(float vx, float vy, float omega)
{
    float actual_omega = omega;

    // 如果需要使用姿态校正
    if (attitude_control.is_enabled) {
        actual_omega += attitude_correction;

        // 限制最大旋转速度
        #define MAX_OMEGA 20.0f
        if (actual_omega > MAX_OMEGA) actual_omega = MAX_OMEGA;
        if (actual_omega < -MAX_OMEGA) actual_omega = -MAX_OMEGA;
    }
	// 如果需要使用摄像头校正
    if (cam_stabilization.is_enabled) {
        vy += cam_correction_x * 0.01f;
		vx += cam_correction_y * 0.01f;
	}

    // 1. 计算各轮的目标线速度（m/s）
    // 轮子顺序：左前(4) 右前(2) 左后(3) 右后(1)
    float v4 =  vx - vy - k_omega * actual_omega;  // 左前 (4)
    float v2 =  vx + vy + k_omega * actual_omega;  // 右前 (2)
    float v3 =  vx + vy - k_omega * actual_omega;  // 左后 (3)
    float v1 =  vx - vy + k_omega * actual_omega;  // 右后 (1)

    // 2. 转换为 RPM（RPM = (线速度 / (2π * 轮半径)) * 60）
    const float linear_to_rpm = 60.0f / (2 * 3.1415926f * chassis_config.wheel_radius);
    motor4.speed_rpm = v4 * linear_to_rpm;
    motor2.speed_rpm = v2 * linear_to_rpm;
    motor3.speed_rpm = v3 * linear_to_rpm;
    motor1.speed_rpm = v1 * linear_to_rpm;

    // 3. 限制转速范围
    #define CONSTRAIN(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
    motor4.speed_rpm = CONSTRAIN(motor4.speed_rpm, -chassis_config.max_rpm, chassis_config.max_rpm);
    motor2.speed_rpm = CONSTRAIN(motor2.speed_rpm, -chassis_config.max_rpm, chassis_config.max_rpm);
    motor3.speed_rpm = CONSTRAIN(motor3.speed_rpm, -chassis_config.max_rpm, chassis_config.max_rpm);
    motor1.speed_rpm = CONSTRAIN(motor1.speed_rpm, -chassis_config.max_rpm, chassis_config.max_rpm);

    // 4. 设置电机方向
    motor4.direction = (motor4.speed_rpm >= 0) ? 0 : 1;  // 左前 (4)
    motor2.direction = (motor2.speed_rpm >= 0) ? 1 : 0;  // 右前 (2)
    motor3.direction = (motor3.speed_rpm >= 0) ? 0 : 1;  // 左后 (3)
    motor1.direction = (motor1.speed_rpm >= 0) ? 1 : 0;  // 右后 (1)
}

// 以下是原有的运动控制函数
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
    SetTargetVelocity(0.0f, - 0.5f, 0.0f);
}

void right(void) {
    SetTargetVelocity(0.0f, 0.5f, 0.0f);
}
void right_back(void){
	SetTargetVelocity(-0.5f, -0.5f, 0.0f);
}

void turn_left(void) {
    SetTargetVelocity(0.0f, 0.0f, 4.5f);
}

void turn_right(void) {
    SetTargetVelocity(0.0f, 0.0f, -4.5f);
}

//********************************循迹pid部分（没用到）************************************//
uint8_t IsRun = 0;
// PID控制器初始化
void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd, float max_output) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->max_output = max_output;
}
void PID_Init_cam(PID_Controller* pid, float Kp,float Kp2, float Ki, float Kd, float max_output) {
    pid->Kp = Kp;
	pid->Kp2 = Kp2;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->max_output = max_output;
}


// PID更新计算
float PID_Update(PID_Controller* pid, float error, float dt) {
    // 比例项
    float proportional = pid->Kp * error;

    // 积分项（带抗饱和）
    pid->integral += error * dt;
    float integral = pid->Ki * pid->integral;

    // 微分项
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // 计算总输出
    float output = proportional + integral + derivative;

    // 限制输出范围
    if (output > pid->max_output) {
        output = pid->max_output;
        pid->integral -= error * dt; // 抗饱和处理
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
        pid->integral -= error * dt; // 抗饱和处理
    }

    return output;
}
float PID_Update_cam(PID_Controller* pid, float error, float dt) {
    // 比例项
    float proportional = pid->Kp * error;
	float proportional2 =pid->Kp2*error*fabsf(error);

    // 积分项（带抗饱和）
    pid->integral += error * dt;
    float integral = pid->Ki * pid->integral;

    // 微分项
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // 计算总输出

    float output = proportional +proportional2 + integral + derivative;

//	if(output>0){output +=0.4f;}
//	else if(output<0){output -=0.4f;}
//	else output =0;

    // 限制输出范围
    if (output > pid->max_output) {
        output = pid->max_output;
        pid->integral -= error * dt; // 抗饱和处理
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
        pid->integral -= error * dt; // 抗饱和处理
    }

    return output;
}
// PID更新计算
float PID_Update_angle(PID_Controller* pid, float error, float dt) {
    // 比例项
    float proportional = pid->Kp * error;

    // 积分项（带抗饱和）
    pid->integral += error * dt;
    float integral = pid->Ki * pid->integral;

    // 微分项
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

	float G = robot_pose_state.yaw_gyro_dps*g;
    // 计算总输出
    float output = proportional + integral + derivative;

    // 限制输出范围
    if (output > pid->max_output) {
        output = pid->max_output;
        pid->integral -= error * dt; // 抗饱和处理
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
        pid->integral -= error * dt; // 抗饱和处理
    }

    return output;
}
// 全局PID控制器实例
static PID_Controller line_follow_pid;
uint8_t is_line_follow_enabled = 0; // 新增：循迹使能标志
// 循迹初始化函数
void Line_Follow_Init(void) {
    // 初始化PID参数，这些参数可能需要根据实际调试调整
    PID_Init(&line_follow_pid, 0.9f, 0.02f, 200.0f, 0.48f);  // Kp, Ki, Kd, max_output
	is_line_follow_enabled = 0; // 初始状态为禁用
	IsRun = 0;
}
// 新增：循迹使能/禁用函数
void Line_Follow_Enable(uint8_t enable) {
    is_line_follow_enabled = enable;

    if (!enable) {
        // 禁用时重置PID状态
        line_follow_pid.integral = 0.0f;
        line_follow_pid.prev_error = 0.0f;
		stop();
    }
}
// 循迹控制函数
void Line_Follow_Control(uint8_t* sensor_values, float base_speed, float dt) {
	// 如果循迹禁用，重置状态并返回
    if (!is_line_follow_enabled) {
        line_follow_pid.integral = 0.0f;
        line_follow_pid.prev_error = 0.0f;

		return;
    }
	// 新的误差计算方式 - 更强调中心传感器
    float error = 0.0f;

    // 中心传感器优先
    if(is_task1_active()){
		if( sensor_values[2] == 0 ||sensor_values[3] == 0) {
			// 当中心传感器检测到黑线时，强制误差归零
			error = 0.0f;
			// 同时重置PID积分项，避免历史误差影响
			line_follow_pid.integral = 0.0f;
			line_follow_pid.prev_error = 0.0f;
		}
		else {
			// 只有当中心没有检测到时，才计算加权误差

			float total_weight = 0.0f;
			const float weights[8] = {-6.f, -3.0f, -1.0f, 1.0f, 1.0f, 2.0f, 4.0f, 6.0f};

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
	else{
			if( sensor_values[2] == 0 || sensor_values[3] == 0) {
				// 当中心传感器检测到黑线时，强制误差归零
				error = 0.0f;
				// 同时重置PID积分项，避免历史误差影响
				line_follow_pid.integral = 0.0f;
			}
			else {
				float total_weight = 0.0f;
				const float weights[8] = {-4.f, -1.0f, -1.0f, 1.0f, 10.0f, 10.0f, 11.0f, 12.0f};

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

    // 更新PID控制器
    float omega = PID_Update(&line_follow_pid, error, dt);

    // 使用运动学控制
    if(task1_state.active){
		if(!IsRun)SetTargetVelocity(0, 0.0f, -omega);
		else SetTargetVelocity(base_speed, 0.0f, -0.68f - omega);}
	else{
		if(!IsRun)SetTargetVelocity(0, 0.0f, -omega);
		else SetTargetVelocity(base_speed, 0.0f, 0.695f - omega);}
}
void Follow_line_task(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f;
    last_time = current_time;

    if (dt <= 0) dt = 0.01f; // 最小时间间隔
    if(is_task1_active()){
		Line_Follow_Control(Anolog, 0.2376f, dt);}
	else{
		Line_Follow_Control(Anolog, 0.229f, dt);
	}
}

//********************************循迹pid部分************************************//

//**************************** 任务调度实现 ****************************//


//==================================================================
//函 数 名：前往目标点
//功    能：设置目标点并使能运动，然后判断是否移动完成
//输入参数：目标x，y坐标
//返 回
uint8_t target_set = 0;
uint8_t move_to_target(float x, float y) {


    if (!is_position_control_enabled && !target_set) {
        // 设置新目标
        SetTargetPosition(x, y);
        PositionControl_Enable(1);
        target_set = 1;
        return 0; // 移动刚开始
    }

    if (!is_position_control_enabled && target_set) {
        // 移动完成
        target_set = 0;
        return 1; // 移动完成
    }

    return 0; // 移动中
}
/***********Task1***************/

// 启动任务1
void start_task1(void) {
    task1_state.active = 1;
    task1_state.current_step = 1;
    task1_state.completed = 0;
    task1_state.step_start = HAL_GetTick();
}

uint8_t is_hasgetcolor = 0;
// 非阻塞任务执行
void run_task1(void) {
    if (!task1_state.active) return;//未使能任务则返回
    uint32_t current_time = HAL_GetTick();
    switch (task1_state.current_step) {
//		case 0:	EnableAttitudeStabilization(1);SetTargetYaw(0.0f);
//				Servo_Action_Group(1);pospid_reinit(LOWSPEED);
//				if (move_to_target(task1_targets[20].x, task1_targets[20].y)/*点1*/) {task1_state.current_step++;}
//				if(IsPositionReached(0.10f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 2060);}
////				if(IsPositionReached(0.05f)){SetTargetPosition(task1_targets[0].x, task1_targets[0].y);task1_state.current_step++;}
//				break;
        case 1:	EnableAttitudeStabilization(1);SetTargetYaw(0.0f);Move_to_turnplate(1);
				Servo_Action_Group(1);
				tasknum = 1;
				pospid_reinit(LOWSPEED);
				if (move_to_target(task1_targets[0].x, task1_targets[0].y)/*点二维码*/) {Servo_Action_Group(2);task1_state.current_step++;}
				if(IsPositionReached(0.50f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 2060);}
				if(IsPositionReached(0.45f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1980);}if(IsPositionReached(0.40f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1930);}
				if(IsPositionReached(0.35f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1900);}if(IsPositionReached(0.30f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1880);}
				if(IsPositionReached(0.25f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1860);}if(IsPositionReached(0.20f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1840);}
				if(IsPositionReached(0.15f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1820);}if(IsPositionReached(0.10f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1810);}
				if(IsPositionReached(0.04f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1800);}
				break;
        case 2:Servo_Action_Group(2);PositionControl_Enable(0);EnableAttitudeStabilization(0);
				SetTargetVelocity(1.0f, 0.0f, -2.47f);
				HAL_Delay(500);
				Move_to_turnplate(2);
				HAL_Delay(1000);
				Get_Materialcolor(0);
				Move_to_turnplate(3);
				HAL_Delay(450);
				SetTargetVelocity(1.0f, 0.0f, -2.08f);
				HAL_Delay(500);
				Get_Materialcolor(1);
				Move_to_turnplate(4);
				HAL_Delay(500);
				SetTargetVelocity(1.0f, 0.0f, -2.14f);
				HAL_Delay(550);
				Get_Materialcolor(2);
				Move_to_turnplate(5);
				HAL_Delay(1000);Get_Materialcolor(3);__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Material[3]+200);
				stop();
				task1_state.current_step =11;
				break;


		case 11:EnableAttitudeStabilization(1);
				pospid_reinit(MIDSPEED);SetTargetYaw(-90.0f);Get_Materialcolor(4);
				if (move_to_target(task1_targets[6].x, task1_targets[6].y)/*移到点a*/) {
										Find_corresponding_material(1);
										task1_state.current_step++;}break;
		case 12:if(Wait_for_uart_with_search(6)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 超时处理，比如重试或错误处理 */ } break;
		case 13:if(CamStabilizationToCenter(TOLERANCE,TOLERANCE,max_min_error)){POS_move();task1_state.current_step++;}break;
		case 14:pospid_reinit(HIGHSPEED);if (move_to_target(task1_targets[7].x, task1_targets[7].y)/*移到点b*/) {
					Find_corresponding_material(2);
					task1_state.current_step++;}break;
		case 15:if(Wait_for_uart_with_search(7)) { task1_state.current_step++; }
				else {task1_state.current_step++;/* 超时处理，比如重试或错误处理 */ } break;
		case 16:if(CamStabilizationToCenter(TOLERANCE,TOLERANCE,max_min_error)){
						POS_move();task1_state.current_step++;}break;
		case 17:if (move_to_target(task1_targets[8].x, task1_targets[8].y)) {
					Find_corresponding_material(3);
					task1_state.current_step++;}break;
		case 18:if(Wait_for_uart_with_search(8)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 超时处理，比如重试或错误处理 */ } break;
		case 19:if(CamStabilizationToCenter(TOLERANCE,TOLERANCE,max_min_error)){
						POS_move();task1_state.current_step++;}break;
		case 20:if (move_to_target(task1_targets[9].x, task1_targets[9].y)) {
					Find_corresponding_material(4);
					task1_state.current_step++;}break;
		case 21:if(Wait_for_uart_with_search(9)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 超时处理，比如重试或错误处理 */ } break;
		case 22:if(CamStabilizationToCenter(TOLERANCE,TOLERANCE,max_min_error)){
						POS_move();task1_state.current_step++;}break;
		case 23:if (move_to_target(task1_targets[10].x, task1_targets[10].y)) {
					Find_corresponding_material(5);
					task1_state.current_step++;}break;
		case 24:if(Wait_for_uart_with_search(10)) { task1_state.current_step++; }
				else { task1_state.current_step++;/* 超时处理，比如重试或错误处理 */ } break;
		case 25:if(CamStabilizationToCenter(TOLERANCE,TOLERANCE,max_min_error)){
						POS_move();task1_state.current_step++;}break;
		case 26://Servo_Action_Group(2);EnableAttitudeStabilization(1);
				tasknum = 2;SetTargetYaw(0.0f);if (move_to_target(task1_targets[11].x, task1_targets[11].y)/*点二维码2*/) {Move_to_turnplate(3);/*移到转盘1*/task1_state.current_step++;}break;
		case 27:SetTargetYaw(-90.0f);if (move_to_target(task1_targets[19].x, task1_targets[19].y)/*奖杯3下面*/){task1_state.current_step++;}break;
		case 28:pospid_reinit(LOWESTSPEED);SetTargetYaw(-90.0f);if (move_to_target(task1_targets[12].x, task1_targets[12].y)/*奖杯3*/){task1_state.current_step++;Move_to_turnplate(2);}break;
		case 29:if (move_to_target(task1_targets[13].x, task1_targets[13].y)/*奖杯2*/) {task1_state.current_step++;Move_to_turnplate(1);}break;
		case 30:if (move_to_target(task1_targets[14].x, task1_targets[14].y)/*奖杯1*/) {__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Material[1]+200);SetTargetYaw(-90.0f);task1_state.current_step++;}	break;
		case 31:
				pospid_reinit(HIGHSPEED);
				if (move_to_target(task1_targets[15].x, task1_targets[15].y+15.0f)/*亚军台*/) {Servo_Action_Group(6);Find_corresponding_material(2);task1_state.current_step++;break;}
				if(IsPositionReached(0.6f)){Servo_Action_Group(5);}break;
		case 32:if(Wait_for_uart_with_search(15)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 超时处理，比如重试或错误处理 */ } break;
		case 33:if(CamStabilizationToCenter(TOLERANCE,TOLERANCE,max_min_error)){
						POS_move();Servo_Action_Group(3);HAL_Delay(100);
						task1_state.current_step++;}break;
		case 34:if (move_to_target(task1_targets[16].x, task1_targets[16].y+15.0f)/*冠军台*/) {Servo_Action_Group(4);Find_corresponding_material(1);task1_state.current_step++;}break;
		case 35:if(Wait_for_uart_with_search(16)) { task1_state.current_step++; }
				else { task1_state.current_step++;/* 超时处理，比如重试或错误处理 */ } break;
		case 36:if(CamStabilizationToCenter(TOLERANCE,TOLERANCE,max_min_error)){
						POS_move();Servo_Action_Group(3);
						task1_state.current_step++;}break;
		case 37:if (move_to_target(task1_targets[17].x, task1_targets[17].y+25.0f)) {Servo_Action_Group(7);Find_corresponding_material(3);task1_state.current_step++;}break;
		case 38:if(Wait_for_uart_with_search(17)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 超时处理，比如重试或错误处理 */ } break;
		case 39:if(CamStabilizationToCenter(TOLERANCE,TOLERANCE,max_min_error)){
						POS_move();task1_state.current_step++;}break;
		case 40:pospid_reinit(HIGHESTSPEED);raceh = 0.200f;
				if (move_to_target(task1_targets[20].x, task1_targets[20].y)/*回家*/) {task1_state.current_step = 41;}break;
		case 41:raceh = 0.002f;if (move_to_target(task1_targets[18].x, task1_targets[18].y)/*回家*/) {task1_state.current_step++;}break;
        default:
            task1_state.active = 0;
            task1_state.completed = 1;
            break;
    }
}
//非阻塞写法
//case 6:SetTargetYaw(-90.0f);if (move_to_target(task1_targets[6].x, task1_targets[6].y)/*点a*/) {
//										Pos_moveconctrl(1,100,6400);task1_state.step_start = current_time; task1_state.current_step++;}break;
//		case 7:if (current_time - task1_state.step_start >= 1000) {//延时一秒
//					Pos_moveconctrl(0,100,6400);task1_state.step_start = current_time;task1_state.current_step++;
//					}break;
//		case 8:if (current_time - task1_state.step_start >= 1000) {//延时一秒
//					if (move_to_target(task1_targets[7].x, task1_targets[7].y)/*点b*/) {task1_state.current_step++;}
//					}break;
// 检查任务是否激活
uint8_t is_task1_active(void) {
    return task1_state.active;
}
/***********Task1***************/
