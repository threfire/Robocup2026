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


float raceh = 0.002f;  // 浣嶇疆鐜粯璁ゅ宸紝鍗曚綅锛歮锛?mm
uint16_t Material[5] = {575,970,1365,1785,2165};//杞洏鍧愭爣
// 浠诲姟1鐨勭洰鏍囩偣鍧愭爣鏁扮粍
static const Point2D_t task1_targets[] = {
    {250.25f, 650.0f},     // 绱㈠紩0		鐐?鎵簩缁寸爜1
    {258.25f, 1058.24f},   // 绱㈠紩1		鐐?鐗╁潡1
    {604.89f, 1399.38f},   // 绱㈠紩2		鐐?鐗╁潡2
    {1075.0f, 1524.0f},    // 绱㈠紩3		鐐?鐗╁潡3
    {1545.10f, 1399.38f},  // 绱㈠紩4		鐐?鐗╁潡4
    {1891.74f, 1058.24f},  // 绱㈠紩5		鐐?鐗╁潡5
	{835.14f, 631.60f},     // 绱㈠紩6		鐐?  a
	{553.90f, 425.91f},	   // 绱㈠紩7		鐐?  b
	{836.02f, -220.80f},   // 绱㈠紩8		鐐?  c
	{554.98f, -334.63f},   // 绱㈠紩9		鐐?0 d
	{623.60f, -837.96f},    // 绱㈠紩10	鐐?1 e
	{345.80f, -646.16f},   // 绱㈠紩11	鐐?2鎵簩缁寸爜2
	{409.58f, -1467.21f},  // 绱㈠紩12	鐐?3 濂栨澂3
	{860.0f, -1564.0f},    // 绱㈠紩13	鐐?2 濂栨澂2
	{1315.42f, -1442.21f}, // 绱㈠紩14	鐐?2 濂栨澂1
	{1710.0f, -270.0f},    // 绱㈠紩15	鐐?6 浜氬啗濂栧彴
	{1695.0f, 0.0f},       // 绱㈠紩16	鐐?7 鍐犲啗濂栧彴
	{1715.0f, 270.0f},     // 绱㈠紩17	鐐?8 瀛ｅ啗濂栧彴
	{0.0f, 0.0f},          // 绱㈠紩18	鐐?9 璧峰鐐?
	{150.00f, -1467.21f},	// 绱㈠紩19	鐐?0 濂栨澂3鍓嶉潰杩囧害鐐?
	{1000.f, 0.0f},		// 绱㈠紩20	鐐?1 鍥炲鍓嶈繃搴︾偣
	{156.27f, 682.54f}		// 绱㈠紩21

};
// X42浣嶇疆鎺ュ彛浣跨敤瑙掑害鍗曚綅锛屽師浣嶇疆鍏ュ彛鍙傛暟鎸?200鑴夊啿/鍦堟崲绠椼€?
#define ZDT_X42_POS_DEG_PER_CLK (360.0f / 3200.0f)
#define TASK1_RELATIVE_X_MOVE_MM 60.0f
// 浠诲姟鐘舵€佸彉閲?
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
#define acc 10
#define dec 10
void Pos_moveconctrl(uint8_t dir, uint16_t vel, uint16_t clk){
	float position_deg = (float)clk * ZDT_X42_POS_DEG_PER_CLK;

	if(dir == 1){
		ZDT_X42_V2_Traj_Position_Control(1,1,acc,dec,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(2,1,acc,dec,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(3,0,acc,dec,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(4,0,acc,dec,(float)vel,position_deg,0,1);
		HAL_Delay(5);
		ZDT_X42_V2_Synchronous_motion(0); 								 // 骞挎挱鍦板潃0瑙﹀彂

	}
	if(dir == 0){
		ZDT_X42_V2_Traj_Position_Control(1,0,acc,dec,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(2,0,acc,dec,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(3,1,acc,dec,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(4,1,acc,dec,(float)vel,position_deg,0,1);
		HAL_Delay(5);
		ZDT_X42_V2_Synchronous_motion(0); 								 // 骞挎挱鍦板潃0瑙﹀彂

	}
}
void POS_move(void){
	Is_vel_mode = 0;//鍏抽棴閫熷害妯″紡
	Pos_moveconctrl(1,50,1058);
	HAL_Delay(1800);
	Pos_moveconctrl(0,50,1442);
	HAL_Delay(1800);
	Is_vel_mode = 1;//鎵撳紑閫熷害妯″紡
}
//void POS_move2(void){
//	Is_vel_mode = 0;//鍏抽棴閫熷害妯″紡
//	Pos_moveconctrl(1,200,457);
//	HAL_Delay(2000);
//	Pos_moveconctrl(0,200,457);
//	HAL_Delay(2000);
//	Is_vel_mode = 1;//鎵撳紑閫熷害妯″紡
//}
//void POS_move3(void){
//	Is_vel_mode = 0;//鍏抽棴閫熷害妯″紡
//	Pos_moveconctrl(1,200,457);
//	HAL_Delay(2000);
//	Pos_moveconctrl(0,200,457);
//	HAL_Delay(2000);
//	Is_vel_mode = 1;//鎵撳紑閫熷害妯″紡
//}
	//********************************浣嶇疆鎺у埗pid****************************************//
// 鍏ㄥ眬浣嶇疆鎺у埗瀹炰緥
PositionController_t position_control;
uint8_t is_position_control_enabled = 0;
PoseCoordination_t pose_coordination = {0};
static float position_lpf_vx_local = 0.0f;
static float position_lpf_vy_local = 0.0f;

static float NormalizeYawError(float error)
{
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

// 浣嶇疆鎺у埗鍒濆鍖?
void PositionControl_Init(void) {
    // 鍒濆鍖朮杞翠綅缃甈ID - 鍙傛暟闇€瑕佹牴鎹疄闄呰皟璇?
    PID_Init(&position_control.pid_x, POSITION_PID_DEFAULT_KP, 0.00f, 0.000f, HIGHSPEED); // Kp, Ki, Kd, max_output

    // 鍒濆鍖朰杞翠綅缃甈ID
    PID_Init(&position_control.pid_y, POSITION_PID_DEFAULT_KP, 0.00f, 0.000f, HIGHSPEED);

    position_control.is_enabled = 0;
    position_control.target_x = 0.0f;
    position_control.target_y = 0.0f;
    position_control.last_update_time = HAL_GetTick();

    is_position_control_enabled = 0;
}

void pospid_reinit(float remax_output, float re_kp){
	position_control.pid_x.max_output = remax_output;
	position_control.pid_y.max_output = remax_output;
	position_control.pid_x.Kp = re_kp;
	position_control.pid_y.Kp = re_kp;
}
// 鍚敤/绂佺敤浣嶇疆鎺у埗
void PositionControl_Enable(uint8_t enable) {
    position_control.is_enabled = enable;
    is_position_control_enabled = enable;

    if (!enable) {
        // 绂佺敤鏃堕噸缃甈ID鐘舵€?
        position_control.pid_x.integral = 0.0f;
        position_control.pid_x.prev_error = 0.0f;
        position_control.pid_y.integral = 0.0f;
        position_control.pid_y.prev_error = 0.0f;
        position_lpf_vx_local = 0.0f;
        position_lpf_vy_local = 0.0f;

        // 鍋滄灏忚溅
        stop();
    } else {
        // 鍚敤鏃惰褰曞綋鍓嶆椂闂?
        position_control.last_update_time = HAL_GetTick();
    }
}

// 璁剧疆鐩爣浣嶇疆锛堝彧鎺у埗x,y浣嶇疆锛岃搴﹀崟鐙帶鍒讹級
void SetTargetPosition(float x, float y) {
    Is_vel_mode = 1;
	position_control.target_x = x;
    position_control.target_y = y;
    target_velocity.omega = 0.0f;

    // 閲嶇疆PID绉垎椤癸紝閬垮厤绐佸彉甯︽潵鐨勭Н鍒嗛ケ鍜?
    position_control.pid_x.integral = 0.0f;
    position_control.pid_x.prev_error = 0.0f;
    position_control.pid_y.integral = 0.0f;
    position_control.pid_y.prev_error = 0.0f;
    position_lpf_vx_local = 0.0f;
    position_lpf_vy_local = 0.0f;
}

// 妫€鏌ユ槸鍚﹀埌杈剧洰鏍囦綅缃紙鍙鏌,y浣嶇疆锛?
uint8_t IsPositionReached(float tolerance_xy) {
    // 鍗曚綅杞崲
    float current_x_m = robot_pose_state.pos_x_mm * 0.001f;
    float current_y_m = robot_pose_state.pos_y_mm * 0.001f;

    // 璁＄畻浣嶇疆璇樊锛堢背锛?
    float error_x = position_control.target_x * 0.001f - current_x_m;
    float error_y = position_control.target_y * 0.001f - current_y_m;

    // 璁＄畻璺濈璇樊锛堢背锛?
    float distance_error = sqrtf(error_x * error_x + error_y * error_y);

    // 妫€鏌ユ槸鍚﹀湪瀹瑰樊鑼冨洿鍐?
    if (distance_error <= tolerance_xy) {
        return 1;
    }

    return 0;
}
uint8_t testflag = 0;
// 鏇存柊浣嶇疆鎺у埗锛堝彧鎺у埗x,y浣嶇疆锛?
void UpdatePositionControl(float dt) {
    if (!position_control.is_enabled || dt <= 0) {
        return;
    }

    // 鑾峰彇褰撳墠鍋忚埅瑙掞紙寮у害锛?
    float current_yaw_rad = GetYaw() * 3.1415926f / 180.0f;

    // 鑾峰彇褰撳墠浣嶇疆锛堝叏灞€鍧愭爣绯伙級
    float current_x_global = robot_pose_state.pos_x_mm * 0.001f;
    float current_y_global = robot_pose_state.pos_y_mm * 0.001f;
    float target_x_global = position_control.target_x * 0.001f;
    float target_y_global = position_control.target_y * 0.001f;

    // 璁＄畻鍏ㄥ眬鍧愭爣绯讳笅鐨勮宸?
    float error_x_global = target_x_global - current_x_global;
    float error_y_global = target_y_global - current_y_global;

    // 灏嗚宸粠鍏ㄥ眬鍧愭爣绯昏浆鎹㈠埌灏忚溅灞€閮ㄥ潗鏍囩郴
    float error_x_local = error_x_global * cosf(current_yaw_rad) + error_y_global * sinf(current_yaw_rad);
    float error_y_local = -error_x_global * sinf(current_yaw_rad) + error_y_global * cosf(current_yaw_rad);

    // 浣跨敤灞€閮ㄥ潗鏍囩郴鐨勮宸繘琛孭ID鎺у埗
    float vx_local = PID_Update(&position_control.pid_x, error_x_local, dt);
    float vy_local = PID_Update(&position_control.pid_y, error_y_local, dt);
    // 位置环通用速度低通滤波
    position_lpf_vx_local += POSITION_SPEED_LPF_ALPHA * (vx_local - position_lpf_vx_local);
    position_lpf_vy_local += POSITION_SPEED_LPF_ALPHA * (vy_local - position_lpf_vy_local);
    vx_local = position_lpf_vx_local;
    vy_local = position_lpf_vy_local;

    // 闄愬埗鏈€澶ч€熷害
    #define MAX_LINEAR_SPEED 7.0f
    float linear_speed = sqrtf(vx_local * vx_local + vy_local * vy_local);
    if (linear_speed > MAX_LINEAR_SPEED) {
        float ratio = MAX_LINEAR_SPEED / linear_speed;
        vx_local *= ratio;
        vy_local *= ratio;
    }

    // 璁剧疆鐩爣閫熷害锛堜娇鐢ㄥ眬閮ㄥ潗鏍囩郴鐨勯€熷害锛?
    vx_local *= pose_coordination.sync_scale;
    vy_local *= pose_coordination.sync_scale;
    SetTargetLinearVelocity(vy_local, vx_local);  // 浣嶇疆鐜疿杞存柟鍚戜笌搴曠洏妯悜閫熷害鏂瑰悜涓€鑷?

    // 妫€鏌ユ槸鍚﹀埌杈剧洰鏍囦綅缃紙浣跨敤鍏ㄥ眬鍧愭爣绯伙級
    if (IsPositionReached(raceh)) {
        stop();
        testflag = 1;
        PositionControl_Enable(0);
    }
}

void Example_PositionControl(void) {
    // 璁剧疆鐩爣浣嶇疆锛歺=1.0m, y=0.5m
    SetTargetPosition(1.0f, 0.5f);

    // 鍚敤浣嶇疆鎺у埗
    PositionControl_Enable(1);

    // 绛夊緟鍒拌揪鐩爣浣嶇疆
    while (!IsPositionReached(0.02f)) {
        HAL_Delay(10);
    }

    // 鍒拌揪鍚庢墽琛屽叾浠栦换鍔?..
}

//********************************浣嶇疆鎺у埗pid****************************************//
//********************************瑙掑害pid****************************************//

// 濮挎€佺ǔ瀹氭帶鍒剁粨鏋勪綋
typedef struct {
    PID_Controller yaw_pid;      // 鍋忚埅瑙扨ID鎺у埗鍣?
    float target_yaw;             // 鐩爣鍋忚埅瑙?搴?
    uint8_t is_enabled;           // 鎺у埗浣胯兘鏍囧織
    uint32_t last_update_time;    // 鏈€鍚庢洿鏂版椂闂?
} AttitudeStabilization_t;

// 鍏ㄥ眬濮挎€佹帶鍒跺疄渚?
AttitudeStabilization_t attitude_control;
float attitude_correction = 0.0f; // 濮挎€佹牎姝ｉ噺
uint8_t isPosture_correction = 0;
// 鑾峰彇濮挎€佹牎姝ｉ噺
float GetAttitudeCorrection(void) {
    return attitude_correction;
}
// 濮挎€佺ǔ瀹氬垵濮嬪寲
void AttitudeStabilization_Init(void) {
    // 鍒濆鍖栧亸鑸PID鎺у埗鍣?
    // 鍙傛暟闇€瑕佹牴鎹疄闄呮満鍣ㄤ汉鍔ㄦ€佽皟鏁?
    PID_Init(&attitude_control.yaw_pid, 0.24f, 0.0f, 0.0001f, 10.0f); // Kp, Ki, Kd, max_output

    isPosture_correction=0;//鍒濆涓嶅紑鍚Э鎬佺煫姝ｏ紝闇€瑕佺敤鏃剁疆1

    attitude_control.target_yaw = 0.0f;
    attitude_control.is_enabled = 0;
    attitude_control.last_update_time = HAL_GetTick();
}

// 璁剧疆鐩爣鍋忚埅瑙?
void SetTargetYaw(float target) {
    if (fabsf(NormalizeYawError(attitude_control.target_yaw - target)) < 0.001f) {
        return;
    }
     // 纭繚鐩爣瑙掑害鍦╗0, 360)鑼冨洿鍐?
    attitude_control.target_yaw = target;

    // 閲嶇疆PID绉垎椤癸紝閬垮厤绐佸彉
    attitude_control.yaw_pid.integral = 0.0f;
    attitude_control.yaw_pid.prev_error = 0.0f;
}
// 妫€鏌ユ槸鍚﹀埌杈剧洰鏍囪搴?
uint8_t IsYawReached(float target_yaw, float tolerance_degrees) {
    float current_yaw = GetYaw();
    float error = target_yaw - current_yaw;

    // 灏嗚宸綊涓€鍖栧埌[-180, 180)鑼冨洿
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // 妫€鏌ユ槸鍚﹀湪瀹瑰樊鑼冨洿鍐?
    return (fabsf(error) <= tolerance_degrees);
}
// 鍚敤/绂佺敤濮挎€佺ǔ瀹?
void EnableAttitudeStabilization(uint8_t enable) {
    if (attitude_control.is_enabled == enable) {
        return;
    }
    attitude_control.is_enabled = enable;

    if (enable) {
        // 鍚敤鏃朵娇鐢ㄥ綋鍓嶈搴︿綔涓虹洰鏍?
        attitude_control.target_yaw = GetYaw();//GetYaw();
        attitude_control.last_update_time = HAL_GetTick();

        // 閲嶇疆PID
        attitude_control.yaw_pid.integral = 0.0f;
        attitude_control.yaw_pid.prev_error = 0.0f;
    }
}


// 鏇存柊濮挎€佺ǔ瀹氭帶鍒?
void UpdateAttitudeStabilization(float dt) {
    if (!attitude_control.is_enabled || dt <= 0) {
        attitude_correction = 0.0f; // 绂佺敤鏃舵牎姝ｉ噺涓?
        return;
    }

    // 鑾峰彇褰撳墠鍋忚埅瑙?
    float current_yaw = GetYaw();

    // 璁＄畻瑙掑害璇樊锛堣€冭檻瑙掑害鐜粫锛?
    float error = attitude_control.target_yaw - current_yaw;

    // 灏嗚宸綊涓€鍖栧埌[-180, 180)鑼冨洿
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // 姝诲尯澶勭悊
    if (abs_float(error) < DEAD_ZONE) {
        error = 0.0f;
        attitude_control.yaw_pid.integral = 0.0f; // 閲嶇疆绉垎椤归槻姝㈢Н鍒嗛ケ鍜?
		attitude_correction = 0.0f;
        return;
    }

    // 浣跨敤PID璁＄畻鏃嬭浆閫熷害鎺у埗閲?
    attitude_correction = PID_Update_angle(&attitude_control.yaw_pid, error, dt);
    attitude_correction *= pose_coordination.sync_scale;

//	if (IsYawReached(attitude_control.target_yaw,0.5f)) { // 妫€鏌ユ槸鍚﹀埌杈剧洰鏍囪搴?
//        attitude_correction = 0.0f;
//    }
}

void PoseCoordination_Init(void)
{
    pose_coordination.is_enabled = 0;
    pose_coordination.sync_scale = 1.0f;
    pose_coordination.slow_distance_m = POSE_SYNC_SLOW_DISTANCE_M;
    pose_coordination.slow_yaw_deg = POSE_SYNC_SLOW_YAW_DEG;
    pose_coordination.min_scale = POSE_SYNC_MIN_SCALE;
}

void PoseCoordination_Enable(uint8_t enable)
{
    pose_coordination.is_enabled = enable;
    pose_coordination.sync_scale = 1.0f;
}

void SetTargetPose(float x, float y, float yaw)
{
    SetTargetPosition(x, y);
    PoseCoordination_Enable(1);
    PositionControl_Enable(1);
    EnableAttitudeStabilization(1);
    SetTargetYaw(yaw);
}

uint8_t IsPoseReached(float tolerance_xy, float tolerance_yaw_degrees)
{
    return IsPositionReached(tolerance_xy) && IsYawReached(attitude_control.target_yaw, tolerance_yaw_degrees);
}

void UpdatePoseCoordination(float dt)
{
    float distance_error_m;
    float yaw_error_deg;
    float pos_scale;
    float yaw_scale;

    if (!pose_coordination.is_enabled || dt <= 0.0f) {
        pose_coordination.sync_scale = 1.0f;
        return;
    }

    distance_error_m = sqrtf(
        powf(position_control.target_x * 0.001f - robot_pose_state.pos_x_mm * 0.001f, 2.0f) +
        powf(position_control.target_y * 0.001f - robot_pose_state.pos_y_mm * 0.001f, 2.0f));
    yaw_error_deg = NormalizeYawError(attitude_control.target_yaw - GetYaw());

    pos_scale = distance_error_m / pose_coordination.slow_distance_m;
    yaw_scale = fabsf(yaw_error_deg) / pose_coordination.slow_yaw_deg;
    pos_scale = ClampFloat(pos_scale, pose_coordination.min_scale, 1.0f);
    yaw_scale = ClampFloat(yaw_scale, pose_coordination.min_scale, 1.0f);
    pose_coordination.sync_scale = ClampFloat((pos_scale < yaw_scale) ? pos_scale : yaw_scale, pose_coordination.min_scale, 1.0f);
}

uint8_t move_to_pose_target(float x, float y, float yaw)
{
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
uint8_t is_rotating = 0;
// 鏃嬭浆鍒版寚瀹氳搴︼紙闈為樆濉烇級
uint8_t RotateToYaw(float target_yaw, float tolerance) {


    if (!is_rotating) {
        // 寮€濮嬫棆杞?
        SetTargetYaw(target_yaw);
        is_rotating = 1;
        return 0;
    }

    // 妫€鏌ユ槸鍚﹀埌杈剧洰鏍囪搴?
    if (IsYawReached(target_yaw, tolerance)) {
//        EnableAttitudeStabilization(0);
        is_rotating = 0;
        return 1; // 鏃嬭浆瀹屾垚
    }

    return 0; // 鏃嬭浆涓?
}
//*******************************瑙掑害pid***************************************//
//********************************鎽勫儚澶存牎姝id************************************//
// 鍏ㄥ眬鎽勫儚澶存牎姝ｅ彉閲?
CamStabilization_t cam_stabilization = {0};
float cam_correction_x = 0.0f;
float cam_correction_y = 0.0f;
uint8_t is_cam_stabilizing = 0;
int Is_coordgot =0;
// 鍒濆鍖栨憚鍍忓ごPID
void CamStabilization_Init(void) {
    // 鍒濆鍖朮杞碢ID: kp=0.8, ki=0.05, kd=0.2, 杈撳嚭闄愬箙卤100
    PID_Init_cam(&cam_stabilization.pid_x, 0.26f,0.0000f, 0.0f, 0.002f, 20.0f);

    // 鍒濆鍖朰杞碢ID: kp=0.7, ki=0.03, kd=0.15, 杈撳嚭闄愬箙卤100,
    PID_Init_cam(&cam_stabilization.pid_y, 0.26f,0.0000f,0.0f, 0.002f, 20.0f);

    cam_stabilization.is_enabled = 0;
    cam_correction_x = 0.0f;
    cam_correction_y = 0.0f;
	lookmode_centrl = 0;
	is_cam_stabilizing = 0;
	Is_coordgot =0;
}

// 鍚敤/绂佺敤鎽勫儚澶存牎姝?
void CamStabilization_Enable(uint8_t enable) {
    cam_stabilization.is_enabled = enable;

    IsLight = 1;
    if (!enable) {
        // 绂佺敤鏃堕噸缃姸鎬?
        cam_stabilization.pid_x.integral = 0.0f;
        cam_stabilization.pid_x.prev_error = 0.0f;
        cam_stabilization.pid_y.integral = 0.0f;
        cam_stabilization.pid_y.prev_error = 0.0f;
        cam_correction_x = 0.0f;
        cam_correction_y = 0.0f;
		IsLight = 0;
    }
}

// 鏇存柊鎽勫儚澶存牎姝?
void UpdateCamStabilization(float x_err, float y_err, float dt) {
    if (!cam_stabilization.is_enabled || dt <= 0) {
        cam_correction_x = 0.0f;
        cam_correction_y = 0.0f;
        return;
    }

    // 瀹氫箟姝诲尯闃堝€硷紙2鍍忕礌锛?
    #define CAM_DEAD_ZONE 0.0f

    // X杞存鍖哄鐞?
    float actual_x_err = (abs_float (x_err) > CAM_DEAD_ZONE) ? x_err : 0.0f;

    // Y杞存鍖哄鐞?
    float actual_y_err = (abs_float(y_err) > CAM_DEAD_ZONE) ? y_err : 0.0f;

    // 璁＄畻X杞存牎姝?
    cam_correction_x = PID_Update_cam(&cam_stabilization.pid_x, actual_x_err, dt);

    // 璁＄畻Y杞存牎姝?
    cam_correction_y = PID_Update_cam(&cam_stabilization.pid_y, actual_y_err, dt);
}

static ErrorQueue_t error_queue = {0};

// 鍒濆鍖栬宸槦鍒?
void ErrorQueue_Init(void) {
    error_queue.index = 0;
    error_queue.count = 0;
    error_queue.is_full = 0;
    memset(error_queue.errors_x, 0, sizeof(error_queue.errors_x));
    memset(error_queue.errors_y, 0, sizeof(error_queue.errors_y));
}

// 鍚戣宸槦鍒楁坊鍔犳暟鎹?
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

// 璁＄畻璇樊闃熷垪鐨勫潎鍊?
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
#define INT_MAX 480       //鏈€澶у亸宸粷瀵瑰€?
// 璁＄畻璇樊闃熷垪涓瓁/y鏂瑰悜鐨勬渶澶ф尝鍔ㄥ箙搴︼紙max - min锛?
void ErrorQueue_GetFluctuate(int *fluct_x, int *fluct_y) {
    int max_x = -INT_MAX, min_x = INT_MAX;
    int max_y = -INT_MAX, min_y = INT_MAX;
    uint8_t valid_count = error_queue.is_full ? ERROR_QUEUE_SIZE : error_queue.count;

    // 鑻ラ槦鍒楃┖锛屾尝鍔ㄥ箙搴︿负0
    if (valid_count == 0) {
        *fluct_x = 0;
        *fluct_y = 0;
        return;
    }

    // 閬嶅巻鏈夋晥鏁版嵁锛屾壘max鍜宮in
    for (int i = 0; i < valid_count; i++) {
        // x鏂瑰悜
        if (error_queue.errors_x[i] > max_x) max_x = error_queue.errors_x[i];
        if (error_queue.errors_x[i] < min_x) min_x = error_queue.errors_x[i];
        // y鏂瑰悜
        if (error_queue.errors_y[i] > max_y) max_y = error_queue.errors_y[i];
        if (error_queue.errors_y[i] < min_y) min_y = error_queue.errors_y[i];
    }

    // 娉㈠姩骞呭害 = 鏈€澶у€?- 鏈€灏忓€?
    *fluct_x = max_x - min_x;
    *fluct_y = max_y - min_y;
}

uint8_t IsCamReached(int tolerance_x, int tolerance_y,int MAX_Fluctuate_error) {
    int error_x = Camera_err.x;
    int error_y = Camera_err.y;

    // 灏嗗綋鍓嶈宸姞鍏ラ槦鍒?
    ErrorQueue_Push(error_x, error_y);

    // 璁＄畻闃熷垪涓墍鏈夎宸殑鍧囧€?
    float avg_error_x, avg_error_y;
	  int F_x,F_y;
    ErrorQueue_GetAverage(&avg_error_x, &avg_error_y);
    ErrorQueue_GetFluctuate(&F_x,&F_y);
    // 妫€鏌ラ槦鍒楁槸鍚﹀凡婊′笖鍧囧€煎湪瀹瑰樊鑼冨洿鍐?
    if (error_queue.is_full && avg_error_x <= tolerance_x && avg_error_y <= tolerance_y&&F_x<=MAX_Fluctuate_error&&F_y<=MAX_Fluctuate_error) {
        return 1;
    }

    return 0;
}

// 绠€鍖栫殑鎼滅储绛夊緟鍑芥暟 - 鍙湪鍥涗釜鏂瑰悜绉诲姩涓€娆★紝鐪嬪埌鐩爣绔嬪嵆杩斿洖
uint8_t Wait_for_uart_with_search(uint8_t point_index) {
    uint32_t start_time = HAL_GetTick();
    Is_coordgot = 0;

    // 绗竴姝ワ細鍏堟甯哥瓑寰呬竴灏忔鏃堕棿锛堟瘮濡?00ms锛?
    while (!Is_coordgot) {
        if (HAL_GetTick() - start_time > UART_WAIT_TIMEOUT_MS) {
            break;  // 鐭殏绛夊緟鍚庤繘鍏ユ悳绱㈡ā寮?
        }
        HAL_Delay(1);
    }

    if (Is_coordgot) {
        return 1;  // 鍦ㄧ瓑寰呮湡闂村氨鏀跺埌浜嗘暟鎹?
    }

    // 绗簩姝ワ細寮€濮嬫悳绱㈢Щ鍔紙鍥涗釜鏂瑰悜鍚勭Щ鍔?0mm锛?
    float base_x = task1_targets[point_index].x;
    float base_y = task1_targets[point_index].y;

    // 瀹氫箟鍥涗釜鎼滅储鏂瑰悜锛氬墠銆佸悗銆佸乏銆佸彸
    float search_points[4][2] = {
        {base_x + 0.01f, base_y},  // 鍓?
        {base_x - 0.01f, base_y},  // 鍚?
        {base_x, base_y + 0.01f},  // 宸?
        {base_x, base_y - 0.01f}   // 鍙?
    };

    // 渚濇绉诲姩鍒板洓涓悳绱㈢偣
    for (int i = 0; i < 4; i++) {
        // 绉诲姩鍒版悳绱㈢偣
        SetTargetPosition(search_points[i][0], search_points[i][1]);
        PositionControl_Enable(1);

        // 绛夊緟绉诲姩瀹屾垚锛屾湡闂村鏋滄敹鍒版暟鎹珛鍗宠繑鍥?
        while (!IsPositionReached(0.002f)) {
            if (Is_coordgot) {
                PositionControl_Enable(0);  // 绔嬪嵆鍋滄绉诲姩
                HAL_Delay(5);
                return 1;  // 鎼滅储鎴愬姛锛屾敹鍒版暟鎹?
            }
            HAL_Delay(10);
        }

        // 妫€鏌ユ槸鍚﹀湪绉诲姩瀹屾垚鍚庣珛鍗虫敹鍒版暟鎹?
        if (Is_coordgot) {
            HAL_Delay(5);
            return 1;
        }
    }

    // 鍥涗釜鏂瑰悜閮界Щ鍔ㄥ畬姣曪紝浠嶆湭鏀跺埌鏁版嵁
    // 鍥炲埌鍘熺偣
    SetTargetPosition(base_x, base_y);
    PositionControl_Enable(1);

    // 绛夊緟鍥炲埌鍘熺偣
    while (!IsPositionReached(0.002f)) {
        if (Is_coordgot) {
            PositionControl_Enable(0);
            HAL_Delay(5);
            return 1;
        }
        HAL_Delay(10);
    }

    PositionControl_Enable(0);
    return 0;  // 鎼滅储澶辫触
}
extern uint32_t camtim;
extern uint8_t camtim_button;
uint8_t iscamreached_safecount = 0;
// 鎽勫儚澶存牎姝ｅ埌涓績锛堥潪闃诲锛?
uint8_t CamStabilizationToCenter(int tolerance_x, int tolerance_y,int MAX_Fluctuate_error) {

    if (!is_cam_stabilizing) {
        // 寮€濮嬫牎姝?
        ErrorQueue_Init();				// 寮€濮嬫牎姝ｆ椂娓呯┖闃熷垪
        PositionControl_Enable(0);		//鍏堝叧闂潗鏍囨牎姝ｏ紝纭繚鍧愭爣鏍℃鍜屾憚鍍忓ご鏍℃浜掓枼
		CamStabilization_Enable(1);
        is_cam_stabilizing = 1;
		IsLight = 1;
        return 0;
    }

    // 妫€鏌ユ槸鍚﹀埌杈剧洰鏍?
    if (IsCamReached(tolerance_x, tolerance_y,MAX_Fluctuate_error)) {
		iscamreached_safecount ++;
		if(iscamreached_safecount>10){
			CamStabilization_Enable(0);
			is_cam_stabilizing = 0;
			IsLight = 0;
			return 1; // 鏍℃瀹屾垚
		}
    }

    return 0; // 鏍℃涓?
}
//********************************鎽勫儚澶存牎姝id************************************//
// 鍏ㄥ眬鐩爣閫熷害鍙橀噺
TargetVelocity_t target_velocity = {0};

// 瀹氫箟4涓數鏈虹殑鍏ㄥ眬鍙橀噺
Motor_t motor1 = {0, 0};
Motor_t motor2 = {0, 0};
Motor_t motor3 = {0, 0};
Motor_t motor4 = {0, 0};
Motor_t motor5 = {0, 0};
// 搴曠洏鍙傛暟
static ChassisConfig_t chassis_config;
static float k_omega;  // 鏃嬭浆绯绘暟 k = (Lx + Ly)/2

void SetTargetVelocity(float vx, float vy, float omega) {
    target_velocity.vx = vx;
    target_velocity.vy = vy;
    target_velocity.omega = omega;
}

void SetTargetLinearVelocity(float vx, float vy) {
    target_velocity.vx = vx;
    target_velocity.vy = vy;
    target_velocity.omega = 0.0f;
}
// 鍒濆鍖栧簳鐩樺弬鏁?
void Mecanum_Init(ChassisConfig_t config) {
    chassis_config = config;
    k_omega = (config.wheel_dist_x + config.wheel_dist_y) / 2.0f;
}

// 杩愬姩瑙ｇ畻鏍稿績鍑芥暟锛堜笉甯﹁搴﹁ˉ鍋匡級
void Mecanum_Kinematics(float vx, float vy, float omega)
{
    float actual_omega = omega;

    // 濡傛灉闇€瑕佷娇鐢ㄥЭ鎬佹牎姝?
    if (attitude_control.is_enabled) {
        actual_omega += attitude_correction;

        // 闄愬埗鏈€澶ф棆杞€熷害
        #define MAX_OMEGA 20.0f
        if (actual_omega > MAX_OMEGA) actual_omega = MAX_OMEGA;
        if (actual_omega < -MAX_OMEGA) actual_omega = -MAX_OMEGA;
    }
	// 濡傛灉闇€瑕佷娇鐢ㄦ憚鍍忓ご鏍℃
    if (cam_stabilization.is_enabled) {
        vy += cam_correction_x * 0.01f;
		vx += cam_correction_y * 0.01f;
	}

    // 1. 璁＄畻鍚勮疆鐨勭洰鏍囩嚎閫熷害锛坢/s锛?
    // 杞瓙椤哄簭锛氬乏鍓?4) 鍙冲墠(2) 宸﹀悗(3) 鍙冲悗(1)
    float v4 =  vx - vy - k_omega * actual_omega;  // 宸﹀墠 (4)
    float v2 =  vx + vy + k_omega * actual_omega;  // 鍙冲墠 (2)
    float v3 =  vx + vy - k_omega * actual_omega;  // 宸﹀悗 (3)
    float v1 =  vx - vy + k_omega * actual_omega;  // 鍙冲悗 (1)

    // 2. 杞崲涓?RPM锛圧PM = (绾块€熷害 / (2蟺 * 杞崐寰?) * 60锛?
    const float linear_to_rpm = 60.0f / (2 * 3.1415926f * chassis_config.wheel_radius);
    float rpm4 = v4 * linear_to_rpm;
    float rpm2 = v2 * linear_to_rpm;
    float rpm3 = v3 * linear_to_rpm;
    float rpm1 = v1 * linear_to_rpm;

    // 3. 鎸夋渶澶ц疆閫熺粺涓€姣斾緥缂╂斁锛屼繚鎸佸洓杞€熷害姣斾緥
    float max_abs_rpm = fabsf(rpm4);
    float scale = 1.0f;

    if (fabsf(rpm2) > max_abs_rpm) max_abs_rpm = fabsf(rpm2);
    if (fabsf(rpm3) > max_abs_rpm) max_abs_rpm = fabsf(rpm3);
    if (fabsf(rpm1) > max_abs_rpm) max_abs_rpm = fabsf(rpm1);

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

    // 4. 璁剧疆鐢垫満鏂瑰悜
    motor4.direction = (motor4.speed_rpm >= 0) ? 0 : 1;  // 宸﹀墠 (4)
    motor2.direction = (motor2.speed_rpm >= 0) ? 1 : 0;  // 鍙冲墠 (2)
    motor3.direction = (motor3.speed_rpm >= 0) ? 0 : 1;  // 宸﹀悗 (3)
    motor1.direction = (motor1.speed_rpm >= 0) ? 1 : 0;  // 鍙冲悗 (1)
}

// 浠ヤ笅鏄師鏈夌殑杩愬姩鎺у埗鍑芥暟
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

//********************************寰抗pid閮ㄥ垎锛堟病鐢ㄥ埌锛?***********************************//
uint8_t IsRun = 0;
// PID鎺у埗鍣ㄥ垵濮嬪寲
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


// PID鏇存柊璁＄畻
float PID_Update(PID_Controller* pid, float error, float dt) {
    // 姣斾緥椤?
    float proportional = pid->Kp * error;

    // 绉垎椤癸紙甯︽姉楗卞拰锛?
    pid->integral += error * dt;
    float integral = pid->Ki * pid->integral;

    // 寰垎椤?
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // 璁＄畻鎬昏緭鍑?
    float output = proportional + integral + derivative;

    // 闄愬埗杈撳嚭鑼冨洿
    if (output > pid->max_output) {
        output = pid->max_output;
        pid->integral -= error * dt; // 鎶楅ケ鍜屽鐞?
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
        pid->integral -= error * dt; // 鎶楅ケ鍜屽鐞?
    }

    return output;
}
float PID_Update_cam(PID_Controller* pid, float error, float dt) {
    // 姣斾緥椤?
    float proportional = pid->Kp * error;
	float proportional2 =pid->Kp2*error*fabsf(error);

    // 绉垎椤癸紙甯︽姉楗卞拰锛?
    pid->integral += error * dt;
    float integral = pid->Ki * pid->integral;

    // 寰垎椤?
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // 璁＄畻鎬昏緭鍑?

    float output = proportional +proportional2 + integral + derivative;

//	if(output>0){output +=0.4f;}
//	else if(output<0){output -=0.4f;}
//	else output =0;

    // 闄愬埗杈撳嚭鑼冨洿
    if (output > pid->max_output) {
        output = pid->max_output;
        pid->integral -= error * dt; // 鎶楅ケ鍜屽鐞?
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
        pid->integral -= error * dt; // 鎶楅ケ鍜屽鐞?
    }

    return output;
}
// PID鏇存柊璁＄畻
float PID_Update_angle(PID_Controller* pid, float error, float dt) {
    // 姣斾緥椤?
    float proportional = pid->Kp * error;

    // 绉垎椤癸紙甯︽姉楗卞拰锛?
    pid->integral += error * dt;
    float integral = pid->Ki * pid->integral;

    // 寰垎椤?
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

	float G = robot_pose_state.yaw_gyro_dps*g;
    // 璁＄畻鎬昏緭鍑?
    float output = proportional + integral + derivative;

    // 闄愬埗杈撳嚭鑼冨洿
    if (output > pid->max_output) {
        output = pid->max_output;
        pid->integral -= error * dt; // 鎶楅ケ鍜屽鐞?
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
        pid->integral -= error * dt; // 鎶楅ケ鍜屽鐞?
    }

    return output;
}
// 鍏ㄥ眬PID鎺у埗鍣ㄥ疄渚?
static PID_Controller line_follow_pid;
uint8_t is_line_follow_enabled = 0; // 鏂板锛氬惊杩逛娇鑳芥爣蹇?
// 寰抗鍒濆鍖栧嚱鏁?
void Line_Follow_Init(void) {
    // 鍒濆鍖朠ID鍙傛暟锛岃繖浜涘弬鏁板彲鑳介渶瑕佹牴鎹疄闄呰皟璇曡皟鏁?
    PID_Init(&line_follow_pid, 0.9f, 0.02f, 200.0f, 0.48f);  // Kp, Ki, Kd, max_output
	is_line_follow_enabled = 0; // 鍒濆鐘舵€佷负绂佺敤
	IsRun = 0;
}
// 鏂板锛氬惊杩逛娇鑳?绂佺敤鍑芥暟
void Line_Follow_Enable(uint8_t enable) {
    is_line_follow_enabled = enable;

    if (!enable) {
        // 绂佺敤鏃堕噸缃甈ID鐘舵€?
        line_follow_pid.integral = 0.0f;
        line_follow_pid.prev_error = 0.0f;
		stop();
    }
}
// 寰抗鎺у埗鍑芥暟
void Line_Follow_Control(uint8_t* sensor_values, float base_speed, float dt) {
	// 濡傛灉寰抗绂佺敤锛岄噸缃姸鎬佸苟杩斿洖
    if (!is_line_follow_enabled) {
        line_follow_pid.integral = 0.0f;
        line_follow_pid.prev_error = 0.0f;

		return;
    }
	// 鏂扮殑璇樊璁＄畻鏂瑰紡 - 鏇村己璋冧腑蹇冧紶鎰熷櫒
    float error = 0.0f;

    // 涓績浼犳劅鍣ㄤ紭鍏?
    if(is_task1_active()){
		if( sensor_values[2] == 0 ||sensor_values[3] == 0) {
			// 褰撲腑蹇冧紶鎰熷櫒妫€娴嬪埌榛戠嚎鏃讹紝寮哄埗璇樊褰掗浂
			error = 0.0f;
			// 鍚屾椂閲嶇疆PID绉垎椤癸紝閬垮厤鍘嗗彶璇樊褰卞搷
			line_follow_pid.integral = 0.0f;
			line_follow_pid.prev_error = 0.0f;
		}
		else {
			// 鍙湁褰撲腑蹇冩病鏈夋娴嬪埌鏃讹紝鎵嶈绠楀姞鏉冭宸?

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
				// 褰撲腑蹇冧紶鎰熷櫒妫€娴嬪埌榛戠嚎鏃讹紝寮哄埗璇樊褰掗浂
				error = 0.0f;
				// 鍚屾椂閲嶇疆PID绉垎椤癸紝閬垮厤鍘嗗彶璇樊褰卞搷
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

    // 鏇存柊PID鎺у埗鍣?
    float omega = PID_Update(&line_follow_pid, error, dt);

    // 浣跨敤杩愬姩瀛︽帶鍒?
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

    if (dt <= 0) dt = 0.01f; // 鏈€灏忔椂闂撮棿闅?
    if(is_task1_active()){
		Line_Follow_Control(Anolog, 0.2376f, dt);}
	else{
		Line_Follow_Control(Anolog, 0.229f, dt);
	}
}

//********************************寰抗pid閮ㄥ垎************************************//

//**************************** 浠诲姟璋冨害瀹炵幇 ****************************//


//==================================================================
//鍑?鏁?鍚嶏細鍓嶅線鐩爣鐐?
//鍔?   鑳斤細璁剧疆鐩爣鐐瑰苟浣胯兘杩愬姩锛岀劧鍚庡垽鏂槸鍚︾Щ鍔ㄥ畬鎴?
//杈撳叆鍙傛暟锛氱洰鏍噚锛寉鍧愭爣
//杩?鍥?
uint8_t target_set = 0;
uint8_t move_to_target(float x, float y) {


    if (!is_position_control_enabled && !target_set) {
        SetTargetPosition(x, y);
        PositionControl_Enable(1);
        target_set = 1;
        return 0;
    }

    if (!is_position_control_enabled && target_set) {
        // 绉诲姩瀹屾垚
        target_set = 0;
        return 1; // 绉诲姩瀹屾垚
    }

    return 0; // 绉诲姩涓?
}
static uint8_t move_relative_x_once(float delta_x_mm) {
    static uint8_t relative_target_set = 0;
    float target_x;
    float target_y;

    if (!is_position_control_enabled && !relative_target_set) {
        target_x = robot_pose_state.pos_x_mm + delta_x_mm;
        target_y = robot_pose_state.pos_y_mm;
        SetTargetPosition(target_x, target_y);
        PositionControl_Enable(1);
        relative_target_set = 1;
        return 0;
    }

    if (!is_position_control_enabled && relative_target_set) {
        relative_target_set = 0;
        return 1;
    }

    return 0;
}

static uint8_t task1_relative_forward_back(void) {
    static uint8_t relative_stage = 0;

    if (relative_stage == 0) {
				pospid_reinit(LOWESTSPEED, POSITION_PID_TASK1_RELATIVE_KP);
        if (move_relative_x_once(TASK1_RELATIVE_X_MOVE_MM)) {
            relative_stage = 1;
        }
        return 0;
    }

    if (relative_stage == 1) {
        if (move_relative_x_once(-TASK1_RELATIVE_X_MOVE_MM)) {
            relative_stage = 0;
						pospid_reinit(MIDSPEED, POSITION_PID_DEFAULT_KP);
            return 1;
        }
        return 0;
    }

    relative_stage = 0;
    return 0;
}
/***********Task1***************/

// 鍚姩浠诲姟1
void start_task1(void) {
    task1_state.active = 1;
    task1_state.current_step = 1;
    task1_state.completed = 0;
    task1_state.step_start = HAL_GetTick();
}

uint8_t is_hasgetcolor = 0;
// 闈為樆濉炰换鍔℃墽琛?
void run_task1(void) {
    if (!task1_state.active) return;//鏈娇鑳戒换鍔″垯杩斿洖
    uint32_t current_time = HAL_GetTick();
    switch (task1_state.current_step) {
//		case 0:	EnableAttitudeStabilization(1);SetTargetYaw(0.0f);
//				Servo_Action_Group(1);pospid_reinit(LOWSPEED, POSITION_PID_DEFAULT_KP);
//				if (move_to_target(task1_targets[20].x, task1_targets[20].y)/*鐐?*/) {task1_state.current_step++;}
//				if(IsPositionReached(0.10f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 2060);}
////				if(IsPositionReached(0.05f)){SetTargetPosition(task1_targets[0].x, task1_targets[0].y);task1_state.current_step++;}
//				break;
        case 1:	EnableAttitudeStabilization(1);SetTargetYaw(0.0f);Move_to_turnplate(1);
				Servo_Action_Group(1);
				tasknum = 1;
				pospid_reinit(LOWSPEED, POSITION_PID_DEFAULT_KP);
				if (move_to_pose_target(task1_targets[0].x, task1_targets[0].y, 0.0f)/*鐐逛簩缁寸爜*/) {Servo_Action_Group(2);task1_state.current_step++;}
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
				pospid_reinit(MIDSPEED, POSITION_PID_DEFAULT_KP);Get_Materialcolor(4);
				if (move_to_pose_target(task1_targets[6].x, task1_targets[6].y, -90.0f)/*绉诲埌鐐筧*/ ) {
										Find_corresponding_material(1);
										task1_state.current_step++;}break;
		case 12:if(Wait_for_uart_with_search(6)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 瓒呮椂澶勭悊锛屾瘮濡傞噸璇曟垨閿欒澶勭悊 */ } break;
		case 13:if(task1_relative_forward_back()){task1_state.current_step++;}break;
		case 14:pospid_reinit(HIGHSPEED, POSITION_PID_DEFAULT_KP);if (move_to_pose_target(task1_targets[7].x, task1_targets[7].y, -90.0f)/*绉诲埌鐐筨*/) {
					Find_corresponding_material(2);
					task1_state.current_step++;}break;
		case 15:if(Wait_for_uart_with_search(7)) { task1_state.current_step++; }
				else {task1_state.current_step++;/* 瓒呮椂澶勭悊锛屾瘮濡傞噸璇曟垨閿欒澶勭悊 */ } break;
		case 16:if(task1_relative_forward_back()){task1_state.current_step++;}break;
		case 17:if (move_to_pose_target(task1_targets[8].x, task1_targets[8].y, -90.0f)) {
					Find_corresponding_material(3);
					task1_state.current_step++;}break;
		case 18:if(Wait_for_uart_with_search(8)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 瓒呮椂澶勭悊锛屾瘮濡傞噸璇曟垨閿欒澶勭悊 */ } break;
		case 19:if(task1_relative_forward_back()){task1_state.current_step++;}break;
		case 20:if (move_to_pose_target(task1_targets[9].x, task1_targets[9].y, -90.0f)) {
					Find_corresponding_material(4);
					task1_state.current_step++;}break;
		case 21:if(Wait_for_uart_with_search(9)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 瓒呮椂澶勭悊锛屾瘮濡傞噸璇曟垨閿欒澶勭悊 */ } break;
		case 22:if(task1_relative_forward_back()){task1_state.current_step++;}break;
		case 23:if (move_to_pose_target(task1_targets[10].x, task1_targets[10].y, -90.0f)) {
					Find_corresponding_material(5);
					task1_state.current_step++;}break;
		case 24:if(Wait_for_uart_with_search(10)) { task1_state.current_step++; }
				else { task1_state.current_step++;/* 瓒呮椂澶勭悊锛屾瘮濡傞噸璇曟垨閿欒澶勭悊 */ } break;
		case 25:if(task1_relative_forward_back()){task1_state.current_step++;}break;
		case 26://Servo_Action_Group(2);EnableAttitudeStabilization(1);
				tasknum = 2;if (move_to_pose_target(task1_targets[11].x, task1_targets[11].y, 0.0f)/*鐐逛簩缁寸爜2*/) {Move_to_turnplate(3);/*绉诲埌杞洏1*/task1_state.current_step++;}break;
		case 27:if (move_to_pose_target(task1_targets[19].x, task1_targets[19].y, -90.0f)/*濂栨澂3涓嬮潰*/){task1_state.current_step++;}break;
		case 28:pospid_reinit(LOWESTSPEED, POSITION_PID_DEFAULT_KP);if (move_to_pose_target(task1_targets[12].x, task1_targets[12].y, -90.0f)/*濂栨澂3*/){task1_state.current_step++;Move_to_turnplate(2);}break;
		case 29:if (move_to_pose_target(task1_targets[13].x, task1_targets[13].y, -90.0f)/*濂栨澂2*/) {task1_state.current_step++;Move_to_turnplate(1);}break;
		case 30:if (move_to_pose_target(task1_targets[14].x, task1_targets[14].y, -90.0f)/*濂栨澂1*/) {__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Material[1]+200);task1_state.current_step++;}	break;
		case 31:
				pospid_reinit(HIGHSPEED, POSITION_PID_DEFAULT_KP);
				if (move_to_pose_target(task1_targets[15].x, task1_targets[15].y+15.0f, -90.0f)/*浜氬啗鍙?*/) {Servo_Action_Group(6);Find_corresponding_material(2);task1_state.current_step++;break;}
				if(IsPositionReached(0.6f)){Servo_Action_Group(5);}break;
		case 32:if(Wait_for_uart_with_search(15)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 瓒呮椂澶勭悊锛屾瘮濡傞噸璇曟垨閿欒澶勭悊 */ } break;
		case 33:if(task1_relative_forward_back()){Servo_Action_Group(3);HAL_Delay(100);
						task1_state.current_step++;}break;
		case 34:if (move_to_pose_target(task1_targets[16].x, task1_targets[16].y+15.0f, -90.0f)/*鍐犲啗鍙?*/) {Servo_Action_Group(4);Find_corresponding_material(1);task1_state.current_step++;}break;
		case 35:if(Wait_for_uart_with_search(16)) { task1_state.current_step++; }
				else { task1_state.current_step++;/* 瓒呮椂澶勭悊锛屾瘮濡傞噸璇曟垨閿欒澶勭悊 */ } break;
		case 36:if(task1_relative_forward_back()){Servo_Action_Group(3);
						task1_state.current_step++;}break;
		case 37:if (move_to_pose_target(task1_targets[17].x, task1_targets[17].y+25.0f, -90.0f)) {Servo_Action_Group(7);Find_corresponding_material(3);task1_state.current_step++;}break;
		case 38:if(Wait_for_uart_with_search(17)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 瓒呮椂澶勭悊锛屾瘮濡傞噸璇曟垨閿欒澶勭悊 */ } break;
		case 39:if(task1_relative_forward_back()){task1_state.current_step++;}break;
		case 40:pospid_reinit(HIGHESTSPEED, POSITION_PID_DEFAULT_KP);raceh = 0.200f;
				if (move_to_pose_target(task1_targets[20].x, task1_targets[20].y, -90.0f)/*鍥炲*/) {task1_state.current_step = 41;}break;
		case 41:raceh = 0.002f;if (move_to_pose_target(task1_targets[18].x, task1_targets[18].y, -90.0f)/*鍥炲*/) {task1_state.current_step++;}break;
        default:
            task1_state.active = 0;
            task1_state.completed = 1;
            break;
    }
}
//闈為樆濉炲啓娉?
//case 6:SetTargetYaw(-90.0f);if (move_to_target(task1_targets[6].x, task1_targets[6].y)/*鐐筧*/) {
//										Pos_moveconctrl(1,100,6400);task1_state.step_start = current_time; task1_state.current_step++;}break;
//		case 7:if (current_time - task1_state.step_start >= 1000) {//寤舵椂涓€绉?
//					Pos_moveconctrl(0,100,6400);task1_state.step_start = current_time;task1_state.current_step++;
//					}break;
//		case 8:if (current_time - task1_state.step_start >= 1000) {//寤舵椂涓€绉?
//					if (move_to_target(task1_targets[7].x, task1_targets[7].y)/*鐐筨*/) {task1_state.current_step++;}
//					}break;
// 妫€鏌ヤ换鍔℃槸鍚︽縺娲?
uint8_t is_task1_active(void) {
    return task1_state.active;
}
/***********Task1***************/

