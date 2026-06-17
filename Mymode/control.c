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


float raceh = POSITION_REACHED_TOLERANCE_M;  // 娴ｅ秶鐤嗛悳顖氱秼閸撳秴鍩屾担宥咁啇瀹割噯绱濋崡鏇氱秴m
uint16_t Material[5] = {575,970,1365,1785,2165};//閺夌儐鍓涘ú蹇涘锤閹邦厾鍨?
// 濞寸姾顕ф慨?闁汇劌瀚ú浼村冀閸モ晛浠柛褎鍔栭悥锝夊极閹殿喚鐭?
static const Point2D_t task1_targets[] = {
    {250.25f, 650.0f},     // 缂佷究鍨圭槐?		闁?闁规鍋傜花鈺冪磼鐎靛摜鍨?
    {258.25f, 1058.24f},   // 缂佷究鍨圭槐?		闁?闁绘せ鏅涘?
    {604.89f, 1399.38f},   // 缂佷究鍨圭槐?		闁?闁绘せ鏅涘?
    {1075.0f, 1524.0f},    // 缂佷究鍨圭槐?		闁?闁绘せ鏅涘?
    {1545.10f, 1399.38f},  // 缂佷究鍨圭槐?		闁?闁绘せ鏅涘?
    {1891.74f, 1058.24f},  // 缂佷究鍨圭槐?		闁?闁绘せ鏅涘?
	{835.14f, 631.60f},     // 缂佷究鍨圭槐?		闁?  a
	{553.90f, 425.91f},	   // 缂佷究鍨圭槐?		闁?  b
	{836.02f, -220.80f},   // 缂佷究鍨圭槐?		闁?  c
	{554.98f, -334.63f},   // 缂佷究鍨圭槐?		闁?0 d
	{623.60f, -837.96f},    // 缂佷究鍨圭槐?0	闁?1 e
	{345.80f, -646.16f},   // 缂佷究鍨圭槐?1	闁?2闁规鍋傜花鈺冪磼鐎靛摜鍨?
	{409.58f, -1467.21f},  // 缂佷究鍨圭槐?2	闁?3 濠靛倹鐗楀?
	{860.0f, -1564.0f},    // 缂佷究鍨圭槐?3	闁?2 濠靛倹鐗楀?
	{1315.42f, -1442.21f}, // 缂佷究鍨圭槐?4	闁?2 濠靛倹鐗楀?
	{1710.0f, -270.0f},    // 缂佷究鍨圭槐?5	闁?6 濞存粍鑹鹃崯妤佺附閺嵮冮叡
	{1695.0f, 0.0f},       // 缂佷究鍨圭槐?6	闁?7 闁告劗濮撮崯妤佺附閺嵮冮叡
	{1715.0f, 270.0f},     // 缂佷究鍨圭槐?7	闁?8 閻庢冻绲介崯妤佺附閺嵮冮叡
	{0.0f, 0.0f},          // 缂佷究鍨圭槐?8	闁?9 閻犙冨槻椤劙鎮?
	{150.00f, -1467.21f},	// 缂佷究鍨圭槐?9	闁?0 濠靛倹鐗楀?闁告挸绉瑰鐗堟交閸パ冾唺闁?
	{1000.f, 0.0f},		// 缂佷究鍨圭槐?0	闁?1 闁搞儳鍋涢宥夊礈瀹ュ牏绠栭幖杈惧閸?
	{156.27f, 682.54f}		// 缂佷究鍨圭槐?1

};
// X42濞达絽绉堕悿鍡涘箳閵夈儱缍撳ù锝堟硶閺併倗鎲撮幒鎴濐唺闁告娲戠紞鍛存晬鐏炶棄鏂уù锝呯Ф閻ゅ棝宕楅妷銉ョ稉闁告瑥鍊归弳鐔煎箰?200闁兼潙顦崯?闁革箑鐗婂畷鑼不濡炲皷鍋?
#define ZDT_X42_POS_DEG_PER_CLK (360.0f / 3200.0f)
#define TASK1_RELATIVE_X_MOVE_MM 100.0f
// 濞寸姾顕ф慨鐔兼偐閼哥鍋撴担绋跨秮闂?
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
		ZDT_X42_V2_Synchronous_motion(0); 								 // 妤犵偞瀵ч幐閬嶅捶閺夋寧绲?閻熸瑱绠戣ぐ?

	}
	if(dir == 0){
		ZDT_X42_V2_Traj_Position_Control(1,0,acc,dec,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(2,0,acc,dec,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(3,1,acc,dec,(float)vel,position_deg,0,1);

		ZDT_X42_V2_Traj_Position_Control(4,1,acc,dec,(float)vel,position_deg,0,1);
		HAL_Delay(5);
		ZDT_X42_V2_Synchronous_motion(0); 								 // 妤犵偞瀵ч幐閬嶅捶閺夋寧绲?閻熸瑱绠戣ぐ?

	}
}
void POS_move(void){
	Is_vel_mode = 0;//闁稿繑濞婂Λ鎾焻閻斿嘲顔婃俊顖椻偓宕囩
	Pos_moveconctrl(1,50,1058);
	HAL_Delay(1800);
	Pos_moveconctrl(0,50,1442);
	HAL_Delay(1800);
	Is_vel_mode = 1;//闁瑰灚鎸哥槐鎴︽焻閻斿嘲顔婃俊顖椻偓宕囩
}
//void POS_move2(void){
//	Is_vel_mode = 0;//闁稿繑濞婂Λ鎾焻閻斿嘲顔婃俊顖椻偓宕囩
//	Pos_moveconctrl(1,200,457);
//	HAL_Delay(2000);
//	Pos_moveconctrl(0,200,457);
//	HAL_Delay(2000);
//	Is_vel_mode = 1;//闁瑰灚鎸哥槐鎴︽焻閻斿嘲顔婃俊顖椻偓宕囩
//}
//void POS_move3(void){
//	Is_vel_mode = 0;//闁稿繑濞婂Λ鎾焻閻斿嘲顔婃俊顖椻偓宕囩
//	Pos_moveconctrl(1,200,457);
//	HAL_Delay(2000);
//	Pos_moveconctrl(0,200,457);
//	HAL_Delay(2000);
//	Is_vel_mode = 1;//闁瑰灚鎸哥槐鎴︽焻閻斿嘲顔婃俊顖椻偓宕囩
//}
	//********************************濞达絽绉堕悿鍡涘箳瑜嶉崺姊d****************************************//
// 闁稿繈鍔岄惇顒佹媴瀹ュ洨鏋傞柟璨夊啫鐓戦悗鍦仒缁?
PositionController_t position_control;
uint8_t is_position_control_enabled = 0;
PoseCoordination_t pose_coordination = {0};
static float position_plan_speed_mps = 0.0f;
static float position_plan_accel_mps2 = 0.0f;

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

static void PositionPlanner_Reset(void)
{
    position_plan_speed_mps = 0.0f;
    position_plan_accel_mps2 = 0.0f;
}

static void MotionState_Reset(void);

static float PositionSpeedPlan_Update(float distance_error_m, float dt)
{
    float cruise_speed = ClampFloat(POSITION_PLANNER_CRUISE_SPEED_MPS, 0.0f, position_control.pid_x.max_output);
    float brake_speed = sqrtf(2.0f * POSITION_PLANNER_MAX_DECEL_MPS2 * distance_error_m);
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

    if (position_plan_speed_mps > target_speed) {
        target_accel = -POSITION_PLANNER_MAX_DECEL_MPS2;
    } else if (position_plan_speed_mps < target_speed) {
        target_accel = POSITION_PLANNER_MAX_ACCEL_MPS2;
    }

    accel_delta = target_accel - position_plan_accel_mps2;
    accel_delta = ClampFloat(accel_delta, -POSITION_PLANNER_MAX_JERK_MPS3 * dt, POSITION_PLANNER_MAX_JERK_MPS3 * dt);
    position_plan_accel_mps2 += accel_delta;
    position_plan_accel_mps2 = ClampFloat(position_plan_accel_mps2, -POSITION_PLANNER_MAX_DECEL_MPS2, POSITION_PLANNER_MAX_ACCEL_MPS2);

    position_plan_speed_mps += position_plan_accel_mps2 * dt;
    position_plan_speed_mps = ClampFloat(position_plan_speed_mps, 0.0f, cruise_speed);

    if (position_plan_speed_mps > target_speed && position_plan_accel_mps2 > 0.0f) {
        position_plan_speed_mps = target_speed;
        position_plan_accel_mps2 = 0.0f;
    }

    if (position_plan_speed_mps < POSITION_PLANNER_MIN_SPEED_MPS && target_speed > POSITION_PLANNER_MIN_SPEED_MPS) {
        position_plan_speed_mps = POSITION_PLANNER_MIN_SPEED_MPS;
    }

    return position_plan_speed_mps;
}

// 濞达絽绉堕悿鍡涘箳瑜嶉崺妤呭礆濠靛棭娼楅柛?
void PositionControl_Init(void) {
    // 闁告帗绻傞～鎰板礌閺堫喗娼紙鐘电Т缂傚啰鏀琁D - 闁告瑥鍊归弳鐔兼閳ь剛鎲版担鍦闁硅鍠栭悿鍕⒔閸涙壆娈堕悹?
    PID_Init(&position_control.pid_x, POSITION_PID_DEFAULT_KP, 0.00f, POSITION_PID_DEFAULT_KD, HIGHSPEED); // Kp, Ki, Kd, max_output

    // 闁告帗绻傞～鎰板礌閺堢増娼紙鐘电Т缂傚啰鏀琁D
    PID_Init(&position_control.pid_y, POSITION_PID_DEFAULT_KP, 0.00f, POSITION_PID_DEFAULT_KD, HIGHSPEED);

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
// 闁告凹鍨抽弫?缂佸倷鑳堕弫銈嗘媴瀹ュ洨鏋傞柟璨夊啫鐓?
void PositionControl_Enable(uint8_t enable) {
    position_control.is_enabled = enable;
    is_position_control_enabled = enable;

    if (!enable) {
        // 缂佸倷鑳堕弫銈夊籍閸洖娅㈢紓鍐敩ID闁绘鍩栭埀?
        position_control.pid_x.integral = 0.0f;
        position_control.pid_x.prev_error = 0.0f;
        position_control.pid_y.integral = 0.0f;
        position_control.pid_y.prev_error = 0.0f;
        PositionPlanner_Reset();

        // 闁稿绮嶉娑氫焊韫囨碍绨?
        stop();
    } else {
        // 闁告凹鍨抽弫銈夊籍閹偊鍞剁憸鐗堟礀缂嶅宕滃鍡橆槯闂?
        position_control.last_update_time = HAL_GetTick();
    }
}

// 閻犱礁澧介悿鍡涙儎椤旂晫鍨煎ù锝呯Ф閻ゅ棝鏁嶉崼婵嗘锭闁硅矇鍐ㄧ厬x,y濞达絽绉堕悿鍡涙晬瀹€鍐炬健閹艰揪绠戝畷鐔兼偑椤掍礁浠橀柛鎺曨啇缁?
void SetTargetPosition(float x, float y) {
    Is_vel_mode = 1;
	position_control.target_x = x;
    position_control.target_y = y;
    target_velocity.omega = 0.0f;
    // 闂佹彃绉堕悿鍝朓D缂佸鍨伴崹搴亜閻у摜绀夐梺顒€鐏濋崢銈囩玻娴ｇ缍侀悽顖ら檮濞肩敻鎯冮崟顓夋繈宕氶崱娑栧仚闁?
    position_control.pid_x.integral = 0.0f;
    position_control.pid_x.prev_error = 0.0f;
    position_control.pid_y.integral = 0.0f;
    position_control.pid_y.prev_error = 0.0f;
    PositionPlanner_Reset();
}

// 婵☆偀鍋撻柡灞诲劜濡叉悂宕ラ敃鈧崺灞炬綇閸撗勭獥闁哄秴娲ｇ紞鍛磾椤曞棛绀勯柛娆樹簼椤ュ懘寮婚¨?y濞达絽绉堕悿鍡涙晬?
uint8_t IsPositionReached(float tolerance_xy) {
    // 闁告娲戠紞鍛姜椤掍礁搴?
    float current_x_m = robot_pose_state.pos_x_mm * 0.001f;
    float current_y_m = robot_pose_state.pos_y_mm * 0.001f;

    // 閻犱緤绱曢悾缁樻媴瀹ュ洨鏋傞悹鍥跺灠濡﹪鏁嶉崼銏ｅ剹闁?
    float error_x = position_control.target_x * 0.001f - current_x_m;
    float error_y = position_control.target_y * 0.001f - current_y_m;

    // 閻犱緤绱曢悾鑽ゆ崉濠靛牜鐎查悹鍥跺灠濡﹪鏁嶉崼銏ｅ剹闁?
    float distance_error = sqrtf(error_x * error_x + error_y * error_y);

    // 婵☆偀鍋撻柡灞诲劜濡叉悂宕ラ敃鈧﹢顏嗏偓鍦嚀濡﹪鎳犻崘銊︾函闁?
    if (distance_error <= tolerance_xy) {
        return 1;
    }

    return 0;
}

static uint8_t IsPositionControlFinished(float tolerance_xy)
{
    return IsPositionReached(tolerance_xy) && (position_plan_speed_mps <= POSITION_PLANNER_STOP_SPEED_MPS);
}

uint8_t testflag = 0;
// 闁哄洤鐡ㄩ弻濠冩媴瀹ュ洨鏋傞柟璨夊啫鐓戦柨娑樼墕瑜把囧箳瑜嶉崺姊?y濞达絽绉堕悿鍡涙晬?
void UpdatePositionControl(float dt) {
    if (!position_control.is_enabled || dt <= 0) {
        return;
    }

    // 闁兼儳鍢茶ぐ鍥亹閹惧啿顤呴柛瀣箺閸╁懐鎲撮幒鐐电鐎殿啫鍐唺闁?
    float current_yaw_rad = GetYaw() * 3.1415926f / 180.0f;

    // 闁兼儳鍢茶ぐ鍥亹閹惧啿顤呭ù锝呯Ф閻ゅ棝鏁嶉崼婵嗗伎閻忕偐鍋撻柛褎鍔栭悥锝囧寲娴兼瑧绀?
    float current_x_global = robot_pose_state.pos_x_mm * 0.001f;
    float current_y_global = robot_pose_state.pos_y_mm * 0.001f;
    float target_x_global = position_control.target_x * 0.001f;
    float target_y_global = position_control.target_y * 0.001f;

    // 閻犱緤绱曢悾濠氬礂閵娿儳婀伴柛褎鍔栭悥锝囧寲鐠佸磭鐟撻柣銊ュ椤曘倕顔?
    float error_x_global = target_x_global - current_x_global;
    float error_y_global = target_y_global - current_y_global;
    float distance_error = sqrtf(error_x_global * error_x_global + error_y_global * error_y_global);

    // 閻忓繐妫滈銈咁啅椤旇崵鐭ら柛蹇嬪妼閻剟宕搁幇顓犲灱缂侇垱妲掑ù鍡涘箲閵忕姴鐓傞悘蹇撶箺濠у懐浠﹂埀顒勬焾閵娿儲缍忛柡宥呮川闁?
    float error_x_local = error_x_global * cosf(current_yaw_rad) + error_y_global * sinf(current_yaw_rad);
    float error_y_local = -error_x_global * sinf(current_yaw_rad) + error_y_global * cosf(current_yaw_rad);

    // 濞达綀娉曢弫銈囦沪閳ь剟鏌堥妸銉︾稄闁哄秴娲ㄩ柈鎾儍閸曨噮鍤栫€瑰壊鍠涚换妯兼偘鐎涚拤D闁硅矇鍐ㄧ厬
    // 鐟欏嫬鍨濋崜宥夘洯閹貉冨煑闁喎瀹宠ぐ銏㈠Ц閿涘奔缍呯純顔煎冀妫ｅ牅鎱ㄥ锝呯杽闂勫懎浜稿?
    float direction_x = 0.0f;
    float direction_y = 0.0f;
    float error_norm = sqrtf(error_x_local * error_x_local + error_y_local * error_y_local);
    float planned_speed = PositionSpeedPlan_Update(distance_error, dt);

    if (error_norm > 0.001f) {
        direction_x = error_x_local / error_norm;
        direction_y = error_y_local / error_norm;
    }

    float feedforward_vx = direction_x * planned_speed;
    float feedforward_vy = direction_y * planned_speed;
    float feedback_vx = PID_Update(&position_control.pid_x, error_x_local, dt);
    float feedback_vy = PID_Update(&position_control.pid_y, error_y_local, dt);
    float feedback_speed = sqrtf(feedback_vx * feedback_vx + feedback_vy * feedback_vy);

    if (feedback_speed > POSITION_FEEDBACK_MAX_SPEED_MPS) {
        float feedback_ratio = POSITION_FEEDBACK_MAX_SPEED_MPS / feedback_speed;
        feedback_vx *= feedback_ratio;
        feedback_vy *= feedback_ratio;
    }

    float vx_local = feedforward_vx + feedback_vx;
    float vy_local = feedforward_vy + feedback_vy;
    // 闂傚嫭鍔曢崺妤呭嫉閳ь剚寰勮閳ь剛鍠庣€?
    #define MAX_LINEAR_SPEED 7.0f
    float linear_speed = sqrtf(vx_local * vx_local + vy_local * vy_local);
    if (linear_speed > MAX_LINEAR_SPEED) {
        float ratio = MAX_LINEAR_SPEED / linear_speed;
        vx_local *= ratio;
        vy_local *= ratio;
    }

    // 閻犱礁澧介悿鍡涙儎椤旂晫鍨奸梺顐ゅ枎鐎规娊鏁嶉崼婊冣枏闁活潿鍔岄惇顒勬焾閵娿儲缍忛柡宥呮川闁挳鎯冮崟顖楀亾閻斿嘲顔婇柨?
    vx_local *= pose_coordination.sync_scale;
    vy_local *= pose_coordination.sync_scale;
    SetTargetLinearVelocity(vy_local, vx_local);  // 濞达絽绉堕悿鍡涙偝閻ゆ寧娼€涙ɑ鐓欓柛姘灣缁楀本鎯旈弴鐘崇８婵☆垼浜滈幃婊堟焻閻斿嘲顔婇柡鍌滄嚀閹粍绋夐埀顒勬嚊?

    // 婵☆偀鍋撻柡灞诲劜濡叉悂宕ラ敃鈧崺灞炬綇閸撗勭獥闁哄秴娲ｇ紞鍛磾椤曞棛绀勫ù锝堟硶閺併倝宕楅妸銉ф拱闁秆勫姈閻栵絿鍖栨导娆戠
    if (IsPositionControlFinished(raceh)) {
        testflag = 1;
        PositionPlanner_Reset();
    }
}

void Example_PositionControl(void) {
    // 閻犱礁澧介悿鍡涙儎椤旂晫鍨煎ù锝呯Ф閻ゅ棝鏁嶅?1.0m, y=0.5m
    SetTargetPosition(1.0f, 0.5f);

    // 闁告凹鍨抽弫銈嗘媴瀹ュ洨鏋傞柟璨夊啫鐓?
    PositionControl_Enable(1);

    // 缂佹稑顦欢鐔煎礆閹峰本褰ч柣鈺婂枟閻栵絾鎷呭鍥╂瀭
    while (!IsPositionReached(0.02f)) {
        HAL_Delay(10);
    }

    // 闁告帗濯介幓顏堝触鎼淬垹鈷旈悶娑樿嫰閸欑偓绂掗弽锔藉床闁?..
}

//********************************濞达絽绉堕悿鍡涘箳瑜嶉崺姊d****************************************//
//********************************閻熸瑦甯掔€圭id****************************************//

// 濠殿喗瀵ч埀顑胯兌鑿欓悗瑙勭鐢爼宕氶崜浣烘尝闁哄瀚紞?
typedef struct {
    PID_Controller yaw_pid;      // 闁稿绻楅崺鍛喆閹碘問D闁硅矇鍐ㄧ厬闁?
    float target_yaw;             // 闁烩晩鍠楅悥锝夊磻韫囨艾鐒婚悷?閹?
    uint8_t is_enabled;           // 闁硅矇鍐ㄧ厬濞达綀鍎婚崗姗€寮介崶褏绠?
    uint32_t last_update_time;    // 闁哄牃鍋撻柛姘濞插潡寮悧鍫燁槯闂?
} AttitudeStabilization_t;

// 闁稿繈鍔岄惇顒佹叏閹稿簶鍋撴担鐟颁粯闁告帟娉涢悿鍕瑹?
AttitudeStabilization_t attitude_control;
float attitude_correction = 0.0f; // 濠殿喗瀵ч埀顑跨劍閻楀骸顫㈤敐澶婃
uint8_t isPosture_correction = 0;
// 闁兼儳鍢茶ぐ鍥ㄦ叏閹稿簶鍋撴担鍦ⅰ婵繐缍侀崳?
float GetAttitudeCorrection(void) {
    return attitude_correction;
}
// 濠殿喗瀵ч埀顑胯兌鑿欓悗瑙勮壘閸ㄥ灚鎱ㄧ€ｎ亜顕?
void AttitudeStabilization_Init(void) {
    // 闁告帗绻傞～鎰板礌閺嵮傜剨闁煎壊浜ｉ～妗篒D闁硅矇鍐ㄧ厬闁?
    // 闁告瑥鍊归弳鐔兼閳ь剛鎲版担鍦闁硅鍠栭悿鍕⒔閸涱喗绨氶柛锝冨妺濮瑰宕濋妸锔瑰亾娴ｇ晫娈堕柡?
    PID_Init(&attitude_control.yaw_pid, 0.26f, 0.0f, 0.0051f, 6.0f); // Kp, Ki, Kd, max_output

    isPosture_correction=0;//闁告帗绻傞～鎰▔瀹ュ懐纾婚柛姘煎灠鍗忛柟顑胯兌閻擃偄顫㈤敐蹇曠闂傚洠鍋撻悷鏇氳兌閺併倝寮崜浣烘瀭1

    attitude_control.target_yaw = 0.0f;
    attitude_control.is_enabled = 0;
    attitude_control.last_update_time = HAL_GetTick();
}

// 閻犱礁澧介悿鍡涙儎椤旂晫鍨奸柛瀣箺閸╁懐鎲?
void SetTargetYaw(float target) {
    if (fabsf(NormalizeYawError(attitude_control.target_yaw - target)) < 0.001f) {
        return;
    }
     // 缁绢収鍠曠换姘舵儎椤旂晫鍨奸悷娆愬笒鐎规娊宕烽埡?, 360)闁肩厧鍟ú鍧楀礃?
    attitude_control.target_yaw = target;

    // 闂佹彃绉堕悿鍝朓D缂佸鍨伴崹搴亜閻у摜绀夐梺顒€鐏濋崢銈囩玻娴ｇ缍?
    attitude_control.yaw_pid.integral = 0.0f;
    attitude_control.yaw_pid.prev_error = 0.0f;
}
// 婵☆偀鍋撻柡灞诲劜濡叉悂宕ラ敃鈧崺灞炬綇閸撗勭獥闁哄秴娲╅～妤佹償?
uint8_t IsYawReached(float target_yaw, float tolerance_degrees) {
    float current_yaw = GetYaw();
    float error = target_yaw - current_yaw;

    // 閻忓繐妫滈銈咁啅椤旇偐绉哄☉鎾亾闁告牗鐗曢崺瀛?180, 180)闁肩厧鍟ú?
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // 婵☆偀鍋撻柡灞诲劜濡叉悂宕ラ敃鈧﹢顏嗏偓鍦嚀濡﹪鎳犻崘銊︾函闁?
    return (fabsf(error) <= tolerance_degrees);
}
// 闁告凹鍨抽弫?缂佸倷鑳堕弫銈嗘叏閹稿簶鍋撴担鍛垫梻鈧?
void EnableAttitudeStabilization(uint8_t enable) {
    if (attitude_control.is_enabled == enable) {
        return;
    }
    attitude_control.is_enabled = enable;

    if (enable) {
        // 闁告凹鍨抽弫銈夊籍閺堥潧鈻忛柣顫妼缂嶅宕滃鍫健閹艰揪缂氱紞鏃€绋夐搹瑙勭獥闁?
        attitude_control.target_yaw = GetYaw();//GetYaw();
        attitude_control.last_update_time = HAL_GetTick();

        // 闂佹彃绉堕悿鍝朓D
        attitude_control.yaw_pid.integral = 0.0f;
        attitude_control.yaw_pid.prev_error = 0.0f;
    }
}


// 闁哄洤鐡ㄩ弻濠冩叏閹稿簶鍋撴担鍛垫梻鈧纰嶇敮鍫曞礆?
void UpdateAttitudeStabilization(float dt) {
    if (!attitude_control.is_enabled || dt <= 0) {
        attitude_correction = 0.0f; // 缂佸倷鑳堕弫銈夊籍閼哥數澧℃慨婵撶秮閸ｇ儤绋?
        return;
    }

    // 闁兼儳鍢茶ぐ鍥亹閹惧啿顤呴柛瀣箺閸╁懐鎲?
    float current_yaw = GetYaw();

    // 閻犱緤绱曢悾鑽ゆ喆閹烘垵顔婇悹鍥跺灠濡﹪鏁嶉崼锝傚亾閸愵厽顎氶悷娆愬笒鐎规娊鎮抽婊呮悘闁?
    float error = attitude_control.target_yaw - current_yaw;

    // 閻忓繐妫滈銈咁啅椤旇偐绉哄☉鎾亾闁告牗鐗曢崺瀛?180, 180)闁肩厧鍟ú?
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // 婵繆顕х亸顖涘緞閸曨厽鍊?
    if (abs_float(error) < DEAD_ZONE) {
        error = 0.0f;
        attitude_control.yaw_pid.integral = 0.0f; // 闂佹彃绉堕悿鍡欑矓椤栨艾鐎诲銈呯秺濡茶顫㈤姀鈶挎繈宕氶崱娑栧仚闁?
		attitude_correction = 0.0f;
        return;
    }

    // 濞达綀娉曢弫顥禝D閻犱緤绱曢悾濠氬籍鐎ｎ厽绁梺顐ゅ枎鐎规娊骞掕閸╂鏌?
    attitude_correction = PID_Update_angle(&attitude_control.yaw_pid, error, dt);
    attitude_correction *= pose_coordination.sync_scale;

//	if (IsYawReached(attitude_control.target_yaw,0.5f)) { // 婵☆偀鍋撻柡灞诲劜濡叉悂宕ラ敃鈧崺灞炬綇閸撗勭獥闁哄秴娲╅～妤佹償?
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
// 闁哄啫顑堝ù鍡涘礆閻楀牆鐦归悗瑙勪亢椤鎯旈敂鑲╃闂傚牏鍋ゅΟ鍡樼箙閻戝洨绀?
uint8_t RotateToYaw(float target_yaw, float tolerance) {


    if (!is_rotating) {
        // 鐎殿喒鍋撳┑顔碱儐濡棙娼?
        SetTargetYaw(target_yaw);
        is_rotating = 1;
        return 0;
    }

    // 婵☆偀鍋撻柡灞诲劜濡叉悂宕ラ敃鈧崺灞炬綇閸撗勭獥闁哄秴娲╅～妤佹償?
    if (IsYawReached(target_yaw, tolerance)) {
//        EnableAttitudeStabilization(0);
        is_rotating = 0;
        return 1; // 闁哄啫顑堝ù鍡欌偓鐟版湰閸?
    }

    return 0; // 闁哄啫顑堝ù鍡樼▔?
}
//*******************************閻熸瑦甯掔€圭id***************************************//
//********************************闁硅棄瀚崕姘緞鐎涙澧℃慨婵愭煐id************************************//
// 闁稿繈鍔岄惇顒勫箺閸曨偄鍓煎璺虹摠閻楀骸顫㈤敐鍛秮闂?
CamStabilization_t cam_stabilization = {0};
float cam_correction_x = 0.0f;
float cam_correction_y = 0.0f;
uint8_t is_cam_stabilizing = 0;
int Is_coordgot =0;
// 闁告帗绻傞～鎰板礌閺嶃劍鍟旈柛宥呯箰閵囨摨ID
void CamStabilization_Init(void) {
    // 闁告帗绻傞～鎰板礌閺堫喗娼喊顢疍: kp=0.8, ki=0.05, kd=0.2, 閺夊牊鎸搁崵顓㈡⒔閹邦剛鐣介崡?00
    PID_Init_cam(&cam_stabilization.pid_x, 0.26f,0.0000f, 0.0f, 0.002f, 20.0f);

    // 闁告帗绻傞～鎰板礌閺堢増娼喊顢疍: kp=0.7, ki=0.03, kd=0.15, 閺夊牊鎸搁崵顓㈡⒔閹邦剛鐣介崡?00,
    PID_Init_cam(&cam_stabilization.pid_y, 0.26f,0.0000f,0.0f, 0.002f, 20.0f);

    cam_stabilization.is_enabled = 0;
    cam_correction_x = 0.0f;
    cam_correction_y = 0.0f;
	lookmode_centrl = 0;
	is_cam_stabilizing = 0;
	Is_coordgot =0;
}

// 闁告凹鍨抽弫?缂佸倷鑳堕弫銈夊箺閸曨偄鍓煎璺虹摠閻楀骸顫?
void CamStabilization_Enable(uint8_t enable) {
    cam_stabilization.is_enabled = enable;

    IsLight = 1;
    if (!enable) {
        // 缂佸倷鑳堕弫銈夊籍閸洖娅㈢紓鍐惧枤婵悂骞€?
        cam_stabilization.pid_x.integral = 0.0f;
        cam_stabilization.pid_x.prev_error = 0.0f;
        cam_stabilization.pid_y.integral = 0.0f;
        cam_stabilization.pid_y.prev_error = 0.0f;
        cam_correction_x = 0.0f;
        cam_correction_y = 0.0f;
		IsLight = 0;
    }
}

// 闁哄洤鐡ㄩ弻濠囧箺閸曨偄鍓煎璺虹摠閻楀骸顫?
void UpdateCamStabilization(float x_err, float y_err, float dt) {
    if (!cam_stabilization.is_enabled || dt <= 0) {
        cam_correction_x = 0.0f;
        cam_correction_y = 0.0f;
        return;
    }

    // 閻庤鐭粻鐔奉潰鐠囨彃闅橀梻鍐ㄧ墕閳ь剛銆嬬槐?闁稿秴绻掔粈宀勬晬?
    #define CAM_DEAD_ZONE 0.0f

    // X閺夌偛鐡ㄩ鎾礌閸濆嫷妲遍柣?
    float actual_x_err = (abs_float (x_err) > CAM_DEAD_ZONE) ? x_err : 0.0f;

    // Y閺夌偛鐡ㄩ鎾礌閸濆嫷妲遍柣?
    float actual_y_err = (abs_float(y_err) > CAM_DEAD_ZONE) ? y_err : 0.0f;

    // 閻犱緤绱曢悾绫滈弶鐐茬摠閻楀骸顫?
    cam_correction_x = PID_Update_cam(&cam_stabilization.pid_x, actual_x_err, dt);

    // 閻犱緤绱曢悾绫濋弶鐐茬摠閻楀骸顫?
    cam_correction_y = PID_Update_cam(&cam_stabilization.pid_y, actual_y_err, dt);
}

static ErrorQueue_t error_queue = {0};

// 闁告帗绻傞～鎰板礌閺嶎剦鍤栫€瑰壊鍣ｅΣ锕傚礆?
void ErrorQueue_Init(void) {
    error_queue.index = 0;
    error_queue.count = 0;
    error_queue.is_full = 0;
    memset(error_queue.errors_x, 0, sizeof(error_queue.errors_x));
    memset(error_queue.errors_y, 0, sizeof(error_queue.errors_y));
}

// 闁告碍鍨奸銈咁啅椤曗偓濡诧箓宕氬Δ浣告綉闁告梻濮甸弳鐔煎箲?
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

// 閻犱緤绱曢悾鑽ゆ嫚椤栨碍鈻曢梻鍐枎閸亪鎯冮崟顐ｇ秵闁?
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
#define INT_MAX 480       //闁哄牃鍋撳鍫嗗啩鐒婄€瑰壊鍠氱划椋庘偓鐢垫嚀閳?
// 閻犱緤绱曢悾鑽ゆ嫚椤栨碍鈻曢梻鍐枎閸亝绋夐悺?y闁哄倻鎳撻幃婊堟儍閸曨剚浠樺鍫嗗嫬鐨鹃柛鏂诲妼缁犳瑦鎯旈敂鑲╃max - min闁?
void ErrorQueue_GetFluctuate(int *fluct_x, int *fluct_y) {
    int max_x = -INT_MAX, min_x = INT_MAX;
    int max_y = -INT_MAX, min_y = INT_MAX;
    uint8_t valid_count = error_queue.is_full ? ERROR_QUEUE_SIZE : error_queue.count;

    // 闁兼眹鍎靛Σ锕傚礆濡ゅ啠鏁勯柨娑樻湰鐏忔繈宕濋妸銉х暯閹艰揪缂氱拹?
    if (valid_count == 0) {
        *fluct_x = 0;
        *fluct_y = 0;
        return;
    }

    // 闂侇剙绉村濠氬嫉婢跺娅忛柡浣哄瀹撲線鏁嶇仦鎯ь棟max闁告粌顔卛n
    for (int i = 0; i < valid_count; i++) {
        // x闁哄倻鎳撻幃?
        if (error_queue.errors_x[i] > max_x) max_x = error_queue.errors_x[i];
        if (error_queue.errors_x[i] < min_x) min_x = error_queue.errors_x[i];
        // y闁哄倻鎳撻幃?
        if (error_queue.errors_y[i] > max_y) max_y = error_queue.errors_y[i];
        if (error_queue.errors_y[i] < min_y) min_y = error_queue.errors_y[i];
    }

    // 婵炲鍨规慨鈺呯嵁閸涱厼顔?= 闁哄牃鍋撳鍫嗗啠鍋?- 闁哄牃鍋撻悘蹇撶箰閳?
    *fluct_x = max_x - min_x;
    *fluct_y = max_y - min_y;
}

uint8_t IsCamReached(int tolerance_x, int tolerance_y,int MAX_Fluctuate_error) {
    int error_x = Camera_err.x;
    int error_y = Camera_err.y;

    // 閻忓繐妫楃紞瀣礈瀹ュ牜鍤栫€瑰壊鍠栨慨鐐哄礂閵夆晜袝闁?
    ErrorQueue_Push(error_x, error_y);

    // 閻犱緤绱曢悾濠氭⒓閻斿嘲鐏欏☉鎿冨幗婢у秹寮垫径搴殩鐎瑰壊鍠氬▓鎴﹀锤閸パ€鍋?
    float avg_error_x, avg_error_y;
	  int F_x,F_y;
    ErrorQueue_GetAverage(&avg_error_x, &avg_error_y);
    ErrorQueue_GetFluctuate(&F_x,&F_y);
    // 婵☆偀鍋撻柡灞诲劦濡诧箓宕氬Δ浣剐﹂柛姘剧畱閸戔€愁煥閳ヨ尙鐟柛褍娲ら埀顒傚帶濠€顏嗏偓鍦嚀濡﹪鎳犻崘銊︾函闁?
    if (error_queue.is_full && avg_error_x <= tolerance_x && avg_error_y <= tolerance_y&&F_x<=MAX_Fluctuate_error&&F_y<=MAX_Fluctuate_error) {
        return 1;
    }

    return 0;
}

// 缂佺姭鍋撻柛鏍ㄧ墱濞堟垿骞栧鍛亶缂佹稑顦欢鐔煎礄閼恒儲娈?- 闁告瑯浜滃﹢顏堝炊濞戞鍤嬮柡鍌滄嚀閹粎绮旂拠鎻捫楀☉鎾亾婵炲棌妲勭槐婵嬫儑鐎ｎ亜鐓傞柣鈺婂枟閻栵絿绮╃€ｎ亜绁弶鈺傛煥濞?
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
extern uint32_t camtim;
extern uint8_t camtim_button;
uint8_t iscamreached_safecount = 0;
// 闁硅棄瀚崕姘緞鐎涙澧℃慨婵撶到閸╁本绋夐鐐靛闁挎稑鐗撳顏堟⒓鐠囧樊鏁氶柨?
uint8_t CamStabilizationToCenter(int tolerance_x, int tolerance_y,int MAX_Fluctuate_error) {

    if (!is_cam_stabilizing) {
        // 鐎殿喒鍋撳┑顔碱儐閻楀骸顫?
        ErrorQueue_Init();				// 鐎殿喒鍋撳┑顔碱儐閻楀骸顫㈤敐鍡橆槯婵炴挸鎳愰埞鏍⒓閻斿嘲鐏?
        PositionControl_Enable(0);		//闁稿繐鐗嗛崣褔姊婚鐐寸稄闁哄秴娲﹂悧搴☆潰閿濆繒绀夌痪顓у枙缁绘岸宕搁幇顓犲灱闁哄秮鍓濋婊堝椽鐏炵偓鍟旈柛宥呯箰閵囨棃寮介埄鍐惧妧濞存粍甯楅弸?
		CamStabilization_Enable(1);
        is_cam_stabilizing = 1;
		IsLight = 1;
        return 0;
    }

    // 婵☆偀鍋撻柡灞诲劜濡叉悂宕ラ敃鈧崺灞炬綇閸撗勭獥闁?
    if (IsCamReached(tolerance_x, tolerance_y,MAX_Fluctuate_error)) {
		iscamreached_safecount ++;
		if(iscamreached_safecount>10){
			CamStabilization_Enable(0);
			is_cam_stabilizing = 0;
			IsLight = 0;
			return 1; // 闁哄秮鍓濋婊呪偓鐟版湰閸?
		}
    }

    return 0; // 闁哄秮鍓濋婊勭▔?
}
//********************************闁硅棄瀚崕姘緞鐎涙澧℃慨婵愭煐id************************************//
// 闁稿繈鍔岄惇顒勬儎椤旂晫鍨奸梺顐ゅ枎鐎规娊宕ｅ鈧崳?
TargetVelocity_t target_velocity = {0};

// 閻庤鐭粻?濞戞搩浜為弫鎼佸嫉閾忚鐣遍柛蹇嬪妼閻剟宕ｅ鈧崳?
Motor_t motor1 = {0, 0};
Motor_t motor2 = {0, 0};
Motor_t motor3 = {0, 0};
Motor_t motor4 = {0, 0};
Motor_t motor5 = {0, 0};
// 閹煎瓨娲滃ú蹇涘矗閸屾稒娈?
static ChassisConfig_t chassis_config;
static float k_omega;  // 闁哄啫顑堝ù鍡欏寲缂佹ɑ娈?k = (Lx + Ly)/2

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
// 闁告帗绻傞～鎰板礌閺嵮呬亢闁烩晜锚瀵剟寮?
void Mecanum_Init(ChassisConfig_t config) {
    chassis_config = config;
    k_omega = (config.wheel_dist_x + config.wheel_dist_y) / 2.0f;
}

// 閺夆晜鍔曟慨鈺冩喆閿濆洨鏆柡宥囶焾缁洪箖宕欓懞銉︽闁挎稑鐗呯粭澶屾暜閿曚緡娼￠幖杈剧畳钘熼柛瀣皡缁?
void Mecanum_Kinematics(float vx, float vy, float omega)
{
    float actual_omega = omega;

    // 濠碘€冲€归悘澶愭閳ь剛鎲版担宄扳枏闁活潿鍔屽崗闁诡兛鐒﹂悧搴☆潰?
    if (attitude_control.is_enabled) {
        actual_omega += attitude_correction;

        // 闂傚嫭鍔曢崺妤呭嫉閳ь剚寰勮濡棙娼鍕ㄥ亾閻斿嘲顔?
        #define MAX_OMEGA 20.0f
        if (actual_omega > MAX_OMEGA) actual_omega = MAX_OMEGA;
        if (actual_omega < -MAX_OMEGA) actual_omega = -MAX_OMEGA;
    }
	// 濠碘€冲€归悘澶愭閳ь剛鎲版担宄扳枏闁活潿鍔嶉幉姘跺磽韫囨挶浠堥柡宥佸墲椤?
    if (cam_stabilization.is_enabled) {
        vy += cam_correction_x * 0.01f;
		vx += cam_correction_y * 0.01f;
	}

    // 1. 閻犱緤绱曢悾濠氬触閸曨喚鏋傞柣銊ュ濞蹭即寮介崶鈺佹疇闂侇偆鍠庣€规娊鏁嶉崸?s闁?
    // 閺夌儐鍠栭悺娆愩亜閸濆嫮纰嶉柨娑欒壘娑斿繘宕?4) 闁告瑥鍟挎晶?2) 鐎归潻绠戦幃?3) 闁告瑥鍟块幃?1)
    float v4 =  vx - vy - k_omega * actual_omega;  // 鐎归潻绠戞晶?(4)
    float v2 =  vx + vy + k_omega * actual_omega;  // 闁告瑥鍟挎晶?(2)
    float v3 =  vx + vy - k_omega * actual_omega;  // 鐎归潻绠戦幃?(3)
    float v1 =  vx - vy + k_omega * actual_omega;  // 闁告瑥鍟块幃?(1)

    // 2. 閺夌儐鍓氬畷鍙夌▔?RPM闁挎稑婀㏄M = (缂佹儳娼￠埀顒傚枎鐎?/ (2閿?* 閺夌儐鍠栧畷鎰嚗?) * 60闁?
    const float linear_to_rpm = 60.0f / (2 * 3.1415926f * chassis_config.wheel_radius);
    float rpm4 = v4 * linear_to_rpm;
    float rpm2 = v2 * linear_to_rpm;
    float rpm3 = v3 * linear_to_rpm;
    float rpm1 = v1 * linear_to_rpm;

    // 3. 闁圭顦板〒鑸靛緞瑜戦悿鍡涙焻閻旇櫣鍩犲☉鎾亾婵絾鏌х欢銉х磽閳哄倹鏉归柨娑樺缁绘岸骞愭担鍛婄＝閺夌儐鍣ｉ埀顒傚枎鐎瑰啿袙閺傚墽浼?
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

    // 4. 閻犱礁澧介悿鍡涙偨閸偅绨氶柡鍌滄嚀閹?
    motor4.direction = (motor4.speed_rpm >= 0) ? 0 : 1;  // 鐎归潻绠戞晶?(4)
    motor2.direction = (motor2.speed_rpm >= 0) ? 1 : 0;  // 闁告瑥鍟挎晶?(2)
    motor3.direction = (motor3.speed_rpm >= 0) ? 0 : 1;  // 鐎归潻绠戦幃?(3)
    motor1.direction = (motor1.speed_rpm >= 0) ? 1 : 0;  // 闁告瑥鍟块幃?(1)
}

// 濞寸姰鍎扮粭鍛村及椤栨艾鏂ч柡鍫濐槺濞堟垶娼婚幇顒€袟闁硅矇鍐ㄧ厬闁告垼濮ら弳?
void stop(void) {
    SetTargetVelocity(0.0f, 0.0f, 0.0f);
}

static void MotionState_Reset(void)
{
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

//********************************鐎甸偊浜ｉ幎姊d闂侇喓鍔岄崹搴ㄦ晬閸喓姊鹃柣顫妼閸╁矂鏁?***********************************//
uint8_t IsRun = 0;
// PID闁硅矇鍐ㄧ厬闁革絻鍔岄崹鍨叏鐎ｎ亜顕?
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


// PID闁哄洤鐡ㄩ弻濠勬媼閿涘嫮鏆?
float PID_Update(PID_Controller* pid, float error, float dt) {
    // 婵絾鏌х欢銉︺亜?
    float proportional = pid->Kp * error;

    // 缂佸鍨伴崹搴亜閻у摜绀勯悽顖ら檮婵顨滈崡鐐村闁?
    pid->integral += error * dt;
    float integral = pid->Ki * pid->integral;

    // 鐎甸偊鍠栭崹搴亜?
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // 閻犱緤绱曢悾濠氬箑閺勫繒缈婚柛?
    float output = proportional + integral + derivative;

    // 闂傚嫭鍔曢崺妤佹綇閹惧啿姣夐柤鐓庡暙濞?
    if (output > pid->max_output) {
        output = pid->max_output;
        pid->integral -= error * dt; // 闁硅埖顨婇妶閬嶅椽鐏炵瓔妲遍柣?
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
        pid->integral -= error * dt; // 闁硅埖顨婇妶閬嶅椽鐏炵瓔妲遍柣?
    }

    return output;
}
float PID_Update_cam(PID_Controller* pid, float error, float dt) {
    // 婵絾鏌х欢銉︺亜?
    float proportional = pid->Kp * error;
	float proportional2 =pid->Kp2*error*fabsf(error);

    // 缂佸鍨伴崹搴亜閻у摜绀勯悽顖ら檮婵顨滈崡鐐村闁?
    pid->integral += error * dt;
    float integral = pid->Ki * pid->integral;

    // 鐎甸偊鍠栭崹搴亜?
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // 閻犱緤绱曢悾濠氬箑閺勫繒缈婚柛?

    float output = proportional +proportional2 + integral + derivative;

//	if(output>0){output +=0.4f;}
//	else if(output<0){output -=0.4f;}
//	else output =0;

    // 闂傚嫭鍔曢崺妤佹綇閹惧啿姣夐柤鐓庡暙濞?
    if (output > pid->max_output) {
        output = pid->max_output;
        pid->integral -= error * dt; // 闁硅埖顨婇妶閬嶅椽鐏炵瓔妲遍柣?
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
        pid->integral -= error * dt; // 闁硅埖顨婇妶閬嶅椽鐏炵瓔妲遍柣?
    }

    return output;
}
// PID闁哄洤鐡ㄩ弻濠勬媼閿涘嫮鏆?
float PID_Update_angle(PID_Controller* pid, float error, float dt) {
    // 婵絾鏌х欢銉︺亜?
    float proportional = pid->Kp * error;

    // 缂佸鍨伴崹搴亜閻у摜绀勯悽顖ら檮婵顨滈崡鐐村闁?
    pid->integral += error * dt;
    float integral = pid->Ki * pid->integral;

    // 鐎甸偊鍠栭崹搴亜?
    float derivative = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

	float G = robot_pose_state.yaw_gyro_dps*g;
    // 閻犱緤绱曢悾濠氬箑閺勫繒缈婚柛?
    float output = proportional + integral + derivative;

    // 闂傚嫭鍔曢崺妤佹綇閹惧啿姣夐柤鐓庡暙濞?
    if (output > pid->max_output) {
        output = pid->max_output;
        pid->integral -= error * dt; // 闁硅埖顨婇妶閬嶅椽鐏炵瓔妲遍柣?
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
        pid->integral -= error * dt; // 闁硅埖顨婇妶閬嶅椽鐏炵瓔妲遍柣?
    }

    return output;
}
// 闁稿繈鍔岄惇鐞岻D闁硅矇鍐ㄧ厬闁革絻鍔岄悿鍕瑹?
static PID_Controller line_follow_pid;
uint8_t is_line_follow_enabled = 0; // 闁哄倹婢橀·鍐晬濮橆剚鍎曢弶鈺呪偓娑樷枏闁煎疇濮ら悥锝堢疀?
// 鐎甸偊浜ｉ幎妤呭礆濠靛棭娼楅柛鏍ㄧ墪閸ら亶寮?
void Line_Follow_Init(void) {
    // 闁告帗绻傞～鎰板礌閺堢嚐D闁告瑥鍊归弳鐔兼晬瀹€鍐濞存粍绋戝顒勫极閺夊灝璁查柤鍏呯矙濞撳墎鎲版担鍦闁硅鍠栭悿鍕⒔閸涙壆娈堕悹鍥ㄦ礉閻ㄧ喖寮?
    PID_Init(&line_follow_pid, 0.9f, 0.02f, 200.0f, 0.48f);  // Kp, Ki, Kd, max_output
	is_line_follow_enabled = 0; // 闁告帗绻傞～鎰版偐閼哥鍋撴担鐤缂佸倷鑳堕弫?
	IsRun = 0;
}
// 闁哄倹婢橀·鍐晬濮橆剚鍎曢弶鈺呪偓娑樷枏闁?缂佸倷鑳堕弫銈夊礄閼恒儲娈?
void Line_Follow_Enable(uint8_t enable) {
    is_line_follow_enabled = enable;

    if (!enable) {
        // 缂佸倷鑳堕弫銈夊籍閸洖娅㈢紓鍐敩ID闁绘鍩栭埀?
        line_follow_pid.integral = 0.0f;
        line_follow_pid.prev_error = 0.0f;
		stop();
    }
}
// 鐎甸偊浜ｉ幎妤呭箳瑜嶉崺妤呭礄閼恒儲娈?
void Line_Follow_Control(uint8_t* sensor_values, float base_speed, float dt) {
	// 濠碘€冲€归悘澶婎嚗椤忓洦濮夌紒鍌欒兌閺併倝鏁嶅畝鍕缂傚喚鍠氭慨鎼佸箑娴ｆ瓕瀚欓弶鈺傛煥濞?
    if (!is_line_follow_enabled) {
        line_follow_pid.integral = 0.0f;
        line_follow_pid.prev_error = 0.0f;

		return;
    }
	// 闁哄倹澹嗗▓鎴犳嫚椤栨碍鈻曢悹渚婄磿閻ｅ寮悷鎵 - 闁哄洦娼欏杈╂嫬閸愌嗗幀闊洤鍟╃槐鍫曞箛閻旈攱鐝?
    float error = 0.0f;

    // 濞戞搩鍘肩缓鐐閻樺啿濡抽柛锝冨妺缁鳖參宕?
    if(is_task1_active()){
		if( sensor_values[2] == 0 ||sensor_values[3] == 0) {
			// 鐟滅増鎸烽懙鎴ｇ疀閸愌呯倞闁规壆鍠庡▍鎺懳涢埀顒€霉鐎ｎ亜鐓傚娑欏灩閸ゅ酣寮拋鍦鐎殿喖鎼崺妤冩嫚椤栨碍鈻曠憸鐗堝浮濞?
			error = 0.0f;
			// 闁告艾鏈鍌炴煂瀹ュ洨鏋侾ID缂佸鍨伴崹搴亜閻у摜绀夐梺顒€鐏濋崢銈夊储閸℃钑夐悹鍥跺灠濡﹨銇愰崡鐐存儥
			line_follow_pid.integral = 0.0f;
			line_follow_pid.prev_error = 0.0f;
		}
		else {
			// 闁告瑯浜濆﹢浣姐亹閹捐尪鍘煫鍥у暞閻ュ懘寮垫径瀣垫⒕婵炴潙顑呴崺宀勫籍鐠佸湱绀夐柟闈涚Х椤撳摜绮诲Δ鈧慨鐐哄级閸愵収鍤栫€?

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
				// 鐟滅増鎸烽懙鎴ｇ疀閸愌呯倞闁规壆鍠庡▍鎺懳涢埀顒€霉鐎ｎ亜鐓傚娑欏灩閸ゅ酣寮拋鍦鐎殿喖鎼崺妤冩嫚椤栨碍鈻曠憸鐗堝浮濞?
				error = 0.0f;
				// 闁告艾鏈鍌炴煂瀹ュ洨鏋侾ID缂佸鍨伴崹搴亜閻у摜绀夐梺顒€鐏濋崢銈夊储閸℃钑夐悹鍥跺灠濡﹨銇愰崡鐐存儥
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

    // 闁哄洤鐡ㄩ弻濂滻D闁硅矇鍐ㄧ厬闁?
    float omega = PID_Update(&line_follow_pid, error, dt);

    // 濞达綀娉曢弫銈嗘交閹邦剙袟閻庢冻闄勭敮鍫曞礆?
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

    if (dt <= 0) dt = 0.01f; // 闁哄牃鍋撻悘蹇撶箲濡炲倿姊婚幘顔斤紵闂?
    if(is_task1_active()){
		Line_Follow_Control(Anolog, 0.2376f, dt);}
	else{
		Line_Follow_Control(Anolog, 0.229f, dt);
	}
}

//********************************鐎甸偊浜ｉ幎姊d闂侇喓鍔岄崹?***********************************//

//**************************** 濞寸姾顕ф慨鐔烘嫬閸愩劌顔婇悗鍦仧楠?****************************//


//==================================================================
//闁?闁?闁告艾绋勭槐浼村礈瀹ュ懐绐旈柣鈺婂枟閻栵綁鎮?
//闁?   闁艰櫕鏋荤槐鎵媼閸撗呮瀭闁烩晩鍠楅悥锝夋倷閻熸媽瀚欏ù锝堝劵閸忔ɑ娼婚幇顒€袟闁挎稑鐬奸崝褔宕ユ惔鈥崇伈闁哄偆鍘藉Σ鎼佸触閿旀嫎鈺呭礉閵娿儳鏆氶柟?
//閺夊牊鎸搁崣鍡涘矗閸屾稒娈堕柨娑欐皑濞蹭即寮介崳姘舵晬鐎靛宕搁幇顓犲灱
//閺?闁?
uint8_t target_set = 0;
uint8_t move_to_target(float x, float y) {


    if (!is_position_control_enabled && !target_set) {
        SetTargetPosition(x, y);
        PositionControl_Enable(1);
        target_set = 1;
        return 0;
    }

    if (!is_position_control_enabled && target_set) {
        // 缂佸顕ф慨鈺冣偓鐟版湰閸?
        target_set = 0;
        return 1; // 缂佸顕ф慨鈺冣偓鐟版湰閸?
    }

    return 0; // 缂佸顕ф慨鈺傜▔?
}
uint8_t task1_relative_forward_back(void) {
    static uint8_t relative_stage = 0;
    static uint8_t yaw_hold_set = 0;
    static uint32_t switch_pause_start = 0;
    static float base_x_mm = 0.0f;
    static float base_y_mm = 0.0f;

    if (relative_stage == 0) {
        if (!yaw_hold_set) {
            PoseCoordination_Enable(0);
            stop();
            attitude_correction = 0.0f;
            attitude_control.yaw_pid.integral = 0.0f;
            attitude_control.yaw_pid.prev_error = 0.0f;
            EnableAttitudeStabilization(1);
            SetTargetYaw(-90.0f);
            base_x_mm = robot_pose_state.pos_x_mm;
            base_y_mm = robot_pose_state.pos_y_mm;
            yaw_hold_set = 1;
        }
				pospid_reinit(TASK1_RELATIVE_SPEED, POSITION_PID_TASK1_RELATIVE_KP);
        if (move_to_pose_target(base_x_mm + TASK1_RELATIVE_X_MOVE_MM, base_y_mm, -90.0f)) {
            relative_stage = 1;
            switch_pause_start = HAL_GetTick();
            stop();
            PositionPlanner_Reset();
        }
        return 0;
    }

    if (relative_stage == 1) {
        stop();
        if ((HAL_GetTick() - switch_pause_start) < TASK1_RELATIVE_SWITCH_PAUSE_MS) {
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

// 闁告凹鍨版慨鈺傜鐠囨彃顫?
void start_task1(void) {
    task1_state.active = 1;
    task1_state.current_step = 1;
    task1_state.completed = 0;
    task1_state.step_start = HAL_GetTick();
}

uint8_t is_hasgetcolor = 0;
// 闂傚牏鍋ゅΟ鍡樼箙閻愮増宕查柛鏂哄墲婢х晫鎮?
void run_task1(void) {
    if (!task1_state.active) return;//闁哄牜浜欐繛鍥嚄閹存帗宕查柛鏂衡偓鍐茬仧閺夆晜鏌ㄥú?
    uint32_t current_time = HAL_GetTick();
    switch (task1_state.current_step) {
//		case 0:	EnableAttitudeStabilization(1);SetTargetYaw(0.0f);
//				Servo_Action_Group(1);pospid_reinit(LOWSPEED, POSITION_PID_DEFAULT_KP);
//				if (move_to_target(task1_targets[20].x, task1_targets[20].y)/*闁?*/) {task1_state.current_step++;}
//				if(IsPositionReached(0.10f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 2060);}
////				if(IsPositionReached(0.05f)){SetTargetPosition(task1_targets[0].x, task1_targets[0].y);task1_state.current_step++;}
//				break;
        case 1:	EnableAttitudeStabilization(1);SetTargetYaw(0.0f);Move_to_turnplate(1);
				Servo_Action_Group(1);
				tasknum = 1;
				pospid_reinit(LOWSPEED, POSITION_PID_DEFAULT_KP);
				if (move_to_pose_target(task1_targets[0].x, task1_targets[0].y, 0.0f)/*闁绘劙鈧稓鐧岀紓浣割嚟閻?*/) {Servo_Action_Group(2);task1_state.current_step++;}
				if(IsPositionReached(0.50f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 2060);}
				if(IsPositionReached(0.45f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1980);}if(IsPositionReached(0.40f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1930);}
				if(IsPositionReached(0.35f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1900);}if(IsPositionReached(0.30f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1880);}
				if(IsPositionReached(0.25f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1860);}if(IsPositionReached(0.20f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1840);}
				if(IsPositionReached(0.15f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1820);}if(IsPositionReached(0.10f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1810);}
				if(IsPositionReached(0.04f)){__HAL_TIM_SetCompare(&htim14, TIM_CHANNEL_1, 1800);}
				break;
        case 2:Servo_Action_Group(2);PositionControl_Enable(0);EnableAttitudeStabilization(0);
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
				HAL_Delay(1000);Get_Materialcolor(3);__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Material[3]+200);
				MotionState_Reset();
				HAL_Delay(300);
				task1_state.current_step =11;
				break;


		case 11:EnableAttitudeStabilization(1);
				pospid_reinit(LOWSPEED, POSITION_PID_DEFAULT_KP);Get_Materialcolor(4);
				if (move_to_pose_target(task1_targets[6].x, task1_targets[6].y, -90.0f)/*缂佸顕ч崺宀**/ ) {
										Find_corresponding_material(1);
										task1_state.current_step++;}break;
		case 12:if(Wait_for_uart_with_search(6)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 閻℃帒鎳忓鍌涘緞閸曨厽鍊為柨娑樻湰閻︻喗淇婇崒鐐叉閻犲洦娲橀崹銊╂煥濞嗘帩鍤栧璺哄閹?*/ } break;
		case 13:if(task1_relative_forward_back()){task1_state.current_step++;}break;
		case 14:pospid_reinit(MIDSPEED, POSITION_PID_DEFAULT_KP);if (move_to_pose_target(task1_targets[7].x, task1_targets[7].y, -90.0f)/*缂佸顕ч崺宀勬倷缁?*/) {
					Find_corresponding_material(2);
					task1_state.current_step++;}break;
		case 15:if(Wait_for_uart_with_search(7)) { task1_state.current_step++; }
				else {task1_state.current_step++;/* 閻℃帒鎳忓鍌涘緞閸曨厽鍊為柨娑樻湰閻︻喗淇婇崒鐐叉閻犲洦娲橀崹銊╂煥濞嗘帩鍤栧璺哄閹?*/ } break;
		case 16:if(task1_relative_forward_back()){task1_state.current_step++;}break;
		case 17:if (move_to_pose_target(task1_targets[8].x, task1_targets[8].y, -90.0f)) {
					Find_corresponding_material(3);
					task1_state.current_step++;}break;
		case 18:if(Wait_for_uart_with_search(8)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 閻℃帒鎳忓鍌涘緞閸曨厽鍊為柨娑樻湰閻︻喗淇婇崒鐐叉閻犲洦娲橀崹銊╂煥濞嗘帩鍤栧璺哄閹?*/ } break;
		case 19:if(task1_relative_forward_back()){task1_state.current_step++;}break;
		case 20:if (move_to_pose_target(task1_targets[9].x, task1_targets[9].y, -90.0f)) {
					Find_corresponding_material(4);
					task1_state.current_step++;}break;
		case 21:if(Wait_for_uart_with_search(9)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 閻℃帒鎳忓鍌涘緞閸曨厽鍊為柨娑樻湰閻︻喗淇婇崒鐐叉閻犲洦娲橀崹銊╂煥濞嗘帩鍤栧璺哄閹?*/ } break;
		case 22:if(task1_relative_forward_back()){task1_state.current_step++;}break;
		case 23:if (move_to_pose_target(task1_targets[10].x, task1_targets[10].y, -90.0f)) {
					Find_corresponding_material(5);
					task1_state.current_step++;}break;
		case 24:if(Wait_for_uart_with_search(10)) { task1_state.current_step++; }
				else { task1_state.current_step++;/* 閻℃帒鎳忓鍌涘緞閸曨厽鍊為柨娑樻湰閻︻喗淇婇崒鐐叉閻犲洦娲橀崹銊╂煥濞嗘帩鍤栧璺哄閹?*/ } break;
		case 25:if(task1_relative_forward_back()){task1_state.current_step++;}break;
		case 26://Servo_Action_Group(2);EnableAttitudeStabilization(1);
				tasknum = 2;if (move_to_pose_target(task1_targets[11].x, task1_targets[11].y, 0.0f)/*闁绘劙鈧稓鐧岀紓浣割嚟閻?*/) {Move_to_turnplate(3);/*缂佸顕ч崺灞炬姜椤掑倹纾?*/task1_state.current_step++;}break;
		case 27:if (move_to_pose_target(task1_targets[19].x, task1_targets[19].y, -90.0f)/*濠靛倹鐗楀?濞戞挸顑夊?*/){task1_state.current_step++;}break;
		case 28:pospid_reinit(LOWESTSPEED, POSITION_PID_DEFAULT_KP);if (move_to_pose_target(task1_targets[12].x, task1_targets[12].y, -90.0f)/*濠靛倹鐗楀?*/){task1_state.current_step++;Move_to_turnplate(2);}break;
		case 29:if (move_to_pose_target(task1_targets[13].x, task1_targets[13].y, -90.0f)/*濠靛倹鐗楀?*/) {task1_state.current_step++;Move_to_turnplate(1);}break;
		case 30:if (move_to_pose_target(task1_targets[14].x, task1_targets[14].y, -90.0f)/*濠靛倹鐗楀?*/) {__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1, Material[1]+200);task1_state.current_step++;}	break;
		case 31:
				pospid_reinit(HIGHSPEED, POSITION_PID_DEFAULT_KP);
				if (move_to_pose_target(task1_targets[15].x, task1_targets[15].y+15.0f, -90.0f)/*濞存粍鑹鹃崯妤呭矗?*/) {Servo_Action_Group(6);Find_corresponding_material(2);task1_state.current_step++;break;}
				if(IsPositionReached(0.6f)){Servo_Action_Group(5);}break;
		case 32:if(Wait_for_uart_with_search(15)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 閻℃帒鎳忓鍌涘緞閸曨厽鍊為柨娑樻湰閻︻喗淇婇崒鐐叉閻犲洦娲橀崹銊╂煥濞嗘帩鍤栧璺哄閹?*/ } break;
		case 33:if(task1_relative_forward_back()){Servo_Action_Group(3);HAL_Delay(100);
						task1_state.current_step++;}break;
		case 34:if (move_to_pose_target(task1_targets[16].x, task1_targets[16].y+15.0f, -90.0f)/*闁告劗濮撮崯妤呭矗?*/) {Servo_Action_Group(4);Find_corresponding_material(1);task1_state.current_step++;}break;
		case 35:if(Wait_for_uart_with_search(16)) { task1_state.current_step++; }
				else { task1_state.current_step++;/* 閻℃帒鎳忓鍌涘緞閸曨厽鍊為柨娑樻湰閻︻喗淇婇崒鐐叉閻犲洦娲橀崹銊╂煥濞嗘帩鍤栧璺哄閹?*/ } break;
		case 36:if(task1_relative_forward_back()){Servo_Action_Group(3);
						task1_state.current_step++;}break;
		case 37:if (move_to_pose_target(task1_targets[17].x, task1_targets[17].y+25.0f, -90.0f)) {Servo_Action_Group(7);Find_corresponding_material(3);task1_state.current_step++;}break;
		case 38:if(Wait_for_uart_with_search(17)) { task1_state.current_step++; }
				else {task1_state.current_step++; /* 閻℃帒鎳忓鍌涘緞閸曨厽鍊為柨娑樻湰閻︻喗淇婇崒鐐叉閻犲洦娲橀崹銊╂煥濞嗘帩鍤栧璺哄閹?*/ } break;
		case 39:if(task1_relative_forward_back()){task1_state.current_step++;}break;
		case 40:pospid_reinit(HIGHESTSPEED, POSITION_PID_DEFAULT_KP);raceh = POSITION_RETURN_REACHED_TOLERANCE_M;
				if (move_to_pose_target(task1_targets[20].x, task1_targets[20].y, -90.0f)/*闁搞儳鍋涢?*/) {task1_state.current_step = 41;}break;
		case 41:raceh = POSITION_FINE_REACHED_TOLERANCE_M;if (move_to_pose_target(task1_targets[18].x, task1_targets[18].y, -90.0f)/*闁搞儳鍋涢?*/) {task1_state.current_step++;}break;
        default:
            task1_state.active = 0;
            task1_state.completed = 1;
            break;
    }
}
//闂傚牏鍋ゅΟ鍡樼箙閻愭彃鏅告繛?
//case 6:SetTargetYaw(-90.0f);if (move_to_target(task1_targets[6].x, task1_targets[6].y)/*闁绘劗顒?/) {
//										Pos_moveconctrl(1,100,6400);task1_state.step_start = current_time; task1_state.current_step++;}break;
//		case 7:if (current_time - task1_state.step_start >= 1000) {//鐎点倛鍩栧鍌涚▔閳ь剛绮?
//					Pos_moveconctrl(0,100,6400);task1_state.step_start = current_time;task1_state.current_step++;
//					}break;
//		case 8:if (current_time - task1_state.step_start >= 1000) {//鐎点倛鍩栧鍌涚▔閳ь剛绮?
//					if (move_to_target(task1_targets[7].x, task1_targets[7].y)/*闁绘劗顒?/) {task1_state.current_step++;}
//					}break;
// 婵☆偀鍋撻柡灞诲劙閹广垽宕濋埄鍐┬﹂柛姘鹃檮缁哄搫煤?
uint8_t is_task1_active(void) {
    return task1_state.active;
}
/***********Task1***************/

