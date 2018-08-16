#include "gimbal_task.h"

float pitch = PITCH_MAX;
float yaw = 0;
float pitch_now = 0;
float yaw_now = 0;
float pitch_out = 0;
float yaw_out = 0;
struct Point pr;

float pitch_edge[2];
float yaw_right;

GMEncoder_t GMYawEncoder;
GMEncoder_t GMPitchEncoder;

void gimbal_task(void) {
	switch(INFANTRY_Mode.gimbal) {
		case GIMBAL_Depart_Mode:
			depart_handle();
			break;
		case GIMBAL_Cruise_Mode:
			cruise_handle();
			break;
		case GIMBAL_Auto_Mode:
		case GIMBAL_Test_Mode:
			auto_handle();
			break;
		case GIMBAL_Stay_Mode:
			stay_handle();
			break;
		default:
			relax_handle();
	}
}


void stay_handle(void) {
	pitch += 0.1f * rc.LV;
	yaw = yaw_right;
	gimbal_action();
}

void depart_handle(void) {
	INFANTRY_Mode.gimbal_state = NoTarget;
	pitch += 0.1f * rc.LV;
	yaw += 0.12f * rc.LH;
	gimbal_action();
}

void cruise_handle(void) {
	static uint8_t dir = 0;
	float dangle = 0.02f;
	float p_cruise_max = pitch_edge[0]>pitch_edge[1]?pitch_edge[0]:pitch_edge[1];
	float p_cruise_min = pitch_edge[0]>pitch_edge[1]?pitch_edge[1]:pitch_edge[0];
	if(pitch >= p_cruise_max) dir = 0;
	if(pitch <= p_cruise_min) dir = 1;
	if(dir) pitch += dangle;
	else pitch -= dangle;
	gimbal_action();
}

void auto_handle(void) {
	if(INFANTRY_Mode.gimbal == GIMBAL_Auto_Mode) {
		if(time_tick_wait < TIME_WAIT) {
			INFANTRY_Mode.gimbal_state = NoTarget;
			cruise_handle();
			return;
		}
	}
	if((HAL_GetTick()-tick_TX2) < 100) {
		if(rc.sw1 == RC_UP) {
			static uint8_t RC_STATE = RC_MI;
			if(rc.sw2 == RC_STATE) {
				pitch = pitch_out;
				yaw = yaw_out;
				if(RC_STATE == RC_DN) RC_STATE = RC_MI;
				else RC_STATE = RC_DN;
			}
		} else {
			pitch = pitch_out;
			yaw = yaw_out;
		}
		gimbal_action_auto();
		INFANTRY_Mode.gimbal_state = Aiming;
	} else {
		INFANTRY_Mode.gimbal_state = NoTarget;
		gimbal_action();
	}
	if(INFANTRY_Mode.gimbal == GIMBAL_Auto_Mode) auto_shoot();
}

void auto_shoot(void) {
	INFANTRY_Mode.fire = 0;
	if(INFANTRY_Mode.gimbal_state == Aiming) INFANTRY_Mode.fire = 1;
}

void relax_handle(void) {
	Set_Gimbal_Current(&hcan1, 0, 0);
}


void gimbal_action_auto() {
	if(pitch > PITCH_MAX) pitch = PITCH_MAX;
	else if(pitch < PITCH_MIN) pitch = PITCH_MIN;
	if(yaw > YAW_MAX) yaw = YAW_MAX;
	else if(yaw < YAW_MIN) yaw = YAW_MIN;
	
	//PITCH
	MPUx50Hz.raw_value = imu_data.gx * 0.061037019f;
	Chebyshev50HzLPF(&MPUx50Hz);
	
	AutoPitchPID.ref = pitch;
	AutoPitchPID.fdb = GMPitchEncoder.angle;
	PID_Calc(&AutoPitchPID);
	
	GMPSpeedPID.ref = AutoPitchPID.output;
	GMPSpeedPID.fdb = -MPUx50Hz.filtered_value;
	PID_Calc(&GMPSpeedPID);
	
	//YAW
	MPUz50Hz.raw_value = imu_data.gz * 0.061037019f;
	Chebyshev50HzLPF(&MPUz50Hz);
	
	AutoYawPID.ref = yaw;
	AutoYawPID.fdb = GMYawEncoder.angle;
	PID_Calc(&AutoYawPID);
	
	GMYSpeedPID.ref = AutoYawPID.output;
	GMYSpeedPID.fdb = -MPUz50Hz.filtered_value;
	PID_Calc(&GMYSpeedPID);	
	
	Set_Gimbal_Current(&hcan1,-(int16_t)GMPSpeedPID.output , -(int16_t)GMYSpeedPID.output);
}


void gimbal_action() {
	if(pitch > PITCH_MAX) pitch = PITCH_MAX;
	else if(pitch < PITCH_MIN) pitch = PITCH_MIN;
	if(yaw > YAW_MAX) yaw = YAW_MAX;
	else if(yaw < YAW_MIN) yaw = YAW_MIN;
	
	//PITCH
	MPUx50Hz.raw_value = imu_data.gx * 0.061037019f;
	Chebyshev50HzLPF(&MPUx50Hz);
	
	GMPPositionPID.ref = pitch;
	GMPPositionPID.fdb = GMPitchEncoder.angle;
	PID_Calc(&GMPPositionPID);
	
	GMPSpeedPID.ref = GMPPositionPID.output;
	GMPSpeedPID.fdb = -MPUx50Hz.filtered_value;
	PID_Calc(&GMPSpeedPID);
	
	//YAW
	MPUz50Hz.raw_value = imu_data.gz * 0.061037019f;
	Chebyshev50HzLPF(&MPUz50Hz);

	GMYPositionPID.ref = yaw;
	GMYPositionPID.fdb = GMYawEncoder.angle;
	PID_Calc(&GMYPositionPID);
		
	GMYSpeedPID.ref = GMYPositionPID.output;
	GMYSpeedPID.fdb = -MPUz50Hz.filtered_value;
	PID_Calc(&GMYSpeedPID);	
	
	Set_Gimbal_Current(&hcan1,-(int16_t)GMPSpeedPID.output , -(int16_t)GMYSpeedPID.output);
}
void calc_angle(float pitch_in, float yaw_in, struct Point pr) {
	float tx = 0;
	float ty = 15;
	float tz = 160;
	float offset = 66;
	float htm[4][4];
	matrix_Identity(htm);
	rotate_y(htm, yaw_in);
	rotate_x(htm, pitch_in);
	trans(htm, tx, ty, tz);
	struct Point pw;
	pw = multiply(htm, pr);
	if(pw.z <= 0 || norm(pw) == 0) {
		//cout << "[error] 0\n";
		return;
	}
	struct Point py;
	py.x = py.z = 0;
	py.y = 1;
	float angle2 = acos(offset/norm(pw));
	float angle1 = acos(dot(pw,py)/norm(pw)/norm(py));
	pitch_out = -(angle2-angle1) *180/3.1415926f;
	yaw_out = atan(pw.x/pw.z) *180/3.1415926f;
	
	static float pitch_auto_offset = 4.0f;
	static float yaw_auto_offset = 3.0f;
	pitch_auto_offset += 0.1f * rc.LV;
	yaw_auto_offset += 0.12f * rc.LH;
	pitch_out += pitch_auto_offset;
	yaw_out += yaw_auto_offset;
}

