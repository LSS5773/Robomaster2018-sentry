#include "gimbal_task.h"

#define PITCH_INIT_ANGLE -18.0f
#define DIS_MIN 850.0f
#define DIS_MAX 4000.0f
#define SIZE_OFFSET_Y 4
#define OFFSET_Y_VALUE {-90.0f, -142.1f, -190.0f, -300.0f}
float dis_th[SIZE_OFFSET_Y+1] = {DIS_MIN, 1300, 2700, 3400, DIS_MAX};
float pitch = PITCH_INIT_ANGLE;
float yaw = 0;
float pitch_out = 0;
float yaw_out = 0;
struct Point pr;
float w_yaw[3];
uint8_t isHero = 0;
uint8_t ignoreLock = 0;
uint32_t tick_lock = 0;
uint32_t tick_out = 0;
uint32_t count_out = 0;
GMEncoder_t GMYawEncoder;
GMEncoder_t GMPitchEncoder;

void gimbal_task(void) {
	if((HAL_GetTick()-tick_controller) > TIMEOUT) {
		relax_handle();
		pitch = GMPitchEncoder.angle;
		yaw = abs_yaw;
		return;
	}
	switch(Sentry_Mode.gimbal) {
		case GIMBAL_Depart_Mode: depart_handle(); break;
		case GIMBAL_Cruise_Mode: cruise_handle(); break;
		case GIMBAL_Auto_Mode:
		case GIMBAL_Test_Mode:   auto_handle();   break;
		default:                 relax_handle();  break;
	}
}


void depart_handle(void) {
	Sentry_Mode.gimbal_state = NoTarget;
	pitch += 0.1f * rc.LV;
	yaw += -0.12f * rc.LH;
	gimbal_action();
}

void cruise_handle(void) {
	float dangle = 0.05f;
	if(Sentry_Mode.chassis_state != Cruise) dangle *= 2.0f;
	yaw += dangle;
	gimbal_action();
}

void auto_handle(void) {
	if(Sentry_Mode.gimbal == GIMBAL_Auto_Mode) {
		if(tick_wait < TIME_WAIT) {
			Sentry_Mode.gimbal_state = NoTarget;
			cruise_handle();
			return;
		}
	}
	
	const uint32_t COUNT_MAX = 10000;
	if(GMPitchEncoder.angle > -1) count_out++;
	else count_out = 0;
	if(count_out > COUNT_MAX) tick_lock = HAL_GetTick();
	const uint32_t TIME_LOCK = 5000;
	if(HAL_GetTick()-tick_lock < TIME_LOCK) {
		pitch = PITCH_INIT_ANGLE;
		cruise_handle();
		Sentry_Mode.gimbal_state = NoTarget;
		return;
	}
	
	uint8_t ignore = 0;
	if(HAL_GetTick()-tick[eTX2] > 100) ignore = 1;
	if(pr.z < DIS_MIN || pr.z > DIS_MAX) ignore = 1;
	if(isHero && GMYawEncoder.angle > 90 && GMYawEncoder.angle < 270) ignore = 1;	
	if(!ignore) {		
		gimbal_action_auto();
		Sentry_Mode.gimbal_state = Aiming;
		tick_notarget = HAL_GetTick();
	} else {
		Sentry_Mode.gimbal_state = NoTarget;
		pitch = PITCH_INIT_ANGLE;
		cruise_handle();
	}
	if(Sentry_Mode.gimbal == GIMBAL_Auto_Mode) auto_shoot();
}

void auto_shoot(void) {
	Sentry_Mode.fire = 0;
	if(Sentry_Mode.gimbal_state != Aiming) return;
	float dis_shoot = 300;
	if(isHero) dis_shoot *= 2.0f;
	if(fabs(pr.x) > dis_shoot) return;
	Sentry_Mode.fire = 1;
}

void relax_handle(void) {
	Set_Gimbal_Current(&hcan1, 0, 0);
}


void gimbal_action() {
	if(pitch > PITCH_MAX) pitch = PITCH_MAX;
	else if(pitch < PITCH_MIN) pitch = PITCH_MIN;
	
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
	GMYPositionPID.fdb = abs_yaw;
	PID_Calc(&GMYPositionPID);
	
	GMYSpeedPID.ref = GMYPositionPID.output;
	GMYSpeedPID.fdb = MPUz50Hz.raw_value;
	PID_Calc(&GMYSpeedPID);	
	
	Set_Gimbal_Current(&hcan1, GMYSpeedPID.output, -GMPSpeedPID.output);
}

void gimbal_action_auto() {	
	static float k_p = 0.0001f;
	static float offset_y_value[SIZE_OFFSET_Y] = OFFSET_Y_VALUE;
	float offset_y = 0;
	for(int i=0; i<SIZE_OFFSET_Y; i++) {
		if(pr.z >= dis_th[i] && pr.z <= dis_th[i+1]) {
			offset_y_value[i] += -0.1f * rc.LV;
			offset_y = offset_y_value[i];
			break;
		}
	}
	
	pitch += -k_p * (pr.y + offset_y);
	yaw += -0.004f * yaw_out;
	
	if(pitch > PITCH_MAX) pitch = PITCH_MAX;
	else if(pitch < PITCH_MIN) pitch = PITCH_MIN;
	
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
	
	GMYPositionPID.ref = yaw;
	GMYPositionPID.fdb = abs_yaw;
	PID_Calc(&GMYPositionPID);
	
	GMYSpeedPID.ref = GMYPositionPID.output;
	GMYSpeedPID.fdb = MPUz50Hz.filtered_value;
	PID_Calc(&GMYSpeedPID);	
	
	Set_Gimbal_Current(&hcan1, GMYSpeedPID.output, -GMPSpeedPID.output);
	
	for(int i=0; i<3-1; i++) w_yaw[i+1] = w_yaw[i];
	w_yaw[0] = MPUz50Hz.filtered_value;
}

void calc_angle(float pitch_in, struct Point pr) {
	float tx = 0;
	float ty = 44.6;
	float tz = 192.85;
	float offset_x = 82;
	float offset_z = 140;
	float htm[4][4];
	matrix_Identity(htm);
	trans(htm, -offset_x, 0, -offset_z);
//	rotate_x(htm, pitch_in);
	trans(htm, tx, ty, tz);
	struct Point pw;
	pw = multiply(htm, pr);
		
	static float pw_offset = 24.731f;
	pw_offset += 0.5f * rc.LH;
	pw.x += pw_offset;
	
	struct Point pw_xz;
	pw_xz.x = pw.x;
	pw_xz.y = 0;
	pw_xz.z = pw.z;
	if(pw.z < 0.1f || norm(pw) < 0.1f || norm(pw_xz) < 0.1f) return;
	float angle_pw_xz = atan(pw_xz.x/pw_xz.z);
	float angle_triangle = acos(offset_x/norm(pw_xz));
	yaw_out = PI/2.0f-(angle_triangle-angle_pw_xz);
	
//	struct Point vx, vz, vector;
//	vx.y = 0;
//	vx.x = -offset_x*cos(yaw_out);
//	vx.z = offset_x*sin(yaw_out);
//	vz.y = 0;
//	vz.x = -offset_z*sin(yaw_out);
//	vz.z = -offset_z*cos(yaw_out);
//	vector.x = pw.x - (vx.x + vz.x);
//	vector.y = pw.y - (vx.y + vz.y);
//	vector.z = pw.z - (vx.z + vz.z);
//	if(norm(vector) < 0.1) return;
//	pitch_out = -asin(fabs(pw.y)/norm(vector));
//	pitch_out *= 180/3.1415926f;

	yaw_out *= 180/3.1415926f;
}

