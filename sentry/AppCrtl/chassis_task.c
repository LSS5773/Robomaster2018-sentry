#include "chassis_task.h"

// mm/s
#define V_MAX_CONTROLLER 844
#define SPEED_CRUISE     500
#define SPEED_MAX        850

// ms
#define DT_SWITCH	8000
#define DT_SAFE		5000

// mm
#define DDIS_MIN 800.0f
#define DDIS_MAX 1400.0f

chassis_t chassis;
CMEncoder_t CMEncoder[4];

uint16_t last_HP = 3000;
uint32_t tick_rand = 0;
uint32_t tick_stay = 0;
uint32_t tick_hit = 0;
uint32_t tick_first_hit = 0;
uint32_t tick_cruise = 0;
uint8_t needTarget = 1;
uint8_t hit = 0;
float speed = 0;
float dis = 0;
float disTarget = 0;
float edge;
float dDisMin = DDIS_MIN;
float dDisMax = DDIS_MAX;
int speed_cruise = SPEED_CRUISE;
int speed_max = SPEED_MAX;

void value_init() {
	rc.sw1 = RC_DN;
	rc.sw2 = RC_MI;
	rc.LH = rc.LV = rc.RH = rc.RV = 0;
	for(int i = 0; i < 4; i++) PID_struct_init(&pid_spd[i], POSITION_PID, 10000, 1000, 10.0f, 0, 0);
	for(int i = 0; i < 4; i++) CMEncoder[i].round = 0;
	chassis.vx = chassis.vw = 0;
}

void chassis_task(void){
	switch(Sentry_Mode.chassis){
		case CHASSIS_Auto_Mode:   _auto_mode_handle();   break;
		case CHASSIS_Depart_Mode: _depart_mode_handle(); break;
		default:;
	}
}

void _auto_mode_handle(void){
	if(remain_HP < last_HP) hit = 1;
	last_HP = remain_HP;
	
//	if(rc.sw2 == RC_DN) hit = 1; // for debug
	
	switch(Sentry_Mode.chassis_state) {
		case Rand:   auto_rand();   break;
		case Twist:  auto_twist();  break;
		case Follow: auto_follow(); break;
		case Cruise: auto_cruise(); break;
		default: chassis.vy = 0; chassis_action();
	}
}

void _depart_mode_handle(void){
	float RH = 0;
	if(fabs(rc.RH) >= 0.02) RH = rc.RH;
	chassis.vy = RH * V_MAX_CONTROLLER;
	chassis_action();
	
	if(rc.RV < -0.8) {
		CMEncoder[0].zero = CMEncoder[0].angle;
		Sing(Do1L);
	}
	dis = -(int)((CMEncoder[0].angle-CMEncoder[0].zero)*PI/180.0f * RADIUS);
	edge = dis;
}

void switch_to_follow_mode() {
	Sentry_Mode.chassis_state = Follow;
}
void switch_to_cruise_mode() {
	needTarget = 1;
	Sentry_Mode.chassis_state = Cruise;
}
void switch_to_rand_mode() {
	tick_first_hit = HAL_GetTick();
	needTarget = 1;
	Sentry_Mode.chassis_state = Rand;
}
void switch_to_twist_mode() {
	tick_first_hit = HAL_GetTick();
	tick_hit = HAL_GetTick();
	if(dis > 0.5f*edge) speed = -speed_max;
	else speed = speed_max;
	Sentry_Mode.chassis_state = Twist;
}
void set_edge() {
	if(edge < 0) {
		edge = -edge;
		dis = 0;
		CMEncoder[0].zero = CMEncoder[0].angle;
	}
}

void move() {
	if(!offline[eCM1]) dis = -(int)((CMEncoder[0].angle-CMEncoder[0].zero)*PI/180.0f * RADIUS);
	chassis.vy = speed;
	chassis_action();
}


void mecanum_calc(float vx, float vy, float vw, int16_t speed[]) {
  static float wheel_rpm_ratio;
  wheel_rpm_ratio = 60.0f/(PERIMETER*CHASSIS_DECELE_RATIO);
  
  int16_t wheel_rpm[4];
  float   max = 0;
  
  wheel_rpm[FR] = (-vx - vy) * wheel_rpm_ratio;
  wheel_rpm[FL] = ( vx - vy) * wheel_rpm_ratio;
  wheel_rpm[BL] = ( vx + vy) * wheel_rpm_ratio;
  wheel_rpm[BR] = (-vx + vy) * wheel_rpm_ratio;
	
  for (uint8_t i = 0; i < 4; i++)
  {
    if (abs(wheel_rpm[i]) > max)
      max = abs(wheel_rpm[i]);
  }
  if (max > MAX_WHEEL_RPM)
  {
    float rate = MAX_WHEEL_RPM / max;
    for (uint8_t i = 0; i < 4; i++)
      wheel_rpm[i] *= rate;
  }
  memcpy(speed, wheel_rpm, 4*sizeof(int16_t));
}

void chassis_action() {
	int i;
	mecanum_calc(chassis.vx, chassis.vy, chassis.vw, chassis.wheel_spd_ref);
	for(i=0; i<4; i++) {
		chassis.wheel_spd_fdb[i] = CMEncoder[i].velocity;
		chassis.current[i] = pid_calc(&pid_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
	}
	uint8_t TxMessage[8];
	TxMessage[0] = (unsigned char)0;
	TxMessage[1] = (unsigned char)0;
	TxMessage[2] = (unsigned char)0;
	TxMessage[3] = (unsigned char)0;
	TxMessage[4] = (unsigned char)(chassis.current[FL] >> 8);
	TxMessage[5] = (unsigned char)chassis.current[FL];
	TxMessage[6] = (unsigned char)(chassis.current[FR] >> 8);
	TxMessage[7] = (unsigned char)chassis.current[FR];
	CAN_Send_Msg(&hcan2, TxMessage, 0x1ff , 8);
}

void set_target_distance(int velocity) {
	int r;
	double dDis;	
	if(dDisMax > edge) dDisMax = 0.7f*edge;
	if(dDisMin >= dDisMax) dDisMin = 0.5f*dDisMax;
	r = rand()%11;
	dDis = dDisMin + 0.1*r*(dDisMax-dDisMin);
	if(rand()%2 == 0) dDis *= -1;
	if(dis+dDis < 0 || dis+dDis > edge) dDis *= -1;
	
	disTarget = dis+dDis;
	if(disTarget < 0) disTarget = 0;
	else if(disTarget > edge) disTarget = edge;
	if(dis > disTarget) speed = -velocity;
	else speed = velocity;
}

void auto_rand() {
	if(hit) {
		tick_hit = HAL_GetTick();
		srand(tick_rand);
		if(needTarget) {
			set_target_distance(speed_max);
			needTarget = 0;
		} else {
			if(speed > 0) {
				if(dis >= disTarget) {
					needTarget = 1;
					hit = 0;
				}
			} else {
				if(dis <= disTarget) {
					needTarget = 1;
					hit = 0;
				}
			}
		}
		move();
		if(HAL_GetTick()-tick_first_hit > DT_SWITCH) switch_to_twist_mode();
	} else {
		needTarget = 1;
		if(HAL_GetTick()-tick_hit > DT_SAFE) switch_to_cruise_mode();
		else hit = 1;
	}
}
void auto_cruise() {
	srand(tick_rand);
	if(needTarget) {
		set_target_distance(speed_cruise);
		needTarget = 0;
	} else {
		if(speed > 0) {
			if(dis >= disTarget) needTarget = 1;
		} else {
			if(dis <= disTarget) needTarget = 1;
		}
	}
	move();
//	if(Sentry_Mode.gimbal_state == Aiming) switch_to_follow_mode();
	if(hit) switch_to_twist_mode();
}
void auto_twist() {
	if(abs(speed) != speed_max) {
		if(dis >= edge) speed = -speed_max;
		else if(dis <= 0) speed = speed_max;
		else {
			if(rand()%2) speed = speed_max;
			else speed = -speed_max;
		}
	}
	
	if(dis >= edge) speed = -speed_max;
	else if(dis <= 0) speed = speed_max;
	move();
	
	if(hit) {
		hit = 0;
		tick_hit = HAL_GetTick();
		if(HAL_GetTick()-tick_first_hit > DT_SWITCH) switch_to_rand_mode();
	}
	if(HAL_GetTick()-tick_hit > DT_SAFE) switch_to_cruise_mode();
}
void auto_follow() {
	static const float angle_area = 30;
	float k = 20;
	float yaw_feedback = GMYawEncoder.angle;
	if(yaw_feedback >= 0 && yaw_feedback < 90) yaw_feedback += 180;
	else if(yaw_feedback > 270) yaw_feedback -= 180;
	if(GMYawEncoder.angle >= 90 && GMYawEncoder.angle <= 270) k = -k;
	if(yaw_feedback > 180+angle_area) speed = k*(180+angle_area-yaw_feedback);
	else if(yaw_feedback < 180-angle_area) speed = k*(180-angle_area-yaw_feedback);
	else speed = 0;
	
	if(speed > speed_max) speed = speed_max;
	else if(speed < -speed_max) speed = -speed_max;	
	if(dis <= 0 && speed < 0) speed = 0;
	if(dis >= edge && speed > 0) speed = 0;
	move();
	if(Sentry_Mode.gimbal_state == NoTarget) switch_to_cruise_mode();
	if(hit) switch_to_rand_mode();
}
