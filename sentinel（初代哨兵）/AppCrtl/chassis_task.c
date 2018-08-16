#include "chassis_task.h"

chassis_t chassis;
CMEncoder_t CMEncoder[4];

uint16_t last_HP = 3000;
uint32_t timeTick = 0;
uint32_t tick_stay = 0;
uint32_t tick_hit = 0;
uint8_t needTarget = 1;
uint8_t hit = 0;
float speed = SPEED_CRUISE;
float dis = 0;
float disTarget = 0;
float edge_cruise;
float edge_twist;
float dDisMin = 800.0f;
float dDisMax = 1200.0f;
extern uint16_t time_tick_1ms;

void value_init() {
	rc.sw1 = RC_DN;
	rc.sw2 = RC_MI;
	rc.LH = rc.LV = rc.RH = rc.RV = 0;
	for(int i = 0; i < 4; i++) PID_struct_init(&pid_spd[i], POSITION_PID, 10000, 1000, 10.0f, 0, 0);
	chassis.vx = chassis.vw = 0;
}

void chassis_task(void){
	switch(INFANTRY_Mode.chassis){
		case CHASSIS_Auto_Mode:
			_auto_mode_handle();
			break;
		case CHASSIS_Cruise_Mode:
			_cruise_mode_handle();
			break;
		case CHASSIS_Depart_Mode:
			_depart_mode_handle();
			break;
		case CHASSIS_Stay_Mode:
			_stay_mode_handle();
			break;
		case CHASSIS_Move_Mode:
			_move_mode_handle();
		default:;
	}
}

void _auto_mode_handle(void){
	if(remain_HP < last_HP) hit = 1;
	last_HP = remain_HP;
	switch(INFANTRY_Mode.chassis_state) {
		case Rand: auto_rand(); break;
		case Twist: auto_twist(); break;
		case Follow: auto_follow(); break;
		case Stay: auto_stay(); break;
		case Cruise: auto_cruise(); break;
		default:
			chassis.vy = 0;
			chassis_action();
	}
}

void _cruise_mode_handle(void){
	if(abs(speed) != SPEED_CRUISE) {
		if(rand()%2) speed = SPEED_CRUISE;
		else speed = -SPEED_CRUISE;
	}
	if(speed > 0) {
		if(dis >= 0) speed = -SPEED_CRUISE;
	} else {
		if(dis <= edge_cruise) speed = SPEED_CRUISE;
	}
	dis += speed * 0.001f*CHASSIS_PERIOD;
	chassis.vy = speed;
	chassis_action();
}

void _depart_mode_handle(void){
	dis = 0;
	chassis.vy = rc.RH * V_MAX_CONTROLLER;
	chassis_action();
}
void _stay_mode_handle(void) {
	chassis.vy = 0;
	chassis_action();
}
void _move_mode_handle(void) {
	float speed = -400;
	dis += speed * 0.001f*CHASSIS_PERIOD;
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
}
void set_target_distance() {
	int r;
	double dDis;
//	r = rand()%11;
//	speed = SPEED_MIN + 0.1*r*(SPEED_MAX-SPEED_MIN);
	r = rand()%2;
	if(r == 0) speed = SPEED_MAX;
	else speed = -SPEED_MAX;
	r = rand()%11;
	if(dDisMax > fabs(edge_twist)) dDisMax = 0.7f*edge_twist;
	if(dDisMin > dDisMax) dDisMin = 0.8f*dDisMax;
	dDis = dDisMin + 0.1*r*(dDisMax-dDisMin);
	if(rand()%6 == 0) {
		dDis *= -1;
		speed *= -1;
	}
	if(dis+dDis > 0 || dis+dDis < edge_twist) {
		dDis *= -1;
		speed *= -1;
	}
	disTarget = dis+dDis;
}

void auto_rand() {
	if(hit) {
		srand(timeTick);
		if(needTarget) {
			set_target_distance();
			needTarget = 0;
		} else {
			dis += speed * 0.001f*CHASSIS_PERIOD;
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
		chassis.vy = speed;
		chassis_action();
		if(HAL_GetTick()-tick_hit > DT_SWITCH_MODE) {
			tick_hit = HAL_GetTick();
			if(dis > 0.5f*edge_twist) speed = -SPEED_MAX;
			else speed = SPEED_MAX;
			INFANTRY_Mode.chassis_state = Twist;
		}
	} else {
		needTarget = 1;
		_cruise_mode_handle();
		if(HAL_GetTick()-tick_hit > DT_SAFE) INFANTRY_Mode.chassis_state = Cruise;
	}
}
void auto_cruise() {
	_cruise_mode_handle();
	if(INFANTRY_Mode.gimbal_state == Aiming) INFANTRY_Mode.chassis_state = Follow;
	if(hit) {
		tick_hit = HAL_GetTick();
		if(dis > 0.5f*edge_twist) speed = -SPEED_MAX;
		else speed = SPEED_MAX;
		INFANTRY_Mode.chassis_state = Twist;
	}
}
void auto_twist() {
	if((speed > 0 && dis >= 0) || (speed < 0 && dis <= edge_twist)) {
		speed = 0;
		hit = 0;
		tick_stay = HAL_GetTick();
		INFANTRY_Mode.chassis_state = Stay;
	}
	dis += speed * 0.001f*CHASSIS_PERIOD;
	chassis.vy = speed;
	chassis_action();
}
void auto_follow() { // stay
	chassis.vy = 0;
	chassis_action();
	if(INFANTRY_Mode.gimbal_state != Aiming) INFANTRY_Mode.chassis_state = Cruise;
	if(hit) {
		needTarget = 1;
		INFANTRY_Mode.chassis_state = Rand;
	}
}
void auto_stay() {
	chassis.vy = 0;
	chassis_action();
	if(HAL_GetTick()-tick_stay > DT_SAFE) INFANTRY_Mode.chassis_state = Cruise;
	if(hit) {
		if(dis >= 0) speed = -SPEED_MAX;
		else speed = SPEED_MAX;
		INFANTRY_Mode.chassis_state = Twist;
	}
}
