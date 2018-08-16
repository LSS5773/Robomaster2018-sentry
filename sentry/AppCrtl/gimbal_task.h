#ifndef _GIMBAL_TASK_H_
#define _GIMBAL_TASK_H_

#include "stdint.h"
#include "pid.h"
#include "filter.h"
#include "control_task.h"
#include "bsp_imu.h"
#include "bsp_can.h"
#include "imu_task.h"
#include "matrix.h"

#define PITCH_MAX 0
#define PITCH_MIN -28
#define OFFSET_PITCH_ANGLE 54.0f
#define OFFSET_YAW_ANGLE 52.2f

typedef struct{
	uint8_t canbus[2];   							
	int16_t angle_raw_value;
	float angle;
} GMEncoder_t;

extern GMEncoder_t GMYawEncoder;
extern GMEncoder_t GMPitchEncoder;
extern float pitch, yaw;
extern float pitch_out, yaw_out;
extern struct Point pr;
extern uint8_t isHero;

void gimbal_task(void);
void depart_handle(void);
void cruise_handle(void);
void auto_handle(void);
void relax_handle(void);
void gimbal_action(void);
void gimbal_action_auto(void);
void calc_angle(float pitch_in, struct Point pr);
void auto_shoot(void);
#endif
