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

#define PITCH_MAX -9
#define PITCH_MIN -41
#define YAW_MAX 45
#define YAW_MIN -45
#define OFFSET_PITCH_ANGLE 10.0f
#define OFFSET_YAW_ANGLE 122.2f

typedef struct{
	uint8_t canbus[2];   							
	int16_t angle_raw_value;
	float angle;
} GMEncoder_t;

extern GMEncoder_t GMYawEncoder;
extern GMEncoder_t GMPitchEncoder;
extern float pitch, yaw;
extern float pitch_now, yaw_now;
extern float pitch_out, yaw_out;
extern float pitch_edge[2];
extern float yaw_right;
extern struct Point pr;

void gimbal_task(void);
void depart_handle(void);
void cruise_handle(void);
void auto_handle(void);
void relax_handle(void);
void gimbal_action(void);
void gimbal_action_auto(void);
void stay_handle(void);
void calc_angle(float pitch_in, float yaw_in, struct Point pr);
void auto_shoot(void);
#endif
