#ifndef _SHOOT_H_
#define _SHOOT_H_
#include "stm32f4xx_hal.h"
#include "bsp_can.h"
#include "tim.h"
#include "stdint.h"
#include "pid.h"
#include "control_task.h"
#include "string.h"
#include "judge_task.h"
#include "beep.h"

#define FRIC_HIGH   5500
#define FRIC_LOW    5000
#define FRIC_IDLING 1000

typedef struct {
	uint8_t canbus[2];
	int16_t velocity;
	int16_t angle_raw_value;
	int16_t angle_last;
	float angle_real;
} SHOOTEncoder_t;

void shoot_task(void);
void turn_on_friction_wheel(uint16_t spd);
void shoot_stop(void);

extern SHOOTEncoder_t SHOOTEncoder[3];
extern int fric_speed;

#endif

