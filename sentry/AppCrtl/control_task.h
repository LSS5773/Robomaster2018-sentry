#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "gimbal_task.h"
#include "shoot_task.h"
#include "chassis_task.h"
#include "beep.h"
#include "stm32f4xx_it.h"
#include "tim.h"
#include "main.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "judge_task.h"
#include "bsp_io.h"

#define TIMEOUT 200	// ms
#define TIME_WAIT 60000 // ms
//#define TIME_WAIT 500 // ms   for debug

#define TONE_MAX_SIZE 21   // Must larger than TICK_MAX_SIZE and EX_MAX_SIZE
#define TICK_MAX_SIZE 9    // Size of Tick_Type_t
typedef enum {
	eTX2,
	ePitch,
	eYaw,
	eFeed,
	eFric1,
	eFric2,
	eCM1,
	eCM2,
	eJudge,
} Tick_Type_t;

#define EX_MAX_SIZE 2
typedef enum {
	eFricLocked,
	eEdgeError,
} EX_Type_t;

typedef enum {
	Red_Armor,
	Blue_Armor,
} Aim_Armor_Color_t;

typedef enum {
	GIMBAL_Depart_Mode,
	GIMBAL_Cruise_Mode,
	GIMBAL_Auto_Mode,
	GIMBAL_Test_Mode,
} GIMBAL_Mode_t;

typedef enum {
	CHASSIS_Depart_Mode,
	CHASSIS_Auto_Mode,
} CHASSIS_Mode_t;

typedef enum {
	NoTarget,
	Aiming,
} AUTO_Gimbal_t;

typedef enum {
	Cruise,
	Twist,
	Rand,
	Follow,
} AUTO_Chassis_t;

typedef struct{
	uint8_t fire;
	GIMBAL_Mode_t gimbal;
	CHASSIS_Mode_t chassis;
	AUTO_Gimbal_t gimbal_state;
	AUTO_Chassis_t chassis_state;
} Sentry_Mode_t;


extern uint8_t exception[EX_MAX_SIZE];
extern uint8_t offline[TICK_MAX_SIZE];
extern uint32_t tick[TICK_MAX_SIZE];
extern uint32_t tick_wait;
extern uint32_t tick_FPS;
extern uint32_t tick_notarget;
extern Sentry_Mode_t Sentry_Mode;
extern unsigned char aim_armor_color;

void mode_switch(void);
void mode_init(void);
void Control_Task(void);

#endif
