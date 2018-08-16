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
#define TIME_WAIT 40000 // ms

#define TONE_MAX_SIZE 21   // Must larger than TICK_MAX_SIZE
#define TICK_MAX_SIZE 8    // Size of Tick_Type_t
typedef enum{
	ePitch,
	eYaw,
	eFeed,
	eFric1,
	eFric2,
	eCM1,
	eCM2,
	eJudge,
} Tick_Type_t;


typedef enum{
	Red_Armor,
	Blue_Armor,
} Aim_Armor_Color_t;

typedef enum{
	MAIN_Relax_Mode,
    MAIN_Debug_Mode,
    MAIN_Default_Mode,
    MAIN_Buff_Mode,
    MAIN_Auxi_Mode,
    MAIN_Shoot_Mode,
    MAIN_Move_Mode
} MAIN_Mode_t;

typedef enum{
    GIMBAL_Relax_Mode,
    GIMBAL_Stay_Mode,
    GIMBAL_Follow_Mode,
    GIMBAL_Depart_Mode,
    GIMBAL_Test_Mode,
    GIMBAL_Cruise_Mode,
    GIMBAL_Auto_Mode
} GIMBAL_Mode_t;

typedef enum{
	CHASSIS_Depart_Mode,
	CHASSIS_Cruise_Mode,
	CHASSIS_Auto_Mode,
	CHASSIS_Stay_Mode,
	CHASSIS_Move_Mode,
} CHASSIS_Mode_t;

typedef enum{
	Aiming,
	NoTarget,
	OutOfRange,
} AUTO_Gimbal_t;

typedef enum{
	Rand,
	Twist,
	Follow,
	Stay,
	Cruise,
} AUTO_Chassis_t;

typedef struct{
	MAIN_Mode_t main;
	uint8_t fire;
	GIMBAL_Mode_t gimbal;
	CHASSIS_Mode_t chassis;
	AUTO_Gimbal_t gimbal_state;
	AUTO_Chassis_t chassis_state;
} INFANTRY_Mode_t;


extern uint32_t tick[TICK_MAX_SIZE];
extern uint32_t time_tick_wait;
extern INFANTRY_Mode_t INFANTRY_Mode;
extern unsigned char aim_armor_color;


void refreshCurrentMode(void);
void mode_init(void);
void Control_Task(void);
void TIM6_DAC_IRQHandler(void);

#endif
