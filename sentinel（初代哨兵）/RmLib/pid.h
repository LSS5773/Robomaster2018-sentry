#ifndef __PID_H
#define __PID_H	 

#include "stm32f4xx.h"
#define		FLOATMAX	5000

#define		AUTO_PITCH_KP 15
#define		AUTO_PITCH_KI 0.05
#define		AUTO_PITCH_KD 0.0
#define		AUTO_PITCH_OUTPUTMAX 300

#define		AUTO_YAW_KP 15
#define		AUTO_YAW_KI 0.05
#define		AUTO_YAW_KD 0.1
#define		AUTO_YAW_OUTPUTMAX 400

#define		PITCH_POSITION_KP_DEFAULTS  25
#define		PITCH_POSITION_KI_DEFAULTS  0.0
#define		PITCH_POSITION_KD_DEFAULTS	0.0
#define		PITCH_POSITION_KPMAX	FLOATMAX
#define		PITCH_POSITION_KIMAX	FLOATMAX
#define		PITCH_POSITION_KDMAX	FLOATMAX
#define		PITCH_POSITION_OUTPUTMAX	300

#define		PITCH_SPEED_KP_DEFAULTS  8
#define		PITCH_SPEED_KI_DEFAULTS  0.1
#define		PITCH_SPEED_KD_DEFAULTS		0.0
#define		PITCH_SPEED_KPMAX	FLOATMAX
#define		PITCH_SPEED_KIMAX	FLOATMAX
#define		PITCH_SPEED_KDMAX	FLOATMAX
#define		PITCH_SPEED_OUTPUTMAX	5000

#define		YAW_POSITION_KP_DEFAULTS  25
#define		YAW_POSITION_KI_DEFAULTS  0.0
#define		YAW_POSITION_KD_DEFAULTS	0.0
#define		YAW_POSITION_KPMAX	FLOATMAX
#define		YAW_POSITION_KIMAX	FLOATMAX
#define		YAW_POSITION_KDMAX	FLOATMAX
#define		YAW_POSITION_OUTPUTMAX	400

#define		YAW_SPEED_KP_DEFAULTS  8
#define		YAW_SPEED_KI_DEFAULTS  0.1
#define		YAW_SPEED_KD_DEFAULTS		0.0
#define		YAW_SPEED_KPMAX	FLOATMAX
#define		YAW_SPEED_KIMAX	FLOATMAX
#define		YAW_SPEED_KDMAX	FLOATMAX
#define		YAW_SPEED_OUTPUTMAX	5000

#define		AbsYawPID_SPEED_KP_DEFAULTS  15
#define		AbsYawPID_SPEED_KI_DEFAULTS  0.1
#define		AbsYawPID_SPEED_KD_DEFAULTS		0.0
#define		AbsYawPID_SPEED_KPMAX	FLOATMAX
#define		AbsYawPID_SPEED_KIMAX	FLOATMAX
#define		AbsYawPID_SPEED_KDMAX	FLOATMAX
#define		AbsYawPID_SPEED_OUTPUTMAX	400

#define		AbsYawPID_POSITION_KP_DEFAULTS  8
#define		AbsYawPID_POSITION_KI_DEFAULTS  0.0
#define		AbsYawPID_POSITION_KD_DEFAULTS		0.0
#define		AbsYawPID_POSITION_KPMAX	FLOATMAX
#define		AbsYawPID_POSITION_KIMAX	FLOATMAX
#define		AbsYawPID_POSITION_KDMAX	FLOATMAX
#define		AbsYawPID_POSITION_OUTPUTMAX	5000

#define		FRIC1_SPEED_KP_DEFAULTS  30
#define		FRIC1_SPEED_KI_DEFAULTS  0.1
#define		FRIC1_SPEED_KD_DEFAULTS		0
#define		FRIC1_SPEED_KPMAX	FLOATMAX
#define		FRIC1_SPEED_KIMAX	FLOATMAX
#define		FRIC1_SPEED_KDMAX	FLOATMAX
#define		FRIC1_SPEED_OUTPUTMAX	14000

#define		FRIC2_SPEED_KP_DEFAULTS  30
#define		FRIC2_SPEED_KI_DEFAULTS  0.1
#define		FRIC2_SPEED_KD_DEFAULTS		0
#define		FRIC2_SPEED_KPMAX	FLOATMAX
#define		FRIC2_SPEED_KIMAX	FLOATMAX
#define		FRIC2_SPEED_KDMAX	FLOATMAX
#define		FRIC2_SPEED_OUTPUTMAX	14000

#define		FEED_POSITION_KP_DEFAULTS  25
#define		FEED_POSITION_KI_DEFAULTS  0.0
#define		FEED_POSITION_KD_DEFAULTS		0.0
#define		FEED_POSITION_KPMAX	FLOATMAX
#define		FEED_POSITION_KIMAX	FLOATMAX
#define		FEED_POSITION_KDMAX	FLOATMAX
#define		FEED_POSITION_OUTPUTMAX	10000

#define		FEED_SPEED_KP_DEFAULTS  12
#define		FEED_SPEED_KI_DEFAULTS  0.7
#define		FEED_SPEED_KD_DEFAULTS		0.0
#define		FEED_SPEED_KPMAX	FLOATMAX
#define		FEED_SPEED_KIMAX	FLOATMAX
#define		FEED_SPEED_KDMAX	FLOATMAX
#define		FEED_SPEED_OUTPUTMAX	10000

#define		TEMP_KP_DEFAULTS	10
#define		TEMP_KI_DEFAULTS  1
#define		TEMP_KD_DEFAULTS	0.0
#define		TEMP_KPMAX	100
#define		TEMP_KIMAX	100
#define		TEMP_KDMAX	100
#define		TEMP_OUTPUTMAX	200

typedef struct
{
	float ref;//输入：系统待调节量的给定值
	float fdb;//输入：系统待调节量的反馈值
	float inte;//积分值
	float err[2];
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;
	float componentKpMax;
	float componentKiMax;
	float componentKdMax;
	float output;
	float outputMax;
	//float kp_offset;
	//float ki_offset;
	//float kd_offset;
	//void (*Calc)(struct PID_Regulator_t *pid);//函数指针
	//void (*Reset)(struct PID_Regulator_t *pid);
}PID_Regulator_t;


extern PID_Regulator_t AutoPitchPID;
extern PID_Regulator_t AutoYawPID;

extern PID_Regulator_t GMPPositionPID;
extern PID_Regulator_t GMPSpeedPID;
extern PID_Regulator_t GMYPositionPID;
extern PID_Regulator_t GMYSpeedPID;
extern PID_Regulator_t AbsYawSpeedPID;
extern PID_Regulator_t AbsYawPositionPID;
extern PID_Regulator_t ShootFeedPPID;
extern PID_Regulator_t ShootFeedSPID;
extern PID_Regulator_t ShootFric1PID;
extern PID_Regulator_t ShootFric2PID;
extern PID_Regulator_t TempPID;

void PID_Init(PID_Regulator_t *pid,float kp,float ki,float kd,float componentKpMax,float componentKiMax,float componentKdMax,float outputMax);
void PID_Calc(PID_Regulator_t *pid);
void PID_Calc_EX(PID_Regulator_t *pid);
void PID_ALL_Init(void);

#endif
