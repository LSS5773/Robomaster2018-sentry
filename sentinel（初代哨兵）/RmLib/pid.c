#include "pid.h"
#include <math.h>

PID_Regulator_t AutoPitchPID;
PID_Regulator_t AutoYawPID;
PID_Regulator_t GMPPositionPID;
PID_Regulator_t GMPSpeedPID;
PID_Regulator_t GMYPositionPID;
PID_Regulator_t GMYSpeedPID;
PID_Regulator_t AbsYawSpeedPID;
PID_Regulator_t AbsYawPositionPID;

PID_Regulator_t ShootFeedPPID;
PID_Regulator_t ShootFeedSPID;
PID_Regulator_t ShootFric1PID;
PID_Regulator_t ShootFric2PID;

PID_Regulator_t TempPID;

void PID_Calc_EX(PID_Regulator_t *pid)
{
	pid->err[0] = pid->err[1];
	pid->err[1] = pid->ref - pid->fdb;
	pid->inte += pid->err[1];
	
	if(fabs(pid->ref) < 5 && fabs(pid->fdb) < 5) pid->componentKp  = 0.5f * pid->kp * pid->err[1];
	else pid->componentKp  = pid->kp * pid->err[1];
	
	if(fabs(pid->ref) < 5) pid->componentKi = 0.5f * pid->ki * pid->inte;
	else pid->componentKi  = pid->ki * pid->inte;
	
	pid->componentKd  = pid->kd * (pid->err[1] - pid->err[0]);
	
	if(pid->inte > 2000)//对积分项进行限制
		pid->inte = 2000;
	else if (pid->inte < -2000)
		pid->inte = -2000;
	
	if(pid->componentKp > pid->componentKpMax)
		pid->componentKp = pid->componentKpMax;
	else if (pid->componentKp < -pid->componentKpMax)
		pid->componentKp = -pid->componentKpMax;
	
	if(pid->componentKi > pid->componentKiMax)
		pid->componentKi = pid->componentKiMax;
	else if (pid->componentKi < -pid->componentKiMax)
		pid->componentKi = -pid->componentKiMax;

	if(pid->componentKd > pid->componentKdMax)
		pid->componentKd = pid->componentKdMax;
	else if (pid->componentKd < -pid->componentKdMax)
		pid->componentKd = -pid->componentKdMax;
	
	
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	
	if(pid->output > pid->outputMax)
		pid->output = pid->outputMax;
	else if (pid->output < -pid->outputMax)
		pid->output = -pid->outputMax;	
}

void PID_Calc(PID_Regulator_t *pid)
{
	pid->err[0] = pid->err[1];
	pid->err[1] = pid->ref - pid->fdb;
	pid->inte += pid->err[1];
		
	
	pid->componentKp  = pid->kp * pid->err[1];
	pid->componentKi  = pid->ki * pid->inte;
	pid->componentKd  = pid->kd * (pid->err[1] - pid->err[0]);
	
	if(pid->inte > 2000)//对积分项进行限制
		pid->inte = 2000;
	else if (pid->inte < -2000)
		pid->inte = -2000;
	
	if(pid->componentKp > pid->componentKpMax)
		pid->componentKp = pid->componentKpMax;
	else if (pid->componentKp < -pid->componentKpMax)
		pid->componentKp = -pid->componentKpMax;
	
	if(pid->componentKi > pid->componentKiMax)
		pid->componentKi = pid->componentKiMax;
	else if (pid->componentKi < -pid->componentKiMax)
		pid->componentKi = -pid->componentKiMax;

	if(pid->componentKd > pid->componentKdMax)
		pid->componentKd = pid->componentKdMax;
	else if (pid->componentKd < -pid->componentKdMax)
		pid->componentKd = -pid->componentKdMax;
	
	
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	
	if(pid->output > pid->outputMax)
		pid->output = pid->outputMax;
	else if (pid->output < -pid->outputMax)
		pid->output = -pid->outputMax;	
}

void PID_Init(PID_Regulator_t *pid,float kp,float ki,float kd,float componentKpMax,float componentKiMax,float componentKdMax,float outputMax)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->inte = 0;
	pid->componentKpMax = componentKpMax;
	pid->componentKiMax = componentKiMax;
	pid->componentKdMax = componentKdMax;
	pid->outputMax = outputMax;
}

void PID_ALL_Init(void)
{
	PID_Init(&AutoPitchPID,AUTO_PITCH_KP,AUTO_PITCH_KI,AUTO_PITCH_KD,\
					FLOATMAX,FLOATMAX,FLOATMAX,AUTO_PITCH_OUTPUTMAX);
	
	PID_Init(&AutoYawPID,AUTO_YAW_KP,AUTO_YAW_KI,AUTO_YAW_KD,\
					FLOATMAX,FLOATMAX,FLOATMAX,AUTO_YAW_OUTPUTMAX);
	
	
	
	PID_Init(&GMPPositionPID,PITCH_POSITION_KP_DEFAULTS,PITCH_POSITION_KI_DEFAULTS,PITCH_POSITION_KD_DEFAULTS,\
					PITCH_POSITION_KPMAX,PITCH_POSITION_KIMAX,PITCH_POSITION_KDMAX,PITCH_POSITION_OUTPUTMAX);
	
	PID_Init(&GMPSpeedPID,PITCH_SPEED_KP_DEFAULTS,PITCH_SPEED_KI_DEFAULTS,PITCH_SPEED_KD_DEFAULTS,\
					PITCH_SPEED_KPMAX,PITCH_SPEED_KIMAX,PITCH_SPEED_KDMAX,PITCH_SPEED_OUTPUTMAX);
	
	PID_Init(&GMYPositionPID,YAW_POSITION_KP_DEFAULTS,YAW_POSITION_KI_DEFAULTS,YAW_POSITION_KD_DEFAULTS,\
					YAW_POSITION_KPMAX,YAW_POSITION_KIMAX,YAW_POSITION_KDMAX,YAW_POSITION_OUTPUTMAX);
	
	PID_Init(&GMYSpeedPID,YAW_SPEED_KP_DEFAULTS,YAW_SPEED_KI_DEFAULTS,YAW_SPEED_KD_DEFAULTS,\
					YAW_SPEED_KPMAX,YAW_SPEED_KIMAX,YAW_SPEED_KDMAX,YAW_SPEED_OUTPUTMAX);
	
	PID_Init(&AbsYawSpeedPID,AbsYawPID_SPEED_KP_DEFAULTS,AbsYawPID_SPEED_KI_DEFAULTS,AbsYawPID_SPEED_KD_DEFAULTS,\
					AbsYawPID_SPEED_KPMAX,AbsYawPID_SPEED_KIMAX,AbsYawPID_SPEED_KDMAX,AbsYawPID_SPEED_OUTPUTMAX);
	
	PID_Init(&AbsYawPositionPID,AbsYawPID_POSITION_KP_DEFAULTS,AbsYawPID_POSITION_KI_DEFAULTS,AbsYawPID_POSITION_KD_DEFAULTS,\
					AbsYawPID_POSITION_KPMAX,AbsYawPID_POSITION_KIMAX,AbsYawPID_POSITION_KDMAX,AbsYawPID_POSITION_OUTPUTMAX);
	//Shoot PID
	PID_Init(&ShootFeedPPID,FEED_POSITION_KP_DEFAULTS,FEED_POSITION_KI_DEFAULTS,FEED_POSITION_KD_DEFAULTS,\
			  	FEED_POSITION_KPMAX,FEED_POSITION_KIMAX,FEED_POSITION_KDMAX,FEED_POSITION_OUTPUTMAX);
	
	PID_Init(&ShootFeedSPID,FEED_SPEED_KP_DEFAULTS,FEED_SPEED_KI_DEFAULTS,FEED_SPEED_KD_DEFAULTS,\
			  	FEED_SPEED_KPMAX,FEED_SPEED_KIMAX,FEED_SPEED_KDMAX,FEED_SPEED_OUTPUTMAX);
	
	PID_Init(&ShootFric1PID,FRIC1_SPEED_KP_DEFAULTS,FRIC1_SPEED_KI_DEFAULTS,FRIC1_SPEED_KD_DEFAULTS,\
					FRIC1_SPEED_KPMAX,FRIC1_SPEED_KIMAX,FRIC1_SPEED_KDMAX,FRIC1_SPEED_OUTPUTMAX);
	
	PID_Init(&ShootFric2PID,FRIC2_SPEED_KP_DEFAULTS,FRIC2_SPEED_KI_DEFAULTS,FRIC2_SPEED_KD_DEFAULTS,\
					FRIC2_SPEED_KPMAX,FRIC2_SPEED_KIMAX,FRIC2_SPEED_KDMAX,FRIC2_SPEED_OUTPUTMAX);
					
	PID_Init(&TempPID,TEMP_KP_DEFAULTS,TEMP_KI_DEFAULTS,TEMP_KD_DEFAULTS,\
			  	TEMP_KPMAX,TEMP_KIMAX,TEMP_KDMAX,TEMP_OUTPUTMAX);
}


