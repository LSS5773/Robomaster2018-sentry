#include "shoot_task.h"

SHOOTEncoder_t SHOOTEncoder[3];
int threshold = 0;
int fric_speed = 0;
int shoot_feed = 0;
int16_t shoot_fric[2]={0,0};

void shoot_task(void) {
	if(INFANTRY_Mode.fire) {
		if(shootHeat > 240) ShootFeedSPID.ref = 1000;
		else ShootFeedSPID.ref = 4000;
		threshold = 250;
	} else {
		ShootFeedSPID.ref = 0;
		threshold = 999;
	}
	turn_on_friction_wheel(fric_speed);
	
	if(shootHeat > threshold) ShootFeedSPID.ref = 0;
	ShootFeedSPID.fdb = SHOOTEncoder[0].velocity;
	PID_Calc(&ShootFeedSPID);
	shoot_feed = ShootFeedSPID.output;
	Set_Shoot_Current(&hcan1, shoot_feed, shoot_fric);
}

void shoot_stop() {
	uint8_t TxMessage[8];
	TxMessage[0] = 0;
	TxMessage[1] = 0;
	TxMessage[2] = 0;
	TxMessage[3] = 0;
	TxMessage[4] = 0;
	TxMessage[5] = 0;
	TxMessage[6] = 0;
	TxMessage[7] = 0;
	CAN_Send_Msg(&hcan1, TxMessage, 0x200 , 8);
}



void turn_on_friction_wheel(uint16_t spd) {
	ShootFric1PID.ref = -spd;
	ShootFric2PID.ref = spd;
	ShootFric1PID.fdb = SHOOTEncoder[1].velocity;
	ShootFric2PID.fdb = SHOOTEncoder[2].velocity;
	PID_Calc(&ShootFric1PID);
	PID_Calc(&ShootFric2PID);
	shoot_fric[0] = ShootFric1PID.output;
	shoot_fric[1] = ShootFric2PID.output;
}
