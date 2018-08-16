#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#include "stm32f4xx.h"
#include "stdint.h"
#include "can.h"

#define FEED_ID			0x201
#define FRIC1_ID		0x202
#define FRIC2_ID		0x203

#define GMPITCH_ID	0x205
#define GMYAW_ID		0x206
#define CM1_ID			0x207
#define CM2_ID			0x208

void can_send_TX2(int signal);
void can_send_task(void);
void CanFilter_Init(CAN_HandleTypeDef* hcan);
void CAN_Send_Msg(CAN_HandleTypeDef* hcan, uint8_t *msg, uint32_t id, uint8_t len);
void CanReceiveMsgProcess(CAN_HandleTypeDef* hcan);
void Set_Gimbal_Current(CAN_HandleTypeDef* hcan, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq);
void Set_CM_Speed(CAN_HandleTypeDef* hcan, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void Set_Shoot_Current(CAN_HandleTypeDef* hcan, int16_t shoot_feed, int16_t* shoot_fric);

extern uint8_t feed_round;
extern int16_t global_pitch_iq;
extern int16_t global_yaw_iq;
extern uint32_t tick_TX2;
#endif
