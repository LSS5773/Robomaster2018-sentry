#include "bsp_can.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "shoot_task.h"

short data[8] = {0};
int16_t global_pitch_iq;
int16_t global_yaw_iq;
uint8_t feed_round;
uint16_t FPS_TX2 = 0;
uint16_t count_FPS_TX2 = 0;

/********************************************************************************
	接受并处理CAN总线上的电调发回的数据
*********************************************************************************/
void CanReceiveMsgProcess(CAN_HandleTypeDef* hcan) {
	uint8_t index;
	if(hcan == &hcan1) {
		switch(hcan->pRxMsg->StdId) {
			case CAN_R_ID:
				for(int i=0; i<8; i++) data[i] = hcan->pRxMsg->Data[i];
				pr.x = (short)(data[0]<<8|data[1]);
				pr.y = (short)(data[2]<<8|data[3]);
				pr.z = (short)(data[4]<<8|data[5]);
				isHero = data[6];
				calc_angle(GMPitchEncoder.angle, pr);
				if(tick_FPS < 1000) count_FPS_TX2++;
				else {
					FPS_TX2 = count_FPS_TX2;
					count_FPS_TX2 = 0;
					tick_FPS = 0;
				}
				tick[eTX2] = HAL_GetTick();
				break;
			
			case GMYAW_ID:
				GMYawEncoder.canbus[0] = hcan->pRxMsg->Data[0];
				GMYawEncoder.canbus[1] = hcan->pRxMsg->Data[1];

				GMYawEncoder.angle_raw_value = GMYawEncoder.canbus[0] << 8 | GMYawEncoder.canbus[1];
				GMYawEncoder.angle = GMYawEncoder.angle_raw_value * 0.0439453125f;	
				if(GMYawEncoder.angle > 360) GMYawEncoder.angle -= 360;
				GMYawEncoder.angle -= OFFSET_YAW_ANGLE;
				if(GMYawEncoder.angle < 0) GMYawEncoder.angle += 360;
				tick[eYaw] = HAL_GetTick();
				break;
			
			case GMPITCH_ID:
				GMPitchEncoder.canbus[0] = hcan->pRxMsg->Data[0];
				GMPitchEncoder.canbus[1] = hcan->pRxMsg->Data[1];
				GMPitchEncoder.angle_raw_value = GMPitchEncoder.canbus[0] << 8 | GMPitchEncoder.canbus[1];
				GMPitchEncoder.angle = GMPitchEncoder.angle_raw_value * 0.0439453125f;
				if(GMPitchEncoder.angle>180) GMPitchEncoder.angle = GMPitchEncoder.angle - 360;
				GMPitchEncoder.angle = GMPitchEncoder.angle - OFFSET_PITCH_ANGLE;
				tick[ePitch] = HAL_GetTick();
				break;
			
			case FEED_ID:
				SHOOTEncoder[0].angle_last = SHOOTEncoder[0].angle_raw_value;
				SHOOTEncoder[0].velocity = (short)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
				SHOOTEncoder[0].angle_raw_value = (short)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]);
				if(SHOOTEncoder[0].angle_raw_value < 3500 && SHOOTEncoder[0].angle_last > 4500) feed_round++;
				else if(SHOOTEncoder[0].angle_raw_value > 5000 && SHOOTEncoder[0].angle_last < 3000) feed_round--;
				SHOOTEncoder[0].angle_real = SHOOTEncoder[0].angle_raw_value + feed_round * 8192;
				SHOOTEncoder[0].angle_real = SHOOTEncoder[0].angle_real * 0.00228843593358f; // 0.0012207031f
				tick[eFeed] = HAL_GetTick();
				break;
				
			case FRIC1_ID:
			case FRIC2_ID:
				index = hcan->pRxMsg->StdId - FEED_ID;
				SHOOTEncoder[index].velocity = (short)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);			
				SHOOTEncoder[index].angle_raw_value = (short)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]);
				tick[eFric1+index-1] = HAL_GetTick();
				break;
		}
	} else if(hcan == &hcan2) {
		switch(hcan->pRxMsg->StdId) {
			case CM1_ID:
			case CM2_ID:
				index = hcan->pRxMsg->StdId - CM1_ID;
				CMEncoder[index].angle_last = CMEncoder[index].angle_raw;
				CMEncoder[index].angle_raw = (short)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]);
				CMEncoder[index].velocity = (short)(hcan->pRxMsg->Data[2]<<8 | hcan->pRxMsg->Data[3]);
				CMEncoder[index].current = (short)(hcan->pRxMsg->Data[4]<<8 | hcan->pRxMsg->Data[5]);
				if(CMEncoder[index].angle_raw < 2000 && CMEncoder[index].angle_last > 6000) CMEncoder[index].round++;
				else if(CMEncoder[index].angle_raw > 6000 && CMEncoder[index].angle_last < 2000) CMEncoder[index].round--;
				CMEncoder[index].angle = CMEncoder[index].angle_raw + CMEncoder[index].round*8192;
				CMEncoder[index].angle *= 0.00228843593358f; // 1/8192/3591/187*360 (3591:187) unit: deg
				tick[eCM1+index] = HAL_GetTick();
				break;
		}
	}
}
/********************************************************************************
	can filter 在使用前必须初始化
*********************************************************************************/
void CanFilter_Init(CAN_HandleTypeDef* hcan) {
  CAN_FilterConfTypeDef canfilter;
  
  //create memory to save the message, if not will raise error
  static CanTxMsgTypeDef  Tx1Message;
  static CanRxMsgTypeDef  Rx1Message;
  static CanTxMsgTypeDef  Tx2Message;
  static CanRxMsgTypeDef  Rx2Message;
  
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
  
  //filtrate any ID you want here
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;
  
  canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
  canfilter.FilterActivation = ENABLE;
  canfilter.BankNumber = 14;
  
  //use different filter for can1&can2
  if(hcan == &hcan1)
  {
    canfilter.FilterNumber = 0;
    hcan->pTxMsg = &Tx1Message;
    hcan->pRxMsg = &Rx1Message;
  }
  if(hcan == &hcan2)
  {
    canfilter.FilterNumber = 14;
    hcan->pTxMsg = &Tx2Message;
    hcan->pRxMsg = &Rx2Message;
  }
  
  HAL_CAN_ConfigFilter(hcan, &canfilter);
  
}

/********************************************************************************
	CAN的回调函数
*********************************************************************************/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan) {
	CanReceiveMsgProcess(hcan);
	HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}

/********************************************************************************
   向CAN总线发送数据的函数
*********************************************************************************/
void CAN_Send_Msg(CAN_HandleTypeDef* hcan, uint8_t *msg, uint32_t id, uint8_t len) {
  uint8_t index = 0;
  
  hcan->pTxMsg->StdId = id;
  hcan->pTxMsg->IDE = CAN_ID_STD;
  hcan->pTxMsg->RTR = CAN_RTR_DATA;
  hcan->pTxMsg->DLC = len;
  
  for(index = 0; index <len; index++)
    hcan->pTxMsg->Data[index] = msg[index];
  
   HAL_CAN_Transmit_IT(hcan);
}

void can_send_TX2(int signal) {
	uint8_t TxMessage[8];
	for(int i=0; i<8; i++) TxMessage[i] = 0;
	if(signal == Red_Armor) {
		TxMessage[0] = 0x0;
		TxMessage[1] = 0x0;
	} else {
		TxMessage[0] = 0x0;
		TxMessage[1] = 0x1;
	}
	CAN_Send_Msg(&hcan1, TxMessage, CAN_T_ID , 8);
}


void Set_Gimbal_Current(CAN_HandleTypeDef* hcan, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq) {
	uint8_t TxMessage[8];
	TxMessage[0] = (unsigned char)(gimbal_yaw_iq >> 8);
	TxMessage[1] = (unsigned char)gimbal_yaw_iq;
	TxMessage[2] = (unsigned char)(gimbal_pitch_iq >> 8);
	TxMessage[3] = (unsigned char)gimbal_pitch_iq;
	TxMessage[4] = 0x00;
	TxMessage[5] = 0x00;
	TxMessage[6] = 0x00;
	TxMessage[7] = 0x00;
	CAN_Send_Msg(hcan, TxMessage, 0x1ff , 8);
}

void Set_Shoot_Current(CAN_HandleTypeDef* hcan, int16_t shoot_feed, int16_t* shoot_fric) {
	uint8_t TxMessage[8];
	TxMessage[0] = (unsigned char)(shoot_feed >> 8);
	TxMessage[1] = (unsigned char)shoot_feed;
	TxMessage[2] = (unsigned char)(shoot_fric[0] >> 8);
	TxMessage[3] = (unsigned char)shoot_fric[0];
	TxMessage[4] = (unsigned char)(shoot_fric[1] >> 8);
	TxMessage[5] = (unsigned char)shoot_fric[1];
	TxMessage[6] = 0x00;
	TxMessage[7] = 0x00;
	CAN_Send_Msg(hcan, TxMessage, 0x200 , 8);
}

