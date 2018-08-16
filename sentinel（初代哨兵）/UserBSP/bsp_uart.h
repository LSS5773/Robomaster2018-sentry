/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       bsp_uart.h
	* @brief      uart receive data from DBus/bt/judge_system/manifold etc.
	* @update
  * @note       use DMA receive, but donot trigger DMA interrupt
	*             handle received data in usart idle interrupt handle function
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Jun-01-2017   Richard.luo      remove some useless module
  * @verbatim
	*		idle interrupt --> handle data --> clear it flag --> initialize DMA again
	*
	********************************(C) COPYRIGHT 2017 DJI************************
	*/

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

//#include "sys.h"
#include "usart.h"

#define DBUS_HUART huart1
#define CV_HUART	 huart6
#define AUTOP_SIZE	10
#define DBUS_RX_MAX_BUFLEN	21
#define MAX_DMA_COUNT		100

#pragma pack()

extern uint8_t uart2_rx_buff[50];
extern uint8_t uart3_rx_buff[50];
extern uint8_t uart6_rx_buff[50];
extern uint32_t tick_controller;

typedef struct {
	float LV;
	float LH;
	float RV;
	float RH;
	
	//ch value: -660 ~ 660
	int16_t ch1;	
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	
	//switch val: 1 3 2
	uint8_t sw1;	
	uint8_t sw2;
	
	struct {
		int16_t x;
		int16_t y;
		int16_t z;//no use
	
		//press:1 release:0
		uint8_t l;
		uint8_t r;
	}mouse;
	
	union {
		uint16_t key_code;
/**********************************************************************************
 * keyboard :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 *            V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 ************************************************************************************/
		struct {
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		}bit;
	}kb;
	
	int16_t wheel;
	
}RC_Type;


enum{
	RC_UP = 1,
	RC_MI = 3,
	RC_DN = 2,
};

typedef __packed struct
{
	char sofa; 	//'a'
	char sof5; 	//'5'
	float time;
	char endf1; //'f'
	char endf2;	//'f'	
} tReceTXoneData;

typedef __packed struct
{
	uint8_t sof;      //0xA5
	int16_t angle;    //chassis angle
	int16_t v_w;      //chassis angluar rate  deg/s
	int16_t v_x;      //x velocity  mm/s
	int16_t v_y;      //y velocity  mm/s

  uint8_t  flag;    //uwb 1:useful 0:useless
  uint32_t x;
  uint32_t y;
  uint32_t z;
  uint32_t compass;
	uint8_t dataStatus;  //
	uint8_t end;      //0xFE
} tSendTXoneData;


typedef struct
{
  uint8_t vision_state;
  uint8_t target_num;
  int16_t pit;
  int16_t yaw;
 
} rece_buff_t;



extern RC_Type rc;
extern RC_Type last_rc;

extern tSendTXoneData send_data;
extern tReceTXoneData rece_data;
extern float rece_distance[];
extern uint8_t rece_flag;

void dbus_init(void);
void manifold_uart_init(void);
void judge_sys_init(void);

void uart_send_data(UART_HandleTypeDef *huart, uint8_t *send_data);
void uart_receive_handler(UART_HandleTypeDef *huart);

#endif
