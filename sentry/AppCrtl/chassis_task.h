#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "gimbal_task.h"
#include "judge_task.h"
#include "rm_pid.h"
#include "filter.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "control_task.h"


#define PI 3.1415926f
#define FL 0
#define FR 1
#define BR 2
#define BL 3

/* chassis control period time (ms) */
//#define CHASSIS_PERIOD 10

#define RADIAN_COEF 57.3f
/************************ chassis parameter ****************************/
/* the radius of wheel(mm) */
#define RADIUS     18.0f
/* the perimeter of wheel(mm) */
#define PERIMETER  113
/* the deceleration ratio of chassis motor */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* single 3508 motor maximum speed, unit is rpm */
#define MAX_WHEEL_RPM        8500  //8347rpm = 844mm/s

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

typedef struct{
	uint8_t canbus[2];
	int16_t velocity;
	int16_t current;
	float angle;
	int16_t angle_last;
	int16_t angle_raw;
	int16_t zero;
	int32_t round;
} CMEncoder_t;

typedef struct
{
  float           vx; // forward/back
  float           vy; // left/right
  float           vw;
  int16_t         rotate_x_offset;
  int16_t         rotate_y_offset;
  
  float           gyro_angle;
  float           gyro_palstance;

  int16_t         wheel_spd_fdb[4];
  int16_t         wheel_spd_ref[4];
  int16_t         current[4];
  
  int16_t         position_ref;
  uint8_t         follow_gimbal;
} chassis_t;

extern CMEncoder_t CMEncoder[4];
extern chassis_t chassis;
extern float speed;
extern float dis;
extern float edge;
extern uint8_t hit;
extern uint16_t last_HP;
extern int speed_cruise;
extern int speed_max;


void _auto_mode_handle(void);
void _depart_mode_handle(void);

void value_init(void);
void chassis_action(void);
void chassis_task(void);
void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);

void set_edge(void);
void set_target_distance(int velocity);
void auto_rand(void);
void auto_twist(void);
void auto_follow(void);
void auto_cruise(void);

#endif
