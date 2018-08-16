// 2018-06-11 by LSS
#include "control_task.h"

static uint16_t time_tick_1ms = 0;
uint32_t time_tick_wait = 0;
extern uint32_t timeTick;
uint8_t offline[TICK_MAX_SIZE] = {0};
uint32_t tick[TICK_MAX_SIZE] = {0};
uint8_t exception = 0;
INFANTRY_Mode_t INFANTRY_Mode;
unsigned char aim_armor_color = Red_Armor;

const Sound_tone_e Tone_Warning[TONE_MAX_SIZE] = {
  Do1L,     ///*261.63Hz*/    3822us
  Re2L,     ///*293.66Hz*/    3405us
  Mi3L,     ///*329.63Hz*/    3034us
  Fa4L,     ///*349.23Hz*/    2863us
  So5L,     ///*392.00Hz*/    2551us
  La6L,     ///*440.00Hz*/    2272us
  Si7L,     ///*493.88Hz*/    2052us
  Do1M,     ///*523.25Hz*/    1911us
  Re2M,     ///*587.33Hz*/    1703us
  Mi3M,     ///*659.26Hz*/    1517us
  Fa4M,     ///*698.46Hz*/    1432us
  So5M,     ///*784.00Hz*/    1276us
  La6M,     ///*880.00Hz*/    1136us
  Si7M,     ///*987.77Hz*/    1012us
  Do1H,     ///*1046.50Hz*/   956us
  Re2H,     ///*1174.66Hz*/   851us
  Mi3H,     ///*1318.51Hz*/   758us
  Fa4H,     ///*1396.91Hz*/   716us
  So5H,     ///*1567.98Hz*/   638us
  La6H,     ///*1760.00Hz*/   568us
  Si7H,     ///*1975.53Hz*/   506us
};

void offline_warning(int index) {
	if(Startup_Success_music_index < Startup_Success_music_len) return;
	if(index < TICK_MAX_SIZE) Sing(Tone_Warning[index]);
}

void offline_check(void) {
	uint32_t Period = 400; // ms
	uint32_t TimeOut = 200; // ms
	uint8_t offline_num = 0;
	static uint8_t offline_count = 0;
	for(int i=0; i<TICK_MAX_SIZE; i++) {
		if((HAL_GetTick()-tick[i]) < TimeOut) offline[i] = 0;
		else offline[i] = 1;
		offline_num += offline[i];
	}
	uint8_t offline_index = 0;
	for(int i=0; i<TICK_MAX_SIZE; i++) {
		if(offline[i]) {
			offline_index = i;
			break;
		}
	}
	
	if(Startup_Success_music_index < Startup_Success_music_len) return;
	if(exception) {
		Sing(Do1L);
		return;
	}
	
	if(offline_num == 0) {
		Sing(Silent);
	} else {
		if(time_tick_1ms < 0.5f*Period) {
			if(offline_count <= offline_index) offline_warning(offline_count);
			else Sing(Silent);
		}
		if(time_tick_1ms % Period == 0) {
			offline_count++;
			if(offline_count > offline_index+2) offline_count = 0;
		}
	}
}


void mode_init() {
	pr.x = 0;
	pr.y = 200;
	pr.z = 1000;
	INFANTRY_Mode.chassis_state = Cruise;
	INFANTRY_Mode.gimbal_state = NoTarget;
}

void refreshCurrentMode() {
	switch(rc.sw1) {
		case RC_UP:
			INFANTRY_Mode.gimbal = GIMBAL_Auto_Mode;
			INFANTRY_Mode.chassis = CHASSIS_Auto_Mode;
			fric_speed = HIGH_FRIC_WHEEL_SPEED;
			break;
		
		case RC_MI:
			INFANTRY_Mode.gimbal = GIMBAL_Test_Mode;
			INFANTRY_Mode.chassis = CHASSIS_Auto_Mode;
			INFANTRY_Mode.fire = 0;
			if(rc.sw2 == RC_MI) aim_armor_color = Red_Armor;
			else if(rc.sw2 == RC_DN) aim_armor_color = Blue_Armor;
			time_tick_wait = 0;
			speed = SPEED_CRUISE;
			fric_speed = IDLING_FRIC_WHEEL_SPEED;
			break;
		
		case RC_DN:
			INFANTRY_Mode.gimbal = GIMBAL_Depart_Mode;
			INFANTRY_Mode.chassis = CHASSIS_Depart_Mode;
			if(rc.sw2 == RC_UP) {
				INFANTRY_Mode.fire = 1;
				fric_speed = IDLING_FRIC_WHEEL_SPEED;
			} else {
				INFANTRY_Mode.fire = 0;
				fric_speed = 0;
			}
			if(rc.sw2 == RC_MI) aim_armor_color = Red_Armor;
			else if(rc.sw2 == RC_DN) aim_armor_color = Blue_Armor;
			mode_init();
			time_tick_wait = 0;
			pitch_edge[0] = pitch;
			pitch_edge[1] = pitch;
			yaw_right = yaw;
			break;
		
		default:;
	}
	last_rc = rc;
}

void Control_Task(void) {
	time_tick_1ms++;
	timeTick++;
	if(time_tick_wait < TIME_WAIT) time_tick_wait++;
	refreshCurrentMode();
	offline_check();
	
	if((HAL_GetTick()-tick_controller) < TIMEOUT) {
		shoot_task();
		turn_on_laser();
	} else if(SHOOTEncoder[FRIC1_ID-FEED_ID].velocity || SHOOTEncoder[FRIC2_ID-FEED_ID].velocity) {
		exception = 0;
		shoot_stop();
		turn_off_laser();
	}
	gimbal_task();
	chassis_task();
	can_send_task();
	
	if(time_tick_1ms % 80 == 0) {
    if(Startup_Success_music_index < Startup_Success_music_len)
      Sing_Startup_music(Startup_Success_music_index);
    Startup_Success_music_index++;
  }
	if(time_tick_1ms % 400 == 0) {
		can_send_TX2(aim_armor_color);
		if(aim_armor_color == Red_Armor) {
			LED_Red_On();
			LED_Green_Off();
		} else {
			LED_Green_On();
			LED_Red_Off();
		}
		time_tick_1ms = 0;
	}
}


void TIM6_DAC_IRQHandler(void) {
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  Control_Task();
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

