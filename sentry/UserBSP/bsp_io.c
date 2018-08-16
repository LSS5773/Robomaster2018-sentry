/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file bsp_io.c
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief basic IO port operation
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "bsp_io.h"
#include "tim.h"



/****************************************************************************
 *  @brief  微动开关状态
 *  @return  0/1
 ***************************************************************************/
uint8_t get_trigger_key_state(void)
{
  return 0;
}



/****************************************************************************
 *  @brief  mpu温度控制
 ***************************************************************************/
void mpu_heat_ctrl(uint16_t pwm_pulse)
{
  IMU_PWM_PULSE = pwm_pulse;
}

/****************************************************************************
 *  @brief  初始化加热电阻、蜂鸣器、摩擦轮
 ***************************************************************************/
void pwm_device_init(void)
{
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // ctrl imu temperature
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // beep
}
void turn_on_laser(void)
{
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
}

void turn_off_laser(void)
{
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
}

