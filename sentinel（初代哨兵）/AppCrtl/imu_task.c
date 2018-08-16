/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of?
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.? See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file imu_task.c
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief imu attitude calculation task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
  
#include "imu_task.h"
#include "gimbal_task.h"
#include "bsp_imu.h"
#include "bsp_io.h"
#include "pid.h"
#include "math.h"



/* imu task global parameter */
imu_data_t     imu;
imu_attitude_t atti;

/* imu task static parameter */
static volatile float qq0 = 1.0f;
static volatile float qq1 = 0.0f;
static volatile float qq2 = 0.0f;
static volatile float qq3 = 0.0f;
static volatile uint32_t last_update, now_update;
static volatile float exInt, eyInt, ezInt;
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;   //
float yaw_offset;
/**
  * @brief     Fast inverse square-root, to calculate 1/Sqrt(x)
  * @param[in] input:x
  * @retval    1/Sqrt(x)
  */
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

#define BOARD_DOWN 1   //

static void init_quaternion(void)
{
  int16_t hx, hy;
  float temp;

  hx = imu.mx;
  hy = imu.my;

  if (hy != 0)
    temp = hx/hy;
  else
    return ;

  #ifdef BOARD_DOWN
  if(hx<0 && hy <0)   //OK
  {
    if(fabs(temp) >= 1)
    {
      qq0 = -0.005;
      qq1 = -0.199;
      qq2 = 0.979;
      qq3 = -0.0089;
    }
    else
    {
      qq0 = -0.008;
      qq1 = -0.555;
      qq2 = 0.83;
      qq3 = -0.002;
    }
    
  }
  else if (hx<0 && hy > 0) //OK
  {
    if(fabs(temp) >= 1)   
    {
      qq0 = 0.005;
      qq1 = -0.199;
      qq2 = -0.978;
      qq3 = 0.012;
    }
    else
    {
      qq0 = 0.005;
      qq1 = -0.553;
      qq2 = -0.83;
      qq3 = -0.0023;
    }
    
  }
  else if (hx > 0 && hy > 0)   //OK
  {
    if(fabs(temp) >= 1)
    {
      qq0 = 0.0012;
      qq1 = -0.978;
      qq2 = -0.199;
      qq3 = -0.005;
    }
    else
    {
      qq0 = 0.0023;
      qq1 = -0.83;
      qq2 = -0.553;
      qq3 = 0.0023;
    }
    
  }
  else if (hx > 0 && hy < 0)     //OK
  {
    if(fabs(temp) >= 1)
    {
      qq0 = 0.0025;
      qq1 = 0.978;
      qq2 = -0.199;
      qq3 = 0.008;
    }
    else
    {
      qq0 = 0.0025;
      qq1 = 0.83;
      qq2 = -0.56;
      qq3 = 0.0045;
    }
  }
  #else
    if(hx<0 && hy <0)
  {
    if(fabs(temp) >= 1)
    {
      qq0 = 0.195;
      qq1 = -0.015;
      qq2 = 0.0043;
      qq3 = 0.979;
    }
    else
    {
      qq0 = 0.555;
      qq1 = -0.015;
      qq2 = 0.006;
      qq3 = 0.829;
    }
    
  }
  else if (hx<0 && hy > 0)
  {
    if(fabs(temp) >= 1)
    {
      qq0 = -0.193;
      qq1 = -0.009;
      qq2 = -0.006;
      qq3 = 0.979;
    }
    else
    {
      qq0 = -0.552;
      qq1 = -0.0048;
      qq2 = -0.0115;
      qq3 = 0.8313;
    }
    
  }
  else if (hx>0 && hy > 0)
  {
    if(fabs(temp) >= 1)
    {
      qq0 = -0.9785;
      qq1 = 0.008;
      qq2 = -0.02;
      qq3 = 0.195;
    }
    else
    {
      qq0 = -0.9828;
      qq1 = 0.002;
      qq2 = -0.0167;
      qq3 = 0.5557;
    }
    
  }
  else if (hx > 0 && hy < 0)
  {
    if(fabs(temp) >= 1)
    {
      qq0 = -0.979;
      qq1 = 0.0116;
      qq2 = -0.0167;
      qq3 = -0.195;
    }
    else
    {
      qq0 = -0.83;
      qq1 = 0.014;
      qq2 = -0.012;
      qq3 = -0.556;
    }
  }
  #endif
   
}

float halfT;
float Kp  = 2.0, Ki = 0.01;

//#define Kp 2.0f    // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases
static void imu_AHRS_update(void) 
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;//, halfT;
  float tempq0,tempq1,tempq2,tempq3;

  float q0q0 = qq0*qq0;
  float q0q1 = qq0*qq1;
  float q0q2 = qq0*qq2;
  float q0q3 = qq0*qq3;
  float q1q1 = qq1*qq1;
  float q1q2 = qq1*qq2;
  float q1q3 = qq1*qq3;
  float q2q2 = qq2*qq2;
  float q2q3 = qq2*qq3;
  float q3q3 = qq3*qq3;

  gx = imu.wx;
  gy = imu.wy;
  gz = imu.wz;
  ax = imu.ax;
  ay = imu.ay;
  az = imu.az;
  mx = imu.mx;
  my = imu.my;
  mz = imu.mz;

  now_update = HAL_GetTick(); //ms
  halfT =  ((float)(now_update - last_update) / 2000.0f);
  last_update = now_update;
	
  //Fast inverse square-root
  norm = invSqrt(ax*ax + ay*ay + az*az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

  norm = invSqrt(mx*mx + my*my + mz*mz);
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  // compute reference direction of flux
  hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
  hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
  hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz; 

  // estimated direction of gravity and flux (v and w)
  vx = 2.0f*(q1q3 - q0q2);
  vy = 2.0f*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
  wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
  wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
  {
      exInt = exInt + ex * Ki * halfT;
      eyInt = eyInt + ey * Ki * halfT;
      ezInt = ezInt + ez * Ki * halfT;
      // PI
      gx = gx + Kp*ex + exInt;
      gy = gy + Kp*ey + eyInt;
      gz = gz + Kp*ez + ezInt;
  }
  // 
  tempq0 = qq0 + (-qq1*gx - qq2*gy - qq3*gz)*halfT;
  tempq1 = qq1 + (qq0*gx + qq2*gz - qq3*gy)*halfT;
  tempq2 = qq2 + (qq0*gy - qq1*gz + qq3*gx)*halfT;
  tempq3 = qq3 + (qq0*gz + qq1*gy - qq2*gx)*halfT;  

  //normalise quaternion
  norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  qq0 = tempq0 * norm;
  qq1 = tempq1 * norm;
  qq2 = tempq2 * norm;
  qq3 = tempq3 * norm;

}

static void imu_attitude_update(void)
{
  imu.rol = atan2(2*qq2*qq3 + 2*qq0*qq1, -2*qq1*qq1 - 2*qq2*qq2 + 1)* 57.3; // roll       -pi----pi
  imu.pit = asin(-2*qq1*qq3 + 2*qq0*qq2)* 57.3;                         // pitch    -pi/2----pi/2 
  imu.yaw = atan2(2*qq1*qq2 + 2*qq0*qq3, -2*qq2*qq2 - 2*qq3*qq3 + 1)* 57.3; // yaw        -pi----pi
  
  
  if (imu.yaw - atti.last_yaw > 330)
    atti.yaw_cnt--;
  else if (imu.yaw - atti.last_yaw < -330)
    atti.yaw_cnt++;
  
  atti.last_yaw = imu.yaw;
  
  atti.yaw   = imu.yaw + atti.yaw_cnt*360;
  atti.pitch = imu.pit;
  atti.roll  = imu.rol;
	
	//Æ«²î
  yaw_offset += YAWOFFSET;
	atti.yaw += yaw_offset;
	atti.yaw = atti.yaw / 10;
  //gim.sensor.gyro_angle = atti.yaw;
}

void imu_param_init(void)
{
  init_quaternion();
	atti.yaw = 0;
}


void imu_task()
{
	imu_temp_keep();
	imu_AHRS_update();
	imu_attitude_update();
}


void imu_temp_keep(void)
{
	GMPPositionPID.ref = DEFAULT_IMU_TEMP;
	GMPPositionPID.fdb = imu.temp;
	PID_Calc(&TempPID);

  mpu_heat_ctrl(TempPID.output);
}
