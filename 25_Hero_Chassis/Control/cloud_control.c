#include "cloud_control.h"
#include "DM4310_motor.h"
#include "pid.h"

/**
  * @brief  过零处理，计算最小偏差
  */
int ComputeMinOffset(int target, int value) 
{
    int err = target - value;
	
    if (err > 4096)
    {
        err -= 8191;
    }
    else if (err < -4096)
    {
        err += 8191;
    }
    return err;
}

/**
 * @brief      计算最小偏差，使转向轮保持劣弧旋转
 */
float Turn_InferiorArc(float target, float current)
{
  float Error = target - current;

  if(Error > 180.0f)
	{
		return (target - 360.0f);
	}
	else if(Error < -180.0f)
	{
		return (target + 360.0f);
	}
	else
	{
		return target;
	}
}

/**
  * @brief  云台使能		
  */
bool yaw_enable;
void Ship_ChassisWorkMode_cloud(float delta_yaw)
{
	
	if(yaw_enable == 0)
	{
		motor_enable();
		yaw_enable = 1;
	}
	
	DM4310s_yaw.target_rotor_angle = DM4310s_yaw.position;
	DM4310s_yaw.target_rotor_angle += delta_yaw;
		

	DM4310s_yaw.torque = pid_CascadeCalc(&motor_pid_Cas_Yaw, DM4310s_yaw.target_rotor_angle,DM4310s_yaw.position,DM4310s_yaw.speed);
	
	MIT_CtrlMotor(&hcan1,DM_Send_ID,0,0,0,0,DM4310s_yaw.torque);
}

/**
  * @brief  云台失能
  */
void Robot_control_cloud_disable()
{
	motor_disable();
	yaw_enable = 0;
}

