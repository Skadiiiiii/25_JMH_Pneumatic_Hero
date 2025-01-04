#include "shoot_control.h"

/**
  * @brief  获取拨盘转速（防卡弹逻辑）
  */
static void Get_Dial_Targetspeed()
{
	static uint8_t Turn_Back;
	static uint16_t Back_time;
	static uint16_t Dial_Speed_Time;
	
	if(M3508s_dial.torque_current > 6500)
	{
		Dial_Speed_Time++;
		if(Dial_Speed_Time > 500)
		{
			Turn_Back = 1;
			Dial_Speed_Time = 0;	
		}
	}
	if(Turn_Back == 0)
	{
		M3508s_dial.target_rotor_speed = 1500;
	}
	else if(Turn_Back == 1)//反转
	{
		Back_time ++;
		M3508s_dial.target_rotor_speed = -1500;
		if(Back_time > 300)
		{
			Turn_Back = 0;
			Back_time = 0;
		}
	}
}

/**
  * @brief  拨盘使能
  */
void Ship_ChassisWorkMode_shoot()
{
	Get_Dial_Targetspeed();
	
	M3508s_dial.set_current = pid_calc_incremental(&motor_pid_dial_speed,M3508s_dial.target_rotor_speed,M3508s_dial.rotor_speed);

	set_M3508_200_current(&hcan1,0,0,0,M3508s_dial.set_current);
}

/**
  * @brief  拨盘停转
  */
void Ship_ChassisWorkMode_shoot_stop()
{
	M3508s_dial.set_current = pid_calc_incremental(&motor_pid_dial_speed,0,M3508s_dial.rotor_speed);

	set_M3508_200_current(&hcan1,0,0,0,M3508s_dial.set_current);
}

/**
  * @brief  拨盘失能
  */
void Robot_control_dial_disable()
{
	set_M3508_200_current(&hcan1,0,0,0,0);
}

