#include "chassis_control.h"
#include "cloud_control.h"
#include "shoot_control.h"
#include "power_limit_control.h"
#include "robot_control.h"
#include "SupCap.h"
#include "dr16.h"

RemoteMode_e s_RemoteMode = Stop_car;

/**
	* @brief  设置遥控和超电模式
  */
static void SetRemoteMode(void)
{
	
	if(DR16.rc.sw1 == remote_rc_mid && DR16.rc.sw2 == remote_rc_mid)
	{
		SetSuperCap_Mode(Cap_Enable); 
		SupCap.FUN.SupCap_SupplySwitch(Power_Supply);
	}
	else
	{	
		SetSuperCap_Mode(Cap_Close);
		SupCap.FUN.SupCap_SupplySwitch(Power_NotSupply);
	}
	if(DR16.rc.sw1 == remote_rc_up && DR16.rc.sw2 == remote_rc_up)
	{
		s_RemoteMode = KeyMouseControl;//键鼠模式
	}
	else
	{
		s_RemoteMode = RemoteControl;//遥控模式
	}
}

/**
  * @brief  机器人主控制
  */
static void Robot_control ()        
{
	if(DR16_Export_Data.ControlSwitch->Left == 3  && DR16_Export_Data.ControlSwitch->Right == 2)//左中右下（底盘）
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Chassis;
		
		Robot_control_cloud_disable();
		
		Ship_ChassisWorkMode(14.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,
												 14.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value,
												 -10.0f*DR16_Export_Data.Robot_TargetValue.Yaw_Value);
	}
	else if(DR16_Export_Data.ControlSwitch->Left == 2 && DR16_Export_Data.ControlSwitch->Right == 3)//左下右中（云台）
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Cloud;
		Robot_control_chassis_disable(); 
		
		Ship_ChassisWorkMode_cloud(0.0003f*DR16_Export_Data.Robot_TargetValue.Yaw_Value);
		
	}
	else if(DR16_Export_Data.ControlSwitch->Left == 3 && DR16_Export_Data.ControlSwitch->Right == 3)//双中（跟随）
	{
//		DR16_Export_Data.ChassisWorkMode = WorkMode_Follow;
//		
//		Ship_ChassisWorkMode_follow(10.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,
//																10.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value);
		
//		Ship_ChassisWorkMode(14.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,
//												 14.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value,
//												 -10.0f*DR16_Export_Data.Robot_TargetValue.Yaw_Value);
		
		Ship_ChassisWorkMode_cloud(0.0003f*DR16_Export_Data.Robot_TargetValue.Yaw_Value);
	}
	else if(DR16_Export_Data.ControlSwitch->Left == 1 && DR16_Export_Data.ControlSwitch->Right == 3)//左上右中（发射）
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Shoot;
	
		Ship_ChassisWorkMode_shoot();
		Ship_ChassisWorkMode_follow(10.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,
															10.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value);
	}
	else if(DR16_Export_Data.ControlSwitch->Left == 3 && DR16_Export_Data.ControlSwitch->Right == 1)//左中右上（小陀螺）
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Tuoluo;
	
		Ship_ChassisWorkMode_Tuoluo(10.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,
															10.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value);
	}
	else if(DR16_Export_Data.ControlSwitch->Left == 2 && DR16_Export_Data.ControlSwitch->Right == 2)//双下（失能）
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Disable;

		Robot_control_chassis_disable();	
		Robot_control_dial_disable();
		Robot_control_cloud_disable();
	}
	else
	{
		Robot_control_chassis_disable();	
		Robot_control_dial_disable();
		Robot_control_cloud_disable();
		DR16_Export_Data.ChassisWorkMode = 0;
	}
}

/**
  * @brief	遥控器控制机器人
**/
void Robot_Control_Fun()
{
	SupCap.FUN.SendMsg();
	SetRemoteMode();
	SupCap.FUN.Ctrl();
	RemoteControl_Output();
	Robot_control();
}

/**
  * @brief  遥控器控制机器人
**/
void Robot_Control_Disable()
{
	Robot_control_chassis_disable();	
	DR16_Export_Data.ChassisWorkMode = 0;
}


