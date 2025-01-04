#include "chassis_control.h"
#include "cloud_control.h"
#include "dr16.h"
#include "bsp_can.h"
#include "M3508_motor.h"
#include "M6020_motor.h"
#include "pid.h"
#include "math.h"
#include "DJI_IMU.h"
#include "iir_filter.h"

uint8_t imu_start_flag;

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

///**
// * @brief 获取开机后的陀螺仪数据，并设为基准
// */
//static void read_start_imu(void)
//{
//	float imu_yaw_start_angle;
//	float imu_pitch_start_angle;
//	if(imu_start_flag == 0)
//	{
//		imu_yaw_start_angle = 22.7527f * DJI_C_IMU.total_yaw;
//		M6020s_Yaw.target_rotor_angle   = imu_yaw_start_angle;

//		imu_pitch_start_angle = 4196;
//		M6020s_Pitch.target_rotor_angle = imu_pitch_start_angle;

//		imu_start_flag = 1;
//	}
//}

///**
//  * @brief  云台使能		
//  */
//static void Ship_ChassisWorkMode_cloud(float delta_yaw,float delta_pitch)
//{
//	read_start_imu();

//	M6020s_Yaw.target_rotor_angle   += delta_yaw;
//	M6020s_Pitch.target_rotor_angle += delta_pitch;
//		
//	if(M6020s_Pitch.target_rotor_angle < M6020s_Pitch_Angle_Min)
//	{
//		M6020s_Pitch.target_rotor_angle = M6020s_Pitch_Angle_Min;
//	}
//	else if(M6020s_Pitch.target_rotor_angle > M6020s_Pitch_Angle_Max)
//	{
//		M6020s_Pitch.target_rotor_angle = M6020s_Pitch_Angle_Max;
//	}

//	M6020s_Yaw.set_voltage   = pid_CascadeCalc_yaw(&motor_pid_Cas_Yaw, M6020s_Yaw.target_rotor_angle,22.7527f * DJI_C_IMU.total_yaw,M6020s_Yaw.rotor_speed);
//	
//	M6020s_Pitch.set_voltage = pid_CascadeCalc_pitch(&motor_pid_Cas_Pitch, M6020s_Pitch.target_rotor_angle,22.7527f * DJI_C_IMU.pitch,M6020s_Pitch.rotor_speed);
//	
//	set_M6020_2ff_can1_voltage(M6020s_Yaw.set_voltage,0,0,0);
//	set_M6020_2ff_can2_voltage(0,M6020s_Pitch.set_voltage,0,0);
//}

///**
//  * @brief  摩擦轮使能		
//  */
//static void Ship_ChassisWorkMode_shoot()
//{
////	for (uint8_t i = 0; i < 2; i++)
////	{
////		M3508s_shoot[i].set_voltage  = pid_calc_shoot(&motor_pid_shoot[i],2000,M3508s_shoot[i].rotor_speed);
////	}
//	
//	M3508s_shoot[0].set_voltage  = pid_calc_shoot(&motor_pid_shoot[0],8000,M3508s_shoot[0].rotor_speed);
//	M3508s_shoot[1].set_voltage  = pid_calc_shoot(&motor_pid_shoot[1],-8000,M3508s_shoot[1].rotor_speed);

//	set_M3508_shoot_voltage( 
//							M3508s_shoot[0].set_voltage, //shoot
//							M3508s_shoot[1].set_voltage,
//							0,0);
//}

///**
//  * @brief  云台失能
//  */
//static void Robot_control_cloud_disable()
//{

//	M6020s_Yaw.set_voltage = 0; 
//	M6020s_Pitch.set_voltage = 0;
//	for (uint8_t i = 0; i < 2; i++)
//	{
//		M3508s_shoot[i].set_voltage  = 0;
//	}

//	set_M6020_2ff_can1_voltage(0,0,0,0);
//	set_M6020_2ff_can2_voltage(0,0,0,0);
//	set_M3508_shoot_voltage(0,0,0,0);
//}

///**
//  * @brief  机器人主控制
//  */
//void Robot_control ()        
//{
//	if(DR16_Export_Data.ChassisWorkMode == WorkMode_Shoot)
//	{
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
//		Ship_ChassisWorkMode_shoot();
//		Ship_ChassisWorkMode_cloud(-0.03f*DR16_Export_Data.Robot_TargetValue.Omega_Value,0.01f*DR16_Export_Data.Robot_TargetValue.Pitch_Value);

//	}
//	else if(DR16_Export_Data.ChassisWorkMode == WorkMode_Cloud || DR16_Export_Data.ChassisWorkMode == WorkMode_Follow || DR16_Export_Data.ChassisWorkMode == WorkMode_Tuoluo)
//	{
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
//		set_M3508_shoot_voltage(0,0,0,0);
//		Ship_ChassisWorkMode_cloud(-0.03f*DR16_Export_Data.Robot_TargetValue.Omega_Value,0.01f*DR16_Export_Data.Robot_TargetValue.Pitch_Value);
//	}

//	else
//	{
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
//		Robot_control_cloud_disable();	
//		imu_start_flag = 0;
//		
//		ClearFilter(yaw_speed_filter,3);
////		ClearFilter(pitch_speed_filter,3);
//	}
//}

///**
//  * @brief  遥控器控制机器人
//**/
//void Robot_Control_Fun(void)
//{
//	Robot_control();
//}


