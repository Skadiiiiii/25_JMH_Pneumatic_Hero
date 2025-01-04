#ifndef __DM_4310_MOTOR_H
#define __DM_4310_MOTOR_H
#include "main.h"
#include "bsp_can.h"
#include "pid.h"

#define 	DM_Send_ID 0x215
#define  	DM_Receive_ID 0x225
#define   Radio_toAngle 0.0174444444444444f

typedef struct
{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float position;
	float last_position;
	float speed;
	float torque;
	float target_rotor_angle;
	float Kp;
	float Kd;
	float Temp_mos;
	float Temp_rotor;
}DM_J4310_t;

extern DM_J4310_t DM4310s_yaw;

#define P_MIN  -3.14		//位置最小值
#define P_MAX	 3.14		//位置最大值
#define V_MIN -30			//速度最小值
#define V_MAX  30			//速度最大值
#define KP_MIN 0.0		//Kp最小值
#define KP_MAX 500.0	//Kp最大值
#define KD_MIN 0.0		//Kd最小值
#define KD_MAX 5.0		//Kd最大值
#define T_MIN -10			//转矩最大值
#define T_MAX 10			//转矩最小值


void motor_enable(void);
void motor_disable(void);
void motor_Erase_error(void);
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq);
void PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel);
void DM_4310_getInfo(Can_Export_Data_t RxMessage);

#endif
