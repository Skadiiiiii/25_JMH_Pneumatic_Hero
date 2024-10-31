#ifndef __M6020_MOTOR_H
#define __M6020_MOTOR_H

#include "bsp_can.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct
{
    int16_t  set_voltage;
		int16_t  set_current;
		int16_t  set_torque;
    float    rotor_angle;
		float    target_rotor_angle;
	
		float    real_rotor_angle;
    int16_t  rotor_speed;
		int16_t  speed_rpm;
    int16_t  torque_current;
    uint8_t  temp;
		uint16_t last_rotor_angle;
		int32_t  turn_count;
		float 	 total_angle;
		float 	 target_total_rotor_angle;
	
	  uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率

		uint8_t state;
	
		float Ship_error;
		float Init_angle;
		float RUD_totalAngle;
		int32_t RUD_turnCount;
} M6020s_t;

extern M6020s_t M6020s_Yaw;
extern M6020s_t M6020s_chassis[4];

void M6020_Yaw_getInfo(Can_Export_Data_t RxMessage);
void M6020_chassis_getInfo(Can_Export_Data_t RxMessage);
void Check_Chassis_6020(void);
void set_M6020_1ff_voltage(CAN_HandleTypeDef *CANx,int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_M6020_2ff_voltage(CAN_HandleTypeDef *CANx,int16_t v1, int16_t v2, int16_t v3, int16_t v4);

#endif
