#ifndef __M3508_MOTOR_H
#define __M3508_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_can.h"
 
typedef struct
{
    int16_t  set_voltage;
		int16_t  set_current;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
		int16_t  target_rotor_speed;
		int16_t  speed_rpm;
    int16_t  torque_current;
    uint8_t  temp;
		
	  uint8_t  InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
		uint8_t  state;
} M3508s_t;

extern M3508s_t M3508s_chassis[4];

void set_M3508_200_voltage(CAN_HandleTypeDef *CANx,int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_M3508_1ff_voltage(CAN_HandleTypeDef *CANx,int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void M3508_chassis_getInfo(Can_Export_Data_t RxMessage);
void Check_Chassis_3508(void);

#endif
