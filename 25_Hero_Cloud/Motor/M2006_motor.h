#ifndef __M2006_MOTOR_H
#define __M2006_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_can.h"
 
typedef struct
{
    int16_t  set_voltage;
		int16_t  set_current;
    uint16_t rotor_angle;
    float  rotor_speed;
    float  torque_current;
    uint8_t  temp;
	
	  uint8_t InfoUpdateFlag;   //信息读取更新标志
    uint16_t InfoUpdateFrame; //帧率
} M2006s_t;

extern M2006s_t M2006s;

void set_M2006_200_current(CAN_HandleTypeDef *CANx,int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_M2006_1ff_current(CAN_HandleTypeDef *CANx,int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void M2006_shoot_getInfo(Can_Export_Data_t RxMessage);

#endif
