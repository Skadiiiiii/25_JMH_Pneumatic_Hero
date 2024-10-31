#ifndef __SUPCAP_H
#define __SUPCAP_H


#include <stdint.h>
#include "bsp_can.h"
#include <stdbool.h>

#define SCCM_RECEIVE_ID 0x600
#define SCCM_SEND_ID 0x601

#define SupCap_ON 1
#define SupCap_OFF 0
#define Charging_ON 1
#define Charging_OFF 0
#define Power_Supply 1
#define Power_NotSupply 0

typedef union
{
	uint8_t data[8];
	
	struct
	{
			float chassis_power;  /* 底盘功率，单位：W */
			uint8_t chassis_buff; /* 底盘功率缓冲 */
			uint8_t Is_enable;    /* 电容可以进行输出 */
			uint8_t cap_cell;     /* 电容剩余电量，会出现负数 */
	}Pack;
		
} SCCM_RecvData_u;

typedef union
{
	uint8_t data[8];
	
	struct
	{
			float charge_power;    /* 充电功率，单位：W ,范围 0-80W */
			uint8_t charge_enable; /* 充电使能 */
			uint8_t is_cap_output; /* 使用电容供电 */
	}Pack;

} SCCM_SendData_u;

typedef struct
{
    uint8_t Data[8];
    uint8_t State;
    uint16_t ChassisPower_Limit;
    float Charging_Power;      // 充电的功率
    SCCM_RecvData_u RecvData;
    SCCM_SendData_u SendData;
} SupCap_classdef;

typedef struct  
{
		float infoUpdateFrame;
		float OffLineFlag;
}SupCap_t;


void SupCap_Update(Can_Export_Data_t RxMessage);
#endif
