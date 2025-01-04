 #ifndef __DR16_REMOTE__H__
#define __DR16_REMOTE__H__

#include "bsp_can.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart3


typedef struct
{
    struct
    {
        float Forward_Back_Value; //Vx
        float Omega_Value;        //自旋值。
        float Left_Right_Value;   //Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel; //拨轮
    } Robot_TargetValue;  //遥控计算比例后的运动速度
    uint16_t infoUpdateFrame; //帧率
    uint8_t OffLineFlag;      //设备离线标志 
		uint8_t Alldead;
		int16_t ChassisWorkMode;
} DR16_Export_Data_t;         //供其他文件使用的输出数据。



typedef struct
{
	uint64_t LF_6020:1;
	uint64_t RF_6020:1;
	uint64_t LB_6020:1;
	uint64_t RB_6020:1;
	
	uint64_t LF_3508:1;
	uint64_t RF_3508:1;
	uint64_t LB_3508:1;
	uint64_t RB_3508:1;
	
	uint64_t WorkMode:8;
	
	uint64_t :12;
	uint64_t :12;
	int64_t  Omega_Value:12;
	int64_t  Pitch_Value:12;
}DR_data_t;

typedef union
{
	uint8_t data[8];
	DR_data_t pack;
}DR16_data_t;

//# pragma pack(1)
//typedef struct 
//{	
//	uint16_t a: 2;
//	uint16_t b: 2;
//	uint16_t c: 15;
//}DR_data;
//#pragma pack()

//# pragma pack(1)
//typedef struct __attribute__((packed)) 
//{
//	uint16_t a;
//	uint16_t b;
//	uint16_t c;
//}DR_data;
//#pragma pack()

#define RF_205_6020 0 //左前
#define LF_206_6020 1 //右前
#define RB_207_6020 2 //右后
#define LB_208_6020 3 //左后

#define RF_201_3508 0 //右前
#define LF_202_3508 1 //左前
#define RB_203_3508 2 //右后
#define LB_204_3508 3 //左后

extern uint8_t M6020_state[4];
extern uint8_t M3508_state[4];
extern DR16_Export_Data_t DR16_Export_Data;

extern int size;
		
void dr16_getInfo(uint8_t can_rx_data[]);

#endif

