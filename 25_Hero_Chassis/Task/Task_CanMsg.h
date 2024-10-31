#ifndef __TASK_CANMSG_H
#define __TASK_CANMSG_H
#include "bsp_can.h"


#define DJI_C_IMU_ID 0x185
#define M6020_Yaw_ID 0x209
#define M3508_Chassis_Begin_ID 0x201
#define M3508_Chassis_End_ID 0x204
#define M6020_Chassis_Begin_ID 0x205
#define M6020_Chassis_End_ID 0x208
#define M2006_Shoot_ID 0x201

extern osMessageQId CAN1_ReceiveHandle;
extern osMessageQId CAN2_ReceiveHandle;

#endif 

