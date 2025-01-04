#ifndef __TASK_CANMSG_H
#define __TASK_CANMSG_H
#include "bsp_can.h"

#define M6020_Yaw_ID 0x209
#define M6020_Pitch_ID 0x20A
#define M3508_Shoot_Begin_ID 0x201
#define M3508_Shoot_End_ID 0x202
#define DR16_C_ID 0x175

extern osMessageQId CAN1_ReceiveHandle;
extern osMessageQId CAN2_ReceiveHandle;

#endif 

