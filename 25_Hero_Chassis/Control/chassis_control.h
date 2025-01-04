#pragma once

#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "main.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "cloud_control.h"
#include "power_limit_control.h"
#include "M3508_motor.h"
#include "M6020_motor.h"
#include "pid.h"
#include <math.h>
#include "arm_math.h"

#define WorkMode_Cloud 12
#define WorkMode_Chassis 21
#define WorkMode_Follow 22
#define WorkMode_Tuoluo 23
#define WorkMode_Shoot 32
#define WorkMode_Disable 11

#define RF_206_6020 1 //右前
#define LF_207_6020 2 //左前
#define LB_208_6020 3 //左后
#define RB_205_6020 0 //右后

#define RF_202_3508 1 //右前
#define LF_203_3508 2 //左前
#define LB_204_3508 3 //左后
#define RB_201_3508 0 //右后

#define RF_206_6020_Init_Angle 162.266 // 3692/22.7527
#define RB_205_6020_Init_Angle 74.057 // 1685/22.7527
#define LB_208_6020_Init_Angle 163.673  // 3724/22.7527
#define LF_207_6020_Init_Angle 71.332 // 1623/22.7527


#define RUD_OPSI       1
#define RUD_NOT_OPSI   0
#define RUD_RESET      1
#define RUD_NOT_RESET  0

#define M6020_mAngleRatio 22.7527f //机械角度与真实角度的比率
#define DEG_TO_RAD 0.017453292519943295769236907684886f

/* --- 转向轮电机相关参数 -------------------------------------------------------*/
typedef struct 
{
    float Init_angle;   // 初始化校准角度
    float Target_angle; // 目标角度
    float PreTar_angle; // 前一次目标角度
    float Total_angle;  // 当前总角度
    int32_t Turns_cnt;
    int32_t TarTurns_cnt;
    int32_t Turns_flag;
}RUD_Param_t;

extern int16_t speed_buff[4];		/*<! 驱动轮3508目标转速 */
extern RUD_Param_t RUD_Param[4]; /*<! 转向轮6020相关参数 */

void Chassis_Init(void);
void Ship_ChassisWorkMode(float Vx, float Vy,float VOmega);
void Ship_ChassisWorkMode_follow(float Vx, float Vy);
void Ship_ChassisWorkMode_Tuoluo(float Vx, float Vy);
void Robot_control_chassis_disable();

#endif
