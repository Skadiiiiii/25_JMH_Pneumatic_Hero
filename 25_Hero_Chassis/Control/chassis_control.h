#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include "main.h"
#include "cmsis_os.h"

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

#define RF_206_6020_Init_Angle 167 // 3791/22.7527
#define RB_205_6020_Init_Angle 75 // 1720/22.7527
#define LB_208_6020_Init_Angle 163  // 3700/22.7527
#define LF_207_6020_Init_Angle 135 // 3081/22.7527


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

typedef enum
{
	RemoteControl	=	1,	//遥控器模式
	KeyMouseControl =	2,//键鼠模式
	Stop_car = 3  			//关闭机器
}RemoteMode_e;


void Chassis_Init(void);
void Robot_Control_Disable(void);
void Robot_Control_Fun(void);

#endif
