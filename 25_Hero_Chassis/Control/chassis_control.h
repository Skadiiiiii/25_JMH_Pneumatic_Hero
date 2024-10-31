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

#define RF_205_6020 0 //��ǰ
#define LF_206_6020 1 //��ǰ
#define RB_207_6020 2 //�Һ�
#define LB_208_6020 3 //���

#define RF_201_3508 0 //��ǰ
#define LF_202_3508 1 //��ǰ
#define RB_203_3508 2 //�Һ�
#define LB_204_3508 3 //���

#define LF_206_6020_Init_Angle -3
#define RF_205_6020_Init_Angle 2
#define RB_207_6020_Init_Angle 3
#define LB_208_6020_Init_Angle 61

#define RUD_OPSI       1
#define RUD_NOT_OPSI   0
#define RUD_RESET      1
#define RUD_NOT_RESET  0

#define M6020_mAngleRatio 22.7527f //��е�Ƕ�����ʵ�Ƕȵı���
#define DEG_TO_RAD 0.017453292519943295769236907684886f

/* --- ת���ֵ����ز��� -------------------------------------------------------*/
typedef struct 
{
    float Init_angle;   // ��ʼ��У׼�Ƕ�
    float Target_angle; // Ŀ��Ƕ�
    float PreTar_angle; // ǰһ��Ŀ��Ƕ�
    float Total_angle;  // ��ǰ�ܽǶ�
    int32_t Turns_cnt;
    int32_t TarTurns_cnt;
    int32_t Turns_flag;
}RUD_Param_t;

float abs(float num);
void Chassis_Init(void);
void Robot_Control_Disable(void);
void Robot_Control_Fun(void);

#endif
