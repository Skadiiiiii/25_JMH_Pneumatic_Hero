#pragma once

#ifndef __POWER_Limit_CONTROL_H
#define __POWER_Limit_CONTROL_H

#include <stdint.h>
#include "string.h"
#include "math.h"
#include "M3508_motor.h"
#include "M6020_motor.h"
#include "RM_JudgeSystem.h"
#include "AddMath.h"
#include "Power_Meter.h"
#include "PowerController.h"

#define RPM_TO_RAD 0.10471975511965977f;
#define k3_cap 0.819f
#define k3_chassis 6.959f
#define k3_ship 3.166f   
#define k3_wheel 2.974f

typedef struct
{
	float LimitPowerMax; //当前限制的功率大小 单位W    
	float chassis_powerBuff;
	float RealChassisPower;
	M3508s_t  chassis_motor_3508[4];
	M6020s_t  chassis_motor_6020[4];
}Chassis_PowerLimit_t;

typedef enum
{
	RESTRICTED,
	NOT_RESTRICTED
}PowerControlStatus;

typedef struct
{
	float maxPower;
	float currentPower[4];
	float cmdPower[4];
	float k1,k2,k3;
	float Kt;
	float estimatedPower,commandPower;
	float decayCurrent[4];
	PowerControlStatus powerControlStatus;
}Chassis_Power_t;

#define Chassis_M3508_Power_Init 	\
{                                 \
    0,           									\
		{0, 0, 0, 0},                 \
    {0, 0, 0, 0},                 \
		0.22f,                        \
    1.2f,                         \
		2.974f,                       \
		0.01562f,                     \
		0,                            \
    0,                            \
    {0, 0, 0, 0} ,                \
    NOT_RESTRICTED,               \
}

#define Chassis_M6020_Power_Init 	\
{                                 \
    0,           									\
		{0, 0, 0, 0},                 \
    {0, 0, 0, 0},                 \
		0.22f,                        \
    1.2f,                         \
		3.166f,                       \
		0.741f,                       \
		0,                            \
    0,                            \
    {0, 0, 0, 0},                 \
    NOT_RESTRICTED,               \
}

extern Chassis_PowerLimit_t Chassis_PowerLimit;	

extern Chassis_Power_t Chassis_Power_M3508;
extern Chassis_Power_t Chassis_Power_M6020;

void chassis_power_control(Chassis_PowerLimit_t *chassis_power_control);
void updateWheelMaxPower(float MaxPower);
void updateShiplMaxPower(float MaxPower);
void getDecayCurrent(Chassis_Power_t *Chassis_Power);

#endif


