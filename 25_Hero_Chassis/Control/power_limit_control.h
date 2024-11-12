#ifndef __POWER_Limit_CONTROL_H
#define __POWER_Limit_CONTROL_H

#include <stdint.h>
#include "string.h"
#include "math.h"
#include "M3508_motor.h"
#include "M6020_motor.h"

typedef struct
{
	float LimitPowerMax; //当前限制的功率大小 单位W    
	float chassis_powerBuff;
	float RealChassisPower;
	M3508s_t  chassis_motor_3508[4];
	M6020s_t  chassis_motor_6020[4];

}Chassis_PowerLimit_t;

extern Chassis_PowerLimit_t Chassis_PowerLimit;	

void chassis_power_control(Chassis_PowerLimit_t *chassis_power_control);

#endif


