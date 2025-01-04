#pragma once

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "main.h"
#include "cmsis_os.h"

typedef enum
{
	RemoteControl	=	1,	//遥控器模式
	KeyMouseControl =	2,//键鼠模式
	Stop_car = 3  			//关闭机器
}RemoteMode_e;

void Robot_Control_Fun();
void Robot_Control_Disable();

#endif