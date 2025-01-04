#pragma once

#ifndef SHOOT_CONTROL_H
#define SHOOT_CONTROL_H

#include "main.h"
#include "cmsis_os.h"
#include "M3508_motor.h"
#include "pid.h"

void Ship_ChassisWorkMode_shoot();
void Ship_ChassisWorkMode_shoot_stop();
void Robot_control_dial_disable();

#endif