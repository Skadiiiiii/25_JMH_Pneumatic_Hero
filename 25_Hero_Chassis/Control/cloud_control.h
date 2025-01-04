#pragma once

#ifndef CLOUD_CONTROL_H
#define CLOUD_CONTROL_H

#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include "DM4310_motor.h"
#include "pid.h"

int ComputeMinOffset(int target, int value);
float Turn_InferiorArc(float target, float current);
void Ship_ChassisWorkMode_cloud(float delta_yaw);
void Robot_control_cloud_disable(void);

#endif
