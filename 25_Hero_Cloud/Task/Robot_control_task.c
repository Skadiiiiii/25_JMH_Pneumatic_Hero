#include "main.h"
#include "chassis_control.h"
#include "cloud_control.h"
#include "cmsis_os.h"
#include "Robot_control_task.h"


/**
  * @brief    机器人主控制任务
**/
void Robot_Control(void const *argument)
{	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每2毫秒强制进入总控制
	for (;;)
  {
//		Robot_Control_Fun();

		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}

