#include "main.h"
#include "chassis_control.h"
#include "cmsis_os.h"
#include "Robot_control_task.h"
#include "dr16.h"

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
		if(DR16_Export_Data.Alldead == 1)
		{
			Robot_Control_Disable();
		}
		else
		{
			Robot_Control_Fun();
		}

		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}

