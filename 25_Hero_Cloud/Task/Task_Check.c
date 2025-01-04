#include "Task_Check.h" 
#include "M6020_motor.h"
#include "M3508_motor.h"

void DEV_Check(void const *argument)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(200); //每200毫秒强制进入总控制

	for(;;)
	{

//		Check_Cloud_6020();
//		Check_Shoot_3508();

		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}

