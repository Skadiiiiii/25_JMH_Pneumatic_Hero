#include "Task_Check.h" 
#include "M6020_motor.h"
#include "M3508_motor.h"
#include "chassis_control.h"
#include "dr16.h"

/**
  * @brief	检查设备状态
  */
void DEV_Check(void const *argument)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(200); //每200毫秒强制进入总控制

	for(;;)
	{
		Check_DR16();//DR16检测

		if(DR16.OffLineFlag == 1)
		{
			DR16_Export_Data.Alldead = 1;
		}
		else
		{
			DR16_Export_Data.Alldead = 0;
		}
		
		Check_Chassis_6020();
		Check_Chassis_3508();
		
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}

