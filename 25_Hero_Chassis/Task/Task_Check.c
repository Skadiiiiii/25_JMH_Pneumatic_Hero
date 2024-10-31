#include "Task_Check.h" 
#include "M6020_motor.h"
#include "M3508_motor.h"
#include "chassis_control.h"
#include "dr16.h"

/**
  * @brief	����豸״̬
  */
void DEV_Check(void const *argument)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t TimeIncrement = pdMS_TO_TICKS(200); //ÿ200����ǿ�ƽ����ܿ���

	for(;;)
	{
		Check_DR16();//DR16���

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

