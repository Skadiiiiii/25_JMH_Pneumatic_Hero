#include "FreeRTOSConfig.h"
#include "IMU_Send_task.h"
#include "calibrate_task.h"
#include "INS_task.h"
#include "DJI_IMU.h"
#include "dr16.h"
#include "bsp_can.h"

void IMU_Send(void const * argument)
{
  /* USER CODE BEGIN IMU_Send */
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1);  //每十毫秒强制进入总控制
  /* Infinite loop */
  for(;;)
  {
		//欧拉角 
		if(cali_sensor[0].cali_done == CALIED_FLAG && cali_sensor[0].cali_cmd == 0)
		{
			Euler_Send_Fun(); //更新陀螺仪角度
			
		}
    vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
  }
  /* USER CODE END IMU_Send */
}



