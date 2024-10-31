#include "Task_CanMsg.h"
#include "M3508_Motor.h"
#include "M6020_Motor.h"
#include "M2006_Motor.h"

/**
  * @brief  can1接收任务
  */
void CAN1_REC(void const *argument)
{
	Can_Export_Data_t Can_Export_Data;
	uint32_t ID;
	for (;;)
	{
		xQueueReceive(CAN1_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
		ID = Can_Export_Data.CAN_RxHeader.StdId;
		if (ID == M6020_Yaw_ID)
		{
			M6020_Yaw_getInfo(Can_Export_Data);
		}
		else if (ID == M2006_Shoot_ID)
		{
			M2006_shoot_getInfo(Can_Export_Data);
		}
	}
}

/**
  * @brief  can2接收任务
  */
void CAN2_REC(void const *argument)
{
	uint32_t ID;
	Can_Export_Data_t Can_Export_Data;
	for (;;)
	{
		xQueueReceive(CAN2_ReceiveHandle, &Can_Export_Data, portMAX_DELAY);
		ID = Can_Export_Data.CAN_RxHeader.StdId;
		
		if (ID >= M3508_Chassis_Begin_ID && ID <= M3508_Chassis_End_ID)
		{
			M3508_chassis_getInfo(Can_Export_Data);
		}
		else if (ID >= M6020_Chassis_Begin_ID && ID <= M6020_Chassis_End_ID)
		{
			M6020_chassis_getInfo(Can_Export_Data);
		}
//		if(ID == 0x203)
//		{
//			M2006_shoot_getInfo(Can_Export_Data);
//		}
		
	}
}

