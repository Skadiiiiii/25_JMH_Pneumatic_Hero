#include "cmsis_os.h"
#include "can_send_task.h"
#include "bsp_can.h"
#include "dr16.h"
#include "M6020_motor.h"
#include "M3508_motor.h"
#include "chassis_control.h"

DR16_data_t can1_send_data;
uint8_t DR16_Date[8];

/**
  * @brief   can发送遥控器以及电机状态
  */
static void DR16_0x175_Can1_SendData(uint8_t *data)
{
	CAN_TxHeaderTypeDef TxMessage;
	
  TxMessage.IDE = CAN_ID_STD;     //设置ID类型
	TxMessage.RTR = CAN_RTR_DATA;   //设置传送数据帧
	TxMessage.DLC = 0x08;           //设置数据长度
	TxMessage.StdId = 0x175;        //设置ID号
	
	can1_send_data.pack.RF_3508 = M3508s_chassis[RF_201_3508].state;
	can1_send_data.pack.LF_3508 = M3508s_chassis[LF_202_3508].state;
	can1_send_data.pack.RB_3508 = M3508s_chassis[RB_203_3508].state;
	can1_send_data.pack.LB_3508 = M3508s_chassis[LB_204_3508].state;
	
	can1_send_data.pack.RF_6020 = M6020s_chassis[RF_205_6020].state;
	can1_send_data.pack.LF_6020 = M6020s_chassis[LF_206_6020].state;
	can1_send_data.pack.RB_6020 = M6020s_chassis[RB_207_6020].state;
	can1_send_data.pack.LB_6020 = M6020s_chassis[LB_208_6020].state;
	
	can1_send_data.pack.WorkMode = DR16_Export_Data.ChassisWorkMode;
	
	can1_send_data.pack.Omega_Value = DR16_Export_Data.Robot_TargetValue.Omega_Value;
	can1_send_data.pack.Pitch_Value = DR16_Export_Data.Robot_TargetValue.Pitch_Value;
	
	memcpy(data, &can1_send_data.data, sizeof(can1_send_data.data));
		
  HAL_CAN_AddTxMessage(&hcan1,&TxMessage,data,0);
}


/**
  * @brief   A板can发送给C板
  */
static void Can_Send_Fun()
{
	DR16_0x175_Can1_SendData(DR16_Date);
}


/**
  * @brief    can板间通讯函数
  */
void CAN_Send(void const *argument)
{	
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每2毫秒强制进入总控制
	for (;;)
  {
		Can_Send_Fun();
		
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}


