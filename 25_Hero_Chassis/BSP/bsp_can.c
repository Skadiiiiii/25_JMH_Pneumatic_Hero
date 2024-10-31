#include "bsp_can.h"
#include "DJI_IMU.h"
#include "M6020_motor.h"
#include "M2006_motor.h"
#include "M3508_motor.h"

Can_Data_t Can_Data[2] = Can_DataGroundInit;
Can_Export_Data_t Can1_Export_Data;//用来存储接收到的CAN1数据
Can_Export_Data_t Can2_Export_Data;//用来存储接收到的CAN2数据

/**
  * @brief	CAN1过滤器初始化
  */
static void CAN_1_Filter_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;
    
  sFilterConfig.FilterBank = 0;                       //CAN过滤器编号，范围0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN过滤器模式，掩码模式或列表模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN过滤器尺度，16位或32位
  sFilterConfig.FilterIdHigh = 0x0000;			//32位下，存储要过滤ID的高16位
  sFilterConfig.FilterIdLow = 0x0000;					//32位下，存储要过滤ID的低16位
  sFilterConfig.FilterMaskIdHigh = 0x0000;			//掩码模式下，存储的是掩码
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;				//报文通过过滤器的匹配后，存储到哪个FIFO
  sFilterConfig.FilterActivation = ENABLE;    		//激活过滤器
  sFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	HAL_CAN_Start(&hcan1) ;
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief	CAN2过滤器初始化
  */
static void CAN_2_Filter_Config(void)
{
  CAN_FilterTypeDef  sFilterConfig;
    
  sFilterConfig.FilterBank = 14;                       //CAN过滤器编号，范围0-27
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   //CAN过滤器模式，掩码模式或列表模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  //CAN过滤器尺度，16位或32位
  sFilterConfig.FilterIdHigh = 0x0000;			//32位下，存储要过滤ID的高16位
  sFilterConfig.FilterIdLow = 0x0000;					//32位下，存储要过滤ID的低16位
  sFilterConfig.FilterMaskIdHigh = 0x0000;			//掩码模式下，存储的是掩码
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;				//报文通过过滤器的匹配后，存储到哪个FIFO
  sFilterConfig.FilterActivation = ENABLE;    		//激活过滤器
	
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	HAL_CAN_Start(&hcan2) ;
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief	CAN初始化
  */
void bsp_can_init(void)
{
	CAN_1_Filter_Config();
	CAN_2_Filter_Config();
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //  CAN_IT_FMP0
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); //  CAN_IT_FMP0
}

/**
  * @brief	处理CAN接收到的数据并发送到队列
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//CAN总线接收回调函数
{
	if(hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can1_Export_Data.CAN_RxHeader, Can1_Export_Data.CAN_RxMessage); //从CAN总线接收数据
		
		xQueueSendToBackFromISR(CAN1_ReceiveHandle, &Can1_Export_Data, 0);//将数据发送到FreeRTOS消息队列
  }
	
  else if(hcan->Instance == CAN2)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can2_Export_Data.CAN_RxHeader, Can2_Export_Data.CAN_RxMessage); //从CAN总线接收数据
		
		xQueueSendToBackFromISR(CAN2_ReceiveHandle, &Can2_Export_Data, 0);//将数据发送到FreeRTOS消息队列
  }
}


	
