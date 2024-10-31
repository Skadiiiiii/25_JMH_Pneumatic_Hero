#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"
#include "cmsis_os.h"
#include "string.h"

typedef struct
{
    CAN_RxHeaderTypeDef CAN_RxHeader;
    uint8_t CAN_RxMessage[8];
} Can_Export_Data_t;

typedef struct
{
//	uint8_t CANx;															//指定哪个CAN
	CAN_RxHeaderTypeDef rx_header;			//CAN 接收缓冲区句柄
	uint8_t             rx_data[1];				//存储CAN接收数据 数组
}CAN_Rx_TypeDef;

typedef struct
{
    struct
    {
        CAN_FilterTypeDef CAN_Filter;
    } CAN_FilterTypedef;

    struct
    {
        CAN_RxHeaderTypeDef CANx_RxHeader;
        uint8_t CANx_RxMessage[8];
    } CAN_RxTypedef;
} Can_Data_t;

#define Can_DataGroundInit \
    {                      \
        {0}, {0},          \
    }
		
#define Can1_Type 1
#define Can2_Type 2
		
extern osMessageQId CAN1_ReceiveHandle;
extern osMessageQId CAN2_ReceiveHandle;

void bsp_can_init(void);
void CAN_RxMessage_Export_Data(CAN_HandleTypeDef *hcanx, osMessageQId CANx_Handle, uint8_t Can_type);


#endif
