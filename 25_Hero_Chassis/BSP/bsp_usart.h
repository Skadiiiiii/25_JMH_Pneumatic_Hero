#ifndef _BSP_USART_H__
#define _BSP_USART_H__

#include "main.h"

#include "usart.h"

#define Rc_BuffSIZE 18+2
#define Vision_BuffSIZE (13+2) 								//视觉数据缓冲区长度

extern uint8_t DBUS_DataBuf[Rc_BuffSIZE];

int USART_Receive_DMA_NO_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);

#endif

