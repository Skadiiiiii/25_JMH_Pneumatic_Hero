#include "string.h"
#include "stdlib.h"
#include "dr16.h"
#include "usart.h"
#include "main.h"
#include "can.h"

/*******************************用户数据定义************************************/
uint8_t   dbus_buf[DBUS_BUFLEN];
DR16_t DR16 = DR16_GroundInit;
ControlSwitch_t ControlSwitch;
DR16_Export_Data_t DR16_Export_Data = DR16_ExportDataGroundInit;
/*******************************************************************************/


/**
  * @brief  将遥控器的数据赋值给控制变量，供其他文件使用
  */
void RemoteControl_Output(void)
{
	DR16_Export_Data.ControlSwitch->Left  = (RemotePole_e)DR16.rc.sw1;
	DR16_Export_Data.ControlSwitch->Right = (RemotePole_e)DR16.rc.sw2;
	DR16_Export_Data.Robot_TargetValue.Forward_Back_Value = DR16.rc.ch3;
	DR16_Export_Data.Robot_TargetValue.Left_Right_Value = DR16.rc.ch2;
	DR16_Export_Data.Robot_TargetValue.Yaw_Value	= DR16.rc.ch0;
	DR16_Export_Data.Robot_TargetValue.Pitch_Value = DR16.rc.ch1;
	DR16_Export_Data.Robot_TargetValue.Omega_Value = DR16.rc.roll;
}

/**
  * @brief  用于处理接收到的遥控器（rc）数据，将接收到的字节解码
  */
static void rc_callback_handler(DR16_t *DR16, uint8_t *buff)
{
  DR16->rc.ch0 = (buff[0] | buff[1] << 8) & 0x07FF;//将buff[0]和buff[1]的值组合为ch0通道的值，并将其限制在11位（通过与0x07FF按位与）
  DR16->rc.ch0 -= 1024;//由于数据在364到1684，将解码后的数据减去1024，使其中心值为0
  DR16->rc.ch1 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  DR16->rc.ch1 -= 1024;
  DR16->rc.ch2 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  DR16->rc.ch2 -= 1024;
  DR16->rc.ch3 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  DR16->rc.ch3 -= 1024;
	
  DR16->rc.roll = (buff[16] | (buff[17] << 8)) & 0x07FF;  //左上角滚轮
  DR16->rc.roll -= 1024;
	
	DR16->mouse.x = ((int16_t)buff[6]) | ((int16_t)buff[7] << 8);
	DR16->mouse.y = ((int16_t)buff[8]) | ((int16_t)buff[9] << 8);
	DR16->mouse.z = ((int16_t)buff[10]) | ((int16_t)buff[11] << 8);

	DR16->mouse.keyLeft  = buff[12];
	DR16->mouse.keyRight = buff[13];
	DR16->keyBoard.key_code = ((int16_t)buff[14]) | ((int16_t)buff[15] << 8);
	
	if(DR16->rc.ch0 <= 5 && DR16->rc.ch0 >= -5)
    DR16->rc.ch0 = 0;
	if(DR16->rc.ch1 <= 5 && DR16->rc.ch1 >= -5)
    DR16->rc.ch1 = 0;
  if(DR16->rc.ch2 <= 5 && DR16->rc.ch2 >= -5)
    DR16->rc.ch2 = 0;
  if(DR16->rc.ch3 <= 5 && DR16->rc.ch3 >= -5)
    DR16->rc.ch3 = 0;


  DR16->rc.sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  DR16->rc.sw2 = (buff[5] >> 4) & 0x0003;//sw1和sw2的值分别由相应的位计算得出
	
	DR16->infoUpdateFrame++;
	
  if ((abs(DR16->rc.ch0) > 660) || \
      (abs(DR16->rc.ch1) > 660) || \
      (abs(DR16->rc.ch2) > 660) || \
      (abs(DR16->rc.ch3) > 660))
	  
  {
    memset(DR16, 0, sizeof(DR16_t));//如果任一通道的绝对值超过660，表示接收到异常数据，则将rc结构体的所有内容清零
  }		
}

/**
  * @brief  返回DMA预定义的缓冲区剩余的长度，方便了解传输过程中还有多少数据尚未传输
  */
static uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  return ((uint16_t)(dma_stream->NDTR));
}

/**
  * @brief  处理接收到的不定长度数据
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	//清除UART的空闲标志，以便下一次接收时能够正确检测到空闲状态
	
	if (huart == &DBUS_HUART)//确保只处理DBUS串口
	{
		__HAL_DMA_DISABLE(huart->hdmarx);//失能DMA接收，以便在处理数据时防止数据被覆盖

		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{//计算当前接收的数据长度，如果接收到的数据长度等于18字节，则调用处理数据函数
			rc_callback_handler(&DR16, dbus_buf);//处理接收的数据并解码
		}
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);//设置DMA接收数据长度，以便为下一次接收做好准备
		__HAL_DMA_ENABLE(huart->hdmarx);//重新启用DMA接收，以便继续接收数据
	}
}


/**
  * @brief	检查UART接收状态并在接收到空闲状态时调用相应的回调函数
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && //检查UART是否设置了空闲标志，表示UART接收完成并进入空闲状态
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))//检查UART空闲中断是否被使能。只有在中断使能的情况下，才会处理空闲状态
	{
		uart_rx_idle_callback(huart);//调用之前定义的函数，处理接收到的数据
	}
}

/**
  * @brief	配置UART以通过DMA接收数据
  */
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{//用于接收数据的函数，使用DMA方式来接收UART数据
  uint32_t tmp1 = 0;

  tmp1 = huart->RxState;
	//创建一个临时变量tmp1，并将UART的接收状态赋值给它
	if (tmp1 == HAL_UART_STATE_READY)//判断UART是否处于就绪状态
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
		//启动DMA接收，源地址为UART数据寄存器，目标地址为接收缓冲区
	
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
		//使能UART的DMA接收功能
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

/**
  * @brief	DBUS串口初始化
  */
void dbus_uart_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);//用于清除UART的空闲标志，空闲标志指示UART在接收数据时处于空闲状态，通常在接收完成后设置
	//清除这个标志是为了确保后续的接收操作能够正确检测到新的空闲状态
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);//使能UART的空闲中断，当UART处于空闲状态并且接收缓冲区没有数据时，会触发这个中断
	//使能这个中断后，可以在中断服务例程中处理空闲状态
	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}

/**
  * @brief	检查遥控器离线状态
  */
void Check_DR16(void)
{
	if(DR16.infoUpdateFrame < 1)
	{
		DR16.OffLineFlag = 1;
	}
	else
	{
		DR16.OffLineFlag = 0;
	}
	DR16.infoUpdateFrame = 0;
}



