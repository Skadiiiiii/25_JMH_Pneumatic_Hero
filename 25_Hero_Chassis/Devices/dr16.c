#include "string.h"
#include "stdlib.h"
#include "dr16.h"
#include "usart.h"
#include "main.h"
#include "can.h"

/*******************************�û����ݶ���************************************/
uint8_t   dbus_buf[DBUS_BUFLEN];
DR16_t DR16 = DR16_GroundInit;
ControlSwitch_t ControlSwitch;
DR16_Export_Data_t DR16_Export_Data = DR16_ExportDataGroundInit;
/*******************************************************************************/


/**
  * @brief  ��ң���������ݸ�ֵ�����Ʊ������������ļ�ʹ��
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
  * @brief  ���ڴ������յ���ң������rc�����ݣ������յ����ֽڽ���
  */
static void rc_callback_handler(DR16_t *DR16, uint8_t *buff)
{
  DR16->rc.ch0 = (buff[0] | buff[1] << 8) & 0x07FF;//��buff[0]��buff[1]��ֵ���Ϊch0ͨ����ֵ��������������11λ��ͨ����0x07FF��λ�룩
  DR16->rc.ch0 -= 1024;//����������364��1684�������������ݼ�ȥ1024��ʹ������ֵΪ0
  DR16->rc.ch1 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  DR16->rc.ch1 -= 1024;
  DR16->rc.ch2 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  DR16->rc.ch2 -= 1024;
  DR16->rc.ch3 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  DR16->rc.ch3 -= 1024;
	
  DR16->rc.roll = (buff[16] | (buff[17] << 8)) & 0x07FF;  //���Ͻǹ���
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
  DR16->rc.sw2 = (buff[5] >> 4) & 0x0003;//sw1��sw2��ֵ�ֱ�����Ӧ��λ����ó�
	
	DR16->infoUpdateFrame++;
	
  if ((abs(DR16->rc.ch0) > 660) || \
      (abs(DR16->rc.ch1) > 660) || \
      (abs(DR16->rc.ch2) > 660) || \
      (abs(DR16->rc.ch3) > 660))
	  
  {
    memset(DR16, 0, sizeof(DR16_t));//�����һͨ���ľ���ֵ����660����ʾ���յ��쳣���ݣ���rc�ṹ���������������
  }		
}

/**
  * @brief  ����DMAԤ����Ļ�����ʣ��ĳ��ȣ������˽⴫������л��ж���������δ����
  */
static uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  return ((uint16_t)(dma_stream->NDTR));
}

/**
  * @brief  �������յ��Ĳ�����������
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	//���UART�Ŀ��б�־���Ա���һ�ν���ʱ�ܹ���ȷ��⵽����״̬
	
	if (huart == &DBUS_HUART)//ȷ��ֻ����DBUS����
	{
		__HAL_DMA_DISABLE(huart->hdmarx);//ʧ��DMA���գ��Ա��ڴ�������ʱ��ֹ���ݱ�����

		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{//���㵱ǰ���յ����ݳ��ȣ�������յ������ݳ��ȵ���18�ֽڣ�����ô������ݺ���
			rc_callback_handler(&DR16, dbus_buf);//�������յ����ݲ�����
		}
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);//����DMA�������ݳ��ȣ��Ա�Ϊ��һ�ν�������׼��
		__HAL_DMA_ENABLE(huart->hdmarx);//��������DMA���գ��Ա������������
	}
}


/**
  * @brief	���UART����״̬���ڽ��յ�����״̬ʱ������Ӧ�Ļص�����
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && //���UART�Ƿ������˿��б�־����ʾUART������ɲ��������״̬
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))//���UART�����ж��Ƿ�ʹ�ܡ�ֻ�����ж�ʹ�ܵ�����£��Żᴦ������״̬
	{
		uart_rx_idle_callback(huart);//����֮ǰ����ĺ������������յ�������
	}
}

/**
  * @brief	����UART��ͨ��DMA��������
  */
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{//���ڽ������ݵĺ�����ʹ��DMA��ʽ������UART����
  uint32_t tmp1 = 0;

  tmp1 = huart->RxState;
	//����һ����ʱ����tmp1������UART�Ľ���״̬��ֵ����
	if (tmp1 == HAL_UART_STATE_READY)//�ж�UART�Ƿ��ھ���״̬
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
		//����DMA���գ�Դ��ַΪUART���ݼĴ�����Ŀ���ַΪ���ջ�����
	
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
		//ʹ��UART��DMA���չ���
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

/**
  * @brief	DBUS���ڳ�ʼ��
  */
void dbus_uart_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);//�������UART�Ŀ��б�־�����б�־ָʾUART�ڽ�������ʱ���ڿ���״̬��ͨ���ڽ�����ɺ�����
	//��������־��Ϊ��ȷ�������Ľ��ղ����ܹ���ȷ��⵽�µĿ���״̬
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);//ʹ��UART�Ŀ����жϣ���UART���ڿ���״̬���ҽ��ջ�����û������ʱ���ᴥ������ж�
	//ʹ������жϺ󣬿������жϷ��������д�������״̬
	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}

/**
  * @brief	���ң��������״̬
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


