#include "M6020_motor.h"
#include <stdio.h>
#include "bsp_can.h"

M6020s_t M6020s_Yaw;
M6020s_t M6020s_chassis[4];

/**
  * @brief  ��ȡ��̨Yaw��6020������
  */
void M6020_Yaw_getInfo(Can_Export_Data_t RxMessage)
{
    //������ݣ����ݸ�ʽ���C620���˵����P33
    M6020s_Yaw.last_rotor_angle = M6020s_Yaw.rotor_angle;
    M6020s_Yaw.rotor_angle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
    M6020s_Yaw.rotor_speed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
    M6020s_Yaw.torque_current = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);
    M6020s_Yaw.temp = RxMessage.CAN_RxMessage[6];

    if (M6020s_Yaw.rotor_angle - M6020s_Yaw.last_rotor_angle < -4096)
    {
        M6020s_Yaw.turn_count++;
    }

    if (M6020s_Yaw.last_rotor_angle - M6020s_Yaw.rotor_angle < -4096)
    {
        M6020s_Yaw.turn_count--;
    }
		
    M6020s_Yaw.total_angle = M6020s_Yaw.rotor_angle + (8192 * M6020s_Yaw.turn_count);
		
		M6020s_Yaw.real_rotor_angle = M6020s_Yaw.total_angle * 26.0f / 47.0f;
		
		if(M6020s_Yaw.real_rotor_angle < 0)
		{
			M6020s_Yaw.real_rotor_angle += 8191;
		}
		M6020s_Yaw.real_rotor_angle = (int)M6020s_Yaw.real_rotor_angle % 8191;

    //֡��ͳ�ƣ����ݸ��±�־λ
    M6020s_Yaw.InfoUpdateFrame++;
//    M6020s_Yaw.InfoUpdateFlag = 1;
}

/**
  * @brief  ��ȡ����ת����6020������
  */
void M6020_chassis_getInfo(Can_Export_Data_t RxMessage)
{   
    uint32_t StdId;
    StdId = (int32_t)(RxMessage.CAN_RxHeader.StdId - 0x205);
    //������ݣ����ݸ�ʽ���C620���˵����P33
    M6020s_chassis[StdId].rotor_angle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
    M6020s_chassis[StdId].rotor_speed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
    M6020s_chassis[StdId].torque_current = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);
    M6020s_chassis[StdId].temp = RxMessage.CAN_RxMessage[6];

    //֡��ͳ�ƣ����ݸ��±�־λ
    M6020s_chassis[StdId].InfoUpdateFrame++;
//    M6020s_chassis[StdId].InfoUpdateFlag = 1;
}

/**
  * @brief  ������ת����6020��״̬
  */
void Check_Chassis_6020(void)
{
	for(uint8_t i = 0; i < 4; i++)
	{
		if(M6020s_chassis[i].InfoUpdateFrame < 1)
		{
			M6020s_chassis[i].state = 0;
		}
		else
		{
			M6020s_chassis[i].state = 1;
		}
		M6020s_chassis[i].InfoUpdateFrame = 0;
	}
}

/**
  * @brief  ���ͱ�ʶ��Ϊ0x1ff��6020��Ŀ���ѹֵ
  */
void set_M6020_1ff_voltage(CAN_HandleTypeDef *CANx,int16_t v1, int16_t v2, int16_t v3, int16_t v4)//�βα�ʾ4������ĵ�ѹֵ
{
	CAN_TxHeaderTypeDef tx_header;//�����洢���͵�CAN֡��ͷ����Ϣ������֡ID��֡���͡�֡���ȵ�
  uint8_t             tx_data[8];//�����洢�ӷ��͵�CAN֡�����ݲ���
	
	tx_header.StdId = 0x1ff;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;//���巢�͸�ʽ

  tx_data[0] = v1>>8;//��v1�ĸ�8λ�洢��0
  tx_data[1] =    v1;//��v1�ĵ�8λ�洢��1
  tx_data[2] = v2>>8;
  tx_data[3] =    v2;
  tx_data[4] = v3>>8;
  tx_data[5] =    v3;
  tx_data[6] = v4>>8;
  tx_data[7] =    v4;//���巢������		
	
  HAL_CAN_AddTxMessage(CANx, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); //���������ָ��������Ž�CAN֡���͵���Ӧ������
												//CAN, ��Ϣ��ʽ��      ��Ϣ���ݣ�  ָ������ŵ�ָ��
}

/**
  * @brief  ���ͱ�ʶ��Ϊ0x2ff��6020��Ŀ���ѹֵ
  */
void set_M6020_2ff_voltage(CAN_HandleTypeDef *CANx,int16_t v1, int16_t v2, int16_t v3, int16_t v4)//�βα�ʾ4������ĵ�ѹֵ
{
	CAN_TxHeaderTypeDef tx_header;//�����洢���͵�CAN֡��ͷ����Ϣ������֡ID��֡���͡�֡���ȵ�
  uint8_t             tx_data[8];//�����洢�ӷ��͵�CAN֡�����ݲ���
	
	tx_header.StdId = 0x2ff;
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;//���巢�͸�ʽ

  tx_data[0] = v1>>8;//��v1�ĸ�8λ�洢��0
  tx_data[1] =    v1;//��v1�ĵ�8λ�洢��1
  tx_data[2] = v2>>8;
  tx_data[3] =    v2;
  tx_data[4] = v3>>8;
  tx_data[5] =    v3;
  tx_data[6] = v4>>8;
  tx_data[7] =    v4;//���巢������		
	
  HAL_CAN_AddTxMessage(CANx, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); //���������ָ��������Ž�CAN֡���͵���Ӧ������
												//CAN, ��Ϣ��ʽ��      ��Ϣ���ݣ�  ָ������ŵ�ָ��
}


