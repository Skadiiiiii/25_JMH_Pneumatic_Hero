#include "Power_Meter.h"

/*************************************   超电   **********************************/ 

PowerMeter_Meg_t Power_Meter;

/**
  * @brief  检查功率计是否离线
  */
void Check_Power_Meter(void)
{
	for(uint8_t i = 0; i < 4; i++)
	{
		if(Power_Meter.infoUpdateFrame < 1)
		{
			Power_Meter.OffLineFlag = 1;
		}
		else
		{
			Power_Meter.OffLineFlag = 0;
		}
		Power_Meter.infoUpdateFrame = 0;
	}
}

/**
  * @brief  电容数据更新
  * @param	can_rx_data
  * @retval None
  */
void Power_meter_getInfo(Can_Export_Data_t RxMessage)
{
#if INA226_Self
  memcpy(Power_Meter.Self_RecvData.data, RxMessage.CAN_RxMessage, 8);
	Power_Meter.Chassis_RealPower = Power_Meter.Self_RecvData.Pack.Shunt_Current / 1000 * Power_Meter.Self_RecvData.Pack.voltageVal/1000; 
#else
  RecvData.data[0] = can_rx_data[1];
	RecvData.data[1] = can_rx_data[0];
  RecvData.data[2] = can_rx_data[3];
  RecvData.data[3] = can_rx_data[2];
  RecvData.data[4] = can_rx_data[5];
  RecvData.data[5] = can_rx_data[4];
  RecvData.data[6] = can_rx_data[7];
  RecvData.data[7] = can_rx_data[6];
#endif
	Power_Meter.infoUpdateFrame++;
}

/**
  * @brief  获取底盘功率 W
  * @param	None
  * @retval cap data
  */
float Get_Power_Val()
{
#if INA226_Self
  return Get_Shunt_Current()*Get_voltageVal();
#else
	return (float)RecvData.Pack.Power_Val/1000;
#endif
}
/**
  * @brief  获取底盘电压 V
  * @param	None
  * @retval cap data
  */
float Get_voltageVal()
{
#if INA226_Self
  return (float)Power_Meter.Self_RecvData.Pack.voltageVal/1000;
#else
	return (float)RecvData.Pack.voltageVal/1000;
#endif
}
/**
  * @brief  获取底盘电流 A
  * @param	None
  * @retval cap data
  */
float Get_Shunt_Current()
{
#if INA226_Self
  return (float)Power_Meter.Self_RecvData.Pack.Shunt_Current/1000;
#else
	return (float)RecvData.Pack.Shunt_Current/1000;
#endif
}
/**
  * @brief  获取底盘分流电压 mV
  * @param	None
  * @retval cap data
  */
float Get_Shunt_voltage()
{
#if INA226_Self
  return NULL;
#else
  return (float)RecvData.Pack.Shunt_voltage/1000;
#endif	
}


