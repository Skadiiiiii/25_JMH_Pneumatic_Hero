#include "SupCap.h"
#include "AddMath.h"
#include "dr16.h"
#include "RM_JudgeSystem.h"
#include "chassis_control.h"

void SCCM_MsgProcess(Can_Export_Data_t RxMessage);
void SCCM_SendMsg(void);
void SuperCapation_Funtion(void);
void SupCap_ChargeControl(float Charging_power);
void SupCap_SupplySwitch(bool Switch);
void SupCap_MainSwitch(bool Cotrol_Switch, bool Charge_Switch, bool Supply_Switch);
void Super_Init(void);

CAN_TxHeaderTypeDef SCCMCanParam;
uint8_t SCCM_Send_Data[8];
SuperCapMode_e s_SuperCapMode = Cap_Close;   	//超级电容状态机
SupCap_t SupCap = SupCap_Init;

/** 
	* @brief  设置超级电容状态
  * @param	void
  * @retval void
  */
void SetSuperCap_Mode(SuperCapMode_e status)
{
  s_SuperCapMode = status;
}

/**
	* @brief  获取超级电容状态
  * @param	void
  * @retval 超级电容状态
  */
SuperCapMode_e GetSuperCap_Mode(void)
{
	return s_SuperCapMode;
}

/**
	* @brief  超级电容初始化函数
  * @param	初始化
  * @retval None
  */
void Super_Init(void)
{

	SupCap.FUN.MainSwitch(SupCap_OFF, Charging_OFF, Power_NotSupply);
	
	SupCap.SCCM_SendData.Charge_Power = SupCap.Charging_Power = 0;
	SupCap.Last_Cap = false;
	
	SetSuperCap_Mode(Cap_Close);
}

/**
  * @brief  检查超电管理模块是否离线
  */
void Check_SupCap(void)
{
	if(SupCap.infoUpdateFrame < 1)
	{
		SupCap.OffLineFlag = 1;
	}
	else
	{
		SupCap.OffLineFlag = 0;
	}
	SupCap.infoUpdateFrame = 0;
}

/**
	* @brief  超级电容接收函数
  * @param	函数buff
  * @retval None
  */
void SCCM_MsgProcess(Can_Export_Data_t RxMessage)
{
	if(RxMessage.CAN_RxHeader.StdId != SCCM_RECEIVE_ID)
	{
		return;
	}
	
	memcpy(SupCap.SCCM_ReceiveData.data, RxMessage.CAN_RxMessage, 8);
	
		//帧率
	SupCap.infoUpdateFrame++;
}

/**
	* @brief  超级电容发送函数
  * @param	充电功率 充电使能 是否电容供电
  * @retval None
  */
int errSend = 0;
void SCCM_SendMsg(void)
{
	uint32_t send_SCCM_mail_box = 10;
	
  SCCMCanParam.DLC = 0x08;
	SCCMCanParam.IDE = CAN_ID_STD;
	SCCMCanParam.RTR = CAN_RTR_DATA;
	SCCMCanParam.StdId = SCCM_SEND_ID;
	
	
	memcpy(SCCM_Send_Data, SupCap.SCCM_SendData.data, 8);
	
	HAL_CAN_AddTxMessage(&hcan1, &SCCMCanParam, SCCM_Send_Data, &send_SCCM_mail_box);
	
}

/**
	* @brief  超级电容控制函数
  * @param	
  * @retval None
  */
uint16_t ChassisPower_Limit = 0;
float IN226_Val = 0;
float Sup_c = 0;
float Supcap_Rec_ChassisPower_Limit = 0;
void SuperCapation_Funtion(void)
{
	
	  if (ext_game_robot_state.data.chassis_power_limit < 45)
    {
        Supcap_Rec_ChassisPower_Limit = 45;
    }
    else if (ext_game_robot_state.data.chassis_power_limit > 120)
    {
        Supcap_Rec_ChassisPower_Limit = 120;
    }
    else
    {
        Supcap_Rec_ChassisPower_Limit = ext_game_robot_state.data.chassis_power_limit;
    }

    //---充能功率——剩余功率
    if (SupCap.SCCM_SendData.Is_Cap_Output == 1 && SupCap.SCCM_ReceiveData.Cap_Cell >40)
    {
        //--- 边放边充，放电的时候满功率充电
        Sup_c = Supcap_Rec_ChassisPower_Limit;
    }
    else
    {
            //--- 功率计掉线则使用裁判系统的数据
        Sup_c = Supcap_Rec_ChassisPower_Limit - (ext_game_robot_state.data.chassis_power_limit - ext_power_heat_data.data.chassis_power);
    }

    //充能功率限幅
    if (SupCap.SCCM_SendData.Charge_Power > Supcap_Rec_ChassisPower_Limit + 5) // 底盘输出功率
    {
        Sup_c = Supcap_Rec_ChassisPower_Limit;
    }
    else if (SupCap.SCCM_SendData.Charge_Power < 0)
    {  
        Sup_c = 0.0f;
    }

    //缓存功率限制充能功率
    if (ext_power_heat_data.data.chassis_power_buffer < 60 && ext_power_heat_data.data.chassis_power_buffer > 55)
    {
        Sup_c -= 5;
    }
    else if (ext_power_heat_data.data.chassis_power_buffer <= 55)
    {
        Sup_c = 0.1f;
    }
		
		SupCap.SCCM_SendData.Charge_Power = Sup_c;
		
	//失能 遥控 超电离线时不充电不输出 
	if(DR16_Export_Data.ChassisWorkMode == 0 || DR16_Export_Data.ChassisWorkMode == WorkMode_Disable || SupCap.OffLineFlag == 1)
	{
		SupCap.FUN.MainSwitch(SupCap_OFF, Charging_OFF, Power_NotSupply);
		SupCap.Last_Cap = false;
		return;
	}
	else
	{
		SupCap.EnableCap = SupCap_ON;
		SupCap.SCCM_SendData.Charge_Enable = Charging_ON;
	}
		
////	/**********************	放电 ************************/
	switch(SupCap.SCCM_SendData.Is_Cap_Output)
	{
	case Power_Supply://开启超电
		
		if(SupCap.SCCM_ReceiveData.Cap_Cell >= 25.0f)
		{
			SupCap.FUN.MainSwitch(SupCap_ON, Charging_ON, Power_Supply);//电容电量超过50开启
		}
		else if(SupCap.SCCM_ReceiveData.Cap_Cell < 25.0f)
		{
			SetSuperCap_Mode(Cap_Close);
			SupCap.FUN.MainSwitch(SupCap_ON, Charging_ON, Power_NotSupply);//充电但不放电
		}
		
		SupCap.Charge_BuffTime = 0;
		
		SupCap.Last_Cap = true;
		
		break;

	case Power_NotSupply:
		
		SetSuperCap_Mode(Cap_Close);
	
		if(SupCap.Last_Cap == true)
		{
			SupCap.Charge_BuffTime++;
			if(SupCap.Charge_BuffTime > 5000)
			{
				SupCap.FUN.MainSwitch(SupCap_ON, Charging_ON, Power_NotSupply);
				SupCap.Last_Cap = false;
				
			}
			else
			{
				SupCap.FUN.MainSwitch(SupCap_ON, Charging_OFF, Power_NotSupply);
				SupCap.Charging_Power = 0;
				SupCap.SCCM_SendData.Charge_Power = 0;
			}
		}
		else
		{
			SupCap.FUN.MainSwitch(SupCap_ON, Charging_ON, Power_NotSupply);
		}	
		break;

	default:
		SetSuperCap_Mode(Cap_Close);
		SupCap.FUN.MainSwitch(SupCap_OFF, Charging_OFF, Power_NotSupply); 
		break;
	}
}
	


/**
  * @brief  超级电容充电控制
  * @param	Charging_Power
  * @retval None
  */
void SupCap_ChargeControl(float Charging_power)
{
	SupCap.SCCM_SendData.Charge_Power = Charging_power;
}

/**
  * @brief  超级电容放电开关
  * @param	Charging_Power
  * @retval None
  */
void SupCap_SupplySwitch(bool Switch)
{
	SupCap.SCCM_SendData.Is_Cap_Output = Switch;
}

/**
  * @brief  超级电容总开关
  * @param	void
  * @retval void
  */
void SupCap_MainSwitch(bool Cotrol_Switch, bool Charge_Switch, bool Supply_Switch)
{
	SupCap.EnableCap = Cotrol_Switch;
	SupCap.SCCM_SendData.Charge_Enable = Charge_Switch;
	SupCap.SCCM_SendData.Is_Cap_Output = Supply_Switch;
}

