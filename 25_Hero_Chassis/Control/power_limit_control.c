#include "power_limit_control.h"

int16_t powerBuffErr;  // 用掉的缓冲能量
int16_t Power_Buffer;   /*<! 缓冲功率 */
int32_t SumCurrent_In;  /*<! 电流输入总和 */
int32_t SumCurrent_Out; /*<! 计算后的电流输出总和 */
float DRV_CalcRatio;       /*<! 用于计算限制功率的系数 */
float RUD_CalcRatio;       /*<! 用于计算限制功率的系数 */
//float debug_powercoe = 80.0f;
uint8_t Go_Up = 0;

static void Limit_Calc (void)
{
    Power_Buffer = ext_power_heat_data.data.chassis_power_buffer;

    powerBuffErr = 60 - Power_Buffer;

    DRV_CalcRatio = 0;
//    DRV_CalcRatio = (float)Power_Buffer / debug_powercoe;
		DRV_CalcRatio = (float)Power_Buffer / ext_game_robot_state.data.chassis_power_limit;
    DRV_CalcRatio *= DRV_CalcRatio;  // 平方的关系

    if(powerBuffErr > 0 /* && Infantry.Write_Msg[Cap_Ctrl] != true */)  // 若用到缓冲功率则进行功率限制处理
    {
        SumCurrent_Out = SumCurrent_In * DRV_CalcRatio;
    }
}

static void Limit(int16_t *wheelCurrent, int8_t amount)
{
    float coe[8] = {0.0f};

    //--- 不限功率
    if(ext_game_robot_state.data.chassis_power_limit == 65535)
    {
        SumCurrent_Out = SumCurrent_In;  //--- 无处理时为原来的值
        return;
    }

    SumCurrent_In = SumCurrent_Out = 0;

    /*-----------------------------------------*/
    for(uint8_t i = 0 ; i < 8 ; i++)
    {
        SumCurrent_In += abs(wheelCurrent[i]);
    }
		
    SumCurrent_Out = SumCurrent_In;  // 无处理时为原来的值

    // 计算每个电机的电流占比
		if(Go_Up == 0)
		{
			for(uint8_t i = 0 ; i < 4 ; i++)
			{
				coe[i] = ((float)(wheelCurrent[i])) / ((float)(SumCurrent_In));
				coe[i+4] = ((float)(wheelCurrent[i+4])) / ((float)(SumCurrent_In));
			}
		}
		else
		{
			coe[0] = ((float)(wheelCurrent[0])) / ((float)(SumCurrent_In)) * 1.8f;
			coe[1] = ((float)(wheelCurrent[1])) / ((float)(SumCurrent_In)) * 1.8f;
			coe[2] = ((float)(wheelCurrent[2])) / ((float)(SumCurrent_In)) * 1.8f;
			coe[3] = ((float)(wheelCurrent[3])) / ((float)(SumCurrent_In)) * 1.8f;
			coe[4] = ((float)(wheelCurrent[0])) / ((float)(SumCurrent_In)) * 0.2f;
			coe[5] = ((float)(wheelCurrent[1])) / ((float)(SumCurrent_In)) * 0.2f;
			coe[6] = ((float)(wheelCurrent[2])) / ((float)(SumCurrent_In)) * 0.2f;
			coe[7] = ((float)(wheelCurrent[3])) / ((float)(SumCurrent_In)) * 0.2f;
		}
    Limit_Calc();

    for(uint8_t i = 0 ; i < amount ; i++)
    {
        wheelCurrent[i] = ((SumCurrent_Out) * coe[i]);
    }
}

Chassis_PowerLimit_t Chassis_PowerLimit;
int16_t drv_tempcurrent[8]; //老功率限制
void chassis_power_control(Chassis_PowerLimit_t *chassis_power_control)
{
		for(uint8_t i = 0 ; i < 4 ; i++)
		{
			 drv_tempcurrent[i]   = M3508s_chassis[i].set_current;
			 drv_tempcurrent[i+4] = M6020s_chassis[i].set_current;
		}
		Limit(drv_tempcurrent,8);
		for(uint8_t i = 0 ; i < 4 ; i++)
		{
			 M3508s_chassis[i].set_current = drv_tempcurrent[i];
			 M6020s_chassis[i].set_current = drv_tempcurrent[i+4];
		} 
		
//		updateShiplMaxPower(60);
////		updateWheelMaxPower(60);
////		rlsupdate();
////		getDecayCurrent(&Chassis_Power_M3508);
//		getDecayCurrent(&Chassis_Power_M6020);
}


static float rpm2rad(float rpm) { return rpm * RPM_TO_RAD; }

static float current2Torque(float current,Chassis_Power_t *Chassis_Power) { return current * Chassis_Power->Kt; }//将电流转换为扭矩

static float floatEqual(float a, float b) { return fabs(a - b) < 1e-6f; }

Chassis_Power_t Chassis_Power_M3508 = Chassis_M3508_Power_Init;
Chassis_Power_t Chassis_Power_M6020 = Chassis_M6020_Power_Init;

void updateWheelMaxPower(float MaxPower)
{
	Chassis_Power_M3508.maxPower = MaxPower;
	
	float ePower = 0, cPower = 0;
	//ePower 用于累加估计功率
	//cPower 用于累加命令功率
	
	for (int i = 0; i < 4; ++i)
	{
			float w = rpm2rad(M3508s_chassis[i].rotor_speed);//获取电机转速
			float t = current2Torque(M3508s_chassis[i].torque_current / 1000.0f,&Chassis_Power_M3508);//获取电机扭矩

			// Current Power
			Chassis_Power_M3508.currentPower[i] = t * w + Chassis_Power_M3508.k1 * fabs(w) + Chassis_Power_M3508.k2 * t * t + Chassis_Power_M3508.k3 / 4.0f;//计算估计功率
			ePower += Chassis_Power_M3508.currentPower[i];

			// Command Power
			float ct = current2Torque(M3508s_chassis[i].set_current * 20.0f / 16384.0f,&Chassis_Power_M3508);//获取发送扭矩
			Chassis_Power_M3508.cmdPower[i] = ct * w + Chassis_Power_M3508.k1 * fabs(w) + ct * ct * Chassis_Power_M3508.k2 + Chassis_Power_M3508.k3 / 4.0f;//计算命令功率
			cPower += Chassis_Power_M3508.cmdPower[i];
	}

	Chassis_Power_M3508.estimatedPower = ePower;
	Chassis_Power_M3508.commandPower   = cPower;
}

void updateShiplMaxPower(float MaxPower)
{
	Chassis_Power_M6020.maxPower = MaxPower;
	
	float ePower = 0, cPower = 0;
	//ePower 用于累加估计功率
	//cPower 用于累加命令功率
	
	for (int i = 0; i < 4; ++i)
	{
			float w = rpm2rad(M6020s_chassis[i].rotor_speed);//获取电机转速
			float t = current2Torque(M6020s_chassis[i].torque_current / 1000.0f,&Chassis_Power_M6020);//获取电机扭矩

			// Current Power
			Chassis_Power_M6020.currentPower[i] = t * w + Chassis_Power_M6020.k1 * fabs(w) + Chassis_Power_M6020.k2 * t * t + Chassis_Power_M6020.k3 / 4.0f;//计算估计功率
			ePower += Chassis_Power_M6020.currentPower[i];

			// Command Power
			float ct = current2Torque(M6020s_chassis[i].set_current * 20.0f / 16384.0f,&Chassis_Power_M6020);//获取发送扭矩
			Chassis_Power_M6020.cmdPower[i] = ct * w + Chassis_Power_M6020.k1 * fabs(w) + ct * ct * Chassis_Power_M6020.k2 + Chassis_Power_M6020.k3 / 4.0f;//计算命令功率
			cPower += Chassis_Power_M6020.cmdPower[i];
	}

	Chassis_Power_M6020.estimatedPower = ePower;
	Chassis_Power_M6020.commandPower   = cPower;
}


void getDecayCurrent(Chassis_Power_t *Chassis_Power)
{
	float allocatablePower = Chassis_Power->maxPower;
	float powerSumRequired = 0;
	float Kt = Chassis_Power->Kt;

	for (int i = 0; i < 4; i++)
	{
			if (Chassis_Power->cmdPower[i] > 0.0f)
			{
					powerSumRequired += Chassis_Power->cmdPower[i];
			}
			else
			{
					allocatablePower -= Chassis_Power->cmdPower[i];
			}
	}

	// Start to calculate the decay current, if the power is over the limit
	//如果命令功率超过最大功率，则将功率控制状态设置为 RESTRICTED
	if (Chassis_Power->commandPower > Chassis_Power->maxPower)
	{
			Chassis_Power->powerControlStatus = RESTRICTED;
			for (int i = 0; i < 4; i++)
			{
					if (floatEqual(Chassis_Power->cmdPower[i],0.0f) || Chassis_Power->cmdPower[i] < 0.0f)
					{
							continue;//如果命令功率为零或负值，则跳过该电机的计算
					}
					float curAv       = rpm2rad(M3508s_chassis[i].rotor_speed * 1000.0f);
					float powerWeight = Chassis_Power->cmdPower[i] / powerSumRequired;
					float delta       = curAv * curAv - 4.0f * Chassis_Power->k2 * (Chassis_Power->k1 * fabs(curAv) + Chassis_Power->k3 / 4.0f - powerWeight * allocatablePower);//计算Δ
					if (floatEqual(delta,0.0f))  // repeat roots
					{
							Chassis_Power->decayCurrent[i] = -curAv / (2.0f * Chassis_Power->k2) / Kt;//存在重根，使用重根公式计算衰减电流
					}
					else if (delta > 0.0f)  // distinct roots
					{
							Chassis_Power->decayCurrent[i] = Chassis_Power->decayCurrent[i] > 0.0f ? (-curAv + sqrtf(delta)) / (2.0f * Chassis_Power->k2) / Kt : (-curAv - sqrtf(delta)) / (2.0f * Chassis_Power->k2) / Kt;//存在不同的根，根据当前的 decayCurrent[i] 值选择合适的根
					}
					else  // imaginary roots
					{
							Chassis_Power->decayCurrent[i] = -curAv / (2.0f * Chassis_Power->k2) / Kt;//根是虚数，使用重根公式计算
					}
			}
	}
	else
	{
			Chassis_Power->powerControlStatus = NOT_RESTRICTED;//如果命令功率未超过最大功率，则将功率控制状态设置为 NOT_RESTRICTED
	}
}

