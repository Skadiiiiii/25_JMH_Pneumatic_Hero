#include "power_limit_control.h"
#include "RM_JudgeSystem.h"
#include "AddMath.h"
#include "Power_Meter.h"

int16_t powerBuffErr;  // 用掉的缓冲能量
int16_t Power_Buffer;   /*<! 缓冲功率 */
int32_t SumCurrent_In;  /*<! 电流输入总和 */
int32_t SumCurrent_Out; /*<! 计算后的电流输出总和 */
float DRV_CalcRatio;       /*<! 用于计算限制功率的系数 */
float RUD_CalcRatio;       /*<! 用于计算限制功率的系数 */
float debug_powercoe = 80.0f;
uint8_t Go_Up = 0;

static void Limit_Calc (void)
{
    Power_Buffer = ext_power_heat_data.data.chassis_power_buffer;

    powerBuffErr = 60 - Power_Buffer;

    DRV_CalcRatio = 0;
    DRV_CalcRatio = (float)Power_Buffer / debug_powercoe;
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
			 drv_tempcurrent[i]   = M3508s_chassis[i].set_voltage;
			 drv_tempcurrent[i+4] = M6020s_chassis[i].set_voltage;
		}
		Limit(drv_tempcurrent,8);
		for(uint8_t i = 0 ; i < 4 ; i++)
		{
			 M3508s_chassis[i].set_voltage = drv_tempcurrent[i];
			 M6020s_chassis[i].set_voltage = drv_tempcurrent[i+4];
		} 
}


