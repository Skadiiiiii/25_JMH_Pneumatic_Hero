#include "string.h"
#include "stdlib.h"
#include "dr16.h"
#include "main.h"
#include "M6020_motor.h"
#include "M3508_motor.h"

uint8_t M6020_state[4];
uint8_t M3508_state[4];
DR16_Export_Data_t DR16_Export_Data;
DR16_data_t can1_send_data;


/**
  * @brief  获取A板发送的dr16数据
  */
void dr16_getInfo(uint8_t can_rx_data[])
{   
	memcpy(can1_send_data.data, can_rx_data, sizeof(can1_send_data.data));
	
	M6020_state[RF_205_6020] = can1_send_data.pack.RF_6020;
	M6020_state[LF_206_6020] = can1_send_data.pack.LF_6020;
	M6020_state[RB_207_6020] = can1_send_data.pack.RB_6020;
	M6020_state[LB_208_6020] = can1_send_data.pack.LB_6020;
	
	M3508_state[RF_201_3508] = can1_send_data.pack.RF_3508;
	M3508_state[LF_202_3508] = can1_send_data.pack.LF_3508;
	M3508_state[RB_203_3508] = can1_send_data.pack.RB_3508;
	M3508_state[LB_204_3508] = can1_send_data.pack.LB_3508;
	
	DR16_Export_Data.ChassisWorkMode = can1_send_data.pack.WorkMode;
	
	DR16_Export_Data.Robot_TargetValue.Omega_Value = can1_send_data.pack.Omega_Value;
  DR16_Export_Data.Robot_TargetValue.Pitch_Value = can1_send_data.pack.Pitch_Value;
}


