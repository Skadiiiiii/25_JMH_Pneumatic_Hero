#include "DM4310_motor.h"

DM_J4310_t DM4310s_yaw;
uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};		//电机使能命令
uint8_t Data_Failure[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};		//电机失能命令
uint8_t Data_Erase_error[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

void motor_enable(void)
{
	CANx_Send_Data(&hcan1,DM_Send_ID,Data_Enable);
}
void motor_disable(void)
{
	CANx_Send_Data(&hcan1,DM_Send_ID,Data_Failure);
}
void motor_Erase_error(void)
{
	CANx_Send_Data(&hcan1,DM_Send_ID,Data_Erase_error);
}
/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief  MIT模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 */
void MIT_CtrlMotor(CAN_HandleTypeDef *CANx,uint16_t id, float _pos, float _vel,float _KP, float _KD, float _torq)
{ 
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
	uint8_t data[8];

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	CANx_Send_Data(CANx,id,data);
 }

/**
 * @brief  位置速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 */
void PosSpeed_CtrlMotor(CAN_HandleTypeDef *CANx, uint16_t id, float _pos, float _vel)
{
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;

		uint8_t data[8];
		data[0] = *pbuf;
		data[1] = *(pbuf + 1);
		data[2] = *(pbuf + 2);
		data[3] = *(pbuf + 3);
	
		data[4] = *vbuf;
		data[5] = *(vbuf + 1);
		data[6] = *(vbuf + 2);
		data[7] = *(vbuf + 3);
		CANx_Send_Data(CANx,id + 0x100,data);

}

/**
  * @brief  获取4310数据
  */
void DM_4310_getInfo(Can_Export_Data_t RxMessage)
{
	DM4310s_yaw.id         = (RxMessage.CAN_RxMessage[0])&0x0F;
	DM4310s_yaw.state      = (RxMessage.CAN_RxMessage[0])>>4;
	DM4310s_yaw.p_int      = (RxMessage.CAN_RxMessage[1]<<8)|RxMessage.CAN_RxMessage[2];
	DM4310s_yaw.v_int      = (RxMessage.CAN_RxMessage[3]<<4)|(RxMessage.CAN_RxMessage[4]>>4);
	DM4310s_yaw.t_int      = ((RxMessage.CAN_RxMessage[4]&0xF)<<8)|RxMessage.CAN_RxMessage[5];
	DM4310s_yaw.position   = uint_to_float(DM4310s_yaw.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	DM4310s_yaw.speed      = uint_to_float(DM4310s_yaw.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
	DM4310s_yaw.torque     = uint_to_float(DM4310s_yaw.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
	DM4310s_yaw.Temp_mos   = (float)(RxMessage.CAN_RxMessage[6]);
	DM4310s_yaw.Temp_rotor = (float)(RxMessage.CAN_RxMessage[7]);
	
}
