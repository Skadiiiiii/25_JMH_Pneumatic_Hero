#ifndef POWER_METER_H
#define POWER_METER_H

#include "main.h"
#include "can.h"
#include "bsp_can.h"

//#pragma anon_unions


typedef struct 
{
    float Voltage; //--- 电压
    float Current; //--- 电流
    float Power;   //--- 功率
}PowerMeter_t;

//SEASKY-INA226——https://github.com/SEASKY-Master/Seasky_INA22
#define INA226_USART_SIZE 24
#define INA226_CAN_SIZE 8
#define INA226_Self 1
#define INA226_CANID 0x301
typedef union 
{
	struct
    {
        int16_t Power_Val;//功率mW
        int16_t voltageVal;//mV
        int16_t Shunt_Current;//mA
        int16_t Shunt_voltage;//uV
    }Pack;
	uint8_t data[INA226_CAN_SIZE]; 
}ina226RecvMsg_u;

typedef union 
{
	struct
    {
        float voltageVal;//mV
        float Shunt_Current;//mA
    }Pack;
		uint8_t data[INA226_CAN_SIZE]; 
}ina226RecvMsg_Self_u;

typedef struct{

	ina226RecvMsg_Self_u Self_RecvData;
	float Chassis_RealPower;
	uint16_t infoUpdateFrame; //帧率
  uint8_t OffLineFlag;      //设备离线标志
}PowerMeter_Meg_t;


extern PowerMeter_Meg_t Power_Meter;

void Check_Power_Meter(void);
void Power_meter_getInfo(Can_Export_Data_t RxMessage);

extern float Get_Power_Val(void);
extern float Get_voltageVal(void);
extern float Get_Shunt_Current(void);
extern float Get_Shunt_voltage(void);
#endif
