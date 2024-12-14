#ifndef _DRIVER_SUPERCAPATION_H
#define _DRIVER_SUPERCAPATION_H

#include "main.h"
#include "stdbool.h"
#include "bsp_can.h"
#define SCCM_RECEIVE_ID 0x600
#define SCCM_SEND_ID 0x601

//#pragma anon_unions

#define SupCap_ON 1
#define SupCap_OFF 0
#define Charging_ON 1
#define Charging_OFF 0
#define Power_Supply 1       //电源供应
#define Power_NotSupply 0
 
#define SupCap_Init          \
  {                          \
    .FUN =                   \
    { &Super_Init,					\
			&SuperCapation_Funtion,\
      &SupCap_MainSwitch,      \
      &SCCM_MsgProcess,         \
      &SCCM_SendMsg,    \
      &SupCap_ChargeControl, \
      &SupCap_SupplySwitch,  \
    }                        \
  }

//超级电容模式
typedef enum
{
	Cap_Enable,  //开启电容
	Cap_Close		 //关闭电容
	
}SuperCapMode_e;

typedef union{
  uint8_t data[8];
   struct{
    float Chassis_Power;    /* 底盘功率，单位：W */
    uint8_t Chassis_Buff;   /* 底盘功率缓冲 */
    uint8_t Cap_Usable;    /* 电容可以进行输出 */
		uint8_t Cap_Cell;				/* 电容剩余电量，会出现负数 */
  };
} SCCM_ReceiveData_t;



typedef union{
  uint8_t data[8];
  struct{
    float Charge_Power;    /* 充电功率，单位：W ,范围 0-80W */
    uint8_t Charge_Enable;  /* 充电使能 */
    uint8_t Is_Cap_Output;  /* 使用电容供电 */
  };
} SCCM_SendData_t;

typedef struct{
	SCCM_ReceiveData_t SCCM_ReceiveData;
	SCCM_SendData_t SCCM_SendData;
	
	uint16_t infoUpdateFrame; //帧率
  bool OffLineFlag;      //设备离线标志
	uint8_t EnableCap;	/* 超电使能开关*/
	float Charging_Power;      // 充电的功率 
	uint8_t Usable_Flag;       // 电容可用标志位
	bool Last_Cap;
	int Charge_BuffTime;
	
	struct{
		void (*Init)(void);
		void (*Ctrl)(void);
		void (*MainSwitch)(bool Cotrol_Switch, bool Charge_Switch, bool Supply_Switch);
		void (*MsgProcess)(Can_Export_Data_t RxMessage);
		void (*SendMsg)(void);
		void (*ChargeCtrl)(float Charging_power);
		void (*SupCap_SupplySwitch)(bool Switch);
	}FUN;
	
}SupCap_t;

void Check_SupCap(void);
void SetSuperCap_Mode(SuperCapMode_e status);

extern SupCap_t SupCap;
#endif
