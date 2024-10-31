#include "SupCap.h"
#include "Power_Meter.h" 

/**
  ******************************************************************************
  * @file     Supercapacitor.c
  * @author   Shake
  * @version  V2.0
  * @brief    ³¬¼¶µçÈÝÄ£¿é
  ******************************************************************************
 **/

SupCap_t SupCap_Check;
SupCap_classdef Supcap_Rec;
void SupCap_Update(Can_Export_Data_t RxMessage)
{
	if(RxMessage.CAN_RxHeader.ExtId != SCCM_RECEIVE_ID)
	{
		return;
	}
  memcpy(Supcap_Rec.RecvData.data, RxMessage.CAN_RxMessage, 8);
	SupCap_Check.infoUpdateFrame ++;
}

