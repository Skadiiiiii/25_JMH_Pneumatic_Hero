#include "DJI_IMU.h"
//由于can的数据帧最大的字节数为8
//欧拉角的一个轴的数据就是一个float类型变量，是4个字节
//那也就最多只能发两个轴
Euler_Send_u Euler_Send;
Gyro_Send_u Gyro_Send;
IMU_Rec_Data_t DJI_C_IMU;
//接收变量
IMU_CAL_t IMU_CAL;


//void IMU_getInfo(Can_Export_Data_t IMU_Receive_Data)
//{
//	DJI_C_IMU.yaw   = (IMU_Receive_Data.CAN_RxMessage[0] <<24 | IMU_Receive_Data.CAN_RxMessage[1] <<16 | IMU_Receive_Data.CAN_RxMessage[2] <<8 | IMU_Receive_Data.CAN_RxMessage[3] <<24);
//	DJI_C_IMU.pitch = (IMU_Receive_Data.CAN_RxMessage[4] <<24 | IMU_Receive_Data.CAN_RxMessage[5] <<16 | IMU_Receive_Data.CAN_RxMessage[6] <<8 | IMU_Receive_Data.CAN_RxMessage[7] <<24);
//}
