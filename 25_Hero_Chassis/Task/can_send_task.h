#ifndef CAN_SEND_TASK_H
#define CAN_SEND_TASK_H
#include "main.h"

typedef struct
{
	uint64_t LF_6020:1;//∑∂Œß0-1
	uint64_t RF_6020:1;
	uint64_t LB_6020:1;
	uint64_t RB_6020:1;
	
	uint64_t LF_3508:1;
	uint64_t RF_3508:1;
	uint64_t LB_3508:1;
	uint64_t RB_3508:1;
	
	uint64_t WorkMode:8;//∑∂Œß0-64
	
	uint64_t :12;//∑∂Œß0-1320
	uint64_t :12;
	int64_t  Omega_Value:12;
	int64_t  Pitch_Value:12;
}DR_data_t;


typedef union//π≤”√ÃÂ
{
	uint8_t data[8];
	DR_data_t pack;
}DR16_data_t;

#endif

