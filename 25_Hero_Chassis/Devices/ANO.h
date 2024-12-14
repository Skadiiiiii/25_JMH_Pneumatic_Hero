#ifndef NIMING_H_
#define NIMING_H_
#include "main.h"

//小端模式（低字节在前，高字节在后）
#define BYTE0(dwTemp) (*(char *)(&dwTemp))    //取低八位
#define BYTE1(dwTemp) (*((char *)(&dwTemp)+1))//取高八位
#define BYTE2(dwTemp) (*((char *)(&dwTemp)+2))//取高十六位
#define BYTE3(dwTemp) (*((char *)(&dwTemp)+3))//取高二十四位


void sent_data_ano(UART_HandleTypeDef* huart,float A,float B,int16_t C);

#endif

