#include "ANO.h"
#include "main.h"
#include "usart.h"

//小端模式（低字节在前，高字节在后）
#define BYTE0(dwTemp) (*(char *)(&dwTemp))    //取低八位
#define BYTE1(dwTemp) (*((char *)(&dwTemp)+1))//取高八位
#define BYTE2(dwTemp) (*((char *)(&dwTemp)+2))//取高十六位
#define BYTE3(dwTemp) (*((char *)(&dwTemp)+3))//取高二十四位

/**
  * @brief	匿名上位机协议
  */
void sent_data1(float A,float B,int16_t C)
{	
		uint8_t BUFF[100];
		uint8_t _cnt = 0;
		uint16_t flen;
	
		BUFF[_cnt++]=0xAB;//枕头
		BUFF[_cnt++]=0xFF;//源地址
		BUFF[_cnt++]=0xFF;//目标地址
		BUFF[_cnt++]=0xF1;//功能码(ID)
		BUFF[_cnt++]=10;//数据长度(2字节)
		BUFF[_cnt++]=0;//数据长度
		BUFF[_cnt++]=BYTE0(A);//数据内容A（2字节）
		BUFF[_cnt++]=BYTE1(A);//数据内容A
		BUFF[_cnt++]=BYTE2(A);//数据内容A（2字节）
		BUFF[_cnt++]=BYTE3(A);//数据内容A
	
		BUFF[_cnt++]=BYTE0(B);//数据内容B（2字节）
		BUFF[_cnt++]=BYTE1(B);//数据内容B
		BUFF[_cnt++]=BYTE2(B);//数据内容A（2字节）
		BUFF[_cnt++]=BYTE3(B);//数据内容A
	
		BUFF[_cnt++]=BYTE0(C);
		BUFF[_cnt++]=BYTE1(C);
	
		uint8_t sumcheck = 0;
		uint8_t addcheck = 0;
		flen = BUFF[4] + BUFF[5] * 255;
		for(uint16_t i=0; i < (flen+6); i++)//校验计算
		{
			sumcheck += BUFF[i]; 
			addcheck += sumcheck; 
		}
		
		BUFF[_cnt++] = sumcheck;//和校验
		BUFF[_cnt++] = addcheck;//附加校验
		HAL_UART_Transmit(&huart7,BUFF,_cnt,0xffff);//串口发送
}
