 #ifndef __DR16_REMOTE__H__
#define __DR16_REMOTE__H__

#include "usart.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart1


typedef struct
{
    uint8_t DR16Buffer[DBUS_BUFLEN];
    struct
    {
        int16_t ch0; //yaw
        int16_t ch1; //pitch
        int16_t ch2; //left_right
        int16_t ch3; //forward_back
        uint8_t sw1;
        uint8_t sw2;
        int16_t roll; //����
    } rc;               //ң�������յ���ԭʼֵ��

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t keyLeft;
        uint8_t keyRight;

    } mouse;

    union
    { //union�������÷�
        uint16_t key_code;
        struct
        { //λ���ʹ��
            bool press_W : 1;
            bool press_S : 1;
            bool press_A : 1;
            bool press_D : 1;

            bool press_Shift : 1;
            bool press_Ctrl : 1;
            bool press_Q : 1;
            bool press_E : 1;

            bool press_R : 1;
            bool press_F : 1;
            bool press_G : 1;
            bool press_Z : 1;

            bool press_X : 1;
            bool press_C : 1;
            bool press_V : 1;
            bool press_B : 1;
        } Key_Code;
    } keyBoard;
    uint16_t infoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־
} DR16_t;

typedef enum
{
    RemotePole_UP = 1,  //��
    RemotePole_MID = 3, //��
    RemotePole_DOWM = 2 //��
} RemotePole_e;

typedef struct
{
    RemotePole_e Left;
    RemotePole_e Right;

} ControlSwitch_t; //ң������s1��s2����

typedef struct
{
    struct
    {
        float Forward_Back_Value; //Vx
        float Omega_Value;        //����ֵ��
        float Left_Right_Value;   //Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel; //����
    } Robot_TargetValue;  //ң�ؼ����������˶��ٶ�
    ControlSwitch_t *ControlSwitch;
    uint16_t infoUpdateFrame; //֡��
    uint8_t OffLineFlag;      //�豸���߱�־ 
		uint8_t Alldead;
		int16_t ChassisWorkMode;
} DR16_Export_Data_t;         //�������ļ�ʹ�õ�������ݡ�


#define DR16_GroundInit            \
    {                              \
        {0},                       \
            {0, 0, 0, 0, 0, 0, 0}, \
            {0, 0, 0, 0, 0},       \
            {0},                   \
    }

#define DR16_ExportDataGroundInit \
{                                 \
    {0, 0, 0, 0, 0, 0},           \
    &ControlSwitch,               \
    0,                            \
    0,                            \
}

extern uint8_t   dbus_buf[DBUS_BUFLEN];
extern DR16_Export_Data_t DR16_Export_Data;
//extern rc_info_t rc;
extern DR16_t DR16;
		
void RemoteControl_Output(void);
//void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
void Check_DR16(void);
void uart_receive_handler(UART_HandleTypeDef *huart);

#endif

