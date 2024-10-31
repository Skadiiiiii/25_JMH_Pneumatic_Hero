#include "chassis_control.h"
#include "dr16.h"
#include "bsp_can.h"
#include "M3508_motor.h"
#include "M6020_motor.h"
#include "M2006_motor.h"
#include "pid.h"
#include <math.h>
#include "cloud_control.h"
#include "arm_math.h"

int16_t speed_buff[4];
RUD_Param_t RUD_Param[4]; /*<! ת������ز��� */

/**
  * @brief  ȡ�����ľ���ֵ
  */
float abs(float num)
{
		int temp;
		if(num<0) temp=-num;
		else temp=num;
		return temp;
}

/**
  * @brief  ���õ���ת����6020��ʼ�Ƕȣ�45����У�
  */
void Chassis_Init(void)
{
	M6020s_chassis[LF_206_6020].Init_angle = LF_206_6020_Init_Angle + 45;
	M6020s_chassis[RF_205_6020].Init_angle = RF_205_6020_Init_Angle - 45;
	M6020s_chassis[RB_207_6020].Init_angle = RB_207_6020_Init_Angle + 45;
	M6020s_chassis[LB_208_6020].Init_angle = LB_208_6020_Init_Angle - 45;
}

/**
  * @brief  ����ת����6020�Ƕ�����
  */
static void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes < 100)
	{
		if(*angle > 180.0f)
		{
			*angle -= 360.0f;
			AngleLimit(angle);
		}
		else if(*angle < -180.0f)
		{
			*angle += 360.0f;
			AngleLimit(angle);
		}
	}
	recursiveTimes--;
}

/**
  * @brief  ����ת����6020��ǰ�Ƕ�
  */
static void RUDTotalAngle_Calc(int8_t motor_num , int8_t reset , uint8_t opposite)
{   
    float Cur_encoder = M6020s_chassis[motor_num].rotor_angle/8192*360;
    static float Pre_encoder[4] = {0};

    if(Cur_encoder - Pre_encoder[motor_num] > 180)
    {
        RUD_Param[motor_num].Turns_cnt--;
    }
    else if(Cur_encoder - Pre_encoder[motor_num] < -180)
    {
        RUD_Param[motor_num].Turns_cnt++;
    }
    Pre_encoder[motor_num] = Cur_encoder;

    if(reset == true)  //---Ȧ������
    {
        RUD_Param[motor_num].Turns_cnt = 0;
    }

    if(opposite == false)   //---�Ƿ���
    {
        RUD_Param[motor_num].Total_angle = Cur_encoder + RUD_Param[motor_num].Turns_cnt*360;
    }
    else if(opposite == true)
    {
        RUD_Param[motor_num].Total_angle = -Cur_encoder - RUD_Param[motor_num].Turns_cnt*360;
    }
}

/**
  * @brief  ����ת����6020Ŀ��Ƕ�
  */
float error;
uint8_t Move_flag;
uint16_t Brake_cnt;//--- ɲ��������
uint8_t No_move_flag = false;//--- ��ֹ��־λ
static void RUDTargetAngle_Calc(int8_t motor_num , int8_t reset , uint8_t opposite)
{
    float Cur_Target = RUD_Param[motor_num].Target_angle;

    static float temp_pre = 0;  //--- �����ӻ���ת����һ��δ�����㴦���Ŀ��Ƕ�

    //--- ��Ŀ��Ƕ��뵱ǰ�ǶȻ��㵽ͬһ��������
    RUD_Param[motor_num].TarTurns_cnt = (int32_t)(RUD_Param[motor_num].Total_angle/180.0f) - (int32_t)(RUD_Param[motor_num].Total_angle/360.0f);

    RUD_Param[motor_num].Target_angle = RUD_Param[motor_num].TarTurns_cnt*360.0f + Cur_Target + RUD_Param[motor_num].Init_angle;

    temp_pre = Cur_Target;

    //--- ��ǰĿ��ǶȺ͵�ǰ�Ƕȵ����
    error = RUD_Param[motor_num].Target_angle - RUD_Param[motor_num].Total_angle;

    AngleLimit(&error);

    //--- ����Ƕ�������90�����ٶȷ��򲢽�Ŀ��Ƕȵ���180��
    if(fabs(error) > 90.0f && (Move_flag == true || No_move_flag == false || Brake_cnt < 500))
    {
        //--- Ŀ��ֵ���Ӱ������
        RUD_Param[motor_num].Target_angle += 180.0f;

        temp_pre += 180.0f;

        //--- �����ַ�ת
				speed_buff[motor_num] = -speed_buff[motor_num];

        //--- ȷ��Ŀ��ǶȺ͵�ǰ�Ƕȴ���ͬһ������
        if(RUD_Param[motor_num].Target_angle > RUD_Param[motor_num].TarTurns_cnt*360.0f + 180.0f)
        {
            RUD_Param[motor_num].Target_angle -= 360.0f;
        }
        else if(RUD_Param[motor_num].Target_angle < RUD_Param[motor_num].TarTurns_cnt*360.0f - 180.0f)
        {
            RUD_Param[motor_num].Target_angle += 360.0f;
        }
    }
}

/**
 * @brief  �����˶�����(ת����)
 */
uint8_t spin_flag;
uint8_t stop_pid_flag;
float Radius = 1.0f;  // Բ�ľ�
static void RudAngle_Calc(int16_t Vx, int16_t Vy, int16_t Vw)
{
    float const theta = atan(1.0/1.0);
    static uint16_t No_move_cnt = 0;

    static float last_vx = 0, last_vy = 0, last_vw = 0;

    if(Vx == 0 && Vy == 0)
    {
        No_move_flag = true;
        if(abs(Vw) < 70) //---IMU��Ư�����������ٶ�
        {
            if(No_move_cnt < 500) //--- ��ֹ��1000ms�ڼ䲻������̸���
            {
                No_move_cnt++;
                Vw = 0;
            }
            else
            {}
        }
    }
    else
    {
        spin_flag = false;
        No_move_flag = false;
        No_move_cnt = 0;
    }

    if(Vx == 0 && Vy == 0 && Vw == 0)
    {
        Move_flag = false;

        if(Brake_cnt < 500)
        {
            Brake_cnt++;

            //--- ����һ֡���Ǿ�ֹ��ʱ�����һ��Vw���ٶȣ�Ϊ�˲�����Ŀ��Ƕ���һ������Ķ���
            last_vw = (spin_flag == true ? 50 : 0);

            //--- ʹ����һ�ε�Ŀ���ٶ���������һ�ε�Ŀ��Ƕ�
            RUD_Param[LF_206_6020].Target_angle = atan2(last_vx - last_vw*(Radius*arm_sin_f32(theta)),last_vy - last_vw*Radius*arm_cos_f32(theta))*(180/PI);
						RUD_Param[RF_205_6020].Target_angle = atan2(last_vx - last_vw*(Radius*arm_sin_f32(theta)),last_vy + last_vw*Radius*arm_cos_f32(theta))*(180/PI);
            RUD_Param[RB_207_6020].Target_angle = atan2(last_vx + last_vw*(Radius*arm_sin_f32(theta)),last_vy + last_vw*Radius*arm_cos_f32(theta))*(180/PI);
						RUD_Param[LB_208_6020].Target_angle = atan2(last_vx + last_vw*(Radius*arm_sin_f32(theta)),last_vy - last_vw*Radius*arm_cos_f32(theta))*(180/PI);
        }
        else
        {
            spin_flag = false;
            //--- 45�ȹ���
            RUD_Param[LF_206_6020].Init_angle = LF_206_6020_Init_Angle + 45;
						RUD_Param[RF_205_6020].Init_angle = RF_205_6020_Init_Angle - 45;
            RUD_Param[RB_207_6020].Init_angle = RB_207_6020_Init_Angle + 45;
						RUD_Param[LB_208_6020].Init_angle = LB_208_6020_Init_Angle - 45;

						stop_pid_flag = 1;
					
            //--- Ŀ��Ƕȹ���
            for(uint8_t i = 0 ; i < 4 ; i++)
            {
                RUD_Param[i].Target_angle = 0;
            }
        }
    }
    else
    {
				stop_pid_flag = 0;
			
        if(No_move_flag != true)
        {
            Brake_cnt = 0;
        }
        Move_flag = true;

        //--- ���45�ȹ���
				RUD_Param[LF_206_6020].Init_angle = LF_206_6020_Init_Angle;
        RUD_Param[RF_205_6020].Init_angle = RF_205_6020_Init_Angle;
        RUD_Param[RB_207_6020].Init_angle = RB_207_6020_Init_Angle;
        RUD_Param[LB_208_6020].Init_angle = LB_208_6020_Init_Angle;    

        //--- ��Ŀ���ٶȵ�ʱ��Ž��ж��ֽ���ļ���
        RUD_Param[LF_206_6020].Target_angle = atan2(Vx - Vw*(Radius*arm_sin_f32(theta)),Vy - Vw*Radius*arm_cos_f32(theta))*(180/PI);
				RUD_Param[RF_205_6020].Target_angle = atan2(Vx - Vw*(Radius*arm_sin_f32(theta)),Vy + Vw*Radius*arm_cos_f32(theta))*(180/PI);
        RUD_Param[RB_207_6020].Target_angle = atan2(Vx + Vw*(Radius*arm_sin_f32(theta)),Vy + Vw*Radius*arm_cos_f32(theta))*(180/PI);
				RUD_Param[LB_208_6020].Target_angle = atan2(Vx + Vw*(Radius*arm_sin_f32(theta)),Vy - Vw*Radius*arm_cos_f32(theta))*(180/PI);

        if(abs(Vw)>100)
        {
            spin_flag = true;
        }

        //--- ��Ŀ���ٶȵ�ʱ��ʹ����һ�νǶ�����������Ϊ����ģʽ��IMU��ֹ��˲��������΢��Vw�ٶ�
        last_vx = Vx;
        last_vy = 0;
        last_vw = 0;
    }
}

/**
 * @brief  �����˶�����(������)
 */
static void Wheel_calc(int16_t Vx, int16_t Vy, int16_t Vw, int16_t *cal_speed)
{
    float Param = 1.0f;

		float const theta = atan(1.0/1.0);//����45��

    static uint16_t No_move_cnt = 0;
    
    /* �����ٶȽ���ת���ֽǶ� ---------------------------------------------------------------------------*/
    RudAngle_Calc(Vx, Vy, Vw);

    if(Vx == 0 && Vy == 0)
    {
        if(abs(Vw) < 70) //---IMU��Ư�����������ٶ�
        {
            if(No_move_cnt < 500/* 500 */) //--- ��ֹ��1000ms�ڼ䲻������̸���
            {
                No_move_cnt++;
                Vw = 0;
            }
            else
            {}
        }
        
    }
    else
    {
        No_move_cnt = 0;
    }

    /* ������ �ٶȽ��� ---------------------------------------------------------------------------------*/
    cal_speed[RF_201_3508] = -sqrt(pow(Vx - Vw*arm_sin_f32(theta),2) + pow(Vy + Vw*arm_cos_f32(theta),2));
    cal_speed[LF_202_3508] =  sqrt(pow(Vx - Vw*arm_sin_f32(theta),2) + pow(Vy - Vw*arm_cos_f32(theta),2));
    cal_speed[RB_203_3508] = -sqrt(pow(Vx + Vw*arm_sin_f32(theta),2) + pow(Vy - Vw*arm_cos_f32(theta),2));
		cal_speed[LB_204_3508] =  sqrt(pow(Vx + Vw*arm_sin_f32(theta),2) + pow(Vy + Vw*arm_cos_f32(theta),2));

		cal_speed[0] *= Param;
		cal_speed[1] *= Param;
		cal_speed[2] *= Param;
		cal_speed[3] *= Param;


    /* ����ת���ֽǶȵ�ǰֵ ----------------------------------------------------------------------------*/
    RUDTotalAngle_Calc(0, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(1, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(2, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(3, RUD_NOT_RESET, RUD_NOT_OPSI);

    /* ����ת���ֽǶ�Ŀ��ֵ ----------------------------------------------------------------------------*/
    RUDTargetAngle_Calc(0, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(1, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(2, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(3, RUD_NOT_RESET, RUD_NOT_OPSI);

    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        //--- ת���� �ӻ���ת
        RUD_Param[i].Target_angle = Turn_InferiorArc(RUD_Param[i].Target_angle, RUD_Param[i].Total_angle);
    }
}

/**
 * @brief ��ȡ�������6020���ĽǶȣ�����Ϊ��׼
 */
float M6020s_Yaw_Angle_Centre;
static void read_start_yaw(void)
{
		M6020s_Yaw_Angle_Centre = M6020s_Yaw.real_rotor_angle;
}

/**
 * @brief ȫ��ʽ
 */
static float* Speed_Decompose(float Vx, float Vy)
{
	float RadRaw = 0.0f;
	static float Chassis[2];
	
	float angle = (M6020s_Yaw.real_rotor_angle - M6020s_Yaw_Angle_Centre) / M6020_mAngleRatio; //��е�Ƕ�ƫ��
	RadRaw = angle * DEG_TO_RAD;   
	
  Chassis[0] = Vx * arm_cos_f32(RadRaw) - Vy * arm_sin_f32(RadRaw);
	Chassis[1] = Vy * arm_cos_f32(RadRaw) + Vx * arm_sin_f32(RadRaw);
	
	return Chassis;
}

/**
  * @brief  ����ʹ��
  */
static void Ship_ChassisWorkMode(float Vx, float Vy,float VOmega)
{
	Wheel_calc(Vx,Vy,VOmega,speed_buff);
	
	for (uint8_t i = 0; i < 4; i++)
	{
			M3508s_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], speed_buff[i], M3508s_chassis[i].rotor_speed);
		
			if(stop_pid_flag == 1)
			{
				M6020s_chassis[i].set_current = pid_CascadeCalc(&motor_pid_chassis_6020_stop[i], RUD_Param[i].Target_angle, RUD_Param[i].Total_angle,M6020s_chassis[i].rotor_speed);
			}
			else
			{
				M6020s_chassis[i].set_current = pid_CascadeCalc(&motor_pid_chassis_6020[i], RUD_Param[i].Target_angle, RUD_Param[i].Total_angle,M6020s_chassis[i].rotor_speed);
			}
	}

	set_M3508_200_voltage(&hcan2,
							M3508s_chassis[0].set_current, 
							M3508s_chassis[1].set_current, 
							M3508s_chassis[2].set_current,
							M3508s_chassis[3].set_current);	
	
	set_M6020_1ff_voltage(&hcan2,
							M6020s_chassis[0].set_current, 
							M6020s_chassis[1].set_current, 
							M6020s_chassis[2].set_current, 
							M6020s_chassis[3].set_current);	

}

/**
  * @brief  ���̸���ģʽ
  */
static void Ship_ChassisWorkMode_follow(float Vx, float Vy)
{
	int16_t Chassis_target_rotor_speed;
	
//	if(M6020s_Yaw_Angle_Centre - M6020s_Yaw.real_rotor_angle > 1024)
//	if(rc.roll > 0)
//	{
//		M6020s_Yaw_Angle_Centre -= 2048;
//	}
//	else if(rc.roll < 0)
//	{
//		M6020s_Yaw_Angle_Centre += 2048;
//	}
////	else if(M6020s_Yaw.real_rotor_angle - M6020s_Yaw_Angle_Centre > 1024)
////	{
////		M6020s_Yaw_Angle_Centre += 2048;
////	}
//	if(M6020s_Yaw_Angle_Centre > 8191)
//	{
//		M6020s_Yaw_Angle_Centre -= 8191;
//	}
//	else if(M6020s_Yaw_Angle_Centre < 0)
//	{
//		M6020s_Yaw_Angle_Centre += 8191;
//	}

	Chassis_target_rotor_speed = pid_calc_cloud(&motor_pid_chassis_pos,M6020s_Yaw_Angle_Centre,M6020s_Yaw.real_rotor_angle);
	
	Wheel_calc(Vx,Vy,-Chassis_target_rotor_speed,speed_buff);
								
	for (uint8_t i = 0; i < 4; i++)
	{
			M3508s_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], speed_buff[i], M3508s_chassis[i].rotor_speed);
		
			if(stop_pid_flag == 1)
			{
				M6020s_chassis[i].set_current = pid_CascadeCalc(&motor_pid_chassis_6020_stop[i], RUD_Param[i].Target_angle, RUD_Param[i].Total_angle,M6020s_chassis[i].rotor_speed);
			}
			else
			{
				M6020s_chassis[i].set_current = pid_CascadeCalc(&motor_pid_chassis_6020[i], RUD_Param[i].Target_angle, RUD_Param[i].Total_angle,M6020s_chassis[i].rotor_speed);
			}
	}

	set_M3508_200_voltage(&hcan2,
							M3508s_chassis[0].set_current, 
							M3508s_chassis[1].set_current, 
							M3508s_chassis[2].set_current,
							M3508s_chassis[3].set_current);	
	
	set_M6020_1ff_voltage(&hcan2,
							M6020s_chassis[0].set_current, 
							M6020s_chassis[1].set_current, 
							M6020s_chassis[2].set_current, 
							M6020s_chassis[3].set_current);		
}

/**
  * @brief  ����С����ģʽ
  */
static void Ship_ChassisWorkMode_Tuoluo(float Vx, float Vy)
{
	float* Chassis = Speed_Decompose(Vx,Vy);
	float target_Vomega = 1500;
	
	Wheel_calc(Chassis[0],Chassis[1],-target_Vomega,speed_buff);
								
	for (uint8_t i = 0; i < 4; i++)
	{
			M3508s_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], speed_buff[i], M3508s_chassis[i].rotor_speed);
		
			M6020s_chassis[i].set_current = pid_CascadeCalc(&motor_pid_chassis_6020[i], RUD_Param[i].Target_angle, RUD_Param[i].Total_angle,M6020s_chassis[i].rotor_speed);
	}

	set_M3508_200_voltage(&hcan2,
							M3508s_chassis[0].set_current, 
							M3508s_chassis[1].set_current, 
							M3508s_chassis[2].set_current,
							M3508s_chassis[3].set_current);	
	
	set_M6020_1ff_voltage(&hcan2,
							M6020s_chassis[0].set_current, 
							M6020s_chassis[1].set_current, 
							M6020s_chassis[2].set_current, 
							M6020s_chassis[3].set_current);	
}

/**
  * @brief  ����ʹ��
  */
static void Ship_ChassisWorkMode_shoot(float speed)
{
	M2006s.set_current = pid_calc(&motor_pid_shoot,speed,M2006s.rotor_speed);
	
	set_M2006_200_voltage(&hcan1,M2006s.set_current,0,0,0);
}

/**
  * @brief  ����ʧ��
  */
static void Robot_control_chassis_disable()
{
	for(uint8_t i = 0;i < 4;i++)
	{
		M3508s_chassis[i].set_current = 0;
		M6020s_chassis[i].set_current = 0;
	}
	set_M3508_200_voltage(&hcan2,0,0,0,0);
	set_M6020_1ff_voltage(&hcan2,0,0,0,0);
	set_M2006_200_voltage(&hcan1,0,0,0,0);
}

/**
  * @brief  ������������
  */
static void Robot_control ()        
{
	if(DR16_Export_Data.ControlSwitch->Left == 3  && DR16_Export_Data.ControlSwitch->Right == 2)//�������£����̣�
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Chassis;
		Ship_ChassisWorkMode(10.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,
												 10.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value,
												-7.0f*DR16_Export_Data.Robot_TargetValue.Omega_Value);
	}
	else if(DR16_Export_Data.ControlSwitch->Left == 2  && DR16_Export_Data.ControlSwitch->Right == 3)//�������У���̨��
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Cloud;
		Robot_control_chassis_disable(); 
	}
	else if(DR16_Export_Data.ControlSwitch->Left == 3  && DR16_Export_Data.ControlSwitch->Right == 3)//˫�У����棩
	{
			DR16_Export_Data.ChassisWorkMode = WorkMode_Follow;
			
			Ship_ChassisWorkMode_follow(10.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,
																	10.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value);
	}
	else if(DR16_Export_Data.ControlSwitch->Left == 1 && DR16_Export_Data.ControlSwitch->Right == 3)//�������У����䣩
	{
			DR16_Export_Data.ChassisWorkMode = WorkMode_Shoot;
		
			Ship_ChassisWorkMode_shoot(15.0f * DR16.rc.roll);
			Ship_ChassisWorkMode_follow(10.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,
																10.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value);
	}
	else if(DR16_Export_Data.ControlSwitch->Left == 3 && DR16_Export_Data.ControlSwitch->Right == 1)//�������ϣ�С���ݣ�
	{
			DR16_Export_Data.ChassisWorkMode = WorkMode_Tuoluo;
		
			Ship_ChassisWorkMode_Tuoluo(10.0f*DR16_Export_Data.Robot_TargetValue.Left_Right_Value,
																10.0f*DR16_Export_Data.Robot_TargetValue.Forward_Back_Value);
	}
	else if(DR16_Export_Data.ControlSwitch->Left == 2 && DR16_Export_Data.ControlSwitch->Right == 2)//˫�£�ʧ�ܣ�
	{
		DR16_Export_Data.ChassisWorkMode = WorkMode_Disable;
		if(DR16_Export_Data.Robot_TargetValue.Omega_Value != 0)
		{
			read_start_yaw();
		}
		Robot_control_chassis_disable();	
	}
	else
	{
		Robot_control_chassis_disable();	
		DR16_Export_Data.ChassisWorkMode = 0;
	}
}

/**
  * @brief    ң�������ƻ�����
**/
void Robot_Control_Fun()
{
	RemoteControl_Output();
	Robot_control();
}

/**
  * @brief    ң�������ƻ�����
**/
void Robot_Control_Disable()
{
	Robot_control_chassis_disable();	
	DR16_Export_Data.ChassisWorkMode = 0;
}

