#include "pid.h"
#include "M3508_motor.h"
#include "M6020_motor.h"
#include "cloud_control.h"

pid_struct_t motor_pid_chassis[4];
pid_Cascade_t motor_pid_chassis_6020[4];
pid_Cascade_t motor_pid_chassis_6020_stop[4];
pid_Cascade_t motor_pid_Cas_Chassis[4];
pid_struct_t motor_pid_chassis_pos;
pid_struct_t motor_pid_shoot;
pid_Cascade_t motor_pid_Cas_Yaw;
pid_Cascade_t motor_pid_Cas_Pitch;

/**
  * @brief PID������ʼ��
  */
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

/**
  * @brief  PID�ٶȻ�����
  */
float pid_calc(pid_struct_t *pid, float tar, float now)//λ��ʽ
{
  pid->tar = tar;
  pid->now = now;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->tar - pid->now;
  
  pid->p_out  = pid->kp * pid->err[0];//��������
  pid->i_out += pid->ki * pid->err[0];//���ֲ���
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//΢�ֲ���
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//�޷�
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;//������
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}

/**
  * @brief  ��̨6020�ٶȻ�PID����
  */
float pid_calc_cloud(pid_struct_t *pid, float tar, float now)//λ��ʽ
{
  pid->tar = tar;
  pid->now = now;

	pid->err[0] = ComputeMinOffset(pid->tar,pid->now);//���㴦��
  pid->p_out  = pid->kp * pid->err[0];//��������
  pid->i_out += pid->ki * pid->err[0];//���ֲ���
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//΢�ֲ���
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//�޷�
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;//������
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
	pid->err[1] = pid->err[0];
  return pid->output;
}

/**
  * @brief  ����3508�ٶ�λ�û�PID����
  */
float pid_CascadeCalc(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow)
{//�������������������Ϊ�ڲ�������Ĳο�ֵ���м���
	pid_calc(&pid->outer,angleTar,angleNow);//���Ƕ�
	pid_calc(&pid->inner,pid->outer.output,speedNow);//�ڲ��ٶ�
	pid->output=pid->inner.output;//������Ϊ�ڲ�����������
	return pid->output;
}

/**
  * @brief  ����3508�ٶ�λ�û�PID����
  */
float pid_CascadeCalc_chassis(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow)
{//�������������������Ϊ�ڲ�������Ĳο�ֵ���м���
	pid_calc_cloud(&pid->outer,angleTar,angleNow);//���Ƕ�
	pid_calc(&pid->inner,pid->outer.output,speedNow);//�ڲ��ٶ�
	pid->output=pid->inner.output;//������Ϊ�ڲ�����������
	return pid->output;
}

/**
  * @brief  ��̨6020�ٶ�λ�û�PID����
  */
float pid_CascadeCalc_cloud(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow)
{//�������������������Ϊ�ڲ�������Ĳο�ֵ���м���
	pid_calc_cloud(&pid->outer,angleTar,angleNow);//���Ƕ�
	pid_calc(&pid->inner,pid->outer.output,speedNow);//�ڲ��ٶ�
	pid->output=pid->inner.output;//������Ϊ�ڲ�����������
	return pid->output;
}

/**
  * @brief  ���PID������ʼ��
  */
void motor_pid_init()
{	
	for (uint8_t i = 0; i < 4; i++)
  {
    pid_init(&motor_pid_Cas_Chassis[i].inner,  500, 0.01f, 0, 30000, 10000);//init pid �ڻ��ٶ�, kp=40, ki=3, kd=0, output limit = 30000
		pid_init(&motor_pid_Cas_Chassis[i].outer,  2.5f, 0, 0, 30000, 10000);//init pid �⻷�Ƕ�, kp=40, ki=3, kd=0, output limit = 30000
  }
	
	pid_init(&motor_pid_chassis_pos,3.5f,0,0,30000,10000);
	
	pid_init(&motor_pid_shoot,0.4f,0.0f,1.1f,1000,10000);
	
	for (uint8_t i = 0; i < 4; i++)
	{
		pid_init(&motor_pid_chassis[i], 10.0f, 0.8f ,0, 8000, 19000); //init pid �ٶ�, kp=9.5, ki=0.3, kd=0, output limit = 15000
	}
	

	pid_init(&motor_pid_chassis_6020_stop[0].inner, 50.0f, 0.8f ,0, 4000, 9000); //init pid �ٶ�, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020_stop[0].outer, 0.5f,0,0,2000,1000);

	pid_init(&motor_pid_chassis_6020_stop[1].inner, 50.0f, 0.8f ,0, 4000, 9000); //init pid �ٶ�, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020_stop[1].outer, 0.5f,0,0,2000,1000);

	pid_init(&motor_pid_chassis_6020_stop[2].inner, 50.0f, 0.8f ,0, 4000, 9000); //init pid �ٶ�, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020_stop[2].outer, 0.5f,0,0,2000,1000);

	pid_init(&motor_pid_chassis_6020_stop[3].inner, 50.0f, 0.8f ,0, 4000, 9000); //init pid �ٶ�, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020_stop[3].outer, 0.5f,0,0,2000,1000);
	
	pid_init(&motor_pid_chassis_6020[0].inner, 50.0f, 0.8f ,0, 8000, 15000); //init pid �ٶ�, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020[0].outer, 15.0f,0,0,2000,1000);

	pid_init(&motor_pid_chassis_6020[1].inner, 50.0f, 0.8f ,0, 8000, 15000); //init pid �ٶ�, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020[1].outer, 15.0f,0,0,2000,1000);

	pid_init(&motor_pid_chassis_6020[2].inner, 50.0f, 0.8f ,0, 8000, 15000); //init pid �ٶ�, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020[2].outer, 15.0f,0,0,2000,1000);

	pid_init(&motor_pid_chassis_6020[3].inner, 50.0f, 0.8f ,0, 8000, 15000); //init pid �ٶ�, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020[3].outer, 15.0f,0,0,2000,1000); 
}
