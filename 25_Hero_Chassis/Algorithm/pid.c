#include "pid.h"
#include "M3508_motor.h"
#include "M6020_motor.h"
#include "cloud_control.h"
#include "AddMath.h"

/* 底盘 */
pid_struct_t motor_pid_chassis[4];
pid_Cascade_t motor_pid_chassis_6020[4];
pid_Cascade_t motor_pid_chassis_6020_stop[4];

pid_struct_t motor_pid_chassis_pos;

/* 云台 */
pid_Cascade_t motor_pid_Cas_Yaw;
pid_Cascade_t motor_pid_Cas_Pitch;

/* 拨盘 */
pid_struct_t motor_pid_dial_speed;

/**
  * @brief  PID参数初始化
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
  * @brief  PID速度环计算
  */
float pid_calc(pid_struct_t *pid, float tar, float now)//位置式
{
  pid->tar = tar;
  pid->now = now;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->tar - pid->now;
  
  pid->p_out  = pid->kp * pid->err[0];//比例部分
  pid->i_out += pid->ki * pid->err[0];//积分部分
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//微分部分
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//限幅
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;//控制量
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}

/**
  * @brief  PID增量式计算
  */
float pid_calc_incremental(pid_struct_t *pid, float tar, float now)//增量式
{
	pid->tar = tar;
  pid->now = now;
	pid->err[2] = pid->err[1];
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->tar - pid->now;
	
	pid->p_out  = pid->kp * (pid->err[0] - pid->err[1]);//比例部分
  pid->i_out  = pid->ki * pid->err[0];//积分部分
  pid->d_out  = pid->kd * (pid->err[0] - 2*pid->err[1] + pid->err[2]);//微分部分
	
	LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//限幅
	
  pid->delta_u = pid->p_out + pid->i_out + pid->d_out;//控制量
	pid->delta_out = pid->last_delta_out + pid->delta_u;
	
	LIMIT_MIN_MAX(pid->delta_out, -pid->out_max, pid->out_max);
	pid->last_delta_out = pid->delta_out;
  return pid->delta_out;
}

/**
  * @brief  云台6020速度环PID计算
  */
float pid_calc_cloud(pid_struct_t *pid, float tar, float now)//位置式
{
  pid->tar = tar;
  pid->now = now;

	pid->err[0] = ComputeMinOffset(pid->tar,pid->now);//过零处理
  pid->p_out  = pid->kp * pid->err[0];//比例部分
  pid->i_out += pid->ki * pid->err[0];//积分部分
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//微分部分
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//限幅
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;//控制量
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
	pid->err[1] = pid->err[0];
  return pid->output;
}

/**
  * @brief  底盘3508速度位置环PID计算
  */
float pid_CascadeCalc(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow)
{//外层控制器输出控制量作为内层控制器的参考值进行计算
	pid_calc(&pid->outer,angleTar,angleNow);//外层角度
	pid_calc(&pid->inner,pid->outer.output,speedNow);//内层速度
	pid->output=pid->inner.output;//控制量为内层控制器的输出
	return pid->output;
}

/**
  * @brief  底盘3508速度位置环PID计算
  */
float pid_CascadeCalc_chassis(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow)
{//外层控制器输出控制量作为内层控制器的参考值进行计算
	pid_calc_cloud(&pid->outer,angleTar,angleNow);//外层角度
	pid_calc(&pid->inner,pid->outer.output,speedNow);//内层速度
	pid->output=pid->inner.output;//控制量为内层控制器的输出
	return pid->output;
}

/**
  * @brief  云台6020速度位置环PID计算
  */
float pid_CascadeCalc_cloud(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow)
{//外层控制器输出控制量作为内层控制器的参考值进行计算
	pid_calc_cloud(&pid->outer,angleTar,angleNow);//外层角度
	pid_calc(&pid->inner,pid->outer.output,speedNow);//内层速度
	pid->output=pid->inner.output;//控制量为内层控制器的输出
	return pid->output;
}

/**
  * @brief  电机PID参数初始化
  */
void motor_pid_init()
{	
	pid_init(&motor_pid_chassis_pos,3.5f,0,0,30000,10000);
	
	pid_init(&motor_pid_dial_speed,10.0f,0.6f,0.0f,1000,10000);
	
	for (uint8_t i = 0; i < 4; i++)
	{
		pid_init(&motor_pid_chassis[i], 10.0f, 0.8f ,0, 8000, 19000); //init pid 速度, kp=9.5, ki=0.3, kd=0, output limit = 15000
	}
	
	pid_init(&motor_pid_Cas_Yaw.inner,0.00385f,0.0157f,0,4,6);
	pid_init(&motor_pid_Cas_Yaw.outer,54,0,0,160,5000);
	

	pid_init(&motor_pid_chassis_6020_stop[0].inner, 50.0f, 2.5f ,0, 6000, 9000); //init pid 速度, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020_stop[0].outer, 1.5f,0,0,2000,1000);

	pid_init(&motor_pid_chassis_6020_stop[1].inner, 50.0f, 2.5f ,0, 6000, 9000); //init pid 速度, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020_stop[1].outer, 1.5f,0,0,2000,1000);

	pid_init(&motor_pid_chassis_6020_stop[2].inner, 50.0f, 2.5f ,0, 6000, 9000); //init pid 速度, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020_stop[2].outer, 1.5f,0,0,2000,1000);

	pid_init(&motor_pid_chassis_6020_stop[3].inner, 50.0f, 2.5f ,0, 6000, 9000); //init pid 速度, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020_stop[3].outer, 1.5f,0,0,2000,1000);
	
	pid_init(&motor_pid_chassis_6020[0].inner, 60.0f, 2.5f ,0, 8000, 15000); //init pid 速度, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020[0].outer, 20.0f,0,0,2000,2000);

	pid_init(&motor_pid_chassis_6020[1].inner, 60.0f, 2.5f ,0, 8000, 15000); //init pid 速度, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020[1].outer, 20.0f,0,0,2000,2000);

	pid_init(&motor_pid_chassis_6020[2].inner, 60.0f, 2.5f ,0, 8000, 15000); //init pid 速度, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020[2].outer, 15.0f,0,0,2000,2000);

	pid_init(&motor_pid_chassis_6020[3].inner, 60.0f, 2.5f ,0, 8000, 15000); //init pid 速度, kp=9.5, ki=0.3, kd=0, output limit = 15000
	pid_init(&motor_pid_chassis_6020[3].outer, 10.0f,0,0,2000,2000); 
}

