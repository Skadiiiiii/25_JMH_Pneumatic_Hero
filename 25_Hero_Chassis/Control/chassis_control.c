#include "chassis_control.h"

int16_t speed_buff[4];		/*<! 驱动轮3508目标转速 */
RUD_Param_t RUD_Param[4]; /*<! 转向轮6020相关参数 */

uint8_t stop_pid_flag;		/*<! 静止PID标志位 */


/**
  * @brief  设置底盘转向轮6020初始角度（45°归中）
  */
void Chassis_Init(void)
{
	M6020s_chassis[LF_207_6020].Init_angle = LF_207_6020_Init_Angle + 45;
	M6020s_chassis[RF_206_6020].Init_angle = RF_206_6020_Init_Angle - 45;
	M6020s_chassis[RB_205_6020].Init_angle = RB_205_6020_Init_Angle + 45;
	M6020s_chassis[LB_208_6020].Init_angle = LB_208_6020_Init_Angle - 45;
	
	RUD_Param[LF_207_6020].Init_angle = LF_207_6020_Init_Angle + 45;
	RUD_Param[RF_206_6020].Init_angle = RF_206_6020_Init_Angle - 45;
	RUD_Param[RB_205_6020].Init_angle = RB_205_6020_Init_Angle + 45;
	RUD_Param[LB_208_6020].Init_angle = LB_208_6020_Init_Angle - 45;
	
	stop_pid_flag = 1;
}

/**
  * @brief  底盘转向轮6020角度限制
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
  * @brief  计算转向轮6020当前角度
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

    if(reset == true)  //---圈数清零
    {
        RUD_Param[motor_num].Turns_cnt = 0;
    }

    if(opposite == false)   //---是否反向
    {
        RUD_Param[motor_num].Total_angle = Cur_encoder + RUD_Param[motor_num].Turns_cnt*360;
    }
    else if(opposite == true)
    {
        RUD_Param[motor_num].Total_angle = -Cur_encoder - RUD_Param[motor_num].Turns_cnt*360;
    }
}

/**
  * @brief  计算转向轮6020目标角度
  */
float error;
uint8_t Move_flag;
uint16_t Brake_cnt;//--- 刹车计数器
uint8_t No_move_flag = false;//--- 静止标志位
static void RUDTargetAngle_Calc(int8_t motor_num , int8_t reset , uint8_t opposite)
{
    float Cur_Target = RUD_Param[motor_num].Target_angle;

    static float temp_pre = 0;  //--- 保存劣弧旋转的上一次未经过零处理的目标角度

    //--- 将目标角度与当前角度换算到同一个周期中
    RUD_Param[motor_num].TarTurns_cnt = (int32_t)(RUD_Param[motor_num].Total_angle/180.0f) - (int32_t)(RUD_Param[motor_num].Total_angle/360.0f);

    RUD_Param[motor_num].Target_angle = RUD_Param[motor_num].TarTurns_cnt*360.0f + Cur_Target + RUD_Param[motor_num].Init_angle;

    temp_pre = Cur_Target;

    //--- 当前目标角度和当前角度的误差
    error = RUD_Param[motor_num].Target_angle - RUD_Param[motor_num].Total_angle;

    AngleLimit(&error);

    //--- 如果角度误差大于90度则将速度反向并将目标角度叠加180度
    if(fabs(error) > 90.0f && (Move_flag == true || No_move_flag == false || Brake_cnt < 500))
    {
        //--- 目标值叠加半个周期
        RUD_Param[motor_num].Target_angle += 180.0f;

        temp_pre += 180.0f;

        //--- 驱动轮反转
				speed_buff[motor_num] = -speed_buff[motor_num];

        //--- 确保目标角度和当前角度处于同一个周期
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
 * @brief  舵轮运动解算(转向轮)
 */
uint8_t spin_flag;
float Radius = 1.0f;  // 圆心距
static void RudAngle_Calc(int16_t Vx, int16_t Vy, int16_t Vw)
{
    float const theta = atan(1.0/1.0);
    static uint16_t No_move_cnt = 0;

    static float last_vx = 0, last_vy = 0, last_vw = 0;

    if(Vx == 0 && Vy == 0)
    {
        No_move_flag = true;
        if(abs(Vw) < 70) //---IMU零漂产生的自旋速度
        {
            if(No_move_cnt < 500) //--- 静止后1000ms期间不允许底盘跟随
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

            //--- 在上一帧不是静止的时候产生一个Vw的速度，为了不让他目标角度有一个归零的动作
            last_vw = (spin_flag == true ? 50 : 0);

            //--- 使用上一次的目标速度来保存上一次的目标角度
            RUD_Param[LF_207_6020].Target_angle = atan2(last_vx - last_vw*(Radius*arm_sin_f32(theta)),last_vy - last_vw*Radius*arm_cos_f32(theta))*(180/PI);
						RUD_Param[RF_206_6020].Target_angle = atan2(last_vx - last_vw*(Radius*arm_sin_f32(theta)),last_vy + last_vw*Radius*arm_cos_f32(theta))*(180/PI);
            RUD_Param[RB_205_6020].Target_angle = atan2(last_vx + last_vw*(Radius*arm_sin_f32(theta)),last_vy + last_vw*Radius*arm_cos_f32(theta))*(180/PI);
						RUD_Param[LB_208_6020].Target_angle = atan2(last_vx + last_vw*(Radius*arm_sin_f32(theta)),last_vy - last_vw*Radius*arm_cos_f32(theta))*(180/PI);
        }
        else
        {
            spin_flag = false;
            //--- 45度归中
            RUD_Param[LF_207_6020].Init_angle = LF_207_6020_Init_Angle + 45;
						RUD_Param[RF_206_6020].Init_angle = RF_206_6020_Init_Angle - 45;
            RUD_Param[RB_205_6020].Init_angle = RB_205_6020_Init_Angle + 45;
						RUD_Param[LB_208_6020].Init_angle = LB_208_6020_Init_Angle - 45;

						stop_pid_flag = 1;
					
            //--- 目标角度归零
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

        //--- 解除45度归中
				RUD_Param[LF_207_6020].Init_angle = LF_207_6020_Init_Angle;
        RUD_Param[RF_206_6020].Init_angle = RF_206_6020_Init_Angle;
        RUD_Param[RB_205_6020].Init_angle = RB_205_6020_Init_Angle;
        RUD_Param[LB_208_6020].Init_angle = LB_208_6020_Init_Angle;    

        //--- 有目标速度的时候才进行舵轮解算的计算
        RUD_Param[LF_207_6020].Target_angle = atan2(Vx - Vw*(Radius*arm_sin_f32(theta)),Vy - Vw*Radius*arm_cos_f32(theta))*(180/PI);
				RUD_Param[RF_206_6020].Target_angle = atan2(Vx - Vw*(Radius*arm_sin_f32(theta)),Vy + Vw*Radius*arm_cos_f32(theta))*(180/PI);
        RUD_Param[RB_205_6020].Target_angle = atan2(Vx + Vw*(Radius*arm_sin_f32(theta)),Vy + Vw*Radius*arm_cos_f32(theta))*(180/PI);
				RUD_Param[LB_208_6020].Target_angle = atan2(Vx + Vw*(Radius*arm_sin_f32(theta)),Vy - Vw*Radius*arm_cos_f32(theta))*(180/PI);

        if(abs(Vw)>100)
        {
            spin_flag = true;
        }

        //--- 无目标速度的时候不使用上一次角度来保存是因为跟随模式下IMU静止的瞬间会产生轻微的Vw速度
        last_vx = Vx;
        last_vy = 0;
        last_vw = 0;
    }
}

/**
 * @brief  舵轮运动解算(驱动轮)
 */
static void Wheel_calc(int16_t Vx, int16_t Vy, int16_t Vw, int16_t *cal_speed)
{
    float Param = 1.0f;

		float const theta = atan(1.0/1.0);//代表45°

    static uint16_t No_move_cnt = 0;
    
    /* 三轴速度解算转向轮角度 ---------------------------------------------------------------------------*/
    RudAngle_Calc(Vx, Vy, Vw);

    if(Vx == 0 && Vy == 0)
    {
        if(abs(Vw) < 70) //---IMU零漂产生的自旋速度
        {
            if(No_move_cnt < 500/* 500 */) //--- 静止后1000ms期间不允许底盘跟随
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

    /* 驱动轮 速度解算 ---------------------------------------------------------------------------------*/
    cal_speed[RF_202_3508] = -sqrt(pow(Vx - Vw*arm_sin_f32(theta),2) + pow(Vy + Vw*arm_cos_f32(theta),2));
    cal_speed[LF_203_3508] =  sqrt(pow(Vx - Vw*arm_sin_f32(theta),2) + pow(Vy - Vw*arm_cos_f32(theta),2));
    cal_speed[RB_201_3508] = -sqrt(pow(Vx + Vw*arm_sin_f32(theta),2) + pow(Vy - Vw*arm_cos_f32(theta),2));
		cal_speed[LB_204_3508] =  sqrt(pow(Vx + Vw*arm_sin_f32(theta),2) + pow(Vy + Vw*arm_cos_f32(theta),2));

		cal_speed[0] *= Param;
		cal_speed[1] *= Param;
		cal_speed[2] *= Param;
		cal_speed[3] *= Param;


    /* 计算转向轮角度当前值 ----------------------------------------------------------------------------*/
    RUDTotalAngle_Calc(0, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(1, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(2, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTotalAngle_Calc(3, RUD_NOT_RESET, RUD_NOT_OPSI);

    /* 计算转向轮角度目标值 ----------------------------------------------------------------------------*/
    RUDTargetAngle_Calc(0, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(1, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(2, RUD_NOT_RESET, RUD_NOT_OPSI);
    RUDTargetAngle_Calc(3, RUD_NOT_RESET, RUD_NOT_OPSI);

    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        //--- 转向轮 劣弧旋转
        RUD_Param[i].Target_angle = Turn_InferiorArc(RUD_Param[i].Target_angle, RUD_Param[i].Total_angle);
    }
}

/**
 * @brief  获取开机后的6020中心角度，并设为基准
 */
float M6020s_Yaw_Angle_Centre;
static void read_start_yaw(void)
{
		M6020s_Yaw_Angle_Centre = M6020s_Yaw.rotor_angle;
}

/**
 * @brief  全向公式
 */
static float* Speed_Decompose(float Vx, float Vy)
{
	float RadRaw = 0.0f;
	static float Chassis[2];
	
	float angle = (M6020s_Yaw.rotor_angle - M6020s_Yaw_Angle_Centre) / M6020_mAngleRatio; //机械角度偏差
	RadRaw = angle * DEG_TO_RAD;   
	
  Chassis[0] = Vx * arm_cos_f32(RadRaw) - Vy * arm_sin_f32(RadRaw);
	Chassis[1] = Vy * arm_cos_f32(RadRaw) + Vx * arm_sin_f32(RadRaw);
	
	return Chassis;
}

/**
  * @brief  底盘使能
  */
void Ship_ChassisWorkMode(float Vx, float Vy,float VOmega)
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
	
	chassis_power_control(&Chassis_PowerLimit);

	set_M3508_200_current(&hcan2,
							M3508s_chassis[0].set_current, 
							M3508s_chassis[1].set_current, 
							M3508s_chassis[2].set_current,
							M3508s_chassis[3].set_current);	
	
	set_M6020_1fe_current(&hcan2,
							M6020s_chassis[0].set_current, 
							M6020s_chassis[1].set_current, 
							M6020s_chassis[2].set_current, 
							M6020s_chassis[3].set_current);	
	
//		set_M3508_200_current(&hcan2,0,0,0,0);	
//		set_M6020_1fe_current(&hcan2,0,0,0,0);	

}

/**
  * @brief  底盘跟随模式
  */
void Ship_ChassisWorkMode_follow(float Vx, float Vy)
{
	int16_t Chassis_target_rotor_speed;

	Chassis_target_rotor_speed = pid_calc_cloud(&motor_pid_chassis_pos,M6020s_Yaw_Angle_Centre,M6020s_Yaw.rotor_angle);
	
	Wheel_calc(Vx,Vy,-Chassis_target_rotor_speed,speed_buff);
								
	for (uint8_t i = 0; i < 4; i++)
	{
			M3508s_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], speed_buff[i], M3508s_chassis[i].rotor_speed);
		
//			if(stop_pid_flag == 1)
//			{
//				M6020s_chassis[i].set_current = pid_CascadeCalc(&motor_pid_chassis_6020_stop[i], RUD_Param[i].Target_angle, RUD_Param[i].Total_angle,M6020s_chassis[i].rotor_speed);
//			}
//			else
//			{
				M6020s_chassis[i].set_current = pid_CascadeCalc(&motor_pid_chassis_6020[i], RUD_Param[i].Target_angle, RUD_Param[i].Total_angle,M6020s_chassis[i].rotor_speed);
//			}
	}

	set_M3508_200_current(&hcan2,
							M3508s_chassis[0].set_current,
							M3508s_chassis[1].set_current,
							M3508s_chassis[2].set_current,
							M3508s_chassis[3].set_current);	
	
	set_M6020_1fe_current(&hcan2,
							M6020s_chassis[0].set_current, 
							M6020s_chassis[1].set_current, 
							M6020s_chassis[2].set_current, 
							M6020s_chassis[3].set_current);		
}

/**
  * @brief  底盘小陀螺模式
  */
void Ship_ChassisWorkMode_Tuoluo(float Vx, float Vy)
{
	float* Chassis = Speed_Decompose(Vx,Vy);
	float target_Vomega = 1500;
	
	Wheel_calc(Chassis[0],Chassis[1],-target_Vomega,speed_buff);
								
	for (uint8_t i = 0; i < 4; i++)
	{
			M3508s_chassis[i].set_current = pid_calc(&motor_pid_chassis[i], speed_buff[i], M3508s_chassis[i].rotor_speed);
		
			M6020s_chassis[i].set_current = pid_CascadeCalc(&motor_pid_chassis_6020[i], RUD_Param[i].Target_angle, RUD_Param[i].Total_angle,M6020s_chassis[i].rotor_speed);
	}

	set_M3508_200_current(&hcan2,
							M3508s_chassis[0].set_current, 
							M3508s_chassis[1].set_current, 
							M3508s_chassis[2].set_current,
							M3508s_chassis[3].set_current);	
	
	set_M6020_1fe_current(&hcan2,
							M6020s_chassis[0].set_current, 
							M6020s_chassis[1].set_current, 
							M6020s_chassis[2].set_current, 
							M6020s_chassis[3].set_current);	
}

/**
  * @brief  底盘失能
  */
void Robot_control_chassis_disable()
{
	for(uint8_t i = 0;i < 4;i++)
	{
		M3508s_chassis[i].set_current = 0;
		M6020s_chassis[i].set_current = 0;
	}
	set_M3508_200_current(&hcan2,0,0,0,0);
	set_M6020_1fe_current(&hcan2,0,0,0,0);
}


