#include "cloud_control.h"
#include "bsp_can.h"
#include "M3508_motor.h"
#include "M6020_motor.h"
#include "pid.h"


/**
  * @brief  ���㴦����������Сƫ��
  */
int ComputeMinOffset(int target, int value) 
{
    int err = target - value;
	
    if (err > 4096)
    {
        err -= 8191;
    }
    else if (err < -4096)
    {
        err += 8191;
    }
    return err;
}

/**
 * @brief      ������Сƫ�ʹת���ֱ����ӻ���ת
 */
float Turn_InferiorArc(float target, float current)
{
  float Error = target - current;

  if(Error > 180.0f)
	{
		return (target - 360.0f);
	}
	else if(Error < -180.0f)
	{
		return (target + 360.0f);
	}
	else
	{
		return target;
	}
}

