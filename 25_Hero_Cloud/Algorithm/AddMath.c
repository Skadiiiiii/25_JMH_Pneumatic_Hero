/**
  ******************************************************************************
  * @file    UserMath.c
  * @author  IMTao
  * @version V1.1
  * @date
  * @brief
  ******************************************************************************
  */
#include "AddMath.h"

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double powDirect(double x, double y) { //保留方向的幂次计算函数。
	double res = pow(abs(x), y);
	if (x > 0)
	{
		return abs(res);
	}
	else
	{
		return  -abs(res);
	}


}


//限幅
int Constrain_int(int amt, int low, int high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}



float Constrain_float(float amt, float low, float high)
{
	if (amt < low)
			return low;
	else if (amt > high)
			return high;
	else
			return amt;
}

void Constrain( float *val, float min, float max)
{
    if (*val <= min)
    {
        *val =  min;
    }
    else if(*val >= max)
    {
        *val =  max;
    }
}


