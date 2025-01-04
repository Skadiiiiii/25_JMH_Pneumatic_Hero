#include "power_limit_control.h"
#include "Power_Meter.h"
#include "PowerController.h"
#include "RLS.hpp"

static Matrixf<2, 1> samples;
static Matrixf<2, 1> params;
Core::Control::Math::RLS<2> rls(1e-5f, 0.99999f);

void rlsupdate()
{
	params     = rls.update(samples, Power_Meter.Chassis_RealPower - Chassis_Power_M6020.k3 - Chassis_Power_M3508.commandPower - Chassis_Power_M3508.k3);//测量功率减去有效功率再减去k3
	Chassis_Power_M3508.k1 = fmax(params[0][0], 1e-5f);  // In case the k1 diverge to negative number
	Chassis_Power_M3508.k2 = fmax(params[1][0], 1e-5f);  // In case the k2 diverge to negative number，更新k1和k2的值，确保它们不小于1e-5f，以防止它们出现负值

//	params     = rls.update(samples, Power_Meter.Chassis_RealPower - Chassis_Power_M3508.k3 - Chassis_Power_M6020.commandPower - Chassis_Power_M6020.k3);//测量功率减去有效功率再减去k3
//	Chassis_Power_M6020.k1 = fmax(params[0][0], 1e-5f);  // In case the k1 diverge to negative number
//	Chassis_Power_M6020.k2 = fmax(params[1][0], 1e-5f);  // In case the k2 diverge to negative number，更新k1和k2的值，确保它们不小于1e-5f，以防止它们出现负值
}

