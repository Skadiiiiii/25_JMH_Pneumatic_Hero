#include "Oled_Show_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "DJI_OLED.h"



void OLED_Show(void const *argument)
{	
	oled_init();
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(20); //每2毫秒强制进入总控制
	for (;;)
  {
		Oled();

		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
}

