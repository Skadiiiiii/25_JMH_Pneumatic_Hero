/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "dr16.h"
#include "M6020_motor.h"
#include "bsp_delay.h"
#include "bsp_can.h"
#include "INS_task.h"
#include "calibrate_task.h"
#include "IMU_Send_task.h"
#include "Task_CanMsg.h"
#include "Robot_control_task.h"
#include "Oled_Show_task.h"
#include "Task_Check.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId All_InitHandle;
osThreadId CAN1_TaskHandle;
osThreadId CAN2_TaskHandle;
osThreadId IMU_Send_TaskHandle;
osThreadId Cali_TaskHandle;
osThreadId Imu_TaskHandle;
osThreadId OLED_TaskHandle;
osThreadId Check_TaskHandle;
osThreadId Robot_Ctrl_TaskHandle;
osMessageQId CAN1_ReceiveHandle;
osMessageQId CAN2_ReceiveHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void All_Init_Run(void const * argument);
extern void CAN1_REC(void const * argument);
extern void CAN2_REC(void const * argument);
extern void IMU_Send(void const * argument);
extern void calibrate_task(void const * argument);
extern void INS_task(void const * argument);
extern void OLED_Show(void const * argument);
extern void DEV_Check(void const * argument);
extern void Robot_Control(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of CAN1_Receive */
  osMessageQDef(CAN1_Receive, 64, Can_Export_Data_t);
  CAN1_ReceiveHandle = osMessageCreate(osMessageQ(CAN1_Receive), NULL);

  /* definition and creation of CAN2_Receive */
  osMessageQDef(CAN2_Receive, 64, Can_Export_Data_t);
  CAN2_ReceiveHandle = osMessageCreate(osMessageQ(CAN2_Receive), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of All_Init */
  osThreadDef(All_Init, All_Init_Run, osPriorityNormal, 0, 128);
  All_InitHandle = osThreadCreate(osThread(All_Init), NULL);

  /* definition and creation of CAN1_Task */
  osThreadDef(CAN1_Task, CAN1_REC, osPriorityHigh, 0, 256);
  CAN1_TaskHandle = osThreadCreate(osThread(CAN1_Task), NULL);

  /* definition and creation of CAN2_Task */
  osThreadDef(CAN2_Task, CAN2_REC, osPriorityHigh, 0, 256);
  CAN2_TaskHandle = osThreadCreate(osThread(CAN2_Task), NULL);

  /* definition and creation of IMU_Send_Task */
  osThreadDef(IMU_Send_Task, IMU_Send, osPriorityHigh, 0, 128);
  IMU_Send_TaskHandle = osThreadCreate(osThread(IMU_Send_Task), NULL);

  /* definition and creation of Cali_Task */
  osThreadDef(Cali_Task, calibrate_task, osPriorityAboveNormal, 0, 512);
  Cali_TaskHandle = osThreadCreate(osThread(Cali_Task), NULL);

  /* definition and creation of Imu_Task */
  osThreadDef(Imu_Task, INS_task, osPriorityRealtime, 0, 1024);
  Imu_TaskHandle = osThreadCreate(osThread(Imu_Task), NULL);

  /* definition and creation of OLED_Task */
  osThreadDef(OLED_Task, OLED_Show, osPriorityBelowNormal, 0, 128);
  OLED_TaskHandle = osThreadCreate(osThread(OLED_Task), NULL);

  /* definition and creation of Check_Task */
  osThreadDef(Check_Task, DEV_Check, osPriorityNormal, 0, 256);
  Check_TaskHandle = osThreadCreate(osThread(Check_Task), NULL);

  /* definition and creation of Robot_Ctrl_Task */
  osThreadDef(Robot_Ctrl_Task, Robot_Control, osPriorityRealtime, 0, 512);
  Robot_Ctrl_TaskHandle = osThreadCreate(osThread(Robot_Ctrl_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_All_Init_Run */
/**
  * @brief  Function implementing the All_Init thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_All_Init_Run */
void All_Init_Run(void const * argument)
{
  /* USER CODE BEGIN All_Init_Run */
	
	delay_init();
	cali_param_init();
	bsp_can_init(); 
	motor_pid_init();   //电机pid初始化
	
	taskEXIT_CRITICAL();   //退出临界区
	vTaskDelete(NULL);  
	
  /* USER CODE END All_Init_Run */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
