/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
#include "chassis.h"
#include "motors.h"
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
uint16_t remotedata[13];
uint16_t lastRemote[13];
/* USER CODE END Variables */
/* Definitions for mySendCan */
osThreadId_t mySendCanHandle;
const osThreadAttr_t mySendCan_attributes = {
  .name = "mySendCan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskAssign */
osThreadId_t myTaskAssignHandle;
const osThreadAttr_t myTaskAssign_attributes = {
  .name = "myTaskAssign",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void SendCanTask(void *argument);
void TaskAssignTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  for(int i = 0;i < 13;i++)
	{
		lastRemote[i]=0;
	}
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mySendCan */
  mySendCanHandle = osThreadNew(SendCanTask, NULL, &mySendCan_attributes);

  /* creation of myTaskAssign */
  myTaskAssignHandle = osThreadNew(TaskAssignTask, NULL, &myTaskAssign_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_SendCanTask */
/**
  * @brief  Function implementing the mySendCan thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SendCanTask */
void SendCanTask(void *argument)
{
  /* USER CODE BEGIN SendCanTask */
  /* Infinite loop */
  for(;;)
  {
		chassis_sendcan();
		chassis_pid();
    osDelay(1);
  }
  /* USER CODE END SendCanTask */
}

/* USER CODE BEGIN Header_TaskAssignTask */
/**
* @brief Function implementing the myTaskAssign thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskAssignTask */
void TaskAssignTask(void *argument)
{
  /* USER CODE BEGIN TaskAssignTask */
  /* Infinite loop */
	int flag1=0;
  for(;;)
  {
		if(!((lastRemote[2]==remotedata[2])&(lastRemote[3]==remotedata[3])&(lastRemote[6]==remotedata[6])))
		{
		chassis_changeSpeed(remotedata[2], remotedata[3], remotedata[6]);
		lastRemote[2] = remotedata[2];
		lastRemote[3] = remotedata[3];
		lastRemote[6] = remotedata[6];
		}
		
		//按键
		if(lastRemote[8]!=remotedata[8]){
			if(remotedata[8]!=0)
			{
				if((remotedata[8]>>4)&1)//开启/关闭摩擦轮
				{
					if(flag1==0){
					 Shot_Control(99);
					 flag1=1;
					}
				  else{
					 Shot_Control(0);
					 flag1=0;
				  }
			  }
			  else if((remotedata[8]>>4)&1)//弹仓上升
				{
					/*功能函数*/
			  }
        else if((remotedata[8]>>4)&1)//弹仓下降
				{
					/*功能函数*/
			  }				
		  }	
      lastRemote[8] = remotedata[8];			
	}
	
	
		
			
    osDelay(1);
	vTaskSuspend(myTaskAssignHandle);
  }
  /* USER CODE END TaskAssignTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

