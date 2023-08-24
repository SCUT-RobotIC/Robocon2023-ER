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

#include <stdio.h>
#include <stdint.h>
#include "bsp_can.h"
#include "chassis.h"
#include "usart.h"
#include "string.h"
#include "pick.h"
#include "friction.h"
#include "pitch.h"
#include "iwdg.h"
#include "pitch_init.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define weight 20  // the speed weight
#define DEAD_ZONE_MV 5


#define ch_vx_s1 	1
#define ch_vy_s1  0 
#define ch_wx_s1 	3 
#define ch_re_s1  2  // first to 160 than load  
#define ch_up_s1  6  // tap to up load 
#define ch_fr_s1  5  // up and donw // or
#define ch_ca_s1  4  // two type 
#define ch_pi_s1  7  // the pich



// 定义常量，用于表示数组的�?

const int ARRAY_SIZE = 8;

// 定义常量，用于表示死区的�?

const int DEAD_ZONE = 30 ;
	
int n = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t tx_buff[255];   //  tf
extern uint8_t receive_buff[8];// original  data 
int filter_buff[8];           //  filter  data 

extern const motor_measure_t   *motor_data[4]; // 3508 data
extern float wheel_speed[4];                   // 3508 speed
extern float  vx, vy,wz ;                      // vector speed

extern int state ; // watch dog 

float Position = 0 ;    														// pitch postion read
float target_position = 180 ;                      //  TGT postion




void filter_array(uint8_t* arr); // filter function
int my_abs(int x);               // abs cc 
int map_value(int x);            // map
int map_fire(int x);


int speed_fire    = 0 ;   // fire speed



int state_fire    =0 ; // state_fire
int state_last_fire = 0; //  last fire
int state_reload  =0 ; // state_reload
int state_last_reload = 0 ; // state_last
int state_upload  =0; // state_upload
int state_last_upload = 0; // last_upload
int state_catch   =0; // state_catch
int state_last_catch = 0;
int state_maxload =0; // state_maxload  // 0 - closed  1 - opened
int catch_delay = 0;
int reload_time = 0;
/* for debug
int p = 0;
float a = 0.0f;
*/
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, StartTask04, osPriorityIdle, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of myTask05 */
  osThreadDef(myTask05, StartTask05, osPriorityNormal, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	
	
	// in this task we mainly focus on data processing
	
	
	
  /* Infinite loop */
	for(;;)
  {
		filter_array(receive_buff);
		
		if(state >0 ){
			
			// chassis 
			vx = (filter_buff[ch_vx_s1] - 127)* weight;
			vy = (filter_buff[ch_vy_s1] - 127)* weight;
			wz = (filter_buff[ch_wx_s1] - 127)* weight;
			
			
			//pitch
		//	target_position = map_value(filter_buff[ch_pi_s1]);
			target_position = map_value(receive_buff[ch_pi_s1]);
			
			//reload
			
			if(filter_buff[ch_re_s1] > 150 ){
			
			
				state_reload = 0 ; 
				reload_time = 0;
				
			}else if(filter_buff[ch_re_s1] < 90){
			
				state_reload = 1 ; 
				if(reload_time == 0){
				
				target_position = 135  ;
							if(Position <= 137 ){
								reload_time = 1;
							}

				}
				else if(reload_time == 1){
				target_position = 140  ; // 	
				}
				
			//	target_position = 150;
			
			}else if(filter_buff[ch_re_s1] < 150 && filter_buff[ch_re_s1] > 90 ){
			
				state_reload = -1 ; 
				if(reload_time == 0){
				
				target_position = 135  ;
							if(Position  <= 137 ){
								reload_time = 1;
							}
				}
				else if(reload_time == 1){
				target_position = 140  ; //
					
				}
			//	target_position = 150; //
				
			
			}
			
		
			//catch 
			
			if(filter_buff[ch_ca_s1] > 150 ){
			
			
				state_catch = -1 ; 
				
				
			}else if(filter_buff[ch_ca_s1] < 90){
			
				state_catch = 1 ; 
			
			}else if(filter_buff[ch_ca_s1] < 150 && filter_buff[ch_ca_s1] > 90  )
			{
				state_catch = 0 ; 
			
			}
			
			//upload
			if(filter_buff[ch_up_s1] >= 127 ){
			
			
				state_upload = 0 ; 
				
				
			}else if(filter_buff[ch_up_s1] < 90){
			
				state_upload = 1 ; 
			
			
			}
			
			//fire
			
			/*
			if(filter_buff[ch_fr_s1] >= 127 ){
			
				
				state_fire = 0 ; 
				
			}else if(filter_buff[ch_fr_s1] < 127){
			
				state_fire= 1 ; 
			
			}*/
			
			
			speed_fire = map_fire(filter_buff[ch_fr_s1]) ; 
			
			
		
		
		}
		
		
    osDelay(10);
		
		
	
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	
  /* Infinite loop */
  for(;;)
  {
		if(state >=0){
			
			
			
			
			
			
			
		
			
			
			
			//reload a complex relad
			if(state_reload == 0){
				

			
						if((state_last_reload == 1 || state_last_reload == -1 ) && catch_delay == 1){
							
								int pulse = 700 ;
							
								
								
								while(pulse >= 550){
								
								
								
								PICK_Run_Pulse(1,10000-pulse);
								PICK_Run_Pulse(2,10000-(3000-pulse));
								
									
								osDelay(500);
								pulse -= 50;
								}
								
							
							
						
						catch_delay = 0;
						
						}
								
								if(state_catch !=  -1){
									
												if(state_last_catch == -1){
																	int pulse1 = 700 ;
															
																
																
																	while(pulse1 >= 550){
																	
																	
																	
																	PICK_Run_Pulse(1,10000-pulse1);
																	PICK_Run_Pulse(2,10000-(3000-pulse1));
																	
																		
																	osDelay(500);
																	pulse1 -= 50;
																}
																n+= 1;
													}
											
											PICK_Run_Pulse(1,10000-510);
											PICK_Run_Pulse(2,10000-(3000-510));
													
											state_last_catch = state_catch ;
										
								}else if(state_catch == -1){
												
											PICK_Run_Pulse(1,10000-1710);
											PICK_Run_Pulse(2,10000-(3000-1710));
											state_last_catch = state_catch ;
								
								
								}
						
			
						
							if(state_maxload == 1 ){
							
								PICK_Run_Pulse(3,10000-520);
								state_maxload  = 0 ;
							
							}
						
						
			catch_delay = 0;
			state_last_reload = state_reload;
				
			}
			
			else if(state_reload == 1){
			
				speed_fire = 0;
				
				if(state_last_reload == -1){
					
					
						PICK_Run_Pulse(1,10000-2090);
						PICK_Run_Pulse(2,10000-(3000-2090));
							
							
						
						
						osDelay(700);
					
					
						if(state_maxload ==1 ){
						
							
						PICK_Run_Pulse(3,10000-520); 				



						}	
						
						
						
					
						
						
						osDelay(500);
						
						PICK_Run_Pulse(4,10000-600);
						
				
				}
			catch_delay = 1;
			state_last_reload = state_reload  ;
			
			}
			else if(state_reload == -1){
			
			
				
				
				if(state_maxload == 1 && state_last_reload != -1 ){
					
				PICK_Run_Pulse(3,10000-520);
				state_maxload = 0 ;
			
				}	
				else if(state_maxload == 0 && state_last_reload != -1){
				
				PICK_Run_Pulse(3,10000-1500);
				
				state_maxload = 1;
				}
				
				
			state_last_reload = state_reload  ;
				
			}
			
			//maxload 
			
			
			
			/*
			if(state_fire == 0  ){
				
			
				
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1, 0);
				

				
				
			
			}else if(state_fire == 1){
			
			
					
			
			
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 10000);
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2, 10000);
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1, 10000);
				__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1, 10000);
					// fire in task4
	

				}*/
				
		
			
			//catch 
			if(state_catch == 0 && state_reload != 1 && state_reload != -1){
			
					PICK_Run_Pulse(4,10000-600);
			//	state_last_catch = state_catch ;
					
			
			}else if(state_catch == 1 && state_reload != 1 && state_reload != -1){
			
			
				PICK_Run_Pulse(4,10000-1200);
		//	state_last_catch = state_catch ;	
			}
			
			
			// upload 
			
			if(state_upload == 0 ){
				
			
					PICK_Run_Pulse(0, 10000-2500);
				
			
			}else if(state_upload == 1){
			
			
					
					PICK_Run_Pulse(0, 10000-1100);
			
			
				}
		
		
		
			}		

    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
	// in this task we focus on data read
	
  /* Infinite loop */
  for(;;)
  {
		if(state > 0){	
			chassis_vector_to_mecanum_wheel_speed(vx,vy,wz,wheel_speed);
			chassis_control_loop(wheel_speed);
			
			if(state_reload == 1){
					speed_fire = 0;
				
				
			}
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, speed_fire);
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2, speed_fire);
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1, speed_fire);
				__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1, speed_fire);
				
		}
		
		
	
		
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
						
		HAL_IWDG_Refresh(&hiwdg);  // hiwdg
		
		Position = Pitch_Get_Position(0);
		// chassis 
		
		chassis_state_update();
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void const * argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for(;;)
  {
		if(state >0){
			

				// pitch 
			pid_pitch_control(target_position,state_reload);
		}
    osDelay(1);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// data filter




void filter_array(uint8_t* arr) {
    
    int last_value = arr[0];
    // 遍历输入数组中的每个元素
    for (int i = 0; i < ARRAY_SIZE; i++) {
        // 如果当前元素和上丿次的�?�相差很大（比如大于100�?
        if (my_abs(arr[i] - last_value) > 100) {
            // 则保留上丿次的�??
            filter_buff[i] = last_value;
        }
        // 否则，如果当前元素在127附近有一个死�?
				else if(i == ch_vx_s1 || i == ch_vy_s1 || i == ch_wx_s1){
				
						if(arr[i] >= 127 - DEAD_ZONE_MV  && arr[i] <= 127 + DEAD_ZONE_MV ){
								arr[i] = 127;
						}
				
				}
        else if (arr[i] >= 127 - DEAD_ZONE && arr[i] <= 127 + DEAD_ZONE) {
            // 则将当前元素设为127
							arr[i] = 127;
        }
        // 否则，将当前元素保留
        else {
            // do nothing
        }
        // 更新上一次的�?
        filter_buff[i] = arr[i];
    }
	}

	
int my_abs(int x) {
    // 如果x是正数或者零，直接返回x
    if (x >= 0) {
        return x;
    }
    // 否则，用平方的方式来计算x的绝对忿
    else {
        // 定义丿个变量，用于存储x的平方根
        double sqrt_x = sqrt((double)x * x);
        // 将平方根转换为整数，并返�?
        return (int)sqrt_x;
    }
		
}

// 定义丿个函数，接收丿�?0-255的忼，返回丿�?200-160的忿
int map_value(int x) {
    // 定义丿个变量，用于存储输出结�?
    int y;
    // 如果x小于0，将其设�?0
    if (x < 0) {
        x = 0;
    }
    // 如果x大于255，将其设�?255
    if (x > 255) {
        x = 255;
    }
    // 根据线濧映射公式，计算y的忿
    y = 180 + (130- 180 )* (x) / 255;
    // 返回y的忿
    return y;
}


int map_fire(int x) {
  // 使用线濧插�?�的公式，y = y1 + (y2 - y1) * (x - x1) / (x2 - x1)
  // 其中，x1 = 0, x2 = 255, y1 = 8000, y2 = 10000
	 if (x < 60) {
    return 0;
  }
	 
  int y = 2000 + (8000 - 2000) * (x - 60) / (255 - 60);
	
	if (x > 250) {
    return 10000;
  }
	
	
	
	
  return y ;
}

/* USER CODE END Application */
