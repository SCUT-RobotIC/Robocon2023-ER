/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include <stdio.h>
#include <string.h>
#include "friction.h"
#include "pick.h"
#include "pitch.h"
#include "chassis.h"

#include "pitch_init.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*

#define ch_vx_s1 	1
#define ch_vy_s1  0 
#define ch_wx_s1 	3 
#define ch_re_s1  2  // first to 160 than load  
#define ch_up_s1  6  // tap to up load 
#define ch_fr_s1  5  // up and donw // or
#define ch_ca_s1  4  // two type 
#define ch_pi_s1  7  // the pich

*/


uint8_t receive_buff[8] = {127,127,255,127,127,127,0,0}; // init 
uint8_t tx_buff[255];



int state = 0; // swatch dog


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
int32_t set_spd = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	
		
	HAL_IWDG_Init(&hiwdg); 	
		
		HAL_TIM_PWM_Init(&htim1);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);         //使能串UART1 IDLE中断
		HAL_UART_Receive_DMA(&huart3, (uint8_t*)receive_buff, 255);   //设置DMA传输，串�?1的数据搬运到recvive_buff中，
		
	can_filter_init();
	HAL_CAN_Start(&hcan1); 
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_TIM_Base_Start_IT(&htim6);
	
	// friction wheel
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	// friction wheel 
	FRICTION_InitMotor(0, &htim1, &htim8, TIM_CHANNEL_1, TIM_CHANNEL_2);
  FRICTION_InitMotor(1, &htim8, &htim9, TIM_CHANNEL_1, TIM_CHANNEL_1);
		
	// upload servo
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // PD 12 
	PICK_InitMotor(0, &htim4, TIM_CHANNEL_1); // upload - 0
	
	PICK_Run_Pulse(0, 10000-2500);
	
//	HAL_Delay(500);
	// reload servo
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // PB 10
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // PB 11
	PICK_InitMotor(1, &htim2, TIM_CHANNEL_3); // reload- 1 the one 500
	PICK_InitMotor(2, &htim2, TIM_CHANNEL_4); // reload- 2
	
	
	PICK_Run_Pulse(1, 10000-510);
	PICK_Run_Pulse(2, 10000-(3000-510));
	
	
//	HAL_Delay(500);
	// maxload  servo
	
	// ingnore it first 
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // PD 14
	PICK_InitMotor(3, &htim4, TIM_CHANNEL_3); // max load- 3
	
	PICK_Run_Pulse(3, 10000-520);
	
	
//	HAL_Delay(500);
	// catch servo
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // PD 15
	PICK_InitMotor(4, &htim4, TIM_CHANNEL_4); // catch- 4
	
	PICK_Run_Pulse(4, 10000-550);
	HAL_IWDG_Refresh(&hiwdg); 
	
	
//	HAL_Delay(500);
	
	
	// pitch init 
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	Pitch_InitMotor(0, &htim1, &htim3, TIM_CHANNEL_4, TIM_CHANNEL_3, 1, 0, 0);
	pitch_init();
	
//	pid_pitch_control(180);
	
	
	
	
	// chassis_init
	chassis_init();
 	
	CAN_cmd_chassis(0,0,0,0);


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if (htim == &htim6){
		
		state --;
		if(state <= 0){
			
			
			CAN_cmd_chassis(0,0,0,0);
			
	//		FRICTION_SetThrottle(0, 0);
	//		FRICTION_SetThrottle(1, 0);
	//		FRICTION_Run(0);
	//		FRICTION_Run(1);
	

		}
	
		
		
	}
	
	
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
