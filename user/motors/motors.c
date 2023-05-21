#include "motors.h"
#include "tim.h"

void USER_TIM_PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

void Shot_Control(uint16_t pwmval)
{
	// 一左
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwmval);// L_PWM
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);// R_PWM
	// 一右
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwmval);// L_PWM
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);// R_PWM
	// 二左
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwmval);
	// 二右
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwmval);
	// 三左
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwmval);
	// 三右
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwmval);
}


void Servo_Control1(uint16_t angle)
{
	angle = (0.5 + angle / 180.0 * (2.5 - 0.5)) / 20.0 * 1679;
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, angle);
}

void Servo_Control2 (uint16_t angle)
{
	angle = (0.5 + (float)(angle) / 270.0 * (2.5 - 0.5)) / 20.0 * 1679;
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, angle);
}

void Rise_Control_Left(uint16_t pwmval1,uint16_t pwmval2){
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwmval1);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwmval2);
}

void Rise_Control_Right(uint16_t pwmval1,uint16_t pwmval2){
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwmval2);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwmval1);
}