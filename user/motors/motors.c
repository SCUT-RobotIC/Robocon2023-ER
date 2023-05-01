#include "motors.h"
#include "tim.h"

void USER_TIM_PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
}

void Shot_Control(uint16_t pwmval)
{
	//һ��
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwmval);//L_PWM
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);//R_PWM
	//һ��
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwmval);//L_PWM
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);//R_PWM
	//����
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwmval);//L_PWM
	//����
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwmval);//L_PWM
	//����
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwmval);//L_PWM
	//����
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwmval);//R_PWM
}


void Servo_Control1(uint16_t angle)
{
	angle = (0.5 + angle / 180.0 * (2.5 - 0.5)) / 20.0 * 99;
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, angle);
}

void Servo_Control2 (uint16_t angle)
{
	angle = (0.5 + angle / 180.0 * (2.5 - 0.5)) / 20.0 * 99;
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, angle);
}
