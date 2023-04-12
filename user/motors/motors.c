#include "motors.h"
#include "tim.h"

void USER_TIM_PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	//HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
}

void BTS7960_Control(uint16_t pwmVal1, uint16_t pwmVal2)
{
	//一左
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwmVal1);//L_PWM
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwmVal2);//R_PWM
	//一右
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwmVal1);//L_PWM
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwmVal2);//R_PWM
	//二左
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwmVal1);//L_PWM
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pwmVal2);//R_PWM
	//二右
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pwmVal1);//L_PWM
	//__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, pwmVal1);//L_PWM
  __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_2, pwmVal2);//R_PWM
	//三左
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwmVal1);//L_PWM
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwmVal2);//R_PWM
	//三右
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, pwmVal1);//L_PWM
  __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, pwmVal2);//R_PWM
}

void USERw_AT8236_Control(uint8_t Key, uint16_t pwmVal)
{
	switch(Key)
	{
		case 1://正转PWM,快衰减
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwmVal);//IN1
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);			//IN2
		case 2://正转PWM,慢衰减
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 1);			//IN1
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pwmVal);//IN2
		case 3://反转PWM,快衰减
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);			//IN1
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, pwmVal);//IN2
		case 4://反转PWM,慢衰减
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwmVal);//IN1
			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 1);			//IN2
	}	
}

void USERw_Servo_Control(uint16_t pwmVal)
{
	uint16_t angle;
	angle = (0.5 + angle / 270.0 * (2.5 - 0.5)) / 20.0 * 1000;
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, angle);
}