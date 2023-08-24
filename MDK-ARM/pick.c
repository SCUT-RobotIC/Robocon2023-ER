/**
 * @brief 取环结构的舵机控制
 * @detail ER的取环结构舵机，使用时间作为变量进行控制
 * @author 朱逸楠
 */
 
 #include "pick.h"
 
 PICK_Motor  PICK_Motors[PICK_NUMBERS];
 
/**
 * @brief 电机结构体初始化函数
 * @param [in] motor_id 被控电机的编号
 * @param [in] pwm_tim 产生PWM波的定时器
 * @param [in] channel 产生PWM波的通道
 */
void PICK_InitMotor(
  const int motor_id,
  TIM_HandleTypeDef *const pwm_tim,
  const unsigned int channel
)
{
  assert_param(pwm_tim != NULL);

  PICK_Motor *const motor = PICK_Motors + motor_id; // 电机指针
  
  *motor = (PICK_Motor) 
  {
    // 电机控制的相关对象
    .pwm_tim = pwm_tim,
    .channel = channel,
		.angle  =  0
		
  };
	
	
	//  JIAO CHANGE HERE  INIT  & NO NEED  2023 6 18
	
	HAL_TIM_PWM_Start(motor->pwm_tim, motor->channel);
	
	
	
} 