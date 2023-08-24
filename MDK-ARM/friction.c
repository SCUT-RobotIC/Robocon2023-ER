/**
 * @brief 取环发射结构的摩擦轮控制
 * @author 朱逸楠
 */
 
 #include "friction.h"
 
 FRICTION_Motor  FRICTION_Motors[FRICTION_MOTOR_NUMBERS];

 
/**
 * @brief 电机结构体初始化函数
 * @param [in] motor_id 被控电机的编号
 * @param [in] pwm_tim 产生PWM波的定时器
 * @param [in] channel 产生PWM波的通道
 */
void FRICTION_InitMotor(
  const int motor_id,
  TIM_HandleTypeDef *const pwm_tim1,
  TIM_HandleTypeDef *const pwm_tim2,
  const unsigned int channel1, 
  const unsigned int channel2
          
)
{
  //assert_param(pwm_tim1 != NULL);
  //assert_param(pwm_tim2 != NULL);

  FRICTION_Motor *const motor = FRICTION_Motors + motor_id; // 电机指针
  
  *motor = (FRICTION_Motor) 
  {
    // 电机控制的相关对象
    .pwm_tim1 = pwm_tim1,
    .pwm_tim2 = pwm_tim2,
    .channel1 = channel1,
    .channel2 = channel2,
    .throttle = 0
  };
}

