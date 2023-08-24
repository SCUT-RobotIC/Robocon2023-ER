/**
 * @brief 取环结构的舵机控制
 * @detail ER的取环结构舵机，使用时间作为变量进行控制
 * @author 朱逸楠
 */
 
#ifndef __PICK_H
#define __PICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "tim.h"

#define PICK_NUMBERS 5 // now is three             ///< 电机的数量



/**
 * @brief 电机结构体，包含电机的控制信息
 */
typedef struct {
  // 电机控制的相关对象
  TIM_HandleTypeDef *pwm_tim;       ///< PWM生成器
  unsigned int channel;  ///< PWM通道
	
	unsigned int angle ;
	
} PICK_Motor;


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
);
  
/**
 * @brief 电机控制的对象数组
 */
extern  PICK_Motor  PICK_Motors[PICK_NUMBERS];

  
/**
 * @brief 更新油门量，驱动电机
 * @param [in] motor_id 被控电机的编号
 */
__STATIC_FORCEINLINE void PICK_Run_Pulse(const int motor_id, const unsigned int pulse)
{
	PICK_Motor *const motor = PICK_Motors + motor_id; // 电机指针
  
	if(motor->angle != pulse){
		__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel, pulse - 1);
		motor->angle = pulse ;
	
	}
	
  
  
  //HAL_TIM_PWM_Start(motor->pwm_tim, motor->channel);
  
  
}


#ifdef __cplusplus
}
#endif

#endif // __PICK_H
