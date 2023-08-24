/**
 * @brief 取环发射结构的摩擦轮控制
 * @author 朱逸楠
 */
 
#ifndef __FRICTION_H
#define __FRICTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "tim.h"

#define FRICTION_MOTOR_NUMBERS 2             ///< 电机的数量

/**
 * @brief 电机结构体，包含电机的控制信息
 */
typedef struct {
  // 电机控制的相关对象
  TIM_HandleTypeDef *pwm_tim1;       ///< PWM生成器1
  TIM_HandleTypeDef *pwm_tim2;       ///< PWM生成器2
  unsigned int channel1, channel2;  ///< PWM通道
  int throttle;            ///< 电机运行速度
} FRICTION_Motor;


/**
 * @brief 电机控制的对象数组
 */
extern  FRICTION_Motor  FRICTION_Motors[FRICTION_MOTOR_NUMBERS];

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
);

/**
 * @brief 将输出限制在±limit范围内
 * @param [inout] output 待限制量
 * @param [in] limit 极限量
 */
#define limit_output(output, limit) \
  do {                              \
    assert_param((limit) >= 0);     \
    if ((output) > (limit)) {       \
      (output) = (limit);           \
    } else if (output < -limit) {   \
      (output) = -(limit);          \
    }                               \
  } while (0)
  
  
/**
 * @brief 设置油门
 * @param [in] motor_id 电机编号
 * @param [in] new_throttle 新的油门量，绝对值0~MAX对应占空比0到100%。
 * @attention 油门量是一个整数，其绝对值的最大值为定时器的ARR寄存器的值+1。在写这份代码时，最大油门量为8000
 */
__STATIC_FORCEINLINE void FRICTION_SetThrottle(const int motor_id, int new_throttle)
{
  FRICTION_Motor *const motor = FRICTION_Motors + motor_id; // 电机指针
//  const int output_limit = __HAL_TIM_GetAutoreload(motor->pwm_tim1) + 1; // 最大油门量
//  limit_output(new_throttle, output_limit);
  motor->throttle = new_throttle;
}


/**
 * @brief 更新油门量，驱动电机
 * @param [in] motor_id 被控电机的编号
 * @note 如果电机旋转方向不符合预期，可以直接交换if语句中的channel1和channel2
 */
__STATIC_FORCEINLINE void FRICTION_Run(const int motor_id)
{
  FRICTION_Motor *const motor = FRICTION_Motors + motor_id; // 电机指针
  const int throttle = motor->throttle;
  if (throttle < 0)
  {
    __HAL_TIM_SET_COMPARE(motor->pwm_tim1, motor->channel2, -throttle);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim1, motor->channel1, -throttle);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim2, motor->channel2, -throttle);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim2, motor->channel1, -throttle);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(motor->pwm_tim1, motor->channel2, throttle);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim1, motor->channel1, throttle);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim2, motor->channel2, throttle);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim2, motor->channel1, throttle);
  }
}

#ifdef __cplusplus
}
#endif

#endif // __FRICTION_H
