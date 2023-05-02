/**
 * @brief 直流有刷电机（Brushed DC Motor）控制函数，包含直接控制和PID控制
 * @author 方笛
 */
#ifndef __BDC_H
#define __BDC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "dsp/controller_functions.h"
#include "usart.h"

#define BDC_SPEED_FACTOR 319.6	///< 电机减速比*编码器对数
//#define BDC_SPEED_FACTOR (30.0 * 17)
#define SAMPLING_FREQUENCY 200.0

/**
 * @brief 电机结构体，包含电机的控制信息，速度环和位置环的PID控制信息
 */
typedef struct {
  // 电机控制的相关对象
  TIM_HandleTypeDef *pwm_tim;       ///< PWM生成器
  TIM_HandleTypeDef *encoder;       ///< 电机编码器
  unsigned int channel1, channel2;  ///< PWM通道
  int throttle;             ///< 电机当前的油门量

  // 闭环控制相关对象
  int last_position;    /// 上一次采样时，电机的编码器位置（用于计算电机转速）
  int target_speed;     ///< 目标速度，单位为rps (revolutions per second)
  int target_position;  ///< 目标位置，此项为编码器位置
  arm_pid_instance_f32 speed_controller;    ///< PID速度环的控制器
  arm_pid_instance_f32 position_controller; ///< PID位置环的控制器
} BDC_Motor;

/**
 * @brief 电机控制的对象数组
 */
extern BDC_Motor BDC_Motors[2];

/**
 * @brief 电机结构体初始化函数
 */
void BDC_InitMotor(
  const int motor_id,
  TIM_HandleTypeDef *const pwm_tim,
  TIM_HandleTypeDef *const encoder,
  const unsigned int channel1,
  const unsigned int channel2,
  const float p_speed,
  const float i_speed,
  const float d_speed,
  const float p_position,
  const float i_position,
  const float d_position
);

/**
 * @brief 将输出限制在指定范围内
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
 * @brief 更新油门量，驱动电机
 */
__STATIC_FORCEINLINE void BDC_Run(const int motor_id)
{
	BDC_Motor *const motor = BDC_Motors + motor_id; // 电机指针
//	// 计算当前速度，单位为rps (revolutions per second)
//  const int current_position = (int)__HAL_TIM_GetCounter(motor->encoder); // 当前编码器位置
//  const float speed_now = (current_position - motor->last_position) * (SAMPLING_FREQUENCY / BDC_SPEED_FACTOR);  // 当前速度
//  motor->last_position = current_position;  // 更新电机状态
//	report3(
//    &huart1,
//    100 * (float)motor->throttle / (__HAL_TIM_GetAutoreload(motor->pwm_tim) + 1),
//    speed_now,
//    current_position
//  );
  const int throttle = motor->throttle;
  if (throttle < 0) {
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel1, -throttle);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel2, 0);
  } else {
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel1, 0);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel2, throttle);
  }
}

/**
 * @brief 设置油门
 * @param motor_id 电机编号
 * @param new_throttle 新的油门量
 */
__STATIC_FORCEINLINE void BDC_SetThrottle(const int motor_id, const float new_throttle)
{
	BDC_Motor *const motor = BDC_Motors + motor_id; // 电机指针
  const int output_limit = __HAL_TIM_GetAutoreload(motor->pwm_tim) + 1; // 最大油门量
	motor->throttle = new_throttle;
}

/**
 * @brief 设置速度
 * @param motor_id 电机编号
 * @param new_throttle 新的速度量
 */
__STATIC_FORCEINLINE void BDC_SetSpeed(const int motor_id, const float new_speed)
{
  BDC_Motors[motor_id].target_speed = new_speed;
}

/**
 * @brief 设置位置
 * @param motor_id 电机编号
 * @param new_throttle 新的位置量
 */
__STATIC_FORCEINLINE void BDC_SetPosition(const int motor_id, const float new_position)
{
  BDC_Motors[motor_id].target_position = new_position * BDC_SPEED_FACTOR;
}

/**
 * @brief PID闭环控制位置
 * @param motor_id 电机编号
 */
void BDC_ControlSpeed(const int motor_id);

/**
 * @brief PID闭环控制速度
 * @param motor_id 电机编号
 */
void BDC_ControlPosition(const int motor_id);

#ifdef __cplusplus
}
#endif

#endif // __BDC_H
