/**
 * @brief 直流有刷电机（Brushed DC Motor）控制函数，包含直接控制和PID速度和角度控制
 * @detail 关于这个库的设计思想，参见https://www.cnblogs.com/fang-d/articles/CMSIS-DSP_PID_Control.html
 * @attention 此程序适用于**正交编码器**的直流有刷电机的控制
 * @author 方笛
 */

#include "BDC.h"

BDC_Motor BDC_Motors[2];

/**
 * @brief 电机结构体初始化函数
 * @param [in] motor_id 被控电机的编号
 * @param [in] pwm_tim 产生PWM波的定时器
 * @param [in] channel1 产生PWM波的一个通道，与channel2一起，控制电机的正反转
 * @param [in] channel2 产生PWM波的一个通道，与channel1一起，控制电机的正反转
 * @param [in] p_speed 速度环PID控制的Kp
 * @param [in] i_speed 速度环PID控制的Ki
 * @param [in] d_speed 速度环PID控制的Kd
 * @param [in] p_position 位置环PID控制的Kp
 * @param [in] i_position 位置环PID控制的Ki
 * @param [in] d_position 位置环PID控制的Kd
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
  const float d_position)
{
  assert_param(pwm_tim != NULL);
  assert_param(encoder != NULL);
  
  BDC_Motor *const motor = BDC_Motors + motor_id;
  *motor = (BDC_Motor) {
    // 电机控制的相关对象
    .pwm_tim = pwm_tim,
    .encoder = encoder,
    .channel1 = channel1,
    .channel2 = channel2,

    // 闭环控制相关对象
    .last_position = 0,
    .target_speed = 0,
    .target_position = 0,
    .speed_controller = {
      .Kp = p_speed,
      .Ki = i_speed / SAMPLING_FREQUENCY,
      .Kd = d_speed * SAMPLING_FREQUENCY
    },
    .position_controller = {
      .Kp = p_position,
      .Ki = i_position / SAMPLING_FREQUENCY,
      .Kd = d_position * SAMPLING_FREQUENCY
    }
  };

  arm_pid_init_f32(&motor->speed_controller, 1);
  arm_pid_init_f32(&motor->position_controller, 1);
  __HAL_TIM_SetCounter(encoder, 0);
}
