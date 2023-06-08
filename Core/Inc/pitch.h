/**
 * @brief 俯仰机构的PID角度控制库
 * @detail 关于这个库的设计思想，参见https://www.cnblogs.com/fang-d/articles/CMSIS-DSP_PID_Control.html
 * @attention 此程序适用于位置式编码器的控制系统
 * @author 朱逸楠&方笛
 */

#ifndef __PITCH_H
#define __PITCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "dsp/controller_functions.h"
#include "tim.h"
#include "usart.h"  // 引入串口头，用于与上位机通信

#define PITCH_SPEED_FACTOR 200.0	    ///< 电机减速比*编码器对数
#define SAMPLING_FREQUENCY 200.0    ///< 电机编码器的采样频率，和PID的计算频率
#define MOTOR_NUMBERS 1             ///< 电机的数量
#define ANGLE_RST 310.0             ///< 编码器角度校正零点

/**
 * @brief 电机结构体，包含电机的控制信息，速度环和位置环的PID控制信息
 */
typedef struct {
  // 电机控制的相关对象
  TIM_HandleTypeDef *pwm_tim;       ///< PWM生成器
  TIM_HandleTypeDef *encoder;       ///< 编码器定时器
  unsigned int channel1, channel2;  ///< PWM通道

  // 闭环控制相关对象
  int last_position;    /// 上一次采样时，电机的编码器位置（用于计算电机转速）
    int target_position;     ///< 目标位置，单位为角度
  arm_pid_instance_f32 position_controller;    ///< PID位置环的控制器

} Pitch_Motor;

/**
 * @brief 电机控制的对象数组
 */
extern  Pitch_Motor Pitch_Motors[1];

extern float duty;

/**
 * @brief 编码器获取当前角度位置函数
 */
float Pitch_Get_Position(const int motor_id);

/**
 * @brief 电机结构体初始化函数
 * @param [in] motor_id 被控电机的编号
 * @param [in] pwm_tim 产生PWM波的定时器
 * @param [in] encoder 编码器定时器
 * @param [in] channel1 产生PWM波的一个通道，与channel2一起，控制电机的正反转
 * @param [in] channel2 产生PWM波的一个通道，与channel1一起，控制电机的正反转
 * @param [in] p_position 角度环PID控制的Kp
 * @param [in] i_position 角度环PID控制的Ki
 * @param [in] d_position 角度环PID控制的Kd

 */
void Pitch_InitMotor(
  const int motor_id,
  TIM_HandleTypeDef *const pwm_tim,
  TIM_HandleTypeDef *const encoder,
  const unsigned int channel1,
  const unsigned int channel2,
  const float p_position,
  const float i_position,
  const float d_position
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
 * @brief 更新油门量，驱动电机
 * @param [in] motor_id 被控电机的编号
 * @note 如果电机旋转方向不符合预期，可以直接交换if语句中的channel1和channel2
 */
__STATIC_FORCEINLINE void PITCH_Run(const int motor_id)
{
	Pitch_Motor *const motor = Pitch_Motors + motor_id; // 电机指针
  const int throttle = motor->position_controller.state[2];
  if (throttle < 0)
  {
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel1, -throttle);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel2, 0);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel1, 0);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->channel2, throttle);
  }
}

/**
 * @brief 设置油门
 * @param [in] motor_id 电机编号
 * @param [in] new_throttle 新的油门量，绝对值0~MAX对应占空比0到100%。
 * @attention 油门量是一个整数，其绝对值的最大值为定时器的ARR寄存器的值+1。在写这份代码时，最大油门量为8000
 */
__STATIC_FORCEINLINE void PITCH_SetThrottle(const int motor_id, int new_throttle)
{
	Pitch_Motor *const motor = Pitch_Motors + motor_id; // 电机指针
  const int output_limit = __HAL_TIM_GetAutoreload(motor->pwm_tim) + 1; // 最大油门量
  limit_output(new_throttle, output_limit);
	motor->position_controller.state[2] = new_throttle;
}

/**
 * @brief 设置位置（PWM占空比）
 * @param [in] motor_id 电机编号
 * @param [in] new_position 新的速度量
 */
__STATIC_FORCEINLINE void PITCH_SetPosition(const int motor_id, const float new_position)
{
  Pitch_Motors[motor_id].target_position = new_position;
}



/**
 * @brief PID闭环控制位置
 * @param [in] motor_id 电机编号
 */
__STATIC_FORCEINLINE void PITCH_ControlPosition(const int motor_id)
{
  Pitch_Motor *const motor = Pitch_Motors + motor_id; // 电机指针
  const int output_limit = __HAL_TIM_GetAutoreload(motor->pwm_tim) + 1; // 最大油门量


  const int current_position = (int)Pitch_Get_Position(motor_id); // 当前编码器位置
  const float speed_now = \
    (float)(current_position - motor->last_position) * SAMPLING_FREQUENCY / PITCH_SPEED_FACTOR;  // 当前速度
  motor->last_position = current_position;  // 更新电机状态

  // 利用速度误差进行PID控制
  arm_pid_f32(&motor->position_controller, motor->target_position - speed_now);
  limit_output(motor->position_controller.state[2], output_limit);
  PITCH_Run(motor_id);
}



/**
 * @brief 修改位置环PID控制的Kp
 * @param [in] motor_id 电机编号
 * @param [in] p_position 新的位置环PID控制的Kp
 * @note 此函数适合PID调参时使用
 */
__STATIC_FORCEINLINE void PITCH_SetPositionP(const int motor_id, const float p_position)
{
  Pitch_Motor *const motor = Pitch_Motors + motor_id;
  motor->position_controller.Kp = p_position;
  arm_pid_init_f32(&motor->position_controller, 0);
}

/**
 * @brief 修改位置环PID控制的Ki
 * @param [in] motor_id 电机编号
 * @param [in] i_position 新的位置环PID控制的Kp
 * @note 此函数适合PID调参时使用
 */
__STATIC_FORCEINLINE void PITCH_SetPositionI(const int motor_id, const float i_position)
{
  Pitch_Motor *const motor = Pitch_Motors + motor_id;
  motor->position_controller.Ki = i_position / SAMPLING_FREQUENCY;
  arm_pid_init_f32(&motor->position_controller, 0);
}

/**
 * @brief 修改位置环PID控制的Kd
 * @param [in] motor_id 电机编号
 * @param [in] d_position 新的位置环PID控制的Kp
 * @note 此函数适合PID调参时使用
 */
__STATIC_FORCEINLINE void PITCH_SetPositionD(const int motor_id, const float d_position)
{
  Pitch_Motor *const motor = Pitch_Motors + motor_id;
  motor->position_controller.Kd = d_position * SAMPLING_FREQUENCY;
  arm_pid_init_f32(&motor->position_controller, 0);
}




#ifdef __cplusplus
}
#endif

#endif // __PITCH_H
