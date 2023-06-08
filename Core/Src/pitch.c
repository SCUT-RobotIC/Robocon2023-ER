/**
 * @brief 俯仰机构的PID角度控制库
 * @detail 关于这个库的设计思想，参见https://www.cnblogs.com/fang-d/articles/CMSIS-DSP_PID_Control.html
 * @attention 此程序适用于位置式编码器的控制系统
 * @author 朱逸楠&方笛
 */
 
#include "pitch.h"
 
Pitch_Motor Pitch_Motors[MOTOR_NUMBERS];
float duty;

/**
 * @brief 编码器获取当前角度位置函数
 * @param [in] motor_id 被控电机的编号
 */
float Pitch_Get_Position(const int motor_id)
{
  Pitch_Motor *const motor = Pitch_Motors + motor_id; // 电机指针
  const int postiveValue = HAL_TIM_ReadCapturedValue(motor->encoder, TIM_CHANNEL_2) + 1;
  const int period = HAL_TIM_ReadCapturedValue(motor->encoder, TIM_CHANNEL_1) + 1;
  float angle;
      
  duty = (postiveValue * 100.0 ) / period;
  angle = duty / 100 * 360;
    
  if(angle > ANGLE_RST)
  {
    angle = angle - ANGLE_RST;
  }
  else
  {
    angle = angle + 360 - ANGLE_RST;
  }
  
  return angle;
}

/**
 * @brief 电机结构体初始化函数
 * @param [in] motor_id 被控电机的编号
 * @param [in] pwm_tim 产生PWM波的定时器
 * @param [in] encoder 电机对应的编码器
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
)
{
  assert_param(pwm_tim != NULL);
  assert_param(encoder != NULL);
  
  Pitch_Motor *const motor = Pitch_Motors + motor_id;
  *motor = (Pitch_Motor) {
    // 电机控制的相关对象
    .pwm_tim = pwm_tim,
    .encoder = encoder,
    .channel1 = channel1,
    .channel2 = channel2,

    // 闭环控制相关对象
    .last_position = 0,
    .target_position = 0,

    .position_controller = {
      .Kp = p_position,
      .Ki = i_position / SAMPLING_FREQUENCY,
      .Kd = d_position * SAMPLING_FREQUENCY
    },
  };

  arm_pid_init_f32(&motor->position_controller, 1);

}