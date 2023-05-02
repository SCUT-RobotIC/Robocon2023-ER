/**
 * @brief 直流有刷电机（Brushed DC Motor）控制函数，包含直接控制和PID控制
 * @author 方笛
 */

#include "BDC.h"

BDC_Motor BDC_Motors[2];


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
    .throttle = 0,

    // 闭环控制相关对象
    .last_position = 0,
    .target_speed = 0,
    .target_position = 0,
    .speed_controller = {
      .Kp = p_speed,
      .Ki = i_speed,
      .Kd = d_speed
    },
    .position_controller = {
      .Kp = p_position,
      .Ki = i_position,
      .Kd = d_position
    }
  };

  arm_pid_init_f32(&motor->speed_controller, 1);
  arm_pid_init_f32(&motor->position_controller, 1);
  __HAL_TIM_SetCounter(encoder, 0);
}

void BDC_ControlSpeed(const int motor_id)
{
  BDC_Motor *const motor = BDC_Motors + motor_id; // 电机指针
  const int output_limit = __HAL_TIM_GetAutoreload(motor->pwm_tim) + 1; // 最大油门量

  // 计算当前速度，单位为rps (revolutions per second)
  const int current_position = (int)__HAL_TIM_GetCounter(motor->encoder); // 当前编码器位置
  const float speed_now = (current_position - motor->last_position) * SAMPLING_FREQUENCY / BDC_SPEED_FACTOR;  // 当前速度
  motor->last_position = current_position;  // 更新电机状态

  // 利用速度误差进行PID控制
  motor->throttle += arm_pid_f32(&motor->speed_controller, motor->target_speed - speed_now);
  limit_output(motor->throttle, output_limit);
  BDC_Run(motor_id);

//  report_floats(
//    &huart1,
//    100 * motor->throttle / (float)(output_limit),
//    motor->target_speed,
//    speed_now,
//    motor->speed_controller.state[0],
//    motor->speed_controller.Kp,
//    motor->speed_controller.Ki,
//    motor->speed_controller.Kd
//  );
}

void BDC_ControlPosition(const int motor_id)
{
  BDC_Motor *const motor = BDC_Motors + motor_id;
  const int current_position = (int)__HAL_TIM_GetCounter(motor->encoder);

  const float error = (motor->target_position - current_position) / (float)(BDC_SPEED_FACTOR);
  motor->target_speed += arm_pid_f32(&motor->position_controller, error) / SAMPLING_FREQUENCY;
  limit_output(motor->target_speed, 8);

//  report_floats(
//    &huart1,
//    100 * motor->throttle / (float)(motor->pwm_tim->Instance->ARR + 1),
//    motor->target_position / BDC_SPEED_FACTOR,
//    current_position / BDC_SPEED_FACTOR,
//    error,
//    motor->position_controller.Kp,
//    motor->position_controller.Ki,
//    motor->target_speed
//  );
	BDC_ControlSpeed(motor_id);
}
