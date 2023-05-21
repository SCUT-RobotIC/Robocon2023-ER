#ifndef __MOTORS_H__
#define	__MOTORS_H__

#include "stm32f4xx_hal.h"

/**
 * @brief 定时器初始化
 */
void USER_TIM_PWM_Init(void);

/**
 * @brief 发射结构控制
 * @param pwmval 输出的pwm值，范围【0，99】
 */ 
void Shot_Control(uint16_t pwmval);

/**
 * @brief 舵机上控制，TIM12 CH1
 * @param angle 输出的角度值，范围【0，180】
 */
void Servo_Control1(uint16_t angle);

/**
 * @brief 舵机下控制，TIM12 CH2
 * @param angle 输出的角度值，范围【0，270】
 */
void Servo_Control2(uint16_t angle);

/**
 * @brief 升降结构左侧运动控制，TIM12 CH1
 * @param pwmval1,pwmval2
 */
void Rise_Control_Left(uint16_t pwmval1, uint16_t pwmval2);

/**
 * @brief 升降结构右侧运动控制，TIM12 CH2
 * @param pwmval1,pwmval2
 */
void Rise_Control_Right(uint16_t pwmval1, uint16_t pwmval2);
#endif
