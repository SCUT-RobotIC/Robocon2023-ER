#ifndef __MOTORS_H__
#define	__MOTORS_H__

#include "stm32f4xx_hal.h"

void USER_TIM_PWM_Init(void);
void BTS7960_Control(uint16_t pwmVal1, uint16_t pwmVal2);
void USERw_AT8236_Control(uint8_t Key, uint16_t pwmVal);
void USERw_Servo_Control(uint16_t pwmVal);
#endif
