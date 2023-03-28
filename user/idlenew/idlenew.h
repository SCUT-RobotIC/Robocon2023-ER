#ifndef  _IDLENEW_H_

#define  _IDLENEW_H_  //跟在后面的_LED_H_只是一个文件名

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"

extern uint8_t buffer[255];
extern fp32 vx;
extern fp32 vy;
extern fp32 wz;
void Usart_Receive_Data(UART_HandleTypeDef *huart);

#endif
