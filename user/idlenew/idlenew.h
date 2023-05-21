#ifndef  _IDLENEW_H_

#define  _IDLENEW_H_  

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "cmsis_os.h"

extern uint8_t buffer[255];    // 串口接收到的数据缓冲区
extern uint16_t lastRemote[13];// 上一次收到的遥控数据
extern uint16_t remotedata[13];// 当前收到的遥控数据
extern osThreadId_t myTaskAssignHandle;

/**
 * @brief 使用DMA接收串口数据
 * @param *huart 串口编号，遥控的串口数据由huart4接收
 */
void Usart_Receive_Data(UART_HandleTypeDef *huart);

/**
 * @brief 处理遥控数据
 * @param *uartBuffer 接收到的串口数据存储位置
 * @param *data 处理后的数据存储位置
 */
void Data_Processing(uint8_t *uartBuffer, uint16_t *data);

#endif
