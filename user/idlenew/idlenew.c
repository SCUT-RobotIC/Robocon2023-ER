#include "idlenew.h"
#include "string.h"
#include "usart.h"


void Usart_Receive_Data(UART_HandleTypeDef *huart)
{
        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);						//清除空闲中断标志 
					 vx=buffer[0];
           vy=buffer[1];
           wz=buffer[2];
           memset(buffer,0,255);                                    //清空缓冲区
           HAL_UART_Receive_DMA(huart,buffer,255);               //重启开始DMA传输
        }
}
