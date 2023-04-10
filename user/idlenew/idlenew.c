#include "idlenew.h"
#include "string.h"
#include "usart.h"
#include "chassis.h"

//osThreadId_t myTaskAssignHandle;
void Usart_Receive_Data(UART_HandleTypeDef *huart)
{
        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);						//清除空闲中断标志 
					 HAL_UART_DMAStop(huart);
					 Data_Processing(buffer,remotedata);
					 xTaskResumeFromISR(myTaskAssignHandle);
           //memset(buffer,0,255);                                    //清空缓冲区
           HAL_UART_Receive_DMA(huart,buffer,255);               //重启开始DMA传输
        }
}

void Data_Processing(uint8_t *uartBuffer, uint16_t *data)
{
	for (int i = 0; i < 13; i++){
		data[i] =(uartBuffer[2*i]<<8) | uartBuffer[2*i+1];
	}	
}
