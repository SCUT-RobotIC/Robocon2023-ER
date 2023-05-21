#include "idlenew.h"
#include "string.h"
#include "usart.h"
#include "chassis.h"

void Usart_Receive_Data(UART_HandleTypeDef *huart)
{
	int flag = 0;
  if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) // 判断是否是空闲中断
  {
    __HAL_UART_CLEAR_IDLEFLAG(huart); 
	  HAL_UART_DMAStop(huart);
		Data_Processing(buffer,remotedata);
					
		for(int i = 0; i < 13; i++)
    {
			if(lastRemote[i] != remotedata[i])
			{
			  flag = 1;
				break;
			}
	  }

					 
	  if(flag == 1)
    {
			flag = 0;
			xTaskResumeFromISR(myTaskAssignHandle);
		}
    HAL_UART_Receive_DMA(huart, buffer, 255);  // 重启开始DMA传输
  }
}

void Data_Processing(uint8_t *uartBuffer, uint16_t *data)
{
	for(int i = 0; i < 13; i++)
  {
	  data[i] = (uartBuffer[2 * i] << 8) | uartBuffer[2 * i + 1];
	}	
}
