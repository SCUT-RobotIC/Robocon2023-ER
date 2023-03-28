#include "idlenew.h"
#include "string.h"
#include "usart.h"


void Usart_Receive_Data(UART_HandleTypeDef *huart)
{
        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //�ж��Ƿ��ǿ����ж�
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);						//��������жϱ�־ 
					 Data_Processing(buffer,remotedata);
           memset(buffer,0,255);                                    //��ջ�����
           HAL_UART_Receive_DMA(huart,buffer,255);               //������ʼDMA����
        }
}

void Data_Processing(uint8_t *uartBuffer, uint16_t *data)
{
	for (int i = 0; i < 13; i++){
		data[i] =(uartBuffer[2*i]<<8) | uartBuffer[2*i+1];
	}	
}
