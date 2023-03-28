#include "idlenew.h"
#include "string.h"
#include "usart.h"


void Usart_Receive_Data(UART_HandleTypeDef *huart)
{
        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //�ж��Ƿ��ǿ����ж�
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);						//��������жϱ�־�������һֱ���Ͻ����жϣ� 
					 vx=buffer[0];
           vy=buffer[1];
           wz=buffer[2];
					 //HAL_UART_Transmit(&huart1,sendbuffer,3,10000);
           memset(buffer,0,255);                                    //��ջ�����
           HAL_UART_Receive_DMA(huart,buffer,255);               //������ʼDMA����
        }
}
