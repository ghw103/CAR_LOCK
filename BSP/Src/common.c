

#include "common.h"

#include "cmsis_os.h"


#include "usart.h"
#include "gpio.h"
#include <string.h>
//#include <memory.h>




void User_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	uint8_t clean;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	EC20_MSG_T *rx_msg;
	 rx_msg = &EC20_MSG;
	if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET))
	{
       rx_msg->Data[rx_msg->lengh++] = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
	
	}
	if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET))  //空闲中断
		{
                clean = huart->Instance->DR;
				clean=huart->Instance->SR;
//			HAL_UART_Transmit(&huart2, rx_msg->Data, rx_msg->lengh, 0xFFFF);
			
			/* 向消息队列发数据 */
		xQueueSendFromISR(EC20QueueHandle, (void *)&rx_msg, &xHigherPriorityTaskWoken);
			/* 如果 xHigherPriorityTaskWoken = pdTRUE ，那么退出中断后切到当前最高优先级任务执行 */
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  

			__HAL_UART_CLEAR_IDLEFLAG(huart);
		}
}















