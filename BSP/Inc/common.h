#ifndef __COMMMOCN_H
#define __COMMMOCN_H

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
//#include <memory.h>
#include <string.h>
#include "usart.h"


#ifdef __cplusplus
 extern "C" {
#endif

	 
	 extern	 osMessageQId EC20QueueHandle;
	 
	 
//typedef struct Network Network;

	 



	 
//struct Network
//{
//	int my_socket;
//	int (*mqttread) (Network*, unsigned char*, int, int);
//	int (*mqttwrite) (Network*, unsigned char*, int, int);
//	void (*disconnect) (Network*);
//};
	 
#define UAST_BUFFER_SIZE  128 
typedef struct Msg 
{
  uint8_t lengh;
  uint8_t Data[UAST_BUFFER_SIZE]; 
} EC20_MSG_T;/* 定义一个结构体用于消息队列 */
	 
EC20_MSG_T  EC20_MSG ;
	 

		 

	 //extern uint8_t	 setip_flage ;
	 

	 
void User_UART_IRQHandler(UART_HandleTypeDef *huart);	 
void USER_Printf(const char *pFormat, ...);
#endif
	 
	 
	 
	 