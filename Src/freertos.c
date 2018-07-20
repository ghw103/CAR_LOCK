/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "common.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <semphr.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId MQTT_TaskHandle;
osThreadId decod_TaskHandle;
osThreadId monitorTaskHandle;
osMessageQId EC20QueueHandle;
osMessageQId mqttQueueHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void U_MQTT(void const * argument);
void decoding(void const * argument);
void monitor(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
void USER_Printf(const char *pFormat, ...);
uint8_t ec20_cmd(char * cmd, char * ack, uint8_t retry, uint16_t timeout);
uint8_t openlock(void);
uint8_t closelock(void);
uint8_t stoplock(void);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of MQTT_Task */
  osThreadDef(MQTT_Task, U_MQTT, osPriorityHigh, 0, 512);
  MQTT_TaskHandle = osThreadCreate(osThread(MQTT_Task), NULL);

  /* definition and creation of decod_Task */
  osThreadDef(decod_Task, decoding, osPriorityAboveNormal, 0, 512);
  decod_TaskHandle = osThreadCreate(osThread(decod_Task), NULL);

  /* definition and creation of monitorTask */
  osThreadDef(monitorTask, monitor, osPriorityNormal, 0, 256);
  monitorTaskHandle = osThreadCreate(osThread(monitorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of EC20Queue */
/* what about the sizeof here??? cd native code */
//	osMessageQDef(EC20Queue, 16, sizeof(EC20_MSG_T));
//  EC20QueueHandle = osMessageCreate(osMessageQ(EC20Queue), NULL);

  /* definition and creation of mqttQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(mqttQueue, 16, uint16_t);
  mqttQueueHandle = osMessageCreate(osMessageQ(mqttQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
osMessageQDef(EC20Queue, 16, sizeof(EC20_MSG_T));
	EC20QueueHandle = osMessageCreate(osMessageQ(EC20Queue), NULL);
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
	//	if (HAL_UART_Transmit(&huart2, (uint8_t*)aTxBuffer, (COUNTOF(aTxBuffer) - 1), 5000) != HAL_OK)
	//	{
	//		Error_Handler();
	//	}


  /* Infinite loop */
  for(;;)
  {
	
//	  USER_Printf("hello\r\n");
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* U_MQTT function */
void U_MQTT(void const * argument)
{
  /* USER CODE BEGIN U_MQTT */
//	ec20_cmd("ATE0", "ok", 10, 300);
//	ec20_cmd("ATV1", "ok", 10, 300);
//	ec20_cmd("AT", "ok", 10, 300);
//	ec20_cmd("AT+CMEE=2", "ok", 10, 300);
//	ec20_cmd("AT+CPIN?", "+CPIN: READY", 5, 1000);
//	ec20_cmd("AT+CREG?", "+CREG: 0,1", 5, 1000);
//	ec20_cmd("AT+CGREG?", "+CGREG: 0,1", 5, 1000);
//	ec20_cmd("AT+QICSGP=1,1,\"CMNET\",\"\",\"\",0", "ok", 10, 300);
//	ec20_cmd("AT+QIACT=1", "ok", 10, 300);
//	ec20_cmd("AT+QIOPEN=1,0,\"TCP\",\"202.182.113.229\",1992,0,1", "ok", 10, 300);
  /* Infinite loop */
  for(;;)
  { 
	
//	  ec20_cmd("ATV1", "ok", 10, 300);
	 
	  
//	  USER_Printf("hello\r\n");
    osDelay(1000);
  }
  /* USER CODE END U_MQTT */
}

/* decoding function */
void decoding(void const * argument)
{
  /* USER CODE BEGIN decoding */
	char open[] = {"open"};
	char close[] = {"close"};
	char stop[] = {"stop"};
	BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100); /*  wait_time */
	EC20_MSG_T *tcp_c_msg;
	//memset(tcp_c_msg, 0, sizeof(*tcp_c_msg));
  /* Infinite loop */
  for(;  ;)
	{
		xResult = xQueueReceive(EC20QueueHandle,
			(void *)&tcp_c_msg,
			(TickType_t)xMaxBlockTime); /* time */
		if (xResult == pdPASS)
		{
			if (strncmp((char *)&tcp_c_msg->Data, (char *)&open, 4) == 0)
			{
				while(openlock())
				{
					osDelay(10);
				}
			}
			if (strncmp((char *)&tcp_c_msg->Data, (char *)&close, 5) == 0)
			{
				while (closelock())
				{
					osDelay(10);
				}
			}
			if (strncmp((char *)&tcp_c_msg->Data, (char *)&stop, 4) == 0)
			{
				stoplock();
			}
		 
			/* memcpy(tcp_c_msg->Data, rs485, sizeof(rs485));*/
			HAL_UART_Transmit(&huart2, tcp_c_msg->Data, tcp_c_msg->lengh, 0xFFFF);
			memset(&EC20_MSG, 0, sizeof(EC20_MSG));
		}
		osDelay(100);
	}
	//__set_FAULTMASK(1);
	//HAL_NVIC_SystemReset();
  /* USER CODE END decoding */
}

/* monitor function */
void monitor(void const * argument)
{
  /* USER CODE BEGIN monitor */
	uint8_t open, close;
	HAL_GPIO_WritePin(infra_red_GPIO_Port, infra_red_Pin, GPIO_PIN_SET);
  /* Infinite loop */
  for(;;)
  {
	 
	  open = HAL_GPIO_ReadPin(open_GPIO_Port, open_Pin);
	  close = HAL_GPIO_ReadPin(close_GPIO_Port, close_Pin);	
	  USER_Printf("open:%d\r\n", open);
	  USER_Printf("close:%d\r\n", close);
//	  if (open==1&close==1)
//	  {
//		  
//	  }
	  
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	   osDelay(30);
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  osDelay(500);
  }
  /* USER CODE END monitor */
}

/* USER CODE BEGIN Application */
uint8_t ec20_cmd(char * cmd,char * ack,uint8_t retry,uint16_t timeout)
{
	uint8_t	ret = 1;
	
	BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(timeout); /*  wait_time */
	EC20_MSG_T *ec20_msg;
	
	USER_Printf("%s\r\n",cmd);
	while (retry--)
	{
		xResult = xQueueReceive(EC20QueueHandle,
			(void *)&ec20_msg,
			(TickType_t)xMaxBlockTime); /* time */
		if (xResult == pdPASS)
		{
			HAL_UART_Transmit(&huart2, ec20_msg->Data, ec20_msg->lengh, 0xFFFF);
			if (strncmp((char *)&ec20_msg->Data, (char *)ack, sizeof(ack)) == 0)
			{
				ret = 0;
				memset(&EC20_MSG, 0, sizeof(EC20_MSG));
				break;
			}
			/* memcpy(tcp_c_msg->Data, rs485, sizeof(rs485));*/

			memset(&EC20_MSG, 0, sizeof(EC20_MSG));
		}
		else
		{
			USER_Printf("%s\r\n", cmd);
		}
		
	}
	
	return ret;	
	
}

void USER_Printf(const char *pFormat, ...)
{
	char buffer[128];
	
	va_list arg_ptr;
	va_start(arg_ptr, pFormat);  
	vsnprintf(buffer, 128, pFormat, arg_ptr);
	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 0xFFFF);
	va_end(arg_ptr);
//	        xQueueSend(g_OutQueue, &buffer[i], portMAX_DELAY);
}
uint8_t openlock(void)
{
	uint8_t ret;
	HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);
	if (HAL_GPIO_ReadPin(close_GPIO_Port, close_Pin) == 0 && HAL_GPIO_ReadPin(open_GPIO_Port, open_Pin) == 0)
	{
		HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);	 
		ret = 0;
	}
	else
	{
		ret = 1;	
	}
	return ret;
}
uint8_t closelock(void)
{
	uint8_t ret;
	HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_SET);
	if (HAL_GPIO_ReadPin(close_GPIO_Port, close_Pin) == 1 && HAL_GPIO_ReadPin(open_GPIO_Port, open_Pin) == 1)
	{
		HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);	 
		ret = 0;
	}
	else
	{
		ret = 1;
	}
	return ret;
}
uint8_t stoplock(void)
{
	uint8_t ret;
	HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);
	return ret;
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
