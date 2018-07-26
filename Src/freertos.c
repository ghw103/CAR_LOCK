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

#include "lock.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId MQTT_TaskHandle;
osThreadId decod_TaskHandle;
osThreadId monitorTaskHandle;
osThreadId controlTaskHandle;
osMessageQId EC20QueueHandle;
osMessageQId mqttQueueHandle;
osMessageQId controlQueueHandle;
osMessageQId statusQueueHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void U_MQTT(void const * argument);
void decoding(void const * argument);
void monitor(void const * argument);
void control(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
//uint8_t openlock(void);
//uint8_t closelock(void);
//uint8_t stoplock(void);
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

  /* definition and creation of controlTask */
  osThreadDef(controlTask, control, osPriorityAboveNormal, 0, 256);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of EC20Queue */
/* what about the sizeof here??? cd native code */
//  osMessageQDef(EC20Queue, 16, uint16_t);
//  EC20QueueHandle = osMessageCreate(osMessageQ(EC20Queue), NULL);

  /* definition and creation of mqttQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(mqttQueue, 16, uint16_t);
  mqttQueueHandle = osMessageCreate(osMessageQ(mqttQueue), NULL);

  /* definition and creation of controlQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(controlQueue, 2, uint16_t);
  controlQueueHandle = osMessageCreate(osMessageQ(controlQueue), NULL);

  /* definition and creation of statusQueue */
/* what about the sizeof here??? cd native code */
  osMessageQDef(statusQueue, 2, uint16_t);
  statusQueueHandle = osMessageCreate(osMessageQ(statusQueue), NULL);

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
HAL_GPIO_WritePin(infra_red_GPIO_Port, infra_red_Pin, GPIO_PIN_SET);

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
	ec20_send("AT", "OK", 10, 1000);
	ec20_send("ATE1", "OK", 10, 1000);
	ec20_send("ATV1", "OK", 10, 300);
	ec20_send("AT+CMEE=2", "OK", 10, 300);
	ec20_send("AT+CPIN?", "+CPIN: READY", 20, 1000);
	ec20_send("AT+CREG?", "+CREG: 0,1", 60, 1000);
	ec20_send("AT+CGREG?", "+CGREG: 0,1", 60, 1000);
	ec20_send("AT+QICSGP=1,1,\"CTNET\",\"\",\"\",0", "OK", 10, 1000);
	ec20_send("AT+QIACT=1", "OK", 10, 1000);
//	ec20_send("AT+QIOPEN=1,0,\"TCP\",\"115.29.240.46\",9000,0,1", "ok", 10, 300);
/*----------------------------------------------------------------------------------*/
	char open[] = { "open" };
	char close[] = { "close" };
	char stop[] = { "stop" };
	uint8_t mqttcontrol;
	BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100); /*  wait_time */
	EC20_MSG_T *tcp_c_msg;
  /* Infinite loop */
  for(;;)
  { 
	  xResult = xQueueReceive(EC20QueueHandle,
		  (void *)&tcp_c_msg,
		  (TickType_t)xMaxBlockTime); /* time */
	  if (xResult == pdPASS)
	  {
		  if (strncmp((char *)&tcp_c_msg->Data, (char *)&open, 4) == 0)
		  {
			  mqttcontrol = openlock;
			  if (xQueueSend(mqttQueueHandle, (void *) &mqttcontrol, (TickType_t)10) != pdPASS)
			  {
				  /* 发�?�失败，即使等待�?10个时钟节�? */
				  //mqttcontrol = normal;
			  }
			  else
			  {
				  /* 发�?�成�? */                  
			  }
				
		  }
		  if (strncmp((char *)&tcp_c_msg->Data, (char *)&close, 5) == 0)
		  {
			  mqttcontrol = closelock;
			  if (xQueueSend(mqttQueueHandle, (void *) &mqttcontrol, (TickType_t)10) != pdPASS)
			  {
				  /* 发�?�失败，即使等待�?10个时钟节�? */
				  //mqttcontrol = normal;
			  }
			  else
			  {
				  /* 发�?�成�? */                  
			  }
			
		  }
		  if (strncmp((char *)&tcp_c_msg->Data, (char *)&stop, 4) == 0)
		  {
		  }
		  /* memcpy(tcp_c_msg->Data, rs485, sizeof(rs485));*/
		  HAL_UART_Transmit(&huart2, tcp_c_msg->Data, tcp_c_msg->lengh, 0xFFFF);
		  memset(&EC20_MSG, 0, sizeof(EC20_MSG));
	  }
//	  ec20_cmd("ATV1", "ok", 10, 300);
	 
	  
//	  USER_Printf("hello\r\n");
    osDelay(50);
  }
  /* USER CODE END U_MQTT */
}

/* decoding function */
void decoding(void const * argument)
{
  /* USER CODE BEGIN decoding */
	BaseType_t xResult;
	const TickType_t xMaxBlockTime= pdMS_TO_TICKS(50); /* 设置�?大等待时间为ms */
	uint8_t status = stoplock, control, mqttcontrol;
	uint8_t controlflag = 0, sendflag = 1;
	//memset(tcp_c_msg, 0, sizeof(*tcp_c_msg));
  /* Infinite loop */
  for(;  ;)
	{
		xResult = xQueueReceive(mqttQueueHandle, (void *)&mqttcontrol, (TickType_t)xMaxBlockTime);
		if (xResult == pdPASS)
		{
			vTaskSuspend(monitorTaskHandle);
			control = mqttcontrol;
			controlflag = 1;
			sendflag = 0;
		}
		else
		{
			/* 超时 */
		
		}
		if (controlflag == 1)
		{	if (sendflag == 0)
			{
				if (xQueueSend(controlQueueHandle, (void *) &control, (TickType_t)10) != pdPASS)
				{
					/* 发�?�失败，即使等待�?10个时钟节�? */
				}
				else
				{
					sendflag = 1;
					/* 发�?�成�? */                  
				} 
			}
			xResult = xQueueReceive(statusQueueHandle, (void *)&status, (TickType_t)xMaxBlockTime);
			if (xResult == pdPASS)
			{
				if (status == Blockage)
				{ 
////					control = stoplock;
////					sendflag = 0;
//					
//					if (control == closelock)
//					{
//						control = openlock;
//						sendflag = 0;
//					}
//					else if (control == openlock)
//					{
//						control = closelock;
//						sendflag = 0;
//					} 
				}
				else if (status == lock_open || status == lock_close || status == lock_stop)
				{
					vTaskResume(monitorTaskHandle);
					controlflag = 0;
					sendflag = 1;
//					control = stoplock;
				}
			}

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
	BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100); /* 设置�?大等待时间为ms */

	uint8_t controlflag = 0, sendflag = 1;
	uint8_t status = lock_stop, oldstatus = lock_open, control, mqttcontrol;
	uint16_t vbat;
	uint16_t Current;
	status = read_lockstatus();
	
	char buf[50] = { 0 };
	
	uint8_t len;
	
	  /* Infinite loop */
	for (;  ;)
	{
		if (controlflag == 0)
		{
			lcok_readadc(NULL, &vbat);
		//	USER_Printf("vbat:%d  \r\n", vbat);
			if (status == lock_open || status == lock_close)
			{
				oldstatus = status;
			}
			else
			{
				oldstatus = lock_open;
			}
			status = read_lockstatus();
			if (status == lock_illegalopen)
			{
				control = resetlock;
				controlflag = 1;
				sendflag = 0;
				HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
			}
			else  if (status == lock_illegalclose)
			{
				HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
				if (oldstatus == lock_open)
				{
					control = openlock;
					controlflag = 1;
					sendflag = 0;
				}
				else if (oldstatus == lock_close)
				{
					control = closelock;
					controlflag = 1;	
					sendflag = 0;
				}
				else
				{
					control = openlock;
					controlflag = 1;	
					sendflag = 0;
				}	
			}
			else control = stoplock; 
		}
		if (controlflag == 1)
		{
			if (sendflag == 0)
			{
				if (xQueueSend(controlQueueHandle, (void *) &control, (TickType_t)10) != pdPASS)
				{
					/* 发�?�失败，即使等待�?10个时钟节�? */
				}
				else
				{
					sendflag = 1;
					/* 发�?�成�? */                  
				} 
			}
			xResult = xQueueReceive(statusQueueHandle, (void *)&status, (TickType_t)xMaxBlockTime);
			if (xResult == pdPASS)
			{
				//				if (status == Blockage)
				//				{ 
				////					controlflag = 0;
				//					control = stoplock;
				//					sendflag = 0;
				//				}
							 if(status == lock_open || status == lock_close || status == lock_stop)
				{
					controlflag = 0;
					sendflag = 1;
					HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
				}
			}
		}
		
		read_distance();
//		USER_Printf("close:%d  ", HAL_GPIO_ReadPin(close_GPIO_Port, close_Pin));
//		USER_Printf("open:%d  ", HAL_GPIO_ReadPin(open_GPIO_Port, open_Pin));
//		USER_Printf("control:%d  ", control);
//		USER_Printf("oldstatus:%d ", oldstatus);
//		
//		USER_Printf("status:%d\r\n", status);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		osDelay(30);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		osDelay(100);
	}
  /* USER CODE END monitor */
}

/* control function */
void control(void const * argument)
{
  /* USER CODE BEGIN control */
	BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(50); /* 设置�?大等待时间为ms */
	uint16_t Current;
	uint8_t lock_status, lockcontrol, controlflag, M_control;
	uint8_t retry=0;
	uint32_t tickstart = 0U;
	
	M_control = normal;
	lock_status = lock_stop;
	controlflag = 0;
  /* Infinite loop */
  for(;;)
  {
	  xResult = xQueueReceive(controlQueueHandle, (void *)&lockcontrol,(TickType_t)xMaxBlockTime);     
	  if (xResult == pdPASS)
	  {
		  M_control = lockcontrol;
			retry = 0;
	  }
	  else
	  {
		  /* 超时 */

	  }
	  if (M_control != normal)
	  {
		  lock_status = lock_control(M_control);
		  lcok_readadc(&Current, NULL);
		  if (Current > 1500)
		  {
			  retry++;
			  if(retry == 20)
			  {
				  retry = 0;
				  lcok_readadc(&Current, NULL);
				  if (Current > 1500)
				  { 
					  lock_status = Blockage; 
					  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);		
					  lock_control(stoplock);
					  osDelay(500);
					  if (M_control == openlock)
					  {
						  M_control = closelock;
					  }
					  else
					  {
						  M_control = openlock; 
					  }
					  lock_control(M_control);	
				  }
			  }
		  }
		  if (xQueueSend(statusQueueHandle, (void *)&lock_status, (TickType_t)10) == pdPASS)
		  {
			 
		  }
		  else
		  {
			  //			  xQueueSend(statusQueueHandle, (void *)&lock_status, (TickType_t)10);
			  			  /* error */                  
		  } 
		  if (lock_status == lock_open || lock_status == lock_close || lock_status == lock_stop)
		  {
			  M_control = normal;
			  HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
		  }

	  }
    osDelay(100);
  }
  /* USER CODE END control */
}

/* USER CODE BEGIN Application */
uint8_t ec20_send(char * cmd,char * ack,uint8_t retry,uint16_t timeout)
{
	uint8_t	ret = 1;
	
	BaseType_t xResult;
	uint8_t recvtimes;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(timeout); /*  wait_time */
	EC20_MSG_T *ec20_msg;
	if (cmd!=NULL)
	{
		USER_Printf("%s\r\n", cmd);	 
	}
	while (retry--)
	{
		xResult = xQueueReceive(EC20QueueHandle,
			(void *)&ec20_msg,
			(TickType_t)xMaxBlockTime); /* time */
		if (xResult == pdPASS)
		{
			if (recvtimes==0)
			{
				if (strncmp((char *)cmd, (char *)cmd, sizeof(cmd)) == 0)
				{
					memset(&EC20_MSG, 0, sizeof(EC20_MSG));
					recvtimes = 1;
				}
			}
			else if (recvtimes ==1)
			{
				recvtimes = 0;
				HAL_UART_Transmit(&huart2, ec20_msg->Data, ec20_msg->lengh, 0xFFFF);
				if (strstr((char *)&ec20_msg->Data, (char *)ack))
				{
					ret = 0;
					memset(&EC20_MSG, 0, sizeof(EC20_MSG));
					break;
				}
			}
//			if (strncmp((char *)&ec20_msg->Data, (char *)ack, sizeof(ack)) == 0)
//			{
//				ret = 0;
//				memset(&EC20_MSG, 0, sizeof(EC20_MSG));
//				break;
//			}
			/* memcpy(tcp_c_msg->Data, rs485, sizeof(rs485));*/ 

			memset(&EC20_MSG, 0, sizeof(EC20_MSG));
		}
		else
		{
			if (cmd != NULL)
			{
				if (recvtimes == 0)
				{
					USER_Printf("%s\r\n", cmd);	 
				}
		
			}
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
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
