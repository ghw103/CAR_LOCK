

#include "lock.h"
#include "adc.h"
#include  "math.h"
#include "common.h"
#include "tim.h"


uint32_t               uwIC2Value1 = 0;
uint32_t               uwIC2Value2 = 0;
uint32_t               uwDiffCapture = 0;

uint16_t               uhCaptureIndex = 0;

uint8_t lock_control(uint8_t type)
{
	int8_t  lock_status = controling;
	switch (type)
	{
	case openlock:
		HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);
		if (HAL_GPIO_ReadPin(close_GPIO_Port, close_Pin) == 0 && HAL_GPIO_ReadPin(open_GPIO_Port, open_Pin) == 0)
		{
			HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);	 
			lock_status = lock_open;
		}
		
		break;
	case closelock:
		HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_SET);
		if (HAL_GPIO_ReadPin(close_GPIO_Port, close_Pin) == 1 && HAL_GPIO_ReadPin(open_GPIO_Port, open_Pin) == 1)
		{
			HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);	 
			lock_status = lock_close;
		}
		break;
	case resetlock:
		HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_SET);
		if (HAL_GPIO_ReadPin(close_GPIO_Port, close_Pin) == 0 && HAL_GPIO_ReadPin(open_GPIO_Port, open_Pin) == 0)
		{
			HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);	 
			lock_status = lock_open;
		}
			break;
		
	case stoplock:
		HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);
		lock_status = lock_stop;

		break;
	default:
		break;
	}
return lock_status;	
}

void lcok_readadc(uint16_t * imax, uint16_t* vbat)
{
//	hdma_adc1.Instance = DMA1_Channel1;
	uint32_t  ADC_Value[2];
	uint32_t  imax_Value, vbat_Value;
	uint8_t i;
//	
	for (i = 0; i < 2; i++)

	{

		HAL_ADC_Start(&hadc1);

		HAL_ADC_PollForConversion(&hadc1, 0xffff);

		ADC_Value[i] = HAL_ADC_GetValue(&hadc1);

//		USER_Printf("------ch:%d--%d-------\r\n", i, ADC_Value[i]);

	}
	
	*imax =  (uint16_t)ADC_Value[0] * 3.3f / 4096 / 0.2 * 1000;
	*vbat = (uint16_t)ADC_Value[1] * 19.8f / 4096 * 100;
//	HAL_ADC_Stop(&hadc1);
//	osDelay(100);
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, 2);
//	USER_Printf(" value = %d  ", HAL_DMA_PollForTransfer(&hdma_adc1, HAL_DMA_FULL_TRANSFER, 200));   
//	osDelay(100);
//	//HAL_DMA_PollForTransfer(&hdma_adc1, HAL_DMA_HALF_TRANSFER, 200);
//	if (imax == NULL)
//	{
//		for (i = 0, imax_Value = 0, vbat_Value = 0; i < 6;)
//		{
//			i++;
//			vbat_Value += ADC_Value[i++];
//		}
//		vbat_Value /= 3;
//		*vbat = (uint16_t)vbat_Value * 19.8f / 4096 * 100;
//		*imax = 0;
//		USER_Printf(" vbat value = %d \r\n ", *vbat); 
//	}
//	else
//	{
//		for (i = 0, imax_Value = 0, vbat_Value = 0; i < 6;)
//		{
//			
//			imax_Value += ADC_Value[i++];
//			i++;
//		}
//		imax_Value /= 3;
//		*vbat = 0;
//		*imax =  (uint16_t) imax_Value * 3.3f / 4096 / 0.2 * 1000;
//	//	USER_Printf(" imax value = %d \r\n", *imax);
//	}
//	HAL_ADC_Stop_DMA(&hadc1);
}
uint8_t  read_lockstatus(void)
{
	uint8_t open, close;
	int8_t  lock_status = -1;
	open = HAL_GPIO_ReadPin(open_GPIO_Port, open_Pin);
	close = HAL_GPIO_ReadPin(close_GPIO_Port, close_Pin);	
	if (open == 1&close == 1)
	{
		lock_status = lock_close;
	}
	if (open==0&close==0)
	{
		lock_status = lock_open;
	}
	if (open == 0&close == 1)
	{
		lock_status = lock_illegalopen;
	}
	if (open == 1&close == 0)
	{
		lock_status = lock_illegalclose;
	}
	return lock_status;
}




uint16_t read_distance()
{
	uint16_t Meter;
	/*##-3- Start the Input Capture in interrupt mode ##########################*/
	__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
	if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_SET);
	osDelay(1);
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);
	osDelay(30);
	Meter = (uint16_t) uwDiffCapture * 340.0f / 200.0f;
//	USER_Printf(" uwDiffCapture value = %d \r\n ", uwDiffCapture); 
//	USER_Printf(" Meter value = %d \r\n ", Meter); 

		return Meter;
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uwDiffCapture = 0;
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		if (uhCaptureIndex == 0)
		{
			/* Get the 1st Input Capture value */
			uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
			uhCaptureIndex = 1;
		}
		else if (uhCaptureIndex == 1)
		{
			/* Get the 2nd Input Capture value */
			uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); 
			HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_3);
			/* Capture computation */
			if (uwIC2Value2 > uwIC2Value1)
			{
				uwDiffCapture = (uwIC2Value2 - uwIC2Value1); 
			}
			else if (uwIC2Value2 < uwIC2Value1)
			{
				/* 0xFFFF is max TIM2_CCRx value */
				uwDiffCapture = ((0xFFFF - uwIC2Value1) + uwIC2Value2) + 1;
			}
			else
			{
				/* If capture values are equal, we have reached the limit of frequency
				   measures */
			//	Error_Handler();
			}
			/* Frequency computation: for this example TIMx (TIM2) is clocked by
			   APB1Clk */      
			uhCaptureIndex = 0;
		}
	}
}
uint8_t ec20send(unsigned char* buffer, int len, int timeout_ms)
{

	char buf[50] = { 0 };
	sprintf(buf,"AT+QISEND=0,%d", len);

	if (ec20_send(buf,">", 1, 300) == 0)
	{
		HAL_UART_Transmit(&huart2, buffer, len, 0xFFFF);
	}
	return 0;
}

uint8_t ec20recv(unsigned char* buffer, int len, int timeout_ms)
{
	BaseType_t xResult;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(timeout_ms); /*  wait_time */
	EC20_MSG_T *ec20_msg;
	const char ack[] = { "+QIURC: \"recv\"" }; 
	xResult = xQueueReceive(EC20QueueHandle,
		(void *)&ec20_msg,
		(TickType_t)xMaxBlockTime); /* time */
	if (xResult == pdPASS)
	{
		HAL_UART_Transmit(&huart2, ec20_msg->Data, ec20_msg->lengh, 0xFFFF);
		if (strncmp((char *)&ec20_msg->Data, (char *)ack, sizeof(ack)) == 0)
		{
			memset(&EC20_MSG, 0, sizeof(EC20_MSG));
			xResult = xQueueReceive(EC20QueueHandle,
				(void *)&ec20_msg,
				(TickType_t)xMaxBlockTime); /* time */
			if (xResult == pdPASS)
			{
				memcpy(buffer, ec20_msg->Data, ec20_msg->lengh);
			}
			else
			{
				
			}
			memset(&EC20_MSG, 0, sizeof(EC20_MSG));
		}
		/* memcpy(tcp_c_msg->Data, rs485, sizeof(rs485));*/

		memset(&EC20_MSG, 0, sizeof(EC20_MSG));
	}
	else
	{
	}
	return 0;
}

