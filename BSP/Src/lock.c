

#include "lock.h"
#include "adc.h"
#include  "math.h"
#include "common.h"

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
	case stoplock:
		HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);
		lock_status = lock_stop;
		
		break;
//	case:
//		break;
		
	default:
		break;

	}
return lock_status;	
}

void lcok_readadc(uint16_t * imax, uint16_t* vbat)
{
	uint32_t  ADC_Value[20] = { 0 };
	uint32_t  imax_Value, vbat_Value;
	uint8_t i;
	osDelay(100);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, 20);
	osDelay(100);
	
	if (imax == NULL)
	{
		for (i = 0, imax_Value = 0, vbat_Value = 0; i < 20;)
		{
			i++;
			vbat_Value += ADC_Value[i++];
		}
		vbat_Value /= 10;
		*vbat = (uint16_t)vbat_Value * 19.8f / 4096 * 100;
		*imax = 0;
		USER_Printf(" vbat value = %d \r\n ", *vbat); 
	}
	else
	{
		for (i = 0, imax_Value = 0, vbat_Value = 0; i < 20;)
		{
			
			imax_Value += ADC_Value[i++];
			i++;
		}
		imax_Value /= 10;
		*vbat = 0;
		*imax =  (uint16_t) imax_Value * 3.3f / 4096 / 0.2 * 1000;
		USER_Printf(" imax value = %d \r\n", *imax);
		
	}

 
	HAL_ADC_Stop_DMA(&hadc1);
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
 //uint8_t openlock(void)
//{
//	uint8_t ret;
//	HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);
//	if (HAL_GPIO_ReadPin(close_GPIO_Port, close_Pin) == 0 && HAL_GPIO_ReadPin(open_GPIO_Port, open_Pin) == 0)
//	{
//		HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);	 
//		ret = 0;
//	}
//	else
//	{
//		ret = 1;	
//	}
//	return ret;
//}
//uint8_t closelock(void)
//{
//	uint8_t ret;
//	HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_SET);
//	if (HAL_GPIO_ReadPin(close_GPIO_Port, close_Pin) == 1 && HAL_GPIO_ReadPin(open_GPIO_Port, open_Pin) == 1)
//	{
//		HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);	 
//		ret = 0;
//	}
//	else
//	{
//		ret = 1;
//	}
//	return ret;
//}
//uint8_t stoplock(void)
//{
//	uint8_t ret;
//	HAL_GPIO_WritePin(Mh_GPIO_Port, Mh_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(Ml_GPIO_Port, Ml_Pin, GPIO_PIN_RESET);
//	return ret;
//}

