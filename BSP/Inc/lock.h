#ifndef __LOCK_H
#define __LOCK_H


#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f1xx_hal.h"

typedef enum 
	{
	lock_open = 10,
	lock_close,
	lock_stop,
	lock_normal,
	lock_illegalopen,
	lock_illegalclose,	
    controling,
	Blockage,	

	}lock_status;
	 
typedef enum 
	{
	openlock=0,
	closelock,
	stoplock,
	resetlock,
	normal,
	
	}locknum;	 
	 
	 
	 
	 
	 
	 uint8_t lock_control(uint8_t type);
	void lcok_readadc(uint16_t * imax, uint16_t* vbat); 
	 uint8_t  read_lockstatus(void);
	 uint16_t read_distance();
#endif
	 
	 
	 
	 
	 
	 