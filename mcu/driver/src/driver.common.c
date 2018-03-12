/***************************************************************************************
 * @file     driver_common.c
 * @version  V1.0
 * @date     31. March 2016
 *
 * @note
 * VORAGO Technologies 
 *
 * @note
 * Copyright (c) 2013-2016 VORAGO Technologies. 
 *
 * @par
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND BY 
 * ALL THE TERMS AND CONDITIONS OF THE VORAGO TECHNOLOGIES END USER LICENSE AGREEMENT. 
 * THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. VORAGO TECHNOLOGIES 
 * SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL 
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ****************************************************************************************/
#include "driver_common.h"
#include "va108xx.h"
//  #include "reb_board.h"

uint32_t VOR_Get_ClockFreq(void) 
{
	return SystemCoreClock;
}

uint32_t VOR_Get_SysTime(void) 
{
	uint32_t ret;
  return ret;
}

void VOR_Sleep(uint32_t mtime) 
{
	uint32_t i, msec = (VOR_Get_ClockFreq()/6000)*mtime;
	for(i=0; i<msec; i++)
		__NOP( ); 
}

void VOR_GPIO_PinMux(uint32_t bank, uint32_t pin, uint32_t funsel) {
	if(bank==0)
		VOR_IOCONFIG->PORTA[pin] = (VOR_IOCONFIG->PORTA[pin] & ~((0x3)<<IOCONFIG_PORTA_FUNSEL_Pos))	| ((funsel)<<IOCONFIG_PORTA_FUNSEL_Pos);
	else
		VOR_IOCONFIG->PORTB[pin] = (VOR_IOCONFIG->PORTB[pin] & ~((0x3)<<IOCONFIG_PORTB_FUNSEL_Pos))	| ((funsel)<<IOCONFIG_PORTB_FUNSEL_Pos);
	
}
