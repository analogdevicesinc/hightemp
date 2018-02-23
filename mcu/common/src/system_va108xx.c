/**************************************************************************//**
 * @file     system_ARMCM0.c
 * @brief    CMSIS Device System Source File for
 *           ARMCM0 Device Series
 * @version  V1.07
 * @date     30. January 2012
 *
 * @note
 * Copyright (C) 2012 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/
 /*
 From: C:\Keil\ARM\Device\ARM\ARMCM0\Source
 */

#include "va108xx.h"

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define __HSI             ( 8000000UL)
#define __XTAL            ( 50000000UL)   /* Oscillator frequency             */

#define __SYSTEM_CLOCK    (__XTAL)

#define VOR_EF_CONFIGx  (0x81400701)

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = __SYSTEM_CLOCK;/*!< System Clock Frequency (Core Clock)*/


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{

  SystemCoreClock = __SYSTEM_CLOCK;
	


}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void)
{

  SystemCoreClock = __SYSTEM_CLOCK;
	if(VOR_SYSCONFIG->EF_CONFIG != VOR_EF_CONFIGx )
		
	{
		VOR_SYSCONFIG->RST_STAT  = 0x0000   ;  
		NVIC_SystemReset(); /*  reset processor for clean read of eFuse information   */
		
	}
}
