/***************************************************************************************
 * @file     irq_va108xx.h
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
#ifndef __IRQ_VA108XX_H
#define __IRQ_VA108XX_H

#include <stdint.h>
#include "reb_board.h"

typedef enum {
  VOR_IRQ_GPIOA,
  VOR_IRQ_GPIOB,
  VOR_IRQ_TIM,
  VOR_IRQ_UART,
  VOR_IRQ_SPI,
  VOR_IRQ_I2C_MS,
  VOR_IRQ_I2C_SL,
} Irq_module;


/**
  \fn          int32_t Disable_Irq (uint32_t module, uint32_t portNum)
  \brief       Disbable NVIC Interrupt.
  \param[in]   module  specifies peripheral module - GPIO, TIM, SPI, I2C or UART
  \param[in]   portNum specifiec port number
  \return      \ref execution_status
*/
int32_t VOR_Disable_Irq(uint32_t  module,  uint32_t  inst );

/**
  \fn          int32_t VOR_Enable_Irq (uint32_t module, uint32_t portNum)
  \brief       Enable NVIC Interrupt.
  \param[in]   module  specifies peripheral module - GPIO, TIM, SPI, I2C or UART
  \param[in]   portNum specifies port number 
  \return      \ref execution_status
*/
int32_t VOR_Enable_Irq(uint32_t  module,  uint32_t  inst );

#endif

