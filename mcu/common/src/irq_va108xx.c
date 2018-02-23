/***************************************************************************************
 * @file     irq_va108xx.c
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
#include "irq_va108xx.h"

int32_t VOR_Disable_Irq(uint32_t module, uint32_t inst) {
	switch(module) {
		case VOR_IRQ_GPIOA:
			NVIC_DisableIRQ(IRQ_GPIO_IRQn);
			break;
		case VOR_IRQ_GPIOB:
			NVIC_DisableIRQ(IRQ_GPIO_IRQn);
			break;
		case VOR_IRQ_TIM:
			if(inst == 0)
				NVIC_DisableIRQ(IRQ_TIM0_IRQn);
			else if(inst == 1)
				NVIC_DisableIRQ(IRQ_TIM1_IRQn);
			//else if(inst == 3)
				//NVIC_DisableIRQ(IRQ_TIM3_IRQn);
			break;
		case VOR_IRQ_UART:
			if(inst == 0)
				NVIC_DisableIRQ(IRQ_UART0_IRQn);
			else if(inst == 1)
				NVIC_DisableIRQ(IRQ_UART1_IRQn);
			break;
		case VOR_IRQ_SPI:
			if(inst == 0)
				NVIC_DisableIRQ(IRQ_SPI0_IRQn);
			else if(inst == 1)
				NVIC_DisableIRQ(IRQ_SPI1_IRQn);
			else if(inst == 2)
				NVIC_DisableIRQ(IRQ_SPI2_IRQn);
			break;
		case VOR_IRQ_I2C_MS:
			if(inst == 0)
				NVIC_DisableIRQ(IRQ_I2C0_MS_IRQn);
			else if(inst == 1)
				NVIC_DisableIRQ(IRQ_I2C1_MS_IRQn);
		case VOR_IRQ_I2C_SL:
			if(inst == 0)
				NVIC_DisableIRQ(IRQ_I2C0_SL_IRQn);
			else if(inst == 1)
				NVIC_DisableIRQ(IRQ_I2C1_SL_IRQn);
			break;
		default:
			break;
  };
  return 0;
}

int32_t VOR_Enable_Irq(uint32_t module, uint32_t inst) {
	switch(module) {
		case VOR_IRQ_GPIOA:
			VOR_IRQSEL->PORTA[inst] = IRQ_GPIO_IRQn;
			NVIC_SetPriority(IRQ_GPIO_IRQn,IRQ_GPIO_PRIORITY);
			NVIC_EnableIRQ(IRQ_GPIO_IRQn);
			break;
		case VOR_IRQ_GPIOB:
			VOR_IRQSEL->PORTB[inst] = IRQ_GPIO_IRQn;
			NVIC_SetPriority(IRQ_GPIO_IRQn,IRQ_GPIO_PRIORITY);
			NVIC_EnableIRQ(IRQ_GPIO_IRQn);
			break;
		case VOR_IRQ_TIM:
			if(inst == 0) {
				VOR_IRQSEL->TIM[inst] = IRQ_TIM0_IRQn;	
				NVIC_SetPriority(IRQ_TIM0_IRQn,IRQ_TIM0_PRIORITY);
				NVIC_EnableIRQ(IRQ_TIM0_IRQn);
			} else if(inst == 1) {
				VOR_IRQSEL->TIM[inst] = IRQ_TIM1_IRQn;	
				NVIC_SetPriority(IRQ_TIM1_IRQn,IRQ_TIM1_PRIORITY);
				NVIC_EnableIRQ(IRQ_TIM1_IRQn);
			} else if(inst == 3) {
				//VOR_IRQSEL->TIM[inst] = IRQ_TIM3_IRQn;	
				//NVIC_SetPriority(IRQ_TIM3_IRQn,IRQ_TIM3_PRIORITY);
				//NVIC_EnableIRQ(IRQ_TIM3_IRQn);
			} else {
				return -1; 
			}
			break;
		case VOR_IRQ_UART:
			if(inst == 0) {
				VOR_IRQSEL->UART[inst] = IRQ_UART0_IRQn;
				NVIC_SetPriority(IRQ_UART0_IRQn,IRQ_UART0_PRIORITY);
				NVIC_EnableIRQ(IRQ_UART0_IRQn);
			} else if(inst == 1) {
				VOR_IRQSEL->UART[inst] = IRQ_UART1_IRQn;
				NVIC_SetPriority(IRQ_UART1_IRQn,IRQ_UART1_PRIORITY);
				NVIC_EnableIRQ(IRQ_UART1_IRQn);
			}
			break;
		case VOR_IRQ_SPI:
			if(inst == 0) {
				VOR_IRQSEL->SPI[inst] = IRQ_SPI0_IRQn;
				NVIC_SetPriority(IRQ_SPI0_IRQn,IRQ_SPI0_PRIORITY);
				NVIC_EnableIRQ(IRQ_SPI0_IRQn);
			} else if(inst == 1) {
				VOR_IRQSEL->SPI[inst] = IRQ_SPI1_IRQn;
				NVIC_SetPriority(IRQ_SPI1_IRQn,IRQ_SPI1_PRIORITY);
				NVIC_EnableIRQ(IRQ_SPI1_IRQn);
      } else if(inst == 2) {
				VOR_IRQSEL->SPI[inst] = IRQ_SPI2_IRQn;
				NVIC_SetPriority(IRQ_SPI2_IRQn,IRQ_SPI2_PRIORITY);
				NVIC_EnableIRQ(IRQ_SPI2_IRQn);
      }
			break;
		case VOR_IRQ_I2C_MS:
			if(inst == 0) {
				VOR_IRQSEL->I2C_MS[inst] = IRQ_I2C0_MS_IRQn;
				NVIC_SetPriority(IRQ_I2C0_MS_IRQn,IRQ_I2C0_MS_PRIORITY);
				NVIC_EnableIRQ(IRQ_I2C0_MS_IRQn);
			} else if(inst == 1) {
				VOR_IRQSEL->I2C_MS[inst] = IRQ_I2C1_MS_IRQn;
				NVIC_SetPriority(IRQ_I2C1_MS_IRQn,IRQ_I2C1_MS_PRIORITY);
				NVIC_EnableIRQ(IRQ_I2C1_MS_IRQn);
			}
			break;
		case VOR_IRQ_I2C_SL:
			if(inst == 0) {
				VOR_IRQSEL->I2C_SL[inst] = IRQ_I2C0_SL_IRQn;
				NVIC_SetPriority(IRQ_I2C0_SL_IRQn,IRQ_I2C0_SL_PRIORITY);
				NVIC_EnableIRQ(IRQ_I2C0_SL_IRQn);
			} else if(inst == 1) {
				VOR_IRQSEL->I2C_SL[inst] = IRQ_I2C1_SL_IRQn;
				NVIC_SetPriority(IRQ_I2C1_SL_IRQn,IRQ_I2C1_SL_PRIORITY);
				NVIC_EnableIRQ(IRQ_I2C1_SL_IRQn);
			}
			break;
		default:
			break;
  };
  return 0;	
}
