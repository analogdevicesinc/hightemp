/***************************************************************************************
 * @file     ht_dab.h
 * @version  V1.0
 * @date     18 January 2018
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
/** @file */ 
/** \addtogroup HTDAB1 
 *  @{
 */

 
#ifndef __HT_DAB_H
#define __HT_DAB_H

#include "va108xx.h"

#include "irq_vectors.h"

/* */
#undef VOR_PORTA
#undef VOR_PORTB
#define VOR_PORTA           (VOR_GPIO->BANK[0])
#define VOR_PORTB           (VOR_GPIO->BANK[1])

#define VOR_UART0           (VOR_UART->BANK[0])
#define VOR_UART1           (VOR_UART->BANK[1])

#define VOR_SPI0            (VOR_SPI->BANK[0])
#define VOR_SPI1            (VOR_SPI->BANK[1])

/* Timer that generates second tick. */
#define VOR_TIM_SEC_INDEX   (19)
#define VOR_TIM_SEC         (VOR_TIM->BANK[VOR_TIM_SEC_INDEX])

/* Timer that generates ADC0 CONV signal. */
#define VOR_TIM_ADC0_INDEX  (2)            
#define VOR_TIM_ADC0_CONV   (VOR_TIM->BANK[VOR_TIM_ADC0_INDEX])
 
/* Timer that generates ADC1 CONV signal. */
#define VOR_TIM_ADC1_INDEX  (20)
#define VOR_TIM_ADC1_CONV   (VOR_TIM->BANK[VOR_TIM_ADC1_INDEX])

/* Timer that generates ADC2 CONV signal. */
#define VOR_TIM_ADC2_INDEX  (18)
#define VOR_TIM_ADC2_CONV   (VOR_TIM->BANK[VOR_TIM_ADC2_INDEX])

/* Timer that generates interrupt to sample ADC0 and ADC1 simultaneously. */
#define VOR_TIM_TRIG_INDEX 	(0)
#define VOR_TIM_TRIG        (VOR_TIM->BANK[VOR_TIM_TRIG_INDEX])

/**/
#define VOR_SPI_ADC0_INDEX  1
#define VOR_SPI_ADC0        (VOR_SPI->BANK[VOR_SPI_ADC0_INDEX])
#define VOR_SPI_ADC1_INDEX  0
#define VOR_SPI_ADC1        (VOR_SPI->BANK[VOR_SPI_ADC1_INDEX])
#define VOR_SPI_ADC2_INDEX  1
#define VOR_SPI_ADC2        (VOR_SPI->BANK[VOR_SPI_ADC2_INDEX])

/**/
#define EXT_TRIG_PORT       (VOR_UART1)
#define EXT_TRIG_PIN        (14)

#define ADC2_MUX_EN_PIN     (23)
#define ADC2_MUX0_PIN       (20)


#define WDT_PERIOD_SEC      (3.0f)
#define WDT_PERIOD_CLKS     (((uint32_t)(configSYSCLK * WDT_PERIOD_SEC)) + 1)



/* Size of ADC buffer. */
#define ADC_BUFFER_SIZE     (8192)


//Takes a mux ch number (0 to 7) and outputs 3 select pin values, must be masked
#define SHIFTED_MUX_SEL_PINS(X)     ((X & ADC2_MUX_WIDTH) << ADC2_MUX0_PIN)  

/* Cascade sources missing from libraries and used by application. */
#define TIM_CAS_SRC_TIM_0    (64) 
#define TIM_CAS_SRC_PORTB_14 (32 + 14)
#define TIM_CAS_SRC_PORTB_15 (32 + 15)
#define TIM_CAS_SRC_TRIG_PIN (32 + EXT_TRIG_PIN)

#ifdef USE_WDT
#define VOR_TIM_WDT_INDEX   (5)
#define VOR_TIM_WDT         (VOR_SPI->BANK[VOR_TIM_WDT_INDEX])
#endif

//missing from our design environment
#define STAT_SEL_PWMA  (0x3)  

#define NULL_PTR ((uint16_t *)0)

/* RTD pullup and pulldown resistors in ohms. */
#define RTD_R_PULLUP            (1000.0f)
#define RTD_R_PULLDOWN          (1000.0f)

/* VCC reference voltage in volts. */
#define VCC_REF                 (2.5f)

/* VCC voltage divider resistors in ohms. */
#define VCC_R_DIV_UPPER         (2000.0f)
#define VCC_R_DIV_LOWER         (1000.0f)

/* ADC upper and lower bias resistors in ohms. */
#define ADC_R_BIAS_UPPER        (100000.0f)
#define ADC_R_BIAS_LOWER        (100000.0f)



#define PA0   0
#define PA1   1
#define PA2_UARTB_RX   2
#define PA3_UARTB_TX   3
#define PA4   4
#define PA5   5
#define PA6_UARTA_CTS   6
#define PA7   7
#define PA8   8
#define PA9   9
#define PA10  10
#define PA11  11
#define PA12  12
#define PA13  13
#define PA14  14
#define PA15  15
#define PA16_UARTA_RX  16
#define PA17_UARTA_TX  17
#define PA18_UARTB_RX  18
#define PA19_UARTB_TX  19
#define PA20  20
#define PA21  21
#define PA22  22
#define PA23  23
#define PA24  24
#define PA25  25
#define PA26  26
#define PA27  27
#define PA28_TIM20  28
#define PA29_MISOA  29
#define PA30  30
#define PA31_SCKA  31

#define PB0   0
#define PB1   1
#define PB2_TIM2   2
#define PB3_MISOB   3
#define PB4   4
#define PB5_SCKB  5
#define PB6   6
#define PB7   7
#define PB8   8
#define PB9   9
#define PB10  10
#define PB11  11
#define PB12  12
#define PB13  13
#define PB14_EXTRIG  14
#define PB15  15
#define PB16  16
#define PB17_MISOB  17
#define PB18_TIM18 18
#define PB19_SCKB  19
#define PB20  20
#define PB21  21
#define PB22  22
#define PB23  23

#endif

/*  end of ht_dab.h   */

