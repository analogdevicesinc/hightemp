/***************************************************************************************
 * @file     uart.c
 * @version  V1.0
 * @date     18. January 2018
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
/** \addtogroup UART
 *  @{
 */

#include "uart.h"

/* Variables for storing characters received from UART0. */
static uint8_t pui8RxBuffer[configUART_RX_BUFFER_SIZE];
static uint8_t *pui8RxHead;
static uint8_t *pui8RxTail;
static uint8_t *pui8RxEnd;

/*******************************************************************************
 **
 ** @brief Configures UART0 peripheral.
 **
 **  Initializes buffers and configures UART for 2 Mb/sec, RTS/CTS enabled. 
 **
 ******************************************************************************/
void
UART0Init(void)
{
    /* Initialize variables. */
    pui8RxHead = pui8RxBuffer;
    pui8RxTail = pui8RxBuffer;
    pui8RxEnd = pui8RxBuffer + configUART_RX_BUFFER_SIZE;

    /* Configure UART to operate at configUART_BAUDRATE. */
    VOR_UART0.CLKSCALE = ((configSYSCLK / (configUART_BAUDRATE * 16)) <<
                           UART_PERIPHERAL_CLKSCALE_INT_Pos) |
                          (((((configSYSCLK % (configUART_BAUDRATE * 16)) *
                              64 + (configUART_BAUDRATE * 8)) /
                             (configUART_BAUDRATE * 16))) <<
                           UART_PERIPHERAL_CLKSCALE_FRAC_Pos);
                     
    /* Configure word size and RTS behavior. */
    VOR_UART0.CTRL = (3 << UART_PERIPHERAL_CTRL_WORDSIZE_Pos) |
                      (UART_PERIPHERAL_CTRL_DEFRTS_Msk);

    /* Enable CTS flow control IO, if needed. */
#ifdef configUART_CTS_FLOW_CONTROL
    VOR_UART0.CTRL |= UART_PERIPHERAL_CTRL_AUTOCTS_Msk;
#endif

    /* Enable RTS flow control IO, if needed. */
#ifdef configUART_RTS_FLOW_CONTROL
    VOR_UART0.CTRL |= UART_PERIPHERAL_CTRL_AUTORTS_Msk;
#endif

	/* Enable RX interupts as soon as a character is received. */
	VOR_UART0.IRQ_ENB = UARTA_IRQ_ENB_IRQ_RX_Msk;
	VOR_UART0.RXFIFOIRQTRG = 1;
    
	VOR_IRQSEL->UART[0] = VOR_UART0_IRQ_INDEX;
	NVIC_SetPriority(VOR_UART0_IRQ_INDEX, 1);
	NVIC_EnableIRQ(VOR_UART0_IRQ_INDEX);	

    /* Enable UART. */
    VOR_UART0.ENABLE = (UART_PERIPHERAL_ENABLE_RXENABLE_Msk | 
                         UART_PERIPHERAL_ENABLE_TXENABLE_Msk);
}

/*******************************************************************************
 **
 ** @brief  Transmit a string across UART0.
 **
 ** Transmits a string across UART0, blocking until all characters are sent.
 **
 ** @param  pcStr is the pointer to the string that is transmitted across UART0.
 **
 ******************************************************************************/
void
UART0WriteStr(char *pcStr)
{
	while(*pcStr)
    {
        /* Block until UART is done transmitting data. */
        while((VOR_UART0.TXSTATUS & UART_PERIPHERAL_TXSTATUS_WRRDY_Msk) == 0)
        {
        }
        
        /* Transmit another character. */
        VOR_UART0.DATA = *pcStr++;
	}
}

/*******************************************************************************
 **
 ** @brief  Reads a character from the UART0 RX buffer.
 **
 ** @param  pui8Data is the pointer to store received data, if any exists.
 **
 ** @return Returns **true** if data was pulled from the RX buffer, otherwise
 **          returns **false** and zeros out pui8Data.
 **
 ******************************************************************************/
bool
UART0Read(uint8_t *pui8Data)
{   
    /* Disable UART RX interrupt. */
    NVIC_DisableIRQ(VOR_UART0_IRQ_INDEX);
    
    /* Verify that data is available. */
    if(pui8RxTail == pui8RxHead)
    {
        /* Zero out data pointer. */
        *pui8Data = 0;
        
        /* Re-enable UART RX interrupt. */
        NVIC_EnableIRQ(VOR_UART0_IRQ_INDEX);
        
        return(false);
    }
    
    /* Pull byte from buffer. */
	*pui8Data = *pui8RxTail++;
    
    /* Loop pointer if needed. */
	if(pui8RxTail == pui8RxEnd)
    {
		pui8RxTail = pui8RxBuffer;
    }
    
    /* Re-enable UART RX interrupt. */
	NVIC_EnableIRQ(VOR_UART0_IRQ_INDEX);
    
    return(true);
}

/*******************************************************************************
 **
 ** @brief  UART0 RX interrupt handler.
 **
 ** Reads characters received into RX buffer.  If the RX buffer is full, the
 ** contents of the RX buffer will be overwritten.
 **
 ******************************************************************************/
void
VOR_UART0_IRQHandler(void)
{
    /* Read bytes into buffer as long as bytes are available. */
	while((VOR_UART0.RXSTATUS & UART_PERIPHERAL_RXSTATUS_RDAVL_Msk) != 0)
    {
		*pui8RxHead++ = VOR_UART0.DATA;
		
        /* Loop pointer if needed. */
		if(pui8RxHead == pui8RxEnd)
        {
            pui8RxHead = pui8RxBuffer;
        }
	}
}
