/***************************************************************************************
 * @file     uart.h
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
/** \addtogroup UART 
 *  @{
 */
#ifndef __UART_H
#define __UART_H

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "ht_dab_user.h"
#include "ht_dab.h"

#ifndef configUART_RX_BUFFER_SIZE
#define configUART_RX_BUFFER_SIZE   8
#endif

/* Intializes UART0 peripheral. */
void UART0Init(void);

/* Writes a string to UART0, and blocks until complete. */
void UART0WriteStr(char *pcStr);

/* Reads a byte from the software RX Buffer. */
bool UART0Read(uint8_t *pui8Data);

#endif
