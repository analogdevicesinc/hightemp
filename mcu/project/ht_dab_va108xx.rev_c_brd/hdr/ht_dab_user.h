/***************************************************************************************
 * @file     ht_dab_user.h
 * @version  V1.0
 * @date     18. Jan 2018
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
 
#ifndef __HT_DAB_USER_H
#define __HT_DAB_USER_H

#include "va108xx.h"
#include "ht_dab.h"

/*  */
#define configSYSCLK        (50000000)

/* The number of times to sample the RTC after every acquisition. */
#define configRTD_SAMPLES 4

/* The number of times to sample the VCC rail after every acquisition. */
#define configVCC_SAMPLES 4

/* Values to display if RTD conversion ends up out-of-range */
#define RTD_ERRVAL_HIGH   (9999+2732)
#define RTD_ERRVAL_LOW   (-9999+2732)

/* Configures the serial baud rate */
#define configUART_BAUDRATE (2000000)

/* Enables UART CTS flow control. */
#define configUART_CTS_FLOW_CONTROL

/* No UART transmit buffer. All transmissions are blocking operations. */

/* Size of application's UART receive buffer. */
#define configUART_RX_BUFFER_SIZE   256

/* Enables FreeRTOS.  (comment out next line if no RTOS used */
//#define USE_RTOS   1  
//#define configENABLE_RTOS

/* Enables Watchdog. */
//#define configENABLE_WDT

/* Hard coded application version. */
#define SOFTWARE_VERSION    "020518_V1"

/* Set by Vorago -  Do not change - Contains boot config info */
#define VOR_EF_CONFIG  (0x81400701) 

#endif
