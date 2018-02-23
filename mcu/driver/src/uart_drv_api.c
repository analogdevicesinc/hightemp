/***************************************************************************************
 * @file     uart_drv_api.c
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
#include "uart_va108xx.h"
#include "driver_common.h"
#ifndef RTT_LOG
extern UART_RESOURCES UART0_Resources;
extern UART_RESOURCES UART1_Resources;

static int32_t UART0_Initialize (VOR_UART_SignalEvent_t cb_event) {
  return UART_Initialize (cb_event, &UART0_Resources);
}
static int32_t UART0_Uninitialize (void) {
  return UART_Uninitialize(&UART0_Resources);
}
static int32_t UART0_Send (uint8_t *data, uint32_t num) {
  return UART_Send (data, num, &UART0_Resources);
}
static int32_t UART0_Receive (uint8_t *data, uint32_t num) {
  return UART_Receive (data, num, &UART0_Resources);
}

static int32_t UART0_GetTxCount (void) {
  return VOR_UART_GetTxCount(&UART0_Resources);
}

static int32_t UART0_GetRxCount (void) {
  return VOR_UART_GetRxCount(&UART0_Resources);
}

static int32_t UART0_Control (uint32_t control, uint32_t arg) {
  return UART_Control (control, arg, &UART0_Resources);
}
static VOR_UART_STATUS UART0_GetStatus (void) {
  return UART_GetStatus (&UART0_Resources);
}
void VOR_UART0_IRQHandler (void) {
  UART_IRQHandler (&UART0_Resources);
}

// UART0 Driver Control Block
VOR_DRIVER_UART Driver_UART0 = {
    UARTx_GetVersion,
    UART0_Initialize,
    UART0_Uninitialize,
    UART0_Send, 
    UART0_Receive,
	  UART0_GetTxCount,
	  UART0_GetRxCount,
    UART0_Control,
    UART0_GetStatus,
};

// UART1 Driver Wrapper functions
static int32_t UART1_Initialize (VOR_UART_SignalEvent_t cb_event) {
  return UART_Initialize (cb_event, &UART1_Resources);
}
static int32_t UART1_Uninitialize (void) {
  return UART_Uninitialize(&UART1_Resources);
}
static int32_t UART1_Send (uint8_t *data, uint32_t num) {
  return UART_Send (data, num, &UART1_Resources);
}
static int32_t UART1_Receive (uint8_t *data, uint32_t num) {
  return UART_Receive (data, num, &UART1_Resources);
}
static int32_t UART1_GetTxCount (void) {
  return VOR_UART_GetTxCount(&UART1_Resources);
}
static int32_t UART1_GetRxCount (void) {
  return VOR_UART_GetRxCount(&UART1_Resources);
}
static int32_t UART1_Control (uint32_t control, uint32_t arg) {
  return UART_Control (control, arg, &UART1_Resources);
}
static VOR_UART_STATUS UART1_GetStatus (void) {
  return UART_GetStatus (&UART1_Resources);
}
void VOR_UART1_IRQHandler (void) {
  UART_IRQHandler (&UART1_Resources);
}

// UART1 Driver Control Block
VOR_DRIVER_UART Driver_UART1 = {
    UARTx_GetVersion,
    UART1_Initialize,
    UART1_Uninitialize,
    UART1_Send, 
    UART1_Receive,
	  UART1_GetTxCount,
	  UART1_GetRxCount,
    UART1_Control,
    UART1_GetStatus,
};
#endif

