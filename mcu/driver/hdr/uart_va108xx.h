/***************************************************************************************
 * @file     uart_va108xx.h
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
#ifndef __UART_VA108XX_H
#define __UART_VA108XX_H

#include "driver_uart.h"
#include "reb_board.h"

extern VOR_DRIVER_UART Driver_UART0;
extern VOR_DRIVER_UART Driver_UART1;
extern uint32_t VOR_Get_SysTime(void);


#define UART0_CLK_EN_MASK  (1<<8)
#define UART1_CLK_EN_MASK  (1<<9)


#define VOR_UART_DRV_VERSION VOR_DRIVER_VERSION_MAJOR_MINOR(1,00)
#define UART_MAX_TXDATA  					128
#define UART_MAX_RXDATA  					128
#define UART_FLAG_INIT       			(1 << 0)        // Driver initialized

// UART Information (Run-Time)
typedef struct _UART_INFO {
  VOR_UART_SignalEvent_t  cb_event;           		// Event callback
  VOR_UART_STATUS         status;             		// Status flags
	uint32_t                flags;                  // Flags
	uint32_t                poll_mode;              // Polling/Interrupt Mode
  uint32_t                poll_tout;              // Polling Mode Timeout
  uint32_t                baudrate;           		// Baudrate
	uint32_t                txcnt;              		// Transmit data count
  uint32_t                rxcnt;              		// Receive data count
  uint32_t                txidx;              		// Current TX index
  uint32_t                rxidx;              		// Current RX index
  uint8_t                 txdata[UART_MAX_TXDATA];// TX data buffer
	uint8_t                 *rxdata;                // Pointer to TX data buffer
} UART_INFO;


// UART Resources definitions
typedef struct {
	uint32_t                    uart_id;
  VOR_UART_PERIPHERAL_Type    *reg;      // Pointer to UART peripheral
  VOR_DRIVER_UART             *drv;
  UART_INFO                   info;      // Run-Time Information
  uint32_t                    uartClkEn; // UART Peripheral Clock enable Mask 
} UART_RESOURCES;

/**
  \fn          VOR_DRIVER_VERSION UARTx_GetVersion (void)
  \brief       Get driver version.
  \return      \ref VOR_DRIVER_VERSION
*/
VOR_DRIVER_VERSION UARTx_GetVersion (void);

/**
  \fn          int32_t UART_Initialize (VOR_UART_SignalEvent_t  cb_event
                                         UART_RESOURCES         *UART)
  \brief       Initialize UART Interface.
  \param[in]   cb_event  Pointer to \ref VOR_UART_SignalEvent
  \param[in]   UART     Pointer to UART resources
  \return      \ref execution_status
*/
int32_t UART_Initialize (VOR_UART_SignalEvent_t, UART_RESOURCES *);

/**
  \fn          int32_t UART_Uninitialize (UART_RESOURCES *UART)
  \brief       De-initialize UART Interface.
  \param[in]   UART     Pointer to UART resources
  \return      \ref execution_status
*/
int32_t UART_Uninitialize (UART_RESOURCES *);

/**
  \fn          int32_t UART_Send (const void            *data,
                                         uint32_t         num,
                                         UART_RESOURCES *UART)
  \brief       Start sending data to UART transmitter.
  \param[in]   data  Pointer to buffer with data to send to UART transmitter
  \param[in]   num   Number of data items to send
  \param[in]   UART Pointer to UART resources
  \return      \ref execution_status
*/
int32_t UART_Send (uint8_t *, uint32_t, UART_RESOURCES *);

/**
  \fn          int32_t UART_Receive (void            *data,
                                      uint32_t         num,
                                      UART_RESOURCES *UART)
  \brief       Start receiving data from UART receiver.
  \param[out]  data  Pointer to buffer for data to receive from UART receiver
  \param[in]   num   Number of data items to receive
  \param[in]   UART Pointer to UART resources
  \return      \ref execution_status
*/
int32_t UART_Receive (uint8_t *data, uint32_t num, UART_RESOURCES *UART);

/**
  \fn          int32_t UART_Control (uint32_t          control,
                                      uint32_t          arg,
                                      UART_RESOURCES  *UART)
  \brief       Control UART Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \param[in]   UART    Pointer to UART resources
  \return      common \ref execution_status and driver specific \ref UART_execution_status
*/
int32_t UART_Control (uint32_t , uint32_t , UART_RESOURCES *);

/**
  \fn          VOR_UART_STATUS UART_GetStatus (UART_RESOURCES *UART)
  \brief       Get UART status.
  \param[in]   UART     Pointer to UART resources
  \return      UART status \ref VOR_UART_STATUS
*/
VOR_UART_STATUS UART_GetStatus (UART_RESOURCES *);

/**
  \fn          void UART_IRQHandler (UART_RESOURCES *UART)
  \brief       UART Interrupt handler.
  \param[in]   UART     Pointer to UART resources
*/
void UART_IRQHandler (UART_RESOURCES *UART);

/*
  \fn          uint32_t VOR_UART_GetTxCount (void)
  \brief       Get transmitted data count.
  \return      number of data items transmitted
*/
int32_t VOR_UART_GetTxCount (UART_RESOURCES *UART);
/*
  \fn          uint32_t VOR_UART_GetRxCount (void)
  \brief       Get received data count.
  \return      number of data items received
*/
int32_t VOR_UART_GetRxCount (UART_RESOURCES *UART);

#endif /* __UART_VA108XX_H */
