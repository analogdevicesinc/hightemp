/***************************************************************************************
 * @file     driver_uart.h
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
/** \addtogroup UART 
 *  @{
 */
#ifndef __DRIVER_UART_H
#define __DRIVER_UART_H

#include "driver_common.h"

#define VOR_UART_API_VERSION VOR_DRIVER_VERSION_MAJOR_MINOR(1,00)  /* API version */


/*----- UART Control Codes: Mode Parameters -----*/

#define VOR_UART_CONTROL_PARITY_ENABLED    (1)       ///< Enable Parity (default)
#define VOR_UART_CONTROL_PARITY_SEL        (2)       ///< Even Odd Parity
#define VOR_UART_CONTROL_PARITY_MAN        (3)       ///< Manual Parity
#define VOR_UART_CONTROL_STOP_BIT          (4)       ///< Stop bit
#define VOR_UART_CONTROL_DATA_BITS         (5)       ///< Data bits
#define VOR_UART_CONTROL_LOOPBACK          (6)       ///< Loopback
#define VOR_UART_CONTROL_AUTOCTS           (7)       ///< Auto CTS
#define VOR_UART_CONTROL_DEFRTS            (8)       ///< Default RTS
#define VOR_UART_CONTROL_AUTORTS           (9)       ///< Auto RTS
#define VOR_UART_CONTROL_TX                (10)      ///< Transmitter Enabled; 
#define VOR_UART_CONTROL_RX                (11)      ///< Receiver Enabled;
#define VOR_UART_CONTROL_BREAK             (12)      ///< Continuous Break transmission
#define VOR_UART_ABORT_SEND                (13)      ///< Abort \ref VOR_UART_Send
#define VOR_UART_ABORT_RECV                (14)      ///< Abort \ref VOR_UART_Receive
#define VOR_UART_CONTROL_RXFIFOTRG         (15)      ///< RX fifo trigger
#define VOR_UART_CONTROL_TXFIFOTRG         (16)      ///< TX fifo trigger
#define VOR_UART_CONTROL_RXFIFORTSTRG      (17)      ///< RX fifo RTS trigger
#define VOR_UART_CONTROL_ENB9BIT           (18)      ///< ADDR9 
#define VOR_UART_CONTROL_ADDR9MSK          (19)      ///< ADDR9 mask
#define VOR_UART_CONTROL_FIFOCLR           (20)      ///< FIFO Clear
#define VOR_UART_CONTROL_POLL_TO           (21)
#define VOR_UART_CONTROL_POLL_MODE         (22)
#define VOR_UART_CONTROL_BAUD              (23)

/****** UART Control Args Codes *****/
#define VOR_UART_CONTROL_DATA_BITS_5       (0UL)    ///< 5 Data bits
#define VOR_UART_CONTROL_DATA_BITS_6       (1UL)    ///< 6 Data bit
#define VOR_UART_CONTROL_DATA_BITS_7       (2UL)    ///< 7 Data bits
#define VOR_UART_CONTROL_DATA_BITS_8       (3UL)    ///< 8 Data bits (default)
#define VOR_UART_CONTROL_DATA_BITS_9       (4UL)    ///< 9 Data mode
#define VOR_UART_CONTROL_PARITY_ODD        (0UL)    ///< Odd Parity
#define VOR_UART_CONTROL_PARITY_EVEN       (1UL)    ///< EvenParity
#define VOR_UART_CONTROL_STOP_BIT_1        (0UL)    ///< Stop bit 1
#define VOR_UART_CONTROL_STOP_BIT_2        (1UL)    ///< Stop bit 2
#define VOR_UART_CONTROL_PARITY_ENABLE  						1
#define VOR_UART_CONTROL_PARITY_DISABLE 						0
#define VOR_UART_CONTROL_MAN_PARITY_ENABLE 				1
#define VOR_UART_CONTROL_MAN_PARITY_DISABLE 				0


#define VOR_UART_CONTROL_LOOP_BACK_ENABLE 					1
#define VOR_UART_CONTROL_LOOP_BACK_DISABLE 				0
#define VOR_UART_CONTROL_AUTO_CTS_ENABLE 					1
#define VOR_UART_CONTROL_AUTO_CTS_DISABLE 					0
#define VOR_UART_CONTROL_DEFRTS_ENABLE 						1
#define VOR_UART_CONTROL_DEFRTS_DISABLE 						0
#define VOR_UART_CONTROL_AUTORTS_ENABLE 						1
#define VOR_UART_CONTROL_AUTORTS_DISABLE 					0
#define VOR_UART_CONTROL_TX_ENABLE 								1
#define VOR_UART_CONTROL_TX_DISABLE 								0
#define VOR_UART_CONTROL_RX_ENABLE 								1
#define VOR_UART_CONTROL_RX_DISABLE 								0
#define UART_BREAK_BIT 						0
#define VOR_UART_CONTROL_BREAK_CONTINOUS_COUNT 		0X7F
#define VOR_UART_CONTROL_BREAK_RESET 							0
#define UART_RXFIFOIRQTRG_BIT 		0
#define RXFIFOIRQTRG_MASK 				0X1F
#define UART_TXFIFOIRQTRG_BIT 		0
#define TXFIFOIRQTRG_MASK 				0X1F
#define UART_RXFIFORTSTRG_BIT 		0
#define RXFIFORTSTRG_MASK 				0X1F
#define UART_ENB9BIT_BIT 					16
#define VOR_UART_CONTROL_ENB9BIT_ENABLE 						1
#define VOR_UART_CONTROL_ENB9BIT_DISABLE 					0
#define UART_ADDR9MSK_BIT 				0
#define ADDR9MSK_MASK 						0XFF
#define VOR_UART_CONTROL_UART_RXSTS_BIT 						0
#define VOR_UART_CONTROL_UART_TXSTS_BIT 						1
#define VOR_UART_CONTROL_UART_RXFIFO_BIT 					2
#define VOR_UART_CONTROL_UART_TXFIFO_BIT 					3
#define CHECK_BIT(var,pos) 				((var) & (1<<(pos)))
#define INT_MODE 0
#define POLL_MODE 1


/****** UART specific error codes *****/
#define VOR_UART_ERROR_MODE                (VOR_DRIVER_ERROR_SPECIFIC - 1)     ///< Specified Mode not supported
#define VOR_UART_ERROR_BAUDRATE            (VOR_DRIVER_ERROR_SPECIFIC - 2)     ///< Specified baudrate not supported
#define VOR_UART_ERROR_DATA_BITS           (VOR_DRIVER_ERROR_SPECIFIC - 3)     ///< Specified number of Data bits not supported
#define VOR_UART_ERROR_PARITY              (VOR_DRIVER_ERROR_SPECIFIC - 4)     ///< Specified Parity not supported
#define VOR_UART_ERROR_STOP_BITS           (VOR_DRIVER_ERROR_SPECIFIC - 5)     ///< Specified number of Stop bits not supported
#define VOR_UART_ERROR_FLOW_CONTROL        (VOR_DRIVER_ERROR_SPECIFIC - 6)     ///< Specified Flow Control not supported
#define VOR_UART_ERROR_MULTIPLEINIT        (VOR_DRIVER_ERROR_SPECIFIC - 7)     ///< UART already initialized
#define VOR_UART_ERROR_POLL_TOUT           (VOR_DRIVER_ERROR_SPECIFIC - 8)     ///< UART Polling time out
#define VOR_UART_ERROR_FRAME               (VOR_DRIVER_ERROR_SPECIFIC - 9)     ///< UART Framming Error

/****** UART Status fields *****/
#define UART_SUCCESS 0
#define UART_FAILURE 1

/**
\brief UART Status     
*/
typedef struct _VOR_UART_STATUS {
  uint32_t RX_RDAVL 	   		 :1 ;    ///< Read Data Available
	uint32_t RX_RDNFULL        :1 ;    ///< Read FIFO Not Full
	uint32_t RX_BUSY           :1 ;    ///< Receiver busy
	uint32_t RX_TO             :1 ;    ///< Receiver Timeout
	uint32_t RX_OVR            :1 ;    ///< Receiver FIFO Overflow
	uint32_t RX_FRM            :1 ;    ///< Receive Framing error
	uint32_t RX_PAR            :1 ;    ///< Receive parity error
	uint32_t RX_BRK            :1 ;    ///< Receive break condition
	uint32_t RX_BUSYBRK        :1 ;    ///< Break has been received, but the receiver is still busy waiting from the break to end.
	uint32_t RX_ADDR9          :1 ;    ///< Receive 9 bit Address match
	uint32_t RX_RTS            :1 ;    //< Output value of RTS signal
	uint32_t TX_WRRDY          :1 ;    ///< Write Ready (transmit FIFO is not full)
	uint32_t TX_WRBUSY         :1 ;    ///< Write Busy  (transmit FIFO is not empty)
	uint32_t TX_BUSY           :1 ;    ///< Transmitter Busy (transmitter is currently sending a byte)
	uint32_t TX_WRLOST         :1 ;    ///< transmitter FIFO overflowed
	uint32_t TX_CTS            :1 ;    ///< Input value of CTS signal
} VOR_UART_STATUS;

/****** UART Event *****/     
#define VOR_UART_EVENT_RX_DONE   		       (1UL << 0)  ///< Receive completed; 
#define VOR_UART_EVENT_RX_OVERFLOW         (1UL << 1)  ///< Receive data overflow
#define VOR_UART_EVENT_RX_FRAMING_ERROR    (1UL << 2)  ///< Framing error detected on receive
#define VOR_UART_EVENT_RX_PARITY_ERROR     (1UL << 3)  ///< Parity error detected on receive
#define VOR_UART_EVENT_RX_BREAK            (1UL << 4)  ///< Break detected on receive
#define VOR_UART_EVENT_RX_TIMEOUT          (1UL << 5)  ///< Receive character timeout (optional)
#define VOR_UART_EVENT_RX_ADDR9            (1UL << 6)  ///< Receiver interrupt enable for matched address in 9-bit mode
#define VOR_UART_EVENT_TX_DONE             (1UL << 7)  ///< Transmit completed; 
#define VOR_UART_EVENT_TX_OVERFLOW         (1UL << 8)  ///< Transmitter FIFO overflow
#define VOR_UART_EVENT_TX_EMPTY            (1UL << 9)  ///< Transmitter FIFO Empty
#define VOR_UART_EVENT_TX_CTS              (1UL << 10) ///< CTS state changed (optional)


// Function documentation
/**
  \fn          VOR_DRIVER_VERSION VOR_UART_GetVersion (void)
  \brief       Get driver version.
  \return      \ref VOR_DRIVER_VERSION
  
  \fn          int32_t VOR_UART_Initialize (VOR_UART_SignalEvent_t cb_event)
  \brief       Initialize UART Interface.
  \param[in]   cb_event  Pointer to \ref VOR_UART_SignalEvent
  \return      \ref execution_status

  \fn          int32_t VOR_UART_Uninitialize (void)
  \brief       De-initialize UART Interface.
  \return      \ref execution_status

  \fn          int32_t VOR_UART_Send (const void *data, uint32_t num)
  \brief       Start sending data to UART transmitter.
  \param[in]   data  Pointer to buffer with data to send to UART transmitter
  \param[in]   num   Number of data items to send
  \return      \ref execution_status

  \fn          int32_t VOR_UART_Receive (void *data, uint32_t num)
  \brief       Start receiving data from UART receiver.
  \param[out]  data  Pointer to buffer for data to receive from UART receiver
  \param[in]   num   Number of data items to receive
  \return      \ref execution_status

  \fn          uint32_t VOR_UART_GetTxCount (void)
  \brief       Get transmitted data count.
  \return      number of data items transmitted

  \fn          uint32_t VOR_UART_GetRxCount (void)
  \brief       Get received data count.
  \return      number of data items received

  \fn          int32_t VOR_UART_Control (uint32_t control, uint32_t arg)
  \brief       Control UART Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      common \ref execution_status and driver specific \ref UART_execution_status

  \fn          VOR_UART_STATUS VOR_UART_GetStatus (void)
  \brief       Get UART status.
  \return      UART status \ref VOR_UART_STATUS

  \fn          void VOR_UART_SignalEvent (uint32_t event)
  \brief       Signal UART Events.
  \param[in]   event  \ref UART_events notification mask
  \return      none
*/

typedef void (*VOR_UART_SignalEvent_t) (uint32_t event);  ///< Pointer to \ref VOR_UART_SignalEvent : Signal UART Event.


/**
\brief Access structure of the UART Driver.
*/
typedef struct _VOR_DRIVER_UART {
  VOR_DRIVER_VERSION     (*GetVersion)      (void);                                    ///< Pointer to \ref VOR_UART_GetVersion : Get driver version.
  int32_t                (*Initialize)      (VOR_UART_SignalEvent_t cb_event);         ///< Pointer to \ref VOR_UART_Initialize : Initialize UART Interface.
  int32_t                (*Uninitialize)    (void);                                    ///< Pointer to \ref VOR_UART_Uninitialize : De-initialize UART Interface.
  int32_t                (*Send)            (uint8_t *data, uint32_t num);             ///< Pointer to \ref VOR_UART_Send : Start sending data to UART transmitter.
  int32_t                (*Receive)         (uint8_t *data, uint32_t num);             ///< Pointer to \ref VOR_UART_Receive : Start receiving data from UART receiver.
  int32_t                (*GetTxCount)      (void);                                    ///< Pointer to \ref VOR_UART_GetTxCount : Get Tx data count.
  int32_t                (*GetRxCount)      (void);                                    ///< Pointer to \ref VOR_UART_GetRxCount : Get Rx data count.
  int32_t                (*Control)         (uint32_t control, uint32_t arg);          ///< Pointer to \ref VOR_UART_Control : Control UART Interface.
  VOR_UART_STATUS        (*GetStatus)       (void);                                    ///< Pointer to \ref VOR_UART_GetStatus : Get UART status.
} VOR_DRIVER_UART;

extern VOR_DRIVER_UART Driver_UART1;
extern VOR_DRIVER_UART Driver_UART0;

#endif /* __DRIVER_UART_H */
/** @}*/


