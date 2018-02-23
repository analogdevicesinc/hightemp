/***************************************************************************************
 * @file     driver_spi.h
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
/** \addtogroup SPI
 *  @{
 */
#ifndef __DRIVER_SPI_H
#define __DRIVER_SPI_H

#include "driver_common.h"

#define VOR_SPI_API_VERSION VOR_DRIVER_VERSION_MAJOR_MINOR(1,00)  /* API version */


/****** SPI Control Codes *****/

/*CTRL arg inputs  */
#define VOR_SPI_DRV_POLL		  					0
#define VOR_SPI_POLL_TOUT		  					1
#define VOR_SPI_CTRL_CLKMODE          	2
#define VOR_SPI_CTRL_MS                	3
#define VOR_SPI_CTRL_SOD               	4
#define VOR_SPI_CTRL_LOOPBACK          	5
#define VOR_SPI_CTRL_BLOCKMODE          6	
#define VOR_SPI_CTRL_DATA_WIDTH      		7
#define VOR_SPI_CTRL_CS             		8
#define VOR_SPI_CTRL_CLKRATE         	  9
#define VOR_SPI_CTRL_TX_FIFO_TRG     	  10
#define VOR_SPI_CTRL_RX_FIFO_TRG     	  11
#define VOR_SPI_ABORT_TRANSFER          12      ///< Abort Master/Slave Transmit/Receive
#define VOR_SPI_CTRL_INTERFACE					13

/* CTRL arg codes values */
#define VOR_SPI_CTRL_ENABLE        	  1
#define VOR_SPI_CTRL_DISABLE       	  0

/* SPI Chip Select Value */
#define VOR_SPI_SLAVE_SELECT_VAL1  0x1		
#define VOR_SPI_SLAVE_SELECT_VAL2  0x2		
#define VOR_SPI_SLAVE_SELECT_VAL3  0x3		
#define VOR_SPI_SLAVE_SELECT_VAL4  0x4		
#define VOR_SPI_SLAVE_SELECT_VAL5  0x5		
#define VOR_SPI_SLAVE_SELECT_VAL6  0x6		
#define VOR_SPI_SLAVE_SELECT_VAL7  0x7		

#define VOR_SPI_CLKMODE_SPH0_SPO0  0
#define VOR_SPI_CLKMODE_SPH0_SPO1  1
#define VOR_SPI_CLKMODE_SPH1_SPO0  2
#define VOR_SPI_CLKMODE_SPH1_SPO1  3

#define VOR_SPI_FIFO_CLR_RXFIFO   0
#define VOR_SPI_FIFO_CLR_TXFIFO   1

#define VOR_SPI_MASTER        	  (0)
#define VOR_SPI_SLAVE          	  (1)

/****** SPI Slave Select Signal definitions *****/
#define VOR_SPI_SS_INACTIVE              0                                  ///< SPI Slave Select Signal Inactive
#define VOR_SPI_SS_ACTIVE                1                                  ///< SPI Slave Select Signal Active
#define VOR_SPI_CNTR_ENABLE							 1


/****** SPI specific error codes *****/
#define VOR_SPI_ERROR_MODE              (VOR_DRIVER_ERROR_SPECIFIC - 1)     ///< Specified Mode not supported
#define VOR_SPI_ERROR_FRAME_FORMAT      (VOR_DRIVER_ERROR_SPECIFIC - 2)     ///< Specified Frame Format not supported
#define VOR_SPI_ERROR_DATA_BITS         (VOR_DRIVER_ERROR_SPECIFIC - 3)     ///< Specified number of Data bits not supported
#define VOR_SPI_ERROR_BIT_ORDER         (VOR_DRIVER_ERROR_SPECIFIC - 4)     ///< Specified Bit order not supported
#define VOR_SPI_ERROR_SS_MODE           (VOR_DRIVER_ERROR_SPECIFIC - 5)     ///< Specified Slave Select Mode not supported


/*SPI Status */
typedef struct _VOR_SPI_STATUS {
  uint32_t TFE:1;
	uint32_t TNF:1;
	uint32_t RNE:1;
	uint32_t RFF:1;
	uint32_t BUSY:1;
	uint32_t RXDATAFIRST:1;
	uint32_t RXTRIGGER:1;
	uint32_t TXTRIGGER:1;
}VOR_SPI_STATUS;
	
/****** SPI Event *****/
#define VOR_SPI_TRANSFER_DONE                (1UL << 0)  ///< Receive Overrun of the Receive FIFO
#define VOR_SPI_RECEIVE_OVERRUN_RECEIVE_FIFO (1UL << 1)  ///< Receive Overrun of the Receive FIFO

typedef void (*VOR_SPI_SignalEvent_t) (uint32_t event);  ///< Pointer to /ref VOR_SPI_SignalEvent : Signal SPI Event.

typedef struct _VOR_SPI_IRQEVENT {
  uint32_t RORIM:1;
  uint32_t RTIM:1;
	uint32_t RXIM:1;
  uint32_t TXIM:1;
}VOR_SPI_IRQEVENT;

/* Current driver status flag definition */
#define SPI_INITIALIZED                   (1    << 0)       // SPI initialized
#define SPI_POWERED                       (1    << 1)       // SPI powered on
#define SPI_CONFIGURED                    (1    << 2)       // SPI configured
#define SPI_DATA_LOST                     (1    << 3)       // SPI data lost occurred
#define SPI_MODE_FAULT                    (1    << 4)       // SPI mode fault occurred


/****** SPI specific error codes *****/
#define VOR_SPI_ERROR_MODE              (VOR_DRIVER_ERROR_SPECIFIC - 1)     ///< Specified Mode not supported
#define VOR_SPI_ERROR_FRAME_FORMAT      (VOR_DRIVER_ERROR_SPECIFIC - 2)     ///< Specified Frame Format not supported
#define VOR_SPI_ERROR_DATA_BITS         (VOR_DRIVER_ERROR_SPECIFIC - 3)     ///< Specified number of Data bits not supported
#define VOR_SPI_ERROR_BIT_ORDER         (VOR_DRIVER_ERROR_SPECIFIC - 4)     ///< Specified Bit order not supported
#define VOR_SPI_ERROR_SS_MODE           (VOR_DRIVER_ERROR_SPECIFIC - 5)     ///< Specified Slave Select Mode not supported
#define VOR_SPI_ERROR_CTRL              (VOR_DRIVER_ERROR_SPECIFIC - 6)     ///< Specified CTRL data wrong
#define VOR_SPI_ERROR_CTRL_ARG          (VOR_DRIVER_ERROR_SPECIFIC - 7)     ///< Specified CTRL ARG data wrong
#define VOR_SPI_ERROR_POLL_TOUT         (VOR_DRIVER_ERROR_SPECIFIC - 8)     ///< Specified Polling Timeout

// Function documentation
/**
  \fn          VOR_DRIVER_VERSION VOR_SPI_GetVersion (void)
  \brief       Get driver version.
  \return      \ref VOR_DRIVER_VERSION

  \fn          int32_t VOR_SPI_Initialize (VOR_SPI_SignalEvent_t cb_event)
  \brief       Initialize SPI Interface.
  \param[in]   cb_event  Pointer to \ref VOR_SPI_SignalEvent
  \return      \ref execution_status

  \fn          int32_t VOR_SPI_Uninitialize (void)
  \brief       De-initialize SPI Interface.
  \return      \ref execution_status

  \fn          int32_t VOR_SPI_Send (uint8_t *data, uint32_t num)
  \brief       Start sending data to SPI transmitter.
  \param[in]   data  Pointer to buffer with data to send to SPI transmitter
  \param[in]   num   Number of data items to send
  \return      \ref execution_status

  \fn          int32_t VOR_SPI_Receive (uint8_t *data, uint32_t num)
  \brief       Start receiving data from SPI receiver.
  \param[out]  data  Pointer to buffer for data to receive from SPI receiver
  \param[in]   num   Number of data items to receive
  \return      \ref execution_status

  \fn          int32_t VOR_SPI_Transfer (uint8_t *dataIn, uint8_t *dataOut,uint32_t num,
																			SPI_RESOURCES *SPI);
  \brief       Receive data from other port
  \param[in]   dataIn  data receive buffer
  \param[in]   dataOut data send buffer
  \param[in]   num     number of data bytes  
  \param[in]   SPI     Pointer to SPI resources
  \return      \ref execution_status

  \fn          int32_t VOR_SPI_Control (uint32_t control, uint32_t arg)
  \brief       Control SPI Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      common \ref execution_status and driver specific \ref spi_execution_status

  \fn          VOR_SPI_STATUS VOR_SPI_GetStatus (void)
  \brief       Get SPI status.
  \return      SPI status \ref VOR_SPI_STATUS

  \fn          uint32_t VOR_SPI_GetTxCount (void)
  \brief       Get transmitted data count.
  \return      number of data items transmitted

  \fn          uint32_t VOR_SPI_GetRxCount (void)
  \brief       Get received data count.
  \return      number of data items received


  \fn          void VOR_SPI_SignalEvent (uint32_t event)
  \brief       Signal SPI Events.
  \param[in]   event \ref SPI_events notification mask
  \return      none
*/


/**
\brief Access structure of the SPI Driver.
*/
typedef struct _VOR_DRIVER_SPI {
  VOR_DRIVER_VERSION   (*GetVersion)      (void);                             ///< Pointer to \ref VOR_SPI_GetVersion : Get driver version.
  int32_t              (*Initialize)      (VOR_SPI_SignalEvent_t cb_event);   ///< Pointer to \ref VOR_SPI_Initialize : Initialize SPI Interface.
  int32_t              (*Uninitialize)    (void);                             ///< Pointer to \ref VOR_SPI_Uninitialize : De-initialize SPI Interface.
  int32_t              (*Send)            (uint8_t *data, uint32_t num);      ///< Pointer to \ref VOR_SPI_Send : Start sending data to SPI Interface.
  int32_t              (*Receive)         (uint8_t *data, uint32_t num);      ///< Pointer to \ref VOR_SPI_Receive : Start receiving data from SPI Interface.
  int32_t              (*Transfer)        (uint8_t *dataIn,uint8_t *dataOut , uint32_t num);      ///< Pointer to \ref VOR_SPI_Transfer : Start transfering data from SPI Interface.
  int32_t              (*Control)         (uint32_t control, uint32_t arg);   ///< Pointer to \ref VOR_SPI_Control : Control SPI Interface.
  VOR_SPI_STATUS       (*GetStatus)       (void);                             ///< Pointer to \ref VOR_SPI_GetStatus : Get SPI status.
  uint32_t             (*GetTxCount)      (void);                             ///< Pointer to \ref VOR_SPI_GetTxCount : Get transmitted data count.
  uint32_t             (*GetRxCount)      (void);                             ///< Pointer to \ref VOR_SPI_GetRxCount : Get received data count.
} VOR_DRIVER_SPI;


extern VOR_DRIVER_SPI Driver_SPI0;
extern VOR_DRIVER_SPI Driver_SPI1;
extern VOR_DRIVER_SPI Driver_SPI2;

#endif /* __DRIVER_SPI_H */
/** @}*/


