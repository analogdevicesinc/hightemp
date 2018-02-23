/***************************************************************************************
 * @file     driver_i2c.h
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
/** @file */ 
/** \addtogroup I2C 
 *  @{
 */
#ifndef __DRIVER_I2C_H
#define __DRIVER_I2C_H

#include "driver_common.h"

#define VOR_I2C_API_VERSION VOR_DRIVER_VERSION_MAJOR_MINOR(1,00)  ///< API version 

/**
 \brief I2C Control Codes
*/ 
#define VOR_I2C_DRV_POLL                (0x01)      ///< Set Driver Mode Polling/ Interrupt (default)
#define VOR_I2C_POLL_TOUT               (0x02)      ///< Set Polling mode timeout
#define VOR_I2C_BUS_SPEED               (0x03)      ///< Set Bus Speed; arg = speed mode
#define VOR_I2C_OWN_ADDRESS             (0x04)      ///< Set Own Slave Address; arg = address 
#define VOR_I2C_ADDRESS_10BIT           (0x05)      ///< 10 bit address mode
#define VOR_I2C_FIFO_CLR                (0x06)      ///< Clear Tx and Rx FIFO
#define VOR_I2C_LOOPBACK                (0x07)      ///< Enable/Disable Internal Loopback
#define VOR_I2C_ABORT_TRANSFER          (0x08)      ///< Abort Master/Slave Transmit/Receive
#define VOR_I2C_TXFEMD_MODE             (0x09)      ///< Set Transmit on Empty FIFO Mode                
#define VOR_I2C_RXFFMD_MODE             (0x0a)      ///< Set Receive on Full FIFO Mode                
#define VOR_I2C_ALGFILTER               (0x0b)      ///< Enable/Disable Analog glitch filtering
#define VOR_I2C_DIGFILTER               (0x0c)      ///< Enable/Disable Digital glitch filtering
#define VOR_I2C_RXFIFOIRQTRG            (0x0d)      ///< Set Rx Fifo Trigger
#define VOR_I2C_TXFIFOIRQTRG            (0x0e)      ///< Set Tx Fifo Trigger
#define VOR_I2C_CLKTOLIMIT              (0x0f)      ///< Low clock limit
#define VOR_I2C_TMCONFIG                (0x10)      ///< Set Timing Config register

/**
 \brief I2C Control Args 
*/
#define VOR_I2C_CTRL_ENABLE             (0x01)
#define VOR_I2C_CTRL_DISABLE            (0x00)
#define VOR_I2C_BUS_SPEED_STANDARD      (0x01)      ///< Standard Speed (100kHz)
#define VOR_I2C_BUS_SPEED_FAST          (0x02)      ///< Fast Speed     (400kHz)
#define VOR_I2C_BUS_SPEED_FAST_PLUS     (0x03)      ///< Fast+ Speed    (  1MHz)
#define VOR_I2C_BUS_SPEED_HIGH          (0x04)      ///< High Speed     (3.4MHz)
#define VOR_I2C_FIFOCLR_RX              (0x01)      ///< Rx FIFO clear
#define VOR_I2C_FIFOCLR_TX              (0x02)      ///< Tx FIFO clear 
#define VOR_I2C_STALL                   (0x00)      ///< I2C Clock is stretched until data is available.  
#define VOR_I2C_END                     (0x01)      ///< Stop Tx when Tx FIFO is empty
#define VOR_I2C_NACK                    (0x01)      ///< Stop Rx and Send NACK when Rx FIFO is empty

typedef struct _VOR_I2C_TMCONF {
  uint32_t Tr               : 4;        ///< Tr time
  uint32_t Tf               : 4;        ///< Tf time
  uint32_t Thigh            : 4;        ///< Thigh time
  uint32_t Tlow             : 4;        ///< Tlow time
  uint32_t TSUsto           : 4;        ///< Tsu;sto time
  uint32_t TSUsta           : 4;        ///< Tsu;sta time
  uint32_t THDsta           : 4;        ///< Thd;sta time
  uint32_t Tbuf             : 4;        ///< Tbuf time
} VOR_I2C_TMCONF;

/**
 \brief I2C specific error codes 
*/
#define VOR_I2C_ERROR_CTRL                (VOR_DRIVER_ERROR_SPECIFIC - 3)     ///< CONTROL MODE not supported
#define VOR_I2C_ERROR_CTRL_ARG            (VOR_DRIVER_ERROR_SPECIFIC - 4)     ///< CONTROL Args not supported
#define VOR_I2C_ERROR_MULTIPLEINIT        (VOR_DRIVER_ERROR_SPECIFIC - 5)     ///< I2C already initialized
#define VOR_I2C_ERROR_ADDRESS_NACK        (VOR_DRIVER_ERROR_SPECIFIC - 6)     ///< Address not acknowledged from Slave
#define VOR_I2C_ERROR_ARBITRATION_LOST    (VOR_DRIVER_ERROR_SPECIFIC - 7)     ///< Master lost arbitration
#define VOR_I2C_ERROR_TRANSFER_INCOMPLETE (VOR_DRIVER_ERROR_SPECIFIC - 8)     ///< Transfer Incomplete
#define VOR_I2C_ERROR_POLL_TOUT           (VOR_DRIVER_ERROR_SPECIFIC - 9)     ///< Poll Time out error

/**
	\brief I2C Status
*/
typedef struct _VOR_I2C_STATUS {
  uint32_t i2cIdle          : 1;        ///< Bus Idle
  uint32_t Idle             : 1;        ///< Controller Idle
  uint32_t waiting          : 1;        ///< Controller is waiting
  uint32_t stalled          : 1;        ///< Controller is stalled
  uint32_t arb_lost         : 1;        ///< Lost arbitration 
  uint32_t nackAddr         : 1;        ///< Address failed 
  uint32_t nackData         : 1;        ///< Receive negative Ack 
  uint32_t rxNEmpty         : 1;        ///< Receive FIFO is not empty 
  uint32_t rxFull           : 1;        ///< Receive FIFO is full
  uint32_t rxFifoTrg        : 1;        ///< Receive FIFO >= Trigger level
  uint32_t txEmpty          : 1;        ///< Transmit FIFO is empty 
  uint32_t txNFull          : 1;        ///< Transmit FIFO is not full
  uint32_t txFifoTrg        : 1;        ///< Transmit FIFO >= Trigger level
  uint32_t rawSDA           : 1;        ///< Raw I2C Data value
  uint32_t rawSCL           : 1;        ///< Raw I2C Clock value
  uint32_t mode             : 1;        ///< Mode: 0=Slave, 1=Master
  uint32_t direction        : 1;        ///< Direction: 0=Transmitter, 1=Receiver
} VOR_I2C_STATUS;

/**
 \brief I2C Event 
*/
#define VOR_I2C_EVENT_TRANSFER_DONE       (1UL << 0)  	///< Master/Slave Transmit/Receive finished
#define VOR_I2C_EVENT_TRANSFER_INCOMPLETE (1UL << 1)  	///< Master/Slave Transmit/Receive incomplete transfer
#define VOR_I2C_EVENT_SLAVE_TRANSMIT      (1UL << 2)  	///< Slave Transmit operation requested
#define VOR_I2C_EVENT_SLAVE_RECEIVE       (1UL << 3)  	///< Slave Receive operation requested
#define VOR_I2C_EVENT_ADDRESS_NACK        (1UL << 4)  	///< Address not acknowledged from Slave
#define VOR_I2C_EVENT_ARBITRATION_LOST    (1UL << 5)  	///< Master lost arbitration
#define VOR_I2C_EVENT_TXOVERFLOW          (1UL << 6)  	///< Tx FIFO Overflow
#define VOR_I2C_EVENT_RXOVERFLOW          (1UL << 7)  	///< Rx FIFO Overflow
#define VOR_I2C_EVENT_TXREADY             (1UL << 8)  	///< Tx FIFO is ready for more data
#define VOR_I2C_EVENT_RXREADY             (1UL << 9) 		///< Rx FIFO has data ready 
#define VOR_I2C_EVENT_TXEMPTY             (1UL << 10)  	///< Tx FIFO is empty
#define VOR_I2C_EVENT_RXFULL              (1UL << 11)  	///< Rx FIFO is full
#define VOR_I2C_EVENT_CLKLOTO             (1UL << 12)  	///< Clock Low timeout


/**
	\fn          VOR_DRIVER_VERSION VOR_I2C_GetVersion (void)
	\brief       Get driver version.
	\return      \ref VOR_DRIVER_VERSION

	\fn          int32_t VOR_I2C_Initialize (VOR_I2C_SignalEvent_t cb_event)
	\brief       Initialize I2C Interface.
	\param[in]   cb_event  Pointer to \ref VOR_I2C_SignalEvent
	\return      \ref execution_status

	\fn          int32_t VOR_I2C_Uninitialize (void)
	\brief       De-initialize I2C Interface.
	\return      \ref execution_status

	\fn          int32_t VOR_I2C_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
	\brief       Start transmitting data as I2C Master.
	\param[in]   addr          Slave address (7-bit or 10-bit)
	\param[in]   data          Pointer to buffer with data to transmit to I2C Slave
	\param[in]   num           Number of data bytes to transmit
	\param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
	\return      \ref execution_status

	\fn          int32_t VOR_I2C_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
	\brief       Start receiving data as I2C Master.
	\param[in]   addr          Slave address (7-bit or 10-bit)
	\param[out]  data          Pointer to buffer for data to receive from I2C Slave
	\param[in]   num           Number of data bytes to receive
	\param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
	\return      \ref execution_status

	\fn          int32_t VOR_I2C_SlaveTransmit (const uint8_t *data, uint32_t num)
	\brief       Start transmitting data as I2C Slave.
	\param[in]   data  Pointer to buffer with data to transmit to I2C Master
	\param[in]   num   Number of data bytes to transmit
	\return      \ref execution_status

	\fn          int32_t VOR_I2C_SlaveReceive (uint8_t *data, uint32_t num)
	\brief       Start receiving data as I2C Slave.
	\param[out]  data  Pointer to buffer for data to receive from I2C Master
	\param[in]   num   Number of data bytes to receive
	\return      \ref execution_status

	\fn          int32_t VOR_I2C_GetDataCount (void)
	\brief       Get transferred data count.
	\return      number of data bytes transferred; -1 when Slave is not addressed by Master

	\fn          int32_t VOR_I2C_Control (uint32_t control, uint32_t arg)
	\brief       Control I2C Interface.
	\param[in]   control  Operation
	\param[in]   arg      Argument of operation (optional)
	\return      \ref execution_status

	\fn          VOR_I2C_STATUS VOR_I2C_GetStatus (void)
	\brief       Get I2C status.
	\return      I2C status \ref VOR_I2C_STATUS

	\fn          void VOR_I2C_SignalEvent (uint32_t event)
	\brief       Signal I2C Events.
	\param[in]   event  \ref I2C_events notification mask
*/

typedef void (*VOR_I2C_SignalEvent_t) (uint32_t event);  ///< Pointer to \ref VOR_I2C_SignalEvent : Signal I2C Event.

/**
	\brief Access structure of the I2C Driver.
*/
typedef struct _VOR_DRIVER_I2C {
  VOR_DRIVER_VERSION   (*GetVersion)     (void);                                                                ///< Pointer to \ref VOR_I2C_GetVersion : Get driver version.
  int32_t              (*Initialize)     (VOR_I2C_SignalEvent_t cb_event);                                      ///< Pointer to \ref VOR_I2C_Initialize : Initialize I2C Interface.
  int32_t              (*Uninitialize)   (void);                                                                ///< Pointer to \ref VOR_I2C_Uninitialize : De-initialize I2C Interface.
  int32_t              (*MasterTransmit) (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending); ///< Pointer to \ref VOR_I2C_MasterTransmit : Start transmitting data as I2C Master.
  int32_t              (*MasterReceive)  (uint32_t addr,       uint8_t *data, uint32_t num, bool xfer_pending); ///< Pointer to \ref VOR_I2C_MasterReceive : Start receiving data as I2C Master.
  int32_t              (*SlaveTransmit)  (               const uint8_t *data, uint32_t num);                    ///< Pointer to \ref VOR_I2C_SlaveTransmit : Start transmitting data as I2C Slave.
  int32_t              (*SlaveReceive)   (                     uint8_t *data, uint32_t num);                    ///< Pointer to \ref VOR_I2C_SlaveReceive : Start receiving data as I2C Slave.
  int32_t              (*GetDataCount)   (void);                                                                ///< Pointer to \ref VOR_I2C_GetDataCount : Get transferred data count.
  int32_t              (*Control)        (uint32_t control, uint32_t arg);                                      ///< Pointer to \ref VOR_I2C_Control : Control I2C Interface.
  VOR_I2C_STATUS       (*GetStatus)      (void);                                                                ///< Pointer to \ref VOR_I2C_GetStatus : Get I2C status.
} VOR_DRIVER_I2C;

extern VOR_DRIVER_I2C Driver_I2C0;
extern VOR_DRIVER_I2C Driver_I2C1;

#endif /* __DRIVER_I2C_H */
/** @}*/


