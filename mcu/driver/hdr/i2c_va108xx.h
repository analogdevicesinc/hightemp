/***************************************************************************************
 * @file     i2c_va108xx.h
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
#ifndef __I2C_VA108XX_H
#define __I2C_VA108XX_H

#include <stdint.h>
#include <driver_i2c.h>
#include "va108xx.h"
#include "reb_board.h"

#define VOR_I2C_DRV_VERSION VOR_DRIVER_VERSION_MAJOR_MINOR(1,00)

// Driver Version
static const VOR_DRIVER_VERSION i2c_driver_version = { VOR_I2C_API_VERSION, VOR_I2C_DRV_VERSION };

/* I2C Driver state flags */
#define I2C_FLAG_INIT       (1 << 0)        // Driver initialized

#define I2C_STD_SPEED_MULTI 20
#define I2C_FAST_SPEED_MULT 25
#define I2C_STD_SPEED_CLK   100000
#define I2C_FAST_SPEED_CLK  400000
#define I2C_MODE_MASTER     1
#define I2C_MODE_SLAVE      0

#define I2C_DIR_TRANSMIT     0
#define I2C_DIR_RECEIVE      1

#define I2C_ADDR_DIR        0
#define I2C_ADDR_BITS       1
#define I2C_ADDR_MODE       15

#define I2C_CMD_START    1
#define I2C_CMD_STOP     2
#define I2C_CMD_CANCEL   4

#define I2C_MAX_TXDATA  128
#define I2C_MAX_RXDATA  128


#define I2C0_CLK_EN_MASK  (1<<16)
#define I2C1_CLK_EN_MASK  (1<<17)

union tmconfig_t {
  VOR_I2C_TMCONF        conf;
  uint32_t              val;         
};

/* I2C Control Information */
typedef struct {
  VOR_I2C_SignalEvent_t cb_event;           // Event callback
  VOR_I2C_STATUS        status;             // Status 
  uint32_t              flags;              // Flags
  uint32_t              poll_mode;          // Polling/Interrupt Mode
  uint32_t              poll_tout;          // Polling Mode Timeout
  uint32_t              speed;              // I2C speed mode
  uint32_t              slv_addr;           // Own address
  uint32_t              addr_mode;          // 10bit/7bit address mode
  uint32_t              loopback;           // Loopback mode
  uint32_t              txfemd;             // Transmit Empty FIFO Mode
  uint32_t              rxffmd;             // Receive Full FIFO Mode
  uint32_t              algfilter;          // Analog filter
  uint32_t              digfilter;          // Digital filter
  uint32_t              rxfifotrg;          // Rx fifo trigger
  uint32_t              txfifotrg;          // Tx fifo trigger
  uint32_t              clktolimit;         // Low clock limit
  union tmconfig_t      tmconfig;           // Timing configuration
  uint32_t              txcnt;              // Transmit data count
  uint32_t              rxcnt;              // Receive data count
  uint32_t              txidx;              // Current TX index
  uint32_t              rxidx;              // Current RX index
  uint8_t               txdata[I2C_MAX_TXDATA];
  uint8_t               *rxdata;
} I2C_INFO;

/* I2C Resource Definition */
typedef struct {
  uint32_t                    i2cId;        // I2c instance identifier 
  VOR_I2C_PERIPHERAL_Type     *reg;         // Pointer to GPIO peripheral
  VOR_DRIVER_I2C              *drv;         // I2C driver functions
  I2C_INFO                    info;         // Run-Time Information
  uint32_t                    i2cClkEn;     // I2c Peripheral Clock enable Mask 
} I2C_RESOURCES;



/**
  \fn          VOR_DRIVER_VERSION I2Cx_GetVersion (void)
  \brief       Get driver version.
  \return      \ref VOR_DRIVER_VERSION
*/
VOR_DRIVER_VERSION VOR_I2Cx_GetVersion (void);

/**
  \fn          int32_t VOR_I2C_Initialize (VOR_I2C_SignalEvent_t cb_event,
                                        I2C_RESOURCES         *i2c)
  \brief       Initialize I2C Interface.
  \param[in]   cb_event  Pointer to \ref VOR_I2C_SignalEvent
  \param[in]   i2c   Pointer to I2C resources
  \return      \ref execution_status
*/
int32_t VOR_I2C_Initialize (VOR_I2C_SignalEvent_t cb_event, I2C_RESOURCES *i2c);

/**
  \fn          int32_t VOR_I2C_Uninitialize (I2C_RESOURCES *i2c)
  \brief       De-initialize I2C Interface.
  \param[in]   i2c   Pointer to I2C resources
  \return      \ref execution_status
*/
int32_t VOR_I2C_Uninitialize (I2C_RESOURCES *i2c);

/**
  \fn          int32_t VOR_I2C_Control (uint32_t       control,
                                     uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  \brief       Control I2C Interface.
  \param[in]   control  operation
  \param[in]   arg      argument of operation (optional)
  \param[in]   i2c      pointer to I2C resources
  \return      \ref execution_status
*/
int32_t VOR_I2C_Control (uint32_t control, uint32_t arg, I2C_RESOURCES *i2c);

/**
  \fn          VOR_I2C_STATUS VOR_I2C_GetStatus (I2C_RESOURCES *I2C)
  \brief       Get I2C status.
  \param[in]   I2C     Pointer to I2C resources
  \return      I2C status \ref VOR_I2C_STATUS
*/
VOR_I2C_STATUS VOR_I2C_GetStatus (I2C_RESOURCES *i2c);

/**
  \fn          int32_t VOR_I2C_MasterTransmit (uint32_t       addr,
                                            const uint8_t *data,
                                            uint32_t       num,
                                            bool           xfer_pending,
                                            I2C_RESOURCES *i2c)
  \brief       Start transmitting data as I2C Master.
  \param[in]   addr          Slave address (7-bit or 10-bit)
  \param[in]   data          Pointer to buffer with data to transmit to I2C Slave
  \param[in]   num           Number of data bytes to transmit
  \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
  \param[in]   i2c           Pointer to I2C resources
  \return      \ref execution_status
*/
int32_t VOR_I2C_MasterTransmit (uint32_t       addr,
                                    const uint8_t *data,
                                    uint32_t       num,
                                    bool           xfer_pending,
                                    I2C_RESOURCES *i2c);
																		
/**
  \fn          int32_t VOR_I2C_MasterReceive (uint32_t       addr,
                                           uint8_t       *data,
                                           uint32_t       num,
                                           bool           xfer_pending,
                                           I2C_RESOURCES *i2c)
  \brief       Start receiving data as I2C Master.
  \param[in]   addr          Slave address (7-bit or 10-bit)
  \param[out]  data          Pointer to buffer for data to receive from I2C Slave
  \param[in]   num           Number of data bytes to receive
  \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
  \param[in]   i2c           Pointer to I2C resources
  \return      \ref execution_status
*/
int32_t VOR_I2C_MasterReceive (uint32_t       addr,
                                   uint8_t       *data,
                                   uint32_t       num,
                                   bool           xfer_pending,
                                   I2C_RESOURCES *i2c);

/**
  \fn          int32_t VOR_I2C_SlaveTransmit (const uint8_t *data,
                                           uint32_t       num,
                                           I2C_RESOURCES *i2c)
  \brief       Start transmitting data as I2C Slave.
  \param[in]   data  Pointer to buffer with data to transmit to I2C Master
  \param[in]   num   Number of data bytes to transmit
  \param[in]   i2c   Pointer to I2C resources
  \return      \ref execution_status
*/
int32_t VOR_I2C_SlaveTransmit (const uint8_t *data,
                                   uint32_t       num,
                                   I2C_RESOURCES *i2c);

/**
  \fn          int32_t VOR_I2C_SlaveReceive (uint8_t       *data,
                                          uint32_t       num,
                                          I2C_RESOURCES *i2c)
  \brief       Start receiving data as I2C Slave.
  \param[out]  data  Pointer to buffer for data to receive from I2C Master
  \param[in]   num   Number of data bytes to receive
  \param[in]   i2c   Pointer to I2C resources
  \return      \ref execution_status
*/
int32_t VOR_I2C_SlaveReceive (uint8_t       *data,
                                  uint32_t       num,
                                  I2C_RESOURCES *i2c);

/**
  \fn          void VOR_I2C_MasterHandler (I2C_RESOURCES *i2c)
  \brief       I2C Master state event handler.
  \param[in]   i2c  Pointer to I2C resources
  \return      I2C event notification flags
*/
void VOR_I2C_MasterHandler (I2C_RESOURCES *i2c);

/**
  \fn          void VOR_I2C_SlaveHandler (I2C_RESOURCES *i2c)
  \brief       I2C Slave event handler.
  \param[in]   i2c  Pointer to I2C resources
  \return      I2C event notification flags
*/
void VOR_I2C_SlaveHandler (I2C_RESOURCES *i2c);
/**
  \fn          int32_t VOR_I2C_GetDataCount (I2C_RESOURCES *i2c)
  \brief       Get transferred data count.
  \return      number of data bytes transferred; 
*/
int32_t VOR_I2C_GetDataCount (I2C_RESOURCES *i2c);

#endif /* __I2C_VA108XX_H */

