/***************************************************************************************
 * @file     spi_va108xx.h
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
#ifndef __SPI_VA108XX_H
#define __SPI_VA108XX_H

#include <stdint.h>
#include "driver_spi.h"
#include "va108xx.h"
#include "reb_board.h"

#define VOR_SPI_DRV_VERSION    VOR_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

#define MAX_SPI_CNT 3

/* Driver Version */
static const VOR_DRIVER_VERSION DriverVersion = {
    VOR_SPI_API_VERSION,
    VOR_SPI_DRV_VERSION
};

/* SPI Driver state flags */
#define SPI_FLAG_INIT       (1 << 0)        // Driver initialized

/****** SPI Control Codes *****/

#define SPI_MODE_MASTER     0
#define SPI_MODE_SLAVE      1

#define SPI_MAX_TXDATA      128

/*CTRL0 register */
#define SPI_CTRL0_SIZE_POS    (     0)
#define SPI_CTRL0_SIZE_MSK    (0x0F << SPI_CTRL0_SIZE_POS)
#define SPI_CTRL0_SPO   			(1 << 6)
#define SPI_CTRL0_SPH   			(1 << 7)

#define SPI_CTRL0_SIZE   			0
#define SPI_CTRLO_SIZE_4BITS 	0x3
#define SPI_CTRLO_SIZE_8BITS 	0x7
#define SPI_CTRLO_SIZE_16BITS 0xF

/*Data register */
#define SPI_DATA_VALUE    				(     0)
#define SPI_DATA_VALUE_MSK    		(0xFF << SPI_DATA_VALUE)
#define SPI_DATA_BMSKIPDATA   	  (1 << 30)
#define SPI_DATA_BMSTART_BMSTOP   (1 << 31)

/*CLKPRESCALE REGISTER */
#define SPI_CLKPRESCALE_VALUE    				(     0)
#define SPI_CLKPRESCALE_VALUE_MSK    		(0x7 << SPI_CLKPRESCALE_VALUE)

/*IRQ ENB REGISTER */
#define IRQ_ENB_RORIM 				(1 << 0)  
#define IRQ_ENB_RTIM 					(1 << 1)  
#define IRQ_END_RXIM 					(1 << 2) 
#define IRQ_ENB_TXIM 					(1 << 3) 

/*IRQ RAW REGISTER */
#define IRQ_RAW_RORIM 				(1 << 0)  
#define IRQ_RAW_RTIM 					(1 << 1)  
#define IRQ_RAW_RXIM 					(1 << 2) 
#define IRQ_RAW_TXIM 					(1 << 3) 


/*IRQ END REGISTER */
#define IRQ_END_RORIM 				(1 << 0)  
#define IRQ_END_RTIM 					(1 << 1)  
#define IRQ_END_RXIM 					(1 << 2) 
#define IRQ_END_TXIM 					(1 << 3) 

/*IRQ CLR REGISTER */
#define IRQ_CLR_RORIM 				(1 << 0)  
#define IRQ_CLR_RTIM 					(1 << 1)  

/*RXFIFOIRQTRG REGISTER */
#define SPI_RXFIFOIRQTRG_VALUE    				(     0)
#define SPI_RXFIFOIRQTRG_VALUE_MSK    		(0x0F << SPI_RXFIFOIRQTRG_VALUE)

/*TXFIFOIRQTRG REGISTER */
#define SPI_TXFIFOIRQTRG_VALUE    				(     0)
#define SPI_TXFIFOIRQTRG_VALUE_MSK    		(0x0F << SPI_TXFIFOIRQTRG_VALUE)

/*FIFO_CLR Register */
#define SPI_FIFO_CLR_RXFIFO   (1 << 0)
#define SPI_FIFO_CLR_TXFIFO   (1 << 1)

/*STATE Register */
#define SPI_STATE_RXSTATE    						(     0)
#define SPI_STATE_RXSTATE_MSK	      		(0x000F << SPI_STATE_RXSTATE)
#define SPI_STATE_RXFIFO     						(     0)
#define SPI_STATE_RXFIFO_MSK	      		(0x00F0 << SPI_STATE_RXSTATE)
#define SPI_STATE_ZERO      						(     0)
#define SPI_STATE_ZERO_MSK	      		  (0x0F00 << SPI_STATE_ZERO)
#define SPI_STATE_TXFIFO     						(     0)
#define SPI_STATE_TXFIFO_MSK	      		(0xF000 << SPI_STATE_TXFIFO)


/****** SPI Slave Select Signal definitions *****/
#define VOR_SPI_SS_INACTIVE              0                                  ///< SPI Slave Select Signal Inactive
#define VOR_SPI_SS_ACTIVE                1                                  ///< SPI Slave Select Signal Active


/*SPI Status */
#define SPI_STATUS_TFE 					(1 << 0)
#define SPI_STATUS_TNF 					(1 << 1)
#define SPI_STATUS_RNE					(1 << 2)
#define SPI_STATUS_RFF					(1 << 3)
#define SPI_STATUS_BUSY					(1 << 4)
#define SPI_STATUS_RXDATAFIRST	(1 << 5)
#define SPI_STATUS_RXTRIGGER 		(1 << 6)
#define SPI_STATUS_TXTRIGGER		(1 << 7)


#define SPI0_CLK_EN_MASK  (1<<4)
#define SPI1_CLK_EN_MASK  (1<<5)


// SPI Information 
typedef struct _SPI_INFO {
  VOR_SPI_SignalEvent_t   cb_event;      		// Event callback
  VOR_SPI_STATUS          status;        		// Status flags
	uint32_t              flags;              // Flags
  uint32_t              poll_mode;          // Polling/Interrupt Mode
  uint32_t              poll_tout;          // Polling Mode Timeout
  uint32_t              loopback;           // Loopback mode
	uint32_t 							msmode;							// Master Slave Mode
	uint32_t 							transfer;						// transfer Mode
	uint32_t 							blockmode;					// Block Mode
	uint32_t 							mdlycap;					  // Master Delay Capture Mode
	uint32_t              sizeofwords;				// Size of Words 4,8,16 bits
	uint32_t 							chipselectaddr;			// Chip Select Address 
  uint32_t              rxfifotrger;        // Rx fifo trigger
  uint32_t              txfifotrger;        // Tx fifo trigger
  uint32_t              txcnt;              // Transmit data count
  uint32_t              rxcnt;              // Receive data count
  uint32_t              txidx;              // Current TX index
  uint32_t              rxidx;              // Current RX index
  uint8_t               txdata[SPI_MAX_TXDATA];
  uint8_t               *rxdata;
} VOR_SPI_INFO;


// SPI Resources definitions
typedef struct {
	uint32_t                        SPIId;        // SPI instance identifier 
  VOR_SPI_PERIPHERAL_Type         *reg;      		// Pointer to SPI peripheral
  VOR_DRIVER_SPI                  *drv;         // Driver Handler
  VOR_SPI_INFO                    info;         // Run-Time Information
  uint32_t                        spiClkEn;     // Spi Peripheral Clock enable Mask 
} VOR_SPI_RESOURCES;

/**
  \fn          VOR_DRIVER_VERSION VOR_SPIx_GetVersion (void)
  \brief       Get driver version.
  \return      \ref VOR_DRIVER_VERSION
*/
VOR_DRIVER_VERSION VOR_SPIx_GetVersion(void);


/*
\fn          int32_t VOR_SPI_Initialize (VOR_SPI_SignalEvent_t cb_event,
																				 SPI_RESOURCES *GPIO);
  \brief       Initialize SPI port
  \param[in]   cb_event    pointer to the VOR_SPI_SignalEvent callback function
  \param[in]   SPI     Pointer to SPI resources
  \return      \ref execution_status
*/
int32_t VOR_SPI_Initialize (VOR_SPI_SignalEvent_t, VOR_SPI_RESOURCES *);


/*
\fn          int32_t VOR_SPI_Uninitialize (SPI_RESOURCES *GPIO);
  \brief       Uninitialize SPI port
  \param[in]   SPI     Pointer to SPI resources
  \return      \ref execution_status
*/
int32_t VOR_SPI_Uninitialize (VOR_SPI_RESOURCES *);

/* 
\fn          int32_t VOR_SPI_Send (void *data, uint32_t num,
                                SPI_RESOURCES *GPIO);
  \brief       Send data from other port
  \param[in]   data    data need to send to other port   
  \param[in]   arg     number of data bytes to send  
  \param[in]   SPI     Pointer to SPI resources
  \return      \ref execution_status
*/

int32_t VOR_SPI_Send (uint8_t *, uint32_t, VOR_SPI_RESOURCES *);


/*
\fn          int32_t VOR_SPI_Receive (void *data, uint32_t num,
																			SPI_RESOURCES *GPIO);
  \brief       Receive data from other port
  \param[in]   data    data receive in bytes
  \param[in]   num     number of data bytes to receive 
  \param[in]   SPI     Pointer to SPI resources
  \return      \ref execution_status
*/	

int32_t VOR_SPI_Receive (uint8_t *, uint32_t, VOR_SPI_RESOURCES *);

/*
\fn          int32_t VOR_SPI_Transfer (uint8_t *dataIn, uint8_t *dataOut,uint32_t num,
																			SPI_RESOURCES *SPI);
  \brief       Receive data from other port
  \param[in]   dataIn  data receive buffer
  \param[in]   dataOut data send buffer
  \param[in]   num     number of data bytes  
  \param[in]   SPI     Pointer to SPI resources
  \return      \ref execution_status
*/	
int32_t VOR_SPI_Transfer(uint8_t *dataIn, uint8_t *dataOut, uint32_t num, VOR_SPI_RESOURCES *SPI);


/*
  \fn          int32_t VOR_SPI_Control (uint32_t control, uint32_t arg
                                SPI_RESOURCES *GPIO);
  \brief       Set SPI control with Control Code and Args 
  \param[in]   val      Control code 
  \param[in]   val      Control args 
  \param[in]   SPI     Pointer to SPI resources
  \return      \ref execution_status
*/

int32_t VOR_SPI_Control (uint32_t , uint32_t , VOR_SPI_RESOURCES *);

/*
The function VOR_SPI_GetStatus returns the current SPI interface status.
\fn          int32_t VOR_SPI_GetStatus (SPI_RESOURCES *GPIO);
  \brief       Get SPI status  
  \param[in]   SPI     Pointer to SPI resources
  \return      \ref VOR_SPI_STATUS
*/
VOR_SPI_STATUS VOR_SPI_GetStatus (VOR_SPI_RESOURCES *);


/**
  \fn          void VOR_SPI_IrqHandler (void)
  \brief       SPI Interrupt Handler.
  \return      none
*/
void VOR_SPI_IrqHandler (VOR_SPI_RESOURCES *);


/*
  \fn          uint32_t VOR_SPI_GetTxCount (void)
  \brief       Get transmitted data count.
  \return      number of data items transmitted
*/
uint32_t VOR_SPI_GetTxCount (VOR_SPI_RESOURCES *SPI);
/*
  \fn          uint32_t VOR_SPI_GetRxCount (void)
  \brief       Get received data count.
  \return      number of data items received
*/
uint32_t VOR_SPI_GetRxCount (VOR_SPI_RESOURCES *SPI);


#endif    /* SPI_VA108XX.H */
