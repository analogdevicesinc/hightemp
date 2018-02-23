/***************************************************************************************
 * @file     gpio_va108xx.h
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
#ifndef __GPIO_VA108XX_H
#define __GPIO_VA108XX_H

#include <stdint.h>
#include <driver_gpio.h>
#include "va108xx.h"
#include "reb_board.h"

#define VOR_GPIO_DRV_VERSION VOR_DRIVER_VERSION_MAJOR_MINOR(1,00)

#define MAX_GPIO_CNT 56
#define MAX_GPIO_PORTA 32

#define GPIO_PORTA 0
#define GPIO_PORTB 1

// Driver Version
static const VOR_DRIVER_VERSION gpio_driver_version = { VOR_GPIO_API_VERSION, VOR_GPIO_DRV_VERSION };


// GPIO identifier
typedef struct _GPIO_ID {
  uint8_t       port;
  uint8_t       pin;
} GPIO_ID;

// GPIO Information 
typedef struct _GPIO_INFO {
  VOR_GPIO_SignalEvent_t  cb_event;      // Event callback
  VOR_GPIO_STATUS         status;        // Status flags
	uint32_t                trg_mode;      // Trigger Mode
} GPIO_INFO;


// GPIO Resources definitions
typedef struct {
  GPIO_ID                     gpioId;
  VOR_GPIO_PERIPHERAL_Type    *reg;      // Pointer to GPIO peripheral
  VOR_DRIVER_GPIO             *drv;
  GPIO_INFO                   info;      // Run-Time Information
} GPIO_RESOURCES;

extern VOR_DRIVER_GPIO Driver_GPIOA_0;
extern VOR_DRIVER_GPIO Driver_GPIOA_1;
extern VOR_DRIVER_GPIO Driver_GPIOA_2;
extern VOR_DRIVER_GPIO Driver_GPIOA_3;
extern VOR_DRIVER_GPIO Driver_GPIOA_4;
extern VOR_DRIVER_GPIO Driver_GPIOA_5;
extern VOR_DRIVER_GPIO Driver_GPIOA_6;
extern VOR_DRIVER_GPIO Driver_GPIOA_7;
extern VOR_DRIVER_GPIO Driver_GPIOA_8;
extern VOR_DRIVER_GPIO Driver_GPIOA_9;
extern VOR_DRIVER_GPIO Driver_GPIOA_10;
extern VOR_DRIVER_GPIO Driver_GPIOA_11;
extern VOR_DRIVER_GPIO Driver_GPIOA_12;
extern VOR_DRIVER_GPIO Driver_GPIOA_13;
extern VOR_DRIVER_GPIO Driver_GPIOA_14;
extern VOR_DRIVER_GPIO Driver_GPIOA_15;
extern VOR_DRIVER_GPIO Driver_GPIOA_16;
extern VOR_DRIVER_GPIO Driver_GPIOA_17;
extern VOR_DRIVER_GPIO Driver_GPIOA_18;
extern VOR_DRIVER_GPIO Driver_GPIOA_19;
extern VOR_DRIVER_GPIO Driver_GPIOA_20;
extern VOR_DRIVER_GPIO Driver_GPIOA_21;
extern VOR_DRIVER_GPIO Driver_GPIOA_22;
extern VOR_DRIVER_GPIO Driver_GPIOA_23;
extern VOR_DRIVER_GPIO Driver_GPIOA_24;
extern VOR_DRIVER_GPIO Driver_GPIOA_25;
extern VOR_DRIVER_GPIO Driver_GPIOA_26;
extern VOR_DRIVER_GPIO Driver_GPIOA_27;
extern VOR_DRIVER_GPIO Driver_GPIOA_28;
extern VOR_DRIVER_GPIO Driver_GPIOA_29;
extern VOR_DRIVER_GPIO Driver_GPIOA_30;
extern VOR_DRIVER_GPIO Driver_GPIOA_31;
extern VOR_DRIVER_GPIO Driver_GPIOB_0;
extern VOR_DRIVER_GPIO Driver_GPIOB_1;
extern VOR_DRIVER_GPIO Driver_GPIOB_2;
extern VOR_DRIVER_GPIO Driver_GPIOB_3;
extern VOR_DRIVER_GPIO Driver_GPIOB_4;
extern VOR_DRIVER_GPIO Driver_GPIOB_5;
extern VOR_DRIVER_GPIO Driver_GPIOB_6;
extern VOR_DRIVER_GPIO Driver_GPIOB_7;
extern VOR_DRIVER_GPIO Driver_GPIOB_8;
extern VOR_DRIVER_GPIO Driver_GPIOB_9;
extern VOR_DRIVER_GPIO Driver_GPIOB_10;
extern VOR_DRIVER_GPIO Driver_GPIOB_11;
extern VOR_DRIVER_GPIO Driver_GPIOB_12;
extern VOR_DRIVER_GPIO Driver_GPIOB_13;
extern VOR_DRIVER_GPIO Driver_GPIOB_14;
extern VOR_DRIVER_GPIO Driver_GPIOB_15;
extern VOR_DRIVER_GPIO Driver_GPIOB_16;
extern VOR_DRIVER_GPIO Driver_GPIOB_17;
extern VOR_DRIVER_GPIO Driver_GPIOB_18;
extern VOR_DRIVER_GPIO Driver_GPIOB_19;
extern VOR_DRIVER_GPIO Driver_GPIOB_20;
extern VOR_DRIVER_GPIO Driver_GPIOB_21;
extern VOR_DRIVER_GPIO Driver_GPIOB_22;
extern VOR_DRIVER_GPIO Driver_GPIOB_23;

/**
  \fn          VOR_DRIVER_VERSION GPIOx_GetVersion (void)
  \brief       Get driver version.
  \return      \ref VOR_DRIVER_VERSION
*/
VOR_DRIVER_VERSION VOR_GPIOx_GetVersion (void);

/**
  \fn          int32_t VOR_GPIO_Initialize (VOR_GPIO_SignalEvent_t  cb_event
                                         GPIO_RESOURCES         *GPIO)
  \brief       Initialize GPIO Interface.
  \param[in]   cb_event  Pointer to \ref VOR_GPIO_SignalEvent
  \param[in]   GPIO     Pointer to GPIO resources
  \return      \ref execution_status
*/
int32_t VOR_GPIO_Initialize (VOR_GPIO_SignalEvent_t  cb_event,
                                 GPIO_RESOURCES         *GPIO);
																 
/**
  \fn          int32_t VOR_GPIO_Uninitialize (GPIO_RESOURCES *GPIO)
  \brief       De-initialize GPIO Interface.
  \param[in]   GPIO     Pointer to GPIO resources
  \return      \ref execution_status
*/
int32_t VOR_GPIO_Uninitialize (GPIO_RESOURCES *GPIO);

/**
  \fn          uint32_t VOR_GPIO_Read (GPIO_RESOURCES *GPIO)
  \brief       Read data from port pin
  \param[in]   GPIO     Pointer to GPIO resources
  \return      pin value (0 or 1) or   \ref execution_status

*/
int32_t VOR_GPIO_Read (GPIO_RESOURCES *GPIO);

/**
  \fn          uint32_t VOR_GPIO_ReadRaw (GPIO_RESOURCES *GPIO)
  \brief       Read Raw data from port pin
  \param[in]   GPIO     Pointer to GPIO resources
  \return      pin value (0 or 1) or   \ref execution_status
*/
int32_t VOR_GPIO_ReadRaw (GPIO_RESOURCES *GPIO);

/**
  \fn          int32_t VOR_GPIO_Write (uint32_t val,
                                GPIO_RESOURCES *GPIO);
  \brief       Write port pin
  \param[in]   val        Port pin value (0 or 1)
  \param[in]   GPIO     Pointer to GPIO resources
  \return      \ref execution_status
*/
int32_t VOR_GPIO_Write (uint32_t val, GPIO_RESOURCES *GPIO);

/**
  \fn          void VOR_GPIO_WriteRaw (uint32_t val,
                                GPIO_RESOURCES *GPIO);
  \brief       Write data to port pin without mask
  \param[in]   val        Port pin value (0 or 1)
  \param[in]   GPIO     Pointer to GPIO resources
  \return      \ref execution_status
*/
int32_t VOR_GPIO_WriteRaw (uint32_t val, GPIO_RESOURCES *GPIO);

/**
  \fn          int32_t VOR_GPIO_Control (uint32_t control, uint32_t arg)
  \brief       Control GPIO Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \param[in]   GPIO     Pointer to GPIO resources
  \return      common \ref execution_status and driver specific \ref GPIO_execution_status
*/
int32_t VOR_GPIO_Control (uint32_t control, uint32_t arg, GPIO_RESOURCES *GPIO);

/**
  \fn          VOR_GPIO_STATUS VOR_GPIO_GetStatus (GPIO_RESOURCES *GPIO)
  \brief       Get GPIO status.
  \param[in]   GPIO     Pointer to GPIO resources
  \return      GPIO status \ref VOR_GPIO_STATUS
*/
VOR_GPIO_STATUS VOR_GPIO_GetStatus (GPIO_RESOURCES *GPIO);

/**
  \fn          void VOR_GPIO_IRQHandler (void)
  \brief       GPIO Interrupt Handler.
  \return      none
*/
void VOR_GPIO_IRQHandler (void);



#endif /* __GPIO_VA108XX_H */
