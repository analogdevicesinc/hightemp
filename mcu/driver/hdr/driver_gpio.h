/***************************************************************************************
 * @file     driver_gpio.h
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
/** \addtogroup GPIO
 *  @{
 */
#ifndef __DRIVER_GPIO_H
#define __DRIVER_GPIO_H

#include "driver_common.h"

#define VOR_GPIO_API_VERSION VOR_DRIVER_VERSION_MAJOR_MINOR(1,00)  /* API version */


/*----- GPIO Control Codes: Mode Parameters -----*/
#define VOR_GPIO_CONTROL_DIR         1
#define VOR_GPIO_CONTROL_SETOUT      2
#define VOR_GPIO_CONTROL_CLROUT      3
#define VOR_GPIO_CONTROL_TOGOUT      4
#define VOR_GPIO_CONTROL_MASK        5
#define VOR_GPIO_CONTROL_PULSE       6
#define VOR_GPIO_CONTROL_PULSEBASE   7
#define VOR_GPIO_CONTROL_DELAY       8
#define VOR_GPIO_CONTROL_IRQ_TRG     9
#define VOR_GPIO_CONTROL_IRQ_ENB     10

/****** GPIO Control Args Codes *****/
#define VOR_GPIO_CTRL_DISABLE        0
#define VOR_GPIO_CTRL_ENABLE         1
#define VOR_GPIO_DIR_INPUT           0
#define VOR_GPIO_DIR_OUTPUT          1
#define VOR_GPIO_IRQ_TRG_LVL_H       1
#define VOR_GPIO_IRQ_TRG_LVL_L       2
#define VOR_GPIO_IRQ_TRG_EDGE_LH     3
#define VOR_GPIO_IRQ_TRG_EDGE_HL     4
#define VOR_GPIO_IRQ_TRG_EDGE_ANY    5
#define VOR_GPIO_CONTROL_DELAY_0     0
#define VOR_GPIO_CONTROL_DELAY_1     1
#define VOR_GPIO_CONTROL_DELAY_2     2
#define VOR_GPIO_CONTROL_DELAY_3     3


/****** GPIO specific error codes *****/
#define VOR_GPIO_ERROR_PORT                (VOR_DRIVER_ERROR_SPECIFIC - 1)     ///< Invalid Port Number
#define VOR_GPIO_ERROR_PIN                 (VOR_DRIVER_ERROR_SPECIFIC - 2)     ///< Invalid Pin
#define VOR_GPIO_ERROR_CTRL                (VOR_DRIVER_ERROR_SPECIFIC - 3)     ///< CONTROL MODE not supported
#define VOR_GPIO_ERROR_CTRL_ARG            (VOR_DRIVER_ERROR_SPECIFIC - 4)     ///< CONTROL Args not supported
#define VOR_GPIO_ERROR_MULTIPLEINIT        (VOR_DRIVER_ERROR_SPECIFIC - 5)     ///< GPIO already initialized


/**
\brief GPIO Status
*/
typedef struct _VOR_GPIO_STATUS {
  uint32_t direction        : 1;        ///< Direction
  uint32_t pullup           : 1;        ///< Pull Up
  uint32_t pulldown         : 1;        ///< Pull Down
  uint32_t toggle           : 1;        ///< Toggle Mode
  uint32_t mask             : 1;        ///< Data Mask
  uint32_t pulse            : 1;        ///< Pulse Mode
  uint32_t pulsebase        : 1;        ///< Pulse Mode base
  uint32_t delay            : 2;        ///< Output Delay
  uint32_t trgmode          : 1;        ///< Trigger Edge/Level
  uint32_t trgsig           : 1;        ///< Trigger H/L or LH/HL
  uint32_t trgany           : 1;        ///< Trigger Any edge
  uint32_t irqenb           : 1;        ///< Interrupt Enable 
  uint32_t irqraw           : 1;        ///< Raw Interrupt Status  
  uint32_t irqend           : 1;        ///< Enabled Interrupt Status  
	uint32_t edgeStatus       : 1;        ///< Edge Interrupt Status 
} VOR_GPIO_STATUS;


/****** GPIO Event *****/     

/**
  \fn          VOR_DRIVER_VERSION VOR_GPIO_GetVersion (void)
  \brief       Get driver version.
  \return      \ref VOR_DRIVER_VERSION
  
  \fn          int32_t VOR_GPIO_Initialize (VOR_GPIO_SignalEvent_t cb_event)
  \brief       Initialize GPIO Interface.
  \param[in]   cb_event  Pointer to \ref VOR_GPIO_SignalEvent
  \return      \ref execution_status

  \fn          int32_t VOR_GPIO_Uninitialize (void)
  \brief       De-initialize GPIO Interface.
  \return      \ref execution_status

  \fn          int32_t VOR_GPIO_Read (void)
  \brief       Read Data from GPIO pin.
  \return      boolean Data read from GPIO pin \ref execution_status

  \fn          int32_t VOR_GPIO_Write (uint32_t val)
  \brief       Write Data to GPIO pin.
  \param[in]   val   boolean Data to write
  \return      \ref execution_status

  \fn          int32_t VOR_GPIO_ReadRaw (void)
  \brief       Read Raw Data from GPIO pin.
  \return      boolean Data raw read from GPIO pin \ref execution_status

  \fn          int32_t VOR_GPIO_WriteRaw (uint32_t val)
  \brief       Write Data directly to GPIO pin without mask.
  \param[in]   val   boolean Data to write
  \return      \ref execution_status

  \fn          int32_t VOR_GPIO_Control (uint32_t control, uint32_t arg)
  \brief       Control GPIO Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      common \ref execution_status and driver specific \ref GPIO_execution_status

  \fn          VOR_GPIO_STATUS VOR_GPIO_GetStatus (void)
  \brief       Get GPIO status.
  \return      GPIO status \ref VOR_GPIO_STATUS

  \fn          void VOR_GPIO_SignalEvent (uint32_t event)
  \brief       Signal GPIO Events.
  \param[in]   event  \ref GPIO_events notification mask
  \return      none
*/

typedef void (*VOR_GPIO_SignalEvent_t) (uint32_t event);  ///< Pointer to \ref VOR_GPIO_SignalEvent : Signal GPIO Event.


/**
\brief Access structure of the GPIO Driver.
*/
typedef struct _VOR_DRIVER_GPIO {
  VOR_DRIVER_VERSION     (*GetVersion)      (void);                              ///< Pointer to \ref VOR_GPIO_GetVersion   : Get driver version.
  int32_t                (*Initialize)      (VOR_GPIO_SignalEvent_t cb_event);   ///< Pointer to \ref VOR_GPIO_Initialize   : Initialize GPIO Interface.
  int32_t                (*Uninitialize)    (void);                              ///< Pointer to \ref VOR_GPIO_Uninitialize : De-initialize GPIO Interface.
  int32_t                (*Read)            (void);                              ///< Pointer to \ref VOR_GPIO_Read         : Read Data from GPIO pin.
  int32_t                (*Write)           (uint32_t val);                      ///< Pointer to \ref VOR_GPIO_Write        : Write Data to GPIO pin.
  int32_t                (*ReadRaw)         (void);                              ///< Pointer to \ref VOR_GPIO_ReadRaw      : Read Raw Data from GPIO pin.
  int32_t                (*WriteRaw)        (uint32_t val);                      ///< Pointer to \ref VOR_GPIO_WriteRaw     : Write Data directly to GPIO pin without mask.
  int32_t                (*Control)         (uint32_t control, uint32_t arg);    ///< Pointer to \ref VOR_GPIO_Control      : Control GPIO Interface.
  VOR_GPIO_STATUS        (*GetStatus)       (void);                              ///< Pointer to \ref VOR_GPIO_GetStatus    : Get GPIO status.
} VOR_DRIVER_GPIO;

#endif /* __DRIVER_GPIO_H */
/** @}*/
